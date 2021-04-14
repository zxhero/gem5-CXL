/*
 * Copyright (c) 2010-2019 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2010,2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Cache definitions.
 */

#include "mem/cache/l2cacheam.hh"

#include <cassert>
#include <list>

#include "arch/locked_mem.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "debug/Cache.hh"
#include "debug/CacheAM.hh"
#include "debug/CacheTags.hh"
#include "debug/CacheVerbose.hh"
#include "debug/LLSC.hh"
#include "debug/MemoryAccess.hh"
#include "enums/Clusivity.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/write_queue_entry.hh"
#include "mem/request.hh"
#include "params/L2CacheAM.hh"

L2CacheAM::L2CacheAM(const L2CacheAMParams *p)
    : Cache(p), innerRequestorId(p->system->getRequestorId(this, "inner")),
      spmWays(p->spm_init_capacity),
      pmemAddr(new uint8_t[p->size]),
      spmRange(p->spm_base_addr,
        p->spm_base_addr + spmWays *
        (p->size / (p->system->cacheLineSize() * p->assoc))),
      spmstats(*this), latency(p->latency), latency_var(p->latency_var),
      bandwidth(p->bandwidth), isBusy(false),
      retryReq(false), retryResp(false),
      releaseEvent([this] { spmRelease(); }, name()),
      dequeueEvent([this] { spmDequeue(); }, name()),
      asyncMemReqLength(32),
      asyncMemReqs(asyncMemReqLength),
      asyncMemConfigRegs(MEMACC_CFG_COUNT)
{
    fatal_if(spmWays > p->assoc,
             "spm ways must smaller than cache associativity");
    numSets = p->size / (p->system->cacheLineSize() * p->assoc);
}

L2CacheAM::~L2CacheAM()
{
    delete pmemAddr;
}

Tick L2CacheAM::getSpmLatency() const
{
    // return latency +
    //       (latency_var ? random_mt.random<Tick>(0, latency_var) : 0);
    return dataLatency * clockPeriod();
}

// Add load-locked to tracking list.  Should only be called if the
// operation is a load and the LLSC flag is set.
void L2CacheAM::spmTrackLoadLocked(PacketPtr pkt)
{
    const RequestPtr &req = pkt->req;
    Addr paddr = LockedAddr::mask(req->getPaddr());

    // first we check if we already have a locked addr for this
    // xc.  Since each xc only gets one, we just update the
    // existing record with the new address.
    std::list<LockedAddr>::iterator i;

    for (i = spmLockedAddrList.begin(); i != spmLockedAddrList.end(); ++i)
    {
        if (i->matchesContext(req))
        {
            DPRINTF(LLSC, "Modifying lock record: context %d addr %#x\n",
                    req->contextId(), paddr);
            i->addr = paddr;
            return;
        }
    }

    // no record for this xc: need to allocate a new one
    DPRINTF(LLSC, "Adding lock record: context %d addr %#x\n",
            req->contextId(), paddr);
    spmLockedAddrList.push_front(LockedAddr(req));
}

// Called on *writes* only... both regular stores and
// store-conditional operations.  Check for conventional stores which
// conflict with locked addresses, and for success/failure of store
// conditionals.
bool L2CacheAM::checkLockedAddrList(PacketPtr pkt)
{
    const RequestPtr &req = pkt->req;
    Addr paddr = LockedAddr::mask(req->getPaddr());
    bool isLLSC = pkt->isLLSC();

    // Initialize return value.  Non-conditional stores always
    // succeed.  Assume conditional stores will fail until proven
    // otherwise.
    bool allowStore = !isLLSC;

    // Iterate over list.  Note that there could be multiple matching records,
    // as more than one context could have done a load locked to this location.
    // Only remove records when we succeed in finding a record for (xc, addr);
    // then, remove all records with this address.  Failed store-conditionals
    // do not blow unrelated reservations.
    std::list<LockedAddr>::iterator i = spmLockedAddrList.begin();

    if (isLLSC)
    {
        while (i != spmLockedAddrList.end())
        {
            if (i->addr == paddr && i->matchesContext(req))
            {
                // it's a store conditional, and as far as the memory system
                // can tell, the requesting context's lock is still valid.
                DPRINTF(LLSC, "StCond success: context %d addr %#x\n",
                        req->contextId(), paddr);
                allowStore = true;
                break;
            }
            // If we didn't find a match, keep searching!  Someone else
            // may well have a reservation on this line here but we may
            // find ours in just a little while.
            i++;
        }
        req->setExtraData(allowStore ? 1 : 0);
    }
    // LLSCs that succeeded AND non-LLSC stores both fall into here:
    if (allowStore)
    {
        // We write address paddr.  However, there may be several entries
        // with a reservation on this address (for other contextIds) and
        // they must all be removed.
        i = spmLockedAddrList.begin();
        while (i != spmLockedAddrList.end())
        {
            if (i->addr == paddr)
            {
                DPRINTF(LLSC, "Erasing lock record: context %d addr %#x\n",
                        i->contextId, paddr);
                ContextID owner_cid = i->contextId;
                assert(owner_cid != InvalidContextID);
                ContextID requestor_cid = req->hasContextId() ?
                    req->contextId() : InvalidContextID;
                if (owner_cid != requestor_cid)
                {
                    ThreadContext *ctx = system->threads[owner_cid];
                    TheISA::globalClearExclusive(ctx);
                }
                i = spmLockedAddrList.erase(i);
            }
            else
            {
                i++;
            }
        }
    }

    return allowStore;
}

/////////////////////////////////////////////////////
//
// Access path: requests coming in from the CPU side
//
/////////////////////////////////////////////////////

void L2CacheAM::spmAccess(PacketPtr pkt)
{
    uint8_t *hostAddr = pmemAddr + pkt->getAddr() - spmRange.start();
    if (pkt->isRead())
    {
        assert(!pkt->isWrite());
        if (pkt->isLLSC())
        {
            assert(!pkt->fromCache());
            // if the packet is not coming from a cache then we have
            // to do the LL/SC tracking here
            spmTrackLoadLocked(pkt);
        }
        if (pmemAddr)
        {
            pkt->setData(hostAddr);
        }
        spmstats.numReads[pkt->req->requestorId()]++;
        spmstats.bytesRead[pkt->req->requestorId()] += pkt->getSize();
        if (pkt->req->isInstFetch())
            spmstats.bytesInstRead[pkt->req->requestorId()] += pkt->getSize();
    }
    else if (pkt->isInvalidate() || pkt->isClean())
    {
        assert(!pkt->isWrite());
        // in a fastmem system invalidating and/or cleaning packets
        // can be seen due to cache maintenance requests

        // no need to do anything
    }
    else if (pkt->isWrite())
    {
        if (spmWriteOK(pkt))
        {
            if (pmemAddr)
            {
                pkt->writeData(hostAddr);
                DPRINTF(MemoryAccess, "%s write due to %s\n",
                        __func__, pkt->print());
            }
            assert(!pkt->req->isInstFetch());
            spmstats.numWrites[pkt->req->requestorId()]++;
            spmstats.bytesWritten[pkt->req->requestorId()] += pkt->getSize();
        }
    }
    else
    {
        panic("Unexpected packet %s", pkt->print());
    }

    if (pkt->needsResponse())
    {
        pkt->makeResponse();
    }
}

L2CacheAM::SpmStats::SpmStats(L2CacheAM &_mem)
    : Stats::Group(&_mem), mem(_mem),
      bytesRead(this, "bytes_read",
                "Number of bytes read from this memory"),
      bytesInstRead(this, "bytes_inst_read",
                    "Number of instructions bytes read from this memory"),
      bytesWritten(this, "bytes_written",
                   "Number of bytes written to this memory"),
      numReads(this, "num_reads",
               "Number of read requests responded to by this memory"),
      numWrites(this, "num_writes",
                "Number of write requests responded to by this memory"),
      numOther(this, "num_other",
               "Number of other requests responded to by this memory"),
      bwRead(this, "bw_read",
             "Total read bandwidth from this memory (bytes/s)"),
      bwInstRead(this, "bw_inst_read",
                 "Instruction read bandwidth from this memory (bytes/s)"),
      bwWrite(this, "bw_write",
              "Write bandwidth from this memory (bytes/s)"),
      bwTotal(this, "bw_total",
              "Total bandwidth to/from this memory (bytes/s)")
{
}

void L2CacheAM::SpmStats::regStats()
{
    using namespace Stats;

    Stats::Group::regStats();

    System *sys = mem.system;
    assert(sys);
    const auto max_requestors = sys->maxRequestors();

    bytesRead
        .init(max_requestors)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        bytesRead.subname(i, sys->getRequestorName(i));
    }

    bytesInstRead
        .init(max_requestors)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        bytesInstRead.subname(i, sys->getRequestorName(i));
    }

    bytesWritten
        .init(max_requestors)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        bytesWritten.subname(i, sys->getRequestorName(i));
    }

    numReads
        .init(max_requestors)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        numReads.subname(i, sys->getRequestorName(i));
    }

    numWrites
        .init(max_requestors)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        numWrites.subname(i, sys->getRequestorName(i));
    }

    numOther
        .init(max_requestors)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        numOther.subname(i, sys->getRequestorName(i));
    }

    bwRead
        .precision(0)
        .prereq(bytesRead)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        bwRead.subname(i, sys->getRequestorName(i));
    }

    bwInstRead
        .precision(0)
        .prereq(bytesInstRead)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        bwInstRead.subname(i, sys->getRequestorName(i));
    }

    bwWrite
        .precision(0)
        .prereq(bytesWritten)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        bwWrite.subname(i, sys->getRequestorName(i));
    }

    bwTotal
        .precision(0)
        .prereq(bwTotal)
        .flags(total | nozero | nonan);
    for (int i = 0; i < max_requestors; i++)
    {
        bwTotal.subname(i, sys->getRequestorName(i));
    }

    bwRead = bytesRead / simSeconds;
    bwInstRead = bytesInstRead / simSeconds;
    bwWrite = bytesWritten / simSeconds;
    bwTotal = (bytesRead + bytesWritten) / simSeconds;
}

// void L2CacheAM::recvFunctional(PacketPtr pkt)
// {
//     if (spmRange.contains(pkt->getAddr())) {
//         DPRINTF(CacheAM,
//             "%s L2CacheAM: access spm at %lx\n",
//             __func__, pkt->getAddr());
//     } else {
//         DPRINTF(CacheAM,
//             "%s L2CacheAM: access cache at %lx\n",
//             __func__, pkt->getAddr());
//         return Cache::recvFunctional(pkt);
//     }
// }

void L2CacheAM::spmRelease()
{
    assert(isBusy);
    isBusy = false;
    if (retryReq)
    {
        retryReq = false;
        cpuSidePort.sendRetryReq();
    }
}

void L2CacheAM::spmDequeue()
{
    assert(!packetQueue.empty());
    DeferredPacket deferred_pkt = packetQueue.front();

    retryResp = !cpuSidePort.sendTimingResp(deferred_pkt.pkt);

    if (!retryResp)
    {
        packetQueue.pop_front();

        // if the queue is not empty, schedule the next dequeue event,
        // otherwise signal that we are drained if we were asked to do so
        if (!packetQueue.empty())
        {
            // if there were packets that got in-between then we
            // already have an event scheduled, so use re-schedule
            reschedule(dequeueEvent,
                       std::max(packetQueue.front().tick, curTick()), true);
        }
        else if (drainState() == DrainState::Draining)
        {
            DPRINTF(Drain, "Draining of SimpleMemory complete\n");
            signalDrainDone();
        }
    }
}

bool L2CacheAM::spmRecvTimingReq(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
                                     "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller, "
             "saw %s to %#llx\n",
             pkt->cmdString(), pkt->getAddr());

    // we should not get a new request after committing to retry the
    // current one, but unfortunately the CPU violates this rule, so
    // simply ignore it for now
    if (retryReq)
        return false;

    // if we are busy with a read or write, remember that we have to
    // retry
    if (isBusy)
    {
        retryReq = true;
        return false;
    }

    // technically the packet only reaches us after the header delay,
    // and since this is a memory controller we also need to
    // deserialise the payload before performing any write operation
    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    // update the release time according to the bandwidth limit, and
    // do so with respect to the time it takes to finish this request
    // rather than long term as it is the short term data rate that is
    // limited for any real memory

    // calculate an appropriate tick to release to not exceed
    // the bandwidth limit
    Tick duration = pkt->getSize() * bandwidth;

    Tick spmlat = getSpmLatency();

    unsigned bank_id = getBankId(pkt->getAddr());
    if (enableBankModel) {
        bank[bank_id]->markInService(curTick() + spmlat);
    }
    // only consider ourselves busy if there is any need to wait
    // to avoid extra events being scheduled for (infinitely) fast
    // memories
    // if (duration != 0)
    // {
    //     schedule(releaseEvent, curTick() + duration);
    //     isBusy = true;
    // }

    // go ahead and deal with the packet and put the response in the
    // queue if there is one
    bool needsResponse = pkt->needsResponse();
    recvAtomic(pkt);
    // turn packet around to go back to requestor if response expected
    if (needsResponse)
    {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());

        Tick when_to_send = curTick() + receive_delay + spmlat;
        cpuSidePort.schedTimingResp(pkt, when_to_send);

        // typically this should be added at the end, so start the
        // insertion sort with the last element, also make sure not to
        // re-order in front of some existing packet with the same
        // address, the latter is important as this memory effectively
        // hands out exclusive copies (shared is not asserted)

        // auto i = packetQueue.end();
        // --i;
        // while (i != packetQueue.begin() && when_to_send < i->tick &&
        //        !i->pkt->matchAddr(pkt))
        //     --i;

        // emplace inserts the element before the position pointed to by
        // the iterator, so advance it one step

        //packetQueue.emplace(++i, pkt, when_to_send);

        // if (!retryResp && !dequeueEvent.scheduled())
        //    schedule(dequeueEvent, packetQueue.back().tick);
    }
    else
    {
        // pendingDelete.reset(pkt);
    }

    return true;
}

int L2CacheAM::allocAsyncMemReq(uint64_t spmAddr, Addr memAddr)
{
    int i = asyncMemReqs.size();
    while (i--)
    {
        if (!asyncMemReqs[i].valid)
        {
            asyncMemReqs[i].spm_addr = spmAddr;
            asyncMemReqs[i].mem_addr = memAddr;
            asyncMemReqs[i].valid = true;
            return i + 1;
        }
    }
    return 0;
}

bool L2CacheAM::checkAsyncMemReq(unsigned long handle)
{
    if (handle > 0) {
        --handle;
        if (handle < asyncMemReqs.size()) {
            bool ret = false;
            if (asyncMemReqs[handle].valid) {
                ret = asyncMemReqs[handle].finished;
                asyncMemReqs[handle].finished = false;
            }
            if (ret) asyncMemReqs[handle].valid = false;
            return ret;
        }
    }
    return false;
}

void L2CacheAM::recvTimingReq(PacketPtr pkt)
{
    DPRINTF(CacheAM, "recvReq pkt at tick %ld\n", curTick());
    if (pkt->cmd == MemCmd::AsyncMemWrReq ||
        pkt->cmd == MemCmd::AsyncMemLdReq)
    {
        uint64_t headregval = pkt->req->getExtraData();
        uint64_t spm_addr = 0;
        pkt->writeData((uint8_t *)&spm_addr);
        DPRINTF(CacheAM,
                "%s L2CacheAM: async mem load/store at 0x%lx,"
                " spm_addr = 0x%lx, head=%lx\n",
                __func__, pkt->getAddr(), spm_addr, headregval);

        if (pkt->cmd == MemCmd::AsyncMemLdReq)
        {
            PacketPtr _pkt =
                new Packet(pkt->req, MemCmd::ReadReq, 8);
            _pkt->allocate();
            memSidePort.schedTimingReq(
                _pkt, clockEdge(forwardLatency));
        } else if (pkt->cmd == MemCmd::AsyncMemWrReq) {
            // PacketPtr _pkt =
            //     new Packet(pkt->req, MemCmd::WriteReq, 8);
            // _pkt->allocate();
            // _pkt->setData(&pmemAddr[spm_addr - 0x1000000000000000llu]);
            // memSidePort.schedTimingReq(
            //     _pkt, clockEdge(forwardLatency));
            RequestPtr _inner_req = std::make_shared<Request>(
                spm_addr, 8, Request::UNCACHEABLE, innerRequestorId
            );
            PacketPtr _pkt = Packet::createRead(_inner_req);
            _pkt->allocate();
            cpuSidePort.schedInnerTimingReq(_pkt, clockEdge(forwardLatency));
        }

        int spm_addr_pkt_id = allocAsyncMemReq(spm_addr, pkt->getAddr());
        pkt->makeTimingResponse();
        pkt->setData((uint8_t *)&spm_addr_pkt_id);
        cpuSidePort.schedTimingResp(pkt, clockEdge(forwardLatency));
        return;
    }

    if (pkt->cmd == MemCmd::TestFinReq) {
        uint64_t handle = 0;
        pkt->writeData((uint8_t *)&handle);
        pkt->makeTimingResponse();
        uint64_t req_result = checkAsyncMemReq(handle);
        pkt->setData((uint8_t*)&req_result);
        cpuSidePort.schedTimingResp(pkt, clockEdge(forwardLatency));
        DPRINTF(CacheAM,
                "%s L2CacheAM: testfin(handle=%ld,res=%ld)\n",
                __func__, handle,req_result);
        return ;
    }

    if (pkt->cmd == MemCmd::CfgRegReq) {
        int regid = pkt->getAddr() - 0x1000000000000000llu;
        uint64_t val = 0;
        pkt->writeData((uint8_t *)&val);
        pkt->makeTimingResponse();
        asyncMemConfigRegs[regid] = val;
        DPRINTF(CacheAM,
                "%s L2CacheAM: cfgreg(regid=%ld,val=%ld)\n",
                __func__, regid, val);
    }

    if (spmRange.contains(pkt->getAddr()))
    {
        DPRINTF(CacheAM,
                "%s L2CacheAM: access spm at %lx\n",
                __func__, pkt->getAddr());
        spmRecvTimingReq(pkt);
    }
    else
    {
        DPRINTF(CacheAM,
                "%s L2CacheAM: access cache at %lx\n",
                __func__, pkt->getAddr());
        Cache::recvTimingReq(pkt);
    }
}

void L2CacheAM::recvInnerTimingResp(PacketPtr pkt)
{
    DPRINTF(CacheAM, "recvInnerResp at tick %ld\n", curTick());
    for (auto amReqIter = asyncMemReqs.begin();
         amReqIter != asyncMemReqs.end();
         ++amReqIter)
    {
        if (amReqIter->valid && amReqIter->spm_addr == pkt->getAddr())
        {
            if (pkt->cmd == MemCmd::ReadResp) {
                RequestPtr _inner_req = std::make_shared<Request>(
                    amReqIter->mem_addr, 8, Request::UNCACHEABLE,
                    innerRequestorId
                );
                PacketPtr _pkt = Packet::createWrite(_inner_req);
                _pkt->allocate();
                uint64_t data;
                pkt->writeData((uint8_t*)&data);
                _pkt->setData((uint8_t*)&data);
                memSidePort.schedTimingReq(
                    _pkt, clockEdge(forwardLatency));
            } else {
                amReqIter->finished = true;
            }

            delete pkt;

            return;
        }
    }

    // should not reach here!
    assert(0);
}

void L2CacheAM::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    if (pkt->req->isUncacheable() &&
        (pkt->cmd == MemCmd::ReadResp ||
        pkt->cmd == MemCmd::WriteResp))
    {
        for (auto amReqIter = asyncMemReqs.begin();
             amReqIter != asyncMemReqs.end();
             ++amReqIter)
        {
            if (amReqIter->valid && amReqIter->mem_addr == pkt->getAddr())
            {
                if (pkt->cmd == MemCmd::ReadResp) {
                    RequestPtr _inner_req = std::make_shared<Request>(
                        amReqIter->spm_addr, 8, Request::UNCACHEABLE,
                        innerRequestorId
                    );
                    PacketPtr _pkt = Packet::createWrite(_inner_req);
                    _pkt->allocate();
                    uint64_t data;
                    pkt->writeData((uint8_t*)&data);
                    _pkt->setData((uint8_t*)&data);
                    cpuSidePort.schedInnerTimingReq(_pkt,
                        clockEdge(forwardLatency));
                } else {
                    amReqIter->finished = true;
                }

                delete pkt;

                return;
            }
        }
    }

    Cache::recvTimingResp(pkt);
}

Tick L2CacheAM::recvAtomic(PacketPtr pkt)
{
    if (spmRange.contains(pkt->getAddr()))
    {
        DPRINTF(CacheAM,
                "%s L2CacheAM: access spm at %lx\n",
                __func__, pkt->getAddr());
        spmAccess(pkt);
        return getSpmLatency();
    }
    else
    {
        DPRINTF(CacheAM,
                "%s L2CacheAM: access cache at %lx\n",
                __func__, pkt->getAddr());
        return Cache::recvAtomic(pkt);
    }
}

L2CacheAM *
L2CacheAMParams::create()
{
    assert(tags);
    assert(replacement_policy);

    return new L2CacheAM(this);
}
