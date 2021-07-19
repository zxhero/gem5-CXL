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
#include "debug/CacheAMReq.hh"
#include "debug/CacheTags.hh"
#include "debug/CacheVerbose.hh"
#include "debug/LLSC.hh"
#include "debug/MemoryAccess.hh"
#include "enums/Clusivity.hh"
#include "mem/cache/cache_blk.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/tags/base.hh"
#include "mem/cache/tags/indexing_policies/reconf_set_associative.hh"
#include "mem/cache/write_queue_entry.hh"
#include "mem/request.hh"
#include "params/L2CacheAM.hh"

L2CacheAM::L2CacheAM(const L2CacheAMParams *p)
    : Cache(p),
      logicLatency(p->logic_latency),
      spmWays(p->spm_init_capacity),
      asyncmemOutstanding(p->asyncmem_outstanding),
      asyncmemRespPending(p->asyncmem_outstanding),
      pmemAddr(new uint8_t[p->size]),
      spmRange(p->spm_base_addr,
        p->spm_base_addr + spmWays * p->size /  p->assoc),
      spmstats(*this), latency(p->latency),
      latency_var(p->latency_var),
      bandwidth(p->bandwidth), isBusy(false),
      retryReq(false), retryResp(false),
      releaseEvent([this] { spmRelease(); }, name()),
      dequeueEvent([this] { spmDequeue(); }, name()),
      amReqStateMachine(this, p->asyncmem_outstanding,
        p->system->getRequestorId(this, "amReqModule")),
      amRespStateMachine(this, p->asyncmem_outstanding,
        p->system->getRequestorId(this, "amRespModule")),
      freeListStateMachine(this, p->asyncmem_outstanding,
        p->system->getRequestorId(this, "freeListModule")),
      lastSpmFsmTick(0),
      asyncMemReqLength(32),
      // asyncMemReqs(asyncMemReqLength),
      asyncMemConfigRegs(MEMACC_CFG_COUNT)
{
    fatal_if(spmWays > p->assoc,
             "spm ways must smaller than cache associativity");
    numSets = p->size / (p->system->cacheLineSize() * p->assoc);

    routeTo[amReqStateMachine.getRequestorId()] = &amReqStateMachine;
    routeTo[amRespStateMachine.getRequestorId()] = &amRespStateMachine;
    routeTo[freeListStateMachine.getRequestorId()] = &freeListStateMachine;

    ReconfSetAssociative *reconf_idx_policy =
        dynamic_cast<ReconfSetAssociative*>(p->tags->getIndexingPolicy());
    assert(reconf_idx_policy != NULL);
    reconf_idx_policy->reconfWays(p->assoc - spmWays);
    // reconf_idx_policy->reconfWays(1);
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
    bool allowStore =! isLLSC;

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
    assert(spmRange.contains(pkt->getAddr()));
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
              "Total bandwidth to/from this memory (bytes/s)"),
      stateTicks(this, "state_ticks",
              "Ticks of each state of Cache Controller State Machine"),
      afterMemResp(this, "after_mem_resp_ticks",
              "Ticks of states(after memory response received)"),
      numRequest(this, "num_request",
             "Number of asynchronous memory request"),
      numALoad(this, "asyncmem_num_aload_req",
             "Number of asynchronous load request"),
      numAStore(this, "asyncmem_num_astore_req",
             "Number of asynchronous store request"),
      numTestFin(this, "asyncmem_num_testfin_req",
             "Number of asynchronous TestFin request"),
      numGetFin(this, "asyncmem_num_getfin_req",
             "Number of asynchronous GetFin request"),
      maxPendingReq(this, "max_pending_asyncmem_req",
             "Maximum number of pending asynchronous memory request"),
      maxOutstandingReq(this, "max_outstanding_asyncmem_req",
             "Maximum number of outstanding asynchronous memory request"),
      maxFinishCount(this, "max_finish_count",
             "Maximum number of finish count"),
      maxUsedEntry(this, "max_used_req_entry",
             "Maximum number of used asynchronous memory entry")
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

    stateTicks
        .init(SPM_STATE_COUNT)
        .flags(total | nozero | nonan);
    for (int i = 0; i < SPM_STATE_COUNT; i++)
    {
        stateTicks.subname(i, spmStateStr[i]);
    }
    afterMemResp = stateTicks[EXEC_ALOAD]
        + stateTicks[FIN_ALOAD]
        + stateTicks[GET_REQ_ENTRY]
        + stateTicks[FILL_FIN_ENTRY]
        + stateTicks[FIN_REQ_ENTRY];
    // numRequest.flags(total | nozero | nonan);
    // numALoad.flags(total | nozero | nonan);
    // numAStore.flags(total | nozero | nonan);
    // numTestFin.flags(total | nozero | nonan);
    // numGetFin.flags(total | nozero | nonan);
    // maxPendingReq.flags(total | nozero | nonan);
    // maxOutstandingReq.flags(total | nozero | nonan);
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
    // int i = asyncMemReqs.size();
    // while (i--)
    // {
    //     if (!asyncMemReqs[i].valid)
    //     {
    //         asyncMemReqs[i].spm_addr = spmAddr;
    //         asyncMemReqs[i].mem_addr = memAddr;
    //         asyncMemReqs[i].valid = true;
    //         return i + 1;
    //     }
    // }
    return 0;
}

bool L2CacheAM::checkAsyncMemReq(unsigned long handle)
{
    // if (handle > 0) {
    //     --handle;
    //     if (handle < asyncMemReqs.size()) {
    //         bool ret = false;
    //         if (asyncMemReqs[handle].valid) {
    //             ret = asyncMemReqs[handle].finished;
    //             asyncMemReqs[handle].finished = false;
    //         }
    //         if (ret) asyncMemReqs[handle].valid = false;
    //         return ret;
    //     }
    // }
    return false;
}

void L2CacheAM::recvTimingReq(PacketPtr pkt)
{
    if (pkt->req->isUncacheable()) {
        DPRINTF(CacheAM, "recvReq uncacheable pkt at tick %ld\n", curTick());
    }
    if (pkt->cmd == MemCmd::AsyncMemWrReq ||
        pkt->cmd == MemCmd::AsyncMemLdReq)
    {
        uint64_t used_entrys =
            asyncMemReqLength -
            ((asyncMemFreeTail + asyncMemReqLength - asyncMemFreeHead) %
                asyncMemReqLength);
        DPRINTF(CacheAMReq,
                "L2CacheAMReq: %s at cycle %ld"
                "(finish=%d, used=%d, outstanding=%d)\n",
                pkt->cmd == MemCmd::AsyncMemWrReq ? "astore": "aload",
                curCycle(), asyncMemFinishCount,
                used_entrys, asyncMemOutstandingCount);

        bool addPktSuccess = amReqStateMachine.addPacket(pkt);
        assert(addPktSuccess);
        if (amReqStateMachine.isFull()) {
            setBlocked(Blocked_NoAMPktQueues);
        }
        spmstats.numRequest += 1;
        amReqStateMachine.retryProcessAm();

        // uint64_t headregval = pkt->req->getExtraData();
        // uint64_t spm_addr = 0;
        // pkt->writeData((uint8_t *)&spm_addr);
        // DPRINTF(CacheAM,
        //         "%s L2CacheAM: async mem load/store at 0x%lx,"
        //         " spm_addr = 0x%lx, head=%lx\n",
        //         __func__, pkt->getAddr(), spm_addr, headregval);

        return;
    }

    if (pkt->cmd == MemCmd::TestFinReq) {
        assert(0);
        // uint64_t used_entrys =
        //     asyncMemReqLength -
        //     ((asyncMemFreeTail + asyncMemReqLength - asyncMemFreeHead) %
        //         asyncMemReqLength);
        // uint64_t handle = 0;
        // pkt->writeData((uint8_t*)&handle);
        // DPRINTF(CacheAMReq,
        //         "L2CacheAMReq: testfin(handle=%d) at cycle %ld"
        //         "(finish=%d, used=%d, outstanding=%d)\n",
        //         handle, curCycle(),
        //         asyncMemFinishCount, used_entrys, asyncMemOutstandingCount);
        // pendingAsyncMemPkts.push_back(pkt);
        // if (pendingAsyncMemPkts.size() >= asyncmemOutstanding) {
        //     setBlocked(Blocked_NoAMPktQueues);
        // }
        // if (spmstats.maxPendingReq.total() < pendingAsyncMemPkts.size()) {
        //     spmstats.maxPendingReq = pendingAsyncMemPkts.size();
        // }
        // spmstats.numRequest += 1;
        // retryProcessAMReqAndResp();
        // uint64_t handle = 0;
        // pkt->writeData((uint8_t *)&handle);
        // pkt->makeTimingResponse();
        // uint64_t req_result = checkAsyncMemReq(handle);
        // pkt->setData((uint8_t*)&req_result);
        // cpuSidePort.schedTimingResp(pkt, clockEdge(dataLatency));
        // DPRINTF(CacheAM,
        //         "%s L2CacheAM: testfin(handle=%ld,res=%ld)\n",
        //         __func__, handle,req_result);
        return ;
    }

    if (pkt->cmd == MemCmd::CfgRegReq) {
        spmstats.numRequest += 1;
        int regid = pkt->getAddr() - 0x1000000000000000llu;
        switch (regid) {
          case MEMACC_CFG_QBASE: {
            uint64_t val = 0;
            assert(pkt->getSize() == sizeof(val));
            pkt->writeData((uint8_t *)&val);
            asyncMemReqBase = val;
            asyncMemConfigRegs[regid] = val;
            amReqStateMachine.setOutstandingAsyncMemPkt(pkt);
            amReqStateMachine.spmFsmProcess(RECONF_QUEUE_BASE, nullptr);
            return;
          }
          case MEMACC_CFG_QLENGTH: {
            uint64_t val = 0;
            assert(pkt->getSize() == sizeof(val));
            pkt->writeData((uint8_t *)&val);
            asyncMemReqLength = val;
            asyncMemConfigRegs[regid] = val;
            // asyncMemReqs.resize(val);
            amReqStateMachine.setOutstandingAsyncMemPkt(pkt);
            amReqStateMachine.spmFsmProcess(RECONF_QUEUE_LENGTH, nullptr);
            return;
          }
          case MEMACC_CFG_GETFIN: {
            uint64_t used_entrys =
                asyncMemReqLength -
                ((asyncMemFreeTail + asyncMemReqLength - asyncMemFreeHead) %
                    asyncMemReqLength);
            DPRINTF(CacheAMReq,
                    "L2CacheAMReq: getfin at cycle %ld"
                    "(finish=%d, used=%d, outstanding=%d)\n",
                    curCycle(), asyncMemFinishCount,
                    used_entrys, asyncMemOutstandingCount);
            amRespStateMachine.addPacket(pkt);
            if (amRespStateMachine.isFull()) {
                setBlocked(Blocked_NoAMPktQueues);
            }
            amRespStateMachine.retryProcessAm();
            return;
          }
          case MEMACC_CFG_GETFREE: {
            uint64_t used_entrys =
                asyncMemReqLength -
                ((asyncMemFreeTail + asyncMemReqLength - asyncMemFreeHead) %
                    asyncMemReqLength);
            DPRINTF(CacheAMReq,
                    "L2CacheAMReq: getfree at cycle %ld"
                    "(finish=%d, used=%d, outstanding=%d)\n",
                    curCycle(), asyncMemFinishCount,
                    used_entrys, asyncMemOutstandingCount);
            freeListStateMachine.addPacket(pkt);
            if (freeListStateMachine.isFull()) {
                setBlocked(Blocked_NoAMPktQueues);
            }
            freeListStateMachine.retryProcessAm();
            return;
          }
          case MEMACC_CFG_WRITEFREE: {
            uint64_t used_entrys =
                asyncMemReqLength -
                ((asyncMemFreeTail + asyncMemReqLength - asyncMemFreeHead) %
                    asyncMemReqLength);
            DPRINTF(CacheAMReq,
                    "L2CacheAMReq: writefree at cycle %ld"
                    "(finish=%d, used=%d, outstanding=%d)\n",
                    curCycle(), asyncMemFinishCount,
                    used_entrys, asyncMemOutstandingCount);
            freeListStateMachine.addPacket(pkt);
            if (freeListStateMachine.isFull()) {
                setBlocked(Blocked_NoAMPktQueues);
            }
            freeListStateMachine.retryProcessAm();
            return;
          }

          default:
            assert(0);
            break;
        }
        pkt->makeTimingResponse();
        // asyncMemConfigRegs[regid] = val;
        cpuSidePort.schedTimingResp(pkt, clockEdge(logicLatency));
        DPRINTF(CacheAM,
                "%s L2CacheAM: cfgreg(regid=%ld)\n",
                __func__, regid);
        return ;
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
        assert((pkt->getAddr() >> 48) != 0x1000);
        // DPRINTF(CacheAM,
        //         "%s L2CacheAM: access cache at %lx\n",
        //         __func__, pkt->getAddr());
        Cache::recvTimingReq(pkt);
    }
}

void L2CacheAM::recvInnerTimingResp(PacketPtr pkt)
{
    // DPRINTF(CacheAM, "recvInnerTimingResp at tick %ld, addr=%lx\n",
    //     curTick(), pkt->getAddr());
    // for (auto amReqIter = asyncMemReqs.begin();
    //      amReqIter != asyncMemReqs.end();
    //      ++amReqIter)
    // {
    //     if (amReqIter->valid && amReqIter->spm_addr == pkt->getAddr())
    //     {
    //         if (pkt->cmd == MemCmd::ReadResp) {
    //             RequestPtr _inner_req = std::make_shared<Request>(
    //                 amReqIter->mem_addr, 8, Request::UNCACHEABLE,
    //                 innerRequestorId
    //             );
    //             PacketPtr _pkt = Packet::createWrite(_inner_req);
    //             _pkt->allocate();
    //             uint64_t data;
    //             pkt->writeData((uint8_t*)&data);
    //             _pkt->setData((uint8_t*)&data);
    //             memSidePort.schedTimingReq(
    //                 _pkt, clockEdge(dataLatency));
    //         } else {
    //             amReqIter->finished = true;
    //         }

    //         delete pkt;

    //         return;
    //     }
    // }
    auto target = routeTo.find(pkt->req->requestorId());

    if (target != routeTo.end()) {
        if (pkt->cmd == MemCmd::ReadResp) {
            target->second->spmFsmProcess(RECV_SPM_READ_RESP, pkt);
        } else {
            target->second->spmFsmProcess(RECV_SPM_WRITE_RESP, pkt);
        }
    }
    return;

    // should not reach here!
    assert(0);
}

void L2CacheAM::recvTimingResp(PacketPtr pkt)
{
    // DPRINTF(CacheAM, "recvTimingResp at tick %ld, addr=%lx\n",
    //     curTick(), pkt->getAddr());
    assert(pkt->isResponse());

    /* if (pkt->req->isUncacheable() &&
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
                        clockEdge(dataLatency));
                } else {
                    amReqIter->finished = true;
                }

                delete pkt;

                return;
            }
        }
    } */

    auto target = routeTo.find(pkt->req->requestorId());
    if (target != routeTo.end()) {
        assert(pkt->req->isUncacheable());

        if (!amRespStateMachine.addPacket(pkt)) {
            assert(0);
        }
        amRespStateMachine.retryProcessAm();

    //     if (!target->second->addPacket(pkt)) {
    //         assert(0);
    //     }
    //     target->second->retryProcessAm();
    //     // pendingAsyncMemRespPkts.push_back(pkt);
    //     // if (pendingAsyncMemRespPkts.size() >= asyncmemRespPending) {
    //     //     // memSidePort.setBlocked();
    //     // }
        // --asyncMemOutstandingCount;
    //     // retryProcessAMReqAndResp();

        return;
    }


    Cache::recvTimingResp(pkt);
}

#define MEMREQ_ENTRY_SIZE 10
#define FREELIST_ENTRY_SIZE 2
#define FREELIST_SPM_ADDR(pos)                      \
    (asyncMemReqBase +                              \
     asyncMemReqLength * MEMREQ_ENTRY_SIZE +        \
     (pos) * FREELIST_ENTRY_SIZE)
#define FINLIST_SPM_ADDR(pos)                       \
    (asyncMemReqBase +                              \
     asyncMemReqLength *                            \
        (MEMREQ_ENTRY_SIZE + FREELIST_ENTRY_SIZE) + \
     (pos) * FREELIST_ENTRY_SIZE)

PacketPtr L2CacheAM::SpmStateMachine::buildSpmAccessPacket(
    uint64_t spmAddr, bool isRead, size_t pktSize)
{
    RequestPtr _inner_req = std::make_shared<Request>(
        spmAddr, pktSize,
        Request::UNCACHEABLE, innerRequestorId
    );
    PacketPtr _pkt = nullptr;
    if (isRead) {
        _pkt = Packet::createRead(_inner_req);
    } else {
        _pkt = Packet::createWrite(_inner_req);
    }
    assert(_pkt);
    _pkt->allocate();

    return _pkt;
}

/////////////////////////////////////////////////////
// State Machines
/////////////////////////////////////////////////////

bool L2CacheAM::SpmStateMachine::addPacket(PacketPtr pkt)
{
    if (pendingAsyncMemPkts.size() < capacity) {
        pendingAsyncMemPkts.push_back(pkt);
        return true;
    }
    return false;
}

void L2CacheAM::AmReqStateMachine::rebuildAsyncMemReqQueue()
{
    if (val > 0) {
        --val;
        uint64_t spm_addr = asyncMemReqBase +
            val * MEMREQ_ENTRY_SIZE;
        PacketPtr _pkt = buildSpmAccessPacket(
            spm_addr, false, MEMREQ_ENTRY_SIZE);

        // clear spm req queue
        uint8_t tmpBuf[MEMREQ_ENTRY_SIZE] = {0};
        _pkt->setData(tmpBuf);
        // cpuSidePort.schedInnerTimingReq(_pkt, clockEdge(dataLatency));
        parent->cpuSidePort.schedInnerTimingReq(
            _pkt, clockEdge(logicLatency));
    } else {
        // asyncMemReqEnd = asyncMemReqBase +
        //     asyncMemReqLength * MEMREQ_ENTRY_SIZE;
        state = BUILD_FREE_LIST;
        val = asyncMemReqLength - 1;
        asyncMemFreeHead = 0;
        asyncMemFreeTail = asyncMemReqLength - 1;
        rebuildAsyncMemReqFreeList();
    }
}

void L2CacheAM::AmReqStateMachine::rebuildAsyncMemReqFreeList()
{
     if (val> 0) {
        --val;
        uint64_t spm_addr = asyncMemReqBase +
            asyncMemReqLength * MEMREQ_ENTRY_SIZE +
            val * FREELIST_ENTRY_SIZE;
        PacketPtr _pkt = buildSpmAccessPacket(
            spm_addr, false, FREELIST_ENTRY_SIZE);

        // clear spm req queue
        uint16_t freeId = val + 1;
        _pkt->setData((uint8_t*)&freeId);
        // cpuSidePort.schedInnerTimingReq(_pkt, clockEdge(dataLatency));
        parent->cpuSidePort.schedInnerTimingReq(
            _pkt, clockEdge(logicLatency));
    } else {
        // asyncMemFreeEnd = asyncMemReqEnd +
        //     asyncMemReqLength * FREELIST_ENTRY_SIZE;
        state = BUILD_FIN_LIST;
        val = asyncMemReqLength;
        asyncMemFinishHead = 0;
        asyncMemFinishTail = 0;
        asyncMemFinishCount = 0;
        rebuildAsyncMemReqFinList();
    }
}

void L2CacheAM::AmReqStateMachine::rebuildAsyncMemReqFinList()
{
    if (val > 0) {
        --val;
        uint64_t spm_addr = asyncMemReqBase +
            asyncMemReqLength * (MEMREQ_ENTRY_SIZE + FREELIST_ENTRY_SIZE) +
            val * FREELIST_ENTRY_SIZE;
        PacketPtr _pkt = buildSpmAccessPacket(
            spm_addr, false, FREELIST_ENTRY_SIZE);

        // clear spm req queue
        uint8_t tmpBuf[FREELIST_ENTRY_SIZE] = {0};
        _pkt->setData(tmpBuf);
        // cpuSidePort.schedInnerTimingReq(_pkt, clockEdge(dataLatency));
        parent->cpuSidePort.schedInnerTimingReq(
            _pkt, clockEdge(logicLatency));
    } else {
        // asyncMemFinEnd = asyncMemFreeEnd +
        //     asyncMemReqLength * FREELIST_ENTRY_SIZE;
        asyncMemOutstandingCount = 0;
        val = 0;
        ready_to_serve();
        if (outstandingAsyncMemPkt) {
            outstandingAsyncMemPkt->makeTimingResponse();
            parent->cpuSidePort.schedTimingResp(outstandingAsyncMemPkt,
                clockEdge(logicLatency));
            outstandingAsyncMemPkt = nullptr;
        }
    }
}

void L2CacheAM::AmReqStateMachine::retryProcessAm()
{
    if (state == READY_TO_SERVE && !pendingAsyncMemPkts.empty()) {
        PacketPtr tmp = pendingAsyncMemPkts.front();
        pendingAsyncMemPkts.pop_front();
        parent->clearBlocked(Blocked_NoAMPktQueues);
        if (tmp->cmd == MemCmd::AsyncMemLdReq) {
            assert(outstandingAsyncMemPkt == nullptr);
            outstandingAsyncMemPkt = tmp;
            parent->spmstats.numALoad += 1;
            spmFsmProcess(ALOAD_REQ, nullptr);
        } else if (tmp->cmd == MemCmd::AsyncMemWrReq) {
            parent->spmstats.numAStore += 1;
            assert(outstandingAsyncMemPkt == nullptr);
            outstandingAsyncMemPkt = tmp;
            spmFsmProcess(ASTORE_REQ, nullptr);
        } else {
            assert(0);
        }
    }
}

void L2CacheAM::AmReqStateMachine::fillReqEntryHelper(
    uint64_t spm_addr_pkt_id)
{
    assert(0 < spm_addr_pkt_id &&
           spm_addr_pkt_id <= asyncMemReqLength);
    spm_addr_pkt_id = spm_addr_pkt_id - 1;

    PacketPtr reqpkt = outstandingAsyncMemPkt;

    uint64_t headregval = reqpkt->req->getExtraData();
    Addr memreq_entry_addr = asyncMemReqBase +
        spm_addr_pkt_id * MEMREQ_ENTRY_SIZE;
    PacketPtr spm_pkt = buildSpmAccessPacket(
        memreq_entry_addr,
        false, MEMREQ_ENTRY_SIZE);
    spm_pkt->setData((uint8_t*)&entry);

    AsyncMemReqEntryState _test_state = getMemReqEntryState(
        *(AsyncMemReqEntry*)parent->spmDebugAccess(memreq_entry_addr)
    );
    assert(_test_state == AMRE_FINISH ||
           _test_state == AMRE_IDLE);

    // write req entry
    // cpuSidePort.schedInnerTimingReq(spm_pkt,
    //     clockEdge(dataLatency));
    parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
        clockEdge(logicLatency));

    state = FILL_REQ_ENTRY;

    reqpkt->makeTimingResponse();

    // return spm_addr_pkt_id + 1 to user
    val2 = spm_addr_pkt_id + 1;
    reqpkt->setData((uint8_t *)&val2);
    val2 = spm_addr_pkt_id;

    parent->cpuSidePort.schedTimingResp(reqpkt,
        clockEdge(logicLatency));
    outstandingAsyncMemPkt = nullptr;
}

void L2CacheAM::AmReqStateMachine::allocReqEntryHelper(
    AsyncMemReqEntryState _state)
{
    /* |---------------------------------
     * |64      49 | 48                0|
     * |---------------------------------
     * |  free_id  |     spm_addr       |
     * |---------------------------------
     */
    uint64_t async_req_spm_addr = 0;
    PacketPtr reqpkt = outstandingAsyncMemPkt;
    reqpkt->writeData((uint8_t *)&async_req_spm_addr);
    const uint64_t id_mask = (0xffffllu << 48);
    uint64_t spm_addr_pkt_id =
        (async_req_spm_addr & id_mask) >> 48;
    async_req_spm_addr &= ~id_mask;

    DPRINTF(CacheAM, "AmReq::%s(id=%ld)\n",
            _state == AMRE_ALOAD ? "aload" :
            _state == AMRE_ASTORE? "astore" : "unknown",
            spm_addr_pkt_id);
    entry = parent->buildMemReqEntry(
        _state, async_req_spm_addr, reqpkt->getAddr());


    asyncMemOutstandingCount++;
    if (parent->spmstats.maxOutstandingReq.total() <
        asyncMemOutstandingCount) {
        parent->spmstats.maxOutstandingReq = asyncMemOutstandingCount;
    }

    fillReqEntryHelper(spm_addr_pkt_id);
}

void L2CacheAM::AmReqStateMachine::spmFsmProcess(
    SpmFSMEvent spmFsmEvent, void *data)
{
    DPRINTF(CacheAM, "AmReq::spmFsmProcess state = %s, event = %s"
        " at cycle %ld\n",
        spmStateStr[state],
        spmFSMEventStr[spmFsmEvent],
        parent->curCycle());
    switch (state) {
        case READY_TO_SERVE: {
            switch (spmFsmEvent) {
                case RECONF_QUEUE_BASE:
                case RECONF_QUEUE_LENGTH:
                    state = BUILD_REQ_QUEUE;
                    val = asyncMemReqLength;
                    rebuildAsyncMemReqQueue();
                    break;
                case ALOAD_REQ: {
                    allocReqEntryHelper(AMRE_ALOAD);
                    break;
                }
                case ASTORE_REQ: {
                    allocReqEntryHelper(AMRE_ASTORE);
                    break;
                }
                default:
                    assert(0); // should not reach here!
            }
            break;
        }
        case FILL_REQ_ENTRY: {
            switch(spmFsmEvent) {
                case RECV_SPM_WRITE_RESP: {
                    AsyncMemReqEntryState _state;
                    uintptr_t spmAddr, memAddr;
                    uint16_t finListPos = 0;
                    decodeMemReqEntry(entry, _state,
                        spmAddr, memAddr,finListPos);

                    if (_state == AMRE_ALOAD)
                    {
                        RequestPtr _req = std::make_shared<Request>(
                            memAddr, 8, Request::UNCACHEABLE, innerRequestorId
                        );
                        PacketPtr _pkt =
                            new Packet(_req, MemCmd::ReadReq, 8);
                        _pkt->allocate();
                        _pkt->req->setReqInstSeqNum(val2);
                        parent->memSidePort.schedTimingReq(
                            _pkt, clockEdge(logicLatency));
                        val = 0;
                        val2 = 0;
                        ready_to_serve();
                    } else if (_state == AMRE_ASTORE) {
                        RequestPtr _inner_req = std::make_shared<Request>(
                            spmAddr, 8, Request::UNCACHEABLE, innerRequestorId
                        );
                        PacketPtr _pkt = Packet::createRead(_inner_req);
                        _pkt->allocate();
                        state = EXEC_ASTORE;
                        val = memAddr;
                        // cpuSidePort.schedInnerTimingReq(_pkt,
                        //     clockEdge(dataLatency));
                        parent->cpuSidePort.schedInnerTimingReq(_pkt,
                            clockEdge(logicLatency));
                    } else {
                        assert(0);
                    }
                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case EXEC_ASTORE: {
            switch (spmFsmEvent) {
                case RECV_SPM_READ_RESP: {
                    RequestPtr _inner_req = std::make_shared<Request>(
                        val, 8, Request::UNCACHEABLE,
                        innerRequestorId
                    );
                    PacketPtr _pkt = Packet::createWrite(_inner_req);
                    _pkt->allocate();
                    PacketPtr spmpkt = (PacketPtr)data;
                    uint8_t spmdata[spmpkt->getSize()] = {0};
                    spmpkt->writeData(spmdata);
                    delete spmpkt;

                    _pkt->setData(spmdata);
                    _pkt->req->setReqInstSeqNum(val2);
                    parent->memSidePort.schedTimingReq(
                        _pkt, clockEdge(logicLatency));
                    val = 0;
                    val2 = 0;
                    ready_to_serve();
                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case BUILD_REQ_QUEUE: {
            switch(spmFsmEvent) {
                case RECV_SPM_WRITE_RESP:
                    rebuildAsyncMemReqQueue();
                    break;
                default:
                    assert(0);
            }
            break;
        }
        case BUILD_FREE_LIST: {
            switch(spmFsmEvent) {
                case RECV_SPM_WRITE_RESP:
                    rebuildAsyncMemReqFreeList();
                    break;
                default:
                    assert(0);
            }
            break;
        }
        case BUILD_FIN_LIST: {
            switch(spmFsmEvent) {
                case RECV_SPM_WRITE_RESP:
                    rebuildAsyncMemReqFinList();
                    break;
                default:
                    assert(0);
            }
            break;
        }
        default: {
            // should not reach here!
            assert(0);
        }
    }
}

void L2CacheAM::AmRespStateMachine::retryProcessAm()
{
    if (state == READY_TO_SERVE && !pendingAsyncMemPkts.empty()) {
        PacketPtr tmp = pendingAsyncMemPkts.front();
        pendingAsyncMemPkts.pop_front();
        // memSidePort.clearBlocked();
        if (tmp->cmd == MemCmd::ReadResp) {
            spmFsmProcess(RECV_MEM_READ_RESP, tmp);
        } else if (tmp->cmd == MemCmd::WriteResp){
            spmFsmProcess(RECV_MEM_WRITE_RESP, tmp);
        } else if (tmp->cmd == MemCmd::CfgRegReq) {
            // getfin is accomplished by MEMACCRD
            assert(outstandingAsyncMemPkt == nullptr);
            outstandingAsyncMemPkt = tmp;
            parent->spmstats.numGetFin += 1;
            spmFsmProcess(GETFIN_REQ, nullptr);
        } else {
            assert(0);
        }
    }
}

void L2CacheAM::AmRespStateMachine::getFinListIfRemains()
{
    if (pullRemains > 0) {
        uint64_t finheadAddr =
            FINLIST_SPM_ADDR(asyncMemFinishHead);
        int remains = pullRemains;
        if (asyncMemFinishHead + remains > asyncMemReqLength) {
            remains = asyncMemReqLength - asyncMemFinishHead;
        }
        PacketPtr spm_pkt = buildSpmAccessPacket(
            finheadAddr, true,
            FREELIST_ENTRY_SIZE * remains);
        parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
            clockEdge(logicLatency));
        asyncMemFinishHead = (asyncMemFinishHead + remains) %
            asyncMemReqLength;
        pullRemains -= remains;
        state = GET_FINLIST;
    } else {
        if (outstandingAsyncMemPkt != nullptr) {
            getfinRespOrPull();
        } else {
            ready_to_serve();
        }
    }
}

void L2CacheAM::AmRespStateMachine::getfinRespHelper(uint16_t respReg[])
{
    PacketPtr reqpkt = outstandingAsyncMemPkt;
    assert(reqpkt != nullptr);

    char debug_str_buf[8*FL_REG_BYTES] = {0};
    int debug_str_pos = 0;
    for (int i = 1; i <= respReg[0]; ++i) {
        int sprintf_ret =
            sprintf(&debug_str_buf[debug_str_pos],"%d,",
                respReg[i]);
        assert(sprintf_ret > 0);
        debug_str_pos += sprintf_ret;
    }
    DPRINTF(CacheAM, "AmResp::getfin response -- %d@{%s}\n",
            respReg[0], debug_str_buf);
    static int responseFinListNum = 0;
    responseFinListNum += respReg[0];
    DPRINTF(CacheAM, "AmResp::debug -- %d/%d(outstanding:%d)\n",
        responseFinListNum,
        parent->spmstats.numALoad.value() +
        parent->spmstats.numAStore.value(),
        asyncMemOutstandingCount);

    reqpkt->makeTimingResponse();
    reqpkt->setData((uint8_t*)respReg);
    memset(respReg, 0, FL_REG_BYTES);
    parent->cpuSidePort.schedTimingResp(reqpkt,
        clockEdge(logicLatency));
    outstandingAsyncMemPkt = nullptr;
    ready_to_serve();
}

void L2CacheAM::AmRespStateMachine::getfinRespOrPull()
{
    PacketPtr reqpkt = outstandingAsyncMemPkt;
    assert(reqpkt != nullptr);

    if (tempFinListBackReg[0] > 0) {
        getfinRespHelper(tempFinListBackReg);
    } else if (tempFinListReg[0] > 0) {
        getfinRespHelper(tempFinListReg);
    } else if (asyncMemFinishHead != asyncMemFinishTail) {
        // pull new ids from Finish List(in SPM)
        assert(pullRemains == 0);
        pullRemains =
            (asyncMemFinishTail + asyncMemReqLength - asyncMemFinishHead)
                % asyncMemReqLength;
        if (pullRemains > FL_REG_LENGTH - 1) {
            pullRemains = FL_REG_LENGTH - 1;
        }

        getFinListIfRemains();
    } else {
        uint8_t getfin_empty_value[FL_REG_BYTES] = {0};
        reqpkt->makeTimingResponse();
        reqpkt->setData(getfin_empty_value);
        parent->cpuSidePort.schedTimingResp(reqpkt,
            clockEdge(logicLatency));
        outstandingAsyncMemPkt = nullptr;
        ready_to_serve();
    }
}

bool L2CacheAM::AmRespStateMachine::putIntoFinListReg(
    uint16_t spm_addr_pkt_id)
{
    DPRINTF(CacheAM, "AmResp::putIntoFinListReg id=%d\n",
        spm_addr_pkt_id);
    --asyncMemOutstandingCount;
    if (tempFinListReg[0] + 1 < FL_REG_LENGTH) {
        ++tempFinListReg[0];
        tempFinListReg[tempFinListReg[0]] = spm_addr_pkt_id;
        return true;
    } else {
        return false;
    }
}

void L2CacheAM::AmRespStateMachine::tryFillFinEntry()
{
    if (!finListRegFull()) {
        uint64_t spm_addr_pkt_id = val2 + 1;
        bool isOk = putIntoFinListReg(spm_addr_pkt_id);
        assert(true == isOk);

        AsyncMemReqEntryState _state;
        uintptr_t spmAddr, memAddr;
        uint16_t finListPos;
        decodeMemReqEntry(entry, _state,
            spmAddr, memAddr, finListPos);
        finListPos = asyncMemFinishTail;
        entry = buildMemReqEntry(
            AMRE_FINISH, spmAddr, memAddr);
        entry.finListPos = finListPos;

        PacketPtr spm_pkt = buildSpmAccessPacket(asyncMemReqBase +
            val2 * MEMREQ_ENTRY_SIZE,
            false, MEMREQ_ENTRY_SIZE);
        spm_pkt->setData((uint8_t*)&entry);
        parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
            clockEdge(logicLatency));

        state = FIN_REQ_ENTRY;
    } else {
        writebackFinListIfRemains();
    }
}

void L2CacheAM::AmRespStateMachine::writebackFinListIfRemains()
{
    if (tempFinListReg[0] > 0) {
        uint64_t fintailAddr =
            FINLIST_SPM_ADDR(asyncMemFinishTail);
        int remains = tempFinListReg[0];
        if (tempFinListBackReg[0] == 0) {
            memcpy(tempFinListBackReg, tempFinListReg, FL_REG_BYTES);
            memset(tempFinListReg, 0, FL_REG_BYTES);
            parent->schedule(retryNextEvent, clockEdge(logicLatency));
        } else {
            if (asyncMemFinishTail + remains > asyncMemReqLength) {
                remains = asyncMemReqLength - asyncMemFinishTail;
            }

            // write freeList
            PacketPtr spm_pkt = buildSpmAccessPacket(
                fintailAddr, false,
                FREELIST_ENTRY_SIZE * remains);
            uint16_t wbContents[FL_REG_LENGTH] = {0};
            for (int i = 0; i < remains; ++i) {
                wbContents[i] = tempFinListReg[tempFinListReg[0]--];
            }
            spm_pkt->setData((uint8_t*)wbContents);
            parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
                clockEdge(logicLatency));
            asyncMemFinishTail = (asyncMemFinishTail + remains) %
                asyncMemReqLength;
        }
        state = WRITEBACK_FINLIST;
    } else {
        tryFillFinEntry();
    }
}

void L2CacheAM::AmRespStateMachine::spmFsmProcess(
    SpmFSMEvent spmFsmEvent, void* data)
{
    DPRINTF(CacheAM, "AmResp::spmFsmProcess state = %s, event = %s"
        " at cycle %ld\n",
        spmStateStr[state],
        spmFSMEventStr[spmFsmEvent],
        curCycle());
    switch (state) {
        case READY_TO_SERVE: {
            switch (spmFsmEvent) {
                case GETFIN_REQ: {
                    getfinRespOrPull();
                    break;
                }
                case RECV_MEM_READ_RESP: {
                    PacketPtr readRespPkt = (PacketPtr)data;
                    assert(readRespPkt != nullptr);

                    int spm_addr_pkt_id = readRespPkt->req->getReqInstSeqNum();
                    // read memreq entry
                    PacketPtr spm_pkt = buildSpmAccessPacket(asyncMemReqBase +
                        spm_addr_pkt_id * MEMREQ_ENTRY_SIZE,
                        true, MEMREQ_ENTRY_SIZE);

                    // cpuSidePort.schedInnerTimingReq(spm_pkt,
                    //     clockEdge(dataLatency));
                    parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
                        clockEdge(logicLatency));
                    state = EXEC_ALOAD;
                    val = (uint64_t) data;
                    val2 = spm_addr_pkt_id;
                    break;
                }
                case RECV_MEM_WRITE_RESP: {
                    PacketPtr writeRespPkt = (PacketPtr)data;
                    assert(writeRespPkt != nullptr);
                    int spm_addr_pkt_id =
                        writeRespPkt->req->getReqInstSeqNum();
                    delete writeRespPkt;

                    // read memreq entry
                    PacketPtr spm_pkt = buildSpmAccessPacket(asyncMemReqBase +
                        spm_addr_pkt_id * MEMREQ_ENTRY_SIZE,
                        true, MEMREQ_ENTRY_SIZE);

                    // cpuSidePort.schedInnerTimingReq(spm_pkt,
                    //     clockEdge(dataLatency));
                    parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
                        clockEdge(logicLatency));
                    state = GET_REQ_ENTRY;
                    val2 = spm_addr_pkt_id;
                    break;
                }
                default:
                    assert(0); // should not reach here!
            }
            break;
        }
        case EXEC_ALOAD: {
            switch (spmFsmEvent) {
                case RECV_SPM_READ_RESP: {
                    PacketPtr readRespPkt = (PacketPtr)val;
                    assert(readRespPkt != nullptr);

                    PacketPtr spmpkt = (PacketPtr)data;
                    spmpkt->writeData((uint8_t*)&entry);
                    delete spmpkt;

                    AsyncMemReqEntryState _state;
                    uintptr_t spmAddr, memAddr;
                    uint16_t finListPos;
                    decodeMemReqEntry(entry, _state,
                        spmAddr, memAddr, finListPos);

                    assert(_state == AMRE_ALOAD);
                    RequestPtr _inner_req = std::make_shared<Request>(
                        spmAddr, 8, Request::UNCACHEABLE, innerRequestorId
                    );
                    PacketPtr _pkt = Packet::createWrite(_inner_req);
                    _pkt->allocate();

                    uint8_t d[readRespPkt->getSize()] = {0};
                    readRespPkt->writeData(d);
                    _pkt->setData(d);
                    delete readRespPkt;

                    state = FIN_ALOAD;
                    entry = entry;
                    // cpuSidePort.schedInnerTimingReq(_pkt,
                    //     clockEdge(dataLatency));
                    parent->cpuSidePort.schedInnerTimingReq(_pkt,
                        clockEdge(logicLatency));
                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case FIN_ALOAD: {
            switch(spmFsmEvent) {
                case RECV_SPM_WRITE_RESP: {
                    PacketPtr spmpkt = (PacketPtr)data;
                    delete spmpkt;

                    tryFillFinEntry();
                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case GET_REQ_ENTRY: {
            switch (spmFsmEvent) {
                case RECV_SPM_READ_RESP: {
                    PacketPtr spmpkt = (PacketPtr)data;
                    spmpkt->writeData((uint8_t*)&entry);
                    delete spmpkt;

                    AsyncMemReqEntryState _state;
                    uintptr_t spmAddr, memAddr;
                    uint16_t finListPos;
                    decodeMemReqEntry(entry, _state,
                        spmAddr, memAddr, finListPos);
                    assert(_state == AMRE_ASTORE);

                    tryFillFinEntry();
                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case FIN_REQ_ENTRY: {
            switch(spmFsmEvent) {
                case RECV_SPM_WRITE_RESP: {
                    PacketPtr spmpkt = (PacketPtr)data;
                    delete spmpkt;

                    val = 0;
                    val2 = 0;
                    ready_to_serve();
                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case WRITEBACK_FINLIST: {
            switch(spmFsmEvent) {
                case RECV_SPM_WRITE_RESP: {
                    PacketPtr spmpkt = (PacketPtr)data;
                    delete spmpkt;
                    __attribute__((fallthrough));
                    /* fall through */
                }
                case RETRY_EVENT: {
                    writebackFinListIfRemains();

                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case GET_FINLIST: {
            switch(spmFsmEvent)
            {
                case RECV_SPM_READ_RESP: {
                    PacketPtr spmpkt = (PacketPtr)data;
                    spmpkt->writeData(
                        (uint8_t*)&tempFinListReg[tempFinListReg[0] + 1]);

                    tempFinListReg[0] += spmpkt->getSize() / sizeof(uint16_t);
                    delete spmpkt;

                    getFinListIfRemains();

                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        default: {
            // should not reach here!
            assert(0);
        }
    }
}

void L2CacheAM::FreeListStateMachine::retryProcessAm()
{
    if (state == READY_TO_SERVE && !pendingAsyncMemPkts.empty()) {
        PacketPtr tmp = pendingAsyncMemPkts.front();
        pendingAsyncMemPkts.pop_front();
        // memSidePort.clearBlocked();
        if (tmp->cmd == MemCmd::CfgRegReq) {
            assert(outstandingAsyncMemPkt == nullptr);
            outstandingAsyncMemPkt = tmp;
            int regid = tmp->getAddr() - 0x1000000000000000llu;
            switch (regid)
            {
                case MEMACC_CFG_GETFREE: {
                    // parent->spmstats.numGetFin += 1;
                    spmFsmProcess(GET_FREELIST_REQ, nullptr);
                    break;
                }
                case MEMACC_CFG_WRITEFREE: {
                    // parent->spmstats.numGetFin += 1;
                    spmFsmProcess(WRITE_FREELIST_REQ, nullptr);
                    break;
                }
                default:
                    assert(0);
            }
        } else {
            assert(0);
        }
    }
}

void L2CacheAM::FreeListStateMachine::writebackFreeListIfRemains()
{
    if (wbData[0] > 0) {
        uint64_t freetailAddr =
            FREELIST_SPM_ADDR(asyncMemFreeTail);
        int remains = wbData[0];
        if (asyncMemFreeTail + remains > asyncMemReqLength) {
            remains = asyncMemReqLength - asyncMemFreeTail;
        }

        // write freeList
        PacketPtr spm_pkt = buildSpmAccessPacket(
            freetailAddr, false,
            FREELIST_ENTRY_SIZE * remains);
        uint16_t wbContents[FL_REG_LENGTH] = {0};
        for (int i = 0; i < remains; ++i) {
            wbContents[i] = wbData[wbData[0]--];
        }
        spm_pkt->setData((uint8_t*)wbContents);
        parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
            clockEdge(logicLatency));
        asyncMemFreeTail = (asyncMemFreeTail + remains) %
            asyncMemReqLength;

        state = WRITEBACK_FREELIST;
    } else {
        ready_to_serve();
    }
}

void L2CacheAM::FreeListStateMachine::responseAndPull()
{
    PacketPtr reqpkt = outstandingAsyncMemPkt;
    if (tempFreeListReg[0] != 0) {
        char debug_str_buf[8*FL_REG_BYTES] = {0};
        int debug_str_pos = 0;
        for (int i = 1; i <= tempFreeListReg[0]; ++i) {
            int sprintf_ret = sprintf(&debug_str_buf[debug_str_pos],
                "%d,", tempFreeListReg[i]);
            assert(sprintf_ret > 0);
            debug_str_pos += sprintf_ret;
        }
        DPRINTF(CacheAM, "AmResp::getfree response -- %d@{%s}\n",
            tempFreeListReg[0], debug_str_buf);

        reqpkt->makeTimingResponse();
        reqpkt->setData((uint8_t*)tempFreeListReg);
        memset(tempFreeListReg, 0, FL_REG_BYTES);
        parent->cpuSidePort.schedTimingResp(reqpkt,
            clockEdge(logicLatency));
        outstandingAsyncMemPkt = nullptr;
    } else if (asyncMemFreeHead == asyncMemFreeTail) {
        uint8_t getfree_empty_value[FL_REG_BYTES] = {0};
        reqpkt->makeTimingResponse();
        reqpkt->setData(getfree_empty_value);
        parent->cpuSidePort.schedTimingResp(reqpkt,
            clockEdge(logicLatency));
        outstandingAsyncMemPkt = nullptr;
        ready_to_serve();
        return;
    }

    // pull new ids from Free List(in SPM)
    assert(pullRemains == 0);
    pullRemains =
        (asyncMemFreeTail + asyncMemReqLength - asyncMemFreeHead)
            % asyncMemReqLength;
    if (pullRemains > FL_REG_LENGTH - 1) {
        pullRemains = FL_REG_LENGTH - 1;
    }

    getFreeListIfRemains();
}

void L2CacheAM::FreeListStateMachine::getFreeListIfRemains()
{
    if (pullRemains > 0) {
        uint64_t freeheadAddr =
            FREELIST_SPM_ADDR(asyncMemFreeHead);
        int remains = pullRemains;
        if (asyncMemFreeHead + remains > asyncMemReqLength) {
            remains = asyncMemReqLength - asyncMemFreeHead;
        }
        PacketPtr spm_pkt = buildSpmAccessPacket(
            freeheadAddr, true,
            FREELIST_ENTRY_SIZE * remains);
        parent->cpuSidePort.schedInnerTimingReq(spm_pkt,
            clockEdge(logicLatency));
        asyncMemFreeHead = (asyncMemFreeHead + remains) % asyncMemReqLength;
        pullRemains -= remains;
        state = GET_FREELIST;
    } else {
        if (outstandingAsyncMemPkt != nullptr) {
            responseAndPull();
        } else {
            ready_to_serve();
        }
    }
}

void L2CacheAM::FreeListStateMachine::spmFsmProcess(
    SpmFSMEvent spmFsmEvent, void* data)
{
    DPRINTF(CacheAM, "FreeList::spmFsmProcess state = %s, event = %s"
        " at cycle %ld\n",
        spmStateStr[state],
        spmFSMEventStr[spmFsmEvent],
        curCycle());
    switch (state) {
        case READY_TO_SERVE: {
            switch (spmFsmEvent) {
                case WRITE_FREELIST_REQ: {
                    PacketPtr reqpkt = outstandingAsyncMemPkt;
                    reqpkt->writeData((uint8_t*)wbData);

                    while (wbData[0] > 0 &&
                        tempFreeListReg[0] != FL_REG_LENGTH - 1) {
                        tempFreeListReg[++tempFreeListReg[0]] =
                            wbData[wbData[0]--];
                    }

                    reqpkt->makeTimingResponse();
                    parent->cpuSidePort.schedTimingResp(reqpkt,
                        clockEdge(logicLatency));
                    outstandingAsyncMemPkt = nullptr;

                    writebackFreeListIfRemains();

                    break;
                }
                case GET_FREELIST_REQ: {
                    PacketPtr reqpkt = outstandingAsyncMemPkt;

                    responseAndPull();

                    break;
                }
                default:
                    assert(0); // should not reach here!
            }
            break;
        }
        case WRITEBACK_FREELIST: {
            switch(spmFsmEvent)
            {
                case RECV_SPM_WRITE_RESP: {
                    PacketPtr spmpkt = (PacketPtr)data;
                    delete spmpkt;

                    writebackFreeListIfRemains();

                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        case GET_FREELIST: {
            switch(spmFsmEvent)
            {
                case RECV_SPM_READ_RESP: {
                    PacketPtr spmpkt = (PacketPtr)data;
                    spmpkt->writeData((uint8_t*)&tempFreeListReg[
                        tempFreeListReg[0] + 1]);

                    tempFreeListReg[0] += spmpkt->getSize() / sizeof(uint16_t);
                    delete spmpkt;

                    getFreeListIfRemains();

                    break;
                }
                default:
                    assert(0);
            }
            break;
        }
        default: {
            // should not reach here!
            assert(0);
        }
    }
}

// bool L2CacheAM::retryProcessAMResp()
// {
//     if (!pendingAsyncMemPkts.empty()) {
//         assert(outstandingAsyncMemPkt == nullptr);
//         outstandingAsyncMemPkt = pendingAsyncMemPkts.front();
//         pendingAsyncMemPkts.pop_front();
//         clearBlocked(Blocked_NoAMPktQueues);
//         if (outstandingAsyncMemPkt->cmd == MemCmd::AsyncMemLdReq) {
//             spmstats.numALoad += 1;
//             spmFsmProcess(ALOAD_REQ, nullptr);
//         } else if (outstandingAsyncMemPkt->cmd == MemCmd::AsyncMemWrReq) {
//             spmstats.numAStore += 1;
//             spmFsmProcess(ASTORE_REQ, nullptr);
//         } else if (outstandingAsyncMemPkt->cmd == MemCmd::TestFinReq) {
//             spmstats.numTestFin += 1;
//             spmFsmProcess(TESTFIN_REQ, nullptr);
//         }
//         return true;
//     } else {
//         return false;
//     }
// }

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
        // DPRINTF(CacheAM,
        //         "%s L2CacheAM: access cache at %lx\n",
        //         __func__, pkt->getAddr());
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
