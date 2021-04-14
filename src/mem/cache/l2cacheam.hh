/*
 * Copyright (c) 2012-2018 ARM Limited
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
 * Describes a cache
 */

#ifndef __MEM_CACHE_L2CACHEAM_HH__
#define __MEM_CACHE_L2CACHEAM_HH__

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "base/types.hh"
#include "mem/cache/cache.hh"
#include "mem/packet.hh"
#include "mem/request.hh"
#include "mem/simple_mem.hh"

class CacheBlk;
struct L2CacheAMParams;
class MSHR;

enum MemAccConfigRegs
{
    MEMACC_CFG_QBASE = 0x0,
    MEMACC_CFG_QLENGTH = 0x1,
    MEMACC_CFG_HEAD0 = 0x2,

    MEMACC_CFG_COUNT,
};

/**
 * A coherent cache that can be arranged in flexible topologies.
 */
class L2CacheAM : public Cache
{
    RequestorID innerRequestorId;
    /**
     * A deferred packet stores a packet along with its scheduled
     * transmission time
     */
    class DeferredPacket
    {

      public:

        const Tick tick;
        const PacketPtr pkt;

        DeferredPacket(PacketPtr _pkt, Tick _tick) : tick(_tick), pkt(_pkt)
        { }
    };

    unsigned spmWays;
    unsigned numSets;
    uint8_t* pmemAddr;
    AddrRange spmRange;
    struct SpmStats : public Stats::Group {
        SpmStats(L2CacheAM &mem);

        void regStats() override;

        const L2CacheAM &mem;

        /** Number of total bytes read from this memory */
        Stats::Vector bytesRead;
        /** Number of instruction bytes read from this memory */
        Stats::Vector bytesInstRead;
        /** Number of bytes written to this memory */
        Stats::Vector bytesWritten;
        /** Number of read requests */
        Stats::Vector numReads;
        /** Number of write requests */
        Stats::Vector numWrites;
        /** Number of other requests */
        Stats::Vector numOther;
        /** Read bandwidth from this memory */
        Stats::Formula bwRead;
        /** Read bandwidth from this memory */
        Stats::Formula bwInstRead;
        /** Write bandwidth from this memory */
        Stats::Formula bwWrite;
        /** Total bandwidth from this memory */
        Stats::Formula bwTotal;
    } spmstats;

    std::list<LockedAddr> spmLockedAddrList;
    void spmTrackLoadLocked(PacketPtr pkt);
    bool checkLockedAddrList(PacketPtr pkt);
    // Compare a store address with any locked addresses so we can
    // clear the lock flag appropriately.  Return value set to 'false'
    // if store operation should be suppressed (because it was a
    // conditional store and the address was no longer locked by the
    // requesting execution context), 'true' otherwise.  Note that
    // this method must be called on *all* stores since even
    // non-conditional stores must clear any matching lock addresses.
    bool spmWriteOK(PacketPtr pkt) {
        const RequestPtr &req = pkt->req;
        if (spmLockedAddrList.empty()) {
            // no locked addrs: nothing to check, store_conditional fails
            bool isLLSC = pkt->isLLSC();
            if (isLLSC) {
                req->setExtraData(0);
            }
            return !isLLSC; // only do write if not an sc
        } else {
            // iterate over list...
            return checkLockedAddrList(pkt);
        }
    }

    const Tick latency;
    const Tick latency_var;
    /**
     * Bandwidth in ticks per byte. The regulation affects the
     * acceptance rate of requests and the queueing takes place after
     * the regulation.
     */
    const double bandwidth;
    Tick getSpmLatency() const; // same as SimpleMemory's latency
    void spmAccess(PacketPtr pkt);
    bool spmRecvTimingReq(PacketPtr pkt);

    /**
     * Internal (unbounded) storage to mimic the delay caused by the
     * actual memory access. Note that this is where the packet spends
     * the memory latency.
     */
    std::list<DeferredPacket> packetQueue;

    /**
     * Track the state of the memory as either idle or busy, no need
     * for an enum with only two states.
     */
    bool isBusy;

    /**
     * Remember if we have to retry an outstanding request that
     * arrived while we were busy.
     */
    bool retryReq;

    /**
     * Remember if we failed to send a response and are awaiting a
     * retry. This is only used as a check.
     */
    bool retryResp;

/**
     * Release the memory after being busy and send a retry if a
     * request was rejected in the meanwhile.
     */
    void spmRelease();

    EventFunctionWrapper releaseEvent;

    /**
     * Dequeue a packet from our internal packet queue and move it to
     * the port where it will be sent as soon as possible.
     */
    void spmDequeue();

    EventFunctionWrapper dequeueEvent;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    // std::unique_ptr<Packet> pendingDelete;

    struct AsyncMemReqEntry {
        bool valid;
        bool finished;
        uintptr_t spm_addr;
        uintptr_t mem_addr;
        AsyncMemReqEntry(): valid(false),
            finished(false), spm_addr(0),
            mem_addr(0) {}
    };
    unsigned int asyncMemReqLength;
    std::vector<AsyncMemReqEntry> asyncMemReqs;
    std::vector<uint64_t> asyncMemConfigRegs;
    int allocAsyncMemReq(uint64_t spmAddr, Addr memAddr);
    bool checkAsyncMemReq(uint64_t handle);

  protected:
    void recvTimingResp(PacketPtr pkt) override;
    void recvTimingReq(PacketPtr pkt) override;
    Tick recvAtomic(PacketPtr pkt) override;
    void recvInnerTimingResp(PacketPtr pkt) override;
    // void recvFunctional(PacketPtr pkt) override;

  public:
    /** Instantiates a basic cache object. */
    L2CacheAM(const L2CacheAMParams *p);
    ~L2CacheAM();
};

#endif // __MEM_CACHE_L2CACHEAM_HH__
