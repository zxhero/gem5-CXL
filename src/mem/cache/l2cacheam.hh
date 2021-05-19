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

#define FL_REG_LENGTH 32

class CacheBlk;
struct L2CacheAMParams;
class MSHR;

enum MemAccConfigRegs
{
    MEMACC_CFG_QBASE =   0x0,
    MEMACC_CFG_QLENGTH = 0x1,
    MEMACC_CFG_HEAD0 =   0x2,
    MEMACC_CFG_GETFIN =  0x3,

    MEMACC_CFG_COUNT,
};

enum SpmState {
    READY_TO_SERVE,
    BUILD_REQ_QUEUE,
    BUILD_FREE_LIST,
    BUILD_FIN_LIST,
    ALLOC_REQ_ENTRY,
    FILL_REQ_ENTRY,
    WRITEBACK_FREELIST,
    WRITEBACK_FINLIST,
    EXEC_ASTORE,
    EXEC_ALOAD,
    EXEC_TESTFIN,
    EXEC_GETFIN,
    FIN_TEST_FIN,
    // FIN_GET_FIN,
    WRITE_FREE_LIST,
    GET_REQ_ENTRY,
    FIN_REQ_ENTRY,
    ALLOC_FIN_ENTRY,
    FILL_FIN_ENTRY,
    CLEAR_FIN_LIST,
    FIN_ALOAD,
    SPM_STATE_COUNT
};
static const char* spmStateStr[SPM_STATE_COUNT] = {
    "READY_TO_SERVE",
    "BUILD_REQ_QUEUE",
    "BUILD_FREE_LIST",
    "BUILD_FIN_LIST",
    "ALLOC_REQ_ENTRY",
    "FILL_REQ_ENTRY",
    "WRITEBACK_FREELIST",
    "WRITEBACK_FINLIST",
    "EXEC_ASTORE",
    "EXEC_ALOAD",
    "EXEC_TESTFIN",
    "EXEC_GETFIN",
    "FIN_TEST_FIN",
    // "FIN_GET_FIN",
    "WRITE_FREE_LIST",
    "GET_REQ_ENTRY",
    "FIN_REQ_ENTRY",
    "ALLOC_FIN_ENTRY",
    "FILL_FIN_ENTRY",
    "CLEAR_FIN_LIST",
    "FIN_ALOAD"
};

enum SpmFSMEvent {
    RECV_SPM_READ_RESP,
    RECV_SPM_WRITE_RESP,
    RECV_MEM_READ_RESP,
    RECV_MEM_WRITE_RESP,
    RECONF_QUEUE_BASE,
    RECONF_QUEUE_LENGTH,
    ALOAD_REQ,
    ASTORE_REQ,
    TESTFIN_REQ,
    GETFIN_REQ,
    RETRY_EVENT,
    SPM_FSM_EVENT_COUNT
};
static const char* spmFSMEventStr[SPM_FSM_EVENT_COUNT] {
    "RECV_SPM_READ_RESP",
    "RECV_SPM_WRITE_RESP",
    "RECV_MEM_READ_RESP",
    "RECV_MEM_WRITE_RESP",
    "RECONF_QUEUE_BASE",
    "RECONF_QUEUE_LENGTH",
    "ALOAD_REQ",
    "ASTORE_REQ",
    "TESTFIN_REQ",
    "GETFIN_REQ",
    "RETRY_EVENT"
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

    Cycles logicLatency;
    unsigned spmWays;
    unsigned numSets;
    unsigned asyncmemOutstanding;
    unsigned asyncmemRespPending;
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

        /** Number of other requests */
        Stats::Vector stateTicks;
        /** Total bandwidth from this memory */
        Stats::Formula afterMemResp;
        /** Async Mem Request */
        Stats::Scalar numRequest;
        /** number of ALoad request */
        Stats::Scalar numALoad;
        /** number of AStore request */
        Stats::Scalar numAStore;
        /** number of TestFin request */
        Stats::Scalar numTestFin;
        /** number of Getfin request */
        Stats::Scalar numGetFin;
        /** max number of pending AM request */
        Stats::Scalar maxPendingReq;
        /** max number of outstanding AM request */
        Stats::Scalar maxOutstandingReq;
        /** max number of used entry */
        Stats::Scalar maxFinishCount;
        /** max number of used entry */
        Stats::Scalar maxUsedEntry;
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

    EventFunctionWrapper retryProcessAMReqRespEvent;
    std::list<PacketPtr> pendingAsyncMemPkts;
    std::list<PacketPtr> pendingAsyncMemRespPkts;
    EventFunctionWrapper retryNextEvent;

    struct AsyncMemReqEntry {
        uint64_t entry;
        uint16_t finListPos;
    };
    struct SpmStateMachine {
        SpmState state;
        union {
            int64_t val;
            AsyncMemReqEntry entry;
        };
        int64_t val2;
        SpmStateMachine() :
            state(READY_TO_SERVE), val(0), val2(0) {}
    } stateMachine;
    void spmFsmProcess(SpmFSMEvent spmFsmEvent, void* data);
    PacketPtr buildSpmAccessPacket(uint64_t spmAddr,
        bool isRead, size_t pktSize);
    void rebuildAsyncMemReqQueue();
    void rebuildAsyncMemReqFreeList();
    void rebuildAsyncMemReqFinList();
    void retryProcessAMReqAndResp();
    bool retryProcessAMReq();
    bool retryProcessAMResp();
    int retryProcessRoundRobin;

    enum AsyncMemReqEntryState {
        AMRE_IDLE,
        AMRE_ALOAD,
        AMRE_ASTORE,
        AMRE_FINISH
    };
    AsyncMemReqEntryState getAsyncMemReqEntryState(
        AsyncMemReqEntry entry) {
        return (AsyncMemReqEntryState)(entry.entry >> 57);
    }
    AsyncMemReqEntry buildMemReqEntry(AsyncMemReqEntryState state,
        uintptr_t spmAddr, uintptr_t memAddr) {
        AsyncMemReqEntry entry;
        entry.entry =
            ((uint64_t)state<<57llu) |
            ((spmAddr & 0x3ffffllu) << 39) |
            (memAddr & 0x7fffffffffllu);
        entry.finListPos = 0;
        return entry;
    }
    void decodeMemReqEntry(AsyncMemReqEntry entry,
        AsyncMemReqEntryState &state,
        uintptr_t &spmAddr, uintptr_t &memAddr,
        uint16_t &finListPos) {
        uint64_t _entry = entry.entry;
        memAddr = _entry & 0x7fffffffff;
        _entry >>= 39;
        spmAddr = (_entry & 0x3ffff) | 0x1000000000000000llu;
        _entry >>= 18;
        state = (AsyncMemReqEntryState) _entry;
        finListPos = entry.finListPos;
    }

    /* struct AsyncMemReqEntry {
        bool valid;
        bool finished;
        uintptr_t spm_addr;
        uintptr_t mem_addr;
        AsyncMemReqEntry(): valid(false),
            finished(false), spm_addr(0),
            mem_addr(0) {}
    };*/
    Tick lastSpmFsmTick;
    PacketPtr outstandingAsyncMemPkt;
    unsigned int asyncMemReqLength;
    uint64_t asyncMemReqBase;
    int64_t asyncMemFinishHead, asyncMemFinishTail;
    int64_t asyncMemFinishCount;
    int64_t asyncMemOutstandingCount;
    int64_t asyncMemFreeHead, asyncMemFreeTail;
    uintptr_t asyncMemReqEnd, asyncMemFreeEnd;
    uintptr_t asyncMemFinEnd;
    uintptr_t tempFreeListBase;
    uint16_t tempFreeListReg[FL_REG_LENGTH];
    uintptr_t tempFinListBase;
    uint16_t tempFinListReg[FL_REG_LENGTH];
    // std::vector<AsyncMemReqEntry> asyncMemReqs;
    std::vector<uint64_t> asyncMemConfigRegs;
    int allocAsyncMemReq(uint64_t spmAddr, Addr memAddr);
    bool checkAsyncMemReq(uint64_t handle);
    void allocReqEntryHelper(AsyncMemReqEntryState state);
    void fillReqEntryHelper(uint64_t spm_addr_pkt_id);
    void allocFinEntryHelper(uint64_t fin_entry);
    void getFinHelper();
    void retryNext();
    // return true if hit temp reg(free list buffer)
    bool accessFreeList(int pos, bool isRead, uint64_t &spm_addr_pkt_id);
    // return true if hit temp reg(free list buffer)
    bool accessFinList(int pos, bool isRead, uint64_t &spm_addr_pkt_id);

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
