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

#ifndef __MEM_CACHE_L1CACHEAM_HH__
#define __MEM_CACHE_L1CACHEAM_HH__

#include <cstdint>
#include <unordered_set>
#include <vector>

#include "base/types.hh"
#include "mem/cache/cache.hh"
#include "mem/cache/cacheampara.hh"
#include "mem/packet.hh"
#include "mem/simple_mem.hh"

struct L1CacheAMParams;

/**
 * A coherent cache that can be arranged in flexible topologies.
 */
class L1CacheAM : public Cache
{
    Cache *spmCache;
    Cache *normalCache;
    Addr spmBaseAddr;
    bool isUnifiedCache;
    unsigned spmCacheWays;
    unsigned numSets;
    const Cycles forwardLatency;
    bool finListRegValid;
    uint16_t tempFinListReg[FL_REG_LENGTH];
    bool freeListRegValid;
    uint16_t tempFreeListReg[FL_REG_LENGTH];

    std::vector<PacketPtr> asyncMemCmdPackets;
    std::vector<PacketPtr> spmBypassedPackets;

    // MemSidePort toSpmSidePort, toCacheSidePort;
    // CpuSidePort fromSpmSidePort, fromCacheSidePort;

  protected:
    void recvTimingReq(PacketPtr pkt) override;
    void recvTimingResp(PacketPtr pkt) override;
    Tick recvAtomic(PacketPtr pkt) override;
    // void recvFunctional(PacketPtr pkt) override;

  public:
    /** Instantiates a basic cache object. */
    L1CacheAM(const L1CacheAMParams *p);
    ~L1CacheAM();
};

#endif // __MEM_CACHE_L1CACHEAM_HH__
