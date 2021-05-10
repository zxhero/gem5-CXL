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

#include "mem/cache/l1cacheam.hh"

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
#include "params/L1CacheAM.hh"

L1CacheAM::L1CacheAM(const L1CacheAMParams *p)
    : Cache(p), spmBaseAddr(p->spm_base_addr),
    isUnifiedCache(p->is_unified),
    spmCacheWays(p->spm_init_capacity),
    forwardLatency(p->asyncmem_forward_delay)
{
    fatal_if(spmCacheWays > p->assoc,
        "spm ways must smaller than cache associativity");
    numSets = p->size / (p->system->cacheLineSize() * p->assoc);
    CacheParams spmCacheParams(*static_cast<const CacheParams*>(p));
    CacheParams normalCacheParams(*static_cast<const CacheParams*>(p));
    spmCacheParams.assoc = p->spm_init_capacity;
    normalCacheParams.assoc = p->assoc - p->spm_init_capacity;
    spmCache = new Cache(&spmCacheParams);
    normalCache = new Cache(&normalCacheParams);

    // toSpmSidePort.bind(spmCache->getPort("cpu_side"));
    // toCacheSidePort.bind(normalCache->getPort("cpu_side"));
    // fromSpmSidePort.bind(spmCache->getPort("mem_side"));
    // fromCacheSidePort.bind(spmCache->getPort("cpu_side"));
}

L1CacheAM::~L1CacheAM()
{
    delete spmCache;
    delete normalCache;
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

void L1CacheAM::recvTimingReq(PacketPtr pkt)
{
    if (pkt->cmd == MemCmd::AsyncMemWrReq ||
        pkt->cmd == MemCmd::AsyncMemLdReq) {
        uint64_t spm_addr = 0;
        pkt->writeData((uint8_t*) &spm_addr);
        DPRINTF(CacheAM,
            "%s L1CacheAM: async mem load/store at 0x%lx, spm_addr = 0x%lx\n",
            __func__, pkt->getAddr(), spm_addr);
        // asyncMemCmdPackets.push_back(pkt);
        memSidePort.schedTimingReq(pkt, clockEdge(forwardLatency));
        return ;
    }

    const Addr vaddr = pkt->getAddr();
    uintptr_t vaddr_prefix = vaddr >> 48;
    bool is_spm_addr = (vaddr_prefix == 0x1000);
    if (is_spm_addr) {
        // TODO: just bypass to L2 Cache
        //spmBypassedPackets.push_back(pkt);
        DPRINTF(CacheAM,
            "%s L1CacheAM: access spm at %lx\n",
            __func__, pkt->getAddr());
        memSidePort.schedTimingReq(pkt, clockEdge(forwardLatency));
        return;
    }
    //     DPRINTF(CacheAM,
    //         "%s L1CacheAM: access spm cache at %lx\n",
    //         __func__, pkt->getAddr());
    //     spmCache->getPort("cpu_side").recvTimingReq(pkt);
    // }
    // else {
    // DPRINTF(CacheAM,
    //     "%s L1CacheAM: access normal cache at %lx\n",
    //     __func__, pkt->getAddr());
    //     normalCache->getPort("cpu_side").recvTimingReq(pkt);
    // }
    return Cache::recvTimingReq(pkt);
}

void L1CacheAM::recvTimingResp(PacketPtr pkt)
{
    const Addr vaddr = pkt->getAddr();
    uintptr_t vaddr_prefix = vaddr >> 48;
    bool is_spm_addr = (vaddr_prefix == 0x1000);
    if (is_spm_addr) {
        handleUncacheableWriteResp(pkt);
        return ;
    }

    Cache::recvTimingResp(pkt);
}

Tick L1CacheAM::recvAtomic(PacketPtr pkt)
{
    // if (spmRange.contains(pkt->getAddr()))
    // {
    //     DPRINTF(CacheAM,
    //         "%s L1CacheAM: access spm at %lx\n",
    //         __func__, pkt->getAddr());
    //     spmAccess(pkt);
    //     return getSpmLatency();
    // }
    // else
    const Addr vaddr = pkt->getAddr();
    uintptr_t vaddr_prefix = vaddr >> 48;
    bool is_spm_addr = (vaddr_prefix == 0x1000);
    if (is_spm_addr) {
        // TODO: just bypass to L2 Cache
        spmBypassedPackets.push_back(pkt);
        return memSidePort.sendAtomic(pkt);
    } else {
        DPRINTF(CacheAM,
            "%s L1CacheAM: access cache at %lx\n",
            __func__, pkt->getAddr());
        return Cache::recvAtomic(pkt);
    }
}

L1CacheAM *
L1CacheAMParams::create()
{
    assert(tags);
    assert(replacement_policy);

    return new L1CacheAM(this);
}
