/*
 * Copyright (c) 2011 Google
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

#ifndef __ARCH_RISCV_AMBUFS_HH__
#define __ARCH_RISCV_AMBUFS_HH__

#include <array>
#include <bitset>
#include <memory>

#include "arch/generic/ambufs.hh"
#include "arch/riscv/faults.hh"
#include "arch/riscv/registers.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "debug/AMBuf.hh"
#include "mem/packet.hh"
#include "params/RiscvAMBufs.hh"
#include "sim/sim_object.hh"

class BaseCPU;
class ThreadContext;

namespace RiscvISA {

class AMBufs : public BaseAMBufs
{
  private:
    int fetchFlags;
    int fetchMask;
    std::array<uint16_t, FL_REG_LENGTH> amBufs[NumBufReg];
    int amBufsCnt[NumBufReg];

  public:
    typedef RiscvAMBufsParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    AMBufs(Params * p) : BaseAMBufs(p),
        fetchFlags(0), fetchMask(0) {}

    uint64_t getAddr(AMBufsFetchFlags fetchType) const
    {
        return 0x1000000000000000llu + MISCREG_GETFIN - MISCREG_QBASE;
    }
    uint64_t getSize(AMBufsFetchFlags fetchType) const
    {
        return sizeof(uint16_t) * FL_REG_LENGTH;
    }

    int needFetchData() const
    {
        return fetchFlags & (!fetchMask);
    }
    void setFetchDataFlags(AMBufsFetchFlags flags)
    {
        fetchFlags |= flags;
    }
    void setFetchDataMask(AMBufsFetchFlags flags)
    {
        fetchMask |= flags;
    }
    void clearFetchDataFlags(AMBufsFetchFlags flags)
    {
        fetchFlags &= ~flags;
    }
    void clearFetchDataMask(AMBufsFetchFlags flags)
    {
        fetchMask &= ~flags;
    }
    void dataFetched(PacketPtr respPkt, AMBufsFetchFlags targetBuf)
    {
        assert(tc != nullptr);

        int pos = __builtin_ffs(targetBuf);
        respPkt->writeData((uint8_t*)amBufs[pos].data());
        amBufsCnt[pos] = respPkt->getSize() / sizeof(uint16_t);

        delete respPkt;
    }
    uint16_t popOne(BufRegIndex idx) {
        int ret = 0;
        if (amBufsCnt[idx] > 0) {
            int _pos = --amBufsCnt[idx];
            ret = amBufs[idx][_pos];
        } else {
            // FIXME:
            setFetchDataFlags(AMBufsFetchFinishList);
        }
        DPRINTF(AMBuf, "popOne: %d(remains:%d)\n",
                ret, amBufsCnt[idx]);
        return ret;
    }

    void
    serialize(CheckpointOut &cp) const
    {
        // TODO:
        // unsigned long ip_ulong = ip.to_ulong();
        // unsigned long ie_ulong = ie.to_ulong();
        // SERIALIZE_SCALAR(ip_ulong);
        // SERIALIZE_SCALAR(ie_ulong);
    }

    void
    unserialize(CheckpointIn &cp)
    {
        // TODO:
        // unsigned long ip_ulong;
        // unsigned long ie_ulong;
        // UNSERIALIZE_SCALAR(ip_ulong);
        // ip = ip_ulong;
        // UNSERIALIZE_SCALAR(ie_ulong);
        // ie = ie_ulong;
    }
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_AMBUFS_HH__
