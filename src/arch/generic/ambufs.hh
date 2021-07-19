/*
 * Copyright 2019 Google, Inc.
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

#ifndef __ARCH_GENERIC_AMBUFS_HH__
#define __ARCH_GENERIC_AMBUFS_HH__

#include "mem/packet.hh"
#include "params/BaseAMBufs.hh"
#include "sim/sim_object.hh"

class ThreadContext;
class BaseCPU;

enum AMBufsFetchFlags {
    AMBufsFetchFinishList   = 0x1,
    AMBufsFetchFreeList     = 0x2,
};

class BaseAMBufs : public SimObject
{
  protected:
    ThreadContext *tc = nullptr;

  public:
    typedef BaseAMBufsParams Params;

    BaseAMBufs(Params *p) : SimObject(p) {}

    virtual void setThreadContext(ThreadContext *_tc) { tc = _tc; }

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    virtual uint64_t getAddr(AMBufsFetchFlags fetchType) const = 0;
    virtual uint64_t getSize(AMBufsFetchFlags fetchType) const = 0;

    virtual int needFetchData() const = 0;
    virtual void setFetchDataFlags(AMBufsFetchFlags flags) = 0;
    virtual void setFetchDataMask(AMBufsFetchFlags flags) = 0;
    virtual void clearFetchDataFlags(AMBufsFetchFlags flags) = 0;
    virtual void clearFetchDataMask(AMBufsFetchFlags flags) = 0;
    virtual void dataFetched(PacketPtr respPkt,
                             AMBufsFetchFlags targetBuf) = 0;
};

#endif // __ARCH_GENERIC_AMBUFS_HH__
