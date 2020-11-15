#ifndef __CXL_DEVICE_HH__
#define __CXL_DEVICE_HH__

#include "mem/noncoherent_xbar.hh"
#include "mem/port.hh"
#include "params/CXLDevice.hh"

class CXLDevice : public NoncoherentXBar{
    public:
        CXLDevice(const CXLDeviceParams *p);
private:
    // Receive a request and distribute it among response ports
    //  Simply forwards the packet to the next serial link based on a
    //  Round-robin counter
    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
};

#endif