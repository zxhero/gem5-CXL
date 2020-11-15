#include "mem/cxl_device.hh"

#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/CXLDevice.hh"

CXLDevice::CXLDevice(const CXLDeviceParams* p) :
    NoncoherentXBar(p)
{
    DPRINTF(CXLDevice, "hello world from cxl device!\n");
}

CXLDevice*
CXLDeviceParams::create()
{
    return new CXLDevice(this);
}

bool CXLDevice::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id){
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    DPRINTF(CXLDevice, "recvTimingReq: src %s %s 0x%x %d\n",
            src_port->name(), pkt->cmdString(),
            pkt->getAddr(), pkt->getSize());

    return NoncoherentXBar::recvTimingReq(pkt, cpu_side_port_id);
}
