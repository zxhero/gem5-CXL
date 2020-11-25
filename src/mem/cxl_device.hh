#ifndef __CXL_DEVICE_HH__
#define __CXL_DEVICE_HH__

#include "base/types.hh"
#include "mem/port.hh"
#include "mem/xbar.hh"
#include "params/CXLDevice.hh"

/*
class CXLDevice : public NoncoherentXBar{
    public:
        CXLDevice(const CXLDeviceParams *p);
private:
    // Receive a request and distribute it among response ports
    //  Simply forwards the packet to the next serial link based on a
    //  Round-robin counter
    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
};*/

class CXLDevice : public BaseXBar
{
public:

    CXLDevice(const CXLDeviceParams *p);
    virtual ~CXLDevice();

protected:
    //std::vector<QueuedRequestPort*> memSidePorts;

    /**
     * Declaration of the non-coherent crossbar CPU-side port type, one
     * will be instantiated for each of the memory-side ports connecting to
     * the crossbar.
     */
    class CXLDeviceResponsePort : public QueuedResponsePort
    {
      private:

        /** A reference to the crossbar to which this port belongs. */
        CXLDevice &xbar;
      public:
        /** A normal packet queue used to store responses. */
        RespPacketQueue queue;

        CXLDeviceResponsePort(const std::string &_name,
                                CXLDevice &_xbar, PortID _id)
            :QueuedResponsePort(_name, &_xbar, queue, _id),
            xbar(_xbar), queue(_xbar, *this)
        { }

        bool combine_command(PacketPtr pkt);

      protected:

        bool
        recvTimingReq(PacketPtr pkt) override
        {
            return xbar.recvTimingReq(pkt, id);
        }

        Tick
        recvAtomic(PacketPtr pkt) override
        {
            return xbar.recvAtomicBackdoor(pkt, id);
        }

        Tick
        recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &backdoor) override
        {
            return xbar.recvAtomicBackdoor(pkt, id, &backdoor);
        }

        void
        recvFunctional(PacketPtr pkt) override
        {
            xbar.recvFunctional(pkt, id);
        }

        AddrRangeList
        getAddrRanges() const override
        {
            return xbar.getAddrRanges();
        }
    };

    /**
     * Declaration of the crossbar memory-side port type, one will be
     * instantiated for each of the CPU-side ports connecting to the
     * crossbar.
     */
    class CXLDeviceRequestPort : public QueuedRequestPort
    {
      private:

        /** A reference to the crossbar to which this port belongs. */
        CXLDevice &xbar;

        /** Packet queue used to store outgoing snoop responses. */
        //no use in this case
        SnoopRespPacketQueue snoopRespQueue;

        /** A normal packet queue used to store responses. */
        ReqPacketQueue queue;

      public:

        CXLDeviceRequestPort(const std::string &_name,
                                 CXLDevice &_xbar, PortID _id)
            : QueuedRequestPort(_name, &_xbar, queue, snoopRespQueue, _id),
            xbar(_xbar), snoopRespQueue(_xbar, *this), queue(_xbar, *this)

        { }

      protected:

        bool
        recvTimingResp(PacketPtr pkt) override
        {
            return xbar.recvTimingResp(pkt, id);
        }

        void
        recvRangeChange() override
        {
            xbar.recvRangeChange(id);
        }
    public:
        int size(){
          return queue.size();
        }
    };

    /* class CXLRespLayer : public RespLayer{
      public:
      CXLRespLayer(CXLDeviceResponsePort& _port, BaseXBar& _xbar,
        const std::string& _name) :
            RespLayer(_port, _xbar, _name), cxl_port(_port)
        {}
      bool combine_command(PacketPtr pkt);
      private:
      CXLDeviceResponsePort& cxl_port;
    };*/
    /**
     * Declare the layers of this crossbar, one vector for requests
     * and one for responses.
     */
    std::vector<ReqLayer*> reqLayers;
    std::vector<RespLayer*> respLayers;

    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
    virtual bool recvTimingResp(PacketPtr pkt, PortID mem_side_port_id);
    Tick recvAtomicBackdoor(PacketPtr pkt, PortID cpu_side_port_id,
                            MemBackdoorPtr *backdoor=nullptr);
    void recvFunctional(PacketPtr pkt, PortID cpu_side_port_id);

private:
    //std::vector<PacketPtr> waiting;
    std::vector<int> ResCrd;
    std::vector<int> ReqCrd;
    std::vector<int> DataCrd;
    unsigned last_rollover;
    std::vector<AddrRange *> addr;

    void generate_cxl_rep(PacketPtr pkt);
    //stats for commands that are combined into 1 packet.
    Stats::Scalar combined_pkt;
public:
    void regStats() override;
};

#endif
