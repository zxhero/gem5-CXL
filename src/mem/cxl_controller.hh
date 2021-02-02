#ifndef __CXL_CONTROLLER_HH__
#define __CXL_CONTROLLER_HH__

#include "mem/xbar.hh"
#include "params/CXLController.hh"

///#include "base/addr_range.hh"
//#include <vector>

class CXLController : public BaseXBar
{
public:

    CXLController(const CXLControllerParams *p);
    virtual ~CXLController();

protected:
    /**
     * Declaration of the non-coherent crossbar CPU-side port type, one
     * will be instantiated for each of the memory-side ports connecting to
     * the crossbar.
     */
    class CXLControllerResponsePort : public QueuedResponsePort
    {
      private:

        /** A reference to the crossbar to which this port belongs. */
        CXLController &xbar;

        /** A normal packet queue used to store responses. */
        RespPacketQueue queue;

      public:

        CXLControllerResponsePort(const std::string &_name,
                                CXLController &_xbar, PortID _id)
            :QueuedResponsePort(_name, &_xbar, queue, _id),
            xbar(_xbar), queue(_xbar, *this)
        { }

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
    class CXLControllerRequestPort : public QueuedRequestPort
    {
      private:

        /** A reference to the crossbar to which this port belongs. */
        CXLController &xbar;

        /** Packet queue used to store outgoing snoop responses. */
        //no use in this case
        SnoopRespPacketQueue snoopRespQueue;

        /** A normal packet queue used to store responses. */
        ReqPacketQueue queue;

      public:

        CXLControllerRequestPort(const std::string &_name,
                                 CXLController &_xbar, PortID _id)
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

    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
    virtual bool recvTimingResp(PacketPtr pkt, PortID mem_side_port_id);
    Tick recvAtomicBackdoor(PacketPtr pkt, PortID cpu_side_port_id,
                            MemBackdoorPtr *backdoor=nullptr);
    void recvFunctional(PacketPtr pkt, PortID cpu_side_port_id);

    class QueuedReqLayer : public ReqLayer{
      public:
      QueuedReqLayer(CXLControllerRequestPort& _port, BaseXBar& _xbar,
        const std::string& _name) :
            ReqLayer(_port, _xbar, _name), cxl_port(_port), pkt_outstanding(0)
        {}
      bool TestOutstanding(ResponsePort* src_port);
      bool CreditRelease(std::vector<int>::iterator CrePtr, int count);
      private:
      CXLController::CXLControllerRequestPort& cxl_port;
      std::deque<ResponsePort*> waitingForCredit;
      unsigned int pkt_outstanding;
    };
    /**
     * Declare the layers of this crossbar, one vector for requests
     * and one for responses.
     */
    std::vector<QueuedReqLayer*> reqLayers;
    std::vector<RespLayer*> respLayers;
    //std::vector<QueuedRequestPort*> memSidePorts;

private:
    //std::vector<PacketPtr> waiting;
    std::vector<int> ResCrd;
    std::vector<int> ReqCrd;
    std::vector<int> DataCrd;
    std::vector<AddrRange *> addr;
    unsigned last_rollover;
    void mkReadPkt(PacketPtr pkt, PortID port_id);
    void mkWritePkt(PacketPtr pkt, PortID port_id);
};

#endif //__CXL_CONTROLLER_HH__
