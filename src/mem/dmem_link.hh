#ifndef __DMEM_LINK_HH__
#define __DMEM_LINK_HH__

#include "mem/noncoherent_xbar.hh"
#include "params/DMemLinkRequester.hh"
#include "base/types.hh"
#include "params/DMemLinkResponder.hh"
#include "sim/eventq.hh"
#include "params/DMemLinkRouter.hh"

///#include "base/addr_range.hh"
//#include <vector>
struct reqMsg{
    PortID cpu_side_port_id;
    PacketPtr pkt;
};

class DMemLinkRequester : public NoncoherentXBar
{
public:

    DMemLinkRequester(const DMemLinkRequesterParams *p);
    virtual ~DMemLinkRequester();

protected:

    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
    virtual bool recvTimingResp(PacketPtr pkt, PortID mem_side_port_id);
    virtual void recvReqRetry(PortID mem_side_port_id) override;

private:
    //recieve from cpu side, send to mem side
    std::vector<bool> RxWaitRetey;
    void processRxEvent();
    void processInst(PortID mem_side_port_id, std::vector<struct reqMsg> &insts, MemCmd::Command _cmd);
    //Tick processST(PortID mem_side_port_id );
    int mallocTID();
    void freeTID(int TID);

    #define MAXTID 256
    EventFunctionWrapper RxEvent;
    std::vector<std::vector<struct reqMsg>> pktQueueRx;
    std::vector<struct reqMsg> LDreq;
    std::vector<struct reqMsg> STreq;
    int nid;
    bool tid[MAXTID];

    //send to cpu side, recieve from mem side
    void processTxEvent();
    EventFunctionWrapper TxEvent;
    std::vector<std::vector<struct reqMsg>> pktQueueTx;
};

class DMemLinkResponder : public NoncoherentXBar
{
public:

    DMemLinkResponder(const DMemLinkResponderParams *p);
    virtual ~DMemLinkResponder();
protected:
    
    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
    virtual bool recvTimingResp(PacketPtr pkt, PortID mem_side_port_id);
    void recvReqRetry(PortID mem_side_port_id);

private:
    int nid;
    //recieve from cpu side, send to mem side
    void processRxEvent();
    EventFunctionWrapper RxEvent;
    std::vector<std::vector<struct reqMsg>> pktQueueRx;

    //send to cpu side, recieve from mem side
    void processInst(PortID cpu_side_port_id, std::vector<struct reqMsg> &insts, MemCmd::Command _cmd);
    void processTxEvent();
    EventFunctionWrapper TxEvent;
    std::vector<std::vector<struct reqMsg>> pktQueueTx;
};

class DMemLinkRouter : public NoncoherentXBar
{
public:

    DMemLinkRouter(const DMemLinkRouterParams *p);
    virtual ~DMemLinkRouter();
protected:

    virtual bool recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id);
    virtual bool recvTimingResp(PacketPtr pkt, PortID mem_side_port_id);

private:
    int nid;
    //downstream
    void downTransLayer();
    EventFunctionWrapper downTransEvent;
    void processDownTxEvent();
    EventFunctionWrapper downTxEvent;
    std::vector<std::vector<PacketPtr>> pktQueueDownTx;
    std::vector<std::vector<PacketPtr>> pktQueueDownRx;
    

    //upstream
    void upTransLayer();
    EventFunctionWrapper upTransEvent;
    void processUpTxEvent();
    EventFunctionWrapper upTxEvent;
    std::vector<std::vector<PacketPtr>> pktQueueUpTx;
    std::vector<std::vector<PacketPtr>> pktQueueUpRx;
};
#endif //__DMEM_LINK_HH__
