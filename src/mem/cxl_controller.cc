#include "mem/cxl_controller.hh"

#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/CXLController.hh"
#include "mem/cxl_protocol.hh"

CXLController::CXLController(const CXLControllerParams* p) :
    BaseXBar(p)
{
    // create the ports based on the size of the memory-side port and
    // CPU-side port vector ports, and the presence of the default port,
    // the ports are enumerated starting from zero
    for (int i = 0; i < p->port_mem_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.mem_side_port[%d]", name(), i);
        QueuedRequestPort* bp = new CXLControllerRequestPort(portName,
                                *this, i);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this,
                                         csprintf("reqLayer%d", i)));

    }

    // see if we have a default CPU-side-port device connected and if so add
    // our corresponding memory-side port
    if (p->port_default_connection_count) {
        defaultPortID = memSidePorts.size();
        std::string portName = name() + ".default";
        QueuedRequestPort* bp = new CXLControllerRequestPort(portName, *this,
                                                      defaultPortID);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this, csprintf("reqLayer%d",
                                                              defaultPortID)));
    }

    // create the CPU-side ports, once again starting at zero
    for (int i = 0; i < p->port_cpu_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.cpu_side_ports[%d]", name(), i);
        QueuedResponsePort* bp = new CXLControllerResponsePort(portName,
                                                                *this, i);
        cpuSidePorts.push_back(bp);
        respLayers.push_back(new RespLayer(*bp, *this,
                                           csprintf("respLayer%d", i)));
    }

    DPRINTF(CXLController, "hello world from cxl controller!\n");
    for (int i = 0; i < 2; i++)
    {
        ReqCrd.push_back(64);
        ResCrd.push_back(64);
        DataCrd.push_back(64);
        //addr.push_back(new AddrRange(
        //    0x20000000+i*0x20000000, 0x20000000+(i+1)*0x20000000));
    }

    //push back local credit
    ReqCrd.push_back(64);
    ResCrd.push_back(64);
    DataCrd.push_back(64);
}

CXLController::~CXLController()
{
    for (auto l: reqLayers)
        delete l;
    for (auto l: respLayers)
        delete l;
}

CXLController*
CXLControllerParams::create()
{
    return new CXLController(this);
}

bool CXLController::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id){
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    // we should never see express snoops on a non-coherent crossbar
    assert(!pkt->isExpressSnoop());

    // determine the destination based on the address
    PortID mem_side_port_id = findPort(pkt->getAddrRange());

    // test if the layer should be considered occupied for the current
    // port
    //waiting.push_back(pkt);
    if (!reqLayers[mem_side_port_id]->tryTiming(src_port)) {
        DPRINTF(CXLController, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    //pkt = waiting.front();
    //waiting.erase(waiting.begin());
    DPRINTF(CXLController, "recvTimingReq: src %s %s 0x%x %d\n",
            src_port->name(), pkt->cmdString(),
            pkt->getAddr(), pkt->req->getSize());

    //we can decide if the sender needs to retry in one cycle after
    //recieve data valid signal from the sender.
    if (ResCrd[2] == 0 ||
    ((CXLControllerRequestPort*)memSidePorts[mem_side_port_id])->size() == 64)
    {
        DPRINTF(CXLController, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        // occupy until the header is sent
        reqLayers[mem_side_port_id]->failedTiming(src_port,
                                                clockEdge(Cycles(1)));

        return false;
    }

    // store the old header delay so we can restore it if needed
    Tick old_header_delay = pkt->headerDelay;

    if (pkt->isRead())
        mkReadPkt(pkt, mem_side_port_id);
    else if (pkt->isWriteback())
        mkWritePkt(pkt, mem_side_port_id);

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->cxl_size;
    unsigned int pkt_cmd = (int)(pkt->cxl_comm);

    // a request sees the frontend and forward latency
    Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long the controller recieves whole packet
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // before forwarding the packet (and possibly altering it),
    // remember if we are expecting a response
    const bool expect_response = pkt->needsResponse() &&
        !pkt->cacheResponding();

    // send the packet through the destination mem-side port, and pay for
    // any outstanding latency
    //packet is put on queue after controller recieving packet header
    Tick latency = pkt->headerDelay;
    pkt->headerDelay = 0;
    ((CXLControllerRequestPort*)memSidePorts[mem_side_port_id])
                            ->schedTimingReq(pkt, curTick() + latency);

    //Send data flit if rollover equals to 4
    //after write request sent
    //Actually, we only update stats
    if (last_rollover == 4){
        last_rollover = 0;
        //PacketPtr data_flit = new Packet(pkt, false, true);
        //data_flit->cxl_comm = MemCmd::Command::DataFlit;
        //((CXLControllerRequestPort*)memSidePorts[mem_side_port_id])
        //->schedTimingReq(pkt, curTick() + latency +  nextCycle());
        // stats updates
        pktCount[cpu_side_port_id][mem_side_port_id]++;
        pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
        transDist[(int)(MemCmd::Command::DataFlit)]++;
    }

    // remember where to route the response to
    if (expect_response) {
        assert(routeTo.find(pkt->req) == routeTo.end());
        routeTo[pkt->req] = cpu_side_port_id;
    }

    reqLayers[mem_side_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

bool CXLController::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id){
    // determine the source port based on the id
    RequestPort *src_port = memSidePorts[mem_side_port_id];

    // determine the destination
    const auto route_lookup = routeTo.find(pkt->req);
    assert(route_lookup != routeTo.end());
    const PortID cpu_side_port_id = route_lookup->second;
    assert(cpu_side_port_id != InvalidPortID);
    assert(cpu_side_port_id < respLayers.size());

    // test if the layer should be considered occupied for the current
    // port
    if (!respLayers[cpu_side_port_id]->tryTiming(src_port)) {
        DPRINTF(CXLController, "recvTimingResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CXLController, "recvTimingResp: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // a response sees the response latency
    Tick xbar_delay = responseLatency * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    // send the packet through the destination CPU-side port, and pay for
    // any outstanding latency
    Tick latency = pkt->headerDelay;
    pkt->headerDelay = 0;
    cpuSidePorts[cpu_side_port_id]->schedTimingResp(pkt,
                                        curTick() + latency);

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    respLayers[cpu_side_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    //credit update
    ResCrd[2] ++;

    return true;
};

Tick CXLController::recvAtomicBackdoor(PacketPtr pkt, PortID cpu_side_port_id,
                            MemBackdoorPtr *backdoor){
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // determine the destination port
    PortID mem_side_port_id = findPort(pkt->getAddrRange());

    // stats updates for the request
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    // forward the request to the appropriate destination
    auto mem_side_port = memSidePorts[mem_side_port_id];
    Tick response_latency = backdoor ?
        mem_side_port->sendAtomicBackdoor(pkt, *backdoor) :
        mem_side_port->sendAtomic(pkt);

    // add the response data
    if (pkt->isResponse()) {
        pkt_size = pkt->hasData() ? pkt->getSize() : 0;
        pkt_cmd = pkt->cmdToIndex();

        // stats updates
        pktCount[cpu_side_port_id][mem_side_port_id]++;
        pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
        transDist[pkt_cmd]++;
    }

    // @todo: Not setting first-word time
    pkt->payloadDelay = response_latency;
    return response_latency;
};

void CXLController::recvFunctional(PacketPtr pkt, PortID cpu_side_port_id){
    // since our CPU-side ports are queued ports we need to check them as well
    for (const auto& p : cpuSidePorts) {
        // if we find a response that has the data, then the
        // downstream caches/memories may be out of date, so simply stop
        // here
        if (p->trySatisfyFunctional(pkt)) {
            if (pkt->needsResponse())
                pkt->makeResponse();
            return;
        }
    }

    // determine the destination port
    PortID dest_id = findPort(pkt->getAddrRange());

    // forward the request to the appropriate destination
    memSidePorts[dest_id]->sendFunctional(pkt);
};

void CXLController::mkReadPkt(PacketPtr pkt, PortID port_id){
    //we can recieve no more than 64 response.
    ResCrd[2] -- ;

    pkt->cxl_comm = MemCmd::Command::MemRd;
    pkt->cmd = MemCmd::Command::MemRd;
    //Currently, we think the device has infinite queue
    pkt->ReqCrd = 64;
    pkt->ResCrd = 64;
    pkt->DataCrd = 64;
    pkt->rollover = 0;
    pkt->cxl_size = FLIT_SIZE;
    //no rollover from the read instruction
    last_rollover = 0;
};

void CXLController::mkWritePkt(PacketPtr pkt, PortID port_id){
    //generate MemWrPtl or MemWr instruction
    //currently, we do not see any write partial
    if (pkt->isWriteback()){
        //we can recieve no more than 64 response.
        //currently, write needs no response.
        //ResCrd[2] -- ;

        pkt->cxl_comm = MemCmd::Command::MemWr;
        //Currently, we think the device has infinite queue
        pkt->ReqCrd = 64;
        pkt->ResCrd = 64;
        pkt->DataCrd = 64;
        pkt->cxl_size = FLIT_SIZE;
        //check rollover for the write instruction
        pkt->rollover = (last_rollover + 4) - 3;
        last_rollover = pkt->rollover;
    }else{

    }
};

