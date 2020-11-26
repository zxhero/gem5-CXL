#include "mem/cxl_device.hh"

#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "debug/CXLDevice.hh"
#include "mem/cxl_protocol.hh"

CXLDevice::CXLDevice(const CXLDeviceParams* p) :
    BaseXBar(p),
    combined_pkt(this, "combined_pkt",
        "the number of CXL command been combined")
{
    // create the ports based on the size of the memory-side port and
    // CPU-side port vector ports, and the presence of the default port,
    // the ports are enumerated starting from zero
    for (int i = 0; i < p->port_mem_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.mem_side_port[%d]", name(), i);
        QueuedRequestPort* bp = new CXLDeviceRequestPort(portName,
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
        QueuedRequestPort* bp = new CXLDeviceRequestPort(portName, *this,
                                                      defaultPortID);
        memSidePorts.push_back(bp);
        reqLayers.push_back(new ReqLayer(*bp, *this, csprintf("reqLayer%d",
                                                              defaultPortID)));
    }

    // create the CPU-side ports, once again starting at zero
    for (int i = 0; i < p->port_cpu_side_ports_connection_count; ++i) {
        std::string portName = csprintf("%s.cpu_side_ports[%d]", name(), i);
        QueuedResponsePort* bp = new CXLDeviceResponsePort(portName,
                                                                *this, i);
        cpuSidePorts.push_back(bp);
        respLayers.push_back(new RespLayer(*bp, *this,
                                           csprintf("respLayer%d", i)));
    }

    DPRINTF(CXLDevice, "hello world from cxl device!\n");
    for (int i = 0; i < 2; i++)
    {
        ReqCrd.push_back(64);
        ResCrd.push_back(64);
        DataCrd.push_back(64);
        //addr.push_back(new AddrRange(
            //0x20000000+i*0x20000000, 0x20000000+(i+1)*0x20000000));
    }

}

CXLDevice::~CXLDevice()
{
    for (auto l: reqLayers)
        delete l;
    for (auto l: respLayers)
        delete l;
}

CXLDevice*
CXLDeviceParams::create()
{
    return new CXLDevice(this);
}

bool CXLDevice::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id){
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    // we should never see express snoops on a non-coherent crossbar
    assert(!pkt->isExpressSnoop());

    // determine the destination based on the address
    PortID mem_side_port_id = findPort(pkt->getCXLAddrRange());

    // test if the layer should be considered occupied for the current
    // port
    if (!reqLayers[mem_side_port_id]->tryTiming(src_port)) {
        DPRINTF(CXLDevice, "recvTimingReq: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CXLDevice, "recvTimingReq: src %s %s 0x%x %d\n",
            src_port->name(), pkt->cmdString(),
            pkt->getAddr(), pkt->getSize());

    //we can decide if the sender needs to retry in one cycle after
    //recieve data valid signal from the sender.
    if (
    ((CXLDeviceRequestPort*)memSidePorts[mem_side_port_id])->size() == 64)
    {
        DPRINTF(CXLDevice, "recvTimingReq: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

        // occupy until the header is sent
        reqLayers[mem_side_port_id]->failedTiming(src_port,
                                                clockEdge(Cycles(1)));

        return false;
    }

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();

    // store the old header delay so we can restore it if needed
    Tick old_header_delay = pkt->headerDelay;

    //If MemWr command has 4 data roll over,
    //we will modify packet size.
    //data flit's size is 65 bytes.
    if (pkt_cmd == MemCmd::Command::MemWr){
        if (pkt->rollover == 4){
            // stats updates
            pktCount[cpu_side_port_id][mem_side_port_id]++;
            transDist[MemCmd::Command::DataFlit]++;
        }
    }

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
    //packet is put on queue after controller recieve packet header.
    //Decode CXL packet, restore packet size.
    //No need to restore command
    pkt->update_size(pkt->cxl_size);
    pkt->cxl_size = pkt_size;
    Tick latency = pkt->headerDelay;
    pkt->headerDelay = 0;
    ((CXLDeviceRequestPort*)memSidePorts[mem_side_port_id])
                            ->schedTimingReq(pkt, curTick() + latency);
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

bool CXLDevice::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id){
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
        DPRINTF(CXLDevice, "recvTimingResp: src %s %s 0x%x BUSY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());
        return false;
    }

    DPRINTF(CXLDevice, "recvTimingResp: src %s %s 0x%x %d\n",
            src_port->name(), pkt->cmdString(),
            pkt->getAddr(), pkt->getSize());

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
    //Check if we can combine packets into 1 CXL requests.
    /*if (((CXLDeviceResponsePort*)cpuSidePorts[cpu_side_port_id])
        ->combine_command(pkt)){
        DPRINTF(CXLDevice, "recvTimingResp: src %s %s 0x%x %d
            is combined with another command.\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr(),
            pkt->getSize());
        combined_pkt++;
    }*/
    //we do not send conbined Cmp cpmmands.
    //if (!pkt->is_combined || pkt->cmd == MemCmd::Command::MemData){
    generate_cxl_rep(pkt);
    if (last_rollover == 4) {
        //send Data Flit
        last_rollover = 0;
        pkt->cxl_size += DATA_FLIT;
    }
    //update packet size
    //we store CXL packet size in @size,
    //old size in @cxl_size. In this way, we do not need to update
    //other Classes.
    pkt->update_size(pkt->cxl_size);
    pkt->cxl_size = pkt_size;
    cpuSidePorts[cpu_side_port_id]->schedTimingResp(pkt,
                                        curTick() + latency);
    //}

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    respLayers[cpu_side_port_id]->succeededTiming(packetFinishTime);

    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
};

Tick CXLDevice::recvAtomicBackdoor(PacketPtr pkt, PortID cpu_side_port_id,
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

void CXLDevice::recvFunctional(PacketPtr pkt, PortID cpu_side_port_id){
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

bool CXLDevice::CXLDeviceResponsePort::combine_command(PacketPtr pkt){
    //return false;
    if (pkt->cmd == MemCmd::Command::Cmp){
        queue.resetQueuePtr();
        PacketPtr tmp = queue.workAroundQueue();
        while (tmp != NULL){
            if (tmp->reserved_for_more_NDR > 0){
                tmp->reserved_for_more_NDR--;
                pkt->is_combined = true;
                return true;
            }
            tmp = queue.workAroundQueue();
        };
    } else if (pkt->cmd == MemCmd::Command::MemData){
        //we can only combine MemData command with the latest packet.
        PacketPtr tmp = queue.LastPkt();
        if (tmp->reserved_for_more_DRS > 0){
            if (tmp->cmd == MemCmd::Command::Cmp){
                //remove cmp commands
                queue.pop_back();

            }
            pkt->reserved_for_more_DRS = tmp->reserved_for_more_DRS - 1;
            pkt->reserved_for_more_NDR = tmp->reserved_for_more_NDR;
            pkt->is_combined = true;
            return true;
        }
    }
    pkt->is_combined = false;
    return false;
};

void CXLDevice::generate_cxl_rep(PacketPtr pkt){
    /*if (pkt->is_combined){
        //we only generate a data Flit
        pkt->rollover = 0;
        pkt->reserved_for_more_NDR = 0;
        pkt->cxl_size = DATA_FLIT;
        return ;
    }*/
    pkt->reserved_for_more_DRS = 3;
    pkt->reserved_for_more_NDR = 2;
    //Consider rollover if this is RWD
    if (pkt->cmd == MemCmd::Command::MemData){
        pkt->reserved_for_more_DRS--;
        pkt->rollover = (last_rollover + 4) - 3;
        last_rollover = pkt->rollover;
    } else {
        pkt->reserved_for_more_NDR--;
        pkt->rollover = 0;
        last_rollover = 0;
    }
    pkt->cxl_size = FLIT_SIZE;
};

CXLRespPacketQueue::CXLRespPacketQueue(
                                CXLDevice& _em,
                                 ResponsePort& _cpu_side_port,
                                 bool force_order,
                                 const std::string _label)
    : RespPacketQueue(_em, _cpu_side_port, force_order, _label),
    cxl_device(_em)
{
}

void CXLRespPacketQueue::sendDeferredPacket(){
    // sanity checks
    assert(!waitingOnRetry);
    assert(deferredPacketReady());

    DeferredPacket dp = transmitList.front();

    // take the packet of the list before sending it, as sending of
    // the packet in some cases causes a new packet to be enqueued
    // (most notaly when responding to the timing CPU, leading to a
    // new request hitting in the L1 icache, leading to a new
    // response)
    transmitList.pop_front();

    PacketPtr tmp2 = dp.pkt;
    for (auto ptr = transmitList.begin(); ptr != transmitList.end(); ptr++)
    {
        //we may combine any possible packet.
        if (ptr->tick <= curTick()){
            cxl_device.combined_pkt++;
            /*PacketPtr tmp = ptr->pkt;

            if (tmp->cmd == MemCmd::Command::MemData){

            }else{

            }*/
        }
    }


    // use the appropriate implementation of sendTiming based on the
    // type of queue
    waitingOnRetry = !sendTiming(dp.pkt);

    // if we succeeded and are not waiting for a retry, schedule the
    // next send
    if (!waitingOnRetry) {
        schedSendEvent(deferredPacketReadyTime());
    } else {
        // put the packet back at the front of the list
        transmitList.emplace_front(dp);
    }
}

void CXLDevice::regStats() {
    BaseXBar::regStats();
    using namespace Stats;
    combined_pkt
        //.reset()
        .flags(total | nozero | nonan);
};
