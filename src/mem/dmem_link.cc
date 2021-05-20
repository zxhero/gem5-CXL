#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "mem/dmem_link.hh"
#include "debug/DMemLinkRequester.hh"
#include "debug/DMemLinkResponder.hh"
#include "debug/DMemLinkRouter.hh"
#include "debug/AddrRanges.hh"

DMemLinkRequester::DMemLinkRequester(const DMemLinkRequesterParams *p)
    : NoncoherentXBar(p), RxEvent([this]{ processRxEvent(); }, std::string("dmemReq_rx")),
    TxEvent([this]{ processTxEvent(); }, std::string("dmemReq_tx"))
{
    pktQueueRx.resize(p->port_mem_side_ports_connection_count);
    nid = p->NID;
    schedule(&RxEvent, clockEdge(Cycles(10)));
    pktQueueTx.resize(p->port_mem_side_ports_connection_count);
    schedule(&TxEvent, clockEdge(Cycles(10)));
    RxWaitRetey.resize(p->port_mem_side_ports_connection_count);
    for (auto i = RxWaitRetey.begin(); i != RxWaitRetey.end(); i++)
    {
        *i = false;
    }
    
}

DMemLinkRequester::~DMemLinkRequester()
{
    //NoncoherentXBar::~NoncoherentXBar();
}

DMemLinkRequester*
DMemLinkRequesterParams::create()
{
    return new DMemLinkRequester(this);
}

int DMemLinkRequester::mallocTID(){
    for (size_t i = 0; i < MAXTID; i++)
    {
        if(tid[i] == false){
            tid[i] = true;
            return i;
        }
    }
    return -1;
}

void DMemLinkRequester::freeTID(int TID){
    tid[TID] = false;
}

void DMemLinkRequester::processInst(PortID mem_side_port_id, std::vector<struct reqMsg> &insts, MemCmd::Command _cmd){
    if(insts.empty()){
        //DPRINTF(DMemLinkRequester, "processInst: EMPTY pakt queue.\n");
        return ;
    }

    RequestPtr req = std::make_shared<Request>();
    req->setPaddr(0x1);
    PacketPtr dmem_pkt = new Packet(req, MemCmd(_cmd));
    dmem_pkt->TID = mallocTID();
    int LID = 0;
    int size = 0;

    for (auto pktptr = insts.begin(); pktptr != insts.end(); pktptr ++)
    {
        PacketPtr pkt = pktptr->pkt;

        //Combine pkts
        pkt->LID = LID++;
        pkt->SNID = nid;
        pkt->TID = dmem_pkt->TID;
        dmem_pkt->instructions.emplace_back(pkt);
        size += pkt->getSize();
    }
    dmem_pkt->setSize(size);
    // since it is a normal request, attempt to send the packet
    bool success = memSidePorts[mem_side_port_id]->sendTimingReq(dmem_pkt);

    if (!success)  {
        /*for (auto pktptr = insts.begin(); pktptr != insts.end(); pktptr ++){
            PacketPtr pkt = pktptr->pkt;
             PortID cpu_side_port_id = pktptr->cpu_side_port_id;
            ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];
            DPRINTF(DMemLinkRequester, "processInst: src %s %s 0x%x RETRY\n",
                src_port->name(), pkt->cmdString(), pkt->getAddr());

            // restore the header delay as it is additive
            pkt->headerDelay = pkt->old_header_delay;
        }
        packetFinishTime = clockPeriod();//clockEdge(Cycles(1));*/
        DPRINTF(DMemLinkRequester, "processInst: send fail\n");
        RxWaitRetey[mem_side_port_id] = true;
        delete dmem_pkt;
        for (auto pktptr = insts.begin(); pktptr != insts.end(); pktptr ++){
            pktQueueRx[mem_side_port_id].emplace_back(*pktptr);
        }
    }else{
        for (auto pktptr = insts.begin(); pktptr != insts.end(); pktptr ++){
            PacketPtr pkt = pktptr->pkt;
            PortID cpu_side_port_id = pktptr->cpu_side_port_id;
            // remember if we are expecting a response
            const bool expect_response = pkt->needsResponse() &&
                !pkt->cacheResponding();
            // remember where to route the response to
            if (expect_response) {
                assert(routeTo.find(pkt->req) == routeTo.end());
                routeTo[pkt->req] = cpu_side_port_id;
            }
        }
        DPRINTF(DMemLinkRequester, "processInst: send success \n");
        
    }
    insts.clear();
    return ;
}

void DMemLinkRequester::processRxEvent(){
    //DPRINTF(DMemLinkRequester, "processRxEvent: Scanning cpu side ports\n");
    schedule(&RxEvent, clockEdge(Cycles(10)));
    for (auto i = pktQueueRx.begin(); i != pktQueueRx.end(); i++)
    {
        /* code */
        PortID mem_side_port_id = (i - pktQueueRx.begin());
        if(RxWaitRetey[mem_side_port_id])
            continue;
        
        for (auto pktptr = i->begin(); pktptr != i->end(); pktptr = i->erase(pktptr))
        {
            PacketPtr pkt = pktptr->pkt;
            if(pkt->isRead()){
                LDreq.emplace_back(*pktptr);
            }else if(pkt->isWrite()){
                STreq.emplace_back(*pktptr);
            }else{
                assert(0);
            }
            
            PortID cpu_side_port_id = pktptr->cpu_side_port_id;
            unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
            unsigned int pkt_cmd = pkt->cmdToIndex();

            // stats updates
            pktCount[cpu_side_port_id][mem_side_port_id]++;
            pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
            transDist[pkt_cmd]++;
        }

        processInst(mem_side_port_id, LDreq, MemCmd::Command::MemRd);
        if(!RxWaitRetey[mem_side_port_id])
            processInst(mem_side_port_id, STreq, MemCmd::Command::MemWr);
    }
}

bool
DMemLinkRequester::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id)
{
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    // we should never see express snoops on a non-coherent crossbar
    assert(!pkt->isExpressSnoop());

    // determine the destination based on the address
    PortID mem_side_port_id = findPort(pkt->getAddrRange());

    // test if the layer should be considered occupied for the current
    // port
    if (!reqLayers[mem_side_port_id]->tryTiming(src_port)) {
        DPRINTF(DMemLinkRequester, "recvTimingReq: mem side port %s BUSY\n",
                memSidePorts[mem_side_port_id]->name());
        return false;
    }

    DPRINTF(DMemLinkRequester, "recvTimingReq: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());
    
    struct reqMsg tmp;// = new struct reqMsg;
    tmp.cpu_side_port_id = cpu_side_port_id;
    tmp.pkt = pkt;
    pktQueueRx[mem_side_port_id].emplace_back(tmp);
    
    // store the old header delay so we can restore it if needed
    pkt->old_header_delay = pkt->headerDelay;

    // a request sees the frontend and forward latency
    Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = (pkt->payloadDelay + clockEdge(Cycles(1)));

    DPRINTF(DMemLinkRequester, "recvTimingReq: mem side port %s freed after %d\n",
                        mem_side_port_id, packetFinishTime);
    reqLayers[mem_side_port_id]->succeededTiming(packetFinishTime);

    return true;
}

void DMemLinkRequester::processTxEvent(){
    schedule(&TxEvent, clockEdge(Cycles(10)));
    for (auto i = pktQueueTx.begin(); i != pktQueueTx.end(); i++){
        PortID mem_side_port_id = i - pktQueueTx.begin();

        for(auto ptr = i->begin(); ptr != i->end(); ){
            PacketPtr pkt = ptr->pkt;
            // determine the destination
            const auto route_lookup = routeTo.find(pkt->req);
            assert(route_lookup != routeTo.end());
            const PortID cpu_side_port_id = route_lookup->second;
            assert(cpu_side_port_id != InvalidPortID);
            assert(cpu_side_port_id < respLayers.size());

            // test if the layer should be considered occupied for the current
            // port
            if (!respLayers[cpu_side_port_id]->tryTiming(NULL)) {
                DPRINTF(DMemLinkRequester, "processTxEvent: cpu side port %s BUSY\n",
                        cpuSidePorts[cpu_side_port_id]->name());
                ptr++;
                continue;
            }

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

            // store size and command as they might be modified when
            // forwarding the packet
            unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
            unsigned int pkt_cmd = pkt->cmdToIndex();

            // stats updates
            pktCount[cpu_side_port_id][mem_side_port_id]++;
            pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
            transDist[pkt_cmd]++;

            ptr = i->erase(ptr);
        }
    }
}

bool
DMemLinkRequester::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id)
{
    // determine the source port based on the id
    RequestPort *src_port = memSidePorts[mem_side_port_id];
    DPRINTF(DMemLinkRequester, "recvTimingResp: src %s %s\n",
                    src_port->name(), pkt->cmdString());

    for (auto pktptr = pkt->instructions.begin(); pktptr != pkt->instructions.end(); pktptr ++){
        struct reqMsg tmp;// = new struct reqMsg;
        tmp.cpu_side_port_id = InvalidPortID;
        tmp.pkt = *pktptr;
        pktQueueTx[mem_side_port_id].emplace_back(tmp);
    }
    
    return true;
}

void DMemLinkRequester::recvReqRetry(PortID mem_side_port_id){
    RxWaitRetey[mem_side_port_id] = false;
    return ;
}

/** Function called by the port when the crossbar is receiving a range change.*/
void
DMemLinkRequester::recvRangeChange(PortID mem_side_port_id)
{
    DPRINTF(AddrRanges, "Received range change from cpu_side_ports %s\n",
            memSidePorts[mem_side_port_id]->getPeer());

    // remember that we got a range from this memory-side port and thus the
    // connected CPU-side-port module
    gotAddrRanges[mem_side_port_id] = true;

    // update the global flag
    if (!gotAllAddrRanges) {
        // take a logical AND of all the ports and see if we got
        // ranges from everyone
        gotAllAddrRanges = true;
        std::vector<bool>::const_iterator r = gotAddrRanges.begin();
        while (gotAllAddrRanges &&  r != gotAddrRanges.end()) {
            gotAllAddrRanges &= *r++;
        }
        if (gotAllAddrRanges)
            DPRINTF(AddrRanges, "Got address ranges from all responders\n");
    }

    // note that we could get the range from the default port at any
    // point in time, and we cannot assume that the default range is
    // set before the other ones are, so we do additional checks once
    // all ranges are provided
    if (mem_side_port_id == defaultPortID) {
        // only update if we are indeed checking ranges for the
        // default port since the port might not have a valid range
        // otherwise
        if (useDefaultRange) {
            AddrRangeList ranges = memSidePorts[mem_side_port_id]->
                                   getAddrRanges();

            if (ranges.size() != 1)
                fatal("Crossbar %s may only have a single default range",
                      name());

            defaultRange = ranges.front();
        }
    } else {
        // the ports are allowed to update their address ranges
        // dynamically, so remove any existing entries
        if (gotAddrRanges[mem_side_port_id]) {
            for (auto p = portMap.begin(); p != portMap.end(); ) {
                if (p->second == mem_side_port_id)
                    // erasing invalidates the iterator, so advance it
                    // before the deletion takes place
                    portMap.erase(p++);
                else
                    p++;
            }
        }

        AddrRangeList ranges = memSidePorts[mem_side_port_id]->
                               getAddrRanges();

        for (const auto& r: ranges) {
            DPRINTF(AddrRanges, "Adding range %s for id %d\n",
                    r.to_string(), mem_side_port_id);
            if (portMap.insert(r, mem_side_port_id) == portMap.end()) {
                PortID conflict_id = portMap.intersects(r)->second;
                fatal("%s has two ports responding within range "
                      "%s:\n\t%s\n\t%s\n",
                      name(),
                      r.to_string(),
                      memSidePorts[mem_side_port_id]->getPeer(),
                      memSidePorts[conflict_id]->getPeer());
            }
        }
    }

    // if we have received ranges from all our neighbouring CPU-side-port
    // modules, go ahead and tell our connected memory-side-port modules in
    // turn, this effectively assumes a tree structure of the system
    if (gotAllAddrRanges) {
        DPRINTF(AddrRanges, "Aggregating address ranges\n");
        xbarRanges.clear();

        // start out with the default range
        if (useDefaultRange) {
            if (!gotAddrRanges[defaultPortID])
                fatal("Crossbar %s uses default range, but none provided",
                      name());

            xbarRanges.push_back(defaultRange);
            DPRINTF(AddrRanges, "-- Adding default %s\n",
                    defaultRange.to_string());
        }

        // merge all interleaved ranges and add any range that is not
        // a subset of the default range
        std::vector<AddrRange> intlv_ranges;
        for (const auto& r: portMap) {
            // keep the current range if not a subset of the default
            if (!(useDefaultRange &&
                  r.first.isSubset(defaultRange))) {
                xbarRanges.push_back(r.first);
                DPRINTF(AddrRanges, "-- Adding range %s\n",
                        r.first.to_string());
            }
        }

        // if there is still interleaved ranges waiting to be merged,
        // go ahead and do it
        if (!intlv_ranges.empty()) {
            DPRINTF(AddrRanges, "-- Merging range from %d ranges\n",
                    intlv_ranges.size());
            AddrRange merged_range(intlv_ranges);
            if (!(useDefaultRange && merged_range.isSubset(defaultRange))) {
                xbarRanges.push_back(merged_range);
                DPRINTF(AddrRanges, "-- Adding merged range %s\n",
                        merged_range.to_string());
            }
        }

        // also check that no range partially intersects with the
        // default range, this has to be done after all ranges are set
        // as there are no guarantees for when the default range is
        // update with respect to the other ones
        if (useDefaultRange) {
            for (const auto& r: xbarRanges) {
                // see if the new range is partially
                // overlapping the default range
                if (r.intersects(defaultRange) &&
                    !r.isSubset(defaultRange))
                    fatal("Range %s intersects the "                    \
                          "default range of %s but is not a "           \
                          "subset\n", r.to_string(), name());
            }
        }

        // tell all our neighbouring memory-side ports that our address
        // ranges have changed
        for (const auto& port: cpuSidePorts)
            port->sendRangeChange();
    }
}

DMemLinkResponder::DMemLinkResponder(const DMemLinkResponderParams *p)
    : NoncoherentXBar(p), RxEvent([this]{ processRxEvent(); }, std::string("dmemResp_rx")),
    TxEvent([this]{ processTxEvent(); }, std::string("dmemResp_tx"))
{
    pktQueueRx.resize(p->port_cpu_side_ports_connection_count);
    schedule(&RxEvent, clockEdge(Cycles(10)));

    pktQueueTx.resize(p->port_cpu_side_ports_connection_count);
    schedule(&TxEvent, clockEdge(Cycles(10)));
}

DMemLinkResponder::~DMemLinkResponder()
{
    //NoncoherentXBar::~NoncoherentXBar();
}

DMemLinkResponder*
DMemLinkResponderParams::create()
{
    return new DMemLinkResponder(this);
}

void DMemLinkResponder::processRxEvent(){
    schedule(&RxEvent, clockEdge(Cycles(10)));

    for (auto i = pktQueueRx.begin(); i != pktQueueRx.end(); i++){
        if(i->empty())
            continue;

        PortID cpu_side_port_id = i - pktQueueRx.begin();
        for (auto pktptr = i->begin(); pktptr != i->end(); ){
            // determine the destination based on the address
            PortID mem_side_port_id = findPort((pktptr->pkt)->getAddrRange());

            // test if the layer should be considered occupied for the current
            // port
            if (!reqLayers[mem_side_port_id]->tryTiming(NULL)) {
                DPRINTF(DMemLinkResponder, "processRxEvent: dest %d BUSY\n",
                        mem_side_port_id);
                pktptr ++;
                continue;
            }

            PacketPtr pkt = pktptr->pkt;
            
            // store the old header delay so we can restore it if needed
            Tick old_header_delay = pkt->headerDelay;

            // a request sees the frontend and forward latency
            Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

            // set the packet header and payload delay
            calcPacketTiming(pkt, xbar_delay);

            // determine how long to be crossbar layer is busy
            Tick packetFinishTime = (pkt->payloadDelay + clockEdge(Cycles(1)));

            // before forwarding the packet (and possibly altering it),
            // remember if we are expecting a response
            const bool expect_response = pkt->needsResponse() &&
            !pkt->cacheResponding();

            // since it is a normal request, attempt to send the packet
            bool success = memSidePorts[mem_side_port_id]->sendTimingReq(pkt);
            
            if (!success)  {
                DPRINTF(DMemLinkResponder, "processExeEvent: dest %s %s 0x%x RETRY\n",
                memSidePorts[mem_side_port_id]->name(), pkt->cmdString(), pkt->getAddr());

                // restore the header delay as it is additive
                pkt->headerDelay = old_header_delay;

                // occupy until the header is sent
                packetFinishTime = clockEdge(Cycles(1));

                pktptr ++;
            }else{
                pktptr = i->erase(pktptr);

                // remember where to route the response to
                if (expect_response) {
                    assert(routeTo.find(pkt->req) == routeTo.end());
                    routeTo[pkt->req] = cpu_side_port_id;
                }

                // store size and command as they might be modified when
                // forwarding the packet
                unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
                unsigned int pkt_cmd = pkt->cmdToIndex();

                // stats updates
                pktCount[cpu_side_port_id][mem_side_port_id]++;
                pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
                transDist[pkt_cmd]++;
            }

            reqLayers[mem_side_port_id]->succeededTiming(packetFinishTime);
        }
    }
}

bool
DMemLinkResponder::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id)
{
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    DPRINTF(DMemLinkResponder, "recvTimingReq: src %s %s\n",
            src_port->name(), pkt->cmdString());

    for (auto pktptr = pkt->instructions.begin(); pktptr != pkt->instructions.end(); pktptr ++){

        struct reqMsg tmp;// = new struct reqMsg;
        tmp.cpu_side_port_id = cpu_side_port_id;
        tmp.pkt = *pktptr;
        pktQueueRx[cpu_side_port_id].emplace_back(tmp);
    }
    delete pkt;
    return true;
}

void DMemLinkResponder::processInst(PortID cpu_side_port_id, std::vector<struct reqMsg> &insts, MemCmd::Command _cmd){
    RequestPtr req = std::make_shared<Request>();
    req->setPaddr(0x2);
    PacketPtr dmem_pkt = new Packet(req, MemCmd(_cmd));
    int size = 0;

    for (auto pktptr = insts.begin(); pktptr != insts.end(); pktptr ++){
        PacketPtr pkt = pktptr->pkt;
        pkt->headerDelay = 0;
        size += pkt->getSize();
        dmem_pkt->instructions.emplace_back(pkt);
    }
    dmem_pkt->setSize(size);
    
    // send the packet through the destination CPU-side port, and pay for
    // any outstanding latency.
    Tick latency = clockEdge() - curTick();
    cpuSidePorts[cpu_side_port_id]->schedTimingResp(dmem_pkt,
                                        curTick() + latency);
}

void DMemLinkResponder::processTxEvent(){
    schedule(&TxEvent, clockEdge(Cycles(10)));

    for (auto i = pktQueueTx.begin(); i != pktQueueTx.end(); i++)
    {
        PortID cpu_side_port_id = i - pktQueueTx.begin();
        //Tick packetFinishTime = clockEdge();
        
        while(i->empty() == false){
            std::vector<struct reqMsg> insts;
            PacketPtr pkt = i->begin()->pkt;
            //insts.emplace_back(*(i->begin()));

            for(auto ptr = i->begin(); ptr != i->end(); ){
                if(ptr->pkt->SNID == pkt->SNID
                    && ptr->pkt->TID == pkt->TID){
                        insts.emplace_back(*ptr);
                        ptr = i->erase(ptr);
                    }else{
                        ptr++;
                    }
            }

            if(pkt->isWrite()){
                processInst(cpu_side_port_id, insts, MemCmd::Cmp);
                //dmem_pkt = new Packet(req, MemCmd(MemCmd::Cmp));
            }else{
                processInst(cpu_side_port_id, insts, MemCmd::MemData);
                //dmem_pkt = new Packet(req, MemCmd(MemCmd::MemData));
            }
        }
    }
}

bool
DMemLinkResponder::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id)
{
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
        DPRINTF(DMemLinkResponder, "recvTimingResp: cpu side port %s BUSY\n",
                cpuSidePorts[cpu_side_port_id]->name());
        return false;
    }

     // a response sees the response latency
    Tick xbar_delay = responseLatency * clockPeriod();

    // set the packet header and payload delay
    calcPacketTiming(pkt, xbar_delay);

    // determine how long to be crossbar layer is busy
    Tick packetFinishTime = clockEdge(Cycles(1)) + pkt->payloadDelay;

    DPRINTF(DMemLinkResponder, "recvTimingResp: src %s %s 0x%x\n",
            src_port->name(), pkt->cmdString(), pkt->getAddr());

    struct reqMsg tmp;// = new struct reqMsg;
    //tmp.port_id = cpu_side_port_id;
    tmp.pkt = pkt;
    pktQueueTx[cpu_side_port_id].emplace_back(tmp);

    // remove the request from the routing table
    routeTo.erase(route_lookup);

    respLayers[cpu_side_port_id]->succeededTiming(packetFinishTime);

    // store size and command as they might be modified when
    // forwarding the packet
    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
    unsigned int pkt_cmd = pkt->cmdToIndex();
    // stats updates
    pktCount[cpu_side_port_id][mem_side_port_id]++;
    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
    transDist[pkt_cmd]++;

    return true;
}

void DMemLinkResponder::recvReqRetry(PortID mem_side_port_id){
    return ;
}

DMemLinkRouter::DMemLinkRouter(const DMemLinkRouterParams *p)
    : NoncoherentXBar(p), 
    downTransEvent([this]{ downTransLayer(); }, std::string("dmemRouterDown_trans")), 
    downTxEvent([this]{ processDownTxEvent(); }, std::string("dmemRouterDown_tx")),
    upTransEvent([this]{ upTransLayer(); }, std::string("dmemRouterUp_trans")), 
    upTxEvent([this]{ processUpTxEvent(); }, std::string("dmemRouterUp_tx"))
{
    pktQueueDownTx.resize(p->port_mem_side_ports_connection_count);
    schedule(&downTxEvent, clockEdge(Cycles(1)));
    schedule(&downTransEvent, clockEdge(Cycles(1)));
    pktQueueDownRx.resize(p->port_cpu_side_ports_connection_count);

    pktQueueUpTx.resize(p->port_cpu_side_ports_connection_count);
    //schedule(&upTxEvent, clockEdge(Cycles(1)));
    schedule(&upTransEvent, clockEdge(Cycles(1)));
    pktQueueUpRx.resize(p->port_mem_side_ports_connection_count);
}

DMemLinkRouter::~DMemLinkRouter()
{
    //NoncoherentXBar::~NoncoherentXBar();
}

DMemLinkRouter*
DMemLinkRouterParams::create()
{
    return new DMemLinkRouter(this);
}

void DMemLinkRouter::downTransLayer(){
    schedule(&downTransEvent, clockEdge(Cycles(1)));
    for (auto i = pktQueueDownRx.begin(); i != pktQueueDownRx.end(); i++){
        if(i->empty())
            continue;

        PortID cpu_side_port_id = i - pktQueueDownRx.begin();
        auto pktptr = i->begin();
            
        // determine the destination based on the address
        PortID mem_side_port_id = findPort((*pktptr)->getAddrRange());

        // test if the layer should be considered occupied for the current
        // port
        if (!reqLayers[mem_side_port_id]->tryTiming(NULL)) {
             DPRINTF(DMemLinkRouter, "downTransLayer: dest %d BUSY\n",
                    mem_side_port_id);
            continue;
        }

        //generate new msg request
        int TID = (*pktptr)->TID;
        int SNID = (*pktptr)->SNID;
        RequestPtr req = std::make_shared<Request>();
        req->setPaddr(0x1);

        PacketPtr dmem_pkt;
        if((*pktptr)->isRead()){
            dmem_pkt = new Packet(req, MemCmd(MemCmd::Command::MemRd));
        }else if((*pktptr)->isWrite()){
            dmem_pkt = new Packet(req, MemCmd(MemCmd::Command::MemWr));
        }
        dmem_pkt->instructions.emplace_back((*pktptr));
        Tick packetFinishTime = clockEdge(Cycles(1));
        int size = (*pktptr)->getSize();
        //combine the command from same transcation and with same DNID into new request
        for(pktptr = i->erase(pktptr);pktptr != i->end(); ){

            PacketPtr pkt = *pktptr;
            if(mem_side_port_id == findPort(pkt->getAddrRange()) 
             && pkt->TID == TID && pkt->SNID == SNID){
                // store the old header delay so we can restore it if needed
                Tick old_header_delay = pkt->headerDelay;

                // a request sees the frontend and forward latency
                Tick xbar_delay = (frontendLatency + forwardLatency) * clockPeriod();

                // set the packet header and payload delay
                calcPacketTiming(pkt, xbar_delay);

                // determine how long to be crossbar layer is busy
                packetFinishTime += (pkt->payloadDelay );

                // since it is a normal request, attempt to send the packet
                {
                    size += pkt->getSize();
                    dmem_pkt->instructions.emplace_back(pkt);
                    pktptr = i->erase(pktptr);

                    // store size and command as they might be modified when
                    // forwarding the packet
                    unsigned int pkt_size = pkt->hasData() ? pkt->getSize() : 0;
                    unsigned int pkt_cmd = pkt->cmdToIndex();

                    // stats updates
                    pktCount[cpu_side_port_id][mem_side_port_id]++;
                    pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
                    transDist[pkt_cmd]++;
                }
            }else{
                pktptr++;
            }
        }
        dmem_pkt->setSize(size);
        pktQueueDownTx[mem_side_port_id].emplace_back(dmem_pkt);
        reqLayers[mem_side_port_id]->succeededTiming(packetFinishTime);
    }
}

void DMemLinkRouter::processDownTxEvent(){
    schedule(&downTxEvent, clockEdge(Cycles(1)));
    for (auto i = pktQueueDownTx.begin(); i != pktQueueDownTx.end(); i++)
    {
        // determine the destination based on the address
        PortID mem_side_port_id = i - pktQueueDownTx.begin();

        // since it is a normal request, attempt to send the packet
        if(i->empty() == false){
            bool success = memSidePorts[mem_side_port_id]->sendTimingReq((i->front()));
            if(success){
                DPRINTF(DMemLinkRouter, "processDownTxEvent: send success \n");
                i->erase(i->begin());
            }
        }
    }
    
    return ;
}

bool DMemLinkRouter::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id){
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    

    //TODO: we only support DNID is router node
    //if(pkt->DNID == nid){
        for (auto pktptr = pkt->instructions.begin(); pktptr != pkt->instructions.end(); pktptr ++){
            DPRINTF(DMemLinkRouter, "recvTimingReq: src %s %s 0x%x\n",
                src_port->name(), (*pktptr)->cmdString(), (*pktptr)->getAddr());

            pktQueueDownRx[cpu_side_port_id].emplace_back(*pktptr);
        }
        delete pkt;
    //}else{
    //    pktQueueDownRx[cpu_side_port_id].emplace_back(pkt);
    //}
    
    
    return true;
}

void DMemLinkRouter::upTransLayer(){
    schedule(&upTransEvent, clockEdge(Cycles(1)));
    
    //Todo: we assume there is one cpu side port for now
    const PortID cpu_side_port_id = 0;
    

    for (auto i = pktQueueUpRx.begin(); i != pktQueueUpRx.end(); i++){
        if(i->empty())
            continue;

        // test if the layer should be considered occupied for the current
        // port
        if (!respLayers[cpu_side_port_id]->tryTiming(NULL)) {
            DPRINTF(DMemLinkRouter, "upTransLayer: dest 0 BUSY\n");
            return;
        }

        PortID mem_side_port_id = i - pktQueueUpRx.begin();
        auto pktptr = i->begin();
        // a response sees the response latency
        Tick xbar_delay = responseLatency * clockPeriod();

        // set the packet header and payload delay
        calcPacketTiming(*pktptr, xbar_delay);

        // determine how long to be crossbar layer is busy
        Tick packetFinishTime = clockEdge(Cycles(1)) + (*pktptr)->payloadDelay;

        // send the packet through the destination CPU-side port, and pay for
        // any outstanding latency
        Tick latency = (*pktptr)->headerDelay;
        (*pktptr)->headerDelay = 0;
        cpuSidePorts[cpu_side_port_id]->schedTimingResp(*pktptr,
                                    curTick() + latency);
        i->erase(i->begin());
        respLayers[cpu_side_port_id]->succeededTiming(packetFinishTime);

        // store size and command as they might be modified when
        // forwarding the packet
        unsigned int pkt_size = (*pktptr)->hasData() ? (*pktptr)->getSize() : 0;
        unsigned int pkt_cmd = (*pktptr)->cmdToIndex();

        // stats updates
        pktCount[cpu_side_port_id][mem_side_port_id]++;
        pktSize[cpu_side_port_id][mem_side_port_id] += pkt_size;
        transDist[pkt_cmd]++;

        return;
    }
}

void DMemLinkRouter::processUpTxEvent(){

}

bool DMemLinkRouter::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id){
    // determine the source port based on the id
    RequestPort *src_port = memSidePorts[mem_side_port_id];
    DPRINTF(DMemLinkRouter, "recvTimingResp: src %s %s\n",
                    src_port->name(), pkt->cmdString());

    pktQueueUpRx[mem_side_port_id].emplace_back(pkt);
    
    return true;
}

/** Function called by the port when the crossbar is receiving a range change.*/
void
DMemLinkRouter::recvRangeChange(PortID mem_side_port_id)
{
    DPRINTF(AddrRanges, "Received range change from cpu_side_ports %s\n",
            memSidePorts[mem_side_port_id]->getPeer());

    // remember that we got a range from this memory-side port and thus the
    // connected CPU-side-port module
    gotAddrRanges[mem_side_port_id] = true;

    // update the global flag
    if (!gotAllAddrRanges) {
        // take a logical AND of all the ports and see if we got
        // ranges from everyone
        gotAllAddrRanges = true;
        std::vector<bool>::const_iterator r = gotAddrRanges.begin();
        while (gotAllAddrRanges &&  r != gotAddrRanges.end()) {
            gotAllAddrRanges &= *r++;
        }
        if (gotAllAddrRanges)
            DPRINTF(AddrRanges, "Got address ranges from all responders\n");
    }

    // note that we could get the range from the default port at any
    // point in time, and we cannot assume that the default range is
    // set before the other ones are, so we do additional checks once
    // all ranges are provided
    if (mem_side_port_id == defaultPortID) {
        // only update if we are indeed checking ranges for the
        // default port since the port might not have a valid range
        // otherwise
        if (useDefaultRange) {
            AddrRangeList ranges = memSidePorts[mem_side_port_id]->
                                   getAddrRanges();

            if (ranges.size() != 1)
                fatal("Crossbar %s may only have a single default range",
                      name());

            defaultRange = ranges.front();
        }
    } else {
        // the ports are allowed to update their address ranges
        // dynamically, so remove any existing entries
        if (gotAddrRanges[mem_side_port_id]) {
            for (auto p = portMap.begin(); p != portMap.end(); ) {
                if (p->second == mem_side_port_id)
                    // erasing invalidates the iterator, so advance it
                    // before the deletion takes place
                    portMap.erase(p++);
                else
                    p++;
            }
        }

        AddrRangeList ranges = memSidePorts[mem_side_port_id]->
                               getAddrRanges();

        for (const auto& r: ranges) {
            DPRINTF(AddrRanges, "Adding range %s for id %d\n",
                    r.to_string(), mem_side_port_id);
            if (portMap.insert(r, mem_side_port_id) == portMap.end()) {
                PortID conflict_id = portMap.intersects(r)->second;
                fatal("%s has two ports responding within range "
                      "%s:\n\t%s\n\t%s\n",
                      name(),
                      r.to_string(),
                      memSidePorts[mem_side_port_id]->getPeer(),
                      memSidePorts[conflict_id]->getPeer());
            }
        }
    }

    // if we have received ranges from all our neighbouring CPU-side-port
    // modules, go ahead and tell our connected memory-side-port modules in
    // turn, this effectively assumes a tree structure of the system
    if (gotAllAddrRanges) {
        DPRINTF(AddrRanges, "Aggregating address ranges\n");
        xbarRanges.clear();

        // start out with the default range
        if (useDefaultRange) {
            if (!gotAddrRanges[defaultPortID])
                fatal("Crossbar %s uses default range, but none provided",
                      name());

            xbarRanges.push_back(defaultRange);
            DPRINTF(AddrRanges, "-- Adding default %s\n",
                    defaultRange.to_string());
        }

        // merge all interleaved ranges and add any range that is not
        // a subset of the default range
        std::vector<AddrRange> intlv_ranges;
        for (const auto& r: portMap) {
            // keep the current range if not a subset of the default
            if (!(useDefaultRange &&
                  r.first.isSubset(defaultRange))) {
                xbarRanges.push_back(r.first);
                DPRINTF(AddrRanges, "-- Adding range %s\n",
                        r.first.to_string());
            }
        }

        // if there is still interleaved ranges waiting to be merged,
        // go ahead and do it
        if (!intlv_ranges.empty()) {
            DPRINTF(AddrRanges, "-- Merging range from %d ranges\n",
                    intlv_ranges.size());
            AddrRange merged_range(intlv_ranges);
            if (!(useDefaultRange && merged_range.isSubset(defaultRange))) {
                xbarRanges.push_back(merged_range);
                DPRINTF(AddrRanges, "-- Adding merged range %s\n",
                        merged_range.to_string());
            }
        }

        // also check that no range partially intersects with the
        // default range, this has to be done after all ranges are set
        // as there are no guarantees for when the default range is
        // update with respect to the other ones
        if (useDefaultRange) {
            for (const auto& r: xbarRanges) {
                // see if the new range is partially
                // overlapping the default range
                if (r.intersects(defaultRange) &&
                    !r.isSubset(defaultRange))
                    fatal("Range %s intersects the "                    \
                          "default range of %s but is not a "           \
                          "subset\n", r.to_string(), name());
            }
        }

        // tell all our neighbouring memory-side ports that our address
        // ranges have changed
        for (const auto& port: cpuSidePorts)
            port->sendRangeChange();
    }
}