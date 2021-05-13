#include "base/logging.hh"
#include "base/random.hh"
#include "base/trace.hh"
#include "mem/dmem_link.hh"
#include "debug/DMemLinkRequester.hh"
#include "debug/DMemLinkResponder.hh"
#include "debug/DMemLinkRouter.hh"

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
    upTransEvent([this]{ downTransLayer(); }, std::string("dmemRouterUp_trans")), 
    upTxEvent([this]{ processDownTxEvent(); }, std::string("dmemRouterUp_tx"))
{
    pktQueueDownTx.resize(p->port_mem_side_ports_connection_count);
    schedule(&downTxEvent, clockEdge(Cycles(1)));
    schedule(&downTransEvent, clockEdge(Cycles(1)));
    pktQueueDownRx.resize(p->port_cpu_side_ports_connection_count);

    pktQueueUpTx.resize(p->port_cpu_side_ports_connection_count);
    schedule(&upTxEvent, clockEdge(Cycles(1)));
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
             DPRINTF(DMemLinkRouter, "processRxEvent: dest %d BUSY\n",
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
        for(pktptr ++ ;pktptr != i->end(); ){

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
                DPRINTF(DMemLinkRouter, "processInst: send success \n");
                i->erase(i->begin());
            }
        }
    }
    
    return ;
}

bool DMemLinkRouter::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id){
    // determine the source port based on the id
    ResponsePort *src_port = cpuSidePorts[cpu_side_port_id];

    DPRINTF(DMemLinkRouter, "recvTimingReq: src %s %s\n",
            src_port->name(), pkt->cmdString());

    if(pkt->DNID == nid){
        for (auto pktptr = pkt->instructions.begin(); pktptr != pkt->instructions.end(); pktptr ++){

            pktQueueDownRx[cpu_side_port_id].emplace_back(*pktptr);
        }
        delete pkt;
    }else{
        pktQueueDownRx[cpu_side_port_id].emplace_back(pkt);
    }
    
    
    return true;
}

void DMemLinkRouter::upTransLayer(){

}

void DMemLinkRouter::processUpTxEvent(){

}

bool DMemLinkRouter::recvTimingResp(PacketPtr pkt, PortID mem_side_port_id){
    //TODO
    return true;
}