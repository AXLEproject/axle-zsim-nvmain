/** $lic$
 *
 * Copyright (C) 2014 by Adria Armejach <adria.armejach@bsc.es>
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include <map>
#include <string>
#include <math.h>
#include "event_recorder.h"
#include "tick_event.h"
#include "timing_event.h"
#include "zsim.h"
#include "nvmain_mem_ctrl.h"
#include "ooo_core.h"

#ifdef _WITH_NVMAIN_ //was compiled with nvmain
#include "SimInterface/NullInterface/NullInterface.h"
#include "Utils/HookFactory.h"

//using namespace NVM; // NOLINT(build/namespaces)

class NVMainAccEvent : public TimingEvent {
    private:
        NVMainMemory* nvram;
        bool write;
        Address addr;

    public:
        uint64_t sCycle;

        NVMainAccEvent(NVMainMemory* _nvram, bool _write, Address _addr, int32_t domain) :  TimingEvent(0, 0, domain), nvram(_nvram), write(_write), addr(_addr) {}

        bool isWrite() const {
            return write;
        }

        Address getAddr() const {
            return addr;
        }

        void simulate(uint64_t startCycle) {
            sCycle = startCycle;
            nvram->enqueue(this, startCycle);
        }
};

/* Globally allocated event for scheduling
 *
 * NOTE: Reusing the same interface used in DDRMemory.
 */
class SchedEventNVMain : public TimingEvent, public GlobAlloc {
    private:
        NVMainMemory* const mem;
        enum State { IDLE, QUEUED, RUNNING, ANNULLED };
        State state;

    public:
        SchedEventNVMain* next;  // for event freelist

        SchedEventNVMain(NVMainMemory* _mem, int32_t domain) : TimingEvent(0, 0, domain), mem(_mem) {
            setMinStartCycle(0);
            setRunning();
            hold();
            state = IDLE;
            next = NULL;
        }

        void parentDone(uint64_t startCycle) {
            panic("This is queued directly");
        }

        void simulate(uint64_t startCycle) {
            if (state == QUEUED) {
                state = RUNNING;
                uint64_t nextCycle = mem->tick(startCycle);
                if (nextCycle) {
                    requeue(nextCycle);
                    state = QUEUED;
                } else {
                    state = IDLE;
                    hold();
                    mem->recycleEvent(this);
                }
            } else {
                assert(state == ANNULLED);
                state = IDLE;
                hold();
                mem->recycleEvent(this);
            }
        }

        void enqueue(uint64_t cycle) {
            assert(state == IDLE);
            state = QUEUED;
            requeue(cycle);
        }

        void annul() {
            assert_msg(state == QUEUED, "sched state %d", state);
            state = ANNULLED;
        }

        // Use glob mem
        using GlobAlloc::operator new;
        using GlobAlloc::operator delete;
};


NVMainMemory::NVMainMemory(std::string& nvmainTechIni, std::string& outputFile, std::string& traceName, uint32_t capacityMB, uint64_t _minLatency, uint32_t _domain, const g_string& _name) {

    nvmainConfig = new NVM::Config();
    nvmainConfig->Read(nvmainTechIni);
    info("NVMainControl: Reading NVMain config file: %s", nvmainTechIni.c_str());

    nvmainPtr = new NVM::NVMain();
    nvmainStatsPtr = new NVM::Stats();
    nvmainSimInterface = new NVM::NullInterface();
    nvmainEventQueue = new NVM::EventQueue();
    nvmainGlobalEventQueue = new NVM::GlobalEventQueue();
    nvmainTagGenerator = new NVM::TagGenerator(1000);

    nvmainConfig->SetSimInterface(nvmainSimInterface);

    SetEventQueue(nvmainEventQueue);
    SetStats(nvmainStatsPtr);
    SetTagGenerator(nvmainTagGenerator);
    nvmainGlobalEventQueue->SetFrequency(nvmainConfig->GetEnergy("CPUFreq") * 1000000.0);
    SetGlobalEventQueue(nvmainGlobalEventQueue);

    /*  Add any specified hooks */
    std::vector<std::string>& hookList = nvmainConfig->GetHooks();

    for(size_t i = 0; i < hookList.size(); i++) {

        NVMObject *hook = NVM::HookFactory::CreateHook(hookList[i]);

        if( hook != NULL ) {
            AddHook( hook );
            hook->SetParent( this );
            hook->Init( nvmainConfig );
        } else {
            warn("Could not create a hook");
        }
    }

    //Setup child and parent modules
    AddChild(nvmainPtr);
    nvmainPtr->SetParent(this);
    nvmainGlobalEventQueue->AddSystem(nvmainPtr, nvmainConfig);
    nvmainPtr->SetConfig(nvmainConfig);

    curCycle = 0;
    updateCycle = 0;
    double cpuFreq = static_cast<double>(nvmainConfig->GetEnergy("CPUFreq"));
    double busFreq = static_cast<double>(nvmainConfig->GetEnergy("CLK"));
    eventDriven = static_cast<bool>(nvmainConfig->GetBool("EventDriven"));
    info("NVMain: with %f cpuFreq, %f busFreq", cpuFreq, busFreq);
    minLatency = _minLatency;
    domain = _domain;

    // No longer necessary, now we do not tick every cycle, we use SchedEvent
    //TickEvent<NVMainMemory>* tickEv = new TickEvent<NVMainMemory>(this, domain);
    //tickEv->queue(0);  // start the sim at time 0

    name = _name;

    // Data
    if( nvmainConfig->KeyExists( "IgnoreData" ) && nvmainConfig->GetString( "IgnoreData" ) == "true" ) {
        ignoreData = true;
    } else {
        ignoreData = false;
    }

    // NVMain stats output file
    std::string path = zinfo->outputDir;
    path += "/";
    nvmainStatsFile = gm_strdup((path + name.c_str() + "-" + outputFile).c_str());
    std::ofstream out(nvmainStatsFile, std::ios_base::out);
    out << "# nvmain stats for " << name << std::endl;
    out << "===" << std::endl;

    // Wave phase handling
    nextSchedRequest = NULL;
    nextSchedEvent = NULL;
    eventFreelist = NULL;
}

void NVMainMemory::initStats(AggregateStat* parentStat) {
    AggregateStat* memStats = new AggregateStat();
    memStats->init(name.c_str(), "Memory controller stats");
    profIssued.init("issued", "Issued requests"); memStats->append(&profIssued);
    profReads.init("rd", "Read requests"); memStats->append(&profReads);
    profWrites.init("wr", "Write requests"); memStats->append(&profWrites);
    profPUTS.init("PUTS", "Clean Evictions (from lower level)"); memStats->append(&profPUTS);
    profPUTX.init("PUTX", "Dirty Evictions (from lower level)"); memStats->append(&profPUTX);
    profTotalRdLat.init("rdlat", "Total latency experienced by read requests"); memStats->append(&profTotalRdLat);
    profTotalWrLat.init("wrlat", "Total latency experienced by write requests"); memStats->append(&profTotalWrLat);
    profMemoryFootprint.init("footprint", "Total memory footprint in bytes"); memStats->append(&profMemoryFootprint);
    profMemoryAddresses.init("addresses", "Total number of distinct memory addresses"); memStats->append(&profMemoryAddresses);
    latencyHist.init("mlh", "latency histogram for memory requests", NUMBINS); memStats->append(&latencyHist);
    addressReuseHist.init("addressReuse", "address reuse histogram for memory requests", NUMBINS); memStats->append(&addressReuseHist);
    parentStat->append(memStats);
}

uint64_t NVMainMemory::access(MemReq& req) {
    switch (req.type) {
        case PUTS:
            profPUTS.inc();
            *req.state = I;
            break;
        case PUTX:
            profPUTX.inc();
            *req.state = I;
            break;
        case GETS:
            *req.state = req.is(MemReq::NOEXCL)? S : E;
            break;
        case GETX:
            *req.state = M;
            break;

        default: panic("!?");
    }

    uint64_t respCycle = req.cycle + minLatency;
    assert(respCycle > req.cycle);

    if ((zinfo->hasDRAMCache || (req.type != PUTS) /*discard clean writebacks going to mainMemory*/) && zinfo->eventRecorders[req.srcId]) {
        Address addr = req.lineAddr << lineBits; // Removes procMask
        addr = addr | procMask; // Set the procMask back
        bool isWrite = ((req.type == PUTX) || (req.type == PUTS));
        NVMainAccEvent* memEv = new (zinfo->eventRecorders[req.srcId]) NVMainAccEvent(this, isWrite, addr, domain);
        memEv->setMinStartCycle(req.cycle);
        TimingRecord tr = {addr, req.cycle, respCycle, req.type, memEv, memEv};
        zinfo->eventRecorders[req.srcId]->pushRecord(tr);
#if 0
        // Print memory trace to be fed to NVMain for debugging -- as seen by the wave phase.
        std::string path = zinfo->outputDir;
        path += "/";
        const char * memtraceFile = gm_strdup((path + "memory.trace").c_str());
        std::ofstream memtrace(memtraceFile, std::ios_base::app);
        char zeros[65];
        std::fill(zeros, zeros + 64, '0');
        zeros[64]='\0';
        std::string data(zeros);
        const char* type = (isWrite)?"W":"R";
        memtrace << req.cycle << " " << type << " 0x" << std::hex << addr << std::dec << " " << data << std::endl;
#endif
    }

    return respCycle;
}

uint64_t NVMainMemory::tick(uint64_t cycle) {

    // Advance NVMain to current cycle
    //info("[%s] [Tick] Update NVMain %lu cycles", getName(), (cycle+1) - updateCycle);
    nvmainGlobalEventQueue->Cycle((cycle+1) - updateCycle);
    updateCycle = cycle + 1;
    curCycle = updateCycle;

    // Check if nextSchedEvent has been serviced by RequestComplete
    // TODO: Maybe I need to save the Address in a nextSchedAddress varaible to know this
    // If the event has not been serviced do 'return 1', which reschedules for next cycle.
    // Else check multiset/multimap for next event if any.
    assert(nextSchedEvent);
    if (nextSchedRequest) { // Not serviced yet, cycle by cycle until it's served
        return cycle + 1;
    } else { // has been serviced, check for inflight requests
        if (inflightRequests.empty()) {
            nextSchedEvent = NULL;
            nextSchedRequest = NULL;
            return 0; //this will recycle the SchedEvent
        } else {
            nextSchedRequest = inflightRequests.front().first;
            if (cycle >= inflightRequests.front().second.second) {
                return cycle + 1; // we are past min finish cycle for next request
            } else {
                return inflightRequests.front().second.second; //get min finish cycle for next request
            }
        }
    }
}

void NVMainMemory::recycleEvent(SchedEventNVMain* ev) {
    assert(ev != nextSchedEvent);
    assert(ev->next == NULL);
    ev->next = eventFreelist;
    eventFreelist = ev;
}

// Use this for debugging
/*
template< class T >
std::ostream & operator << ( std::ostream & os, const std::multiset< T > & v ) {
    for ( const auto & i : v ) {
        os << i << std::endl;
    }
    return os;
}

template< class K, class V >
std::ostream & operator << ( std::ostream & os, const std::map< K, V > & m ) {
    for ( const auto & i : m ) {
        os << i.first << " : " << i.second << std::endl;
    }
    return os;
}

template< class T >
std::ostream & operator << ( std::ostream & os, const std::vector< T > & m ) {
    for ( const auto & i : m ) {
        os << std::hex << i.first->address.GetPhysicalAddress() << std::dec << " : " << i.second.first << ", " << i.second.second << std::endl;
    }
    return os;
}
*/

void NVMainMemory::enqueue(NVMainAccEvent* ev, uint64_t cycle) {

    profIssued.inc();

    // Build NVMainRequest and send it to NVMain
    NVM::NVMainRequest *request = new NVM::NVMainRequest();

    // No data in memory for now
    if (!ignoreData) {
        int transfer_size = zinfo->lineSize;
        request->data.SetSize(transfer_size);
        for(int i = 0; i < transfer_size; i++) {
            request->data.SetByte(i, 0);
        }
    }

    request->access = NVM::UNKNOWN_ACCESS;
    request->address.SetPhysicalAddress(ev->getAddr());
    request->status = NVM::MEM_REQUEST_INCOMPLETE;
    request->type = (ev->isWrite()) ? NVM::WRITE : NVM::READ;
    request->owner = (NVMObject *)this;

    // Sync NVMain state to curCycle
    // NVMain can only issue command in the current cycle
    curCycle = cycle + 1;
    //info("[%s] [Enqueue] Update NVMain %lu cycles", getName(), curCycle - updateCycle);
    nvmainGlobalEventQueue->Cycle(curCycle - updateCycle);
    updateCycle = curCycle;

    // If command cannot be issued due to contention, retry next cycle.
    if (!nvmainPtr->IsIssuable(request, NULL)) {
        //info("[%s] %s access to %lx requeued. Curent cycle %ld, requeue cycle %ld", getName(), ev->isWrite()? "Write" : "Read", ev->getAddr(), cycle, cycle+1);
        ev->requeue(cycle+1);
        delete request;
        return;
    }

    //info("[%s] [Enqueue] Address %lx curCycle %lu, cycle %lu, updateCycle %lu, inflight requests %ld", getName(), ev->getAddr(), curCycle, cycle, updateCycle, inflightRequests.size());
    bool enqueued = nvmainPtr->IssueCommand(request);
    assert(enqueued);

    // Update stats
    const auto it = memoryHistogram.find(ev->getAddr());
    if (it == memoryHistogram.end()){
        memoryHistogram.insert(std::make_pair<uint64_t, uint64_t>(ev->getAddr(), 1));
        profMemoryAddresses.inc(1);
        profMemoryFootprint.inc(zinfo->lineSize);
        addressReuseHist.inc(1);
    } else {
        addressReuseHist.dec(std::min(NUMBINS-1, it->second));
        it->second++;
        addressReuseHist.inc(std::min(NUMBINS-1, it->second));
    }

    // Add this request to list of inflight requests
    inflightRequests.push_back(std::make_pair(request, std::make_pair(ev, cycle + minLatency)));
    ev->hold();

    // Event handling
    if (!nextSchedEvent) {
        assert(inflightRequests.size() == 1);
        if (eventFreelist) {
            nextSchedEvent = eventFreelist;
            eventFreelist = eventFreelist->next;
            nextSchedEvent->next = NULL;
        } else {
            nextSchedEvent = new SchedEventNVMain(this, domain);
        }
        nextSchedEvent->enqueue(cycle + minLatency);
        nextSchedRequest = request;
    }

    return;
}

bool NVMainMemory::RequestComplete(NVM::NVMainRequest *creq) {

    assert(inflightRequests.size() > 0);
    auto it = inflightRequests.begin();
    for (; it != inflightRequests.end(); ++it) {
        if (it->first == creq) break;
    }
    assert(it != inflightRequests.end());
    NVMainAccEvent* ev = it->second.first;

    // Note that curCycle is up to date because we are advancing cycle by cycle in tick while
    // we are waiting for a request completion.
    uint64_t lat = curCycle+1 - ev->sCycle;
    if (ev->isWrite()) {
        profWrites.inc();
        profTotalWrLat.inc(lat);
    } else {
        profReads.inc();
        profTotalRdLat.inc(lat);
        uint32_t bucket = std::min(NUMBINS-1, lat/BINSIZE);
        latencyHist.inc(bucket, 1);
    }

    ev->release();
    ev->done(curCycle+1);

    if (creq == nextSchedRequest)
        nextSchedRequest = NULL;

    inflightRequests.erase(it);

    //info("[%s] [RequestComplete] %s access to %lx DONE at %ld (%ld cycles), %ld inflight reqs", getName(), ev->isWrite()? "W" : "R", ev->getAddr(), curCycle, lat, inflightRequests.size());

    delete creq;
    return true;
}

void NVMainMemory::printStats() {
    //info("Print NVMain stats for %s, curCycle %ld, updateCycle %ld", getName(), dynamic_cast<OOOCore*>(zinfo->cores[0])->getCycles(), updateCycle);
    std::ofstream out(nvmainStatsFile, std::ios_base::app);

    nvmainPtr->CalculateStats();
    nvmainPtr->GetStats()->PrintAll(out);

    out << "===" << std::endl;
}

#else //no nvmain, have the class fail when constructed

NVMainMemory::NVMainMemory(std::string& nvmainTechIni, std::string& outputFile, std::string& traceName,
        uint32_t capacityMB, uint64_t _minLatency, uint32_t _domain, const g_string& _name)
{
    panic("Cannot use NVMainMemory, zsim was not compiled with NVMain");
}

void NVMainMemory::initStats(AggregateStat* parentStat) { panic("???"); }
uint64_t NVMainMemory::access(MemReq& req) { panic("???"); return 0; }
uint64_t NVMainMemory::tick(uint64_t cycle) { panic("???"); return 0; }
void NVMainMemory::enqueue(NVMainAccEvent* ev, uint64_t cycle) { panic("???"); }
void NVMainMemory::recycleEvent(SchedEventNVMain* ev) { panic("???"); }
bool NVMainMemory::RequestComplete(NVM::NVMainRequest *creq) { panic("???"); }
void NVMainMemory::printStats() { panic("???"); }

#endif

