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

#ifndef NVMAIN_MEM_CTRL_H_
#define NVMAIN_MEM_CTRL_H_

#include <map>
#include <unordered_map>
#include <set>
#include <string>
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include "pad.h"
#include "stats.h"

// NVMain defines MAX and MIN as functions.
#ifdef MAX
#undef MAX
#endif

#ifdef MIN
#undef MIN
#endif

#include "NVM/nvmain.h"
#include "src/Config.h"
#include "src/EventQueue.h"
#include "src/NVMObject.h"
#include "src/SimInterface.h"

#define MIN(x, y) ({ __typeof__(x) xx = (x); __typeof__(y) yy = (y); (xx < yy)? xx : yy;})
#define MAX(x, y) ({ __typeof__(x) xx = (x); __typeof__(y) yy = (y); (xx > yy)? xx : yy;})

//using namespace NVM;

class NVMainAccEvent;
class SchedEventNVMain;

class NVMainMemory : public MemObject, public NVM::NVMObject { //one NVMain controller
    private:
        g_string name;
        uint64_t minLatency;
        uint64_t domain;

        NVM::NVMainRequest *nvmainRetryRequest;
        NVM::NVMain *nvmainPtr;
        NVM::SimInterface *nvmainSimInterface;
        NVM::Config *nvmainConfig;
        NVM::EventQueue *nvmainEventQueue;
        NVM::Stats *nvmainStatsPtr;
        NVM::GlobalEventQueue *nvmainGlobalEventQueue;
        NVM::TagGenerator *nvmainTagGenerator;

        std::vector<std::pair<NVM::NVMainRequest*, std::pair<NVMainAccEvent*, uint64_t>>> inflightRequests;
        std::unordered_map<uint64_t,uint64_t> memoryHistogram;

        uint64_t curCycle; //processor cycle, used in callbacks
        uint64_t updateCycle; //latest cycle where nvmain was updated
        bool eventDriven;
        bool ignoreData;

        // R/W stats
        PAD();
        Counter profIssued;
        Counter profReads;
        Counter profWrites;
        Counter profPUTS;
        Counter profPUTX;
        Counter profTotalRdLat;
        Counter profTotalWrLat;
        Counter profMemoryFootprint;
        Counter profMemoryAddresses;
        VectorCounter latencyHist;
        VectorCounter addressReuseHist;
        static const uint64_t BINSIZE = 10, NUMBINS = 100;
        PAD();

        // Stats file name
        const char* nvmainStatsFile;

        // Wave phase information
        SchedEventNVMain* nextSchedEvent;
        NVM::NVMainRequest* nextSchedRequest;
        SchedEventNVMain* eventFreelist;


    public:
        NVMainMemory(std::string& nvmainTechIni, std::string& outputFile, std::string& traceName, uint32_t capacityMB, uint64_t _minLatency, uint32_t _domain, const g_string& _name);

        const char* getName() {return name.c_str();}

        void initStats(AggregateStat* parentStat);

        // Record accesses
        uint64_t access(MemReq& req);

        // Event-driven simulation (phase 2)
        uint64_t tick(uint64_t cycle);
        void enqueue(NVMainAccEvent* ev, uint64_t cycle);
        void recycleEvent(SchedEventNVMain* ev);

        bool RequestComplete(NVM::NVMainRequest *creq);
        void Cycle(NVM::ncycle_t){};
        void printStats();
};

#endif  // NVMAIN_MEM_CTRL_H_
