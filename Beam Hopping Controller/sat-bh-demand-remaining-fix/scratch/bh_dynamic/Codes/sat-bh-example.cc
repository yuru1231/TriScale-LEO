/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-example.cc
 *
 * Unified Beam Hopping example — covers Phase 1 through Phase 3.
 * (Layer2.md Section 10-12; replaces sat-bh-phase1-example.cc)
 *
 * USAGE
 * -----
 * Phase 1 (default — data model + metrics with synthetic driver):
 *   ./ns3 run "sat-bh-example"
 *
 * Phase 2 (enable Scheduler + OBC stubs — interfaces tested):
 *   ./ns3 run "sat-bh-example --enableScheduler=1 --enableObc=1"
 *
 * Phase 3 (enable CacheQueue + MMSE stubs):
 *   ./ns3 run "sat-bh-example --enableScheduler=1 --enableObc=1 \
 *                             --enableCacheQueue=1 --enablePrecoder=1"
 *
 * Phase C (enable ResourceManager self-scheduling loop):
 *   ./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1"
 *
 * Phase E (wire real SNS3 APIs — MoveUtBetweenBeams + SetTxMaxPowerDbw):
 *   ./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
 *                             --enablePowerAllocation=1 --enablePhaseE=1 --satId=1"
 *
 * Phase F (wire real DAMA demand traces — BacklogRequestsTrace → ResourceManager):
 *   ./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
 *                             --enablePowerAllocation=1 --enablePhaseE=1 \
 *                             --enablePhaseF=1 --satId=1 --simTime=120"
 *
 * Phase G (dynamic greedy BSTP — no ResourceManager required):
 *   ./ns3 run "sat-bh-example --enableObc=1 --enableDynamicBstp=1"
 *
 * Phase G + real RBDC demand:
 *   ./ns3 run "sat-bh-example --enableObc=1 --enableDynamicBstp=1 \
 *                             --enableResourceManager=1 --enablePhaseE=1 \
 *                             --enablePhaseF=1 --simTime=120"
 *
 * starlink25 — 25-beam Tokyo fixed-snapshot baseline (Phase G + hotspot 5:1 demand):
 *   ./ns3 run "sat-bh-example --scenario=starlink25 \
 *                             --enableObc=1 --enableDynamicBstp=1 \
 *                             --maxActiveBeams=4 --simTime=60"
 *
 * starlink25 + Phase F (real RBDC demand — Phase G demand path, no ResourceManager needed):
 *   ./ns3 run "sat-bh-example --scenario=starlink25 \
 *                             --enableObc=1 --enableDynamicBstp=1 \
 *                             --enablePhaseF=1 \
 *                             --maxActiveBeams=2 --simTime=120"
 *
 * starlink25 + Phase F (full ResourceManager path — Phase E required):
 *   ./ns3 run "sat-bh-example --scenario=starlink25 \
 *                             --enableObc=1 --enableDynamicBstp=1 \
 *                             --enableResourceManager=1 --enablePhaseE=1 --enablePhaseF=1 \
 *                             --maxActiveBeams=2 --simTime=120"
 *
 * starlink25 ROI-limited (reduce helpers from 1584→N for lighter runs):
 *   ROI centre: Tokyo (35.676°N, 139.650°E).
 *   satIdStart: first satellite ID visible over ROI (from 2D orbit-sgp4 scan).
 *   maxHelperSats: window size — e.g. 20 neighbours around the peak-elev sat.
 *   minElevDeg: logged for reference; use the 2D scan to choose satIdStart.
 *
 *   ./ns3 run "sat-bh-example --scenario=starlink25 \
 *                             --enableObc=1 --enableDynamicBstp=1 \
 *                             --maxActiveBeams=4 --simTime=60 \
 *                             --satIdStart=480 --maxHelperSats=20 \
 *                             --minElevDeg=25 \
 *                             --roiLat=35.676 --roiLon=139.650 --roiRadius=5.0"
 *
 * NOTE: ns-3 CommandLine bool values use 0/1, NOT true/false.
 *
 * DESIGN PRINCIPLE
 * ----------------
 * This file calls ONLY SatBhHelper and reads BhExperimentConfig.
 * It NEVER calls Scheduler / OBC / CacheQueue / Precoder directly.
 * As Phase 2/3 implementations are filled in, ONLY the Helper's .cc changes.
 * This file remains unchanged for the entire Phase 1 → Phase 3 progression.
 * (Satisfies "保留接口，避免後續e2e出問題")
 *
 * OUTPUTS
 * -------
 *   --outputDir=DIR places all example outputs under DIR:
 *   bh-metrics_*.csv        — KPI rows per T_p = 80 ms (SatBhMetrics CSV)
 *   bh-timeplan_*.csv       — BHTP slot table (SatBhTimePlan ToCsv)
 *   sat-bh-traffic_*.tr     — demand/plan/event trace; DEMAND/SERVICE_ACCOUNTING
 *                             rows include scheduler-level remaining_kbps
 *   bh-demand-actual_*.csv  — actual remaining demand from PacketSink::Rx throughput
 *   bh-attributes.xml       — ns-3 ConfigStore attribute snapshot
 *   sns3-stats/             — SNS3 built-in statistics output
 */

#include "ns3/sat-bh-helper.h"
#include "sat-constellation-params.h"

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"

#include "ns3/packet-sink.h"   // Phase F PDR: DynamicCast<PacketSink> + GetTotalRx()

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

using namespace ns3;

// ── Phase F FWD PDR tracker ───────────────────────────────────────────────
//
// Per-beam accumulator keyed by beam index (0..24 for starlink25).
// If the 25-beam ROI grid is applied to multiple helper sats, UTs are created
// in 25-UT groups; utIdx % 25 maps back to beamId.
// FwdRxTrace is wired to PacketSink::Rx on each UT user node in Step 3.5.
// PDR is printed at Step 9 from accumulated rxBytes vs. offered bytes.

struct BeamPdrEntry
{
    uint32_t beamId{0};
    uint64_t rxBytes{0};   ///< bytes received at UT PacketSink (FWD)
};

static std::vector<BeamPdrEntry> g_beamPdr;         ///< size == num beams
static Ptr<OutputStreamWrapper>  g_fwdAsciiStream;  ///< sat-bh-phaseF-fwd.tr
static std::vector<std::vector<uint64_t>> g_beamRxBytesByPeriod;
static double g_actualDemandWarmUpSec{0.0};
static double g_actualDemandPeriodSec{0.0};

/// Callback: fired by PacketSink::Rx on UT user node i.
/// Writes one ASCII line (mimics AsciiTraceHelper format) and accumulates bytes.
static void
FwdRxTrace(uint32_t utIdx, Ptr<const Packet> pkt, const Address& /*from*/)
{
    const uint32_t beamIdx = g_beamPdr.empty() ? 0 : utIdx % g_beamPdr.size();
    const uint32_t beamId = g_beamPdr.empty() ? 0 : g_beamPdr[beamIdx].beamId;

    if (!g_beamPdr.empty())
    {
        g_beamPdr[beamIdx].rxBytes += pkt->GetSize();
    }

    const double nowSec = Simulator::Now().GetSeconds();
    if (!g_beamPdr.empty() &&
        g_actualDemandPeriodSec > 0.0 &&
        nowSec >= g_actualDemandWarmUpSec)
    {
        const uint32_t periodIdx =
            static_cast<uint32_t>((nowSec - g_actualDemandWarmUpSec) /
                                  g_actualDemandPeriodSec);
        if (g_beamRxBytesByPeriod.size() <= periodIdx)
        {
            g_beamRxBytesByPeriod.resize(periodIdx + 1,
                                         std::vector<uint64_t>(g_beamPdr.size(), 0));
        }
        g_beamRxBytesByPeriod[periodIdx][beamIdx] += pkt->GetSize();
    }

    // ASCII line: "time  r  path  bytes=N  beam=B"
    *g_fwdAsciiStream->GetStream()
        << std::fixed << std::setprecision(6)
        << Simulator::Now().GetSeconds()
        << "\tr"
        << "\t/NodeList/UT" << utIdx << "/App/PacketSink/Rx"
        << "\tbytes=" << pkt->GetSize()
        << "\tbeam=" << beamId
        << "\n";
}

NS_LOG_COMPONENT_DEFINE("sat-bh-example");

static std::vector<uint32_t>
ParseHelperSatList(const std::string& helperSatList, uint32_t numSats)
{
    std::vector<uint32_t> satIds;
    if (helperSatList.empty())
    {
        return satIds;
    }

    std::set<uint32_t> seen;
    std::stringstream ss(helperSatList);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        if (token.empty())
        {
            continue;
        }

        uint32_t satId = static_cast<uint32_t>(std::stoul(token));
        NS_ABORT_MSG_IF(satId >= numSats,
                        "helperSatList contains satId=" << satId
                                                        << " but numSats=" << numSats);
        if (seen.insert(satId).second)
        {
            satIds.push_back(satId);
        }
    }

    return satIds;
}

static std::string
FormatHelperSatIds(const std::vector<uint32_t>& satIds)
{
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < satIds.size(); ++i)
    {
        if (i > 0)
        {
            oss << ",";
        }
        oss << satIds[i];
    }
    oss << "]";
    return oss.str();
}

// ── ParseConfig: command-line → BhExperimentConfig ────────────────────────
//
// All simulation knobs are exposed as CommandLine arguments.
// Adding a new knob in Phase 2/3 means adding a line here and in
// BhExperimentConfig — no structural change to the rest of the file.
//
static BhExperimentConfig
ParseConfig(int argc,
            char* argv[],
            std::string& scenario,
            uint32_t& numSats,
            std::string& helperSatList,
            std::string& outputDir)
{
    BhExperimentConfig cfg;

    CommandLine cmd;

    // ── Scenario selector (example-level, not stored in BhExperimentConfig) ──
    cmd.AddValue("scenario",
                 "starlink/starlink25: 25-beam Tokyo baseline | leo2sat: scheduling verify | iridium-next: Iridium NEXT 66-sat system",
                 scenario);
    cmd.AddValue("numSats",  "satellites to install BH on (overridden by scenario)", numSats);

    // ── Scenario ──────────────────────────────────────────────────────────
    cmd.AddValue("simTime",        "Simulation duration [s] (ignored when numPeriods>0)", cfg.simTimeSec);
    cmd.AddValue("numPeriods",     "Run exactly N BHTP periods after warmup (0=use simTime)", cfg.numPeriods);
    cmd.AddValue("warmUp",         "Warm-up period to discard [s]",       cfg.warmUpSec);
    cmd.AddValue("numBeams",       "Total beams per satellite",           cfg.numBeams);
    cmd.AddValue("maxActiveBeams", "K: max simultaneous beams per slot",  cfg.maxActiveBeams);
    cmd.AddValue("numHotspot",     "Number of hotspot beams (1-indexed)", cfg.numHotspotBeams);
    cmd.AddValue("satId",          "Satellite index to install BH on",    cfg.satId);

    // ── Algorithm ─────────────────────────────────────────────────────────
    cmd.AddValue("alpha",  "α: delay sensitivity factor (virtual traffic)", cfg.alphaDelaySensitivity);
    cmd.AddValue("kappa",  "κ: cluster interference merge threshold",       cfg.interferenceKappa);

    // ── Timing ────────────────────────────────────────────────────────────
    cmd.AddValue("slotMs", "T_s: slot duration [ms]",       cfg.slotDurationMs);
    cmd.AddValue("periodMs","T_p: BHTP period [ms]",        cfg.bhtpPeriodMs);
    cmd.AddValue("switchMs","T_sw: switching dead-time [ms]",cfg.switchingTimeMs);
    cmd.AddValue("propMs", "T_prop: propagation delay [ms]", cfg.propagationDelayMs);

    // ── Feature flags (phase activation) ──────────────────────────────────
    // Set false by default; flip to true when Phase N implementation is ready
    cmd.AddValue("enableScheduler",  "Phase 2: use SatBhScheduler",      cfg.enableScheduler);
    cmd.AddValue("enableObc",        "Phase 2: use SatBhObc",            cfg.enableObc);
    cmd.AddValue("enableCacheQueue", "Phase 3: use SatGwCacheQueue",     cfg.enableCacheQueue);
    cmd.AddValue("enablePrecoder",   "Phase 3: use SatBhPrecoder (MMSE)",cfg.enablePrecoder);

    // ── Phase C flags ──────────────────────────────────────────────────────
    cmd.AddValue("enableResourceManager", "Phase C: SatResourceManager self-scheduling loop",
                 cfg.enableResourceManager);
    cmd.AddValue("enableUserAssociation", "Phase C: run SatUserAssociator each frame",
                 cfg.enableUserAssociation);
    // schedulingMode is uint8_t; ns-3 CommandLine only supports uint32_t — proxy variable
    uint32_t schedulingModeArg = cfg.schedulingMode;
    cmd.AddValue("schedulingMode",  "Phase C: 0=WFQ 1=Priority 2=RoundRobin", schedulingModeArg);
    cmd.AddValue("maxReassignment", "Phase C: max MoveUtBetweenBeams calls per frame",
                 cfg.maxReassignmentPerFrame);

    // ── Phase D flags ──────────────────────────────────────────────────────
    cmd.AddValue("enablePowerAllocation", "Phase D: run SatPowerAllocator (IWFA) each frame",
                 cfg.enablePowerAllocation);

    // ── Phase E flags ──────────────────────────────────────────────────────
    cmd.AddValue("enablePhaseE",
                 "Phase E: wire callbacks to real SNS3 (MoveUtBetweenBeams + SetTxMaxPowerDbw)",
                 cfg.enablePhaseE);

    // ── Phase F flags ──────────────────────────────────────────────────────
    cmd.AddValue("enablePhaseF",
                 "Phase F: wire BacklogRequestsTrace -> ResourceManager real RBDC demand",
                 cfg.enablePhaseF);

    // ── Phase G flags (dynamic BSTP provider) ──────────────────────────────
    // Mutually exclusive with enableScheduler (Phase 2 EM algorithm).
    // Requires enableObc=1 to actually toggle GW SatNetDevices.
    // enablePhaseF=1 feeds real RBDC demand; without it, provider uses round-robin.
    //
    // Usage:
    //   ./ns3 run "sat-bh-example --enableObc=1 --enableDynamicBstp=1"
    //
    // With real demand:
    //   ./ns3 run "sat-bh-example --enableObc=1 --enableDynamicBstp=1 \
    //                             --enablePhaseE=1 --enablePhaseF=1 \
    //                             --enableResourceManager=1"
    cmd.AddValue("enableDynamicBstp",
                 "Phase G: use SatDynamicBstpProvider (greedy top-K beam selector)",
                 cfg.enableDynamicBstp);
    cmd.AddValue("bhDemandWeight",
                 "Phase G: score weight for RBDC demand [kbps]",
                 cfg.bhDemandBacklogWeight);
    cmd.AddValue("bhFairnessWeight",
                 "Phase G: score weight for time-since-served [s]",
                 cfg.bhFairnessWeight);
    cmd.AddValue("bhValiditySF",
                 "Phase G: validityInSuperframes (plan window = bhValiditySF * T_p)",
                 cfg.bhValiditySuperframes);
    cmd.AddValue("bhStarvationThr",
                 "Phase G: force-include beam after N consecutive skips",
                 cfg.bhStarvationThreshold);
    cmd.AddValue("enableFwdOfferedDemand",
                 "Phase G: feed FWD offered load as scheduler demand",
                 cfg.enableFwdOfferedDemand);
    cmd.AddValue("fwdOfferedDemandKbps",
                 "Phase G: offered FWD demand per beam [kbps]",
                 cfg.fwdOfferedDemandKbps);

    cmd.AddValue("totalPowerDbm",    "Phase D: total TX power budget [dBm]",
                 cfg.totalPowerBudgetDbm);
    cmd.AddValue("noisePowerDbw",    "Phase D: thermal noise floor [dBW]",
                 cfg.noisePowerDbw);
    cmd.AddValue("powerMaxIter",     "Phase D: IWFA max iterations",
                 cfg.powerMaxIterations);
    cmd.AddValue("powerEps",         "Phase D: IWFA convergence threshold [W]",
                 cfg.powerConvergenceEps);
    cmd.AddValue("interferenceFactor","Phase D: cross-beam ICI leakage fraction",
                 cfg.interferenceFactor);

    // ── ROI / elevation filter (starlink-1584 scale-down) ─────────────────
    // These parameters limit BH helper installation to a geographic window so
    // that the full 1584-sat constellation can be loaded without creating 1584
    // SatBhHelper objects.  Use the 2D orbit-sgp4 scan to choose satIdStart.
    cmd.AddValue("roiLat",        "ROI centre latitude  [deg N]",              cfg.roiLat);
    cmd.AddValue("roiLon",        "ROI centre longitude [deg E]",              cfg.roiLon);
    cmd.AddValue("roiRadius",     "ROI radius [deg] (approx great-circle)",    cfg.roiRadiusDeg);
    cmd.AddValue("minElevDeg",    "Min satellite elevation [deg] (reference)",  cfg.minElevDeg);
    cmd.AddValue("altitudeKm",    "Satellite orbital altitude [km]",            cfg.altitudeKm);
    cmd.AddValue("beamHalfAngle", "Beam half-power HALF-angle [deg] (MIDDLE=2.0)", cfg.beamHalfAngleDeg);
    cmd.AddValue("maxHelperSats", "Max BH helpers to install (0=all)",         cfg.maxHelperSats);
    cmd.AddValue("satIdStart",    "First satId in the monitoring window",      cfg.satIdStart);
    cmd.AddValue("helperSatList",
                 "Comma-separated non-contiguous BH helper satellite IDs; overrides satIdStart/maxHelperSats when set",
                 helperSatList);

    // ── Output ────────────────────────────────────────────────────────────
    cmd.AddValue("metricsFile",  "KPI CSV output file",       cfg.metricsOutputFile);
    cmd.AddValue("timePlanFile", "BHTP slot table CSV file",  cfg.timePlanCsvFile);
    cmd.AddValue("trafficTraceFile",
                 "BH real demand / plan / OBC event trace file",
                 cfg.trafficTraceFile);
    cmd.AddValue("outputDir",
                 "Directory for BH example outputs (empty = current working directory)",
                 outputDir);

    cmd.Parse(argc, argv);
    cfg.schedulingMode = static_cast<uint8_t>(schedulingModeArg);

    // Derive simTimeSec from numPeriods when explicitly set.
    // simTimeSec = warmUpSec + numPeriods × bhtpPeriodMs / 1000
    if (cfg.numPeriods > 0)
        cfg.simTimeSec = cfg.warmUpSec + cfg.numPeriods * cfg.bhtpPeriodMs / 1000.0;

    return cfg;
}

// ── main ──────────────────────────────────────────────────────────────────

int
main(int argc, char* argv[])
{
    // ── Step 0: parse experiment configuration ────────────────────────────
    std::string scenario{"starlink25"};  // "starlink25" | "leo2sat" | "iridium-next"
    uint32_t    numSats{1};

    std::string helperSatList;
    std::string outputDir;
    BhExperimentConfig cfg = ParseConfig(argc, argv, scenario, numSats, helperSatList, outputDir);
    if (scenario == "starlink")
    {
        scenario = "starlink25";
    }
    if (!outputDir.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        NS_ABORT_MSG_IF(ec, "Cannot create outputDir=" << outputDir << ": " << ec.message());
    }
    std::filesystem::path actualDemandPath;
    std::vector<uint32_t> starlinkBeamSatIds;

    // ── Step 1: SNS3 global configuration ────────────────────────────────
    // Regeneration mode: consistent with sat-gw-handover-example.cc reference
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));

    // Iridium NEXT constellation waveform:
    //   constellation-iridium-next-66-sats/waveforms/default_waveform.txt = 3
    //   waveforms.txt index 3 → QPSK 1/3, DVB standard
    // Enable handover so topology evolves as LEO satellites pass overhead
    Config::SetDefault("ns3::SatHelper::HandoversEnabled",          BooleanValue(true));
    Config::SetDefault("ns3::SatHandoverModule::NumberClosestSats",  UintegerValue(2));
    Config::SetDefault("ns3::SatGwMac::DisableSchedulingIfNoDeviceConnected",
                       BooleanValue(true));
    Config::SetDefault("ns3::SatOrbiterMac::DisableSchedulingIfNoDeviceConnected",
                       BooleanValue(true));
    Config::SetDefault("ns3::SatOrbiterFeederPhy::QueueSize",        UintegerValue(100000));

    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));
    Config::SetDefault("ns3::SatHelper::PacketTraceEnabled",         BooleanValue(true));

    // ── Step 2: build SNS3 simulation scenario ────────────────────────────
    Ptr<SimulationHelper>     simHelper = CreateObject<SimulationHelper>("sat-bh-example");
    Ptr<SimulationHelperConf> simConf   = CreateObject<SimulationHelperConf>();

    simHelper->SetSimulationTime(Seconds(cfg.simTimeSec));
    simHelper->SetGwUserCount(1);
    simHelper->SetUserCountPerUt(1);

    // Scenario-dependent: beam set and constellation
    std::set<uint32_t> beamSet;
    if (scenario == "iridium-next" || scenario == "iridium66")
    {
        // Iridium NEXT constellation (constellation-iridium-next-66-sats):
        //   - 66 satellites: 6 orbital planes × 11 sats/plane
        //   - Inclination: 86.4°, Mean motion: 14.80 rev/day → altitude ~636.5 km
        //   - 72 spot beams total (fwdConf.txt: 72 entries)
        //   - 5 GW positions (gw_positions.txt), 4 active GWs (fwdConf column 2: IDs 1–4)
        //   - 5 frequency reuse colours (fwdConf column 3: colours 1–5)
        //   - Antenna pattern: SatAntennaGain72BeamsShifted (peak ~51 dBi)
        //   - Default MODCOD: 3 (QPSK 1/3, DVB; waveforms/default_waveform.txt)
        //
        // cfg.numBeams = total beams to activate (command-line: --numBeams, default 72).
        //   Quick test  : --numBeams=7  --maxActiveBeams=3
        //   Full system : --numBeams=72 --maxActiveBeams=3  (full Iridium NEXT)
        //
        // Beam-to-satellite assignment: round-robin across satellites.
        //   beam b (1-indexed) → satId = (b-1) % numSats
        //   e.g. numBeams=72, numSats=66 → sats 0–5 each carry 2 beams, sats 6–65 carry 1 beam
        //        → maxBeamsPerSat = ceil(72/66) = 2
        //
        // maxBeamsPerSat = ceil(numBeams / numSats), passed to SatBhHelper::SetNumBeams()
        // so the BHTP builder allocates the right number of BHTP slots per satellite.
        numSats = 66;

        // Clamp numBeams to valid range [1, 72] for Iridium NEXT
        if (cfg.numBeams < 1)  cfg.numBeams = 72;  // full constellation default
        if (cfg.numBeams > 72) cfg.numBeams = 72;

        // maxBeamsPerSat: how many beams the busiest satellite carries.
        // Used by SatBhHelper BHTP builder (SetNumBeams).
        uint32_t maxBeamsPerSat = (cfg.numBeams + numSats - 1) / numSats;  // ceil division

        // numHotspotBeams must not exceed total beams; clamp if needed.
        if (cfg.numHotspotBeams > cfg.numBeams)
            cfg.numHotspotBeams = cfg.numBeams / 2 + 1;

        // maxActiveBeams K must not exceed total beams; user controls this via --maxActiveBeams.
        if (cfg.maxActiveBeams > cfg.numBeams)
            cfg.maxActiveBeams = cfg.numBeams;

        // Populate beamSet with all beam IDs for the iridium-next scenario.
        // BeamUserInfoMap_t / SetBeamUserInfo is not available in this SNS3 build;
        // UT placement is handled by the loaded scenario's default configuration.
        for (uint32_t b = 1; b <= cfg.numBeams; ++b)
            beamSet.insert(b);

        NS_LOG_INFO("[iridium-next] numBeams=" << cfg.numBeams
                    << " K=" << cfg.maxActiveBeams
                    << " maxBeamsPerSat=" << maxBeamsPerSat
                    << " hotspot=" << cfg.numHotspotBeams);

        // constellation-iridium-next-66-sats:
        //   altitude ~636.5 km, inclination 86.4°, 72 beams, DVB standard
        //   antenna: SatAntennaGain72BeamsShifted (peak avg 51.4 dBi)
        simHelper->LoadScenario("constellation-iridium-next-66-sats");

        // Override numBeams in cfg to reflect maxBeamsPerSat so SatBhHelper
        // BHTP builder sees the correct per-satellite beam count.
        cfg.numBeams = maxBeamsPerSat;
    }
    else if (scenario == "starlink25")
    {
        // Fixed peak-elevation snapshot: Starlink-550 sat_498, t=4168s,
        // peak_elev=89.755°, alt=511.437km over Tokyo (35.676°N, 139.650°E).
        //
        // Constellation: constellation-starlink-1584-sats (same as 2D/orbit-sgp4).
        // 25-beam 5×5 UPA elliptic grid; beamId mapping: cell_idx i → beamId i+1.
        // Hotspot cells (cell_idx {0,3,12,18,21}) → beamId {1,4,13,19,22} at 5× demand.
        // UTs placed at exact cell centers after CreateSatScenario() in Step 2.5.
        //
        // Compute the 5×5 beam grid lat/lon from altitude-derived geometry.
        // ConstellationParams::GridPositions() returns (x=East, y=North) offsets [km]
        // in the same indexing as the old static arrays (row 0 = South, col 0 = West).
        //
        // Geographic conversion (flat-Earth approximation, valid over ~200 km):
        //   deltaLat_deg = y_km / R_earth × (180/π)
        //   deltaLon_deg = x_km / (R_earth × cos(latC_rad)) × (180/π)
        // where R_earth = 6371 km, (latC, lonC) = ROI centre from cfg.
        constexpr uint32_t kPeakSatId = 498;
        const double kLatC  = cfg.roiLat;                          // Tokyo 35.676 °N
        const double kLonC  = cfg.roiLon;                          // Tokyo 139.650 °E
        const double kRearth = 6371.0;
        const double kPi    = std::acos(-1.0);
        const double kLatCrad = kLatC * kPi / 180.0;

        ConstellationParams cp{cfg.altitudeKm, cfg.minElevDeg};
        auto gridXY = cp.GridPositions(5, 5, cfg.beamHalfAngleDeg);  // 25 positions

        std::vector<double> kCellLat(25), kCellLon(25);
        for (std::size_t i = 0; i < 25; ++i)
        {
            double x_km = gridXY[i].first;   // East  offset from ROI centre
            double y_km = gridXY[i].second;  // North offset from ROI centre
            kCellLat[i] = kLatC + y_km / kRearth * (180.0 / kPi);
            kCellLon[i] = kLonC + x_km / (kRearth * std::cos(kLatCrad)) * (180.0 / kPi);
        }

        // Keep the complete constellation available for helper installation.
        // Apply the 25-beam Tokyo ROI grid only to the selected helper sats,
        // not globally to all 1584 satellites.  This keeps the run bounded
        // while letting each ROI helper map its beams to real SatNetDevices.
        numSats = 1584;
        cfg.numBeams       = 25;
        cfg.maxActiveBeams = std::min(cfg.maxActiveBeams, 25u);

        starlinkBeamSatIds = ParseHelperSatList(helperSatList, numSats);
        if (starlinkBeamSatIds.empty() && cfg.maxHelperSats > 0)
        {
            uint32_t helperStart = std::min(cfg.satIdStart, numSats - 1);
            uint32_t helperCount = std::min(cfg.maxHelperSats, numSats - helperStart);
            for (uint32_t i = 0; i < helperCount; ++i)
            {
                starlinkBeamSatIds.push_back(helperStart + i);
            }
        }

        if (starlinkBeamSatIds.empty())
        {
            starlinkBeamSatIds.push_back(kPeakSatId);
            NS_LOG_WARN("[starlink25] helper set resolves to all/empty; limiting"
                        " ROI beam creation to peak sat " << kPeakSatId);
        }

        // Default 5 hotspot beams (cell_idx {0,3,12,18,21} = beamId {1,4,13,19,22})
        // Phase F uniform load — no hotspot differentiation.
        // All 25 beams carry equal 20 Mbps FWD demand (configured in Step 3).
        // hotCellIndices and bhFwdHotspotBeamIds remain empty so Phase G
        // DynamicBstp sees uniform RBDC → natural round-robin beam selection.
        cfg.numHotspotBeams = 0;
        cfg.enableFwdOfferedDemand = true;
        if (cfg.fwdOfferedDemandKbps <= 0.0)
        {
            cfg.fwdOfferedDemandKbps = 20000.0;  // 1500 bytes / 600 us = 20 Mbps
        }

        SatHelper::BeamUserInfoMap_t beamInfo;
        for (uint32_t satId : starlinkBeamSatIds)
        {
            for (uint32_t i = 0; i < 25; ++i)
            {
                const uint32_t beamId = i + 1;
                SatBeamUserInfo info;
                info.SetPositions(
                    {std::make_pair(GeoCoordinate(kCellLat[i], kCellLon[i], 0.0), 0)});
                info.AppendUt(1);
                beamInfo.emplace(std::make_pair(satId, beamId), info);
                beamSet.insert(beamId);
            }
        }

        NS_LOG_INFO("[starlink25] constellation-starlink-1584-sats x 25 beams"
                    << " K=" << cfg.maxActiveBeams
                    << " hotspot=" << cfg.numHotspotBeams
                    << " ROI=(" << cfg.roiLat << "N," << cfg.roiLon << "E)"
                    << " altitude=" << cfg.altitudeKm << "km"
                    << " halfAngle=" << cfg.beamHalfAngleDeg << "deg"
                    << " roiRadius=" << cfg.roiRadiusDeg << "deg"
                    << " minElev=" << cfg.minElevDeg << "deg"
                    << " helperWindow=[" << cfg.satIdStart
                    << "," << (cfg.satIdStart + cfg.maxHelperSats) << ")"
                    << " (maxHelperSats=" << cfg.maxHelperSats << ")"
                    << " helperSatList="
                    << (helperSatList.empty() ? "(empty)" : helperSatList)
                    << " beamSatIds=" << FormatHelperSatIds(starlinkBeamSatIds));

        Config::SetDefault("ns3::SatHelper::HandoversEnabled", BooleanValue(false));
        Config::SetDefault("ns3::SatHelper::IslsEnabled", BooleanValue(false));
        simHelper->LoadScenario("constellation-starlink-1584-sats");
        simHelper->SetConstellationBeamUserInfo(beamInfo);
        simHelper->SetConstellationTimeOffset(Seconds(4168));
        simHelper->SetCustomConstellationGatewayId(1);
    }
    else
    {
        // leo2sat: scheduling logic verification, numBeams beams on satId=0
        for (uint32_t i = 1; i <= cfg.numBeams; i++)
            beamSet.insert(i);
        numSats = 1;
        simHelper->LoadScenario("constellation-leo-2-satellites");
    }
    simHelper->SetBeamSet(beamSet);
    simHelper->SetUserCountPerMobileUt(simConf->m_utMobileUserCount);

    // Phase F: enable RBDC on DaService3 so UTs generate non-zero capacity requests.
    // By default iridium66 uses CRA (ConstantAssignmentProvided=true), which causes
    // BacklogRequestsTrace to always report 0 kbps RBDC even under traffic load.
    // Disabling CRA and enabling RBDC forces UTs to dynamically request bandwidth,
    // producing non-zero demand values that Phase F can forward to ResourceManager.
    // Config::SetDefault must be called before CreateSatScenario().
    if (cfg.enablePhaseF)
    {
        Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService3_ConstantAssignmentProvided",
                           BooleanValue(false));
        Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService3_RbdcAllowed",
                           BooleanValue(true));
        Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService3_MinimumServiceRate",
                           UintegerValue(10));
        Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService3_MaximumServiceRate",
                           UintegerValue(500));
        Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService3_VolumeAllowed",
                           BooleanValue(false));
        Config::SetDefault("ns3::SatBeamScheduler::ControlSlotsEnabled",
                           BooleanValue(true));
        Config::SetDefault("ns3::SatBeamScheduler::ControlSlotInterval",
                           TimeValue(Seconds(1)));
        NS_LOG_INFO("[PhaseF] RBDC enabled: DaService3 CRA=off, RBDC=on,"
                    " MinRate=10kbps, MaxRate=500kbps, VBDC=off, control slots=1s");
    }

    // ── OBC real-toggle mode: disable SNS3 static BSTP controller to prevent two controllers
    // toggling the same GW SatNetDevices simultaneously.
    //
    // Background: SatBeamHelper::Install() registers ToggleState callbacks with
    // SatBstpController, which fires them on a static BSTP file schedule.
    // SatBhObc fires the SAME callbacks on its own scheduler/dynamic schedule.
    // If both are active, they will interleave and produce undefined beam state.
    //
    // Fix: prevent SatBeamHelper from constructing SatBstpController at all.
    // Setting EnableFwdLinkBeamHopping=false means DoBstpConfiguration() is never
    // scheduled, leaving GW SatNetDevices under exclusive control of SatBhObc.
    //
    // NOTE: SatBstpController enum has only BH_STATIC=1 and BH_DYNAMIC=2 in
    // ns-3.43 — there is no BH_OFF value, so StringValue("BH_OFF") would abort.
    // Must be set before CreateSatScenario() so SatBeamHelper reads it during build.
    if (cfg.enableObc && (cfg.enableDynamicBstp || cfg.enableScheduler))
    {
        Config::SetDefault("ns3::SatBeamHelper::EnableFwdLinkBeamHopping",
                           BooleanValue(false));
        NS_LOG_INFO("[OBC] SatBeamHelper::EnableFwdLinkBeamHopping=false"
                    " — static SatBstpController suppressed to avoid toggle conflict");
    }

    simHelper->CreateSatScenario(SatHelper::NONE);

    // The Starlink BeamUserInfoMap above creates the UTs directly at the
    // peak-elevation cell centres.  Do not call MobilityModel::SetPosition()
    // with GeoCoordinate::ToVector() here: this satellite mobility model
    // expects geographic coordinates, not ECEF metres.
    if (scenario == "starlink25")
    {
        NodeContainer utNodes = Singleton<SatTopology>::Get()->GetUtNodes();
        const uint32_t expectedUts = 25 * std::max<std::size_t>(1, starlinkBeamSatIds.size());
        NS_ASSERT_MSG(utNodes.GetN() == expectedUts,
                      "[starlink25] scenario created " << utNodes.GetN()
                      << " UT nodes; expected " << expectedUts
                      << " (=25 beams x " << starlinkBeamSatIds.size()
                      << " ROI helper sats)");
        std::cout << "[starlink25] " << utNodes.GetN()
                  << " UTs placed at Tokyo ROI beam centers across sats "
                  << FormatHelperSatIds(starlinkBeamSatIds) << ".\n";
    }

    Singleton<SatTopology>::Get()->PrintTopology(std::cout);

    // ── Step 3: traffic setup ─────────────────────────────────────────────
    // starlink25: 5:1 hotspot demand ratio.
    //   FWD hotspot (beamId {1,4,13,19,22}): 20ms × 1500B ≈ 600 kbps/UT
    //   FWD non-hotspot (remaining 20 beams): 100ms × 1500B ≈ 120 kbps/UT
    //   RTN baseline: 100ms × 512B (symmetric; Phase F adds 100kbps demand in Step 4b)
    // Other scenarios: uniform CBR, same as original.

    {
        NodeContainer allUts     = Singleton<SatTopology>::Get()->GetUtNodes();
        NodeContainer allUtUsers = Singleton<SatTopology>::Get()->GetUtUserNodes();
        NodeContainer gwUserNode0;
        gwUserNode0.Add(Singleton<SatTopology>::Get()->GetGwUserNode(0));

        if (scenario == "starlink25")
        {
            // Phase F uniform load: 20 Mbps FWD per UT, no hotspot differentiation.
            // 1500 bytes / 600 μs = 20.000 Mbps per UT.
            // Phase G DynamicBstp receives uniform RBDC → round-robin beam selection.
            simHelper->GetTrafficHelper()->AddCbrTraffic(
                SatTrafficHelper::FWD_LINK, SatTrafficHelper::UDP,
                MicroSeconds(600), 1500,
                gwUserNode0, allUtUsers,
                Seconds(cfg.warmUpSec), Seconds(cfg.simTimeSec), Seconds(0));

            // RTN baseline CBR (Phase F Step 4b adds +100 kbps RBDC demand on top)
            simHelper->GetTrafficHelper()->AddCbrTraffic(
                SatTrafficHelper::RTN_LINK, SatTrafficHelper::UDP,
                MilliSeconds(100), 512,
                gwUserNode0, allUtUsers,
                Seconds(cfg.warmUpSec), Seconds(cfg.simTimeSec), Seconds(0));

            NS_LOG_INFO("[starlink25/PhaseF] FWD uniform 20 Mbps x "
                        << allUtUsers.GetN() << " UTs"
                        << "  RTN baseline ~41 kbps x " << allUtUsers.GetN() << " UTs");
        }
        else
        {
            // Uniform CBR for leo2sat / iridium-next scenarios (unchanged behavior)
            simHelper->GetTrafficHelper()->AddCbrTraffic(
                SatTrafficHelper::FWD_LINK, SatTrafficHelper::UDP,
                MilliSeconds(20), 1500,
                gwUserNode0, allUtUsers,
                Seconds(cfg.warmUpSec), Seconds(cfg.simTimeSec), Seconds(0));

            simHelper->GetTrafficHelper()->AddCbrTraffic(
                SatTrafficHelper::RTN_LINK, SatTrafficHelper::UDP,
                MilliSeconds(100), 512,
                gwUserNode0, allUtUsers,
                Seconds(cfg.warmUpSec), Seconds(cfg.simTimeSec), Seconds(0));
        }
    }

    // ── Step 3.5: FWD PDR tracking + ASCII trace (starlink25 + Phase F) ─────
    //
    // Opens sat-bh-phaseF-fwd.tr (AsciiTraceHelper stream) and wires
    // PacketSink::Rx on each UT user node so every received FWD packet
    // writes one ASCII line:  "time  r  /NodeList/UTi/...  bytes=N  beam=B"
    //
    // Ordering invariant: GetUtUserNodes()[i] corresponds to beamId = i+1.
    // This holds when starlink25 CreateSatScenario places exactly 1 UT per beam
    // in ascending beamId order (asserted in Step 2.5: utNodes.GetN()==25).
    //
    // PacketSink is installed by SatTrafficHelper::AddCbrTraffic (FWD sink at
    // UT user nodes).  We find it via DynamicCast<PacketSink> on each node's
    // application list.  If the cast fails (SNS3 uses a subclass), hooked count
    // will be 0 and a WARN is emitted — PDR table will show 0 bytes.
    if (scenario == "starlink25")
    {
        AsciiTraceHelper ascii;
        std::filesystem::path fwdTracePath("sat-bh-phaseF-fwd.tr");
        if (!outputDir.empty())
        {
            fwdTracePath = std::filesystem::path(outputDir) / fwdTracePath;
        }
        g_fwdAsciiStream = ascii.CreateFileStream(fwdTracePath.string());
        *g_fwdAsciiStream->GetStream()
            << "# Phase F FWD PDR trace  scenario=starlink25  K="
            << cfg.maxActiveBeams << "  offered="
            << (cfg.fwdOfferedDemandKbps / 1000.0) << "Mbps/UT\n"
            << "# columns: time[s]  event  path  bytes=N  beam=B\n";

        NodeContainer utUserAll = Singleton<SatTopology>::Get()->GetUtUserNodes();
        g_beamPdr.resize(cfg.numBeams);
        g_beamRxBytesByPeriod.clear();
        g_actualDemandWarmUpSec = cfg.warmUpSec;
        g_actualDemandPeriodSec = cfg.bhtpPeriodMs / 1000.0;
        for (uint32_t beamIdx = 0; beamIdx < g_beamPdr.size(); ++beamIdx)
        {
            g_beamPdr[beamIdx].beamId = beamIdx + 1;
        }

        uint32_t hooked{0};
        for (uint32_t i = 0; i < utUserAll.GetN(); ++i)
        {
            Ptr<Node> utUser = utUserAll.Get(i);
            for (uint32_t j = 0; j < utUser->GetNApplications(); ++j)
            {
                Ptr<PacketSink> sink =
                    DynamicCast<PacketSink>(utUser->GetApplication(j));
                if (sink)
                {
                    // Bind utIdx so FwdRxTrace can index g_beamPdr correctly
                    sink->TraceConnectWithoutContext(
                        "Rx", MakeBoundCallback(&FwdRxTrace, i));
                    ++hooked;
                    break;
                }
            }
        }

        if (hooked == 0)
            NS_LOG_WARN("[PhaseF/PDR] No PacketSink found on UT user nodes"
                        " — PDR table will show 0 bytes."
                        " SatTrafficHelper may use a non-PacketSink app.");
        else
            NS_LOG_INFO("[PhaseF/PDR] ASCII trace -> sat-bh-phaseF-fwd.tr"
                        "  path=" << fwdTracePath.string()
                        << "  hooked=" << hooked << "/" << utUserAll.GetN() << " UTs");
    }

    // ── Step 4: BH system installation ───────────────────────────────────
    //
    // One SatBhHelper per satellite (66 total).
    // numBeams=2: Iridium-66 has max 2 beams/sat (72 beams / 66 sats).
    // Satellites with only 1 active beam: the 2nd beam has zero UT demand
    // → scheduler assigns it as non-hotspot with minimal slot time (no impact).
    //
    // Handover across satellites is handled transparently:
    // PollUtStates() filters by utSatId each frame, so when SNS3 handover fires,
    // the old satellite's helper drops the UT and the new one picks it up.
    //
    // cfg.numSats / cfg.numBeams already set by scenario block above

    // ── Build run-specific output paths ──────────────────────────────────
    // Format: {base}_{scenario}_{YYYYMMDD_HHMMSS}.csv
    // All satellites share ONE metrics file (differentiated by sat_id column).
    // Each run creates a new file, so previous results are never overwritten.
    {
        auto wallNow = std::time(nullptr);
        std::tm* tmInfo = std::localtime(&wallNow);
        std::ostringstream ts;
        ts << std::put_time(tmInfo, "%Y%m%d_%H%M%S");
        std::string runTag = scenario + "_" + ts.str();

        auto withRunTag = [&runTag](const std::string& s, const std::string& defaultExt) {
            std::filesystem::path p(s);
            std::string stem = p.stem().string();
            std::string ext = p.extension().empty() ? defaultExt : p.extension().string();
            p.replace_filename(stem + "_" + runTag + ext);
            return p;
        };

        std::filesystem::path outRoot(outputDir);
        std::filesystem::path metricsPath = withRunTag(cfg.metricsOutputFile, ".csv");
        std::filesystem::path timePlanPath = withRunTag(cfg.timePlanCsvFile, ".csv");
        std::filesystem::path trafficPath = withRunTag(cfg.trafficTraceFile, ".tr");

        if (!outputDir.empty())
        {
            if (metricsPath.is_relative())
                metricsPath = outRoot / metricsPath;
            if (timePlanPath.is_relative())
                timePlanPath = outRoot / timePlanPath;
            if (trafficPath.is_relative())
                trafficPath = outRoot / trafficPath;
        }

        cfg.metricsOutputFile = metricsPath.string();
        cfg.timePlanCsvFile   = timePlanPath.string();
        cfg.trafficTraceFile  = trafficPath.string();

        std::string trafficStem = trafficPath.stem().string();
        const std::string trafficPrefix = "sat-bh-traffic_";
        std::string actualStem =
            (trafficStem.rfind(trafficPrefix, 0) == 0)
                ? ("bh-demand-actual_" + trafficStem.substr(trafficPrefix.size()))
                : (trafficStem + "-actual-demand");
        actualDemandPath = trafficPath.parent_path() / (actualStem + ".csv");
    }

    std::string base = cfg.metricsOutputFile;  // already has extension
    if (!outputDir.empty())
    {
        std::filesystem::path statsPath = std::filesystem::path(outputDir) / "sns3-stats";
        std::error_code ec;
        std::filesystem::create_directories(statsPath, ec);
        NS_ABORT_MSG_IF(ec, "Cannot create SNS3 stats output path="
                                << statsPath.string() << ": " << ec.message());
        simHelper->SetOutputPath(statsPath.string());
    }

    // ── Determine helper installation set ────────────────────────────────
    // For starlink25, reuse the same ROI-resolved satellite set that received
    // BeamUserInfo above, so BH helpers and trace rows match the real devices.
    // Other scenarios keep the original contiguous helper-window behavior.
    std::vector<uint32_t> helperSatIds =
        (scenario == "starlink25") ? starlinkBeamSatIds : ParseHelperSatList(helperSatList, numSats);

    if (helperSatIds.empty())
    {
        uint32_t helperStart{0};
        uint32_t helperCount{numSats};

        if (scenario == "starlink25" && cfg.maxHelperSats > 0)
        {
            helperStart = std::min(cfg.satIdStart, numSats - 1);
            helperCount = std::min(cfg.maxHelperSats, numSats - helperStart);
            std::cout << "[starlink25] ROI filter: installing " << helperCount
                      << " BH helpers  satId=[" << helperStart
                      << "," << (helperStart + helperCount) << ")"
                      << "  (minElevDeg=" << cfg.minElevDeg << "° reference)\n";
        }

        for (uint32_t i = 0; i < helperCount; i++)
        {
            helperSatIds.push_back(helperStart + i);
        }
    }
    else
    {
        std::cout << "[starlink25] helperSatList: installing "
                  << helperSatIds.size()
                  << " BH helpers satIds="
                  << FormatHelperSatIds(helperSatIds)
                  << "  (overrides satIdStart/maxHelperSats)\n";
    }

    std::vector<Ptr<SatBhHelper>> bhHelpers(helperSatIds.size());
    for (uint32_t i = 0; i < helperSatIds.size(); i++)
    {
        uint32_t satId = helperSatIds[i];

        bhHelpers[i] = CreateObject<SatBhHelper>();
        bhHelpers[i]->Configure(cfg);
        bhHelpers[i]->SetNumBeams(cfg.numBeams);
        bhHelpers[i]->SetSatId(satId);
        // All satellites write to the same timestamped file; sat_id column
        // distinguishes each satellite's rows within the file.
        bhHelpers[i]->SetMetricsOutputFile(base);

        // Phase E and Phase F both require SetSimulationHelper() to access SatNcc.
        if (cfg.enablePhaseE || cfg.enablePhaseF)
            bhHelpers[i]->SetSimulationHelper(simHelper);

        bhHelpers[i]->Install();
    }

    // ── Step 4b: RTN link traffic (Phase F: required to trigger DAMA CR / BacklogRequestsTrace)
    // Adds a CBR stream from every UT to the GW on the return link.
    // Without this, UTs never send Capacity Requests and BacklogRequestsTrace never fires,
    // so the Phase F demand cache stays empty.
    if (cfg.enablePhaseF)
    {
        // AddCbrTraffic(direction, protocol, interval, packetSize, gwUsers, utUsers,
        //               startTime, stopTime, startDelay)
        // 100 kbps CBR: 1000 bytes / 80 ms = 100 kbps per UT.
        // This causes UTs to send RBDC capacity requests (CR), which triggers
        // SatBeamScheduler::BacklogRequestsTrace and populates the Phase F demand cache.
        NodeContainer uts     = Singleton<SatTopology>::Get()->GetUtNodes();
        NodeContainer gwUsers = Singleton<SatTopology>::Get()->GetGwUserNodes();
        NodeContainer utUsers = Singleton<SatTopology>::Get()->GetUtUserNodes(uts);

        Ptr<SatTrafficHelper> trafficHelper = simHelper->GetTrafficHelper();
        trafficHelper->AddCbrTraffic(SatTrafficHelper::RTN_LINK,
                                     SatTrafficHelper::UDP,
                                     MilliSeconds(80), // 1000 B / 80 ms = 100 kbps
                                     1000,             // packet size [bytes]
                                     gwUsers,
                                     utUsers,
                                     Seconds(1.0),
                                     Seconds(cfg.simTimeSec),
                                     MilliSeconds(10));
        NS_LOG_INFO("[PhaseF] RTN CBR traffic added: 100 kbps per UT"
                    << " utCount=" << utUsers.GetN()
                    << " simTime=" << cfg.simTimeSec << "s");
    }

    // ── Step 5: SNS3 statistics (built-in throughput / delay collectors) ─
    Ptr<SatStatsHelperContainer> stats = simHelper->GetStatisticsContainer();

    stats->AddPerBeamFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
    stats->AddPerBeamFwdUserDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
    stats->AddPerBeamRtnAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
    stats->AddPerBeamBeamServiceTime(SatStatsHelper::OUTPUT_SCALAR_FILE);
    stats->AddPerSatFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
    stats->AddPerSatRtnAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
    // Phase F PDR: per-UT FWD throughput (cross-check vs. g_beamPdr rxBytes)
    // PER complements PDR: PDR = 1 − PER (per-beam scalar output)
    stats->AddPerUtFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
    stats->AddPerBeamFwdUserDaPacketError(SatStatsHelper::OUTPUT_SCALAR_FILE);

    // ── Step 6: save attributes snapshot ──────────────────────────────────
    std::filesystem::path attributesPath("bh-attributes.xml");
    if (!outputDir.empty())
    {
        attributesPath = std::filesystem::path(outputDir) / attributesPath;
    }
    Config::SetDefault("ns3::ConfigStore::Filename",   StringValue(attributesPath.string()));
    Config::SetDefault("ns3::ConfigStore::FileFormat", StringValue("Xml"));
    Config::SetDefault("ns3::ConfigStore::Mode",       StringValue("Save"));
    ConfigStore outputConfig;
    outputConfig.ConfigureDefaults();

    // ── Step 7: run simulation ─────────────────────────────────────────────
    std::cout << "\n[BH Example] Starting simulation"
              << "  scenario=" << scenario
              << "  time=" << cfg.simTimeSec << "s"
              << "  warmUp=" << cfg.warmUpSec << "s"
              << "  K=" << cfg.maxActiveBeams
              << "  beams=" << cfg.numBeams
              << "\n  HelperSats=" << FormatHelperSatIds(helperSatIds)
              << "  ROI=(" << cfg.roiLat << "N," << cfg.roiLon << "E)"
              << "  minElev=" << cfg.minElevDeg << "deg"
              << "\n  Phase2(Scheduler=" << cfg.enableScheduler
              << " OBC=" << cfg.enableObc << ")"
              << "\n  Phase3(CacheQueue=" << cfg.enableCacheQueue
              << " MMSE=" << cfg.enablePrecoder << ")"
              << "\n  PhaseC(ResourceManager=" << cfg.enableResourceManager
              << " UserAssoc=" << cfg.enableUserAssociation
              << " SchedMode=" << static_cast<int>(cfg.schedulingMode) << ")"
              << "\n  PhaseD(PowerAlloc=" << cfg.enablePowerAllocation
              << " P_total=" << cfg.totalPowerBudgetDbm << "dBm"
              << " N0=" << cfg.noisePowerDbw << "dBW"
              << " ICI=" << cfg.interferenceFactor << ")"
              << "\n  PhaseE(RealSNS3Wiring=" << cfg.enablePhaseE << ")"
              << "\n  PhaseF(RealDAMADemand=" << cfg.enablePhaseF << ")"
              << "\n  PhaseG(DynamicBstp=" << cfg.enableDynamicBstp
              << " demandW=" << cfg.bhDemandBacklogWeight
              << " fairW=" << cfg.bhFairnessWeight
              << " validitySF=" << cfg.bhValiditySuperframes
              << " starvThr=" << cfg.bhStarvationThreshold << ")\n\n";

    simHelper->EnableProgressLogs();
    simHelper->RunSimulation();

    // ── Step 8: final KPI flush ────────────────────────────────────────────
    // Flush remaining data for all 66 satellite helpers
    for (auto& h : bhHelpers)
        h->GetMetrics()->FinalFlush();

    if (scenario == "starlink25" &&
        !g_beamPdr.empty() &&
        g_actualDemandPeriodSec > 0.0)
    {
        uint32_t totalPeriods = 0;
        for (double t = cfg.warmUpSec; t < cfg.simTimeSec - 1e-9;
             t += g_actualDemandPeriodSec)
        {
            ++totalPeriods;
        }
        totalPeriods = std::max<uint32_t>(
            totalPeriods, static_cast<uint32_t>(g_beamRxBytesByPeriod.size()));

        std::set<uint32_t> hotspotBeams(cfg.bhFwdHotspotBeamIds.begin(),
                                        cfg.bhFwdHotspotBeamIds.end());
        const double roiSatFactor =
            static_cast<double>(std::max<std::size_t>(1, starlinkBeamSatIds.size()));

        std::ofstream actualDemand(actualDemandPath);
        NS_ABORT_MSG_IF(!actualDemand.is_open(),
                        "Cannot open actual demand output file: "
                            << actualDemandPath.string());
        actualDemand << "period_idx,period_start_s,period_end_s,beam_id,"
                     << "offered_kbps,rx_bytes,served_kbps,"
                     << "remaining_kbps,remaining_pct\n";

        for (uint32_t periodIdx = 0; periodIdx < totalPeriods; ++periodIdx)
        {
            const double periodStart =
                cfg.warmUpSec + periodIdx * g_actualDemandPeriodSec;
            const double periodEnd = periodStart + g_actualDemandPeriodSec;

            for (uint32_t beamIdx = 0; beamIdx < g_beamPdr.size(); ++beamIdx)
            {
                const uint32_t beamId = g_beamPdr[beamIdx].beamId;
                double offeredKbps = cfg.fwdOfferedDemandKbps;
                if (hotspotBeams.find(beamId) != hotspotBeams.end())
                {
                    offeredKbps += cfg.bhFwdHotspotBoostKbps;
                }
                offeredKbps *= roiSatFactor;

                uint64_t rxBytes = 0;
                if (periodIdx < g_beamRxBytesByPeriod.size() &&
                    beamIdx < g_beamRxBytesByPeriod[periodIdx].size())
                {
                    rxBytes = g_beamRxBytesByPeriod[periodIdx][beamIdx];
                }

                const double servedKbps =
                    static_cast<double>(rxBytes) * 8.0 / 1000.0 /
                    g_actualDemandPeriodSec;
                const double remainingKbps = std::max(0.0, offeredKbps - servedKbps);
                const double remainingPct =
                    offeredKbps > 0.0 ? remainingKbps / offeredKbps * 100.0 : 0.0;

                actualDemand << periodIdx << ","
                             << std::fixed << std::setprecision(6)
                             << periodStart << ","
                             << periodEnd << ","
                             << beamId << ","
                             << std::setprecision(3)
                             << offeredKbps << ","
                             << rxBytes << ","
                             << servedKbps << ","
                             << remainingKbps << ","
                             << remainingPct << "\n";
            }
        }
    }

    // ── Step 9: summary ───────────────────────────────────────────────────
    std::cout << "\n[BH Example] Simulation complete.\n"
              << "  KPI metrics  : " << cfg.metricsOutputFile  << "\n"
              << "  BHTP table   : " << cfg.timePlanCsvFile    << "\n"
              << "  BH traffic   : " << cfg.trafficTraceFile   << "\n"
              << "  Actual demand: "
              << (scenario == "starlink25" ? actualDemandPath.string()
                                            : std::string("(not generated)"))
              << "\n"
              << "  SNS3 stats   : "
              << (outputDir.empty()
                      ? std::string("data/sims/sat-bh-example/")
                      : (std::filesystem::path(outputDir) / "sns3-stats").string())
              << "\n"
              << "  Attributes   : " << attributesPath.string() << "\n\n";

    // ── Step 9b: Phase F FWD per-beam PDR table ────────────────────────────
    //
    // offeredBytes = 20 Mbps × measuredSec (warmup already excluded: traffic
    // starts at warmUpSec, so GetTotalRx() only counts post-warmup bytes).
    // PDR per beam = rxBytes / offeredBytes × 100.
    // If PDR > 100% it means the link completed more bytes than the offered
    // nominal (e.g., retransmissions or timer artefacts) — clamped to 100%.
    if (scenario == "starlink25" && !g_beamPdr.empty())
    {
        const double measuredSec       = cfg.simTimeSec - cfg.warmUpSec;
        const double offeredBytesPerUt =
            (cfg.fwdOfferedDemandKbps * 1000.0 / 8.0) * measuredSec;
        const double offeredBytesPerBeam =
            offeredBytesPerUt * std::max<std::size_t>(1, starlinkBeamSatIds.size());

        std::cout << "[PhaseF] FWD Per-Beam PDR"
                  << "  offered=" << (cfg.fwdOfferedDemandKbps / 1000.0)
                  << " Mbps/UT  roiSats=" << starlinkBeamSatIds.size()
                  << "  K=" << cfg.maxActiveBeams
                  << "  measured=" << measuredSec << "s\n"
                  << std::left
                  << std::setw(8)  << "BeamID"
                  << std::setw(14) << "RxBytes"
                  << std::setw(14) << "OfferedMB"
                  << "PDR[%]\n"
                  << std::string(46, '-') << "\n";

        double totalRx{0.0}, totalOff{0.0};
        for (const auto& e : g_beamPdr)
        {
            double pdr = (offeredBytesPerBeam > 0.0)
                         ? std::min(100.0, e.rxBytes / offeredBytesPerBeam * 100.0)
                         : 0.0;
            std::cout << std::setw(8)  << e.beamId
                      << std::setw(14) << e.rxBytes
                      << std::setw(14) << std::fixed << std::setprecision(2)
                                       << (offeredBytesPerBeam / 1.0e6)
                      << std::fixed << std::setprecision(1) << pdr << "%\n";
            totalRx  += static_cast<double>(e.rxBytes);
            totalOff += offeredBytesPerBeam;
        }
        double globalPdr = (totalOff > 0.0)
                           ? std::min(100.0, totalRx / totalOff * 100.0)
                           : 0.0;
        std::cout << std::string(46, '-') << "\n"
                  << "  Global FWD PDR : "
                  << std::fixed << std::setprecision(1) << globalPdr << "%\n"
                  << "  Trace file     : "
                  << (outputDir.empty()
                          ? std::string("sat-bh-phaseF-fwd.tr")
                          : (std::filesystem::path(outputDir) / "sat-bh-phaseF-fwd.tr").string())
                  << "\n"
                  << "  SNS3 cross-ref : data/*fwd-app-throughput*  "
                     "data/*fwd-packet-error*\n\n";
    }

    return 0;
}
