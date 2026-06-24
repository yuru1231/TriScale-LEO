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
 *   bh-metrics.csv        — KPI rows per T_p = 80 ms (SatBhMetrics CSV)
 *   bh-timeplan.csv       — BHTP slot table (SatBhTimePlan ToCsv)
 *   bh-attributes.xml     — ns-3 ConfigStore attribute snapshot
 */

#include "ns3/sat-bh-helper.h"
#include "ns3/sat-constellation-params.h"

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("sat-bh-example");

// ── ParseConfig: command-line → BhExperimentConfig ────────────────────────
//
// All simulation knobs are exposed as CommandLine arguments.
// Adding a new knob in Phase 2/3 means adding a line here and in
// BhExperimentConfig — no structural change to the rest of the file.
//
static BhExperimentConfig
ParseConfig(int argc, char* argv[], std::string& scenario, uint32_t& numSats)
{
    BhExperimentConfig cfg;

    CommandLine cmd;

    // ── Scenario selector (example-level, not stored in BhExperimentConfig) ──
    cmd.AddValue("scenario", "leo2sat: scheduling verify | iridium-next: Iridium NEXT 66-sat 72-beam system",
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

    // ── Output ────────────────────────────────────────────────────────────
    cmd.AddValue("metricsFile",  "KPI CSV output file",       cfg.metricsOutputFile);
    cmd.AddValue("timePlanFile", "BHTP slot table CSV file",  cfg.timePlanCsvFile);

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
    std::string scenario{"leo2sat"};  // "leo2sat" | "iridium-next"
    uint32_t    numSats{1};

    BhExperimentConfig cfg = ParseConfig(argc, argv, scenario, numSats);

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

        // Keep the complete constellation available for helper installation,
        // but create the 25 user beams only on the satellite selected by the
        // orbit scan.  Applying beam IDs 1..25 globally makes SNS3 allocate
        // overlapping beam networks across the 1584 satellites.
        numSats = 1584;
        cfg.numBeams       = 25;
        cfg.maxActiveBeams = std::min(cfg.maxActiveBeams, 25u);

        // Default 5 hotspot beams (cell_idx {0,3,12,18,21} = beamId {1,4,13,19,22})
        if (cfg.numHotspotBeams == 0 || cfg.numHotspotBeams > 25)
            cfg.numHotspotBeams = 5;

        // Phase 1 fixed hot cells — 0-indexed cell positions in the 5×5 grid.
        // cell_idx 2 (row0-mid), 10 (row2-left), 12 (centre), 18 (row3-right), 21 (row4-left)
        // → beamId 3, 11, 13, 19, 22.  K_b=2: each slot pairs 1 hotspot + 1 non-hotspot.
        // Only set when not already overridden from the command line.
        if (cfg.hotCellIndices.empty())
            cfg.hotCellIndices = {2, 10, 12, 18, 21};

        // Synthetic FWD demand injection for Phase G greedy provider.
        //
        // Phase F RBDC (RTN demand) is uniform across all UTs (~100 kbps each),
        // so the greedy provider cannot distinguish the FWD 5:1 hotspot ratio
        // from RBDC alone.  Here we inject a synthetic demand boost for the true
        // traffic hotspot beams {1,4,13,19,22} when Phase G is active.
        //
        // Boost = FWD hotspot rate − FWD baseline = 600 − 120 = 480 kbps.
        // Provider sees: hotspot beams ≈ 580 kbps (480 boost + ~100 RBDC),
        //                non-hotspot   ≈ 100 kbps (RBDC only).
        // → 5.8:1 demand ratio, matching the intended traffic asymmetry.
        if (cfg.enableDynamicBstp)
        {
            cfg.bhFwdHotspotBeamIds   = {1, 4, 13, 19, 22};
            cfg.bhFwdHotspotBoostKbps = 480.0;
        }

        SatHelper::BeamUserInfoMap_t beamInfo;
        for (uint32_t i = 0; i < 25; ++i)
        {
            const uint32_t beamId = i + 1;
            SatBeamUserInfo info;
            info.SetPositions(
                {std::make_pair(GeoCoordinate(kCellLat[i], kCellLon[i], 0.0), 0)});
            info.AppendUt(1);
            beamInfo.emplace(std::make_pair(kPeakSatId, beamId), info);
            beamSet.insert(beamId);
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
                    << " (maxHelperSats=" << cfg.maxHelperSats << ")");

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
        NS_ASSERT_MSG(utNodes.GetN() == 25,
                      "[starlink25] scenario created " << utNodes.GetN()
                      << " UT nodes; expected exactly 25");
        std::cout << "[starlink25] 25 UTs placed at Tokyo peak-elevation cell centers.\n";
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
            // Hotspot beamIds: cell_idx {0,3,12,18,21} + 1 → {1,4,13,19,22}
            // UT ordering assumption: GetUtNodes()[i] serves beamId i+1.
            // This holds when CreateSatScenario allocates exactly 1 UT per beam in order.
            // Verify at runtime: check that UT count == 25 (asserted in Step 2.5 above).
            const std::set<uint32_t> kHotspotBeams = {1, 4, 13, 19, 22};

            NodeContainer hotUtUsers, coldUtUsers;
            for (uint32_t i = 0; i < 25 && i < allUts.GetN(); ++i)
            {
                uint32_t beamId = i + 1; // cell_idx i → beamId i+1
                NodeContainer singleUt;
                singleUt.Add(allUts.Get(i));
                NodeContainer users =
                    Singleton<SatTopology>::Get()->GetUtUserNodes(singleUt);
                for (uint32_t j = 0; j < users.GetN(); ++j)
                {
                    if (kHotspotBeams.count(beamId))
                        hotUtUsers.Add(users.Get(j));
                    else
                        coldUtUsers.Add(users.Get(j));
                }
            }

            // FWD hotspot: 600 kbps/UT  (5× demand signal for Phase G)
            if (hotUtUsers.GetN() > 0)
            {
                simHelper->GetTrafficHelper()->AddCbrTraffic(
                    SatTrafficHelper::FWD_LINK, SatTrafficHelper::UDP,
                    MilliSeconds(20), 1500,
                    gwUserNode0, hotUtUsers,
                    Seconds(cfg.warmUpSec), Seconds(cfg.simTimeSec), Seconds(0));
            }

            // FWD non-hotspot: 120 kbps/UT  (1× baseline demand)
            if (coldUtUsers.GetN() > 0)
            {
                simHelper->GetTrafficHelper()->AddCbrTraffic(
                    SatTrafficHelper::FWD_LINK, SatTrafficHelper::UDP,
                    MilliSeconds(100), 1500,
                    gwUserNode0, coldUtUsers,
                    Seconds(cfg.warmUpSec), Seconds(cfg.simTimeSec), Seconds(0));
            }

            // RTN baseline CBR (symmetric; Phase F step 4b adds RBDC demand on top)
            simHelper->GetTrafficHelper()->AddCbrTraffic(
                SatTrafficHelper::RTN_LINK, SatTrafficHelper::UDP,
                MilliSeconds(100), 512,
                gwUserNode0, allUtUsers,
                Seconds(cfg.warmUpSec), Seconds(cfg.simTimeSec), Seconds(0));

            NS_LOG_INFO("[starlink25] FWD traffic:"
                        << " hotspot=" << hotUtUsers.GetN() << " UTs@600kbps"
                        << " nonHotspot=" << coldUtUsers.GetN() << " UTs@120kbps"
                        << " RTN=" << allUtUsers.GetN() << " UTs@~41kbps");
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

        auto stripExt = [](const std::string& s) {
            auto pos = s.rfind('.');
            return pos != std::string::npos ? s.substr(0, pos) : s;
        };
        cfg.metricsOutputFile = stripExt(cfg.metricsOutputFile) + "_" + runTag + ".csv";
        cfg.timePlanCsvFile   = stripExt(cfg.timePlanCsvFile)   + "_" + runTag + ".csv";
    }

    std::string base = cfg.metricsOutputFile;  // already has extension

    // ── Determine helper installation window ──────────────────────────────
    // For starlink25: reduce from 1584 helpers to [satIdStart, satIdStart+maxHelperSats).
    // satIdStart is derived from the 2D orbit-sgp4 pre-scan (which satellite is
    // overhead the ROI at the simulation epoch).  maxHelperSats controls window width.
    // For leo2sat / iridium-next: install on all satellites (helperStart=0).
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

    std::vector<Ptr<SatBhHelper>> bhHelpers(helperCount);
    for (uint32_t i = 0; i < helperCount; i++)
    {
        uint32_t satId = helperStart + i;

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

    // ── Step 6: save attributes snapshot ──────────────────────────────────
    Config::SetDefault("ns3::ConfigStore::Filename",   StringValue("bh-attributes.xml"));
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
              << "\n  HelperWindow=[" << helperStart
              << "," << (helperStart + helperCount) << ")"
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

    // ── Step 9: summary ───────────────────────────────────────────────────
    std::cout << "\n[BH Example] Simulation complete.\n"
              << "  KPI metrics  : " << cfg.metricsOutputFile  << "\n"
              << "  BHTP table   : " << cfg.timePlanCsvFile    << "\n"
              << "  SNS3 stats   : data/\n"
              << "  Attributes   : bh-attributes.xml\n\n";

    return 0;
}
