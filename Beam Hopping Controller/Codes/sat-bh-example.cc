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
 *   ./ns3 run "sat-bh-example --enableScheduler=true --enableObc=true"
 *
 * Phase 3 (enable CacheQueue + MMSE stubs):
 *   ./ns3 run "sat-bh-example --enableScheduler=true --enableObc=true \
 *                             --enableCacheQueue=true --enablePrecoder=true"
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
 *   bh-metrics.csv        — KPI rows per T_p = 503 ms (SatBhMetrics CSV)
 *   bh-timeplan.csv       — BHTP slot table (SatBhTimePlan ToCsv)
 *   bh-attributes.xml     — ns-3 ConfigStore attribute snapshot
 */

#include "ns3/sat-bh-helper.h"

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("sat-bh-example");

// ── ParseConfig: command-line → BhExperimentConfig ────────────────────────
//
// All simulation knobs are exposed as CommandLine arguments.
// Adding a new knob in Phase 2/3 means adding a line here and in
// BhExperimentConfig — no structural change to the rest of the file.
//
static BhExperimentConfig
ParseConfig(int argc, char* argv[])
{
    BhExperimentConfig cfg;

    CommandLine cmd;

    // ── Scenario ──────────────────────────────────────────────────────────
    cmd.AddValue("simTime",        "Simulation duration [s]",             cfg.simTimeSec);
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

    // ── Output ────────────────────────────────────────────────────────────
    cmd.AddValue("metricsFile",  "KPI CSV output file",       cfg.metricsOutputFile);
    cmd.AddValue("timePlanFile", "BHTP slot table CSV file",  cfg.timePlanCsvFile);

    cmd.Parse(argc, argv);
    return cfg;
}

// ── main ──────────────────────────────────────────────────────────────────

int
main(int argc, char* argv[])
{
    // ── Step 0: parse experiment configuration ────────────────────────────
    BhExperimentConfig cfg = ParseConfig(argc, argv);

    // ── Step 1: SNS3 global configuration ────────────────────────────────
    // Regeneration mode: consistent with sat-gw-handover-example.cc reference
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));

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

    // Restrict active beams to the set used by the BH scenario
    // Basic scenario (spec Section 12.1): beams 1..numBeams
    std::set<uint32_t> beamSet;
    for (uint32_t i = 1; i <= cfg.numBeams; i++)
        beamSet.insert(i);
    simHelper->SetBeamSet(beamSet);
    simHelper->SetUserCountPerMobileUt(simConf->m_utMobileUserCount);

    // 2-satellite LEO constellation (closest available to 1-sat basic scenario)
    simHelper->LoadScenario("constellation-leo-2-satellites");
    simHelper->CreateSatScenario(SatHelper::NONE);

    Singleton<SatTopology>::Get()->PrintTopology(std::cout);

    // ── Step 3: traffic setup ─────────────────────────────────────────────
    // FWD link CBR: GW user → all UT users (hotspot: high rate, non-hotspot: low)
    // Traffic starts after warm-up to generate meaningful post-warmup KPIs

    // Hotspot traffic: higher rate (shorter interval)
    simHelper->GetTrafficHelper()->AddCbrTraffic(
        SatTrafficHelper::FWD_LINK,
        SatTrafficHelper::UDP,
        MilliSeconds(20),           // ~50 pkt/s (hotspot approximation)
        1500,                       // bytes
        NodeContainer(Singleton<SatTopology>::Get()->GetGwUserNode(0)),
        Singleton<SatTopology>::Get()->GetUtUserNodes(),
        Seconds(cfg.warmUpSec),     // start after warm-up
        Seconds(cfg.simTimeSec),
        Seconds(0));

    // Return link CBR (for completeness / symmetric scenario)
    simHelper->GetTrafficHelper()->AddCbrTraffic(
        SatTrafficHelper::RTN_LINK,
        SatTrafficHelper::UDP,
        MilliSeconds(100),          // ~10 pkt/s
        512,
        NodeContainer(Singleton<SatTopology>::Get()->GetGwUserNode(0)),
        Singleton<SatTopology>::Get()->GetUtUserNodes(),
        Seconds(cfg.warmUpSec),
        Seconds(cfg.simTimeSec),
        Seconds(0));

    // ── Step 4: BH system installation (the ONLY BH API call in main) ────
    //
    // SatBhHelper internally:
    //   Phase 1 (always): builds static BHTP, creates SatBhMetrics, starts
    //                     synthetic slot driver if OBC not enabled
    //   Phase 2 (opt-in): creates SatBhScheduler + SatBhObc (stubs for now)
    //   Phase 3 (opt-in): creates SatGwCacheQueue + SatBhPrecoder (stubs)
    //
    Ptr<SatBhHelper> bhHelper = CreateObject<SatBhHelper>();
    bhHelper->Configure(cfg);
    bhHelper->Install();

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
              << "  time=" << cfg.simTimeSec << "s"
              << "  warmUp=" << cfg.warmUpSec << "s"
              << "  K=" << cfg.maxActiveBeams
              << "  beams=" << cfg.numBeams
              << "\n  Phase2(Scheduler=" << cfg.enableScheduler
              << " OBC=" << cfg.enableObc << ")"
              << "\n  Phase3(CacheQueue=" << cfg.enableCacheQueue
              << " MMSE=" << cfg.enablePrecoder << ")\n\n";

    simHelper->EnableProgressLogs();
    simHelper->RunSimulation();

    // ── Step 8: final KPI flush ────────────────────────────────────────────
    // Flush any remaining data not yet written by the periodic timer
    bhHelper->GetMetrics()->FinalFlush();

    // ── Step 9: summary ───────────────────────────────────────────────────
    std::cout << "\n[BH Example] Simulation complete.\n"
              << "  KPI metrics  : " << cfg.metricsOutputFile  << "\n"
              << "  BHTP table   : " << cfg.timePlanCsvFile    << "\n"
              << "  SNS3 stats   : data/\n"
              << "  Attributes   : bh-attributes.xml\n\n";

    return 0;
}
