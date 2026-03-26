// v5_test-iridium.cc
// Integrates Layer 1 (ISL routing + FT filter),
//            Layer 2 (Beam Hopping, stub),
//            Layer 3 (QoS, via SNS3 native config, pending attribute verification).
//
// Fixed constants (not expected to change per run):
//   scenarioName  = "constellation-iridium-66-sats-fixed"
//   ns3BasePath   = "/home/wenj/workspace/ns-3.43"
//   numSats       = 66
//   FT positions  = Taipei / Tokyo / SanFrancisco (contracted ground stations)
//
// All other tunable parameters are exposed via CommandLine.

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"
#include "ns3/ft-filter.h"
#include "ns3/beam-hopping-manager.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

using namespace ns3;

// ── Layer 3: QoS configuration ────────────────────────────────────────────
// Call before CreateSatScenario().
// Three priority classes:
//   Class A (CRA):  guaranteed constant-rate, highest priority
//   Class B (RBDC): rate-based elastic, mid priority
//   Class C (VBDC): volume-based best-effort, lowest priority
// WFQ is applied by SNS3's SatBeamScheduler among same-class UTs.
//
// TODO SNS3_QOS_ATTR: verify exact Config key strings against installed SNS3
// version before enabling.  Runtime NS_FATAL occurs if the attribute name is
// wrong.  To find correct names:
//   grep -r "AddAttribute" contrib/satellite/model/satellite-lower-layer-service.cc \
//        | grep -i "daservice"
//
static void
ConfigureQoS()
{
    // Class A – Constant Rate Assignment (VoIP / control traffic)
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService0_ConstantAssignmentProvided",
    //                    BooleanValue(true));
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService0_ConstantServiceRateStream0",
    //                    UintegerValue(50)); // 50 kbps guaranteed

    // Class B – Rate Based Dynamic Capacity (streaming)
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService1_RbdcAllowed",
    //                    BooleanValue(true));
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService1_MaximumServiceRateStream0",
    //                    UintegerValue(2000)); // 2 Mbps max

    // Class C – Volume Based Dynamic Capacity (best-effort)
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService2_VbdcAllowed",
    //                    BooleanValue(true));
}

// ── Main ──────────────────────────────────────────────────────────────────

int
main(int argc, char* argv[])
{
    // ── Fixed constants ───────────────────────────────────────────────────
    // These represent the specific deployment environment and contracted
    // ground stations; they are not expected to vary between simulation runs.
    const std::string ns3BasePath   = "/home/wenj/workspace/ns-3.43";
    const std::string scenarioName  = "constellation-iridium-66-sats-fixed";
    const uint32_t    numSats       = 66;

    const std::string islsFilePath  =
        ns3BasePath + "/contrib/satellite/data/scenarios/"
        + scenarioName + "/positions/isls.txt";

    // ── CommandLine parameters ────────────────────────────────────────────
    double   simTime         = 630.0;  // simulation duration (s)
    double   slotInterval    = 60.0;   // routing / BH slot interval (s)
    uint32_t beamId          = 1;      // SNS3 beam ID to activate

    // Layer 1 – ISL routing
    double   islMaxDistKm    = 5000.0; // ISL activation distance threshold (km)
    double   islRateMbps     = 10.0;   // ISL link rate (Mbps)
    double   emaAlpha        = 0.3;    // EMA weight for load-cost update
    double   changeThresh    = 0.1;    // load-cost change ratio to trigger recompute
    double   cooldownRatio   = 0.5;    // cooldownSeconds = slotInterval * cooldownRatio
    double   elevMinDeg      = 5.0;    // minimum elevation angle for FT visibility (deg)

    // Layer 2 – Beam Hopping (stub)
    double   bhSuperframeSec = 0.25;   // BH superframe duration (s)

    CommandLine cmd;
    cmd.AddValue("simTime",        "Simulation duration (s)",                  simTime);
    cmd.AddValue("slotInterval",   "Routing and BH slot interval (s)",         slotInterval);
    cmd.AddValue("beamId",         "SNS3 beam ID to activate",                 beamId);
    cmd.AddValue("islMaxDistKm",   "ISL activation distance threshold (km)",   islMaxDistKm);
    cmd.AddValue("islRateMbps",    "ISL link rate (Mbps)",                     islRateMbps);
    cmd.AddValue("emaAlpha",       "EMA weight for load-cost smoothing",       emaAlpha);
    cmd.AddValue("changeThresh",   "Load-cost change ratio for recompute",     changeThresh);
    cmd.AddValue("cooldownRatio",  "Cooldown = slotInterval * cooldownRatio",  cooldownRatio);
    cmd.AddValue("elevMinDeg",     "Minimum FT elevation angle (deg)",         elevMinDeg);
    cmd.AddValue("bhSuperframeSec","BH superframe duration (s)",               bhSuperframeSec);
    cmd.Parse(argc, argv);

    // ── Derived values ────────────────────────────────────────────────────
    const double   islRateBps  = islRateMbps * 1.0e6;
    const double   cooldownSec = slotInterval * cooldownRatio;
    const uint32_t numSlots    =
        static_cast<uint32_t>(std::ceil(simTime / slotInterval)) + 1;

    // ── Global NS3 / SNS3 settings ────────────────────────────────────────
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::PointToPointIslHelper::IslDataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(islRateBps))));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionEachRequest",
                       BooleanValue(false));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionPeriod",
                       TimeValue(Seconds(slotInterval)));
    Config::SetDefault("ns3::SatHelper::GwUsers",         UintegerValue(1));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    // Layer 3: QoS class configuration (must be before CreateSatScenario)
    ConfigureQoS();

    // ── Scenario setup ────────────────────────────────────────────────────
    Ptr<SimulationHelper> simHelper =
        CreateObject<SimulationHelper>("test-iridium-v5");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));

    // TODO BH: expand beam set to cover Taiwan once beam IDs are identified.
    simHelper->SetBeamSet({beamId});

    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    auto wallStart = std::chrono::steady_clock::now();

    // ── Layer 1: ISL Routing ──────────────────────────────────────────────
    // Must call SetAttribute before Initialize().
    Ptr<IslRoutingManager> routingMgr = CreateObject<IslRoutingManager>();
    routingMgr->SetAttribute("NumSatellites",    UintegerValue(numSats));
    routingMgr->SetAttribute("NumTimeSlots",     UintegerValue(numSlots));
    routingMgr->SetAttribute("TimeSlotInterval", DoubleValue(slotInterval));
    routingMgr->SetAttribute("IslMaxDistanceKm", DoubleValue(islMaxDistKm));
    routingMgr->SetAttribute("IslsFilePath",     StringValue(islsFilePath));
    routingMgr->SetAttribute("EmaAlpha",         DoubleValue(emaAlpha));
    routingMgr->SetAttribute("ChangeThreshold",  DoubleValue(changeThresh));
    routingMgr->SetAttribute("CooldownSeconds",  DoubleValue(cooldownSec));
    routingMgr->SetAttribute("IslLinkRateBps",   DoubleValue(islRateBps));

    routingMgr->Initialize(islsFilePath);
    routingMgr->PrecomputeAllTables();

    // ── Layer 1 verification (offline, before simulation starts) ─────────
    //
    // Req 1 & 2 – Full path output + cross-slot path change detection.
    // Three pairs are chosen to cover different path lengths in the Iridium
    // 66-sat constellation (6 planes × 11 sats):
    //   (0, 32)  – roughly opposite sides of the constellation (~many hops)
    //   (0, 11)  – first satellite in the adjacent plane (cross-plane, 1–2 hops)
    //   (10, 55) – spans both in-plane and cross-plane ISLs
    routingMgr->PrintRouteReport({
        {0u,  32u},
        {0u,  11u},
        {10u, 55u},
    });

    // Req 3 – ISL unavailability: block the first hop on the (0→32) path at
    // slot 0, recompute offline, and verify no stale next-hop survives.
    routingMgr->RunAvoidanceTest(0u, 32u, 0u);

    routingMgr->ScheduleRoutingUpdates();

    // ── Layer 1 extension: FT Visibility Filter ───────────────────────────
    // Contracted FTs: positions are fixed (actual ground station coordinates).
    Ptr<FtVisibilityFilter> ftFilter = CreateObject<FtVisibilityFilter>();
    ftFilter->SetRoutingManager(routingMgr);
    ftFilter->SetElevationThreshold(elevMinDeg);

    ftFilter->AddFt(0, 25.0,  121.5, "TW-Taipei");       // Taiwan
    ftFilter->AddFt(1, 35.7,  139.7, "JP-Tokyo");        // Japan
    ftFilter->AddFt(2, 37.8, -122.4, "US-SanFrancisco"); // US West Coast

    // Contracted pairs: only these FT-to-FT connections may route
    ftFilter->AddContractedPair(0, 1); // Taiwan <-> Japan
    ftFilter->AddContractedPair(0, 2); // Taiwan <-> US

    ftFilter->PrecomputeVisibility();
    ftFilter->PrintVisibilityReport();

    // ── Layer 2: Beam Hopping Manager (stub) ──────────────────────────────
    // BH logic is independent of ISL routing; running here as background stub
    // to verify no interference with Layer 1.
    Ptr<BeamHoppingManager> bhMgr = CreateObject<BeamHoppingManager>();
    bhMgr->SetAttribute("NumSatellites",         UintegerValue(numSats));
    bhMgr->SetAttribute("SuperframeDurationSec", DoubleValue(bhSuperframeSec));
    bhMgr->SetAttribute("ElevationThresholdDeg", DoubleValue(elevMinDeg));

    // Target region: cells in Taiwan
    bhMgr->AddCell(0, 25.0, 121.5, "Taipei");
    bhMgr->AddCell(1, 24.1, 120.7, "Taichung");
    bhMgr->AddCell(2, 22.6, 120.3, "Kaohsiung");
    bhMgr->AddCell(3, 24.8, 121.0, "Hsinchu");

    // Traffic demand: uniform stub (replace with real provider in Layer 2 work)
    bhMgr->SetDemandProvider(std::make_shared<UniformDemandProvider>(1.0));

    // SNS3 injection callback: stub (logging only)
    // TODO SNS3_BH_INJECT: replace with actual beam switch API call.
    bhMgr->SetBhSwitchCallback(
        [](uint32_t satId, uint32_t cellId, Time t) {
            // std::cout << "[BH] t=" << t.GetSeconds()
            //           << " sat=" << satId << " cell=" << cellId << "\n";
        });

    bhMgr->ComputeBhSchedule(numSlots, slotInterval);
    bhMgr->ScheduleBhUpdates();

    // ── Run simulation ────────────────────────────────────────────────────
    simHelper->RunSimulation();

    // ── Post-run stats ────────────────────────────────────────────────────
    routingMgr->PrintStats();

    auto wallEnd = std::chrono::steady_clock::now();
    auto wallMs  = std::chrono::duration_cast<std::chrono::milliseconds>(
        wallEnd - wallStart).count();

    std::cout << "Total wall time: " << wallMs / 1000.0 << " s\n";
    std::cout << "Event count:     " << Simulator::GetEventCount() << "\n";

    Simulator::Destroy();
    return 0;
}