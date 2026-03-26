// v5_test-iridium.cc
// Integrates Layer 1 (ISL routing + FT filter),
//            Layer 2 (Beam Hopping),
//            Layer 3 (QoS, via SNS3 native config).

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
// TODO: verify exact Config key strings against installed SNS3 version.
//
static void
ConfigureQoS()
{
    // ── TODO SNS3_QOS_ATTR ─────────────────────────────────────────────────
    // The exact Config key strings must be verified against your SNS3 version
    // before enabling.  Runtime NS_FATAL occurs if the attribute name is wrong.
    //
    // To find correct names, check:
    //   contrib/satellite/model/satellite-lower-layer-service.h/.cc
    // Look for TypeId().AddAttribute("DaService0...") calls.
    //
    // Common naming variants across SNS3 builds:
    //   "DaService0ConstantServiceRateStream0"   (no underscore before digit)
    //   "DaService0_ConstantServiceRateStream0"  (underscore before digit)
    //
    // Uncomment one block at a time to confirm each attribute name.
    // ──────────────────────────────────────────────────────────────────────

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
    const std::string scenarioName = "constellation-iridium-66-sats-fixed";
    const std::string islsFilePath =
        "/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/"
        + scenarioName + "/positions/isls.txt";

    double   simTime      = 630.0;
    double   slotInterval = 60.0;
    uint32_t numSats      = 66;

    CommandLine cmd;
    cmd.AddValue("simTime",      "Simulation time (s)",           simTime);
    cmd.AddValue("slotInterval", "Routing/BH slot interval (s)",  slotInterval);
    cmd.Parse(argc, argv);

    // ── Global NS3 / SNS3 settings ────────────────────────────────────────
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::PointToPointIslHelper::IslDataRate",
                       DataRateValue(DataRate("10Mb/s")));
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
    Ptr<SimulationHelper> simHelper = CreateObject<SimulationHelper>("test-iridium-v5");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));

    // TODO BH: expand beam set to cover Taiwan once beam IDs are identified.
    // Identify Taiwan-covering beams from scenario data, e.g.:
    //   simHelper->SetBeamSet({beam_tw_1, beam_tw_2, ...});
    // For now, use beam 1 as placeholder.
    simHelper->SetBeamSet({1});

    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    const uint32_t numSlots =
        static_cast<uint32_t>(std::ceil(simTime / slotInterval)) + 1;

    auto wallStart = std::chrono::steady_clock::now();

    // ── Layer 1: ISL Routing ──────────────────────────────────────────────
    Ptr<IslRoutingManager> routingMgr = CreateObject<IslRoutingManager>();
    routingMgr->SetAttribute("NumSatellites",   UintegerValue(numSats));
    routingMgr->SetAttribute("NumTimeSlots",    UintegerValue(numSlots));
    routingMgr->SetAttribute("TimeSlotInterval",DoubleValue(slotInterval));
    routingMgr->SetAttribute("IslMaxDistanceKm",DoubleValue(5000.0));
    routingMgr->SetAttribute("IslsFilePath",    StringValue(islsFilePath));
    routingMgr->SetAttribute("EmaAlpha",        DoubleValue(0.3));
    routingMgr->SetAttribute("ChangeThreshold", DoubleValue(0.1));
    routingMgr->SetAttribute("CooldownSeconds", DoubleValue(slotInterval / 2.0));
    routingMgr->SetAttribute("IslLinkRateBps",  DoubleValue(10.0e6));

    routingMgr->Initialize(islsFilePath);
    routingMgr->PrecomputeAllTables();
    routingMgr->ScheduleRoutingUpdates();

    // ── Layer 1 extension: FT Visibility Filter ───────────────────────────
    Ptr<FtVisibilityFilter> ftFilter = CreateObject<FtVisibilityFilter>();
    ftFilter->SetRoutingManager(routingMgr);
    ftFilter->SetElevationThreshold(5.0); // degrees

    // Contracted FTs (self-specified positions)
    ftFilter->AddFt(0, 25.0,  121.5, "TW-Taipei");      // Taiwan
    ftFilter->AddFt(1, 35.7,  139.7, "JP-Tokyo");       // Japan
    ftFilter->AddFt(2, 37.8, -122.4, "US-SanFrancisco");// US West Coast

    // Contracted pairs: which FT-to-FT connections are allowed to route
    ftFilter->AddContractedPair(0, 1); // Taiwan <-> Japan
    ftFilter->AddContractedPair(0, 2); // Taiwan <-> US

    ftFilter->PrecomputeVisibility();
    ftFilter->PrintVisibilityReport();

    // ── Layer 2: Beam Hopping Manager ─────────────────────────────────────
    Ptr<BeamHoppingManager> bhMgr = CreateObject<BeamHoppingManager>();
    bhMgr->SetAttribute("NumSatellites",         UintegerValue(numSats));
    bhMgr->SetAttribute("SuperframeDurationSec", DoubleValue(0.25));
    bhMgr->SetAttribute("ElevationThresholdDeg", DoubleValue(5.0));

    // Target region: cells in Taiwan
    bhMgr->AddCell(0, 25.0, 121.5, "Taipei");
    bhMgr->AddCell(1, 24.1, 120.7, "Taichung");
    bhMgr->AddCell(2, 22.6, 120.3, "Kaohsiung");
    bhMgr->AddCell(3, 24.8, 121.0, "Hsinchu");

    // Traffic demand: uniform stub (replace with real provider when ready)
    bhMgr->SetDemandProvider(std::make_shared<UniformDemandProvider>(1.0));

    // SNS3 injection callback (stub: logging only)
    // TODO SNS3_BH_INJECT: replace with actual beam switch API call.
    bhMgr->SetBhSwitchCallback(
        [](uint32_t satId, uint32_t cellId, Time t) {
            // std::cout << "[BH] t=" << t.GetSeconds()
            //           << " sat=" << satId << " cell=" << cellId << "\n";
        });

    bhMgr->ComputeBhSchedule(numSlots, slotInterval);
    bhMgr->ScheduleBhUpdates();
    // bhMgr->PrintSchedule(); // uncomment for verbose BH schedule

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
