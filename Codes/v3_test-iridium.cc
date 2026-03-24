#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

using namespace ns3;

int
main(int argc, char* argv[])
{
    std::string scenarioName = "constellation-iridium-66-sats-fixed";
    std::string islsFilePath =
        "/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/"
        "constellation-iridium-66-sats-fixed/positions/isls.txt";

    double simTime = 630.0;
    double slotInterval = 60.0;
    uint32_t numSatellites = 66;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("slotInterval", "Routing update interval in seconds", slotInterval);
    cmd.Parse(argc, argv);

    // ── ns-3 scenario setup ────────────────────────────────────────────────
    Ptr<SimulationHelper> simulationHelper =
        CreateObject<SimulationHelper>("test-iridium");

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
    Config::SetDefault("ns3::SatHelper::GwUsers", UintegerValue(1));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    simulationHelper->LoadScenario(scenarioName);
    simulationHelper->SetSimulationTime(Seconds(simTime));
    simulationHelper->SetBeamSet({1});
    simulationHelper->SetUserCountPerUt(1);
    simulationHelper->CreateSatScenario();

    // ── IslRoutingManager setup ─────────────────────────────────────────────
    uint32_t numSlots =
        static_cast<uint32_t>(std::ceil(simTime / slotInterval)) + 1;

    Ptr<IslRoutingManager> routingMgr = CreateObject<IslRoutingManager>();
    routingMgr->SetAttribute("NumSatellites", UintegerValue(numSatellites));
    routingMgr->SetAttribute("NumTimeSlots", UintegerValue(numSlots));
    routingMgr->SetAttribute("TimeSlotInterval", DoubleValue(slotInterval));
    routingMgr->SetAttribute("IslMaxDistanceKm", DoubleValue(5000.0));

    // Load-aware routing parameters
    routingMgr->SetAttribute("EmaAlpha", DoubleValue(0.3));
    routingMgr->SetAttribute("ChangeThreshold", DoubleValue(0.1));
    routingMgr->SetAttribute("CooldownSeconds", DoubleValue(slotInterval / 2.0));
    routingMgr->SetAttribute("IslLinkRateBps", DoubleValue(10.0e6)); // 10 Mb/s

    // ── Timing breakdown ───────────────────────────────────────────────────
    auto t0 = std::chrono::steady_clock::now();

    routingMgr->Initialize(islsFilePath);
    auto t1 = std::chrono::steady_clock::now();

    routingMgr->PrecomputeAllTables();
    auto t2 = std::chrono::steady_clock::now();

    routingMgr->ScheduleRoutingUpdates();
    auto t3 = std::chrono::steady_clock::now();

    simulationHelper->RunSimulation();
    auto t4 = std::chrono::steady_clock::now();

    auto ms = [](auto a, auto b) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
    };

    std::cout << "Initialize         : " << ms(t0, t1) << " ms\n";
    std::cout << "PrecomputeAllTables: " << ms(t1, t2) << " ms\n";
    std::cout << "ScheduleRoutes     : " << ms(t2, t3) << " ms\n";
    std::cout << "Simulator::Run     : " << ms(t3, t4) << " ms\n";
    std::cout << "Total Wall         : " << ms(t0, t4) / 1000.0 << " s\n";
    std::cout << "=== Simulation complete ===" << std::endl;

    Simulator::Destroy();
    return 0;
}
