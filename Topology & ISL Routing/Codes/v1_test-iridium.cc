#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"

#include <chrono>

using namespace ns3;

int
main(int argc, char* argv[])
{
    std::string scenarioName = "constellation-iridium-66-sats-fixed";
    std::string islsFilePath =
        "/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/"
        "constellation-iridium-66-sats-fixed/positions/isls.txt";
    double simTime = 630.0;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.Parse(argc, argv);

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
                       TimeValue(Seconds(60)));
    Config::SetDefault("ns3::SatHelper::GwUsers", UintegerValue(1));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    simulationHelper->LoadScenario(scenarioName);
    simulationHelper->SetSimulationTime(Seconds(simTime));
    simulationHelper->SetBeamSet({1});
    simulationHelper->SetUserCountPerUt(1);
    simulationHelper->CreateSatScenario();

    LoadISLDefs(islsFilePath);
    InitOrbiterDevices();

    PrecomputedTables tables;
    PrecomputeAllTables(tables);
    ScheduleRoutingUpdates(tables);

    auto wallStart = std::chrono::steady_clock::now();
    simulationHelper->RunSimulation();
    auto wallEnd = std::chrono::steady_clock::now();

    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(
        wallEnd - wallStart).count();
    std::cout << "Wall time: " << seconds << "s" << std::endl;
    std::cout << "=== Simulation complete ===" << std::endl;

    return 0;
}
