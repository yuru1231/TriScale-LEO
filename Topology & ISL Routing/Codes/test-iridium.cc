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
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

static void
ConfigureQoS()
{
    // 保留空函式，避免不同 SNS3 版本 attribute 名稱不一致時觸發 NS_FATAL。
}

struct GatewayPreset
{
    uint32_t id;
    double latDeg;
    double lonDeg;
    std::string name;
};

const GatewayPreset* FindGatewayPreset(uint32_t gwId)
{
    static const std::vector<GatewayPreset> kPresets = {
        {0, 25.0,  121.5,  "TW-Taipei"},
        {1, 35.7,  139.7,  "JP-Tokyo"},
        {2, 37.8, -122.4,  "US-SanFrancisco"},
    };

    for (const auto& g : kPresets)
    {
        if (g.id == gwId)
        {
            return &g;
        }
    }
    return nullptr;
}

void AddGatewayOrAbort(Ptr<IslRoutingManager> routingMgr, uint32_t gwId)
{
    const GatewayPreset* gw = FindGatewayPreset(gwId);
    NS_ABORT_MSG_IF(gw == nullptr,
                    "Unknown gwId=" << gwId << ". Supported presets: 0(Taipei), 1(Tokyo), 2(SanFrancisco)");
    routingMgr->AddGateway(gw->id, gw->latDeg, gw->lonDeg, gw->name);
}

} // namespace

int
main(int argc, char* argv[])
{
    const std::string ns3BasePath  = "/home/wenj/workspace/ns-3.43";
    const std::string scenarioName = "constellation-iridium-66-sats-fixed";
    const uint32_t    numSats      = 66;

    const std::string islsFilePath =
        ns3BasePath + "/contrib/satellite/data/scenarios/" +
        scenarioName + "/positions/isls.txt";

    std::string mode       = "gw2gw"; // sat2sat | gw2gw | gw2ut
    double      simTime    = 630.0;
    double      slotInterval = 60.0;
    uint32_t    beamId     = 1;

    double      islMaxDistKm  = 5000.0;
    double      islRateMbps   = 10.0;
    double      emaAlpha      = 0.3;
    double      changeThresh  = 0.1;
    double      cooldownRatio = 0.5;
    double      elevMinDeg    = 5.0;

    // sat2sat mode
    uint32_t satSrc = 0;
    uint32_t satDst = 10;

    // gw2gw mode
    uint32_t gwSrc = 0;
    uint32_t gwDst = 1;

    // gw2ut mode
    uint32_t gwId    = 0;
    uint32_t utId    = 0;
    double   utLatDeg = 25.0330;
    double   utLonDeg = 121.5654;
    std::string utName = "UT-Taipei";

    CommandLine cmd;
    cmd.AddValue("mode",          "Routing case: sat2sat | gw2gw | gw2ut", mode);
    cmd.AddValue("simTime",       "Simulation duration (s)", simTime);
    cmd.AddValue("slotInterval",  "Routing slot interval (s)", slotInterval);
    cmd.AddValue("beamId",        "SNS3 beam ID to activate", beamId);
    cmd.AddValue("islMaxDistKm",  "ISL activation distance threshold (km)", islMaxDistKm);
    cmd.AddValue("islRateMbps",   "ISL link rate (Mbps)", islRateMbps);
    cmd.AddValue("emaAlpha",      "EMA weight for load-cost smoothing", emaAlpha);
    cmd.AddValue("changeThresh",  "Load-cost change ratio for recompute", changeThresh);
    cmd.AddValue("cooldownRatio", "Cooldown = slotInterval * cooldownRatio", cooldownRatio);
    cmd.AddValue("elevMinDeg",    "Minimum GW/UT elevation angle (deg)", elevMinDeg);

    cmd.AddValue("satSrc",        "Source satellite ID for sat2sat", satSrc);
    cmd.AddValue("satDst",        "Destination satellite ID for sat2sat", satDst);

    cmd.AddValue("gwSrc",         "Source gateway preset ID for gw2gw", gwSrc);
    cmd.AddValue("gwDst",         "Destination gateway preset ID for gw2gw", gwDst);

    cmd.AddValue("gwId",          "Gateway preset ID for gw2ut", gwId);
    cmd.AddValue("utId",          "User terminal ID for gw2ut", utId);
    cmd.AddValue("utLatDeg",      "UT latitude (deg) for gw2ut", utLatDeg);
    cmd.AddValue("utLonDeg",      "UT longitude (deg) for gw2ut", utLonDeg);
    cmd.AddValue("utName",        "UT name for gw2ut", utName);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(slotInterval <= 0.0, "slotInterval must be > 0");
    NS_ABORT_MSG_IF(simTime < 0.0, "simTime must be >= 0");
    NS_ABORT_MSG_IF(mode != "sat2sat" && mode != "gw2gw" && mode != "gw2ut",
                    "mode must be one of: sat2sat, gw2gw, gw2ut");
    NS_ABORT_MSG_IF(satSrc >= numSats || satDst >= numSats,
                    "satSrc/satDst must be < " << numSats);

    const double   islRateBps  = islRateMbps * 1.0e6;
    const double   cooldownSec = slotInterval * cooldownRatio;
    const uint32_t numSlots    =
        static_cast<uint32_t>(std::floor(simTime / slotInterval)) + 1;

    std::cout << "[CFG] mode=" << mode
              << " simTime=" << simTime
              << " slotInterval=" << slotInterval
              << " numSlots=" << numSlots
              << " lastSlotTime=" << ((numSlots - 1) * slotInterval)
              << "\n";

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
    Config::SetDefault("ns3::SatHelper::GwUsers", UintegerValue(1));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    ConfigureQoS();

    Ptr<SimulationHelper> simHelper =
        CreateObject<SimulationHelper>("test-iridium-3modes");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));
    simHelper->SetBeamSet(std::set<uint32_t>{beamId});
    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    auto wallStart = std::chrono::steady_clock::now();

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

    if (mode == "sat2sat")
    {
        std::cout << "\n[CASE] sat2sat | src=" << satSrc << " dst=" << satDst << "\n";
        routingMgr->PrintRouteReport({{satSrc, satDst}});
    }
    else if (mode == "gw2gw")
    {
        NS_ABORT_MSG_IF(gwSrc == gwDst, "gwSrc and gwDst must be different in gw2gw mode");

        routingMgr->SetGwElevationThreshold(elevMinDeg);
        AddGatewayOrAbort(routingMgr, gwSrc);
        AddGatewayOrAbort(routingMgr, gwDst);
        routingMgr->AddGwPair(gwSrc, gwDst);

        std::cout << "\n[CASE] gw2gw | gwSrc=" << gwSrc << " gwDst=" << gwDst << "\n";
        routingMgr->PrecomputeGwRoutes();
        routingMgr->PrintGwRouteReport();
    }
    else if (mode == "gw2ut")
    {
        routingMgr->SetGwElevationThreshold(elevMinDeg);
        AddGatewayOrAbort(routingMgr, gwId);
        routingMgr->AddUserTerminal(utId, utLatDeg, utLonDeg, utName);
        routingMgr->AddGwUtPair(gwId, utId);

        std::cout << "\n[CASE] gw2ut | gwId=" << gwId
                  << " utId=" << utId
                  << " utLatDeg=" << utLatDeg
                  << " utLonDeg=" << utLonDeg << "\n";
        routingMgr->PrecomputeGwUtRoutes();
        routingMgr->PrintGwUtRouteReport();
    }

    routingMgr->ScheduleRoutingUpdates();
    simHelper->RunSimulation();

    routingMgr->PrintStats();

    auto wallEnd = std::chrono::steady_clock::now();
    auto wallMs  = std::chrono::duration_cast<std::chrono::milliseconds>(
        wallEnd - wallStart).count();

    std::cout << "Total wall time: " << wallMs / 1000.0 << " s\n";
    std::cout << "Event count:     " << Simulator::GetEventCount() << "\n";

    Simulator::Destroy();
    return 0;
}
