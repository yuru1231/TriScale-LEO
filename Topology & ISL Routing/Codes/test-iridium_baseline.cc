#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isl-graph.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

// === Tracing / Stats ========================================================

static void
ConfigureQoS()
{
    // Keep QoS disabled until the exact attribute names are verified for this SNS-3 build.
}

static void
RbdcTraceCallback(uint32_t requestKbps)
{
    if (requestKbps > 0)
    {
        std::cout << "[RBDC] t=" << Simulator::Now().GetSeconds() << "s"
                  << " request=" << requestKbps << " kbps\n";
    }
}

struct IslDropStats
{
    uint64_t total{0};
    uint64_t dropped{0};
};

static std::map<Ptr<Node>, uint32_t>       g_nodeToSatId;
static std::map<std::string, IslDropStats> g_islDropStats;

static void
IslPacketDropCallback(uint32_t /*pktSize*/, Ptr<Node> srcNode, Ptr<Node> dstNode, bool dropped)
{
    auto srcIt = g_nodeToSatId.find(srcNode);
    auto dstIt = g_nodeToSatId.find(dstNode);
    if (srcIt == g_nodeToSatId.end() || dstIt == g_nodeToSatId.end())
    {
        return;
    }

    std::string key = std::to_string(srcIt->second) + "-" + std::to_string(dstIt->second);
    g_islDropStats[key].total++;
    if (dropped)
    {
        g_islDropStats[key].dropped++;
    }
}

static uint32_t
ConnectIslDropTrace()
{
    NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();

    g_nodeToSatId.clear();
    for (uint32_t i = 0; i < sats.GetN(); ++i)
    {
        g_nodeToSatId[sats.Get(i)] = i;
    }

    uint32_t connected = 0;
    for (uint32_t i = 0; i < sats.GetN(); ++i)
    {
        Ptr<Node> satNode = sats.Get(i);
        for (uint32_t d = 0; d < satNode->GetNDevices(); ++d)
        {
            Ptr<SatOrbiterNetDevice> orbDev =
                DynamicCast<SatOrbiterNetDevice>(satNode->GetDevice(d));
            if (!orbDev)
            {
                continue;
            }

            for (auto& islDev : orbDev->GetIslsNetDevices())
            {
                if (islDev->TraceConnectWithoutContext("PacketDropRateTrace",
                                                       MakeCallback(&IslPacketDropCallback)))
                {
                    ++connected;
                }
            }
            break;
        }
    }

    std::cout << "[ISL_DROP] trace connected: " << connected
              << " ISL interfaces (" << connected / 2 << " unique links)\n";
    return connected;
}

static void
PrintIslDropStats(double threshPct, uint32_t connectedInterfaces)
{
    std::cout << "\n=== ISL Packet Drop Rate Summary ===\n";

    if (g_islDropStats.empty())
    {
        if (connectedInterfaces == 0)
        {
            std::cout << "  [FAIL] trace connection failed (0 interfaces connected)\n";
        }
        else
        {
            std::cout << "  [FAIL] " << connectedInterfaces
                      << " interfaces connected but 0 events recorded\n"
                      << "         (traffic may not have reached ISL layer)\n";
        }
        std::cout << "=====================================\n\n";
        return;
    }

    uint64_t sumTotal = 0;
    uint64_t sumDropped = 0;
    for (const auto& kv : g_islDropStats)
    {
        sumTotal += kv.second.total;
        sumDropped += kv.second.dropped;
    }

    double overallDrop = (sumTotal > 0) ? (100.0 * sumDropped / sumTotal) : 0.0;
    double overallSuccess = 100.0 - overallDrop;
    std::cout << "TOTAL: " << sumTotal << " pkts, "
              << sumDropped << " dropped | "
              << "drop_rate=" << std::fixed << std::setprecision(3) << overallDrop << "% | "
              << "success_rate=" << std::fixed << std::setprecision(3) << overallSuccess << "%\n";

    if (overallDrop < threshPct)
    {
        std::cout << "[PASS] overall ISL drop rate < " << threshPct << "%\n";
    }
    else
    {
        std::cout << "[FAIL] overall ISL drop rate = " << overallDrop
                  << "% >= threshold " << threshPct << "%\n";
    }

    std::cout << "=====================================\n\n";
}

// === E2E Config =============================================================

struct TrafficConfig
{
    bool     enableFwd{true};
    bool     enableRtn{true};
    uint32_t fwdIntervalMs{100};
    uint32_t rtnIntervalMs{500};
    uint32_t fwdPktBytes{1500};
    uint32_t rtnPktBytes{512};
    double   startSec{1.0};
    double   stopSec{0.0};
};

struct LinkBandConfig
{
    double   fwdBandwidthHz{2.0e9};
    double   rtnBandwidthHz{2.0e9};
    double   fwdBaseFrequencyHz{27.5e9};
    double   rtnBaseFrequencyHz{17.7e9};
    uint32_t fwdChannels{16};
    uint32_t rtnChannels{16};
};

enum class E2ESegment
{
    FEEDERLINK,
    ISL,
    SERVICELINK
};

enum class TrafficProfile
{
    NONE,
    GW2UT_APP,
    SAT2SAT_BG,
    GW2GW_BG,
    GW2GW_DIRECT
};

static TrafficProfile
ParseTrafficProfile(const std::string& s)
{
    if (s == "none") return TrafficProfile::NONE;
    if (s == "gw2ut") return TrafficProfile::GW2UT_APP;
    if (s == "sat2sat") return TrafficProfile::SAT2SAT_BG;
    if (s == "gw2gw") return TrafficProfile::GW2GW_BG;
    if (s == "gw2gw_direct") return TrafficProfile::GW2GW_DIRECT;

    NS_ABORT_MSG("trafficProfile must be one of: none | gw2ut | sat2sat | gw2gw | gw2gw_direct");
    return TrafficProfile::NONE;
}

static const char*
ToString(E2ESegment segment)
{
    switch (segment)
    {
    case E2ESegment::FEEDERLINK: return "feederlink";
    case E2ESegment::ISL: return "isl";
    case E2ESegment::SERVICELINK: return "servicelink";
    }
    return "unknown";
}

struct E2ESegmentConfig
{
    bool          enabled{false};
    TrafficConfig traffic{};
};

struct E2EConfig
{
    std::string    mode{"gw2gw"};
    TrafficProfile legacyProfile{TrafficProfile::NONE};
    bool           explicitSegments{false};
    double         simTimeSec{0.0};

    E2ESegmentConfig feederlink{};
    E2ESegmentConfig isl{};
    E2ESegmentConfig servicelink{};
    LinkBandConfig    feederBand{};
    LinkBandConfig    serviceBand{
        0.5e9, 0.5e9, 19.7e9, 29.5e9, 4, 4
    };

    bool observeLinkEntities{true};

    uint32_t satSrc{0};
    uint32_t satDst{0};
    uint32_t gwSrc{0};
    uint32_t gwDst{0};
    uint32_t gwId{0};
    uint32_t utId{0};
    double   utLatDeg{0.0};
    double   utLonDeg{0.0};
    std::string utName;
};

struct E2EExecutionPlan
{
    bool     installSharedEdgeTraffic{false};
    bool     installIslBackgroundTraffic{false};
    bool     installGw2GwBackgroundTraffic{false};
    bool     installGw2GwDirectTraffic{false};
    uint32_t edgeGatewayId{0};
};

struct LinkEntitySnapshot
{
    uint32_t satellitesVisited{0};
    uint32_t satellitesWithOrbiterDevice{0};
    uint32_t satellitesWithFeederPhy{0};
    uint32_t satellitesWithUserPhy{0};
    uint32_t totalFeederPhyObjects{0};
    uint32_t totalUserPhyObjects{0};
    uint32_t totalIslInterfaces{0};
};

// === Scenario / Endpoint Discovery ==========================================

struct GatewayPreset
{
    uint32_t    id;
    double      latDeg;
    double      lonDeg;
    std::string name;
};

static const GatewayPreset*
FindGatewayPreset(uint32_t gwId)
{
    static const std::vector<GatewayPreset> kPresets = {
        {0, 25.0, 121.5, "TW-Taipei"},
        {1, 35.7, 139.7, "JP-Tokyo"},
        {2, 37.8, -122.4, "US-SanFrancisco"},
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

static void
AddGatewayOrAbort(Ptr<IslRoutingManager> routingMgr, uint32_t gwId)
{
    const GatewayPreset* gw = FindGatewayPreset(gwId);
    NS_ABORT_MSG_IF(gw == nullptr,
                    "Unknown gwId=" << gwId
                    << ". Supported presets: 0(Taipei), 1(Tokyo), 2(SanFrancisco)");
    routingMgr->AddGateway(gw->id, gw->latDeg, gw->lonDeg, gw->name);
}

static NodeContainer
GetGwUsers(uint32_t gwId)
{
    auto topo = Singleton<SatTopology>::Get();
    return NodeContainer(topo->GetGwUserNode(gwId));
}

static NodeContainer
GetUtUsers()
{
    return Singleton<SatTopology>::Get()->GetUtUserNodes();
}

// === Feeder / User Link Integration ========================================

static void
ApplyLinkBandConfig(const LinkBandConfig& feederBand, const LinkBandConfig& serviceBand)
{
    Config::SetDefault("ns3::SatConf::FwdFeederLinkBandwidth",
                       DoubleValue(feederBand.fwdBandwidthHz));
    Config::SetDefault("ns3::SatConf::RtnFeederLinkBandwidth",
                       DoubleValue(feederBand.rtnBandwidthHz));
    Config::SetDefault("ns3::SatConf::FwdFeederLinkBaseFrequency",
                       DoubleValue(feederBand.fwdBaseFrequencyHz));
    Config::SetDefault("ns3::SatConf::RtnFeederLinkBaseFrequency",
                       DoubleValue(feederBand.rtnBaseFrequencyHz));
    Config::SetDefault("ns3::SatConf::FwdFeederLinkChannels",
                       UintegerValue(feederBand.fwdChannels));
    Config::SetDefault("ns3::SatConf::RtnFeederLinkChannels",
                       UintegerValue(feederBand.rtnChannels));

    Config::SetDefault("ns3::SatConf::FwdUserLinkBandwidth",
                       DoubleValue(serviceBand.fwdBandwidthHz));
    Config::SetDefault("ns3::SatConf::RtnUserLinkBandwidth",
                       DoubleValue(serviceBand.rtnBandwidthHz));
    Config::SetDefault("ns3::SatConf::FwdUserLinkBaseFrequency",
                       DoubleValue(serviceBand.fwdBaseFrequencyHz));
    Config::SetDefault("ns3::SatConf::RtnUserLinkBaseFrequency",
                       DoubleValue(serviceBand.rtnBaseFrequencyHz));
    Config::SetDefault("ns3::SatConf::FwdUserLinkChannels",
                       UintegerValue(serviceBand.fwdChannels));
    Config::SetDefault("ns3::SatConf::RtnUserLinkChannels",
                       UintegerValue(serviceBand.rtnChannels));
}

static void
PrintLinkBandConfig(const LinkBandConfig& feederBand, const LinkBandConfig& serviceBand)
{
    std::cout << "[LINKCFG][feeder] fwdBw=" << feederBand.fwdBandwidthHz
              << " rtnBw=" << feederBand.rtnBandwidthHz
              << " fwdFreq=" << feederBand.fwdBaseFrequencyHz
              << " rtnFreq=" << feederBand.rtnBaseFrequencyHz
              << " fwdCh=" << feederBand.fwdChannels
              << " rtnCh=" << feederBand.rtnChannels << "\n";

    std::cout << "[LINKCFG][service] fwdBw=" << serviceBand.fwdBandwidthHz
              << " rtnBw=" << serviceBand.rtnBandwidthHz
              << " fwdFreq=" << serviceBand.fwdBaseFrequencyHz
              << " rtnFreq=" << serviceBand.rtnBaseFrequencyHz
              << " fwdCh=" << serviceBand.fwdChannels
              << " rtnCh=" << serviceBand.rtnChannels << "\n";
}

static LinkEntitySnapshot
CaptureLinkEntitySnapshot()
{
    LinkEntitySnapshot snapshot;
    NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();

    snapshot.satellitesVisited = sats.GetN();
    for (uint32_t i = 0; i < sats.GetN(); ++i)
    {
        Ptr<Node> satNode = sats.Get(i);
        for (uint32_t d = 0; d < satNode->GetNDevices(); ++d)
        {
            Ptr<SatOrbiterNetDevice> orbDev =
                DynamicCast<SatOrbiterNetDevice>(satNode->GetDevice(d));
            if (!orbDev)
            {
                continue;
            }

            ++snapshot.satellitesWithOrbiterDevice;

            const auto feederPhy = orbDev->GetFeederPhy();
            const auto userPhy = orbDev->GetUserPhy();
            if (!feederPhy.empty())
            {
                ++snapshot.satellitesWithFeederPhy;
                snapshot.totalFeederPhyObjects += static_cast<uint32_t>(feederPhy.size());
            }
            if (!userPhy.empty())
            {
                ++snapshot.satellitesWithUserPhy;
                snapshot.totalUserPhyObjects += static_cast<uint32_t>(userPhy.size());
            }

            snapshot.totalIslInterfaces += static_cast<uint32_t>(orbDev->GetIslsNetDevices().size());
            break;
        }
    }

    return snapshot;
}

static void
PrintLinkEntitySnapshot(const LinkEntitySnapshot& snapshot)
{
    std::cout << "\n=== Feeder / User Link Entity Snapshot ===\n"
              << "satellites_visited      : " << snapshot.satellitesVisited << "\n"
              << "orbiter_devices_found   : " << snapshot.satellitesWithOrbiterDevice << "\n"
              << "sats_with_feeder_phy    : " << snapshot.satellitesWithFeederPhy << "\n"
              << "sats_with_user_phy      : " << snapshot.satellitesWithUserPhy << "\n"
              << "total_feeder_phy_objs   : " << snapshot.totalFeederPhyObjects << "\n"
              << "total_user_phy_objs     : " << snapshot.totalUserPhyObjects << "\n"
              << "total_isl_interfaces    : " << snapshot.totalIslInterfaces << "\n"
              << "==========================================\n\n";
}

// === E2E Planning Helpers ===================================================

static bool
HasAnySegmentEnabled(const E2EConfig& cfg)
{
    return cfg.feederlink.enabled || cfg.isl.enabled || cfg.servicelink.enabled;
}

static bool
HasEdgeSegmentEnabled(const E2EConfig& cfg)
{
    return cfg.feederlink.enabled || cfg.servicelink.enabled;
}

static double
ResolveTrafficStopSec(const TrafficConfig& cfg, double simTimeSec)
{
    return (cfg.stopSec > 0.0) ? cfg.stopSec : (simTimeSec - 1.0);
}

static void
PrintSegmentBanner(E2ESegment segment, bool enabled)
{
    std::cout << "[E2E][" << ToString(segment) << "] "
              << (enabled ? "enabled" : "disabled") << "\n";
}

static void
ApplyLegacySegmentDefaults(E2EConfig& cfg)
{
    if (cfg.explicitSegments)
    {
        return;
    }

    switch (cfg.legacyProfile)
    {
    case TrafficProfile::SAT2SAT_BG:
        cfg.isl.enabled = true;
        break;
    case TrafficProfile::GW2UT_APP:
    case TrafficProfile::GW2GW_BG:
    case TrafficProfile::GW2GW_DIRECT:
        cfg.feederlink.enabled = true;
        cfg.isl.enabled = true;
        cfg.servicelink.enabled = true;
        break;
    case TrafficProfile::NONE:
        if (cfg.mode == "sat2sat")
        {
            cfg.isl.enabled = true;
        }
        else
        {
            cfg.feederlink.enabled = true;
            cfg.isl.enabled = true;
            cfg.servicelink.enabled = true;
        }
        break;
    }
}

static void
ValidateE2EConfig(const E2EConfig& cfg)
{
    NS_ABORT_MSG_IF(!HasAnySegmentEnabled(cfg),
                    "At least one segment must be enabled for the e2e run");
    NS_ABORT_MSG_IF(cfg.mode != "sat2sat" && cfg.mode != "gw2gw" && cfg.mode != "gw2ut",
                    "mode must be one of: sat2sat, gw2gw, gw2ut");
    NS_ABORT_MSG_IF(cfg.mode == "gw2gw" && cfg.gwSrc == cfg.gwDst,
                    "gwSrc and gwDst must be different in gw2gw mode");
}

static E2EExecutionPlan
BuildE2EPlan(const E2EConfig& cfg)
{
    ValidateE2EConfig(cfg);

    E2EExecutionPlan plan;
    plan.edgeGatewayId = (cfg.mode == "gw2gw") ? cfg.gwSrc : cfg.gwId;

    switch (cfg.legacyProfile)
    {
    case TrafficProfile::GW2GW_DIRECT:
        plan.installGw2GwDirectTraffic = HasEdgeSegmentEnabled(cfg);
        break;
    case TrafficProfile::GW2GW_BG:
        plan.installGw2GwBackgroundTraffic = HasEdgeSegmentEnabled(cfg);
        break;
    case TrafficProfile::SAT2SAT_BG:
        plan.installIslBackgroundTraffic = cfg.isl.enabled;
        break;
    case TrafficProfile::GW2UT_APP:
        plan.installSharedEdgeTraffic = HasEdgeSegmentEnabled(cfg);
        break;
    case TrafficProfile::NONE:
        plan.installIslBackgroundTraffic =
            cfg.isl.enabled && !cfg.feederlink.enabled && !cfg.servicelink.enabled;
        if (HasEdgeSegmentEnabled(cfg))
        {
            if (cfg.mode == "gw2gw")
            {
                plan.installGw2GwDirectTraffic = true;
            }
            else
            {
                plan.installSharedEdgeTraffic = true;
            }
        }
        break;
    }

    return plan;
}

static void
PrintE2ERunBanner(const E2EConfig& cfg, const E2EExecutionPlan& plan)
{
    std::cout << "\n[E2E] mode=" << cfg.mode
              << " segments={feederlink=" << (cfg.feederlink.enabled ? "on" : "off")
              << ", isl=" << (cfg.isl.enabled ? "on" : "off")
              << ", servicelink=" << (cfg.servicelink.enabled ? "on" : "off")
              << "} traffic={sharedEdge=" << (plan.installSharedEdgeTraffic ? "on" : "off")
              << ", islBg=" << (plan.installIslBackgroundTraffic ? "on" : "off")
              << ", gw2gwBg=" << (plan.installGw2GwBackgroundTraffic ? "on" : "off")
              << ", gw2gwDirect=" << (plan.installGw2GwDirectTraffic ? "on" : "off")
              << "}\n";
}

// === Segment Traffic Installers ============================================

static void
InstallGwUtSegmentTrafficBase(Ptr<SimulationHelper> simHelper,
                              const TrafficConfig&  cfg,
                              double                simTimeSec,
                              uint32_t              gwId,
                              const std::string&    segmentLabel)
{
    if (!cfg.enableFwd && !cfg.enableRtn)
    {
        std::cout << "[TRAFFIC][" << segmentLabel
                  << "] Both FWD and RTN disabled, skipping traffic installation.\n";
        return;
    }

    NodeContainer gwUsers = GetGwUsers(gwId);
    NodeContainer utUsers = GetUtUsers();
    if (gwUsers.GetN() == 0 || utUsers.GetN() == 0)
    {
        std::cout << "[TRAFFIC][" << segmentLabel << "] WARNING: gwUsers=" << gwUsers.GetN()
                  << " utUsers=" << utUsers.GetN() << ", skipping traffic installation.\n";
        return;
    }

    double stopSec = ResolveTrafficStopSec(cfg, simTimeSec);
    Ptr<SatTrafficHelper> trafficHelper = simHelper->GetTrafficHelper();

    if (cfg.enableFwd)
    {
        trafficHelper->AddCbrTraffic(SatTrafficHelper::FWD_LINK,
                                     SatTrafficHelper::UDP,
                                     MilliSeconds(cfg.fwdIntervalMs),
                                     cfg.fwdPktBytes,
                                     gwUsers,
                                     utUsers,
                                     Seconds(cfg.startSec),
                                     Seconds(stopSec),
                                     Seconds(0));
    }

    if (cfg.enableRtn)
    {
        trafficHelper->AddCbrTraffic(SatTrafficHelper::RTN_LINK,
                                     SatTrafficHelper::UDP,
                                     MilliSeconds(cfg.rtnIntervalMs),
                                     cfg.rtnPktBytes,
                                     gwUsers,
                                     utUsers,
                                     Seconds(cfg.startSec),
                                     Seconds(stopSec),
                                     Seconds(0));
    }

    std::cout << "[TRAFFIC][" << segmentLabel << "] installed via SatTrafficHelper"
              << " gwId=" << gwId
              << " start=" << cfg.startSec
              << " stop=" << stopSec << "\n";
}

static void
InstallSat2SatBackgroundLoad(Ptr<SimulationHelper> simHelper, double simTimeSec, uint32_t gwAnchorId)
{
    TrafficConfig bg;
    bg.enableFwd = true;
    bg.enableRtn = true;
    bg.fwdIntervalMs = 30;
    bg.rtnIntervalMs = 30;
    bg.fwdPktBytes = 1500;
    bg.rtnPktBytes = 1500;
    bg.startSec = 1.0;
    bg.stopSec = simTimeSec - 1.0;

    std::cout << "[TRAFFIC][isl] install aggressive background load via GW="
              << gwAnchorId << " <-> all UTs\n";
    InstallGwUtSegmentTrafficBase(simHelper, bg, simTimeSec, gwAnchorId, "isl");
}

static void
InstallGw2GwBackgroundLoad(Ptr<SimulationHelper> simHelper,
                           double                simTimeSec,
                           uint32_t              gwSrc,
                           uint32_t              gwDst)
{
    TrafficConfig bg;
    bg.enableFwd = true;
    bg.enableRtn = true;
    bg.fwdIntervalMs = 60;
    bg.rtnIntervalMs = 60;
    bg.fwdPktBytes = 1500;
    bg.rtnPktBytes = 1024;
    bg.startSec = 1.0;
    bg.stopSec = simTimeSec - 1.0;

    InstallGwUtSegmentTrafficBase(simHelper, bg, simTimeSec, gwSrc, "feederlink");
    InstallGwUtSegmentTrafficBase(simHelper, bg, simTimeSec, gwDst, "servicelink");
}

static void
InstallGw2GwDirectApp(Ptr<SimulationHelper> simHelper,
                      uint32_t              gwSrc,
                      uint32_t              gwDst,
                      double                startSec,
                      double                stopSec)
{
    auto topo = Singleton<SatTopology>::Get();
    auto satHelper = simHelper->GetSatelliteHelper();
    Ptr<Node> srcUser = topo->GetGwUserNode(gwSrc);
    Ptr<Node> dstUser = topo->GetGwUserNode(gwDst);

    NS_ABORT_MSG_IF(!srcUser || !dstUser, "[GW2GW_DIRECT] missing GW user node");

    Ipv4Address dstAddr = satHelper->GetUserAddress(dstUser);
    const uint16_t port = 9001;

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    ApplicationContainer sinkApps = sink.Install(dstUser);
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec + 2.0));

    OnOffHelper sender("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
    sender.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    sender.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    sender.SetAttribute("DataRate", DataRateValue(DataRate("40960bps")));
    sender.SetAttribute("PacketSize", UintegerValue(512));
    ApplicationContainer senderApps = sender.Install(srcUser);
    senderApps.Start(Seconds(startSec));
    senderApps.Stop(Seconds(stopSec));
}

static bool
InstallFeederlinkTraffic(Ptr<SimulationHelper>   simHelper,
                         const E2EConfig&        cfg,
                         const E2EExecutionPlan& plan)
{
    PrintSegmentBanner(E2ESegment::FEEDERLINK, cfg.feederlink.enabled);
    if (!cfg.feederlink.enabled) return false;

    if (plan.installGw2GwDirectTraffic)
    {
        InstallGw2GwDirectApp(simHelper,
                              cfg.gwSrc,
                              cfg.gwDst,
                              cfg.feederlink.traffic.startSec,
                              ResolveTrafficStopSec(cfg.feederlink.traffic, cfg.simTimeSec));
        return true;
    }
    if (plan.installGw2GwBackgroundTraffic)
    {
        InstallGw2GwBackgroundLoad(simHelper, cfg.simTimeSec, cfg.gwSrc, cfg.gwDst);
        return true;
    }
    if (plan.installSharedEdgeTraffic)
    {
        InstallGwUtSegmentTrafficBase(simHelper,
                                      cfg.feederlink.traffic,
                                      cfg.simTimeSec,
                                      plan.edgeGatewayId,
                                      "feederlink");
        return true;
    }
    return false;
}

static void
InstallIslTraffic(Ptr<SimulationHelper>   simHelper,
                  const E2EConfig&        cfg,
                  const E2EExecutionPlan& plan)
{
    PrintSegmentBanner(E2ESegment::ISL, cfg.isl.enabled);
    if (cfg.isl.enabled && plan.installIslBackgroundTraffic)
    {
        InstallSat2SatBackgroundLoad(simHelper,
                                     cfg.simTimeSec,
                                     (cfg.mode == "gw2gw") ? cfg.gwSrc : cfg.gwId);
    }
}

static void
InstallServicelinkTraffic(Ptr<SimulationHelper>   simHelper,
                          const E2EConfig&        cfg,
                          const E2EExecutionPlan& plan,
                          bool                    edgeTrafficInstalled)
{
    PrintSegmentBanner(E2ESegment::SERVICELINK, cfg.servicelink.enabled);
    if (!cfg.servicelink.enabled || edgeTrafficInstalled) return;

    if (plan.installSharedEdgeTraffic)
    {
        InstallGwUtSegmentTrafficBase(simHelper,
                                      cfg.servicelink.traffic,
                                      cfg.simTimeSec,
                                      plan.edgeGatewayId,
                                      "servicelink");
    }
}

// === E2E Composition ========================================================

static void
InstallE2ETraffic(Ptr<SimulationHelper>   simHelper,
                  const E2EConfig&        cfg,
                  const E2EExecutionPlan& plan)
{
    bool edgeTrafficInstalled = InstallFeederlinkTraffic(simHelper, cfg, plan);
    InstallIslTraffic(simHelper, cfg, plan);
    InstallServicelinkTraffic(simHelper, cfg, plan, edgeTrafficInstalled);
}

// === Routing / Case Execution ===============================================

static void
ConfigureRoutingCase(Ptr<IslRoutingManager> routingMgr, const E2EConfig& cfg, double elevMinDeg)
{
    if (cfg.mode == "sat2sat")
    {
        std::cout << "\n[CASE] sat2sat | src=" << cfg.satSrc << " dst=" << cfg.satDst << "\n";
        routingMgr->PrintRouteReport({{cfg.satSrc, cfg.satDst}});
        return;
    }

    routingMgr->SetGwElevationThreshold(elevMinDeg);
    if (cfg.mode == "gw2gw")
    {
        AddGatewayOrAbort(routingMgr, cfg.gwSrc);
        AddGatewayOrAbort(routingMgr, cfg.gwDst);
        routingMgr->AddGwPair(cfg.gwSrc, cfg.gwDst);
        routingMgr->PrecomputeGwRoutes();
        routingMgr->PrintGwRouteReport();
        return;
    }

    AddGatewayOrAbort(routingMgr, cfg.gwId);
    routingMgr->AddUserTerminal(cfg.utId, cfg.utLatDeg, cfg.utLonDeg, cfg.utName);
    routingMgr->AddGwUtPair(cfg.gwId, cfg.utId);
    routingMgr->PrecomputeGwUtRoutes();
    routingMgr->PrintGwUtRouteReport();
}

} // namespace

int
main(int argc, char* argv[])
{
    const std::string ns3BasePath = "/home/wenj/workspace/ns-3.43";
    const std::string scenarioName = "constellation-iridium-66-sats-fixed";
    const uint32_t    numSats = 66;
    const std::string islsFilePath =
        ns3BasePath + "/contrib/satellite/data/scenarios/" + scenarioName + "/positions/isls.txt";

    std::string mode = "gw2gw";
    std::string trafficProf = "none";
    bool        enableFeederlink = false;
    bool        enableIsl = false;
    bool        enableServicelink = false;
    bool        observeLinkEntities = true;
    bool        rbdcVerbose = false;
    double      islDropThreshPct = 1.0;
    double      simTime = 630.0;
    double      slotInterval = 60.0;
    uint32_t    beamId = 1;
    double      islMaxDistKm = 5000.0;
    double      islRateMbps = 10.0;
    double      emaAlpha = 0.3;
    double      changeThresh = 0.1;
    double      cooldownRatio = 0.5;
    double      elevMinDeg = 5.0;

    TrafficConfig trafficCfg;
    LinkBandConfig feederBand;
    LinkBandConfig serviceBand{0.5e9, 0.5e9, 19.7e9, 29.5e9, 4, 4};
    uint32_t satSrc = 0;
    uint32_t satDst = 10;
    uint32_t gwSrc = 0;
    uint32_t gwDst = 1;
    uint32_t gwId = 0;
    uint32_t utId = 0;
    double   utLatDeg = 25.0330;
    double   utLonDeg = 121.5654;
    std::string utName = "UT-Taipei";

    CommandLine cmd;
    cmd.AddValue("mode", "Routing case: sat2sat | gw2gw | gw2ut", mode);
    cmd.AddValue("trafficProfile", "Legacy profile: none | gw2ut | sat2sat | gw2gw | gw2gw_direct", trafficProf);
    cmd.AddValue("enableFeederlink", "Enable feederlink segment (0/1)", enableFeederlink);
    cmd.AddValue("enableIsl", "Enable ISL segment (0/1)", enableIsl);
    cmd.AddValue("enableServicelink", "Enable servicelink segment (0/1)", enableServicelink);
    cmd.AddValue("observeLinkEntities", "Print feeder/user link entity snapshot after scenario creation", observeLinkEntities);
    cmd.AddValue("rbdcVerbose", "Print each RBDC request", rbdcVerbose);
    cmd.AddValue("islDropThreshPct", "ISL overall drop rate PASS threshold", islDropThreshPct);
    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("slotInterval", "Routing slot interval (s)", slotInterval);
    cmd.AddValue("beamId", "SNS3 beam ID to activate", beamId);
    cmd.AddValue("islMaxDistKm", "ISL activation distance threshold (km)", islMaxDistKm);
    cmd.AddValue("islRateMbps", "ISL link rate (Mbps)", islRateMbps);
    cmd.AddValue("emaAlpha", "EMA weight for load-cost smoothing", emaAlpha);
    cmd.AddValue("changeThresh", "Load-cost change ratio for recompute", changeThresh);
    cmd.AddValue("cooldownRatio", "Cooldown = slotInterval * cooldownRatio", cooldownRatio);
    cmd.AddValue("elevMinDeg", "Minimum GW/UT elevation angle (deg)", elevMinDeg);
    cmd.AddValue("satSrc", "Source satellite ID for sat2sat", satSrc);
    cmd.AddValue("satDst", "Destination satellite ID for sat2sat", satDst);
    cmd.AddValue("gwSrc", "Source gateway preset ID for gw2gw", gwSrc);
    cmd.AddValue("gwDst", "Destination gateway preset ID for gw2gw", gwDst);
    cmd.AddValue("gwId", "Gateway preset ID for gw2ut", gwId);
    cmd.AddValue("utId", "User terminal ID for gw2ut", utId);
    cmd.AddValue("utLatDeg", "UT latitude (deg) for gw2ut", utLatDeg);
    cmd.AddValue("utLonDeg", "UT longitude (deg) for gw2ut", utLonDeg);
    cmd.AddValue("utName", "UT name for gw2ut", utName);
    cmd.AddValue("fwd", "Enable FWD link (0/1)", trafficCfg.enableFwd);
    cmd.AddValue("rtn", "Enable RTN link (0/1)", trafficCfg.enableRtn);
    cmd.AddValue("fwdIntervalMs", "FWD CBR packet interval (ms)", trafficCfg.fwdIntervalMs);
    cmd.AddValue("rtnIntervalMs", "RTN CBR packet interval (ms)", trafficCfg.rtnIntervalMs);
    cmd.AddValue("fwdPktBytes", "FWD CBR packet size (bytes)", trafficCfg.fwdPktBytes);
    cmd.AddValue("rtnPktBytes", "RTN CBR packet size (bytes)", trafficCfg.rtnPktBytes);
    cmd.AddValue("trafficStart", "Traffic start time (s)", trafficCfg.startSec);
    cmd.AddValue("trafficStop", "Traffic stop time (s), 0 = simTime-1", trafficCfg.stopSec);
    cmd.AddValue("fwdFeederBandwidthHz", "Forward feeder link bandwidth", feederBand.fwdBandwidthHz);
    cmd.AddValue("rtnFeederBandwidthHz", "Return feeder link bandwidth", feederBand.rtnBandwidthHz);
    cmd.AddValue("fwdFeederBaseFrequencyHz", "Forward feeder link base frequency", feederBand.fwdBaseFrequencyHz);
    cmd.AddValue("rtnFeederBaseFrequencyHz", "Return feeder link base frequency", feederBand.rtnBaseFrequencyHz);
    cmd.AddValue("fwdFeederChannels", "Forward feeder link channel count", feederBand.fwdChannels);
    cmd.AddValue("rtnFeederChannels", "Return feeder link channel count", feederBand.rtnChannels);
    cmd.AddValue("fwdServiceBandwidthHz", "Forward user/service link bandwidth", serviceBand.fwdBandwidthHz);
    cmd.AddValue("rtnServiceBandwidthHz", "Return user/service link bandwidth", serviceBand.rtnBandwidthHz);
    cmd.AddValue("fwdServiceBaseFrequencyHz", "Forward user/service link base frequency", serviceBand.fwdBaseFrequencyHz);
    cmd.AddValue("rtnServiceBaseFrequencyHz", "Return user/service link base frequency", serviceBand.rtnBaseFrequencyHz);
    cmd.AddValue("fwdServiceChannels", "Forward user/service link channel count", serviceBand.fwdChannels);
    cmd.AddValue("rtnServiceChannels", "Return user/service link channel count", serviceBand.rtnChannels);
    cmd.Parse(argc, argv);

    TrafficProfile profile = ParseTrafficProfile(trafficProf);
    E2EConfig e2eCfg;
    e2eCfg.mode = mode;
    e2eCfg.legacyProfile = profile;
    e2eCfg.explicitSegments = enableFeederlink || enableIsl || enableServicelink;
    e2eCfg.simTimeSec = simTime;
    e2eCfg.feederlink.enabled = enableFeederlink;
    e2eCfg.isl.enabled = enableIsl;
    e2eCfg.servicelink.enabled = enableServicelink;
    e2eCfg.observeLinkEntities = observeLinkEntities;
    e2eCfg.feederlink.traffic = trafficCfg;
    e2eCfg.isl.traffic = trafficCfg;
    e2eCfg.servicelink.traffic = trafficCfg;
    e2eCfg.feederBand = feederBand;
    e2eCfg.serviceBand = serviceBand;
    e2eCfg.satSrc = satSrc;
    e2eCfg.satDst = satDst;
    e2eCfg.gwSrc = gwSrc;
    e2eCfg.gwDst = gwDst;
    e2eCfg.gwId = gwId;
    e2eCfg.utId = utId;
    e2eCfg.utLatDeg = utLatDeg;
    e2eCfg.utLonDeg = utLonDeg;
    e2eCfg.utName = utName;

    ApplyLegacySegmentDefaults(e2eCfg);
    E2EExecutionPlan e2ePlan = BuildE2EPlan(e2eCfg);
    const double islRateBps = islRateMbps * 1.0e6;
    const double cooldownSec = slotInterval * cooldownRatio;
    const uint32_t numSlots = static_cast<uint32_t>(std::floor(simTime / slotInterval)) + 1;

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
    Config::SetDefault("ns3::SatHelper::GwUsers", UintegerValue(3));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));
    ApplyLinkBandConfig(e2eCfg.feederBand, e2eCfg.serviceBand);
    PrintLinkBandConfig(e2eCfg.feederBand, e2eCfg.serviceBand);
    ConfigureQoS();

    Ptr<SimulationHelper> simHelper = CreateObject<SimulationHelper>("test-iridium-3segment-e2e");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));
    simHelper->SetBeamSet(std::set<uint32_t>{beamId});
    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    if (e2eCfg.observeLinkEntities)
    {
        PrintLinkEntitySnapshot(CaptureLinkEntitySnapshot());
    }

    uint32_t islConnected = ConnectIslDropTrace();
    if (rbdcVerbose)
    {
        Config::ConnectWithoutContext(
            "/NodeList/*/DeviceList/*/SatLlc/SatRequestManager/RbdcTrace",
            MakeCallback(&RbdcTraceCallback));
    }

    PrintE2ERunBanner(e2eCfg, e2ePlan);
    InstallE2ETraffic(simHelper, e2eCfg, e2ePlan);

    auto wallStart = std::chrono::steady_clock::now();
    Ptr<IslRoutingManager> routingMgr = CreateObject<IslRoutingManager>();
    routingMgr->SetAttribute("NumSatellites", UintegerValue(numSats));
    routingMgr->SetAttribute("NumTimeSlots", UintegerValue(numSlots));
    routingMgr->SetAttribute("TimeSlotInterval", DoubleValue(slotInterval));
    routingMgr->SetAttribute("IslMaxDistanceKm", DoubleValue(islMaxDistKm));
    routingMgr->SetAttribute("IslsFilePath", StringValue(islsFilePath));
    routingMgr->SetAttribute("EmaAlpha", DoubleValue(emaAlpha));
    routingMgr->SetAttribute("ChangeThreshold", DoubleValue(changeThresh));
    routingMgr->SetAttribute("CooldownSeconds", DoubleValue(cooldownSec));
    routingMgr->SetAttribute("IslLinkRateBps", DoubleValue(islRateBps));
    routingMgr->Initialize(islsFilePath);
    routingMgr->PrecomputeAllTables();
    ConfigureRoutingCase(routingMgr, e2eCfg, elevMinDeg);
    routingMgr->ScheduleRoutingUpdates();
    simHelper->RunSimulation();

    routingMgr->PrintStats();
    routingMgr->PrintLoadStats();
    PrintIslDropStats(islDropThreshPct, islConnected);

    auto wallEnd = std::chrono::steady_clock::now();
    auto wallMs = std::chrono::duration_cast<std::chrono::milliseconds>(wallEnd - wallStart).count();
    std::cout << "Total wall time: " << wallMs / 1000.0 << " s\n";
    std::cout << "Event count:     " << Simulator::GetEventCount() << "\n";

    Simulator::Destroy();
    return 0;
}
