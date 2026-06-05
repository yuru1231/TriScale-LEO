// gw-path-inspector.cc -- standalone inspector/capacity prototype for GW path validation
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <unordered_set>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

struct GatewayPreset
{
    uint32_t    id;
    double      latDeg;
    double      lonDeg;
    std::string name;
};

struct InspectConfig
{
    std::string mode{"inspect"};
    std::string view{"both"};
    std::string reportPrefix{"gw-path-inspector"};
    std::string search{"binary"};
    std::string regenerationMode{"network"};
    double      simTime{180.0};
    double      slotInterval{60.0};
    double      elevMinDeg{5.0};
    uint32_t    beamId{1};
    int32_t     slot{-1};
    uint32_t    gwSrc{0};
    uint32_t    gwDst{1};
    uint32_t    gwId{0};
    uint32_t    utId{0};
    double      utLatDeg{25.0330};
    double      utLonDeg{121.5654};
    std::string utName{"UT-Taipei"};
    double      rateMinKbps{64.0};
    double      rateMaxKbps{500000.0};
    double      dropThreshPct{1.0};
    double      delayThreshMs{100.0};
    double      goodputFloorPct{95.0};
    double      fwdUserBandwidthHz{0.5e9};
    double      rtnUserBandwidthHz{0.5e9};
    double      fwdFeederBandwidthHz{2.0e9};
    double      rtnFeederBandwidthHz{2.0e9};
    uint32_t    fwdUserChannels{4};
    uint32_t    rtnUserChannels{4};
    uint32_t    fwdFeederChannels{16};
    uint32_t    rtnFeederChannels{16};
    double      islRateMbps{1000.0};
    double      userSpectralEfficiency{1.0};
    double      feederSpectralEfficiency{1.0};
};

struct EvidenceRecord
{
    std::string segment;
    std::string status;
    std::string source;
    std::string detail;
};

struct CapacitySegment
{
    std::string name;
    double      kbps{0.0};
    std::string note;
};

struct CapacityResult
{
    bool                        applicable{false};
    std::vector<CapacitySegment> segments;
    double                      bottleneckKbps{0.0};
    std::string                 bottleneckName;
    double                      passRateKbps{0.0};
    uint32_t                    iterations{0};
};

static const std::vector<GatewayPreset>&
GetGatewayPresets()
{
    static const std::vector<GatewayPreset> kPresets = {
        {0, 35.6895,  139.6917, "JP-Tokyo"},
        {1, 28.6667,   77.2167, "IN-NewDelhi"},
        {2, 31.2222,  121.4581, "CN-Shanghai"},
        {3, -23.5475, -46.6361, "BR-SaoPaulo"},
        {4, 19.0740,   72.8808, "IN-Mumbai"},
    };
    return kPresets;
}

static const GatewayPreset*
FindGatewayPreset(uint32_t gwId)
{
    for (const auto& g : GetGatewayPresets())
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
                    << ". Supported presets: 0(Tokyo), 1(NewDelhi), 2(Shanghai), 3(SaoPaulo), 4(Mumbai)");
    routingMgr->AddGateway(gw->id, gw->latDeg, gw->lonDeg, gw->name);
}

static std::string
SatSetToString(const std::set<uint32_t>& sats)
{
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (uint32_t satId : sats)
    {
        if (!first)
        {
            oss << ",";
        }
        first = false;
        oss << satId;
    }
    oss << "}";
    return oss.str();
}

static std::string
PathToString(const std::vector<uint32_t>& path)
{
    if (path.empty())
    {
        return "(empty)";
    }

    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i > 0)
        {
            oss << " -> ";
        }
        oss << "sat" << path[i];
    }
    return oss.str();
}

static std::string
NodeSummary(Ptr<Node> node, const std::string& label)
{
    if (!node)
    {
        return label + "=missing";
    }

    std::ostringstream oss;
    oss << label << "=node" << node->GetId()
        << " devs=" << node->GetNDevices();
    return oss.str();
}

static std::string
ClosestSatSummary(Ptr<Node> node)
{
    if (!node)
    {
        return "closestSat=missing_node";
    }

    Ptr<SatMobilityModel> mm = node->GetObject<SatMobilityModel>();
    if (!mm)
    {
        return "closestSat=no_mobility";
    }

    uint32_t satId = Singleton<SatTopology>::Get()->GetClosestSat(GeoCoordinate(mm->GetPosition()));
    std::ostringstream oss;
    oss << "closestSat=sat" << satId;
    return oss.str();
}

static uint32_t
ResolveTrafficUtUserId(Ptr<IslRoutingManager> routingMgr,
                       uint32_t               gwId,
                       uint32_t               utId,
                       uint32_t               numSlots)
{
    uint32_t servingSatId = UINT32_MAX;
    for (uint32_t k = 0; k < numSlots; ++k)
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(gwId, utId, k);
        if (r.valid)
        {
            servingSatId = r.servingSatId;
            break;
        }
    }

    if (servingSatId == UINT32_MAX)
    {
        return utId;
    }

    Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
    NodeContainer utUsers = topo->GetUtUserNodes();
    for (uint32_t i = 0; i < utUsers.GetN(); ++i)
    {
        Ptr<Node> utNode = topo->GetUtNode(utUsers.Get(i));
        if (utNode && topo->GetUtSatId(utNode) == servingSatId)
        {
            return i;
        }
    }
    return utId;
}

static Ptr<Node>
GetPhysicalGwNodeById(uint32_t gwId)
{
    Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
    NodeContainer gwNodes = topo->GetGwNodes();
    if (gwId >= gwNodes.GetN())
    {
        return nullptr;
    }
    return gwNodes.Get(gwId);
}

static uint32_t
ResolveSlotIndex(const InspectConfig& cfg, uint32_t numSlots)
{
    if (cfg.slot >= 0)
    {
        NS_ABORT_MSG_IF(static_cast<uint32_t>(cfg.slot) >= numSlots,
                        "slot=" << cfg.slot << " >= numSlots=" << numSlots);
        return static_cast<uint32_t>(cfg.slot);
    }
    return 0;
}

static SatEnums::RegenerationMode_t
ParseRegenerationMode(const std::string& mode)
{
    if (mode == "network")
    {
        return SatEnums::REGENERATION_NETWORK;
    }
    if (mode == "phy")
    {
        return SatEnums::REGENERATION_PHY;
    }

    NS_ABORT_MSG("Unsupported regenerationMode=" << mode
                 << ". Expected network or phy");
}

static void
ValidateScenarioConstraints(const InspectConfig& cfg, const std::string& scenarioName)
{
    const bool isConstellationScenario =
        (scenarioName.find("constellation") != std::string::npos);

    NS_ABORT_MSG_IF(isConstellationScenario && cfg.regenerationMode != "network",
                    "Scenario " << scenarioName
                    << " requires regenerationMode=network when using SNS3 constellations; "
                    << "physical multi-GW instantiation is not available in this mode");
}

static std::string
NowTag()
{
    std::ostringstream oss;
    oss << static_cast<uint64_t>(std::time(nullptr));
    return oss.str();
}

static void
WriteEvidenceIndex(const std::string& path, const std::vector<EvidenceRecord>& records)
{
    std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        std::cout << "[REPORT] WARNING: failed to open evidence index " << path << "\n";
        return;
    }

    out << "segment,status,source,detail\n";
    for (const auto& r : records)
    {
        out << r.segment << "," << r.status << "," << r.source << "," << r.detail << "\n";
    }
}

static void
WriteReport(const std::string& path, const std::string& report)
{
    std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        std::cout << "[REPORT] WARNING: failed to open report file " << path << "\n";
        return;
    }
    out << report;
}

static std::string
MakeGw2GwLogicalPathStr(const GatewayPreset& src, const GatewayPreset& dst, const GwToGwRoute& route)
{
    std::ostringstream oss;
    oss << src.name << "(gw" << src.id << ")"
        << " -> entry:sat" << route.entrySatId
        << " -> " << PathToString(route.satPath)
        << " -> exit:sat" << route.exitSatId
        << " -> " << dst.name << "(gw" << dst.id << ")";
    return oss.str();
}

static std::string
MakeGw2UtLogicalPathStr(const GatewayPreset& src, const GwToUtRoute& route, const InspectConfig& cfg)
{
    std::ostringstream oss;
    oss << src.name << "(gw" << src.id << ")"
        << " -> entry:sat" << route.entrySatId
        << " -> " << PathToString(route.satPath)
        << " -> serving:sat" << route.servingSatId
        << " -> " << cfg.utName << "(ut" << cfg.utId << ")";
    return oss.str();
}

static void
AppendScenarioSummary(std::ostringstream& report,
                      const InspectConfig& cfg,
                      uint32_t             numSlots,
                      uint32_t             slotIndex)
{
    report << "Scenario Summary\n";
    report << "================\n";
    report << "mode=" << cfg.mode
           << " view=" << cfg.view
           << " regenerationMode=" << cfg.regenerationMode
           << " beamId=" << cfg.beamId
           << " simTime=" << cfg.simTime
           << " slotInterval=" << cfg.slotInterval
           << " numSlots=" << numSlots
           << " selectedSlot=" << slotIndex
           << " selectedTime=" << (slotIndex * cfg.slotInterval) << "s\n\n";
}

static void
AppendPhysicalGwSummary(std::ostringstream& report,
                        uint32_t             beamId,
                        const InspectConfig& cfg,
                        const std::set<uint32_t>& logicalGwIds)
{
    Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
    NodeContainer gwNodes = topo->GetGwNodes();
    NodeContainer gwUsers = topo->GetGwUserNodes();
    uint32_t requiredGwCount = logicalGwIds.empty()
                                   ? 0
                                   : (*std::max_element(logicalGwIds.begin(), logicalGwIds.end()) + 1);
    bool physicalReady = (gwNodes.GetN() >= requiredGwCount);

    report << "Physical Attachment Path\n";
    report << "========================\n";
    report << "beamId=" << beamId
           << " requiredLogicalGwCount=" << requiredGwCount
           << " physicalGwNodes=" << gwNodes.GetN()
           << " gwUserNodes=" << gwUsers.GetN() << "\n";
    report << "physicalValidation=" << (physicalReady ? "ready" : "logical_only") << "\n";
    if (!physicalReady)
    {
        report << "physicalValidationReason="
               << "logical GW IDs exceed instantiated physical GW nodes";
        if (cfg.regenerationMode == "network")
        {
            report << " (REGENERATION_NETWORK typically exposes a single physical GW node in this scenario)";
        }
        report << "\n";
    }

    Ptr<Node> mappedGw = topo->GetGwFromBeam(beamId);
    report << "beamMapping=confirmed source=SatTopology::GetGwFromBeam detail="
           << NodeSummary(mappedGw, "gwNode") << " "
           << ClosestSatSummary(mappedGw) << "\n";
}

static void
AppendGw2GwInspection(std::ostringstream&          report,
                      std::vector<EvidenceRecord>& evidence,
                      Ptr<IslRoutingManager>       routingMgr,
                      const InspectConfig&         cfg,
                      uint32_t                     slotIndex)
{
    const GatewayPreset* src = FindGatewayPreset(cfg.gwSrc);
    const GatewayPreset* dst = FindGatewayPreset(cfg.gwDst);
    NS_ABORT_MSG_IF(src == nullptr || dst == nullptr, "Invalid GW preset id");

    GwToGwRoute route = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slotIndex);

    report << "\nLogical Path\n";
    report << "============\n";
    report << "case=gw2gw"
           << " valid=" << (route.valid ? "yes" : "no") << "\n";
    report << "srcVisible=" << SatSetToString(routingMgr->GetGwVisibleSats(cfg.gwSrc, slotIndex)) << "\n";
    report << "dstVisible=" << SatSetToString(routingMgr->GetGwVisibleSats(cfg.gwDst, slotIndex)) << "\n";
    if (route.valid)
    {
        report << "entrySat=sat" << route.entrySatId
               << " exitSat=sat" << route.exitSatId
               << " islCost=" << route.islCost << "\n";
        report << "satPath=" << PathToString(route.satPath) << "\n";
        report << "logicalPath=" << MakeGw2GwLogicalPathStr(*src, *dst, route) << "\n";
    }
    else
    {
        report << "logicalPath=invalid_route\n";
    }

    AppendPhysicalGwSummary(report, cfg.beamId, cfg, std::set<uint32_t>{cfg.gwSrc, cfg.gwDst});
    Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
    Ptr<Node> srcUser = topo->GetGwUserNode(cfg.gwSrc);
    Ptr<Node> dstUser = topo->GetGwUserNode(cfg.gwDst);
    Ptr<Node> srcGwNode = GetPhysicalGwNodeById(cfg.gwSrc);
    Ptr<Node> dstGwNode = GetPhysicalGwNodeById(cfg.gwDst);
    report << "physicalPath="
           << NodeSummary(srcUser, "gwUserSrc")
           << " -> "
           << NodeSummary(srcGwNode, "gwNodeSrc")
           << " -> satNetwork(derived from logical path)"
           << " -> "
           << NodeSummary(dstGwNode, "gwNodeDst")
           << " -> "
           << NodeSummary(dstUser, "gwUserDst") << "\n";

    evidence.push_back({"logical_route", "confirmed", "GetGwRoute", route.valid ? MakeGw2GwLogicalPathStr(*src, *dst, route) : "invalid"});
    evidence.push_back({"feeder_uplink", "derived", "gwtx<gwId> / sat<entry>", route.valid ? ("gwtx" + std::to_string(cfg.gwSrc) + " -> sat" + std::to_string(route.entrySatId)) : "invalid"});
    evidence.push_back({"isl_path", route.valid ? "confirmed" : "not_directly_observable", "satPath", route.valid ? PathToString(route.satPath) : "invalid"});
    evidence.push_back({"feeder_downlink", "derived", "sat<exit> / gw<dst>", route.valid ? ("sat" + std::to_string(route.exitSatId) + " -> gw" + std::to_string(cfg.gwDst)) : "invalid"});
    evidence.push_back({"packet_endpoint", "confirmed", "GetGwUserNode / GetGwNodes", "GW user nodes and physical GW nodes resolved from current topology"});
}

static void
AppendGw2UtInspection(std::ostringstream&          report,
                      std::vector<EvidenceRecord>& evidence,
                      Ptr<IslRoutingManager>       routingMgr,
                      const InspectConfig&         cfg,
                      uint32_t                     slotIndex)
{
    const GatewayPreset* src = FindGatewayPreset(cfg.gwId);
    NS_ABORT_MSG_IF(src == nullptr, "Invalid gwId");

    GwToUtRoute route = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slotIndex);
    std::set<uint32_t> commonVisible;
    const auto& gwVisible = routingMgr->GetGwVisibleSats(cfg.gwId, slotIndex);
    const auto& utVisible = routingMgr->GetUtVisibleSats(cfg.utId, slotIndex);
    for (uint32_t satId : gwVisible)
    {
        if (utVisible.count(satId))
        {
            commonVisible.insert(satId);
        }
    }

    report << "\nLogical Path\n";
    report << "============\n";
    report << "case=gw2ut"
           << " valid=" << (route.valid ? "yes" : "no") << "\n";
    report << "gwVisible=" << SatSetToString(gwVisible) << "\n";
    report << "utVisible=" << SatSetToString(utVisible) << "\n";
    report << "commonVisible=" << SatSetToString(commonVisible) << "\n";
    if (route.valid)
    {
        report << "entrySat=sat" << route.entrySatId
               << " servingSat=sat" << route.servingSatId
               << " islCost=" << route.islCost << "\n";
        report << "satPath=" << PathToString(route.satPath) << "\n";
        report << "logicalPath=" << MakeGw2UtLogicalPathStr(*src, route, cfg) << "\n";
    }
    else
    {
        report << "logicalPath=invalid_route\n";
    }

    AppendPhysicalGwSummary(report, cfg.beamId, cfg, std::set<uint32_t>{cfg.gwId});
    Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
    uint32_t trafficUtUserId = ResolveTrafficUtUserId(routingMgr, cfg.gwId, cfg.utId, routingMgr->GetNumTimeSlots());
    Ptr<Node> gwUser = topo->GetGwUserNode(cfg.gwId);
    Ptr<Node> gwNode = GetPhysicalGwNodeById(cfg.gwId);
    Ptr<Node> utUser = topo->GetUtUserNode(trafficUtUserId);
    Ptr<Node> utNode = utUser ? topo->GetUtNode(utUser) : nullptr;
    report << "physicalPath="
           << NodeSummary(gwUser, "gwUser")
           << " -> "
           << NodeSummary(gwNode, "gwNode")
           << " -> satNetwork(derived from logical path)"
           << " -> "
           << (route.valid ? ("serving:sat" + std::to_string(route.servingSatId)) : std::string("serving=invalid"))
           << " -> "
           << NodeSummary(utNode, "utNode")
           << " -> "
           << NodeSummary(utUser, "utUser") << "\n";

    evidence.push_back({"logical_route", "confirmed", "GetGwUtRoute", route.valid ? MakeGw2UtLogicalPathStr(*src, route, cfg) : "invalid"});
    evidence.push_back({"feeder_uplink", "derived", "gwtx<gwId> / sat<entry>", route.valid ? ("gwtx" + std::to_string(cfg.gwId) + " -> sat" + std::to_string(route.entrySatId)) : "invalid"});
    evidence.push_back({"isl_path", route.valid ? "confirmed" : "not_directly_observable", "satPath", route.valid ? PathToString(route.satPath) : "invalid"});
    evidence.push_back({"service_downlink", "derived", "sat<serving> / ut<id>", route.valid ? ("sat" + std::to_string(route.servingSatId) + " -> utUser" + std::to_string(trafficUtUserId)) : "invalid"});
    evidence.push_back({"endpoint", "confirmed", "GetUtUserNode / GetUtNode", "UT node and UT user node resolved from current topology"});
}

static CapacitySegment
MakeCapacitySegment(const std::string& name, double kbps, const std::string& note)
{
    CapacitySegment seg;
    seg.name = name;
    seg.kbps = kbps;
    seg.note = note;
    return seg;
}

static CapacityResult
AnalyzeGw2GwCapacity(const InspectConfig& cfg, const GwToGwRoute& route)
{
    CapacityResult result;
    result.applicable = route.valid;
    if (!route.valid)
    {
        return result;
    }

    const double feederKbps =
        (cfg.fwdFeederBandwidthHz / std::max<uint32_t>(1, cfg.fwdFeederChannels)) *
        cfg.feederSpectralEfficiency / 1000.0;
    result.segments.push_back(MakeCapacitySegment("feeder_per_beam_fwd", feederKbps, "2 GHz / 16 channels * feeder spectral efficiency"));
    result.segments.push_back(MakeCapacitySegment("feeder_per_beam_dst", feederKbps, "assumed symmetric destination feeder"));

    if (route.satPath.size() > 1)
    {
        result.segments.push_back(MakeCapacitySegment("isl", cfg.islRateMbps * 1000.0, "ISL data rate attribute"));
    }

    result.bottleneckKbps = std::numeric_limits<double>::infinity();
    for (const auto& seg : result.segments)
    {
        if (seg.kbps < result.bottleneckKbps)
        {
            result.bottleneckKbps = seg.kbps;
            result.bottleneckName = seg.name;
        }
    }

    double hi = std::min(cfg.rateMaxKbps, result.bottleneckKbps);
    double lo = cfg.rateMinKbps;
    if (hi < lo)
    {
        lo = hi;
    }
    double target = result.bottleneckKbps * (cfg.goodputFloorPct / 100.0);
    for (uint32_t i = 0; i < 32 && hi - lo > 1.0; ++i)
    {
        double mid = (lo + hi) / 2.0;
        if (mid <= target)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
        result.iterations++;
    }
    result.passRateKbps = lo;
    return result;
}

static CapacityResult
AnalyzeGw2UtCapacity(const InspectConfig& cfg, const GwToUtRoute& route)
{
    CapacityResult result;
    result.applicable = route.valid;
    if (!route.valid)
    {
        return result;
    }

    const double feederKbps =
        (cfg.fwdFeederBandwidthHz / std::max<uint32_t>(1, cfg.fwdFeederChannels)) *
        cfg.feederSpectralEfficiency / 1000.0;
    const double serviceKbps =
        (cfg.fwdUserBandwidthHz / std::max<uint32_t>(1, cfg.fwdUserChannels)) *
        cfg.userSpectralEfficiency / 1000.0;

    result.segments.push_back(MakeCapacitySegment("feeder_per_beam_fwd", feederKbps, "2 GHz / 16 channels * feeder spectral efficiency"));
    if (route.satPath.size() > 1)
    {
        result.segments.push_back(MakeCapacitySegment("isl", cfg.islRateMbps * 1000.0, "ISL data rate attribute"));
    }
    result.segments.push_back(MakeCapacitySegment("service_per_beam_fwd", serviceKbps, "500 MHz / 4 channels * user spectral efficiency"));

    result.bottleneckKbps = std::numeric_limits<double>::infinity();
    for (const auto& seg : result.segments)
    {
        if (seg.kbps < result.bottleneckKbps)
        {
            result.bottleneckKbps = seg.kbps;
            result.bottleneckName = seg.name;
        }
    }

    double hi = std::min(cfg.rateMaxKbps, result.bottleneckKbps);
    double lo = cfg.rateMinKbps;
    if (hi < lo)
    {
        lo = hi;
    }
    double target = result.bottleneckKbps * (cfg.goodputFloorPct / 100.0);
    for (uint32_t i = 0; i < 32 && hi - lo > 1.0; ++i)
    {
        double mid = (lo + hi) / 2.0;
        if (mid <= target)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
        result.iterations++;
    }
    result.passRateKbps = lo;
    return result;
}

static void
AppendCapacityReport(std::ostringstream& report, const CapacityResult& result, const InspectConfig& cfg)
{
    report << "\nCapacity Probe\n";
    report << "==============\n";
    if (!result.applicable)
    {
        report << "status=not_applicable reason=invalid_logical_route\n";
        return;
    }

    report << "search=" << cfg.search
           << " rateMinKbps=" << cfg.rateMinKbps
           << " rateMaxKbps=" << cfg.rateMaxKbps
           << " goodputFloorPct=" << cfg.goodputFloorPct
           << " dropThreshPct=" << cfg.dropThreshPct
           << " delayThreshMs=" << cfg.delayThreshMs << "\n";
    for (const auto& seg : result.segments)
    {
        report << "segment=" << seg.name
               << " capKbps=" << std::fixed << std::setprecision(3) << seg.kbps
               << " note=" << seg.note << "\n";
    }
    report << "bottleneck=" << result.bottleneckName
           << " bottleneckKbps=" << result.bottleneckKbps << "\n";
    report << "binarySearchIterations=" << result.iterations
           << " estimatedPassRateKbps=" << result.passRateKbps << "\n";
    report << "note=capacity mode is theoretical in this prototype; drop/delay thresholds are retained as operator-facing limits for future packet-level probe integration.\n";
}

} // namespace

int
main(int argc, char* argv[])
{
    const std::string ns3BasePath = "/home/wenj/workspace/ns-3.43";
    const std::string scenarioName = "constellation-iridium-66-sats-fixed";
    const std::string islsFilePath =
        ns3BasePath + "/contrib/satellite/data/scenarios/" + scenarioName + "/positions/isls.txt";

    InspectConfig cfg;

    CommandLine cmd;
    cmd.AddValue("mode", "inspect | capacity", cfg.mode);
    cmd.AddValue("view", "gw2gw | gw2ut | both", cfg.view);
    cmd.AddValue("reportPrefix", "Prefix for text report and evidence index files", cfg.reportPrefix);
    cmd.AddValue("search", "Capacity search mode (binary)", cfg.search);
    cmd.AddValue("regenerationMode", "Satellite regeneration mode: network | phy", cfg.regenerationMode);
    cmd.AddValue("simTime", "Simulation duration (s)", cfg.simTime);
    cmd.AddValue("slotInterval", "Routing slot interval (s)", cfg.slotInterval);
    cmd.AddValue("elevMinDeg", "Minimum GW/UT elevation angle (deg)", cfg.elevMinDeg);
    cmd.AddValue("beamId", "SNS3 beam ID used for physical beam->GW attachment inspection", cfg.beamId);
    cmd.AddValue("slot", "Slot index to inspect (-1 means first slot)", cfg.slot);
    cmd.AddValue("gwSrc", "Source gateway preset ID for GW->GW view", cfg.gwSrc);
    cmd.AddValue("gwDst", "Destination gateway preset ID for GW->GW view", cfg.gwDst);
    cmd.AddValue("gwId", "Gateway preset ID for GW->UT view", cfg.gwId);
    cmd.AddValue("utId", "Logical UT ID for GW->UT view", cfg.utId);
    cmd.AddValue("utLatDeg", "UT latitude (deg) for GW->UT view", cfg.utLatDeg);
    cmd.AddValue("utLonDeg", "UT longitude (deg) for GW->UT view", cfg.utLonDeg);
    cmd.AddValue("utName", "UT name for GW->UT view", cfg.utName);
    cmd.AddValue("rateMin", "Capacity probe minimum offered rate (kbps)", cfg.rateMinKbps);
    cmd.AddValue("rateMax", "Capacity probe maximum offered rate (kbps)", cfg.rateMaxKbps);
    cmd.AddValue("dropThreshPct", "Drop threshold for future packet-level capacity checks", cfg.dropThreshPct);
    cmd.AddValue("delayThreshMs", "Delay threshold for future packet-level capacity checks", cfg.delayThreshMs);
    cmd.AddValue("goodputFloorPct", "Required goodput floor as a percentage of theoretical bottleneck", cfg.goodputFloorPct);
    cmd.AddValue("fwdUserBandwidthHz", "Forward user link bandwidth (Hz)", cfg.fwdUserBandwidthHz);
    cmd.AddValue("rtnUserBandwidthHz", "Return user link bandwidth (Hz)", cfg.rtnUserBandwidthHz);
    cmd.AddValue("fwdFeederBandwidthHz", "Forward feeder link bandwidth (Hz)", cfg.fwdFeederBandwidthHz);
    cmd.AddValue("rtnFeederBandwidthHz", "Return feeder link bandwidth (Hz)", cfg.rtnFeederBandwidthHz);
    cmd.AddValue("fwdUserChannels", "Forward user link channels", cfg.fwdUserChannels);
    cmd.AddValue("rtnUserChannels", "Return user link channels", cfg.rtnUserChannels);
    cmd.AddValue("fwdFeederChannels", "Forward feeder link channels", cfg.fwdFeederChannels);
    cmd.AddValue("rtnFeederChannels", "Return feeder link channels", cfg.rtnFeederChannels);
    cmd.AddValue("islRateMbps", "ISL data rate (Mbps)", cfg.islRateMbps);
    cmd.AddValue("userSpectralEfficiency", "User-link spectral efficiency assumption (bps/Hz)", cfg.userSpectralEfficiency);
    cmd.AddValue("feederSpectralEfficiency", "Feeder-link spectral efficiency assumption (bps/Hz)", cfg.feederSpectralEfficiency);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(cfg.mode != "inspect" && cfg.mode != "capacity",
                    "mode must be inspect or capacity");
    NS_ABORT_MSG_IF(cfg.view != "gw2gw" && cfg.view != "gw2ut" && cfg.view != "both",
                    "view must be gw2gw, gw2ut, or both");
    ValidateScenarioConstraints(cfg, scenarioName);

    const double islRateBps = cfg.islRateMbps * 1.0e6;
    const SatEnums::RegenerationMode_t regenMode = ParseRegenerationMode(cfg.regenerationMode);
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(regenMode));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(regenMode));
    Config::SetDefault("ns3::PointToPointIslHelper::IslDataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(islRateBps))));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionEachRequest",
                       BooleanValue(false));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionPeriod",
                       TimeValue(Seconds(cfg.slotInterval)));

    Ptr<SimulationHelper> simHelper = CreateObject<SimulationHelper>("gw-path-inspector");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(cfg.simTime));
    simHelper->SetBeamSet(std::set<uint32_t>{cfg.beamId});
    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    Ptr<IslRoutingManager> routingMgr = CreateObject<IslRoutingManager>();
    routingMgr->SetAttribute("NumSatellites", UintegerValue(66));
    routingMgr->SetAttribute("NumTimeSlots",
                             UintegerValue(static_cast<uint32_t>(std::floor(cfg.simTime / cfg.slotInterval)) + 1));
    routingMgr->SetAttribute("TimeSlotInterval", DoubleValue(cfg.slotInterval));
    routingMgr->SetAttribute("IslsFilePath", StringValue(islsFilePath));
    routingMgr->Initialize(islsFilePath);
    routingMgr->SetGwElevationThreshold(cfg.elevMinDeg);

    bool needGw2Gw = (cfg.view == "gw2gw" || cfg.view == "both");
    bool needGw2Ut = (cfg.view == "gw2ut" || cfg.view == "both");

    std::unordered_set<uint32_t> addedGatewayIds;
    auto ensureGatewayAdded = [&](uint32_t gwId) {
        if (addedGatewayIds.insert(gwId).second)
        {
            AddGatewayOrAbort(routingMgr, gwId);
        }
    };

    if (needGw2Gw)
    {
        ensureGatewayAdded(cfg.gwSrc);
        ensureGatewayAdded(cfg.gwDst);
        routingMgr->AddGwPair(cfg.gwSrc, cfg.gwDst);
    }
    if (needGw2Ut)
    {
        ensureGatewayAdded(cfg.gwId);
        routingMgr->AddUserTerminal(cfg.utId, cfg.utLatDeg, cfg.utLonDeg, cfg.utName);
        routingMgr->AddGwUtPair(cfg.gwId, cfg.utId);
    }

    {
        Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
        NodeContainer physicalGwNodes = topo->GetGwNodes();
        NodeContainer utUserNodes = topo->GetUtUserNodes();
        std::set<uint32_t> logicalGwIds;
        if (needGw2Gw)
        {
            logicalGwIds.insert(cfg.gwSrc);
            logicalGwIds.insert(cfg.gwDst);
        }
        if (needGw2Ut)
        {
            logicalGwIds.insert(cfg.gwId);
        }

        std::cout << "[TOPO] regenerationMode=" << cfg.regenerationMode
                  << " physicalGwNodes=" << physicalGwNodes.GetN()
                  << " utUserNodes=" << utUserNodes.GetN() << "\n";
        for (uint32_t gwIdForLog : logicalGwIds)
        {
            Ptr<Node> gwUser = topo->GetGwUserNode(gwIdForLog);
            Ptr<Node> gwNode = GetPhysicalGwNodeById(gwIdForLog);
            std::cout << "[TOPO] logicalGwId=" << gwIdForLog
                      << " gwUserNode=" << (gwUser ? "present" : "missing")
                      << " physicalGwNode=" << (gwNode ? "present" : "not_present")
                      << "\n";
        }
    }

    routingMgr->PrecomputeAllTables();
    if (needGw2Gw)
    {
        routingMgr->PrecomputeGwRoutes();
    }
    if (needGw2Ut)
    {
        routingMgr->PrecomputeGwUtRoutes();
    }

    const uint32_t slotIndex = ResolveSlotIndex(cfg, routingMgr->GetNumTimeSlots());

    std::ostringstream report;
    std::vector<EvidenceRecord> evidence;
    AppendScenarioSummary(report, cfg, routingMgr->GetNumTimeSlots(), slotIndex);

    if (needGw2Gw)
    {
        AppendGw2GwInspection(report, evidence, routingMgr, cfg, slotIndex);
        if (cfg.mode == "capacity")
        {
            AppendCapacityReport(report,
                                 AnalyzeGw2GwCapacity(cfg, routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slotIndex)),
                                 cfg);
        }
    }

    if (needGw2Ut)
    {
        if (needGw2Gw)
        {
            report << "\n";
        }
        AppendGw2UtInspection(report, evidence, routingMgr, cfg, slotIndex);
        if (cfg.mode == "capacity")
        {
            AppendCapacityReport(report,
                                 AnalyzeGw2UtCapacity(cfg, routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slotIndex)),
                                 cfg);
        }
    }

    const std::string tag = NowTag();
    const std::string reportPath = cfg.reportPrefix + "-" + tag + ".txt";
    const std::string evidencePath = cfg.reportPrefix + "-" + tag + "-evidence.csv";
    WriteReport(reportPath, report.str());
    WriteEvidenceIndex(evidencePath, evidence);

    std::cout << report.str();
    std::cout << "\nArtifacts\n";
    std::cout << "=========\n";
    std::cout << "report=" << reportPath << "\n";
    std::cout << "evidenceIndex=" << evidencePath << "\n";

    return 0;
}
