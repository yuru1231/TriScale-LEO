// test-iridium-e2e.cc  -- Iridium-66 3-segment E2E simulation entry point
#include "ns3/applications-module.h"
#include "ns3/arp-cache.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/satellite-point-to-point-isl-channel.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"

#include <algorithm>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

// === Tracing / Stats ========================================================

static void
ConfigureQoS()
{
    // Leave QoS disabled here until the exact SNS3 attribute names are verified.
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
static bool                                g_islNodeMapReady{false};

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
    g_islNodeMapReady = !g_nodeToSatId.empty();
    if (!g_islNodeMapReady)
    {
        std::cout << "[ISL_DROP] WARNING: node-to-sat map is empty; ISL callbacks disabled\n";
        return 0;
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
                if (islDev->TraceConnectWithoutContext(
                        "PacketDropRateTrace",
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

    struct DropRow
    {
        std::string isl;
        uint64_t    total;
        uint64_t    dropped;
        double      dropRate;
        double      successRate;
    };

    std::vector<DropRow> rows;
    uint64_t sumTotal = 0;
    uint64_t sumDropped = 0;

    for (const auto& kv : g_islDropStats)
    {
        sumTotal += kv.second.total;
        sumDropped += kv.second.dropped;
        if (kv.second.dropped > 0)
        {
            double dr = 100.0 * kv.second.dropped / kv.second.total;
            double sr = 100.0 - dr;
            rows.push_back({kv.first, kv.second.total, kv.second.dropped, dr, sr});
        }
    }

    if (!rows.empty())
    {
        std::cout << std::left
                  << std::setw(14) << "ISL"
                  << std::setw(12) << "total_pkts"
                  << std::setw(10) << "dropped"
                  << std::setw(14) << "drop_rate(%)"
                  << "success_rate(%)\n"
                  << std::string(62, '-') << "\n";

        for (const auto& r : rows)
        {
            std::cout << std::left
                      << std::setw(14) << r.isl
                      << std::setw(12) << r.total
                      << std::setw(10) << r.dropped
                      << std::setw(14) << std::fixed << std::setprecision(3) << r.dropRate
                      << std::setprecision(3) << r.successRate << "\n";
        }
        std::cout << std::string(62, '-') << "\n";
    }
    else
    {
        std::cout << "  (all ISLs: 0 drops)\n";
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

// === E2E Link Observability =================================================
//
//
// Monitors drop_rate, throughput, and delay for each active link segment.
// Alert: fires stdout events when drop_rate or throughput crosses a threshold.
// Periodic: writes a CSV log row every obsInterval seconds.
//
// Dependency: ConnectLinkObserverTraces() must be called AFTER ConnectIslDropTrace()
// because the ISL callback shares g_nodeToSatId which ConnectIslDropTrace populates.
struct SegLinkStats
{
    // Cumulative counters
    uint64_t rxPkts{0};     // total received packets
    uint64_t rxBytes{0};    // total received bytes (used for throughput)
    uint64_t txPkts{0};     // total transmitted = rx + drop
    uint64_t dropPkts{0};   // total dropped packets

    // Delay stats: cumulative sum and sample count for avg-delay calculation
    double   sumDelayMs{0.0};
    uint64_t delaySamples{0};

    // Window stats: snapshot baseline for per-interval throughput computation
    uint64_t rxBytesWin{0};  // rxBytes value at the start of current window
    double   tWin{0.0};      // timestamp of window start (seconds)

    double DropRate() const
    {
        return (txPkts > 0) ? 100.0 * dropPkts / txPkts : 0.0;
    }

    double AvgDelayMs() const
    {
        return (delaySamples > 0) ? sumDelayMs / static_cast<double>(delaySamples) : 0.0;
    }

    // Compute throughput over the last measurement window (kbps)
    double WindowThroughputKbps(double nowSec) const
    {
        double dt = nowSec - tWin;
        if (dt < 1e-9)
        {
            return 0.0;
        }
        return static_cast<double>(rxBytes - rxBytesWin) * 8.0 / dt / 1000.0;
    }

    void BeginWindow(double nowSec)
    {
        rxBytesWin = rxBytes;
        tWin       = nowSec;
    }
};

struct IslPropagationStats
{
    uint64_t samples{0};
    double   sumDelayMs{0.0};
    double   minDelayMs{0.0};
    double   maxDelayMs{0.0};

    void AddSample(double delayMs)
    {
        if (samples == 0)
        {
            minDelayMs = delayMs;
            maxDelayMs = delayMs;
        }
        else
        {
            minDelayMs = std::min(minDelayMs, delayMs);
            maxDelayMs = std::max(maxDelayMs, delayMs);
        }
        sumDelayMs += delayMs;
        ++samples;
    }

    double AvgDelayMs() const
    {
        return (samples > 0) ? sumDelayMs / static_cast<double>(samples) : 0.0;
    }
};

struct ObsConfig
{
    double      snapshotIntervalSec{10.0};  // CSV log snapshot interval (s)
    double      dropAlertThreshPct{50.0};   // drop_rate% threshold for stdout alert
    double      trafficStartSec{1.0};       // suppress throughput=0 alert before traffic starts
    std::string logFilePath{"e2e_link_obs.csv"};
};

// --- Per-link observer state ---
static std::map<std::string, SegLinkStats> g_feederObsStats;   // key: "sat<nodeIdx>"
static std::map<std::string, SegLinkStats> g_serviceObsStats;  // key: "sat<nodeIdx>"
static std::map<std::string, SegLinkStats> g_islObsStats2;     // key: "<srcIdx>-<dstIdx>"
static std::map<std::string, IslPropagationStats> g_islPropagationStats;
static std::map<uint32_t, std::map<std::string, IslPropagationStats>> g_islPropagationStatsBySlot;
static std::map<std::string, uint64_t>     g_gwDeviceRxHits;
static std::map<std::string, std::string>  g_gwDeviceTypes;
static std::map<uint32_t, Ptr<Node>>       g_logicalGwNodeMap;
static bool                                g_obsDebug{false};
static ObsConfig                           g_obsCfg;
static std::ofstream                       g_obsLog;
static double                              g_traceSlotIntervalSec{60.0};
// Total count of OrbiterRxFeeder callbacks regardless of scope.
// Used to diagnose whether RxFeeder trace fires at all for a given pathType.
static uint64_t                            g_totalOrbiterFeederRxCalls{0};
static std::map<std::string, double>       g_prevObsDropRate;  // previous drop_rate per link for alert state machine
static std::map<std::string, double>       g_prevObsThroughputKbps;

static std::string
CsvEscape(const std::string& value)
{
    if (value.find_first_of(",\"\n") == std::string::npos)
    {
        return value;
    }

    std::string escaped = "\"";
    for (char c : value)
    {
        if (c == '"')
        {
            escaped += "\"\"";
        }
        else
        {
            escaped += c;
        }
    }
    escaped += "\"";
    return escaped;
}

template <typename T>
static std::string
SetToString(const std::set<T>& values)
{
    std::ostringstream oss;
    bool first = true;
    for (const auto& value : values)
    {
        if (!first)
        {
            oss << "|";
        }
        first = false;
        oss << value;
    }
    return oss.str();
}

static std::string
SatPathToString(const std::vector<uint32_t>& path)
{
    if (path.empty())
    {
        return "";
    }

    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i > 0)
        {
            oss << "->";
        }
        if (path[i] == UINT32_MAX)
        {
            oss << "?";
        }
        else
        {
            oss << "sat" << path[i];
        }
    }
    return oss.str();
}

static std::string
RouteNodeToString(uint32_t nodeId)
{
    return (nodeId == UINT32_MAX) ? "?" : ("sat" + std::to_string(nodeId));
}

static std::string
Ipv4ToString(Ipv4Address address)
{
    std::ostringstream oss;
    oss << address;
    return oss.str();
}

class Gw2GwTxTimeTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::Gw2GwTxTimeTag")
                                .SetParent<Tag>()
                                .AddConstructor<Gw2GwTxTimeTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return sizeof(uint64_t);
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU64(m_txTimeNs);
    }

    void Deserialize(TagBuffer i) override
    {
        m_txTimeNs = i.ReadU64();
    }

    void Print(std::ostream& os) const override
    {
        os << m_txTimeNs;
    }

    void SetTxTime(Time txTime)
    {
        m_txTimeNs = txTime.GetNanoSeconds();
    }

    Time GetTxTime() const
    {
        return NanoSeconds(m_txTimeNs);
    }

  private:
    uint64_t m_txTimeNs{0};
};

NS_OBJECT_ENSURE_REGISTERED(Gw2GwTxTimeTag);

class PacketHopTraceTag : public Tag
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::PacketHopTraceTag")
                                .SetParent<Tag>()
                                .AddConstructor<PacketHopTraceTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return sizeof(uint64_t);
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU64(m_traceId);
    }

    void Deserialize(TagBuffer i) override
    {
        m_traceId = i.ReadU64();
    }

    void Print(std::ostream& os) const override
    {
        os << m_traceId;
    }

    void SetTraceId(uint64_t traceId)
    {
        m_traceId = traceId;
    }

    uint64_t GetTraceId() const
    {
        return m_traceId;
    }

  private:
    uint64_t m_traceId{0};
};

NS_OBJECT_ENSURE_REGISTERED(PacketHopTraceTag);

struct PacketHopTraceConfig
{
    bool     enabled{false};
    uint32_t maxPackets{8};
};

static PacketHopTraceConfig           g_packetHopTraceCfg;
static uint64_t                       g_nextPacketHopTraceId{1};
static std::set<uint64_t>             g_packetHopTraceActiveIds;
static std::map<uint64_t, std::string> g_packetHopTraceFlowLabels;

static bool
TryGetPacketHopTraceId(Ptr<const Packet> pkt, uint64_t& traceId)
{
    if (!pkt)
    {
        return false;
    }

    PacketHopTraceTag tag;
    Ptr<Packet> copy = pkt->Copy();
    if (!copy->PeekPacketTag(tag))
    {
        return false;
    }

    traceId = tag.GetTraceId();
    return (traceId != 0);
}

static uint64_t
AttachPacketHopTraceTag(Ptr<Packet> pkt, const std::string& flowLabel)
{
    if (!g_packetHopTraceCfg.enabled || !pkt)
    {
        return 0;
    }

    const uint64_t traceId = g_nextPacketHopTraceId++;
    PacketHopTraceTag traceTag;
    traceTag.SetTraceId(traceId);
    pkt->AddPacketTag(traceTag);

    if (g_packetHopTraceActiveIds.size() <
        static_cast<size_t>(g_packetHopTraceCfg.maxPackets))
    {
        g_packetHopTraceActiveIds.insert(traceId);
        g_packetHopTraceFlowLabels[traceId] = flowLabel;
    }

    return traceId;
}

static bool
ShouldLogPacketHopTrace(uint64_t traceId)
{
    return g_packetHopTraceCfg.enabled &&
           g_packetHopTraceActiveIds.count(traceId) > 0;
}

static void
LogPacketHopTrace(uint64_t             traceId,
                  const std::string&   stage,
                  const std::string&   nodeLabel,
                  const std::string&   detail,
                  Ptr<const Packet>    pkt = nullptr)
{
    if (!ShouldLogPacketHopTrace(traceId))
    {
        return;
    }

    const std::string flowLabel =
        (g_packetHopTraceFlowLabels.count(traceId) > 0)
            ? g_packetHopTraceFlowLabels[traceId]
            : "unknown";

    std::cout << "[PKT_HOP] t=" << std::fixed << std::setprecision(3)
              << Simulator::Now().GetSeconds()
              << "s traceId=" << traceId
              << " flow=" << flowLabel
              << " stage=" << stage
              << " node=" << nodeLabel;
    if (pkt)
    {
        std::cout << " bytes=" << pkt->GetSize();
    }
    if (!detail.empty())
    {
        std::cout << " " << detail;
    }
    std::cout << "\n";
}

struct EndpointLayerStats
{
    bool     connected{false};
    uint64_t rxPkts{0};
    uint64_t rxBytes{0};
};

struct EndpointAppStats
{
    bool            installed{false};
    bool            traceConnected{false};
    uint64_t        rxPkts{0};
    uint64_t        traceRxBytes{0};
    uint64_t        sinkTotalRxBytes{0};
    Ptr<PacketSink> sink;
};

struct EndpointProbeTargetStats
{
    bool               active{false};
    bool               notApplicable{false};
    std::string        label;
    std::string        reason;
    EndpointLayerStats phy;
    EndpointLayerStats mac;
    EndpointLayerStats dev;
    EndpointAppStats   app;
};

struct EndpointProbeState
{
    bool                     enabled{false};
    uint16_t                 port{9100};
    std::string              pathType;
    EndpointProbeTargetStats ut;
    EndpointProbeTargetStats gw;
    EndpointProbeTargetStats sat;
};

static EndpointProbeState g_endpointProbe;

struct Gw2GwAppDeliveryStats
{
    bool        installed{false};
    bool        traceConnected{false};
    bool        reported{false};
    uint32_t    srcGwId{0};
    uint32_t    dstGwId{0};
    Ipv4Address srcAddr;
    Ipv4Address dstAddr;
    uint64_t    traceRxPkts{0};
    uint64_t    traceRxBytes{0};
    uint64_t    rxBytes{0};
    uint64_t    estPkts{0};
    uint64_t    delaySamples{0};
    double      sumDelayMs{0.0};
    double      minDelayMs{0.0};
    double      maxDelayMs{0.0};
};

static Gw2GwAppDeliveryStats g_gw2gwDelivery;

struct Sat2UtAppDeliveryStats
{
    bool        installed{false};
    bool        traceConnected{false};
    bool        reported{false};
    uint32_t    dstUtUserId{0};
    Ipv4Address dstAddr;
    uint64_t    traceRxPkts{0};
    uint64_t    traceRxBytes{0};
    uint64_t    rxBytes{0};
    uint64_t    estPkts{0};
    uint64_t    delaySamples{0};
    double      sumDelayMs{0.0};
    double      minDelayMs{0.0};
    double      maxDelayMs{0.0};
};

static Sat2UtAppDeliveryStats g_sat2utDelivery;

struct Sat2SatAppDeliveryStats
{
    bool        installed{false};
    bool        traceConnected{false};
    bool        reported{false};
    uint32_t    srcSatId{0};
    uint32_t    dstSatId{0};
    Ipv4Address srcAddr;
    Ipv4Address dstAddr;
    uint64_t    traceRxPkts{0};
    uint64_t    traceRxBytes{0};
    uint64_t    rxBytes{0};
    uint64_t    estPkts{0};
    uint64_t    delaySamples{0};
    double      sumDelayMs{0.0};
    double      minDelayMs{0.0};
    double      maxDelayMs{0.0};
};

static Sat2SatAppDeliveryStats g_sat2satDelivery;

struct ObsScope
{
    bool activeFeeder{false};
    bool activeService{false};
    bool activeIsl{false};
    std::set<std::string> feederKeys;
    std::set<std::string> serviceKeys;
    std::set<std::string> islKeys;
};

static ObsScope g_obsScope;
static ObsScope g_obsVerdictScope;

struct GwFeederInterfaceInfo
{
    uint32_t          ifIndex{UINT32_MAX};
    Ipv4Address       address;
    Ptr<NetDevice>    netDevice;
    Ptr<SatNetDevice> satDevice;
};

struct E2EConfig;

static NodeContainer GetPhysicalGwNodes(uint32_t gwId);
static Ptr<Node> GetPhysicalGwNodeOrNull(uint32_t gwId);
static bool TryGetNodeRoutableIp(Ptr<Node> node, const std::string& label, Ipv4Address& out);
static bool TryGetPhysicalGwRoutableIp(Ptr<Node> node, uint32_t gwId, Ipv4Address& out);
static GwFeederInterfaceInfo ResolveGwFeederInterface(Ptr<Node> node,
                                                      uint32_t gwId,
                                                      Ipv4Address addr);
static void PrefillArpEntry(Ptr<Node> node,
                            uint32_t gwId,
                            const GwFeederInterfaceInfo& srcFeeder,
                            Ipv4Address dstAddr,
                            Mac48Address dstMac);
static double EstimateTheoreticalIslPropagationDelayMs(Ptr<IslRoutingManager> routingMgr,
                                                       const E2EConfig&       cfg,
                                                       uint32_t               slot);
static uint32_t EstimateTheoreticalIslHopCount(Ptr<IslRoutingManager> routingMgr,
                                               const E2EConfig&       cfg,
                                               uint32_t               slot);
static double GroundToSatDelayMs(double latDeg, double lonDeg, const Vector& satEcef);
static double ComputeIslPropagationDelayMs(Ptr<IslRoutingManager> routingMgr,
                                           const std::vector<uint32_t>& path,
                                           uint32_t slot,
                                           bool verbose);
static Ipv4Address ExitSatFeederIp(uint32_t exitSatId);
static void RefreshLogicalGwNodeMap(const std::string& rtnConfFilePath,
                                    const std::set<uint32_t>& targetGwIds);

static std::string
MakeSatKey(uint32_t satId)
{
    return "sat" + std::to_string(satId);
}

static std::string
MakeGwKey(uint32_t gwId)
{
    return "gw" + std::to_string(gwId);
}

static std::string
MakeGwTxKey(uint32_t gwId)
{
    // Distinct from MakeGwKey ("gw<id>") which is used for SAT→GW Rx obs.
    // "gwtx<id>" is used for GW→SAT Tx obs (entry feeder uplink).
    return "gwtx" + std::to_string(gwId);
}

static std::string
MakeUtKey(uint32_t utId)
{
    // Used for service-link FWD observation at UT Rx side (SAT→UT direction).
    return "ut" + std::to_string(utId);
}

static std::string
MakeIslKey(uint32_t srcSatId, uint32_t dstSatId)
{
    return std::to_string(srcSatId) + "-" + std::to_string(dstSatId);
}

static bool
HasPrefix(const std::string& s, const std::string& prefix)
{
    return s.compare(0, prefix.size(), prefix) == 0;
}

static bool
IsObsKeyInScope(const ObsScope& scope, const std::string& linkType, const std::string& key)
{
    if (linkType == "feeder")
    {
        return scope.activeFeeder &&
               scope.feederKeys.count(key) > 0;
    }

    if (linkType == "service")
    {
        return scope.activeService &&
               scope.serviceKeys.count(key) > 0;
    }

    if (linkType == "isl")
    {
        return scope.activeIsl &&
               scope.islKeys.count(key) > 0;
    }

    return false;
}

static bool
IsObsKeyInScope(const std::string& linkType, const std::string& key)
{
    return IsObsKeyInScope(g_obsScope, linkType, key);
}

static void
MergeObsScope(ObsScope& dst, const ObsScope& src)
{
    dst.activeFeeder = dst.activeFeeder || src.activeFeeder;
    dst.activeService = dst.activeService || src.activeService;
    dst.activeIsl = dst.activeIsl || src.activeIsl;
    dst.feederKeys.insert(src.feederKeys.begin(), src.feederKeys.end());
    dst.serviceKeys.insert(src.serviceKeys.begin(), src.serviceKeys.end());
    dst.islKeys.insert(src.islKeys.begin(), src.islKeys.end());
}

// --- Orbiter feeder-link callbacks ---

static void
OrbiterRxFeederCb(std::string key, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    // Feeder link RX: satellite received packet from GW (FWD direction).
    // g_totalOrbiterFeederRxCalls counts every invocation regardless of scope,
    // so the summary can show whether RxFeeder trace ever fires for this pathType.
    ++g_totalOrbiterFeederRxCalls;
    auto& s  = g_feederObsStats[key];
    s.txPkts++;
    s.rxPkts++;
    s.rxBytes += pkt->GetSize();

    uint64_t traceId = 0;
    if (TryGetPacketHopTraceId(pkt, traceId))
    {
        LogPacketHopTrace(traceId, "feeder_rx", key, "", pkt);
    }
}

static void
OrbiterFeederDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // Feeder link one-way delay sample
    auto& s = g_feederObsStats[key];
    s.sumDelayMs += delay.ToDouble(Time::MS);
    s.delaySamples++;
}

// --- Orbiter service-link callbacks ---

static void
OrbiterRxUserCb(std::string key, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    // Service-link orbiter-side RX. Treat this as the stable MAC/orbiter
    // visibility layer and keep UT SatNetDevice::Rx as a separate endpoint probe.
    auto& s  = g_serviceObsStats[key];
    s.txPkts++;
    s.rxPkts++;
    s.rxBytes += pkt->GetSize();
}

static void
OrbiterUserDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // Service link one-way delay sample
    auto& s = g_serviceObsStats[key];
    s.sumDelayMs += delay.ToDouble(Time::MS);
    s.delaySamples++;
}

static void
UtRxServiceCb(std::string key, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    // Service link FWD RX: UT received packet from satellite (FWD direction, SAT→UT).
    // Used for sat2ut and gw2ut_e2e where traffic flows toward the UT.
    // OrbiterRxUserCb fires on RTN (UT→SAT) and cannot observe FWD delivery.
    auto& s  = g_serviceObsStats[key];
    s.txPkts++;
    s.rxPkts++;
    s.rxBytes += pkt->GetSize();

    uint64_t traceId = 0;
    if (TryGetPacketHopTraceId(pkt, traceId))
    {
        LogPacketHopTrace(traceId, "service_rx", key, "", pkt);
    }
}

static void
Gw2GwAppRxCb(Ptr<const Packet> pkt, const Address& /*from*/)
{
    g_gw2gwDelivery.traceRxPkts++;
    g_gw2gwDelivery.traceRxBytes += pkt->GetSize();

    uint64_t hopTraceId = 0;
    if (TryGetPacketHopTraceId(pkt, hopTraceId))
    {
        LogPacketHopTrace(hopTraceId, "app_rx", "gw_app_dst", "", pkt);
    }

    Gw2GwTxTimeTag txTag;
    Ptr<Packet> copy = pkt->Copy();
    if (copy->PeekPacketTag(txTag))
    {
        double nowMs         = Simulator::Now().ToDouble(Time::MS);
        double txMs          = txTag.GetTxTime().ToDouble(Time::MS);
        double oneWayDelayMs = nowMs - txMs;

        // Print first 5 samples to diagnose whether the tag survives satellite
        // regeneration intact. If txMs ≈ nowMs the tag was reset during forwarding.
        // If txMs = 0 the tag was lost and re-deserialized from a zeroed buffer.
        if (g_gw2gwDelivery.delaySamples < 5)
        {
            std::cout << "[GW2GW_TAG_DBG] pkt=" << g_gw2gwDelivery.delaySamples
                      << " now=" << std::fixed << std::setprecision(3) << nowMs
                      << "ms txTime=" << txMs
                      << "ms delay=" << oneWayDelayMs << "ms\n";
        }

        g_gw2gwDelivery.delaySamples++;
        g_gw2gwDelivery.sumDelayMs += oneWayDelayMs;
        if (g_gw2gwDelivery.delaySamples == 1)
        {
            g_gw2gwDelivery.minDelayMs = oneWayDelayMs;
            g_gw2gwDelivery.maxDelayMs = oneWayDelayMs;
        }
        else
        {
            g_gw2gwDelivery.minDelayMs = std::min(g_gw2gwDelivery.minDelayMs, oneWayDelayMs);
            g_gw2gwDelivery.maxDelayMs = std::max(g_gw2gwDelivery.maxDelayMs, oneWayDelayMs);
        }
    }
}

static void
Sat2UtAppRxCb(Ptr<const Packet> pkt, const Address& /*from*/)
{
    g_sat2utDelivery.traceRxPkts++;
    g_sat2utDelivery.traceRxBytes += pkt->GetSize();

    uint64_t hopTraceId = 0;
    if (TryGetPacketHopTraceId(pkt, hopTraceId))
    {
        LogPacketHopTrace(hopTraceId, "app_rx", "ut_app_dst", "", pkt);
    }

    Gw2GwTxTimeTag txTag;
    Ptr<Packet> copy = pkt->Copy();
    if (copy->PeekPacketTag(txTag))
    {
        double oneWayDelayMs = (Simulator::Now() - txTag.GetTxTime()).ToDouble(Time::MS);
        g_sat2utDelivery.delaySamples++;
        g_sat2utDelivery.sumDelayMs += oneWayDelayMs;
        if (g_sat2utDelivery.delaySamples == 1)
        {
            g_sat2utDelivery.minDelayMs = oneWayDelayMs;
            g_sat2utDelivery.maxDelayMs = oneWayDelayMs;
        }
        else
        {
            g_sat2utDelivery.minDelayMs = std::min(g_sat2utDelivery.minDelayMs, oneWayDelayMs);
            g_sat2utDelivery.maxDelayMs = std::max(g_sat2utDelivery.maxDelayMs, oneWayDelayMs);
        }
    }
}

static void
Sat2SatAppRxCb(Ptr<const Packet> pkt, const Address& /*from*/)
{
    g_sat2satDelivery.traceRxPkts++;
    g_sat2satDelivery.traceRxBytes += pkt->GetSize();

    uint64_t hopTraceId = 0;
    if (TryGetPacketHopTraceId(pkt, hopTraceId))
    {
        LogPacketHopTrace(hopTraceId, "app_rx", "sat_app_dst", "", pkt);
    }

    Gw2GwTxTimeTag txTag;
    Ptr<Packet> copy = pkt->Copy();
    if (copy->PeekPacketTag(txTag))
    {
        double nowMs = Simulator::Now().ToDouble(Time::MS);
        double txMs = txTag.GetTxTime().ToDouble(Time::MS);
        double oneWayDelayMs = nowMs - txMs;

        g_sat2satDelivery.delaySamples++;
        g_sat2satDelivery.sumDelayMs += oneWayDelayMs;
        if (g_sat2satDelivery.delaySamples == 1)
        {
            g_sat2satDelivery.minDelayMs = oneWayDelayMs;
            g_sat2satDelivery.maxDelayMs = oneWayDelayMs;
        }
        else
        {
            g_sat2satDelivery.minDelayMs = std::min(g_sat2satDelivery.minDelayMs, oneWayDelayMs);
            g_sat2satDelivery.maxDelayMs = std::max(g_sat2satDelivery.maxDelayMs, oneWayDelayMs);
        }
    }
}

static void
EndpointLayerRxCb(EndpointLayerStats* stats, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    if (!stats)
    {
        return;
    }
    stats->rxPkts++;
    stats->rxBytes += pkt->GetSize();
}

static void
EndpointAppRxCb(EndpointAppStats* stats, Ptr<const Packet> pkt, const Address& /*from*/)
{
    if (!stats)
    {
        return;
    }
    stats->rxPkts++;
    stats->traceRxBytes += pkt->GetSize();
}

static void
ConnectEndpointDeviceProbe(Ptr<Node> node, EndpointProbeTargetStats& target)
{
    if (!node)
    {
        target.reason = "node_missing";
        return;
    }

    uint32_t satDevCount{0};
    for (uint32_t d = 0; d < node->GetNDevices(); ++d)
    {
        Ptr<SatNetDevice> satDev = DynamicCast<SatNetDevice>(node->GetDevice(d));
        if (!satDev)
        {
            continue;
        }
        ++satDevCount;

        if (satDev->TraceConnectWithoutContext(
                "Rx",
                MakeBoundCallback(&EndpointLayerRxCb, &target.dev)))
        {
            target.dev.connected = true;
        }

        Ptr<SatMac> mac = satDev->GetMac();
        if (mac &&
            mac->TraceConnectWithoutContext(
                "Rx",
                MakeBoundCallback(&EndpointLayerRxCb, &target.mac)))
        {
            target.mac.connected = true;
        }

        Ptr<SatPhy> phy = satDev->GetPhy();
        if (phy &&
            phy->TraceConnectWithoutContext(
                "Rx",
                MakeBoundCallback(&EndpointLayerRxCb, &target.phy)))
        {
            target.phy.connected = true;
        }
    }

    if (satDevCount == 0)
    {
        target.reason = "satnetdevice_missing";
    }
}

static void
InstallEndpointAppSink(Ptr<SimulationHelper>      simHelper,
                       Ptr<Node>                  node,
                       EndpointProbeTargetStats&  target,
                       double                     stopSec)
{
    if (!node)
    {
        if (target.reason.empty())
        {
            target.reason = "app_node_missing";
        }
        return;
    }

    Ptr<SatHelper> satHelper = simHelper->GetSatelliteHelper();
    Ipv4Address addr = satHelper->GetUserAddress(node);

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(addr, g_endpointProbe.port));
    ApplicationContainer sinkApps = sink.Install(node);
    target.app.installed = (sinkApps.GetN() > 0);
    if (!target.app.installed)
    {
        target.reason = "app_sink_install_failed";
        return;
    }

    target.app.sink = DynamicCast<PacketSink>(sinkApps.Get(0));
    target.app.traceConnected =
        sinkApps.Get(0)->TraceConnectWithoutContext(
            "Rx",
            MakeBoundCallback(&EndpointAppRxCb, &target.app));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec));
}

static void
InstallEndpointAppSinkAtAddress(Ptr<Node>                 node,
                                Ipv4Address               addr,
                                EndpointProbeTargetStats& target,
                                double                    stopSec)
{
    if (!node)
    {
        if (target.reason.empty())
        {
            target.reason = "app_node_missing";
        }
        return;
    }

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(addr, g_endpointProbe.port));
    ApplicationContainer sinkApps = sink.Install(node);
    target.app.installed = (sinkApps.GetN() > 0);
    if (!target.app.installed)
    {
        target.reason = "app_sink_install_failed";
        return;
    }

    target.app.sink = DynamicCast<PacketSink>(sinkApps.Get(0));
    target.app.traceConnected =
        sinkApps.Get(0)->TraceConnectWithoutContext(
            "Rx",
            MakeBoundCallback(&EndpointAppRxCb, &target.app));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec));
}

static std::string
InterpretEndpointTarget(const EndpointProbeTargetStats& target)
{
    if (target.notApplicable)
    {
        return "endpoint_not_applicable";
    }
    if (!target.active)
    {
        return "endpoint_not_applicable";
    }
    if (!target.phy.connected && !target.mac.connected && !target.dev.connected &&
        !target.app.installed)
    {
        return "probe_connection_incomplete";
    }
    if (target.app.sinkTotalRxBytes > 0 || target.app.traceRxBytes > 0)
    {
        return "app_delivery_observed";
    }
    if (target.dev.rxPkts > 0)
    {
        return target.app.installed ? "device_rx_observed_probe_app_idle"
                                    : "device_rx_observed_app_not_installed";
    }
    if (target.mac.rxPkts > 0)
    {
        return "mac_rx_observed_but_netdevice_rx_missing";
    }
    if (target.phy.rxPkts > 0)
    {
        return "phy_rx_observed_but_mac_rx_missing";
    }
    if (!target.phy.connected || !target.mac.connected || !target.dev.connected)
    {
        return "probe_connection_incomplete";
    }
    return "no_endpoint_observed";
}

static void
PrintEndpointTargetSummary(const EndpointProbeTargetStats& target)
{
    if (!target.active && !target.notApplicable)
    {
        return;
    }

    std::cout << "\n[target=" << target.label << "]\n";
    if (target.notApplicable)
    {
        std::cout << "endpointProbe=not_applicable reason=" << target.reason << "\n"
                  << "interpretation=" << InterpretEndpointTarget(target) << "\n";
        return;
    }

    std::cout << "phy connected=" << (target.phy.connected ? 1 : 0)
              << " rxPkts=" << target.phy.rxPkts
              << " rxBytes=" << target.phy.rxBytes << "\n";
    std::cout << "mac connected=" << (target.mac.connected ? 1 : 0)
              << " rxPkts=" << target.mac.rxPkts
              << " rxBytes=" << target.mac.rxBytes << "\n";
    std::cout << "dev connected=" << (target.dev.connected ? 1 : 0)
              << " rxPkts=" << target.dev.rxPkts
              << " rxBytes=" << target.dev.rxBytes << "\n";
    std::cout << "app installed=" << (target.app.installed ? 1 : 0)
              << " traceConnected=" << (target.app.traceConnected ? 1 : 0)
              << " rxPkts=" << target.app.rxPkts
              << " traceRxBytes=" << target.app.traceRxBytes
              << " sinkTotalRxBytes=" << target.app.sinkTotalRxBytes << "\n";
    if (!target.reason.empty())
    {
        std::cout << "reason=" << target.reason << "\n";
    }
    std::cout << "interpretation=" << InterpretEndpointTarget(target) << "\n";
}

static void
RefreshEndpointProbeAppTotals()
{
    auto refresh = [](EndpointProbeTargetStats& target) {
        if (target.app.sink)
        {
            target.app.sinkTotalRxBytes = target.app.sink->GetTotalRx();
        }
    };
    refresh(g_endpointProbe.ut);
    refresh(g_endpointProbe.gw);
}

static void
PrintEndpointProbeSummary()
{
    if (!g_endpointProbe.enabled)
    {
        return;
    }

    RefreshEndpointProbeAppTotals();

    std::cout << "\n=== Endpoint Probe Summary ===\n"
              << "pathType=" << g_endpointProbe.pathType << "\n"
              << "probePort=" << g_endpointProbe.port << "\n";
    PrintEndpointTargetSummary(g_endpointProbe.ut);
    PrintEndpointTargetSummary(g_endpointProbe.gw);
    PrintEndpointTargetSummary(g_endpointProbe.sat);
    std::cout << "==============================\n\n";
}

// --- ISL observer callback ---
// IslObsCb shares g_nodeToSatId with IslPacketDropCallback (both hook PacketDropRateTrace).
// NS3 TracedCallback supports multiple sinks, so both can connect to the same source.
// ConnectIslDropTrace() must run first to populate g_nodeToSatId before IslObsCb uses it.

static void
GatewayRxFeederCb(std::string key, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    // return feeder RX: gateway receives packets from satellite (SAT -> GW)
    auto& s = g_feederObsStats[key];
    s.txPkts++;
    s.rxPkts++;
    s.rxBytes += pkt->GetSize();

    uint64_t traceId = 0;
    if (TryGetPacketHopTraceId(pkt, traceId))
    {
        LogPacketHopTrace(traceId, "feeder_rx", key, "", pkt);
    }
}

static void
GatewayTxFeederCb(std::string key, Ptr<const Packet> pkt)
{
    // entry feeder TX: gateway sends packets toward satellite (GW -> SAT uplink).
    // TX-only diagnostic: do not increment rxPkts here. Routing-alignment verdicts
    // must be proven by real RX callbacks on scoped feeder/service/ISL observers.
    auto& s = g_feederObsStats[key];
    s.txPkts++;

    uint64_t traceId = 0;
    if (TryGetPacketHopTraceId(pkt, traceId))
    {
        LogPacketHopTrace(traceId, "feeder_tx", key, "", pkt);
    }
}

static void
GatewayFeederLinkDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // feeder_dn (SAT→GW) link propagation delay.
    // Source: SatNetDevice::RxLinkDelay on GW (m_rxLinkDelayTrace, satellite-net-device.h:249).
    // Fires once per packet received at GW feeder interface; delay = one-hop SAT→GW prop delay.
    auto& s = g_feederObsStats[key];
    s.sumDelayMs += delay.ToDouble(Time::MS);
    s.delaySamples++;
}

static void
UtServiceLinkDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // service FWD (SAT→UT) link propagation delay.
    // Source: SatNetDevice::RxLinkDelay on UT (m_rxLinkDelayTrace, satellite-net-device.h:249).
    // Fires once per packet received at UT; delay = one-hop SAT→UT prop delay.
    auto& s = g_serviceObsStats[key];
    s.sumDelayMs += delay.ToDouble(Time::MS);
    s.delaySamples++;
}

static void
GatewayDeviceRxDebugCb(std::string key, Ptr<const Packet> /*pkt*/, const Address& /*addr*/)
{
    g_gwDeviceRxHits[key]++;
}

static void
IslObsCb(uint32_t pktSize, Ptr<Node> srcNode, Ptr<Node> dstNode, bool dropped)
{
    if (!g_islNodeMapReady)
    {
        static bool warned = false;
        if (!warned)
        {
            std::cout << "[OBS][ISL] WARNING: node-to-sat map not initialized; "
                      << "ignoring ISL observer callback\n";
            warned = true;
        }
        return;
    }

    auto srcIt = g_nodeToSatId.find(srcNode);
    auto dstIt = g_nodeToSatId.find(dstNode);
    if (srcIt == g_nodeToSatId.end() || dstIt == g_nodeToSatId.end())
    {
        static bool warned = false;
        if (!warned)
        {
            std::cout << "[OBS][ISL] WARNING: callback node missing from node-to-sat map; "
                      << "ignoring ISL observer callback\n";
            warned = true;
        }
        return;
    }

    std::string key = std::to_string(srcIt->second) + "-" + std::to_string(dstIt->second);
    auto& s = g_islObsStats2[key];
    s.txPkts++;
    if (dropped)
    {
        s.dropPkts++;
    }
    else
    {
        s.rxPkts++;
        s.rxBytes += pktSize;
    }
}

static void
IslPropagationDelayCb(Ptr<const Packet> pkt,
                      Ptr<Node> srcNode,
                      Ptr<Node> dstNode,
                      Time delay)
{
    if (!g_islNodeMapReady)
    {
        return;
    }

    auto srcIt = g_nodeToSatId.find(srcNode);
    auto dstIt = g_nodeToSatId.find(dstNode);
    if (srcIt == g_nodeToSatId.end() || dstIt == g_nodeToSatId.end())
    {
        return;
    }

    std::string key = MakeIslKey(srcIt->second, dstIt->second);
    const double delayMs = delay.ToDouble(Time::MS);
    const uint32_t slot =
        static_cast<uint32_t>(std::floor(Simulator::Now().GetSeconds() / g_traceSlotIntervalSec));
    g_islPropagationStats[key].AddSample(delayMs);
    g_islPropagationStatsBySlot[slot][key].AddSample(delayMs);

    uint64_t traceId = 0;
    if (TryGetPacketHopTraceId(pkt, traceId))
    {
        std::ostringstream detail;
        detail << "link=" << key
               << " delayMs=" << std::fixed << std::setprecision(3) << delayMs;
        LogPacketHopTrace(traceId, "isl_hop", key, detail.str(), pkt);
    }
}

// --- Alert state machine: fires stdout events on link state transitions ---

static void
CheckAndAlertObs(const std::string& linkType,
                 const std::string& key,
                 const SegLinkStats& stats,
                 double              nowSec)
{
    if (!IsObsKeyInScope(linkType, key))
    {
        return;
    }

    double dr     = stats.DropRate();
    const std::string scopeKey = linkType + ":" + key;
    auto   prevIt = g_prevObsDropRate.find(scopeKey);
    double prevDr = (prevIt != g_prevObsDropRate.end()) ? prevIt->second : -1.0;

    // Drop rate crossed above threshold: report LINK DEGRADED
    if (dr >= g_obsCfg.dropAlertThreshPct && prevDr < g_obsCfg.dropAlertThreshPct)
    {
        std::cout << "[OBS][EVENT] t=" << std::fixed << std::setprecision(1) << nowSec
                  << "s  [" << linkType << "] " << key
                  << "  drop_rate=" << std::setprecision(1) << dr << "%"
                  << "  => LINK DEGRADED\n";
    }
    // Drop rate fell below threshold: report LINK RECOVERED
    else if (prevDr >= g_obsCfg.dropAlertThreshPct && dr < g_obsCfg.dropAlertThreshPct)
    {
        std::cout << "[OBS][EVENT] t=" << std::fixed << std::setprecision(1) << nowSec
                  << "s  [" << linkType << "] " << key
                  << "  drop_rate=" << std::setprecision(1) << dr << "%"
                  << "  => LINK RECOVERED\n";
    }

    // Throughput change detection (only checked after traffic warmup period)
    if (nowSec > g_obsCfg.trafficStartSec + g_obsCfg.snapshotIntervalSec)
    {
        double tput = stats.WindowThroughputKbps(nowSec);
        double prevTput = 0.0;
        auto prevTputIt = g_prevObsThroughputKbps.find(scopeKey);
        if (prevTputIt != g_prevObsThroughputKbps.end())
        {
            prevTput = prevTputIt->second;
        }

        if (prevTput > 1e-9 && tput < 1e-9)
        {
            std::cout << "[OBS][EVENT] t=" << std::fixed << std::setprecision(1) << nowSec
                      << "s  [" << linkType << "] " << key
                      << "  window_throughput=0 kbps  => POSSIBLE LINK FAILURE\n";
        }

        g_prevObsThroughputKbps[scopeKey] = tput;
    }

    g_prevObsDropRate[scopeKey] = dr;
}

// --- CSV log row writer ---

static void
WriteObsLogRow(const std::string&  linkType,
               const std::string&  key,
               const SegLinkStats& stats,
               double              nowSec)
{
    if (!IsObsKeyInScope(linkType, key))
    {
        return;
    }

    if (!g_obsLog.is_open())
    {
        return;
    }
    // CSV columns: time_s, link_type, link_id, rx_pkts, rx_bytes,
    //        tx_pkts, drop_pkts, drop_rate_pct, throughput_kbps, avg_delay_ms
    g_obsLog << std::fixed << std::setprecision(3)
             << nowSec                              << ","
             << linkType                            << ","
             << key                                 << ","
             << stats.rxPkts                        << ","
             << stats.rxBytes                       << ","
             << stats.txPkts                        << ","
             << stats.dropPkts                      << ","
             << stats.DropRate()                    << ","
             << stats.WindowThroughputKbps(nowSec)  << ","
             << stats.AvgDelayMs()                  << "\n";
}

// --- Periodic snapshot: called every snapshotIntervalSec during simulation ---

static void
TakeObsSnapshot()
{
    double nowSec = Simulator::Now().GetSeconds();

    for (auto& kv : g_feederObsStats)
    {
        CheckAndAlertObs("feeder", kv.first, kv.second, nowSec);
        WriteObsLogRow("feeder", kv.first, kv.second, nowSec);
        kv.second.BeginWindow(nowSec);
    }
    for (auto& kv : g_serviceObsStats)
    {
        CheckAndAlertObs("service", kv.first, kv.second, nowSec);
        WriteObsLogRow("service", kv.first, kv.second, nowSec);
        kv.second.BeginWindow(nowSec);
    }
    for (auto& kv : g_islObsStats2)
    {
        CheckAndAlertObs("isl", kv.first, kv.second, nowSec);
        WriteObsLogRow("isl", kv.first, kv.second, nowSec);
        kv.second.BeginWindow(nowSec);
    }

    // Reschedule next snapshot event
    Simulator::Schedule(Seconds(g_obsCfg.snapshotIntervalSec), &TakeObsSnapshot);
}

// --- Link observer trace connection ---
// Must run AFTER ConnectIslDropTrace() so ISL callback can look up g_nodeToSatId.

// Connect link observer traces for feeder / service / ISL segments.
//
// useOrbiterFeeder: connect SatOrbiterNetDevice::RxFeeder on each satellite.
//   Enabled only when the pathType verdict needs feeder-up visibility.
//
// useGwFeeder: connect SatNetDevice::Rx on GW nodes for return-feeder observation.
//   Use for: sat2gw (SAT→GW feeder downlink).
//   gw2gw_e2e does NOT connect GwFeeder; its verdict uses link-layer and packet-layer observers.
//
// useUtService: connect SatNetDevice::Rx on each UT node for FWD service-link observation.
//   Use for: sat2ut and gw2ut_e2e (SAT→UT forward direction).
//   OrbiterRxUserCb fires on RTN (UT→SAT) and cannot observe FWD delivery;
//   UtRxServiceCb must be used instead for these path types.
static void
ConnectLinkObserverTraces(bool useOrbiterFeeder, bool useGwFeeder, bool useUtService)
{
    NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();
    uint32_t      connFeeder{0}, connService{0}, connIsl{0};

    g_feederObsStats.clear();
    g_serviceObsStats.clear();
    g_islObsStats2.clear();
    g_islPropagationStats.clear();
    g_islPropagationStatsBySlot.clear();
    g_gwDeviceRxHits.clear();
    g_gwDeviceTypes.clear();
    std::set<const void*> connectedIslChannels;

    if (!g_islNodeMapReady)
    {
        std::cout << "[OBS][ISL] WARNING: node-to-sat map is not initialized; "
                  << "ISL observer trace connection skipped\n";
    }

    for (uint32_t i = 0; i < sats.GetN(); ++i)
    {
        Ptr<Node>   satNode = sats.Get(i);
        std::string satKey  = "sat" + std::to_string(i);

        for (uint32_t d = 0; d < satNode->GetNDevices(); ++d)
        {
            Ptr<SatOrbiterNetDevice> orbDev =
                DynamicCast<SatOrbiterNetDevice>(satNode->GetDevice(d));
            if (!orbDev)
            {
                continue;
            }

            // Orbiter-side feeder uplink trace: satellite receives packet from GW (FWD dir).
            if (useOrbiterFeeder &&
                orbDev->TraceConnectWithoutContext(
                    "RxFeeder",
                    MakeBoundCallback(&OrbiterRxFeederCb, satKey)))
            {
                ++connFeeder;
            }
            if (useOrbiterFeeder)
            {
                orbDev->TraceConnectWithoutContext(
                    "RxFeederLinkDelay",
                    MakeBoundCallback(&OrbiterFeederDelayCb, satKey));
            }

            if (orbDev->TraceConnectWithoutContext(
                    "RxUser",
                    MakeBoundCallback(&OrbiterRxUserCb, satKey)))
            {
                ++connService;
            }
            orbDev->TraceConnectWithoutContext(
                "RxUserLinkDelay",
                MakeBoundCallback(&OrbiterUserDelayCb, satKey));

            if (g_islNodeMapReady)
            {
                auto islDevices = orbDev->GetIslsNetDevices();
                for (auto& islDev : islDevices)
                {
                    if (islDev->TraceConnectWithoutContext(
                            "PacketDropRateTrace",
                            MakeCallback(&IslObsCb)))
                    {
                        ++connIsl;
                    }

                    Ptr<PointToPointIslChannel> islChannel =
                        DynamicCast<PointToPointIslChannel>(islDev->GetChannel());
                    const void* channelKey = PeekPointer(islChannel);
                    if (islChannel &&
                        connectedIslChannels.insert(channelKey).second)
                    {
                        islChannel->TraceConnectWithoutContext(
                            "PropagationDelayTrace",
                            MakeCallback(&IslPropagationDelayCb));
                    }
                }
            }

            if (useOrbiterFeeder)
            {
                // Initialize window baseline for per-interval throughput computation.
                g_feederObsStats[satKey].BeginWindow(0.0);
            }
            g_serviceObsStats[satKey].BeginWindow(0.0);
            break;
        }
    }

    if (useGwFeeder)
    {
        // Use actual GW count from topology instead of a hardcoded constant,
        // so this loop works correctly regardless of how many GWs were instantiated.
        uint32_t numGws = Singleton<SatTopology>::Get()->GetGwNodes().GetN();
        std::cout << "[OBS][GW] physical GW nodes from SatTopology: " << numGws << "\n";
        for (uint32_t gwId = 0; gwId < numGws; ++gwId)
        {
            NodeContainer gwNodes = GetPhysicalGwNodes(gwId);
            std::string   gwKey = MakeGwKey(gwId);
            uint32_t satDevCount = 0;

            for (uint32_t n = 0; n < gwNodes.GetN(); ++n)
            {
                Ptr<Node> gwNode = gwNodes.Get(n);
                if (!gwNode)
                {
                    continue;
                }

                for (uint32_t d = 0; d < gwNode->GetNDevices(); ++d)
                {
                    std::string devKey = gwKey + "/dev" + std::to_string(d);
                    g_gwDeviceTypes[devKey] =
                        gwNode->GetDevice(d)->GetInstanceTypeId().GetName();

                    if (g_obsDebug)
                    {
                        gwNode->GetDevice(d)->TraceConnectWithoutContext(
                            "Rx",
                            MakeBoundCallback(&GatewayDeviceRxDebugCb, devKey));
                    }

                    Ptr<SatNetDevice> satDev =
                        DynamicCast<SatNetDevice>(gwNode->GetDevice(d));
                    if (!satDev)
                    {
                        continue;
                    }

                    ++satDevCount;

                    // SAT→GW downlink (return feeder): observe at GW Rx side.
                    if (satDev->TraceConnectWithoutContext(
                            "Rx",
                            MakeBoundCallback(&GatewayRxFeederCb, gwKey)))
                    {
                        ++connFeeder;
                    }
                    // feeder_dn link propagation delay: SatNetDevice::RxLinkDelay fires
                    // with one-hop SAT→GW propagation delay (m_rxLinkDelayTrace).
                    satDev->TraceConnectWithoutContext(
                        "RxLinkDelay",
                        MakeBoundCallback(&GatewayFeederLinkDelayCb, gwKey));

                    // GW→SAT uplink (entry feeder): observe at MAC Tx side.
                    // SatNetDevice::GetTypeId() registers the MAC as attribute "SatMac"
                    // (Ptr<SatMac>). On a GW node the concrete type is SatGwMac, which
                    // owns the "Tx" trace source (confirmed: satellite-gw-mac.cc line 92).
                    // Direct object traversal (GetMac + DynamicCast) is used instead of a
                    // Config path to avoid the $ns3::TypeId casting issue.
                    {
                        std::string   gwTxKey = MakeGwTxKey(gwId);
                        Ptr<SatMac>   macBase = satDev->GetMac();
                        Ptr<SatGwMac> gwMac   = DynamicCast<SatGwMac>(macBase);
                        if (gwMac &&
                            gwMac->TraceConnectWithoutContext(
                                "Tx",
                                MakeBoundCallback(&GatewayTxFeederCb, gwTxKey)))
                        {
                            ++connFeeder;
                        }
                    }
                }
            }

            std::cout << "[OBS][GW] gwId=" << gwId
                      << " key=" << gwKey
                      << " satNetDevices=" << satDevCount
                      << " totalDevices=" << (gwNodes.GetN() > 0 ? gwNodes.Get(0)->GetNDevices() : 0)
                      << "\n";
            g_feederObsStats[gwKey].BeginWindow(0.0);
            g_feederObsStats[MakeGwTxKey(gwId)].BeginWindow(0.0);
        }
    }

    if (useUtService)
    {
        // Service link FWD: observe at UT Rx side (SAT→UT direction).
        // Map each UT user node back to its physical UT before looking for SatNetDevice.
        Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
        NodeContainer    allUtUsers = topo->GetUtUserNodes();
        for (uint32_t i = 0; i < allUtUsers.GetN(); ++i)
        {
            Ptr<Node>   utUserNode = allUtUsers.Get(i);
            Ptr<Node>   utNode     = topo->GetUtNode(utUserNode);
            std::string utKey      = MakeUtKey(i);

            if (!utNode)
            {
                std::cout << "[OBS][UT] WARNING: utUserId=" << i
                          << " has no physical UT node mapping\n";
                continue;
            }

            for (uint32_t d = 0; d < utNode->GetNDevices(); ++d)
            {
                Ptr<SatNetDevice> satDev =
                    DynamicCast<SatNetDevice>(utNode->GetDevice(d));
                if (!satDev)
                {
                    continue;
                }
                if (satDev->TraceConnectWithoutContext(
                        "Rx",
                        MakeBoundCallback(&UtRxServiceCb, utKey)))
                {
                    ++connService;
                }
                // service FWD link propagation delay: SatNetDevice::RxLinkDelay fires
                // with one-hop SAT→UT propagation delay (m_rxLinkDelayTrace).
                satDev->TraceConnectWithoutContext(
                    "RxLinkDelay",
                    MakeBoundCallback(&UtServiceLinkDelayCb, utKey));
                g_serviceObsStats[utKey].BeginWindow(0.0);
                break;
            }
        }
    }

    // Build the feederSource label to reflect which trace connections are active.
    std::string feederSrc;
    if (useOrbiterFeeder && useGwFeeder) feederSrc = "orbiter_rxfeeder+gw_rx";
    else if (useOrbiterFeeder)           feederSrc = "orbiter_rxfeeder";
    else if (useGwFeeder)                feederSrc = "gw_return_rx";
    else                                 feederSrc = "none";

    std::cout << "[OBS] build=2026-04-17-fullobs-v1"
              << " feederSource=" << feederSrc
              << "\n";
    std::cout << "[OBS] traces connected:"
              << "  feeder=" << connFeeder
              << "  service=" << connService
              << "  isl=" << connIsl
              << "\n";

    if (useGwFeeder && g_obsDebug)
    {
        for (const auto& kv : g_gwDeviceTypes)
        {
            std::cout << "[OBS][GWDEV] " << kv.first
                      << " type=" << kv.second << "\n";
        }
    }
}
// --- Final simulation summary printer ---

static void
PrintObsFinalSummary()
{
    std::cout << "\n=== E2E Link Observability Final Summary ===\n";
    std::cout << std::left
              << std::setw(24) << "link"
              << std::setw(10) << "rx_pkts"
              << std::setw(13) << "rx_bytes"
              << std::setw(11) << "drop_pkts"
              << std::setw(14) << "drop_rate(%)"
              << "avg_delay(ms)\n"
              << std::string(84, '-') << "\n";

    auto printSection = [](const std::string&                          label,
                           const std::map<std::string, SegLinkStats>& statsMap)
    {
        for (const auto& kv : statsMap)
        {
            if (!IsObsKeyInScope(label, kv.first))
            {
                continue;
            }
            std::string id = label + ":" + kv.first;
            // ISL delay is always "--" in this table because PacketDropRateTrace does not
            // carry delay info. ISL PHY propagation is printed separately below from
            // PointToPointIslChannel::PropagationDelayTrace.
            // Feeder/service delay is from RxFeederLinkDelay / RxUserLinkDelay traces.
            std::string delayStr = (kv.second.delaySamples > 0)
                ? ([&]() {
                      std::ostringstream oss;
                      oss << std::fixed << std::setprecision(2) << kv.second.AvgDelayMs();
                      return oss.str();
                  })()
                : std::string("--");

            std::cout << std::left
                      << std::setw(24) << id
                      << std::setw(10) << kv.second.rxPkts
                      << std::setw(13) << kv.second.rxBytes
                      << std::setw(11) << kv.second.dropPkts
                      << std::setw(14) << std::fixed << std::setprecision(2)
                      << kv.second.DropRate()
                      << delayStr << "\n";
        }
    };

    printSection("feeder", g_feederObsStats);
    printSection("service", g_serviceObsStats);
    printSection("isl",     g_islObsStats2);

    if (g_obsVerdictScope.activeIsl)
    {
        std::cout << std::string(84, '-') << "\n";
        std::cout << "ISL channel propagation delay trace (actual ns-3 scheduled delay)\n";
        for (const auto& kv : g_islPropagationStats)
        {
            if (!IsObsKeyInScope(g_obsVerdictScope, "isl", kv.first))
            {
                continue;
            }
            std::cout << "  isl:" << std::setw(12) << std::left << kv.first
                      << " samples=" << kv.second.samples
                      << " avg_ms=" << std::fixed << std::setprecision(3)
                      << kv.second.AvgDelayMs()
                      << " min_ms=" << kv.second.minDelayMs
                      << " max_ms=" << kv.second.maxDelayMs << "\n";
        }
    }

    if (g_obsDebug && !g_gwDeviceTypes.empty())
    {
        std::cout << std::string(84, '-') << "\n";
        std::cout << "GW device Rx debug hits\n";
        for (const auto& kv : g_gwDeviceTypes)
        {
            uint64_t hits = 0;
            auto hitIt = g_gwDeviceRxHits.find(kv.first);
            if (hitIt != g_gwDeviceRxHits.end())
            {
                hits = hitIt->second;
            }

            std::cout << "  " << kv.first
                      << " | " << kv.second
                      << " | rx_hits=" << hits << "\n";
        }
    }

    // Always print total OrbiterRxFeeder callback count for diagnostic traceability.
    std::cout << "[OBS][FEEDER-DIAG] total OrbiterRxFeeder callbacks (all sats, unscoped): "
              << g_totalOrbiterFeederRxCalls << "\n";

    if (g_obsScope.activeFeeder)
    {
        // Print any satellite that had feeder RX hits but was NOT in scope.
        // If these appear, the entry satellite computed by IslRoutingManager differs
        // from the actual satellite that received the feeder packet.
        bool hasOutOfScope = false;
        for (const auto& kv : g_feederObsStats)
        {
            if (kv.second.rxPkts > 0 && !IsObsKeyInScope("feeder", kv.first))
            {
                if (!hasOutOfScope)
                {
                    std::cout << "[OBS][FEEDER-DIAG] out-of-scope feeder hits (possible entry sat mismatch):\n";
                    hasOutOfScope = true;
                }
                std::cout << "  " << kv.first << " rxPkts=" << kv.second.rxPkts
                          << " rxBytes=" << kv.second.rxBytes << "\n";
            }
        }
    }

    std::cout << std::string(84, '-') << "\n";
    std::cout << "Log: " << g_obsCfg.logFilePath << "\n";
    std::cout << "=============================================\n\n";

    if (g_obsLog.is_open())
    {
        g_obsLog.flush();
        g_obsLog.close();
    }
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

enum class E2ESegment
{
    FEEDERLINK,
    ISL,
    SERVICELINK
};

static const char*
ToString(E2ESegment segment)
{
    switch (segment)
    {
    case E2ESegment::FEEDERLINK:
        return "feederlink";
    case E2ESegment::ISL:
        return "isl";
    case E2ESegment::SERVICELINK:
        return "servicelink";
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
    std::string pathType{"gw2gw_e2e"};
    double      simTimeSec{0.0};

    E2ESegmentConfig feederlink{};
    E2ESegmentConfig isl{};
    E2ESegmentConfig servicelink{};

    uint32_t satSrc{0};
    uint32_t satDst{0};
    uint32_t gwSrc{0};
    uint32_t gwDst{0};
    uint32_t gwId{0};
    uint32_t numSats{0};
    uint32_t utId{0};
    uint32_t trafficUtUserId{0};
    bool     trafficUtUserIdResolved{false};
    double   utLatDeg{0.0};
    double   utLonDeg{0.0};
    std::string utName;
    std::string regenerationMode{"network"};
    std::string rtnConfFilePath;
};

enum class TrafficKind
{
    GW_UT_ALL,
    GW_UT_SELECTED,
    ISL_BACKGROUND,
    GW2GW_APPLICATION,
    SAT_UT_APPLICATION,
    SAT_SAT_APPLICATION
};

enum class E2EReportKind
{
    PATH_ONLY,
    SAT2SAT_REPORT,
    GW2GW_REPORT,
    GW2UT_REPORT
};

enum class ObsFeederMode
{
    PHY,
    ROUTING,
    NONE
};

struct PathTypeSpec
{
    std::string   pathType;
    bool          usesFeederlink;
    bool          usesIsl;
    bool          usesServicelink;
    bool          needsGwId;
    bool          needsGwPair;
    bool          needsUt;
    bool          includesIsl;
    ObsFeederMode obsFeederMode;
    E2EReportKind reportKind;
};

struct PathTypePlan
{
    PathTypeSpec spec;
    TrafficKind  trafficKind{TrafficKind::GW_UT_ALL};
    uint32_t     edgeGatewayId{0};
};

static PathTypeSpec GetPathTypeSpec(const std::string& pathType);

// === Scenario / Endpoint Discovery ==========================================

struct GatewayPreset
{
    uint32_t    id;
    double      latDeg;
    double      lonDeg;
    std::string name;
};

static const std::vector<GatewayPreset>&
GetGatewayPresets()
{
    // Coordinates aligned with Iridium-66 scenario gw_positions.txt.
    // gwId order matches line order in the positions file.
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

static std::map<uint32_t, uint32_t>
BuildBeamToLogicalGwIdMap(const std::string& rtnConfFilePath)
{
    std::map<uint32_t, uint32_t> beamToGwId;

    std::ifstream in(rtnConfFilePath);
    if (!in.is_open())
    {
        return beamToGwId;
    }

    uint32_t beam = 0;
    uint32_t userChannel = 0;
    uint32_t gwIdFromFile = 0;
    uint32_t feederChannel = 0;
    while (in >> beam >> userChannel >> gwIdFromFile >> feederChannel)
    {
        if (beam == 0 || gwIdFromFile == 0)
        {
            continue;
        }
        beamToGwId[beam] = gwIdFromFile - 1;
    }

    return beamToGwId;
}

static bool
TryExtractFeederBeamId(Ipv4Address addr, uint32_t& beamId)
{
    uint32_t raw = addr.Get();
    const uint32_t oct1 = (raw >> 24) & 0xFF;
    const uint32_t oct2 = (raw >> 16) & 0xFF;
    if (oct1 != 40 || oct2 == 0)
    {
        return false;
    }

    beamId = oct2;
    return true;
}

// Builds the set of SNS3 beam IDs to activate during network bootstrap.
//
// When includeAllFeederBeams=false (default): enables one representative beam per GW —
// just enough to register each physical GW node in SatTopology. Feeder channels to
// other satellites are not active; only the representative satellite's channel works.
//
// When includeAllFeederBeams=true (gw2gw_e2e): enables ALL beams for every GW in
// gatewayCount. This is required because the entry/exit satellite for each slot is
// determined by IslRoutingManager AFTER bootstrap, creating a chicken-and-egg problem:
// we cannot know which feeder channels to activate until routes are computed, but routes
// are computed after the network (channels) are created. Activating all feeder beams for
// the relevant GWs (2 × 66 = 132 beams for gw2gw_e2e) guarantees any entry/exit sat can
// receive/transmit packets. Without this, feeder channels for off-bootstrap satellites
// (e.g., sat45 = beam 46, not in {1,2,72}) silently drop all packets at the SatNetDevice.
static std::set<uint32_t>
BuildGatewayBootstrapBeamSet(const std::string& rtnConfFilePath,
                             const std::set<uint32_t>& targetGwIds,
                             uint32_t           requestedBeamId,
                             bool               includeAllFeederBeams = false)
{
    std::set<uint32_t> beams;
    std::set<uint32_t> coveredGatewayIds;
    if (targetGwIds.empty())
    {
        beams.insert(requestedBeamId);
    }

    std::ifstream in(rtnConfFilePath);
    if (!in.is_open())
    {
        std::cout << "[TOPO_BOOTSTRAP] WARNING: cannot open " << rtnConfFilePath
                  << "; only requested beam " << requestedBeamId << " will be enabled\n";
        return beams;
    }

    uint32_t beam = 0;
    uint32_t userChannel = 0;
    uint32_t gwIdFromFile = 0;
    uint32_t feederChannel = 0;
    while (in >> beam >> userChannel >> gwIdFromFile >> feederChannel)
    {
        // Use gwIdFromFile directly: satellite-helper.cc DoCreateScenario() assigns GW nodes
        // via gwNodes.Get(rtnConf[GW_ID_INDEX] - 1), i.e. the file column is used as-is.
        // gatewayCount controls how many leading GW IDs to activate (e.g. 2 → GW1+GW2 only).
        if (gwIdFromFile == 0)
        {
            continue;
        }

        uint32_t gwIdZeroBased = gwIdFromFile - 1;
        if (!targetGwIds.empty() && targetGwIds.count(gwIdZeroBased) == 0)
        {
            continue;
        }

        if (includeAllFeederBeams)
        {
            // Activate every beam belonging to the relevant GWs so that all feeder
            // channels are live. Required for gw2gw_e2e: the routing manager selects
            // entry/exit satellites dynamically per slot after network creation, so we
            // cannot predict in advance which feeder channels will be needed.
            beams.insert(beam);
            coveredGatewayIds.insert(gwIdZeroBased);
        }
        else
        {
            // Minimal bootstrap: one representative beam per GW, then stop.
            if (coveredGatewayIds.insert(gwIdZeroBased).second)
            {
                beams.insert(beam);
            }
            if (!targetGwIds.empty() &&
                coveredGatewayIds.size() == targetGwIds.size())
            {
                break;
            }
        }
    }

    if (!targetGwIds.empty() && coveredGatewayIds.size() != targetGwIds.size())
    {
        std::cout << "[TOPO_BOOTSTRAP] WARNING: covered " << coveredGatewayIds.size()
                  << "/" << targetGwIds.size() << " gateway ids from " << rtnConfFilePath << "\n";
    }

    return beams;
}

static std::string
FormatBeamSet(const std::set<uint32_t>& beams)
{
    std::ostringstream oss;
    bool first = true;
    for (uint32_t beam : beams)
    {
        if (!first)
        {
            oss << ",";
        }
        oss << beam;
        first = false;
    }
    return oss.str();
}

static std::set<uint32_t>
DetermineTargetGwIds(const std::string& pathType,
                     uint32_t           gwId,
                     uint32_t           gwSrc,
                     uint32_t           gwDst)
{
    const PathTypeSpec spec = GetPathTypeSpec(pathType);
    std::set<uint32_t> targetGwIds;
    if (spec.needsGwPair)
    {
        targetGwIds.insert(gwSrc);
        targetGwIds.insert(gwDst);
    }
    else if (spec.needsGwId)
    {
        targetGwIds.insert(gwId);
    }
    return targetGwIds;
}

static uint32_t
DetermineRequiredGwNodeCount(const std::set<uint32_t>& targetGwIds)
{
    if (targetGwIds.empty())
    {
        return 1;
    }
    return (*targetGwIds.rbegin()) + 1;
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

static NodeContainer
GetUtUsers()
{
    return Singleton<SatTopology>::Get()->GetUtUserNodes();
}

// Returns a single-element container for one specific UT user node.
// utId maps to the index in SatTopology::GetUtUserNodes().
// Use this for sat2ut and gw2ut_e2e service-link traffic to avoid
// injecting load from all 91 scenario UTs onto the observed satellite.
static NodeContainer
GetSelectedUtUser(uint32_t utId)
{
    NodeContainer allUts = Singleton<SatTopology>::Get()->GetUtUserNodes();
    NS_ABORT_MSG_IF(utId >= allUts.GetN(),
                    "utId=" << utId << " >= utUserNodes.GetN()=" << allUts.GetN());
    return NodeContainer(allUts.Get(utId));
}

static uint32_t
ResolveTrafficUtUserId(Ptr<IslRoutingManager> routingMgr,
                       const E2EConfig&       cfg,
                       uint32_t               numSlots)
{
    if (cfg.pathType != "sat2ut" && cfg.pathType != "gw2ut_e2e")
    {
        return cfg.utId;
    }

    uint32_t servingSatId = UINT32_MAX;
    for (uint32_t k = 0; k < numSlots; ++k)
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, k);
        if (r.valid)
        {
            servingSatId = r.servingSatId;
            break;
        }
    }

    if (servingSatId == UINT32_MAX)
    {
        std::cout << "[UT_SELECT] no valid GW-UT route; using requested utUserId="
                  << cfg.utId << "\n";
        return cfg.utId;
    }

    Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
    NodeContainer    utUsers = topo->GetUtUserNodes();
    for (uint32_t i = 0; i < utUsers.GetN(); ++i)
    {
        Ptr<Node> utNode = topo->GetUtNode(utUsers.Get(i));
        if (!utNode)
        {
            continue;
        }

        if (topo->GetUtSatId(utNode) == servingSatId)
        {
            std::cout << "[UT_SELECT] logicalUtId=" << cfg.utId
                      << " routeServingSat=" << servingSatId
                      << " trafficUtUserId=" << i
                      << " requestedUtUserId=" << cfg.utId << "\n";
            return i;
        }
    }

    std::cout << "[UT_SELECT] WARNING: no scenario UT user found on servingSat="
              << servingSatId << "; using requested utUserId=" << cfg.utId << "\n";
    return cfg.utId;
}

// === E2E Planning Helpers ===================================================

static std::string
NormalizePathType(std::string pathType)
{
    if (pathType == "gw2ut") return "gw2ut_e2e";
    if (pathType == "gw2gw") return "gw2gw_e2e";
    return pathType;
}

static PathTypeSpec
GetPathTypeSpec(const std::string& pathType)
{
    if (pathType == "gw2sat")
    {
        return {"gw2sat", true, false, false, true, false, false, false,
                ObsFeederMode::PHY, E2EReportKind::PATH_ONLY};
    }
    if (pathType == "sat2sat")
    {
        return {"sat2sat", false, true, false, false, false, false, true,
                ObsFeederMode::NONE, E2EReportKind::SAT2SAT_REPORT};
    }
    if (pathType == "sat2ut")
    {
        return {"sat2ut", false, false, true, true, false, true, false,
                ObsFeederMode::NONE, E2EReportKind::PATH_ONLY};
    }
    if (pathType == "sat2gw")
    {
        return {"sat2gw", true, false, false, true, false, false, false,
                ObsFeederMode::PHY, E2EReportKind::PATH_ONLY};
    }
    if (pathType == "gw2ut_e2e")
    {
        return {"gw2ut_e2e", true, true, true, true, false, true, true,
                ObsFeederMode::PHY, E2EReportKind::GW2UT_REPORT};
    }
    if (pathType == "gw2gw_e2e")
    {
        // usesServicelink=false: gw2gw path does not involve a UT service link.
        // The feeder segment covers both GW-SAT legs (entry and exit).
        // obsFeederMode=PHY: connect OrbiterRxFeeder + RxFeederLinkDelay trace on each
        // satellite to measure actual feeder uplink delay from packet timestamps.
        // In REGENERATION_NETWORK the counter reflects L2 reception, not L3 forward
        // completion; cross-check with ISL_LAYER and packet-layer verdict for full confidence.
        return {"gw2gw_e2e", true, true, false, false, true, false, true,
                ObsFeederMode::PHY, E2EReportKind::GW2GW_REPORT};
    }

    NS_ABORT_MSG("Unsupported pathType=" << pathType);
    return {"", false, false, false, false, false, false, false,
            ObsFeederMode::NONE, E2EReportKind::PATH_ONLY};
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

static const char*
ToString(ObsFeederMode mode)
{
    switch (mode)
    {
    case ObsFeederMode::PHY:
        return "PHY";
    case ObsFeederMode::ROUTING:
        return "ROUTING";
    case ObsFeederMode::NONE:
        return "NONE";
    }
    return "UNKNOWN";
}

static void
ValidateE2EConfig(const E2EConfig& cfg)
{
    const PathTypeSpec spec = GetPathTypeSpec(cfg.pathType);

    NS_ABORT_MSG_IF(cfg.regenerationMode != "network" &&
                    cfg.regenerationMode != "phy" &&
                    cfg.regenerationMode != "transparent",
                    "regenerationMode must be network, phy, or transparent");

    if (cfg.pathType == "sat2sat")
    {
        NS_ABORT_MSG_IF(cfg.satSrc == cfg.satDst,
                        "satSrc and satDst must be different in sat2sat pathType");
    }

    if (spec.needsGwPair)
    {
        NS_ABORT_MSG_IF(cfg.gwSrc == cfg.gwDst,
                        "gwSrc and gwDst must be different in gw2gw_e2e pathType");
        NS_ABORT_MSG_IF(FindGatewayPreset(cfg.gwSrc) == nullptr,
                        "Unknown gwSrc=" << cfg.gwSrc);
        NS_ABORT_MSG_IF(FindGatewayPreset(cfg.gwDst) == nullptr,
                        "Unknown gwDst=" << cfg.gwDst);
    }

    if (spec.needsUt)
    {
        NS_ABORT_MSG_IF(cfg.utName.empty(), "utName must not be empty for path types using UT");
    }

    if (cfg.pathType == "sat2ut")
    {
        NS_ABORT_MSG_IF(cfg.regenerationMode == "transparent",
                        "sat2ut service-link-only requires regenerative payload; "
                        "regenerationMode=transparent is not 3GPP Scenario D");
    }
}

static PathTypePlan
BuildPathTypePlan(E2EConfig& cfg)
{
    ValidateE2EConfig(cfg);
    const PathTypeSpec spec = GetPathTypeSpec(cfg.pathType);

    cfg.feederlink.enabled = spec.usesFeederlink;
    cfg.isl.enabled = spec.usesIsl;
    cfg.servicelink.enabled = spec.usesServicelink;

    PathTypePlan plan;
    plan.spec = spec;
    plan.edgeGatewayId = spec.needsGwPair ? cfg.gwSrc : cfg.gwId;

    if (cfg.pathType == "sat2sat")
    {
        plan.trafficKind = TrafficKind::SAT_SAT_APPLICATION;
    }
    else if (cfg.pathType == "sat2ut")
    {
        cfg.servicelink.traffic.enableFwd = true;
        cfg.servicelink.traffic.enableRtn = false;
        plan.trafficKind = TrafficKind::SAT_UT_APPLICATION;
    }
    else if (cfg.pathType == "gw2ut_e2e")
    {
        plan.trafficKind = TrafficKind::GW_UT_SELECTED;
    }
    else if (cfg.pathType == "gw2gw_e2e")
    {
        plan.trafficKind = TrafficKind::GW2GW_APPLICATION;
    }
    else if (cfg.pathType == "sat2gw")
    {
        // sat2gw observer (GatewayRxFeederCb) fires ONLY on SAT→GW feeder downlink (RTN).
        // FWD traffic (GW→SAT→UT) never produces packets at the GW feeder RX side,
        // so GatewayRxFeederCb would always return rxPkts=0 if only FWD runs.
        // Force enableRtn=true, enableFwd=false regardless of caller-supplied TrafficConfig.
        cfg.feederlink.traffic.enableFwd = false;
        cfg.feederlink.traffic.enableRtn = true;
        plan.trafficKind = TrafficKind::GW_UT_ALL;
    }
    else
    {
        plan.trafficKind = TrafficKind::GW_UT_ALL;
    }

    return plan;
}

static const char*
ToString(TrafficKind kind)
{
    switch (kind)
    {
    case TrafficKind::GW_UT_ALL:
        return "gw_ut_all";
    case TrafficKind::GW_UT_SELECTED:
        return "gw_ut_selected";
    case TrafficKind::ISL_BACKGROUND:
        return "isl_background";
    case TrafficKind::GW2GW_APPLICATION:
        return "gw2gw_application";
    case TrafficKind::SAT_UT_APPLICATION:
        return "sat_ut_application";
    case TrafficKind::SAT_SAT_APPLICATION:
        return "sat_sat_application";
    }
    return "unknown";
}

static void
PrintE2ERunBanner(const E2EConfig& cfg, const PathTypePlan& plan)
{
    std::cout << "\n[E2E] pathType=" << cfg.pathType
              << " includesIsl=" << (plan.spec.includesIsl ? "yes" : "no")
              << " regenerationMode=" << cfg.regenerationMode
              << " segments={"
              << "feederlink=" << (cfg.feederlink.enabled ? "on" : "off") << ", "
              << "isl=" << (cfg.isl.enabled ? "on" : "off") << ", "
              << "servicelink=" << (cfg.servicelink.enabled ? "on" : "off") << "}"
              << " traffic=" << ToString(plan.trafficKind)
              << " logicalUtId=" << cfg.utId
              << " trafficUtUserId=" << cfg.trafficUtUserId
              << "\n";
}

static void
ActivateSatEndpointNotApplicable(const std::string& reason)
{
    g_endpointProbe.sat = {};
    g_endpointProbe.sat.active = true;
    g_endpointProbe.sat.notApplicable = true;
    g_endpointProbe.sat.label = "sat";
    g_endpointProbe.sat.reason = reason;
}

static Ptr<Node>
GetUtUserNodeOrNull(uint32_t utId)
{
    NodeContainer uts = Singleton<SatTopology>::Get()->GetUtUserNodes();
    if (utId >= uts.GetN())
    {
        return nullptr;
    }
    return uts.Get(utId);
}

static Ptr<Node>
GetPhysicalUtNodeOrNull(Ptr<Node> utUserNode)
{
    if (!utUserNode)
    {
        return nullptr;
    }
    return Singleton<SatTopology>::Get()->GetUtNode(utUserNode);
}

static void
RefreshLogicalGwNodeMap(const std::string&         rtnConfFilePath,
                        const std::set<uint32_t>&  targetGwIds)
{
    g_logicalGwNodeMap.clear();

    NodeContainer gws = Singleton<SatTopology>::Get()->GetGwNodes();
    if (gws.GetN() == 0)
    {
        std::cout << "[GW_MAP] WARNING: no physical GW nodes exist in topology\n";
        return;
    }

    const std::map<uint32_t, uint32_t> beamToGwId =
        BuildBeamToLogicalGwIdMap(rtnConfFilePath);
    if (!beamToGwId.empty())
    {
        for (uint32_t i = 0; i < gws.GetN(); ++i)
        {
            Ptr<Node> gwNode = gws.Get(i);
            Ptr<Ipv4> ipv4 = gwNode ? gwNode->GetObject<Ipv4>() : nullptr;
            if (!ipv4)
            {
                continue;
            }

            std::map<uint32_t, uint32_t> gwHitCounts;
            for (uint32_t ifIndex = 0; ifIndex < ipv4->GetNInterfaces(); ++ifIndex)
            {
                if (!ipv4->IsUp(ifIndex) || ipv4->GetNAddresses(ifIndex) == 0)
                {
                    continue;
                }

                uint32_t beamId = 0;
                Ipv4Address addr = ipv4->GetAddress(ifIndex, 0).GetLocal();
                if (!TryExtractFeederBeamId(addr, beamId))
                {
                    continue;
                }

                auto beamIt = beamToGwId.find(beamId);
                if (beamIt != beamToGwId.end())
                {
                    gwHitCounts[beamIt->second]++;
                }
            }

            if (gwHitCounts.empty())
            {
                continue;
            }

            auto bestIt = std::max_element(
                gwHitCounts.begin(),
                gwHitCounts.end(),
                [](const auto& a, const auto& b) { return a.second < b.second; });

            g_logicalGwNodeMap[bestIt->first] = gwNode;
            std::cout << "[GW_MAP] logicalGwId=" << bestIt->first
                      << " -> nodeId=" << gwNode->GetId()
                      << " matchedFeederIfs=" << bestIt->second << "\n";
        }
    }
    else
    {
        std::cout << "[GW_MAP] WARNING: beam-to-GW map is empty for "
                  << rtnConfFilePath << "\n";
    }

    // Sanity check: Tier 1 should not assign the same physical node to two logical GWs.
    // This can happen when one physical node has feeder interfaces for multiple logical GWs
    // and wins the max_element vote for both. Log a warning so the issue is visible.
    {
        std::map<Ptr<Node>, uint32_t> nodeToGwId;
        for (const auto& kv : g_logicalGwNodeMap)
        {
            auto ins = nodeToGwId.emplace(kv.second, kv.first);
            if (!ins.second)
            {
                std::cout << "[GW_MAP] WARNING: nodeId=" << kv.second->GetId()
                          << " shared by logicalGwId=" << ins.first->second
                          << " and logicalGwId=" << kv.first
                          << " — Tier 1 conflict, Tier 3 will resolve\n";
            }
        }
    }

    // Fallback 1: if the number of physical GW nodes matches the number of target logical
    // GWs, assume the scenario created them in the same order as the requested GW IDs.
    if (!targetGwIds.empty() && gws.GetN() == targetGwIds.size())
    {
        uint32_t nodeIndex = 0;
        for (uint32_t logicalGwId : targetGwIds)
        {
            if (g_logicalGwNodeMap.count(logicalGwId) == 0 && nodeIndex < gws.GetN())
            {
                Ptr<Node> gwNode = gws.Get(nodeIndex);
                g_logicalGwNodeMap[logicalGwId] = gwNode;
                std::cout << "[GW_MAP] fallback ordered map logicalGwId="
                          << logicalGwId << " -> nodeId=" << gwNode->GetId()
                          << " physicalIndex=" << nodeIndex << "\n";
            }
            ++nodeIndex;
        }
    }

    // Fallback 2: for contiguous leading GW IDs, use the physical container index only
    // when the direct logical index is in range and still unmapped.
    // NOTE: this may assign the same physical node to multiple logical GW IDs when
    // physicalGwNodes < targetGwIds.size() (e.g. gwId=0 and gwId=1 both → gws.Get(0)).
    // In such cases, gw2gw traffic is differentiated by IP endpoint (src/dst feeder IPs
    // on the same physical node), not by separate physical nodes.
    for (uint32_t logicalGwId : targetGwIds)
    {
        if (g_logicalGwNodeMap.count(logicalGwId) == 0 && logicalGwId < gws.GetN())
        {
            Ptr<Node> gwNode = gws.Get(logicalGwId);
            g_logicalGwNodeMap[logicalGwId] = gwNode;
            std::cout << "[GW_MAP] fallback index map logicalGwId="
                      << logicalGwId << " -> nodeId=" << gwNode->GetId()
                      << " physicalIndex=" << logicalGwId << "\n";
        }
    }
}

static Ptr<Node>
GetPhysicalGwNodeOrNull(uint32_t gwId)
{
    auto mapped = g_logicalGwNodeMap.find(gwId);
    if (mapped != g_logicalGwNodeMap.end())
    {
        return mapped->second;
    }
    std::cout << "[GW_MAP] WARNING: no physical GW node mapped for logicalGwId="
              << gwId << "\n";
    return nullptr;
}

static NodeContainer
GetPhysicalGwNodes(uint32_t gwId)
{
    Ptr<Node> physicalGw = GetPhysicalGwNodeOrNull(gwId);
    return physicalGw ? NodeContainer(physicalGw) : NodeContainer();
}

static bool
TryGetPhysicalGwRoutableIp(Ptr<Node> node, uint32_t gwId, Ipv4Address& out)
{
    NS_ABORT_MSG_IF(!node, "[GW] physical GW node missing for gwId=" << gwId);

    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        std::cout << "[GW] physical GW node has no IPv4 stack (gwId=" << gwId << ")\n";
        return false;
    }

    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        if (!ipv4->IsUp(i) || ipv4->GetNAddresses(i) == 0)
        {
            continue;
        }
        Ipv4Address addr = ipv4->GetAddress(i, 0).GetLocal();
        if (addr == Ipv4Address("127.0.0.1") || addr == Ipv4Address("0.0.0.0"))
        {
            continue;
        }
        std::cout << "[GW] physical GW" << gwId
                  << " routable IP=" << addr
                  << " ifIndex=" << i << "\n";
        out = addr;
        return true;
    }

    std::cout << "[GW] no routable IPv4 address on physical GW node (gwId=" << gwId << ")\n";
    return false;
}

static bool
TryGetNodeRoutableIp(Ptr<Node> node, const std::string& label, Ipv4Address& out)
{
    if (!node)
    {
        return false;
    }

    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        std::cout << "[" << label << "] node has no IPv4 stack\n";
        return false;
    }

    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        if (!ipv4->IsUp(i) || ipv4->GetNAddresses(i) == 0)
        {
            continue;
        }

        Ipv4Address addr = ipv4->GetAddress(i, 0).GetLocal();
        if (addr == Ipv4Address("127.0.0.1") || addr == Ipv4Address("0.0.0.0"))
        {
            continue;
        }

        out = addr;
        return true;
    }

    std::cout << "[" << label << "] no routable IPv4 address found\n";
    return false;
}

static GwFeederInterfaceInfo
ResolveGwFeederInterface(Ptr<Node> node, uint32_t gwId, Ipv4Address addr)
{
    NS_ABORT_MSG_IF(!node, "[GW] physical GW node missing for gwId=" << gwId);

    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    NS_ABORT_MSG_IF(!ipv4, "[GW] physical GW node has no IPv4 stack (gwId=" << gwId << ")");

    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        if (ipv4->GetNAddresses(i) == 0 || ipv4->GetAddress(i, 0).GetLocal() != addr)
        {
            continue;
        }

        Ptr<NetDevice> netDevice = ipv4->GetNetDevice(i);
        Ptr<SatNetDevice> satDevice = DynamicCast<SatNetDevice>(netDevice);
        NS_ABORT_MSG_IF(!netDevice,
                        "[GW] no NetDevice bound to gwId=" << gwId
                        << " ifIndex=" << i << " addr=" << addr);
        NS_ABORT_MSG_IF(!satDevice,
                        "[GW] gwId=" << gwId
                        << " ifIndex=" << i
                        << " addr=" << addr
                        << " is not backed by SatNetDevice");

        GwFeederInterfaceInfo info;
        info.ifIndex = i;
        info.address = addr;
        info.netDevice = netDevice;
        info.satDevice = satDevice;
        return info;
    }

    NS_ABORT_MSG("[GW] cannot resolve feeder interface for gwId="
                 << gwId << " addr=" << addr);
    return {};
}

static void
PrefillArpEntry(Ptr<Node> node,
                uint32_t gwId,
                const GwFeederInterfaceInfo& srcFeeder,
                Ipv4Address dstAddr,
                Mac48Address dstMac)
{
    NS_ABORT_MSG_IF(!node, "[GW2GW_APP] source physical GW node missing for gwId=" << gwId);

    Ptr<Ipv4L3Protocol> ipv4 = node->GetObject<Ipv4L3Protocol>();
    NS_ABORT_MSG_IF(!ipv4,
                    "[GW2GW_APP] no Ipv4L3Protocol on source GW" << gwId);

    Ptr<Ipv4Interface> iface = ipv4->GetInterface(srcFeeder.ifIndex);
    NS_ABORT_MSG_IF(!iface,
                    "[GW2GW_APP] no Ipv4Interface for GW" << gwId
                    << " feeder ifIndex=" << srcFeeder.ifIndex);

    Ptr<ArpCache> arpCache = iface->GetArpCache();
    if (!arpCache)
    {
        arpCache = CreateObject<ArpCache>();
        // SetDevice is required: without it cache->GetDevice() returns null, which causes
        // NS_ASSERT(device) crash in ArpL3Protocol::SendArpRequest if ARP is ever triggered
        // on this interface for any IP that doesn't have a permanent entry.
        arpCache->SetDevice(srcFeeder.netDevice, iface);
        iface->SetArpCache(arpCache);
    }

    ArpCache::Entry* arpEntry = arpCache->Lookup(dstAddr);
    if (!arpEntry)
    {
        arpEntry = arpCache->Add(dstAddr);
    }
    NS_ABORT_MSG_IF(!arpEntry,
                    "[GW2GW_APP] failed to allocate ARP entry for dst=" << dstAddr);

    arpEntry->SetMacAddress(dstMac);
    arpEntry->MarkPermanent();
}

static void
ActivateUtEndpointProbe(Ptr<SimulationHelper> simHelper,
                        uint32_t              utId,
                        double                stopSec)
{
    g_endpointProbe.ut = {};
    g_endpointProbe.ut.active = true;
    g_endpointProbe.ut.label = "ut" + std::to_string(utId);

    Ptr<Node> utUserNode = GetUtUserNodeOrNull(utId);
    if (!utUserNode)
    {
        g_endpointProbe.ut.reason = "ut_user_node_missing";
        return;
    }

    Ptr<Node> physicalUtNode = GetPhysicalUtNodeOrNull(utUserNode);
    if (physicalUtNode)
    {
        ConnectEndpointDeviceProbe(physicalUtNode, g_endpointProbe.ut);
    }
    else
    {
        g_endpointProbe.ut.reason = "physical_ut_node_missing";
    }
    InstallEndpointAppSink(simHelper, utUserNode, g_endpointProbe.ut, stopSec);
}

static void
ActivateGwEndpointProbe(Ptr<SimulationHelper> /*simHelper*/,
                        uint32_t              gwId,
                        bool                  installAppSink,
                        double                stopSec)
{
    g_endpointProbe.gw = {};
    g_endpointProbe.gw.active = true;
    g_endpointProbe.gw.label = "gw" + std::to_string(gwId);

    Ptr<Node> physicalGw = GetPhysicalGwNodeOrNull(gwId);
    if (physicalGw)
    {
        ConnectEndpointDeviceProbe(physicalGw, g_endpointProbe.gw);
    }
    else
    {
        g_endpointProbe.gw.reason = "physical_gw_node_missing";
    }

    if (!installAppSink)
    {
        return;
    }

    if (!physicalGw)
    {
        if (g_endpointProbe.gw.reason.empty())
        {
            g_endpointProbe.gw.reason = "physical_gw_node_missing";
        }
        return;
    }
    Ipv4Address gwAddr;
    if (!TryGetPhysicalGwRoutableIp(physicalGw, gwId, gwAddr))
    {
        g_endpointProbe.gw.reason = "physical_gw_routable_ip_missing";
        return;
    }
    InstallEndpointAppSinkAtAddress(physicalGw, gwAddr, g_endpointProbe.gw, stopSec);
}

static void
InstallEndpointProbe(Ptr<SimulationHelper> simHelper,
                     const E2EConfig&      cfg,
                     bool                  enabled,
                     uint16_t              probePort)
{
    g_endpointProbe = {};
    g_endpointProbe.enabled = enabled;
    g_endpointProbe.port = probePort;
    g_endpointProbe.pathType = cfg.pathType;

    if (!enabled)
    {
        return;
    }

    const double stopSec = cfg.simTimeSec;
    std::cout << "[ENDPOINT_PROBE] enabled pathType=" << cfg.pathType
              << " probePort=" << probePort << "\n";

    if (cfg.pathType == "gw2sat")
    {
        ActivateGwEndpointProbe(simHelper, cfg.gwId, false, stopSec);
        ActivateSatEndpointNotApplicable("satellite_orbiter_only");
        return;
    }

    if (cfg.pathType == "sat2gw")
    {
        ActivateGwEndpointProbe(simHelper, cfg.gwId, true, stopSec);
        return;
    }

    if (cfg.pathType == "sat2ut" || cfg.pathType == "gw2ut_e2e")
    {
        ActivateUtEndpointProbe(simHelper, cfg.trafficUtUserId, stopSec);
        return;
    }

    if (cfg.pathType == "gw2gw_e2e")
    {
        ActivateGwEndpointProbe(simHelper, cfg.gwDst, true, stopSec);
        return;
    }

    if (cfg.pathType == "sat2sat")
    {
        ActivateSatEndpointNotApplicable("no_ground_endpoint");
        return;
    }

    ActivateSatEndpointNotApplicable("unsupported_path_type");
}

// === Segment Traffic Installers ============================================

static void
SendPeriodicUdpPacket(Ptr<Socket> socket,
                      Ipv4Address dstAddr,
                      uint16_t    port,
                      uint32_t    packetSize,
                      Time        interval,
                      Time        stopTime)
{
    if (!socket || Simulator::Now() > stopTime)
    {
        return;
    }

    socket->SendTo(Create<Packet>(packetSize), 0, InetSocketAddress(dstAddr, port));

    Time nextTx = Simulator::Now() + interval;
    if (nextTx <= stopTime)
    {
        Simulator::Schedule(interval,
                            &SendPeriodicUdpPacket,
                            socket,
                            dstAddr,
                            port,
                            packetSize,
                            interval,
                            stopTime);
    }
}

// gwNodes / utUsers: caller provides the exact containers to install traffic between.
// This E2E harness now uses physical GW nodes only; there is no separate GW-user path.
// For service-link-focused tests, pair one physical GW with GetSelectedUtUser(utId)
// to avoid background load from all scenario UTs inflating the observed delay.
static void
InstallGwUtSegmentTrafficBase(Ptr<SimulationHelper> simHelper,
                              const TrafficConfig&  cfg,
                              double                simTimeSec,
                              NodeContainer         gwNodes,
                              NodeContainer         utUsers,
                              const std::string&    segmentLabel)
{
    if (!cfg.enableFwd && !cfg.enableRtn)
    {
        std::cout << "[TRAFFIC][" << segmentLabel
                  << "] Both FWD and RTN disabled, skipping traffic installation.\n";
        return;
    }

    if (gwNodes.GetN() == 0 || utUsers.GetN() == 0)
    {
        std::cout << "[TRAFFIC][" << segmentLabel << "] WARNING: gwNodes=" << gwNodes.GetN()
                  << " utUsers=" << utUsers.GetN()
                  << ", skipping traffic installation.\n";
        return;
    }

    double stopSec = ResolveTrafficStopSec(cfg, simTimeSec);

    std::cout << "[TRAFFIC][" << segmentLabel << "] gwNodes=" << gwNodes.GetN()
              << " utUsers=" << utUsers.GetN()
              << " start=" << cfg.startSec << "s"
              << " stop=" << stopSec << "s\n";

    if (cfg.enableFwd)
    {
        const uint16_t port = 9201;
        std::set<uint32_t> sinkNodeIds;

        for (uint32_t u = 0; u < utUsers.GetN(); ++u)
        {
            Ptr<Node> utUser = utUsers.Get(u);
            if (!utUser || !sinkNodeIds.insert(utUser->GetId()).second)
            {
                continue;
            }

            Ipv4Address utAddr = simHelper->GetSatelliteHelper()->GetUserAddress(utUser);
            PacketSinkHelper sink("ns3::UdpSocketFactory",
                                  InetSocketAddress(utAddr, port));
            ApplicationContainer sinkApps = sink.Install(utUser);
            sinkApps.Start(Seconds(0.0));
            sinkApps.Stop(Seconds(stopSec + 2.0));
        }

        for (uint32_t g = 0; g < gwNodes.GetN(); ++g)
        {
            Ptr<Node> gwNode = gwNodes.Get(g);
            Ipv4Address gwAddr;
            if (!gwNode || !TryGetNodeRoutableIp(gwNode, "TRAFFIC/FWD_GW", gwAddr))
            {
                continue;
            }

            for (uint32_t u = 0; u < utUsers.GetN(); ++u)
            {
                Ptr<Node> utUser = utUsers.Get(u);
                if (!utUser)
                {
                    continue;
                }

                Ipv4Address utAddr = simHelper->GetSatelliteHelper()->GetUserAddress(utUser);
                Ptr<Socket> socket = Socket::CreateSocket(gwNode, UdpSocketFactory::GetTypeId());
                socket->Bind(InetSocketAddress(gwAddr, 0));
                socket->SetAllowBroadcast(false);
                Simulator::Schedule(Seconds(cfg.startSec),
                                    &SendPeriodicUdpPacket,
                                    socket,
                                    utAddr,
                                    port,
                                    cfg.fwdPktBytes,
                                    MilliSeconds(cfg.fwdIntervalMs),
                                    Seconds(stopSec));
            }
        }

        std::cout << "[TRAFFIC][" << segmentLabel << "] FWD installed:"
                  << " interval=" << cfg.fwdIntervalMs << "ms"
                  << " pktSize=" << cfg.fwdPktBytes << "B"
                  << " rate~"
                  << (cfg.fwdPktBytes * 8.0 * 1000.0 / cfg.fwdIntervalMs / 1000.0)
                  << " kbps/flow\n";
    }

    if (cfg.enableRtn)
    {
        const uint16_t port = 9202;
        std::vector<std::pair<Ptr<Node>, Ipv4Address>> gwEndpoints;

        for (uint32_t g = 0; g < gwNodes.GetN(); ++g)
        {
            Ptr<Node> gwNode = gwNodes.Get(g);
            Ipv4Address gwAddr;
            if (!gwNode || !TryGetNodeRoutableIp(gwNode, "TRAFFIC/RTN_GW", gwAddr))
            {
                continue;
            }

            PacketSinkHelper sink("ns3::UdpSocketFactory",
                                  InetSocketAddress(gwAddr, port));
            ApplicationContainer sinkApps = sink.Install(gwNode);
            sinkApps.Start(Seconds(0.0));
            sinkApps.Stop(Seconds(stopSec + 2.0));
            gwEndpoints.push_back({gwNode, gwAddr});
        }

        for (uint32_t u = 0; u < utUsers.GetN(); ++u)
        {
            Ptr<Node> utUser = utUsers.Get(u);
            if (!utUser)
            {
                continue;
            }

            Ipv4Address utAddr = simHelper->GetSatelliteHelper()->GetUserAddress(utUser);
            for (const auto& gwEndpoint : gwEndpoints)
            {
                Ptr<Socket> socket =
                    Socket::CreateSocket(utUser, UdpSocketFactory::GetTypeId());
                socket->Bind(InetSocketAddress(utAddr, 0));
                socket->SetAllowBroadcast(false);
                Simulator::Schedule(Seconds(cfg.startSec),
                                    &SendPeriodicUdpPacket,
                                    socket,
                                    gwEndpoint.second,
                                    port,
                                    cfg.rtnPktBytes,
                                    MilliSeconds(cfg.rtnIntervalMs),
                                    Seconds(stopSec));
            }
        }

        std::cout << "[TRAFFIC][" << segmentLabel << "] RTN installed:"
                  << " interval=" << cfg.rtnIntervalMs << "ms"
                  << " pktSize=" << cfg.rtnPktBytes << "B"
                  << " rate~"
                  << (cfg.rtnPktBytes * 8.0 * 1000.0 / cfg.rtnIntervalMs / 1000.0)
                  << " kbps/flow\n";
    }
}

static void
InstallSat2SatBackgroundLoad(Ptr<SimulationHelper> simHelper,
                             double                simTimeSec,
                             uint32_t              gwAnchorId)
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
              << gwAnchorId << " <-> all UTs (physical GW node)\n";

    InstallGwUtSegmentTrafficBase(simHelper, bg, simTimeSec,
                                  GetPhysicalGwNodes(gwAnchorId), GetUtUsers(), "isl");
}

// SendGw2GwTaggedPacket: 每隔 interval 發送一個附有 Gw2GwTxTimeTag 的 UDP 封包。
// 設計原因：OnOffHelper 無法在封包上附加自訂 PacketTag，導致 Gw2GwAppRxCb 的
//   delaySamples 永遠為 0。改用與 SendSat2UtTaggedPacket 相同的遞迴排程模式，
//   在每次送出前設定 TxTime，Rx side 的 Gw2GwAppRxCb 即可計算單向延遲。
// 呼叫方式：在 startSec 時 Simulator::Schedule 第一次呼叫。
static void
SendGw2GwTaggedPacket(Ptr<Socket>  socket,
                      std::shared_ptr<Ipv4Address> currentDstAddr,
                      uint16_t     port,
                      uint32_t     packetSize,
                      Time         interval,
                      Time         stopTime)
{
    if (!socket || !currentDstAddr || Simulator::Now() > stopTime)
        return;

    Ptr<Packet> pkt = Create<Packet>(packetSize);
    Gw2GwTxTimeTag txTag;
    txTag.SetTxTime(Simulator::Now());
    pkt->AddPacketTag(txTag);
    uint64_t hopTraceId = AttachPacketHopTraceTag(pkt, "gw2gw");
    if (hopTraceId != 0)
    {
        std::ostringstream detail;
        detail << "dst=" << *currentDstAddr << ":" << port;
        LogPacketHopTrace(hopTraceId, "app_tx", "gw_app_src", detail.str(), pkt);
    }
    g_gw2gwDelivery.dstAddr = *currentDstAddr;
    socket->SendTo(pkt, 0, InetSocketAddress(*currentDstAddr, port));

    Simulator::Schedule(interval,
                        &SendGw2GwTaggedPacket,
                        socket,
                        currentDstAddr,
                        port,
                        packetSize,
                        interval,
                        stopTime);
}

// Given gwNode and entry satellite ID (0-indexed), returns the GW feeder interface IP
// and satellite-side IP for that satellite's feeder subnet.
// Scans all interfaces for IP pattern 40.{entrySatId+1}.x.x (SNS3 feeder subnet encoding).
// This is robust to selective beam activation: does NOT assume ifIndex = satId+1.
static bool
FindSatelliteGatewayIp(Ptr<Node>    gwNode,
                       uint32_t     entrySatId,
                       Ipv4Address& outGwIp,
                       Ipv4Address& outSatGwIp)
{
    Ptr<Ipv4> ipv4 = gwNode->GetObject<Ipv4>();
    if (!ipv4)
        return false;

    // SNS3 feeder subnet: 40.{satId+1}.0.0/16; GW side = .0.1, sat side = .0.2
    const uint8_t targetOct2 = static_cast<uint8_t>((entrySatId + 1) & 0xFF);

    for (uint32_t i = 1; i < ipv4->GetNInterfaces(); ++i)
    {
        if (ipv4->GetNAddresses(i) == 0)
            continue;
        Ipv4Address addr = ipv4->GetAddress(i, 0).GetLocal();
        uint32_t    raw  = addr.Get();
        if (((raw >> 24) & 0xFF) == 40 && ((raw >> 16) & 0xFF) == targetOct2)
        {
            outGwIp    = addr;
            outSatGwIp = Ipv4Address((raw & 0xFFFF0000u) | 0x00000002u);
            return true;
        }
    }
    return false;
}

// Returns the MAC address of the SatNetDevice on dstNode (destination GW).
// In SNS3 REGENERATION_NETWORK mode, feeder uplink packets must carry L2 dst = destination
// GW's SatNetDevice MAC. SatGwLlc reads it via SatGroundStationAddressTag and passes it to
// SatIslArbiterUnicast::GetSatIdWithMacIsl(), which maps GW MACs to exit satellite IDs.
// Passing a satellite OrbiterNetDevice MAC instead triggers NS_FATAL at arbiter.cc:67
// because the arbiter's MAC table contains only GW MACs, not satellite feeder MACs.
static bool
GetDstGwMac(Ptr<Node> dstNode, Mac48Address& outMac)
{
    for (uint32_t d = 0; d < dstNode->GetNDevices(); ++d)
    {
        Ptr<SatNetDevice> satDev = DynamicCast<SatNetDevice>(dstNode->GetDevice(d));
        if (!satDev)
            continue;
        Address addr = satDev->GetAddress();
        if (Mac48Address::IsMatchingType(addr))
        {
            outMac = Mac48Address::ConvertFrom(addr);
            return true;
        }
    }
    return false;
}

// Removes the /32 host route for dstAddr from gwNode's static routing table and installs
// a new one via the entry satellite for the given slot. Also refreshes the ARP entry so
// the L2 dst stays correct after the entry satellite changes. Scheduled each slot boundary.
static void
UpdateGw2GwHostRoute(Ptr<Node>              gwNode,
                     Ptr<IslRoutingManager> routingMgr,
                     uint32_t               gwSrc,
                     uint32_t               gwDst,
                     uint32_t               slotIndex,
                     std::shared_ptr<Ipv4Address> currentDstAddr,
                     Mac48Address           dstGwMac)
{
    GwToGwRoute route = routingMgr->GetGwRoute(gwSrc, gwDst, slotIndex);
    if (!route.valid)
    {
        std::cout << "[GW2GW_ROUTE] slot=" << slotIndex << " no valid route\n";
        return;
    }

    Ipv4Address newGwIp, newSatGwIp;
    if (!FindSatelliteGatewayIp(gwNode, route.entrySatId, newGwIp, newSatGwIp))
    {
        std::cout << "[GW2GW_ROUTE] slot=" << slotIndex
                  << " FindSatelliteGatewayIp failed for entrySatId=" << route.entrySatId << "\n";
        return;
    }

    Ipv4StaticRoutingHelper staticHelper;
    Ptr<Ipv4StaticRouting> staticRouting =
        staticHelper.GetStaticRouting(gwNode->GetObject<Ipv4>());
    if (!staticRouting)
        return;

    // Remove old /32 host route for dstAddr (scan from end for efficiency)
    uint32_t nRoutes = staticRouting->GetNRoutes();
    for (int32_t i = static_cast<int32_t>(nRoutes) - 1; i >= 0; --i)
    {
        Ipv4RoutingTableEntry e = staticRouting->GetRoute(static_cast<uint32_t>(i));
        if (currentDstAddr &&
            e.GetDest() == *currentDstAddr &&
            e.GetDestNetworkMask() == Ipv4Mask::GetOnes())
        {
            staticRouting->RemoveRoute(static_cast<uint32_t>(i));
            break;
        }
    }

    // Resolve the actual feeder interface by IP scan (not satId+1 assumption).
    // ResolveGwFeederInterface scans all interfaces for the one with IP = newGwIp.
    GwFeederInterfaceInfo newFeeder = ResolveGwFeederInterface(gwNode, gwSrc, newGwIp);

    // Compute new dstAddr from this slot's exit satellite feeder IP.
    // The exit satellite may change between slots (e.g., slot0=sat5 → slot1=sat4),
    // so dstAddr must also change to match the exit sat's GW feeder subnet.
    Ipv4Address newDstAddr = ExitSatFeederIp(route.exitSatId);

    // No explicit gateway — ARP resolves newDstAddr directly so L2 dst = dstGwMac.
    staticRouting->AddHostRouteTo(newDstAddr, newFeeder.ifIndex);

    // Refresh ARP: newDstAddr → dstGwMac on the correct feeder interface
    PrefillArpEntry(gwNode, gwSrc, newFeeder, newDstAddr, dstGwMac);

    std::cout << "[GW2GW_ROUTE] slot=" << slotIndex
              << " entrySat=" << route.entrySatId
              << " exitSat=" << route.exitSatId
              << " if=" << newFeeder.ifIndex
              << " dstAddr=" << newDstAddr
              << " (prev=" << (currentDstAddr ? Ipv4ToString(*currentDstAddr) : std::string("")) << ")"
              << " dstGwMac=" << dstGwMac << "\n";

    if (currentDstAddr)
    {
        *currentDstAddr = newDstAddr;
    }
}

// ExitSatFeederIp: returns 40.(exitSatId+1).0.1 as an Ipv4Address.
static Ipv4Address
ExitSatFeederIp(uint32_t exitSatId)
{
    std::ostringstream oss;
    oss << "40." << (exitSatId + 1) << ".0.1";
    return Ipv4Address(oss.str().c_str());
}

static void
InstallGw2GwApplicationTraffic(Ptr<SimulationHelper>   simHelper,
                               Ptr<IslRoutingManager>  routingMgr,
                               uint32_t                gwSrc,
                               uint32_t                gwDst,
                               double                  startSec,
                               double                  stopSec)
{
    Ptr<Node> srcNode = GetPhysicalGwNodeOrNull(gwSrc);
    // dstNode resolved below after route0 is known (requires exit satellite).
    Ptr<Node> dstNode = GetPhysicalGwNodeOrNull(gwDst);

    NS_ABORT_MSG_IF(!srcNode,
                    "[GW2GW_APP] physical GW node for gwSrc=" << gwSrc << " not found");
    NS_ABORT_MSG_IF(!dstNode,
                    "[GW2GW_APP] physical GW node for gwDst=" << gwDst << " not found");

    // Get route for slot 0 to find the correct entry satellite.
    // entrySatId → feeder ifIndex = entrySatId+1 → GW IP (40.N.0.1) and sat IP (40.N.0.2).
    GwToGwRoute route0 = routingMgr->GetGwRoute(gwSrc, gwDst, 0);
    NS_ABORT_MSG_IF(!route0.valid,
                    "[GW2GW_APP] no valid GwToGwRoute for gwSrc=" << gwSrc
                    << " gwDst=" << gwDst << " slot=0");

    Ipv4Address srcAddr, satGwIp;
    NS_ABORT_MSG_IF(!FindSatelliteGatewayIp(srcNode, route0.entrySatId, srcAddr, satGwIp),
                    "[GW2GW_APP] FindSatelliteGatewayIp failed for entrySatId="
                    << route0.entrySatId);

    // dstAddr: use exit satellite's GW feeder IP (40.(exitSat+1).0.1).
    // Rationale: the ISL arbiter routes by L2 dstGwMac, but the exit satellite
    // (e.g. sat5, beam6) has a directly-connected subnet 40.6.0.0/24 on its feeder
    // downlink.  Using 40.6.0.1 guarantees the exit satellite can deliver the packet
    // to GW1 via its own routing table, regardless of physicalGwNodes ordering.
    // Using TryGetPhysicalGwRoutableIp() (first routable IP) failed because
    // gws.Get(gwDst) may not be Delhi when physicalGwNodes=3.
    Ipv4Address dstAddr = ExitSatFeederIp(route0.exitSatId);

    GwFeederInterfaceInfo srcFeeder = ResolveGwFeederInterface(srcNode, gwSrc, srcAddr);

    std::cout << "[GW2GW_APP] slot0 entrySat=" << route0.entrySatId
              << " GW" << gwSrc
              << " feeder ifIndex=" << srcFeeder.ifIndex
              << " gwIp=" << srcAddr
              << " satGwIp=" << satGwIp << "\n";
    std::cout << "[GW2GW_APP] dstAddr (GW" << gwDst << ")=" << dstAddr << "\n";

    g_gw2gwDelivery = {};
    g_gw2gwDelivery.installed = true;
    g_gw2gwDelivery.srcGwId = gwSrc;
    g_gw2gwDelivery.dstGwId = gwDst;
    g_gw2gwDelivery.srcAddr = srcAddr;
    g_gw2gwDelivery.dstAddr = dstAddr;

    std::cout << "[GW2GW_APP] GW" << gwSrc << "=" << srcAddr
              << " -> GW" << gwDst << "=" << dstAddr
              << " start=" << startSec << "s"
              << " stop=" << stopSec << "s\n";

    // Print GW_src routing table for diagnostic: helps identify backbone vs feeder routes.
    std::cout << "[GW2GW_APP] === GW" << gwSrc << " IPv4 routing table ===\n";
    {
        Ptr<Ipv4> ipv4d = srcNode->GetObject<Ipv4>();
        Ipv4StaticRoutingHelper diagHelper;
        Ptr<Ipv4StaticRouting> diagRouting = diagHelper.GetStaticRouting(ipv4d);
        if (diagRouting)
        {
            for (uint32_t i = 0; i < diagRouting->GetNRoutes(); ++i)
            {
                Ipv4RoutingTableEntry r = diagRouting->GetRoute(i);
                std::cout << "  [" << i << "] dst=" << r.GetDest()
                          << "/" << r.GetDestNetworkMask()
                          << " gw=" << r.GetGateway()
                          << " if=" << r.GetInterface()
                          << " metric=" << diagRouting->GetMetric(i) << "\n";
            }
        }
        else
        {
            std::cout << "  (no Ipv4StaticRouting on GW" << gwSrc << ")\n";
        }
    }
    std::cout << "[GW2GW_APP] ===================================\n";

    // Install /32 host route on GW_src: dstAddr → feeder interface (entrySatId+1), no gateway.
    // ARP resolves dstAddr directly, so L2 dst = dstGwMac (destination GW's SatNetDevice MAC).
    // SatGwLlc reads L2 dst via SatGroundStationAddressTag; the ISL arbiter calls
    // GetSatIdWithMacIsl(dstGwMac) to map the destination GW MAC to the exit satellite ID.
    // Passing a satellite OrbiterNetDevice MAC instead causes NS_FATAL at satellite-isl-arbiter.cc:67
    // because the arbiter's MAC table contains only GW MACs.
    Mac48Address dstGwMac; // declared here so slot schedule loop can capture it
    NS_ABORT_MSG_IF(!GetDstGwMac(dstNode, dstGwMac),
                    "[GW2GW_APP] GetDstGwMac failed for GW" << gwDst);
    {
        Ptr<Ipv4> ipv4 = srcNode->GetObject<Ipv4>();
        Ipv4StaticRoutingHelper staticHelper;
        Ptr<Ipv4StaticRouting> staticRouting = staticHelper.GetStaticRouting(ipv4);
        NS_ABORT_MSG_IF(!staticRouting,
                        "[GW2GW_APP] no Ipv4StaticRouting on GW" << gwSrc);

        // Prefill ARP: dstAddr → dstGwMac (destination GW's SatNetDevice MAC).
        PrefillArpEntry(srcNode, gwSrc, srcFeeder, dstAddr, dstGwMac);
        std::cout << "[GW2GW_APP] ARP prefill: GW" << gwSrc
                  << " if=" << srcFeeder.ifIndex
                  << " " << dstAddr << " -> " << dstGwMac << "\n";

        // /32 host route, no explicit gateway — ARP for dstAddr resolves to dstGwMac.
        staticRouting->AddHostRouteTo(dstAddr, srcFeeder.ifIndex);
        std::cout << "[GW2GW_APP] static host route: dst=" << dstAddr
                  << " if=" << srcFeeder.ifIndex << " (no gateway)\n";
    }

    const uint16_t port         = 9001;
    const uint32_t pktSizeBytes = 512;

    // --- Sink (destination GW) ---
    // PacketSinkHelper listens on 0.0.0.0:port (any interface) so it catches packets
    // regardless of which feeder IP the exit satellite uses.  dstAddr may change between
    // slots (different exit sat → different feeder IP), so a wildcard bind is required.
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps = sink.Install(dstNode);
    g_gw2gwDelivery.traceConnected =
        sinkApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&Gw2GwAppRxCb));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec + 2.0));

    std::cout << "[GW2GW_OBS][PACKET] PacketSink::Rx trace "
              << (g_gw2gwDelivery.traceConnected ? "connected" : "NOT_CONNECTED")
              << " on GW" << gwDst << " (physical)\n";

    std::shared_ptr<Ipv4Address> currentDstAddr =
        std::make_shared<Ipv4Address>(dstAddr);

    // Schedule per-slot route and ARP updates. At each slot boundary the entry satellite may
    // change (e.g., slot0=sat45 → slot1=sat15), so the /32 host route and ARP entry must be
    // refreshed. These are scheduled AFTER ScheduleRoutingUpdates to preserve FIFO order.
    {
        double slotInterval = routingMgr->GetTimeSlotInterval();
        for (uint32_t slot = 1; slot < routingMgr->GetNumTimeSlots(); ++slot)
        {
            double slotTime = slotInterval * slot;
            if (slotTime >= stopSec + 2.0)
                break;
            Simulator::Schedule(Seconds(slotTime),
                                &UpdateGw2GwHostRoute,
                                srcNode,
                                routingMgr,
                                gwSrc,
                                gwDst,
                                slot,
                                currentDstAddr,
                                dstGwMac);
        }
    }

    // --- Source: custom tagged UDP sender (replaces OnOffHelper) ---
    // OnOffHelper cannot attach Gw2GwTxTimeTag, so delaySamples would always be 0.
    // Instead, use SendGw2GwTaggedPacket (same pattern as SendSat2UtTaggedPacket):
    //   bind socket to srcAddr (feeder interface IP) so routing selects SatNetDevice,
    //   attach Gw2GwTxTimeTag before each Send so Gw2GwAppRxCb can compute one-way delay.
    const Time txInterval = MilliSeconds(100);
    Ptr<Socket> txSocket = Socket::CreateSocket(srcNode, UdpSocketFactory::GetTypeId());
    txSocket->Bind(InetSocketAddress(srcAddr, 0));
    txSocket->SetAllowBroadcast(false);

    std::cout << "[GW2GW_APP] tagged UDP source"
              << " local=" << srcAddr
              << " dst=" << dstAddr << ":" << port
              << " interval=100ms"
              << " pktSize=" << pktSizeBytes << "B"
              << " start=" << startSec << "s stop=" << stopSec << "s\n";

    Simulator::Schedule(Seconds(startSec),
                        &SendGw2GwTaggedPacket,
                        txSocket,
                        currentDstAddr,
                        port,
                        pktSizeBytes,
                        txInterval,
                        Seconds(stopSec));

    Simulator::Schedule(
        Seconds(stopSec + 1.0),
        [sinkApps, gwSrc, gwDst, srcAddr, currentDstAddr, pktSizeBytes]()
        {
            auto sinkApp = DynamicCast<PacketSink>(sinkApps.Get(0));
            uint64_t rxBytes = sinkApp ? sinkApp->GetTotalRx() : 0;
            uint64_t estPkts = (rxBytes > 0) ? (rxBytes / pktSizeBytes) : 0;
            double avgDelayMs = (g_gw2gwDelivery.delaySamples > 0)
                                    ? (g_gw2gwDelivery.sumDelayMs /
                                       static_cast<double>(g_gw2gwDelivery.delaySamples))
                                    : 0.0;

            g_gw2gwDelivery.reported = true;
            g_gw2gwDelivery.rxBytes = rxBytes;
            g_gw2gwDelivery.estPkts = estPkts;

            std::cout << "\n[GW2GW_APP] === Packet-layer delivery summary ===\n"
                      << "  src: GW" << gwSrc << " (" << srcAddr << ")\n"
                      << "  dst: GW" << gwDst << " ("
                      << (currentDstAddr ? Ipv4ToString(*currentDstAddr) : std::string(""))
                      << ")\n"
                      << "  received: " << rxBytes << " bytes (~" << estPkts << " pkts)\n";
            std::cout << "[GW2GW_OBS][PACKET] traceConnected="
                      << (g_gw2gwDelivery.traceConnected ? "yes" : "no")
                      << " traceRxPkts=" << g_gw2gwDelivery.traceRxPkts
                      << " traceRxBytes=" << g_gw2gwDelivery.traceRxBytes
                      << " delaySamples=" << g_gw2gwDelivery.delaySamples
                      << " avgOneWayDelayMs=" << std::fixed << std::setprecision(3)
                      << avgDelayMs;
            if (g_gw2gwDelivery.delaySamples > 0)
            {
                std::cout << " minOneWayDelayMs=" << g_gw2gwDelivery.minDelayMs
                          << " maxOneWayDelayMs=" << g_gw2gwDelivery.maxDelayMs;
            }
            std::cout << "\n";

            if (rxBytes == 0)
            {
                std::cout << "  [FAIL] received=0, check GW-side unicast forwarding rules\n";
            }
            else
            {
                std::cout << "  [PASS] received>0, gateway-to-gateway path is active\n";
            }
            std::cout << "======================================\n\n";
        });
}

static void
SendSat2SatTaggedPacket(Ptr<Socket> socket,
                        Ipv4Address dstAddr,
                        uint16_t    port,
                        uint32_t    packetSize,
                        Time        interval,
                        Time        stopTime)
{
    if (!socket || Simulator::Now() > stopTime)
    {
        return;
    }

    Ptr<Packet> pkt = Create<Packet>(packetSize);
    Gw2GwTxTimeTag txTag;
    txTag.SetTxTime(Simulator::Now());
    pkt->AddPacketTag(txTag);
    uint64_t hopTraceId = AttachPacketHopTraceTag(pkt, "sat2sat");
    if (hopTraceId != 0)
    {
        std::ostringstream detail;
        detail << "dst=" << dstAddr << ":" << port;
        LogPacketHopTrace(hopTraceId, "app_tx", "sat_app_src", detail.str(), pkt);
    }
    socket->SendTo(pkt, 0, InetSocketAddress(dstAddr, port));

    Simulator::Schedule(interval,
                        &SendSat2SatTaggedPacket,
                        socket,
                        dstAddr,
                        port,
                        packetSize,
                        interval,
                        stopTime);
}

static void
InstallSat2SatApplicationTraffic(Ptr<SimulationHelper> /*simHelper*/,
                                 uint32_t              satSrc,
                                 uint32_t              satDst,
                                 double                startSec,
                                 double                stopSec)
{
    NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();
    NS_ABORT_MSG_IF(satSrc >= sats.GetN(),
                    "[SAT2SAT_APP] satSrc=" << satSrc << " >= orbiter count=" << sats.GetN());
    NS_ABORT_MSG_IF(satDst >= sats.GetN(),
                    "[SAT2SAT_APP] satDst=" << satDst << " >= orbiter count=" << sats.GetN());

    Ptr<Node> srcNode = sats.Get(satSrc);
    Ptr<Node> dstNode = sats.Get(satDst);
    Ipv4Address srcAddr;
    Ipv4Address dstAddr;
    NS_ABORT_MSG_IF(!TryGetNodeRoutableIp(srcNode, "SAT2SAT_APP/src", srcAddr),
                    "[SAT2SAT_APP] no routable IPv4 address on satSrc=" << satSrc);
    NS_ABORT_MSG_IF(!TryGetNodeRoutableIp(dstNode, "SAT2SAT_APP/dst", dstAddr),
                    "[SAT2SAT_APP] no routable IPv4 address on satDst=" << satDst);

    const uint16_t port = 9003;
    const uint32_t pktSizeBytes = 512;
    const Time txInterval = MilliSeconds(100);

    g_sat2satDelivery = {};
    g_sat2satDelivery.installed = true;
    g_sat2satDelivery.srcSatId = satSrc;
    g_sat2satDelivery.dstSatId = satDst;
    g_sat2satDelivery.srcAddr = srcAddr;
    g_sat2satDelivery.dstAddr = dstAddr;

    std::cout << "[SAT2SAT_APP] SAT" << satSrc << "=" << srcAddr
              << " -> SAT" << satDst << "=" << dstAddr
              << " start=" << startSec << "s"
              << " stop=" << stopSec << "s\n";

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(dstAddr, port));
    ApplicationContainer sinkApps = sink.Install(dstNode);
    g_sat2satDelivery.traceConnected =
        sinkApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&Sat2SatAppRxCb));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec + 2.0));

    Ptr<Socket> txSocket = Socket::CreateSocket(srcNode, UdpSocketFactory::GetTypeId());
    txSocket->Bind(InetSocketAddress(srcAddr, 0));
    txSocket->SetAllowBroadcast(false);

    Simulator::Schedule(Seconds(startSec),
                        &SendSat2SatTaggedPacket,
                        txSocket,
                        dstAddr,
                        port,
                        pktSizeBytes,
                        txInterval,
                        Seconds(stopSec));

    Simulator::Schedule(
        Seconds(stopSec + 1.0),
        [sinkApps, satSrc, satDst, srcAddr, dstAddr, pktSizeBytes]()
        {
            auto sinkApp = DynamicCast<PacketSink>(sinkApps.Get(0));
            uint64_t rxBytes = sinkApp ? sinkApp->GetTotalRx() : 0;
            uint64_t estPkts = (rxBytes > 0) ? (rxBytes / pktSizeBytes) : 0;
            double avgDelayMs = (g_sat2satDelivery.delaySamples > 0)
                                    ? (g_sat2satDelivery.sumDelayMs /
                                       static_cast<double>(g_sat2satDelivery.delaySamples))
                                    : 0.0;

            g_sat2satDelivery.reported = true;
            g_sat2satDelivery.rxBytes = rxBytes;
            g_sat2satDelivery.estPkts = estPkts;

            std::cout << "\n[SAT2SAT_APP] === Packet-layer delivery summary ===\n"
                      << "  src: SAT" << satSrc << " (" << srcAddr << ")\n"
                      << "  dst: SAT" << satDst << " (" << dstAddr << ")\n"
                      << "  received: " << rxBytes << " bytes (~" << estPkts << " pkts)\n";
            std::cout << "[SAT2SAT_OBS][PACKET] traceConnected="
                      << (g_sat2satDelivery.traceConnected ? "yes" : "no")
                      << " traceRxPkts=" << g_sat2satDelivery.traceRxPkts
                      << " traceRxBytes=" << g_sat2satDelivery.traceRxBytes
                      << " delaySamples=" << g_sat2satDelivery.delaySamples
                      << " avgOneWayDelayMs=" << std::fixed << std::setprecision(3)
                      << avgDelayMs << "\n";
            std::cout << ((rxBytes == 0)
                              ? "  [FAIL] received=0, SAT-to-SAT packet delivery failed\n"
                              : "  [PASS] received>0, directed SAT-to-SAT path is active\n");
            std::cout << "======================================\n\n";
        });
}

static void
SendSat2UtTaggedPacket(std::shared_ptr<std::map<uint32_t, Ptr<Socket>>> sockets,
                       Ptr<IslRoutingManager>                             routingMgr,
                       E2EConfig                                          cfg,
                       Ipv4Address                                        dstAddr,
                       uint16_t                                           port,
                       uint32_t                                           packetSize,
                       Time                                               interval,
                       Time                                               stopTime)
{
    if (!sockets || !routingMgr || Simulator::Now() > stopTime)
    {
        return;
    }

    uint32_t slot = static_cast<uint32_t>(
        std::floor(Simulator::Now().GetSeconds() / routingMgr->GetTimeSlotInterval()));
    GwToUtRoute route = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);

    if (route.valid && route.servingSatId < cfg.numSats)
    {
        Ptr<Socket>& socket = (*sockets)[route.servingSatId];
        if (!socket)
        {
            NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();
            NS_ABORT_MSG_IF(route.servingSatId >= sats.GetN(),
                            "[SAT2UT_APP] servingSatId=" << route.servingSatId
                            << " >= orbiterNodes.GetN()=" << sats.GetN());
            socket = Socket::CreateSocket(sats.Get(route.servingSatId),
                                          UdpSocketFactory::GetTypeId());
            NS_ABORT_MSG_IF(!socket,
                            "[SAT2UT_APP] failed to create UDP socket on serving sat "
                            << route.servingSatId);
            socket->SetAllowBroadcast(false);
        }

        Ptr<Packet> pkt = Create<Packet>(packetSize);
        Gw2GwTxTimeTag txTag;
        txTag.SetTxTime(Simulator::Now());
        pkt->AddPacketTag(txTag);
        uint64_t hopTraceId = AttachPacketHopTraceTag(pkt, "sat2ut");
        if (hopTraceId != 0)
        {
            std::ostringstream detail;
            detail << "servingSat=sat" << route.servingSatId
                   << " dst=" << dstAddr << ":" << port;
            LogPacketHopTrace(hopTraceId, "app_tx", "sat_app_src", detail.str(), pkt);
        }
        socket->SendTo(pkt, 0, InetSocketAddress(dstAddr, port));
    }

    Time nextTx = Simulator::Now() + interval;
    if (nextTx <= stopTime)
    {
        Simulator::Schedule(interval,
                            &SendSat2UtTaggedPacket,
                            sockets,
                            routingMgr,
                            cfg,
                            dstAddr,
                            port,
                            packetSize,
                            interval,
                            stopTime);
    }
}

static void
InstallSat2UtApplicationTraffic(Ptr<SimulationHelper> simHelper,
                                Ptr<IslRoutingManager> routingMgr,
                                const E2EConfig&       cfg,
                                double                 startSec,
                                double                 stopSec)
{
    Ptr<Node> utUser = GetUtUserNodeOrNull(cfg.trafficUtUserId);
    NS_ABORT_MSG_IF(!utUser,
                    "[SAT2UT_APP] utUser node for trafficUtUserId="
                    << cfg.trafficUtUserId << " not found");

    Ipv4Address dstAddr = simHelper->GetSatelliteHelper()->GetUserAddress(utUser);
    const uint16_t port = 9002;
    const uint32_t pktSizeBytes = 512;
    const Time     txInterval = MilliSeconds(100);

    g_sat2utDelivery = {};
    g_sat2utDelivery.installed = true;
    g_sat2utDelivery.dstUtUserId = cfg.trafficUtUserId;
    g_sat2utDelivery.dstAddr = dstAddr;

    std::cout << "[SAT2UT_APP] SAT(serving route)"
              << " -> UT_user" << cfg.trafficUtUserId
              << "=" << dstAddr
              << " start=" << startSec << "s"
              << " stop=" << stopSec << "s"
              << " service-link-only=1\n";
    if (cfg.servicelink.traffic.enableRtn)
    {
        std::cout << "[SAT2UT_APP] NOTE: RTN traffic request is ignored in service-link-only "
                  << "sat2ut mode; only FWD SAT->UT packets are generated.\n";
    }

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(dstAddr, port));
    ApplicationContainer sinkApps = sink.Install(utUser);
    g_sat2utDelivery.traceConnected =
        sinkApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&Sat2UtAppRxCb));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec + 2.0));

    std::cout << "[SAT2UT_OBS][PACKET] PacketSink::Rx trace "
              << (g_sat2utDelivery.traceConnected ? "connected" : "NOT_CONNECTED")
              << " on UT_user" << cfg.trafficUtUserId << "\n";

    auto sockets = std::make_shared<std::map<uint32_t, Ptr<Socket>>>();
    Simulator::Schedule(Seconds(startSec),
                        &SendSat2UtTaggedPacket,
                        sockets,
                        routingMgr,
                        cfg,
                        dstAddr,
                        port,
                        pktSizeBytes,
                        txInterval,
                        Seconds(stopSec));

    Simulator::Schedule(
        Seconds(stopSec + 1.0),
        [sinkApps, cfg, dstAddr, pktSizeBytes]()
        {
            auto sinkApp = DynamicCast<PacketSink>(sinkApps.Get(0));
            uint64_t rxBytes = sinkApp ? sinkApp->GetTotalRx() : 0;
            uint64_t estPkts = (rxBytes > 0) ? (rxBytes / pktSizeBytes) : 0;
            double avgDelayMs = (g_sat2utDelivery.delaySamples > 0)
                                    ? (g_sat2utDelivery.sumDelayMs /
                                       static_cast<double>(g_sat2utDelivery.delaySamples))
                                    : 0.0;

            g_sat2utDelivery.reported = true;
            g_sat2utDelivery.rxBytes = rxBytes;
            g_sat2utDelivery.estPkts = estPkts;

            std::cout << "\n[SAT2UT_APP] === Packet-layer delivery summary ===\n"
                      << "  dst: UT_user" << cfg.trafficUtUserId
                      << " (" << dstAddr << ")\n"
                      << "  received: " << rxBytes << " bytes (~" << estPkts << " pkts)\n";
            std::cout << "[SAT2UT_OBS][PACKET] traceConnected="
                      << (g_sat2utDelivery.traceConnected ? "yes" : "no")
                      << " traceRxPkts=" << g_sat2utDelivery.traceRxPkts
                      << " traceRxBytes=" << g_sat2utDelivery.traceRxBytes
                      << " delaySamples=" << g_sat2utDelivery.delaySamples
                      << " avgOneWayDelayMs=" << std::fixed << std::setprecision(3)
                      << avgDelayMs;
            if (g_sat2utDelivery.delaySamples > 0)
            {
                std::cout << " minOneWayDelayMs=" << g_sat2utDelivery.minDelayMs
                          << " maxOneWayDelayMs=" << g_sat2utDelivery.maxDelayMs;
            }
            std::cout << "\n";

            if (rxBytes == 0)
            {
                std::cout << "  [FAIL] received=0, service-link-only packet delivery failed\n";
            }
            else
            {
                std::cout << "  [PASS] received>0, pure SAT->UT service link is active\n";
            }
            std::cout << "======================================\n\n";
        });
}

static bool
InstallFeederlinkTraffic(Ptr<SimulationHelper>   simHelper,
                         Ptr<IslRoutingManager>  routingMgr,
                         const E2EConfig&        cfg,
                         const PathTypePlan&     plan)
{
    PrintSegmentBanner(E2ESegment::FEEDERLINK, cfg.feederlink.enabled);
    if (!cfg.feederlink.enabled)
    {
        return false;
    }

    if (plan.trafficKind == TrafficKind::GW2GW_APPLICATION)
    {
        InstallGw2GwApplicationTraffic(simHelper,
                                       routingMgr,
                                       cfg.gwSrc,
                                       cfg.gwDst,
                                       cfg.feederlink.traffic.startSec,
                                       ResolveTrafficStopSec(cfg.feederlink.traffic, cfg.simTimeSec));
        return true;
    }

    if (plan.trafficKind == TrafficKind::GW_UT_ALL ||
        plan.trafficKind == TrafficKind::GW_UT_SELECTED)
    {
        NodeContainer utUsers = (plan.trafficKind == TrafficKind::GW_UT_SELECTED)
                                    ? GetSelectedUtUser(cfg.trafficUtUserId)
                                    : GetUtUsers();
        InstallGwUtSegmentTrafficBase(simHelper,
                                      cfg.feederlink.traffic,
                                      cfg.simTimeSec,
                                      GetPhysicalGwNodes(plan.edgeGatewayId),
                                      utUsers,
                                      "feederlink");
        return true;
    }

    std::cout << "[E2E][feederlink] no dedicated traffic generator selected\n";
    return false;
}

static void
InstallIslTraffic(Ptr<SimulationHelper>   simHelper,
                  const E2EConfig&        cfg,
                  const PathTypePlan&     plan)
{
    PrintSegmentBanner(E2ESegment::ISL, cfg.isl.enabled);
    if (!cfg.isl.enabled)
    {
        return;
    }

    if (plan.trafficKind == TrafficKind::ISL_BACKGROUND)
    {
        InstallSat2SatBackgroundLoad(simHelper, cfg.simTimeSec, cfg.gwId);
        return;
    }

    if (plan.trafficKind == TrafficKind::SAT_SAT_APPLICATION)
    {
        InstallSat2SatApplicationTraffic(simHelper,
                                         cfg.satSrc,
                                         cfg.satDst,
                                         cfg.isl.traffic.startSec,
                                         ResolveTrafficStopSec(cfg.isl.traffic, cfg.simTimeSec));
        return;
    }

    std::cout << "[E2E][isl] routing/transit enabled without extra ISL-only load generator\n";
}

static bool
InstallServicelinkTraffic(Ptr<SimulationHelper>   simHelper,
                          Ptr<IslRoutingManager>  routingMgr,
                          const E2EConfig&        cfg,
                          const PathTypePlan&     plan)
{
    PrintSegmentBanner(E2ESegment::SERVICELINK, cfg.servicelink.enabled);
    if (!cfg.servicelink.enabled)
    {
        return false;
    }

    if (plan.trafficKind == TrafficKind::GW_UT_SELECTED ||
        plan.trafficKind == TrafficKind::GW_UT_ALL)
    {
        NodeContainer utUsers = (plan.trafficKind == TrafficKind::GW_UT_SELECTED)
                                    ? GetSelectedUtUser(cfg.trafficUtUserId)
                                    : GetUtUsers();
        InstallGwUtSegmentTrafficBase(simHelper,
                                      cfg.servicelink.traffic,
                                      cfg.simTimeSec,
                                      GetPhysicalGwNodes(plan.edgeGatewayId),
                                      utUsers,
                                      "servicelink");
        return true;
    }

    if (plan.trafficKind == TrafficKind::SAT_UT_APPLICATION)
    {
        InstallSat2UtApplicationTraffic(simHelper,
                                        routingMgr,
                                        cfg,
                                        cfg.servicelink.traffic.startSec,
                                        ResolveTrafficStopSec(cfg.servicelink.traffic, cfg.simTimeSec));
        return true;
    }

    if (plan.trafficKind == TrafficKind::GW2GW_APPLICATION)
    {
        std::cout << "[E2E][servicelink] gateway-to-gateway traffic already installed upstream\n";
        return false;
    }

    std::cout << "[E2E][servicelink] no dedicated traffic generator selected\n";
    return false;
}

// === E2E Composition ========================================================

static void
InstallE2ETraffic(Ptr<SimulationHelper>   simHelper,
                  Ptr<IslRoutingManager>  routingMgr,
                  const E2EConfig&        cfg,
                  const PathTypePlan&     plan)
{
    bool edgeTrafficInstalled = InstallFeederlinkTraffic(simHelper, routingMgr, cfg, plan);
    InstallIslTraffic(simHelper, cfg, plan);

    if (!edgeTrafficInstalled)
    {
        InstallServicelinkTraffic(simHelper, routingMgr, cfg, plan);
    }
    else
    {
        PrintSegmentBanner(E2ESegment::SERVICELINK, cfg.servicelink.enabled);
        if (cfg.servicelink.enabled)
        {
            std::cout << "[E2E][servicelink] reusing upstream edge traffic installation\n";
        }
    }
}

static void
ConfigureObsScope(Ptr<IslRoutingManager> routingMgr,
                  const E2EConfig&       cfg,
                  uint32_t               numSlots)
{
    g_obsScope = {};

    const PathTypeSpec spec = GetPathTypeSpec(cfg.pathType);
    g_obsScope.activeFeeder = (spec.obsFeederMode == ObsFeederMode::PHY);
    g_obsScope.activeService = spec.usesServicelink;
    g_obsScope.activeIsl = spec.usesIsl;
    bool hasValidIslRoute = false;
    bool hasIslHop = false;

    if (cfg.pathType == "gw2sat")
    {
        for (uint32_t k = 0; k < numSlots; ++k)
        {
            for (uint32_t satId : routingMgr->GetGwVisibleSats(cfg.gwId, k))
            {
                g_obsScope.feederKeys.insert(MakeSatKey(satId));
            }
        }
    }
    else if (cfg.pathType == "sat2gw")
    {
        // SAT→GW downlink: observe GW Rx side.
        g_obsScope.feederKeys.insert(MakeGwKey(cfg.gwId));
    }
    else if (cfg.pathType == "sat2ut")
    {
        // Service link is FWD (SAT→UT); observation point is UT Rx (UtRxServiceCb).
        // Key follows the scenario UT user chosen for traffic/probe, not the logical routing UT.
        g_obsScope.serviceKeys.insert(MakeUtKey(cfg.trafficUtUserId));
        for (uint32_t k = 0; k < numSlots; ++k)
        {
            GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, k);
            if (r.valid)
            {
                g_obsScope.serviceKeys.insert(MakeSatKey(r.servingSatId));
            }
        }
    }
    else if (cfg.pathType == "sat2sat")
    {
        // sat2sat activates only the ISL segment.
        // Call TracePath per slot to discover all ISL pairs on the route,
        // and add each directed pair to islKeys.
        // IsObsKeyInScope() matches exact keys; no wildcard is needed.
        for (uint32_t k = 0; k < numSlots; ++k)
        {
            std::vector<uint32_t> path =
                routingMgr->TracePath(cfg.satSrc, cfg.satDst, k);
            // TracePath returns a path ending with UINT32_MAX if no route exists.
            if (path.empty() || path.back() == UINT32_MAX)
            {
                continue;
            }
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                g_obsScope.islKeys.insert(MakeIslKey(path[i], path[i + 1]));
            }
        }
    }
    else if (cfg.pathType == "gw2ut_e2e")
    {
        for (uint32_t k = 0; k < numSlots; ++k)
        {
            GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, k);
            if (!r.valid)
            {
                continue;
            }
            hasValidIslRoute = true;
            hasIslHop = hasIslHop || (r.satPath.size() > 1);

            g_obsScope.feederKeys.insert(MakeSatKey(r.entrySatId));
            // Service link is FWD (SAT→UT); observe at UT Rx, not at satellite.
            g_obsScope.serviceKeys.insert(MakeSatKey(r.servingSatId));
            g_obsScope.serviceKeys.insert(MakeUtKey(cfg.trafficUtUserId));

            for (size_t i = 0; i + 1 < r.satPath.size(); ++i)
            {
                g_obsScope.islKeys.insert(MakeIslKey(r.satPath[i], r.satPath[i + 1]));
            }
        }
    }
    else if (cfg.pathType == "gw2gw_e2e")
    {
        for (uint32_t k = 0; k < numSlots; ++k)
        {
            GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, k);
            if (!r.valid)
            {
                continue;
            }
            hasValidIslRoute = true;
            hasIslHop = hasIslHop || (r.satPath.size() > 1);
            g_obsScope.feederKeys.insert(MakeSatKey(r.entrySatId));
            g_obsScope.feederKeys.insert(MakeGwKey(cfg.gwDst));
            for (size_t i = 0; i + 1 < r.satPath.size(); ++i)
            {
                g_obsScope.islKeys.insert(MakeIslKey(r.satPath[i], r.satPath[i + 1]));
            }
        }
    }

    if (cfg.pathType == "gw2gw_e2e")
    {
        std::cout << "[GW2GW_OBS][ROUTING_ISL] scopedIslLinks="
                  << g_obsScope.islKeys.size()
                  << " gwSrc=" << cfg.gwSrc
                  << " gwDst=" << cfg.gwDst
                  << "\n";
    }

    g_obsVerdictScope = g_obsScope;

    std::cout << "[OBS] scope:"
              << " feeder=" << (g_obsScope.activeFeeder ? g_obsScope.feederKeys.size() : 0)
              << " service=" << (g_obsScope.activeService ? g_obsScope.serviceKeys.size() : 0)
              << " isl=" << (g_obsScope.activeIsl ? g_obsScope.islKeys.size() : 0)
              << "\n";
    std::cout << "[OBS] verdict_scope:"
              << " feeder=" << (g_obsVerdictScope.activeFeeder ? g_obsVerdictScope.feederKeys.size() : 0)
              << " service=" << (g_obsVerdictScope.activeService ? g_obsVerdictScope.serviceKeys.size() : 0)
              << " isl=" << (g_obsVerdictScope.activeIsl ? g_obsVerdictScope.islKeys.size() : 0)
              << "\n";

    bool emptyIslScopeDueToNoHop =
        g_obsScope.activeIsl && g_obsScope.islKeys.empty() && hasValidIslRoute && !hasIslHop;
    if (emptyIslScopeDueToNoHop)
    {
        std::cout << "[OBS] ISL scope empty because valid route has no ISL hop\n";
    }
    if ((g_obsScope.activeFeeder && g_obsScope.feederKeys.empty()) ||
        (g_obsScope.activeService && g_obsScope.serviceKeys.empty()) ||
        (g_obsScope.activeIsl && g_obsScope.islKeys.empty() && !emptyIslScopeDueToNoHop))
    {
        std::cout << "[OBS] WARNING: active segment has empty scope; related OBS output may be suppressed\n";
    }
}

// Update g_obsScope for a specific slot so that feeder/service/ISL keys
// reflect only the satellites active in that slot.
// Called at each slot boundary (after ApplyRoutingTable) to prevent stale
// keys from triggering false tput=0 alerts on satellites no longer in use.
static void
UpdateObsScopeForSlot(Ptr<IslRoutingManager> routingMgr,
                      const E2EConfig&       cfg,
                      uint32_t               slot)
{
    if (cfg.pathType == "gw2sat")
    {
        g_obsScope.feederKeys.clear();
        for (uint32_t satId : routingMgr->GetGwVisibleSats(cfg.gwId, slot))
        {
            g_obsScope.feederKeys.insert(MakeSatKey(satId));
        }
    }
    else if (cfg.pathType == "sat2ut")
    {
        // Ensure the key remains set in case scope was cleared elsewhere.
        g_obsScope.serviceKeys.clear();
        g_obsScope.serviceKeys.insert(MakeUtKey(cfg.trafficUtUserId));
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        if (r.valid)
        {
            g_obsScope.serviceKeys.insert(MakeSatKey(r.servingSatId));
        }
    }
    else if (cfg.pathType == "sat2sat")
    {
        g_obsScope.islKeys.clear();
        std::vector<uint32_t> path = routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot);
        if (!path.empty() && path.back() != UINT32_MAX)
        {
            for (size_t i = 0; i + 1 < path.size(); ++i)
            {
                g_obsScope.islKeys.insert(MakeIslKey(path[i], path[i + 1]));
            }
        }
    }
    else if (cfg.pathType == "gw2ut_e2e")
    {
        g_obsScope.feederKeys.clear();
        g_obsScope.serviceKeys.clear();
        g_obsScope.islKeys.clear();

        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        if (!r.valid)
        {
            std::cout << "[OBS][SCOPE_UPDATE] slot=" << slot
                      << " gw2ut_e2e route invalid; scope cleared\n";
            return;
        }
        g_obsScope.feederKeys.insert(MakeSatKey(r.entrySatId));
        g_obsScope.serviceKeys.insert(MakeSatKey(r.servingSatId));
        g_obsScope.serviceKeys.insert(MakeUtKey(cfg.trafficUtUserId));
        for (size_t i = 0; i + 1 < r.satPath.size(); ++i)
        {
            g_obsScope.islKeys.insert(MakeIslKey(r.satPath[i], r.satPath[i + 1]));
        }
    }
    else if (cfg.pathType == "gw2gw_e2e")
    {
        g_obsScope.feederKeys.clear();
        g_obsScope.serviceKeys.clear();
        g_obsScope.islKeys.clear();

        GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
        if (r.valid)
        {
            g_obsScope.feederKeys.insert(MakeSatKey(r.entrySatId));
            g_obsScope.feederKeys.insert(MakeGwKey(cfg.gwDst));
            for (size_t i = 0; i + 1 < r.satPath.size(); ++i)
            {
                g_obsScope.islKeys.insert(MakeIslKey(r.satPath[i], r.satPath[i + 1]));
            }
        }
    }
    // sat2gw: feederKey is always MakeGwKey(gwId); no per-slot change needed.

    MergeObsScope(g_obsVerdictScope, g_obsScope);

    std::cout << "[OBS][SCOPE_UPDATE] slot=" << slot
              << " feeder=" << (g_obsScope.activeFeeder ? g_obsScope.feederKeys.size() : 0)
              << " service=" << (g_obsScope.activeService ? g_obsScope.serviceKeys.size() : 0)
              << " isl=" << (g_obsScope.activeIsl ? g_obsScope.islKeys.size() : 0)
              << "\n";
}

static uint64_t
SumScopedRxPktsInScope(const std::string& linkType, const ObsScope& scope)
{
    const std::map<std::string, SegLinkStats>* statsMap = nullptr;
    if (linkType == "feeder")
    {
        statsMap = &g_feederObsStats;
    }
    else if (linkType == "service")
    {
        statsMap = &g_serviceObsStats;
    }
    else if (linkType == "isl")
    {
        statsMap = &g_islObsStats2;
    }

    uint64_t total{0};
    if (!statsMap)
    {
        return total;
    }

    for (const auto& kv : *statsMap)
    {
        if (IsObsKeyInScope(scope, linkType, kv.first))
        {
            total += kv.second.rxPkts;
        }
    }
    return total;
}

static const std::map<std::string, SegLinkStats>*
GetObsStatsMap(const std::string& linkType)
{
    if (linkType == "feeder")
    {
        return &g_feederObsStats;
    }
    if (linkType == "service")
    {
        return &g_serviceObsStats;
    }
    if (linkType == "isl")
    {
        return &g_islObsStats2;
    }
    return nullptr;
}

static uint64_t
CountScopedDelaySamplesInScope(const std::string& linkType, const ObsScope& scope)
{
    const std::map<std::string, SegLinkStats>* statsMap = GetObsStatsMap(linkType);
    uint64_t total{0};
    if (!statsMap)
    {
        return total;
    }

    for (const auto& kv : *statsMap)
    {
        if (IsObsKeyInScope(scope, linkType, kv.first))
        {
            total += kv.second.delaySamples;
        }
    }
    return total;
}

static uint64_t
CountIslPropagationSamplesInScope(const ObsScope& scope)
{
    uint64_t total{0};
    for (const auto& kv : g_islPropagationStats)
    {
        if (IsObsKeyInScope(scope, "isl", kv.first))
        {
            total += kv.second.samples;
        }
    }
    return total;
}

static double
AvgIslPropagationDelayMsInScope(const ObsScope& scope)
{
    double totalDelayMs{0.0};
    uint64_t totalSamples{0};
    for (const auto& kv : g_islPropagationStats)
    {
        if (IsObsKeyInScope(scope, "isl", kv.first))
        {
            totalDelayMs += kv.second.sumDelayMs;
            totalSamples += kv.second.samples;
        }
    }
    return (totalSamples > 0) ? totalDelayMs / static_cast<double>(totalSamples) : 0.0;
}

static uint64_t
CountIslPropagationSamplesInScopeForSlot(const ObsScope& scope, uint32_t slot)
{
    auto slotIt = g_islPropagationStatsBySlot.find(slot);
    if (slotIt == g_islPropagationStatsBySlot.end())
    {
        return 0;
    }

    uint64_t total{0};
    for (const auto& kv : slotIt->second)
    {
        if (IsObsKeyInScope(scope, "isl", kv.first))
        {
            total += kv.second.samples;
        }
    }
    return total;
}

static double
AvgIslPropagationDelayMsInScopeForSlot(const ObsScope& scope, uint32_t slot)
{
    auto slotIt = g_islPropagationStatsBySlot.find(slot);
    if (slotIt == g_islPropagationStatsBySlot.end())
    {
        return 0.0;
    }

    double totalDelayMs{0.0};
    uint64_t totalSamples{0};
    for (const auto& kv : slotIt->second)
    {
        if (IsObsKeyInScope(scope, "isl", kv.first))
        {
            totalDelayMs += kv.second.sumDelayMs;
            totalSamples += kv.second.samples;
        }
    }
    return (totalSamples > 0) ? totalDelayMs / static_cast<double>(totalSamples) : 0.0;
}

struct ScopedIslPhyCheck
{
    bool applicable{false};
    bool pass{false};
    uint64_t totalSamples{0};
    uint32_t slotsWithSamples{0};
    uint32_t slotsWithTheory{0};
    double observedAvgMs{0.0};
    double theoreticalAvgMs{0.0};
    double toleranceMs{0.0};
    double errorMs{0.0};
    std::string detail;
};

static ScopedIslPhyCheck
EvaluateScopedIslPhyDelay(Ptr<IslRoutingManager> routingMgr,
                          const E2EConfig&       cfg,
                          const ObsScope&        scope,
                          uint32_t               numSlots)
{
    ScopedIslPhyCheck check;
    std::ostringstream oss;
    double observedWeightedSumMs{0.0};
    double theoryWeightedSumMs{0.0};
    uint64_t matchedSamples{0};

    for (uint32_t slot = 0; slot < numSlots; ++slot)
    {
        const uint64_t slotSamples = CountIslPropagationSamplesInScopeForSlot(scope, slot);
        if (slotSamples == 0)
        {
            continue;
        }

        check.slotsWithSamples++;
        const double observedSlotAvgMs = AvgIslPropagationDelayMsInScopeForSlot(scope, slot);
        const double theoryTotalMs =
            EstimateTheoreticalIslPropagationDelayMs(routingMgr, cfg, slot);
        const uint32_t hopCount = EstimateTheoreticalIslHopCount(routingMgr, cfg, slot);

        check.totalSamples += slotSamples;

        if (theoryTotalMs <= 0.0 || hopCount == 0)
        {
            continue;
        }

        check.slotsWithTheory++;
        matchedSamples += slotSamples;
        observedWeightedSumMs += observedSlotAvgMs * static_cast<double>(slotSamples);
        theoryWeightedSumMs +=
            (theoryTotalMs / static_cast<double>(hopCount)) * static_cast<double>(slotSamples);
    }

    if (matchedSamples == 0 || check.slotsWithTheory == 0)
    {
        oss << "phyDelayCheck=not_checked slotsWithSamples=" << check.slotsWithSamples
            << " slotsWithTheory=" << check.slotsWithTheory;
        check.detail = oss.str();
        return check;
    }

    check.applicable = true;
    check.observedAvgMs = observedWeightedSumMs / static_cast<double>(matchedSamples);
    check.theoreticalAvgMs = theoryWeightedSumMs / static_cast<double>(matchedSamples);
    check.toleranceMs = std::max(0.25, check.theoreticalAvgMs * 0.40);
    check.errorMs = std::fabs(check.observedAvgMs - check.theoreticalAvgMs);
    check.pass = check.errorMs <= check.toleranceMs;

    oss << std::fixed << std::setprecision(3)
        << " observedHopPropDelay=" << check.observedAvgMs
        << "ms theoreticalHopPropDelay=" << check.theoreticalAvgMs
        << "ms errorMs=" << check.errorMs
        << " toleranceMs=" << check.toleranceMs
        << " slotsWithSamples=" << check.slotsWithSamples
        << " slotsWithTheory=" << check.slotsWithTheory;
    check.detail = oss.str();
    return check;
}

static uint64_t
CountScopedDelaySamplesWithPrefixInScope(const std::string& linkType,
                                         const std::string& prefix,
                                         const ObsScope&    scope)
{
    const std::map<std::string, SegLinkStats>* statsMap = GetObsStatsMap(linkType);
    uint64_t total{0};
    if (!statsMap)
    {
        return total;
    }

    for (const auto& kv : *statsMap)
    {
        if (HasPrefix(kv.first, prefix) && IsObsKeyInScope(scope, linkType, kv.first))
        {
            total += kv.second.delaySamples;
        }
    }
    return total;
}

static double
AvgScopedDelayMsInScope(const std::string& linkType, const ObsScope& scope)
{
    const std::map<std::string, SegLinkStats>* statsMap = GetObsStatsMap(linkType);
    double   totalDelayMs{0.0};
    uint64_t totalSamples{0};
    if (!statsMap)
    {
        return 0.0;
    }

    for (const auto& kv : *statsMap)
    {
        if (IsObsKeyInScope(scope, linkType, kv.first))
        {
            totalDelayMs += kv.second.sumDelayMs;
            totalSamples += kv.second.delaySamples;
        }
    }
    return (totalSamples > 0) ? totalDelayMs / static_cast<double>(totalSamples) : 0.0;
}

static double
AvgScopedDelayMsWithPrefixInScope(const std::string& linkType,
                                  const std::string& prefix,
                                  const ObsScope&    scope)
{
    const std::map<std::string, SegLinkStats>* statsMap = GetObsStatsMap(linkType);
    double   totalDelayMs{0.0};
    uint64_t totalSamples{0};
    if (!statsMap)
    {
        return 0.0;
    }

    for (const auto& kv : *statsMap)
    {
        if (HasPrefix(kv.first, prefix) && IsObsKeyInScope(scope, linkType, kv.first))
        {
            totalDelayMs += kv.second.sumDelayMs;
            totalSamples += kv.second.delaySamples;
        }
    }
    return (totalSamples > 0) ? totalDelayMs / static_cast<double>(totalSamples) : 0.0;
}

static std::string
FormatDelayMetric(const std::string& label, uint64_t delaySamples, double avgDelayMs)
{
    std::ostringstream oss;
    oss << label << "=";
    if (delaySamples > 0)
    {
        oss << std::fixed << std::setprecision(2) << avgDelayMs << "ms";
    }
    else
    {
        oss << "--";
    }
    return oss.str();
}

static uint64_t
SumScopedRxPktsWithPrefixInScope(const std::string& linkType,
                                 const std::string& prefix,
                                 const ObsScope&    scope)
{
    const std::map<std::string, SegLinkStats>* statsMap = nullptr;
    if (linkType == "feeder")
    {
        statsMap = &g_feederObsStats;
    }
    else if (linkType == "service")
    {
        statsMap = &g_serviceObsStats;
    }
    else if (linkType == "isl")
    {
        statsMap = &g_islObsStats2;
    }

    uint64_t total{0};
    if (!statsMap)
    {
        return total;
    }

    for (const auto& kv : *statsMap)
    {
        if (HasPrefix(kv.first, prefix) && IsObsKeyInScope(scope, linkType, kv.first))
        {
            total += kv.second.rxPkts;
        }
    }
    return total;
}

static uint32_t
CountScopedKeysWithPrefix(const std::set<std::string>& keys, const std::string& prefix)
{
    uint32_t count{0};
    for (const auto& key : keys)
    {
        if (HasPrefix(key, prefix))
        {
            ++count;
        }
    }
    return count;
}

static bool
GwUtRouteHasAnyIslHop(Ptr<IslRoutingManager> routingMgr,
                      const E2EConfig&       cfg,
                      uint32_t               numSlots,
                      uint32_t*              validSlotsOut = nullptr)
{
    bool hasHop{false};
    uint32_t validSlots{0};
    for (uint32_t k = 0; k < numSlots; ++k)
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, k);
        if (!r.valid)
        {
            continue;
        }
        ++validSlots;
        if (r.satPath.size() > 1)
        {
            hasHop = true;
        }
    }

    if (validSlotsOut)
    {
        *validSlotsOut = validSlots;
    }
    return hasHop;
}

static bool
GwGwRouteHasAnyIslHop(Ptr<IslRoutingManager> routingMgr,
                      const E2EConfig&       cfg,
                      uint32_t               numSlots,
                      uint32_t*              validSlotsOut = nullptr)
{
    bool hasHop{false};
    uint32_t validSlots{0};
    for (uint32_t k = 0; k < numSlots; ++k)
    {
        GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, k);
        if (!r.valid)
        {
            continue;
        }
        ++validSlots;
        if (r.satPath.size() > 1)
        {
            hasHop = true;
        }
    }

    if (validSlotsOut)
    {
        *validSlotsOut = validSlots;
    }
    return hasHop;
}

static void
PrintLayerVerdict(const std::string& layer, bool applicable, bool pass, const std::string& detail)
{
    std::cout << "[" << layer << "] "
              << (applicable ? (pass ? "PASS" : "FAIL") : "not_applicable")
              << " | " << detail << "\n";
}

static double
EstimateTheoreticalTotalDelayMs(Ptr<IslRoutingManager> routingMgr,
                                const E2EConfig&       cfg,
                                uint32_t               slot)
{
    if (!routingMgr)
    {
        return -1.0;
    }

    std::vector<Vector> pos =
        routingMgr->GetPositionsAt(Seconds(slot * routingMgr->GetTimeSlotInterval()));

    if (cfg.pathType == "gw2gw_e2e")
    {
        GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
        const GatewayPreset* srcGw = FindGatewayPreset(cfg.gwSrc);
        const GatewayPreset* dstGw = FindGatewayPreset(cfg.gwDst);
        if (!r.valid || !srcGw || !dstGw || r.entrySatId >= pos.size() || r.exitSatId >= pos.size())
        {
            return -1.0;
        }
        return r.islCost * 1000.0 +
               GroundToSatDelayMs(srcGw->latDeg, srcGw->lonDeg, pos[r.entrySatId]) +
               GroundToSatDelayMs(dstGw->latDeg, dstGw->lonDeg, pos[r.exitSatId]);
    }

    if (cfg.pathType == "gw2ut_e2e")
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        const GatewayPreset* gw = FindGatewayPreset(cfg.gwId);
        if (!r.valid || !gw || r.entrySatId >= pos.size() || r.servingSatId >= pos.size())
        {
            return -1.0;
        }
        return r.islCost * 1000.0 +
               GroundToSatDelayMs(gw->latDeg, gw->lonDeg, pos[r.entrySatId]) +
               GroundToSatDelayMs(cfg.utLatDeg, cfg.utLonDeg, pos[r.servingSatId]);
    }

    if (cfg.pathType == "sat2ut")
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        if (!r.valid || r.servingSatId >= pos.size())
        {
            return -1.0;
        }
        return GroundToSatDelayMs(cfg.utLatDeg, cfg.utLonDeg, pos[r.servingSatId]);
    }

    if (cfg.pathType == "sat2sat")
    {
        std::vector<uint32_t> path = routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot);
        return ComputeIslPropagationDelayMs(routingMgr, path, slot, false);
    }

    return -1.0;
}

static double
EstimateTheoreticalIslPropagationDelayMs(Ptr<IslRoutingManager> routingMgr,
                                         const E2EConfig&       cfg,
                                         uint32_t               slot)
{
    if (!routingMgr)
    {
        return -1.0;
    }

    if (cfg.pathType == "gw2gw_e2e")
    {
        GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
        if (!r.valid || r.satPath.empty() || r.satPath.back() == UINT32_MAX)
        {
            return -1.0;
        }
        return ComputeIslPropagationDelayMs(routingMgr, r.satPath, slot, false);
    }

    if (cfg.pathType == "gw2ut_e2e")
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        if (!r.valid || r.satPath.empty() || r.satPath.back() == UINT32_MAX)
        {
            return -1.0;
        }
        return ComputeIslPropagationDelayMs(routingMgr, r.satPath, slot, false);
    }

    if (cfg.pathType == "sat2sat")
    {
        std::vector<uint32_t> path = routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot);
        if (path.empty() || path.back() == UINT32_MAX)
        {
            return -1.0;
        }
        return ComputeIslPropagationDelayMs(routingMgr, path, slot, false);
    }

    return -1.0;
}

static uint32_t
EstimateTheoreticalIslHopCount(Ptr<IslRoutingManager> routingMgr,
                               const E2EConfig&       cfg,
                               uint32_t               slot)
{
    if (!routingMgr)
    {
        return 0;
    }

    std::vector<uint32_t> path;
    if (cfg.pathType == "gw2gw_e2e")
    {
        GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
        if (r.valid)
        {
            path = r.satPath;
        }
    }
    else if (cfg.pathType == "gw2ut_e2e")
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        if (r.valid)
        {
            path = r.satPath;
        }
    }
    else if (cfg.pathType == "sat2sat")
    {
        path = routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot);
    }

    if (path.size() < 2 || path.back() == UINT32_MAX)
    {
        return 0;
    }
    return static_cast<uint32_t>(path.size() - 1);
}

static std::string
FormatObservedPacketDelayDiagnostic(double appAvgDelayMs, uint64_t samples, double theoryMs)
{
    std::ostringstream oss;
    oss << " observedPacketDelay=";
    if (samples == 0)
    {
        oss << "N/A";
        return oss.str();
    }

    oss << std::fixed << std::setprecision(3) << appAvgDelayMs << "ms";
    if (theoryMs > 0.0)
    {
        oss << " theoreticalPathDelay=" << theoryMs
            << "ms note=includes_non_phy_stack_delay";
    }
    else
    {
        oss << " theoreticalPathDelay=N/A";
    }
    return oss.str();
}

static void
PrintE2EFinalVerdict(Ptr<IslRoutingManager> routingMgr,
                     const E2EConfig&       cfg,
                     uint32_t               numSlots)
{
    const PathTypeSpec spec = GetPathTypeSpec(cfg.pathType);
    const ObsScope& verdictScope = g_obsVerdictScope;

    std::cout << "\n=== E2E PathType Verdict ===\n";
    std::cout << "pathType=" << cfg.pathType << "\n";

    if (cfg.pathType == "gw2gw_e2e")
    {
        uint32_t validSlots{0};
        bool hasIslHop = GwGwRouteHasAnyIslHop(routingMgr, cfg, numSlots, &validSlots);
        uint64_t islRxPkts = SumScopedRxPktsInScope("isl", verdictScope);
        uint64_t feederRxPkts = SumScopedRxPktsInScope("feeder", verdictScope);
        uint64_t feederDelaySamples = CountScopedDelaySamplesInScope("feeder", verdictScope);
        double feederAvgDelayMs = AvgScopedDelayMsInScope("feeder", verdictScope);
        bool routePass = (validSlots > 0);
        double appAvgDelayMs = (g_gw2gwDelivery.delaySamples > 0)
                                   ? (g_gw2gwDelivery.sumDelayMs /
                                      static_cast<double>(g_gw2gwDelivery.delaySamples))
                                   : 0.0;
        const double theoryTotalDelayMs = EstimateTheoreticalTotalDelayMs(routingMgr, cfg, 0);
        const uint64_t propSamples = CountIslPropagationSamplesInScope(verdictScope);
        const double propAvgMs = AvgIslPropagationDelayMsInScope(verdictScope);
        const ScopedIslPhyCheck phyCheck =
            EvaluateScopedIslPhyDelay(routingMgr, cfg, verdictScope, numSlots);
        std::string observedPacketDelayDiag =
            FormatObservedPacketDelayDiagnostic(appAvgDelayMs,
                                               g_gw2gwDelivery.delaySamples,
                                               theoryTotalDelayMs);
        bool packetPass = g_gw2gwDelivery.traceConnected &&
                          g_gw2gwDelivery.traceRxBytes > 0;

        PrintLayerVerdict("ROUTING_LAYER",
                          true,
                          routePass,
                          "validSlots=" + std::to_string(validSlots) +
                              "/" + std::to_string(numSlots) +
                              " gwSrc=" + std::to_string(cfg.gwSrc) +
                              " gwDst=" + std::to_string(cfg.gwDst) +
                              " obsFeederMode=" + ToString(spec.obsFeederMode));
        PrintLayerVerdict("FEEDER_LAYER",
                          true,
                          feederRxPkts > 0,
                          "scopedKeys=" + std::to_string(verdictScope.feederKeys.size()) +
                              " scopedRxPkts=" + std::to_string(feederRxPkts) +
                              " " + FormatDelayMetric("avgDelay",
                                                        feederDelaySamples,
                                                        feederAvgDelayMs));
        if (routePass && !hasIslHop)
        {
            PrintLayerVerdict("ISL_TRANSIT_LAYER",
                              false,
                              false,
                              "valid GW-GW route has no ISL hop");
            PrintLayerVerdict("ISL_PHY_LAYER",
                              false,
                              false,
                              "valid GW-GW route has no ISL hop");
        }
        else
        {
            PrintLayerVerdict("ISL_TRANSIT_LAYER",
                              true,
                              routePass &&
                                  !verdictScope.islKeys.empty() &&
                                  islRxPkts > 0,
                              "scopedLinks=" + std::to_string(verdictScope.islKeys.size()) +
                                  " scopedRxPkts=" + std::to_string(islRxPkts));
            PrintLayerVerdict("ISL_PHY_LAYER",
                              true,
                              routePass &&
                                  !verdictScope.islKeys.empty() &&
                                  islRxPkts > 0 &&
                                  phyCheck.applicable &&
                                  phyCheck.pass,
                              "channelDelaySamples=" + std::to_string(propSamples) +
                                  " " + FormatDelayMetric("channelAvgDelay", propSamples, propAvgMs) +
                                  " " + phyCheck.detail);
        }
        PrintLayerVerdict("PACKET_LAYER",
                          true,
                          packetPass,
                          "appInstalled=" + std::string(g_gw2gwDelivery.installed ? "1" : "0") +
                              " traceConnected=" +
                              std::string(g_gw2gwDelivery.traceConnected ? "1" : "0") +
                              " traceRxPkts=" + std::to_string(g_gw2gwDelivery.traceRxPkts) +
                              " traceRxBytes=" + std::to_string(g_gw2gwDelivery.traceRxBytes) +
                              " delaySamples=" + std::to_string(g_gw2gwDelivery.delaySamples) +
	                              " avgOneWayDelayMs=" +
	                              (g_gw2gwDelivery.delaySamples > 0
	                                   ? [&]()
	                                     {
	                                         std::ostringstream oss;
	                                         oss << std::fixed << std::setprecision(3)
	                                             << appAvgDelayMs;
	                                         return oss.str();
	                                     }()
	                                   : std::string("N/A")) +
                                  observedPacketDelayDiag +
	                              " reported=" + std::string(g_gw2gwDelivery.reported ? "1" : "0") +
	                              " sinkTotalRxBytes=" + std::to_string(g_gw2gwDelivery.rxBytes) +
	                              " sinkEstPkts=" + std::to_string(g_gw2gwDelivery.estPkts));
        PrintLayerVerdict("SERVICE_LAYER", false, false, "not part of gw2gw_e2e");
        return;
    }

    if (verdictScope.activeFeeder)
    {
        uint64_t rxPkts = SumScopedRxPktsInScope("feeder", verdictScope);
        uint64_t delaySamples = CountScopedDelaySamplesInScope("feeder", verdictScope);
        double avgDelayMs = AvgScopedDelayMsInScope("feeder", verdictScope);
        bool feederPass = rxPkts > 0;
        std::string detail =
            "scopedKeys=" + std::to_string(verdictScope.feederKeys.size()) +
            " scopedRxPkts=" + std::to_string(rxPkts) +
            " " + FormatDelayMetric("avgDelay", delaySamples, avgDelayMs);

        if (!feederPass && cfg.pathType == "gw2ut_e2e" && g_totalOrbiterFeederRxCalls > 0)
        {
            detail += " orbiterRxFeederCalls=" + std::to_string(g_totalOrbiterFeederRxCalls) +
                      " entry_scope_mismatch=observed verdict=diag_only";
        }

        PrintLayerVerdict("FEEDER_LAYER",
                          true,
                          feederPass,
                          detail);
    }
    else
    {
        std::string detail = spec.usesFeederlink
            ? "traffic uses feederlink; obsFeederMode=" + std::string(ToString(spec.obsFeederMode))
            : "not part of " + cfg.pathType;
        PrintLayerVerdict("FEEDER_LAYER", false, false, detail);
    }

    if (verdictScope.activeService)
    {
        uint64_t satRxPkts = SumScopedRxPktsWithPrefixInScope("service", "sat", verdictScope);
        uint64_t utRxPkts = SumScopedRxPktsWithPrefixInScope("service", "ut", verdictScope);
        uint64_t rxPkts = satRxPkts + utRxPkts;
        uint64_t satDelaySamples =
            CountScopedDelaySamplesWithPrefixInScope("service", "sat", verdictScope);
        uint64_t utDelaySamples =
            CountScopedDelaySamplesWithPrefixInScope("service", "ut", verdictScope);
        double satAvgDelayMs =
            AvgScopedDelayMsWithPrefixInScope("service", "sat", verdictScope);
        double utAvgDelayMs =
            AvgScopedDelayMsWithPrefixInScope("service", "ut", verdictScope);
        std::string detail =
            "satKeys=" + std::to_string(CountScopedKeysWithPrefix(verdictScope.serviceKeys, "sat")) +
            " satRxPkts=" + std::to_string(satRxPkts) +
            " " + FormatDelayMetric("satAvgDelay", satDelaySamples, satAvgDelayMs) +
            " utKeys=" + std::to_string(CountScopedKeysWithPrefix(verdictScope.serviceKeys, "ut")) +
            " utRxPkts=" + std::to_string(utRxPkts) +
            " " + FormatDelayMetric("utAvgDelay", utDelaySamples, utAvgDelayMs);
        if (satRxPkts > 0 && utRxPkts == 0)
        {
            detail += " endpoint_dev_rx=not_observed";
        }

        bool servicePass = rxPkts > 0;
        if (cfg.pathType == "sat2ut" || cfg.pathType == "gw2ut_e2e")
        {
            servicePass = (utRxPkts > 0);
        }

        PrintLayerVerdict("SERVICE_LAYER",
                          true,
                          servicePass,
                          detail);
    }
    else
    {
        PrintLayerVerdict("SERVICE_LAYER", false, false, "not part of " + cfg.pathType);
    }

    if (cfg.pathType == "sat2ut")
    {
        if (!g_sat2utDelivery.installed)
        {
            PrintLayerVerdict("PACKET_LAYER",
                              true,
                              false,
                              "sat2ut packet app was not installed");
        }
        else
        {
            bool packetPass = g_sat2utDelivery.traceConnected &&
                              g_sat2utDelivery.traceRxBytes > 0;
            PrintLayerVerdict("PACKET_LAYER",
                              true,
                              packetPass,
                              "appInstalled=" + std::string(g_sat2utDelivery.installed ? "1" : "0") +
                                  " traceConnected=" +
                                  std::string(g_sat2utDelivery.traceConnected ? "1" : "0") +
                                  " traceRxPkts=" + std::to_string(g_sat2utDelivery.traceRxPkts) +
                                  " traceRxBytes=" + std::to_string(g_sat2utDelivery.traceRxBytes) +
                                  " delaySamples=" + std::to_string(g_sat2utDelivery.delaySamples) +
                                  " avgOneWayDelayMs=" +
                                  (g_sat2utDelivery.delaySamples > 0
                                       ? [&]()
                                         {
                                             std::ostringstream oss;
                                             oss << std::fixed << std::setprecision(3)
                                                 << (g_sat2utDelivery.sumDelayMs /
                                                     static_cast<double>(g_sat2utDelivery.delaySamples));
                                             return oss.str();
                                         }()
                                       : std::string("N/A")) +
                                  " reported=" + std::string(g_sat2utDelivery.reported ? "1" : "0") +
                                  " sinkTotalRxBytes=" + std::to_string(g_sat2utDelivery.rxBytes) +
                                  " sinkEstPkts=" + std::to_string(g_sat2utDelivery.estPkts));
        }
    }
    else if (cfg.pathType == "sat2sat")
    {
        bool packetPass = g_sat2satDelivery.traceConnected &&
                          g_sat2satDelivery.traceRxBytes > 0;
        PrintLayerVerdict("PACKET_LAYER",
                          true,
                          packetPass,
                          "appInstalled=" + std::string(g_sat2satDelivery.installed ? "1" : "0") +
                              " traceConnected=" +
                              std::string(g_sat2satDelivery.traceConnected ? "1" : "0") +
                              " traceRxPkts=" + std::to_string(g_sat2satDelivery.traceRxPkts) +
                              " traceRxBytes=" + std::to_string(g_sat2satDelivery.traceRxBytes) +
                              " delaySamples=" + std::to_string(g_sat2satDelivery.delaySamples) +
                              " avgOneWayDelayMs=" +
                              (g_sat2satDelivery.delaySamples > 0
                                   ? [&]()
                                     {
                                         std::ostringstream oss;
                                         oss << std::fixed << std::setprecision(3)
                                             << (g_sat2satDelivery.sumDelayMs /
                                                 static_cast<double>(g_sat2satDelivery.delaySamples));
                                         return oss.str();
                                     }()
                                   : std::string("N/A")) +
                              " reported=" + std::string(g_sat2satDelivery.reported ? "1" : "0") +
                              " sinkTotalRxBytes=" + std::to_string(g_sat2satDelivery.rxBytes) +
                              " sinkEstPkts=" + std::to_string(g_sat2satDelivery.estPkts));
    }

    if (verdictScope.activeIsl)
    {
        if (cfg.pathType == "gw2ut_e2e")
        {
            uint32_t validSlots{0};
            bool hasIslHop = GwUtRouteHasAnyIslHop(routingMgr, cfg, numSlots, &validSlots);
            if (validSlots > 0 && !hasIslHop)
            {
                PrintLayerVerdict("ISL_TRANSIT_LAYER",
                                  false,
                                  false,
                                  "valid route has no ISL hop");
                PrintLayerVerdict("ISL_PHY_LAYER",
                                  false,
                                  false,
                                  "valid route has no ISL hop");
                return;
            }
        }

        uint64_t rxPkts = SumScopedRxPktsInScope("isl", verdictScope);
        uint64_t delaySamples = CountScopedDelaySamplesInScope("isl", verdictScope);
        double avgDelayMs = AvgScopedDelayMsInScope("isl", verdictScope);
        uint64_t propSamples = CountIslPropagationSamplesInScope(verdictScope);
        double propAvgMs = AvgIslPropagationDelayMsInScope(verdictScope);
        const ScopedIslPhyCheck phyCheck =
            EvaluateScopedIslPhyDelay(routingMgr, cfg, verdictScope, numSlots);
        PrintLayerVerdict("ISL_TRANSIT_LAYER",
                          true,
                          !verdictScope.islKeys.empty() && rxPkts > 0,
                          "scopedLinks=" + std::to_string(verdictScope.islKeys.size()) +
                              " scopedRxPkts=" + std::to_string(rxPkts) +
                              " " + FormatDelayMetric("avgDelay", delaySamples, avgDelayMs));
        PrintLayerVerdict("ISL_PHY_LAYER",
                          true,
                          !verdictScope.islKeys.empty() &&
                              rxPkts > 0 &&
                              phyCheck.applicable &&
                              phyCheck.pass,
                          "channelDelaySamples=" + std::to_string(propSamples) +
                              " " + FormatDelayMetric("channelAvgDelay", propSamples, propAvgMs) +
                              " " + phyCheck.detail);
    }
    else
    {
        PrintLayerVerdict("ISL_TRANSIT_LAYER", false, false, "not part of " + cfg.pathType);
        PrintLayerVerdict("ISL_PHY_LAYER", false, false, "not part of " + cfg.pathType);
    }
}

struct DelayMatrixRow
{
    std::string pathType;
    std::string status;
    std::string reason;
    std::string regenerationMode;
    std::string srcEndpoint;
    std::string dstEndpoint;
    std::string srcIp;
    std::string dstIp;
    uint32_t    slot{0};
    double      slotTimeSec{0.0};
    std::string routeSatPath;
    std::string feederKeys;
    std::string serviceKeys;
    std::string islKeys;
    uint64_t    rxPkts{0};
    uint64_t    rxBytes{0};
    uint64_t    dropPkts{0};
    uint64_t    appDelaySamples{0};
    double      appAvgDelayMs{0.0};
    double      appMinDelayMs{0.0};
    double      appMaxDelayMs{0.0};
    uint64_t    feederDelaySamples{0};
    double      feederAvgDelayMs{0.0};
    uint64_t    serviceDelaySamples{0};
    double      serviceAvgDelayMs{0.0};
    uint64_t    islRxPkts{0};
    uint64_t    islDropPkts{0};
    uint64_t    islChannelDelaySamples{0};
    double      islChannelAvgDelayMs{0.0};
    double      theoreticalIslDelayMs{-1.0};
    double      theoreticalFeederDelayMs{-1.0};
    double      theoreticalServiceDelayMs{-1.0};
    double      theoreticalTotalDelayMs{-1.0};
};

static std::string
NodeLabel(const std::string& label, Ptr<Node> node)
{
    std::ostringstream oss;
    oss << label << "=";
    if (node)
    {
        oss << "node" << node->GetId();
    }
    else
    {
        oss << "missing";
    }
    return oss.str();
}

static Ptr<Node>
GetGwEndpointNode(uint32_t gwId)
{
    return GetPhysicalGwNodeOrNull(gwId);
}

static std::string
GetGwEndpointIpString(Ptr<SimulationHelper> /*simHelper*/, uint32_t gwId)
{
    Ptr<Node> node = GetGwEndpointNode(gwId);
    if (!node)
    {
        return "";
    }
    Ipv4Address addr;
    if (!TryGetPhysicalGwRoutableIp(node, gwId, addr))
    {
        return "unavailable";
    }
    return Ipv4ToString(addr);
}

static uint64_t
SumDropsInScope(const std::string& linkType, const ObsScope& scope)
{
    const std::map<std::string, SegLinkStats>* statsMap = GetObsStatsMap(linkType);
    uint64_t total{0};
    if (!statsMap)
    {
        return total;
    }
    for (const auto& kv : *statsMap)
    {
        if (IsObsKeyInScope(scope, linkType, kv.first))
        {
            total += kv.second.dropPkts;
        }
    }
    return total;
}

static double
DistanceDelayMs(const Vector& a, const Vector& b)
{
    const double c = 3.0e8;
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) / c * 1000.0;
}

static Vector
GroundToEcef(double latDeg, double lonDeg)
{
    static const double earthRadiusM = 6371000.0;
    double latRad = latDeg * M_PI / 180.0;
    double lonRad = lonDeg * M_PI / 180.0;
    return Vector(earthRadiusM * std::cos(latRad) * std::cos(lonRad),
                  earthRadiusM * std::cos(latRad) * std::sin(lonRad),
                  earthRadiusM * std::sin(latRad));
}

static double
GroundToSatDelayMs(double latDeg, double lonDeg, const Vector& satEcef)
{
    return DistanceDelayMs(GroundToEcef(latDeg, lonDeg), satEcef);
}

static double
ComputeIslPropagationDelayMs(Ptr<IslRoutingManager> routingMgr,
                             const std::vector<uint32_t>& path,
                             uint32_t slot,
                             bool printHops)
{
    if (path.size() <= 1 || path.back() == UINT32_MAX)
    {
        return 0.0;
    }

    std::vector<Vector> pos =
        routingMgr->GetPositionsAt(Seconds(slot * routingMgr->GetTimeSlotInterval()));
    double totalMs = 0.0;

    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        if (path[i] == UINT32_MAX || path[i + 1] == UINT32_MAX ||
            path[i] >= pos.size() || path[i + 1] >= pos.size())
        {
            continue;
        }

        double hopMs = DistanceDelayMs(pos[path[i]], pos[path[i + 1]]);
        totalMs += hopMs;

        if (printHops)
        {
            double dx = pos[path[i]].x - pos[path[i + 1]].x;
            double dy = pos[path[i]].y - pos[path[i + 1]].y;
            double dz = pos[path[i]].z - pos[path[i + 1]].z;
            double distKm = std::sqrt(dx * dx + dy * dy + dz * dz) / 1000.0;
            std::cout << "[PROP][ISL_HOP] slot=" << slot
                      << " " << RouteNodeToString(path[i])
                      << "->" << RouteNodeToString(path[i + 1])
                      << " dist_km=" << std::fixed << std::setprecision(2) << distKm
                      << " theory_ms=" << std::setprecision(3) << hopMs
                      << " ns3_channel_ms=applied_on_transmit\n";
        }
    }

    return totalMs;
}

static void
FillTheoreticalDelay(Ptr<IslRoutingManager> routingMgr,
                     const E2EConfig& cfg,
                     uint32_t slot,
                     DelayMatrixRow& row)
{
    std::vector<Vector> pos =
        routingMgr->GetPositionsAt(Seconds(slot * routingMgr->GetTimeSlotInterval()));

    auto addIfKnown = [](double a, double b) {
        if (a < 0.0) return b;
        if (b < 0.0) return a;
        return a + b;
    };

    if (cfg.pathType == "gw2gw_e2e")
    {
        GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
        const GatewayPreset* srcGw = FindGatewayPreset(cfg.gwSrc);
        const GatewayPreset* dstGw = FindGatewayPreset(cfg.gwDst);
        if (r.valid && srcGw && dstGw &&
            r.entrySatId < pos.size() && r.exitSatId < pos.size())
        {
            row.theoreticalIslDelayMs = r.islCost * 1000.0;
            row.theoreticalFeederDelayMs =
                GroundToSatDelayMs(srcGw->latDeg, srcGw->lonDeg, pos[r.entrySatId]) +
                GroundToSatDelayMs(dstGw->latDeg, dstGw->lonDeg, pos[r.exitSatId]);
        }
    }
    else if (cfg.pathType == "gw2ut_e2e")
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        const GatewayPreset* gw = FindGatewayPreset(cfg.gwId);
        if (r.valid && gw &&
            r.entrySatId < pos.size() && r.servingSatId < pos.size())
        {
            row.theoreticalIslDelayMs = r.islCost * 1000.0;
            row.theoreticalFeederDelayMs =
                GroundToSatDelayMs(gw->latDeg, gw->lonDeg, pos[r.entrySatId]);
            row.theoreticalServiceDelayMs =
                GroundToSatDelayMs(cfg.utLatDeg, cfg.utLonDeg, pos[r.servingSatId]);
        }
    }
    else if (cfg.pathType == "sat2sat")
    {
        std::vector<uint32_t> path = routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot);
        if (!path.empty() && path.back() != UINT32_MAX)
        {
            row.theoreticalIslDelayMs =
                ComputeIslPropagationDelayMs(routingMgr, path, slot, false);
        }
    }
    else if (cfg.pathType == "sat2ut")
    {
        GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        if (r.valid && r.servingSatId < pos.size())
        {
            row.theoreticalServiceDelayMs =
                GroundToSatDelayMs(cfg.utLatDeg, cfg.utLonDeg, pos[r.servingSatId]);
        }
    }
    else if (cfg.pathType == "gw2sat" || cfg.pathType == "sat2gw")
    {
        const GatewayPreset* gw = FindGatewayPreset(cfg.gwId);
        const auto& sats = routingMgr->GetGwVisibleSats(cfg.gwId, slot);
        if (gw && !sats.empty())
        {
            double bestMs = std::numeric_limits<double>::infinity();
            for (uint32_t satId : sats)
            {
                if (satId < pos.size())
                {
                    bestMs = std::min(bestMs,
                                      GroundToSatDelayMs(gw->latDeg, gw->lonDeg, pos[satId]));
                }
            }
            if (bestMs < std::numeric_limits<double>::infinity())
            {
                row.theoreticalFeederDelayMs = bestMs;
            }
        }
    }

    row.theoreticalTotalDelayMs = addIfKnown(row.theoreticalIslDelayMs,
                                             row.theoreticalFeederDelayMs);
    row.theoreticalTotalDelayMs = addIfKnown(row.theoreticalTotalDelayMs,
                                             row.theoreticalServiceDelayMs);
}

static DelayMatrixRow
BuildMeasuredDelayRow(Ptr<SimulationHelper> simHelper,
                      Ptr<IslRoutingManager> routingMgr,
                      const E2EConfig&       cfg,
                      uint32_t               slot)
{
    DelayMatrixRow row;
    row.pathType = cfg.pathType;
    row.status = "measured";
    row.regenerationMode = cfg.regenerationMode;
    row.slot = slot;
    row.slotTimeSec = slot * routingMgr->GetTimeSlotInterval();
    row.feederKeys = SetToString(g_obsVerdictScope.feederKeys);
    row.serviceKeys = SetToString(g_obsVerdictScope.serviceKeys);
    row.islKeys = SetToString(g_obsVerdictScope.islKeys);

    if (cfg.pathType == "gw2gw_e2e")
    {
        Ptr<Node> src = GetGwEndpointNode(cfg.gwSrc);
        Ptr<Node> dst = GetGwEndpointNode(cfg.gwDst);
        row.srcEndpoint = NodeLabel("gw" + std::to_string(cfg.gwSrc), src);
        row.dstEndpoint = NodeLabel("gw" + std::to_string(cfg.gwDst), dst);
        row.srcIp = Ipv4ToString(g_gw2gwDelivery.srcAddr);
        row.dstIp = Ipv4ToString(g_gw2gwDelivery.dstAddr);
        GwToGwRoute route = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
        row.routeSatPath = route.valid ? SatPathToString(route.satPath) : "invalid_route";
        row.rxPkts = g_gw2gwDelivery.traceRxPkts;
        row.rxBytes = g_gw2gwDelivery.traceRxBytes;
        row.appDelaySamples = g_gw2gwDelivery.delaySamples;
        if (row.appDelaySamples > 0)
        {
            row.appAvgDelayMs =
                g_gw2gwDelivery.sumDelayMs / static_cast<double>(g_gw2gwDelivery.delaySamples);
            row.appMinDelayMs = g_gw2gwDelivery.minDelayMs;
            row.appMaxDelayMs = g_gw2gwDelivery.maxDelayMs;
        }
    }
    else if (cfg.pathType == "gw2ut_e2e")
    {
        Ptr<Node> src = GetGwEndpointNode(cfg.gwId);
        Ptr<Node> utUser = GetUtUserNodeOrNull(cfg.trafficUtUserId);
        row.srcEndpoint = NodeLabel("gw" + std::to_string(cfg.gwId), src);
        row.dstEndpoint = NodeLabel("utUser" + std::to_string(cfg.trafficUtUserId), utUser);
        row.srcIp = GetGwEndpointIpString(simHelper, cfg.gwId);
        row.dstIp = "unavailable";
        GwToUtRoute route = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        row.routeSatPath = route.valid ? SatPathToString(route.satPath) : "invalid_route";
        row.rxPkts = SumScopedRxPktsInScope("service", g_obsVerdictScope);
        row.rxBytes = 0;
    }
    else if (cfg.pathType == "sat2sat")
    {
        NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();
        row.srcEndpoint = NodeLabel("sat" + std::to_string(cfg.satSrc),
                                    cfg.satSrc < sats.GetN() ? sats.Get(cfg.satSrc) : nullptr);
        row.dstEndpoint = NodeLabel("sat" + std::to_string(cfg.satDst),
                                    cfg.satDst < sats.GetN() ? sats.Get(cfg.satDst) : nullptr);
        row.srcIp = Ipv4ToString(g_sat2satDelivery.srcAddr);
        row.dstIp = Ipv4ToString(g_sat2satDelivery.dstAddr);
        row.routeSatPath = SatPathToString(routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot));
        row.rxPkts = g_sat2satDelivery.traceRxPkts;
        row.rxBytes = g_sat2satDelivery.traceRxBytes;
        row.appDelaySamples = g_sat2satDelivery.delaySamples;
        if (row.appDelaySamples > 0)
        {
            row.appAvgDelayMs =
                g_sat2satDelivery.sumDelayMs / static_cast<double>(g_sat2satDelivery.delaySamples);
            row.appMinDelayMs = g_sat2satDelivery.minDelayMs;
            row.appMaxDelayMs = g_sat2satDelivery.maxDelayMs;
        }
    }
    else if (cfg.pathType == "sat2ut")
    {
        Ptr<Node> utUser = GetUtUserNodeOrNull(cfg.trafficUtUserId);
        row.srcEndpoint = "sat=route_serving";
        row.dstEndpoint = NodeLabel("utUser" + std::to_string(cfg.trafficUtUserId), utUser);
        row.dstIp = "unavailable";
        GwToUtRoute route = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
        row.routeSatPath = route.valid ? RouteNodeToString(route.servingSatId) : "invalid_route";
        row.rxPkts = g_sat2utDelivery.installed
                         ? g_sat2utDelivery.traceRxPkts
                         : SumScopedRxPktsInScope("service", g_obsVerdictScope);
        row.rxBytes = g_sat2utDelivery.traceRxBytes;
        row.appDelaySamples = g_sat2utDelivery.delaySamples;
        if (row.appDelaySamples > 0)
        {
            row.appAvgDelayMs =
                g_sat2utDelivery.sumDelayMs / static_cast<double>(g_sat2utDelivery.delaySamples);
            row.appMinDelayMs = g_sat2utDelivery.minDelayMs;
            row.appMaxDelayMs = g_sat2utDelivery.maxDelayMs;
        }
    }
    else if (cfg.pathType == "gw2sat" || cfg.pathType == "sat2gw")
    {
        Ptr<Node> gw = GetGwEndpointNode(cfg.gwId);
        row.srcEndpoint = (cfg.pathType == "gw2sat")
                              ? NodeLabel("gw" + std::to_string(cfg.gwId), gw)
                              : "sat=visible";
        row.dstEndpoint = (cfg.pathType == "sat2gw")
                              ? NodeLabel("gw" + std::to_string(cfg.gwId), gw)
                              : "sat=visible";
        row.srcIp = (cfg.pathType == "gw2sat") ? GetGwEndpointIpString(simHelper, cfg.gwId) : "";
        row.dstIp = (cfg.pathType == "sat2gw") ? GetGwEndpointIpString(simHelper, cfg.gwId) : "";
        row.routeSatPath = SetToString(routingMgr->GetGwVisibleSats(cfg.gwId, slot));
        row.rxPkts = SumScopedRxPktsInScope("feeder", g_obsVerdictScope);
    }

    row.dropPkts = SumDropsInScope("feeder", g_obsVerdictScope) +
                   SumDropsInScope("service", g_obsVerdictScope) +
                   SumDropsInScope("isl", g_obsVerdictScope);
    row.feederDelaySamples = CountScopedDelaySamplesInScope("feeder", g_obsVerdictScope);
    row.feederAvgDelayMs = AvgScopedDelayMsInScope("feeder", g_obsVerdictScope);
    row.serviceDelaySamples = CountScopedDelaySamplesInScope("service", g_obsVerdictScope);
    row.serviceAvgDelayMs = AvgScopedDelayMsInScope("service", g_obsVerdictScope);
    row.islRxPkts = SumScopedRxPktsInScope("isl", g_obsVerdictScope);
    row.islDropPkts = SumDropsInScope("isl", g_obsVerdictScope);
    row.islChannelDelaySamples = CountIslPropagationSamplesInScopeForSlot(g_obsVerdictScope, slot);
    row.islChannelAvgDelayMs = AvgIslPropagationDelayMsInScopeForSlot(g_obsVerdictScope, slot);
    FillTheoreticalDelay(routingMgr, cfg, slot, row);
    return row;
}

static void
WriteDelayMatrix(const std::string&          path,
                 const std::vector<DelayMatrixRow>& rows)
{
    std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        std::cout << "[DELAY_MATRIX] WARNING: cannot open " << path << "\n";
        return;
    }

    out << "path_type,status,reason,regeneration_mode,src_endpoint,dst_endpoint,"
           "src_ip,dst_ip,slot,slot_time_s,route_sat_path,feeder_keys,service_keys,isl_keys,"
           "rx_pkts,rx_bytes,drop_pkts,app_delay_samples,app_avg_delay_ms,app_min_delay_ms,"
           "app_max_delay_ms,feeder_delay_samples,feeder_avg_delay_ms,service_delay_samples,"
           "service_avg_delay_ms,isl_rx_pkts,isl_drop_pkts,isl_channel_delay_samples,"
           "isl_channel_avg_delay_ms,theoretical_isl_delay_ms,theoretical_feeder_delay_ms,"
           "theoretical_service_delay_ms,theoretical_total_delay_ms\n";
    for (const auto& row : rows)
    {
        out << CsvEscape(row.pathType) << ","
            << CsvEscape(row.status) << ","
            << CsvEscape(row.reason) << ","
            << CsvEscape(row.regenerationMode) << ","
            << CsvEscape(row.srcEndpoint) << ","
            << CsvEscape(row.dstEndpoint) << ","
            << CsvEscape(row.srcIp) << ","
            << CsvEscape(row.dstIp) << ","
            << row.slot << ","
            << std::fixed << std::setprecision(3) << row.slotTimeSec << ","
            << CsvEscape(row.routeSatPath) << ","
            << CsvEscape(row.feederKeys) << ","
            << CsvEscape(row.serviceKeys) << ","
            << CsvEscape(row.islKeys) << ","
            << row.rxPkts << ","
            << row.rxBytes << ","
            << row.dropPkts << ","
            << row.appDelaySamples << ","
            << row.appAvgDelayMs << ","
            << row.appMinDelayMs << ","
            << row.appMaxDelayMs << ","
            << row.feederDelaySamples << ","
            << row.feederAvgDelayMs << ","
            << row.serviceDelaySamples << ","
            << row.serviceAvgDelayMs << ","
            << row.islRxPkts << ","
            << row.islDropPkts << ","
            << row.islChannelDelaySamples << ","
            << row.islChannelAvgDelayMs << ","
            << row.theoreticalIslDelayMs << ","
            << row.theoreticalFeederDelayMs << ","
            << row.theoreticalServiceDelayMs << ","
            << row.theoreticalTotalDelayMs << "\n";
    }

    std::cout << "\n=== Real Delay Matrix ===\n";
    for (const auto& row : rows)
    {
        std::cout << row.pathType
                  << " status=" << row.status
                  << " rxPkts=" << row.rxPkts
                  << " appDelaySamples=" << row.appDelaySamples
                  << " observedPacketAvgDelayMs=" << std::fixed << std::setprecision(3)
                  << row.appAvgDelayMs
                  << " feederAvgDelayMs=" << row.feederAvgDelayMs
                  << " serviceAvgDelayMs=" << row.serviceAvgDelayMs
                  << " islPhyChannelAvgDelayMs=" << row.islChannelAvgDelayMs
                  << " theoreticalTotalDelayMs=" << row.theoreticalTotalDelayMs
                  << " reason=" << row.reason << "\n";
        if (row.theoreticalTotalDelayMs > 0.0 &&
            row.appDelaySamples > 0 &&
            row.appAvgDelayMs < 1.0)
        {
            std::cout << "[DELAY_WARN] appAvgDelayMs=" << row.appAvgDelayMs
                      << "ms is near-zero while theoreticalTotalDelayMs="
                      << row.theoreticalTotalDelayMs << "ms. "
                      << "Check channelDelaySamples/islChannelAvgDelayMs to confirm "
                      << "whether packets actually crossed delayed ISL channels.\n";
        }
    }
    std::cout << "CSV: " << path << "\n";
    std::cout << "=========================\n\n";
}

static std::vector<DelayMatrixRow>
BuildDelayMatrixRows(Ptr<SimulationHelper> simHelper,
                     Ptr<IslRoutingManager> routingMgr,
                     const E2EConfig&       cfg,
                     bool                   delayMatrix)
{
    std::vector<DelayMatrixRow> rows;
    rows.push_back(BuildMeasuredDelayRow(simHelper, routingMgr, cfg, 0));

    if (!delayMatrix)
    {
        return rows;
    }

    const std::vector<std::string> allPaths = {
        "gw2gw_e2e", "gw2ut_e2e", "sat2ut", "sat2gw", "gw2sat", "sat2sat"};
    for (const auto& pathType : allPaths)
    {
        if (pathType == cfg.pathType)
        {
            continue;
        }
        DelayMatrixRow row;
        row.pathType = pathType;
        row.status = "not_run";
        row.reason = "not_run_in_current_single_path_simulation";
        row.regenerationMode = cfg.regenerationMode;
        rows.push_back(row);
    }
    return rows;
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
    if (mode == "transparent")
    {
        return SatEnums::TRANSPARENT;
    }

    NS_ABORT_MSG("Unsupported regenerationMode=" << mode);
}

static void
ConfigureGwFeederSetsFromRtnConf(Ptr<IslRoutingManager>     routingMgr,
                                 const std::string&         rtnConfFilePath,
                                 const std::set<uint32_t>&  targetGwIds,
                                 uint32_t                   numSats)
{
    std::map<uint32_t, std::set<uint32_t>> feederSatsMap;
    std::ifstream rtnIn(rtnConfFilePath);
    if (rtnIn.is_open())
    {
        uint32_t beam = 0;
        uint32_t userChannel = 0;
        uint32_t gwIdFromFile = 0;
        uint32_t feederChannel = 0;
        while (rtnIn >> beam >> userChannel >> gwIdFromFile >> feederChannel)
        {
            if (gwIdFromFile == 0 || beam == 0)
            {
                continue;
            }

            const uint32_t gwIdx = gwIdFromFile - 1;
            if (!targetGwIds.empty() && targetGwIds.count(gwIdx) == 0)
            {
                continue;
            }

            const uint32_t satId = beam - 1;
            if (satId < numSats)
            {
                feederSatsMap[gwIdx].insert(satId);
            }
        }
    }
    else
    {
        std::cout << "[GW_FEEDER] WARNING: cannot open " << rtnConfFilePath
                  << "; feeder filter disabled for target GWs\n";
    }

    for (uint32_t gwIdx : targetGwIds)
    {
        auto it = feederSatsMap.find(gwIdx);
        routingMgr->SetGwFeederSats(
            gwIdx,
            (it != feederSatsMap.end()) ? it->second : std::set<uint32_t>{});
        std::cout << "[GW_FEEDER] gwIdx=" << gwIdx
                  << " feederSats="
                  << ((it != feederSatsMap.end()) ? it->second.size() : 0) << "\n";
    }
}

// === Routing / Case Execution ===============================================

static void
ConfigureRoutingCase(Ptr<IslRoutingManager> routingMgr,
                     const E2EConfig&       cfg,
                     double                 elevMinDeg)
{
    if (cfg.pathType == "sat2sat")
    {
        std::cout << "\n[CASE] sat2sat | src=" << cfg.satSrc
                  << " dst=" << cfg.satDst << "\n";
        routingMgr->PrintRouteReport({{cfg.satSrc, cfg.satDst}});
        return;
    }

    if (cfg.pathType == "gw2gw_e2e")
    {
        routingMgr->SetGwElevationThreshold(elevMinDeg);
        AddGatewayOrAbort(routingMgr, cfg.gwSrc);
        AddGatewayOrAbort(routingMgr, cfg.gwDst);
        routingMgr->AddGwPair(cfg.gwSrc, cfg.gwDst);
        ConfigureGwFeederSetsFromRtnConf(
            routingMgr,
            cfg.rtnConfFilePath,
            std::set<uint32_t>{cfg.gwSrc, cfg.gwDst},
            cfg.numSats);

        #if 0
        // Legacy inline feeder config kept only as reference during refactor.
        // Build feeder satellite map from rtnConf.txt (authoritative source).
        // IP-based scanning is unreliable: with includeAllFeederBeams=true, all GW feeder
        // interfaces appear on GW0's node and o2 values exceed numSats (feederSats=255).
        // rtnConf.txt directly encodes which satellite (beam-1) belongs to which GW (gwIdFromFile-1).
        // Iridium-66 convention: beamNumber = satId + 1 (1-indexed beams, 0-indexed sats).
        // Sats not in rtnConf for a given GW are excluded from its entry/exit candidates.
        {
            std::map<uint32_t, std::set<uint32_t>> feederSatsMap; // gwIdx → {satIds}
            std::ifstream rtnIn(cfg.rtnConfFilePath);
            if (rtnIn.is_open())
            {
                uint32_t beam, userChannel, gwIdFromFile, feederChannel;
                while (rtnIn >> beam >> userChannel >> gwIdFromFile >> feederChannel)
                {
                    if (gwIdFromFile == 0 || beam == 0) continue;
                    uint32_t satId = beam - 1;   // 0-indexed satellite ID
                    uint32_t gwIdx = gwIdFromFile - 1;  // 0-indexed GW index
                    if (satId < cfg.numSats)
                        feederSatsMap[gwIdx].insert(satId);
                }
            }
            else
            {
                std::cout << "[GW_FEEDER] WARNING: cannot open " << cfg.rtnConfFilePath
                          << "; feeder filter disabled for target GWs\n";
            }

            std::set<uint32_t> targetGwIds = {cfg.gwSrc, cfg.gwDst};
            for (uint32_t gwIdx : targetGwIds)
            {
                auto it = feederSatsMap.find(gwIdx);
                if (it != feederSatsMap.end())
                    routingMgr->SetGwFeederSats(gwIdx, it->second);
                else
                    routingMgr->SetGwFeederSats(gwIdx, {});
                uint32_t count = (it != feederSatsMap.end()) ? it->second.size() : 0;
                std::cout << "[GW_FEEDER] gwIdx=" << gwIdx
                          << " feederSats=" << count << "\n";
            }
        }

        #endif
        std::cout << "\n[CASE] gw2gw_e2e | gwSrc=" << cfg.gwSrc
                  << " gwDst=" << cfg.gwDst << "\n";
        routingMgr->PrecomputeGwRoutes();
        routingMgr->PrintGwRouteReport();

        // Register physical GW nodes so ApplyRoutingTable can install
        // Ipv4StaticRouting entries on intermediate satellite nodes.
        // This ensures packets forwarded from the entry satellite toward
        // the exit satellite use the SatOrbiterNetDevice (and arbiter),
        // not get dropped at the IP layer due to missing routes.
        Ptr<Node> dstGwNode = GetPhysicalGwNodeOrNull(cfg.gwDst);
        if (dstGwNode)
        {
            routingMgr->RegisterGwNode(cfg.gwDst, dstGwNode);
            std::cout << "[CASE] registered dstGwNode for gwId=" << cfg.gwDst
                      << " nodeId=" << dstGwNode->GetId() << "\n";
        }
        else
        {
            std::cout << "[CASE] WARNING: physical GW node for gwDst=" << cfg.gwDst
                      << " not found — satellite IP routes will not be installed\n";
        }
        return;
    }

    if (cfg.pathType == "gw2sat")
    {
        routingMgr->SetGwElevationThreshold(elevMinDeg);
        AddGatewayOrAbort(routingMgr, cfg.gwId);
        ConfigureGwFeederSetsFromRtnConf(
            routingMgr,
            cfg.rtnConfFilePath,
            std::set<uint32_t>{cfg.gwId},
            cfg.numSats);
        routingMgr->PrecomputeGwRoutes();

        std::cout << "\n[CASE] gw2sat | gwId=" << cfg.gwId
                  << " | feederlink_up only | isl_cost=N/A\n";
        return;
    }

    if (cfg.pathType == "sat2ut")
    {
        routingMgr->SetGwElevationThreshold(elevMinDeg);
        AddGatewayOrAbort(routingMgr, cfg.gwId);
        routingMgr->AddUserTerminal(cfg.utId, cfg.utLatDeg, cfg.utLonDeg, cfg.utName);
        routingMgr->AddGwUtPair(cfg.gwId, cfg.utId);
        ConfigureGwFeederSetsFromRtnConf(
            routingMgr,
            cfg.rtnConfFilePath,
            std::set<uint32_t>{cfg.gwId},
            cfg.numSats);
        routingMgr->PrecomputeGwUtRoutes();

        std::cout << "\n[CASE] sat2ut | utId=" << cfg.utId
                  << " | servicelink only | isl_cost=N/A\n";
        return;
    }

    if (cfg.pathType == "sat2gw")
    {
        routingMgr->SetGwElevationThreshold(elevMinDeg);
        AddGatewayOrAbort(routingMgr, cfg.gwId);
        ConfigureGwFeederSetsFromRtnConf(
            routingMgr,
            cfg.rtnConfFilePath,
            std::set<uint32_t>{cfg.gwId},
            cfg.numSats);
        routingMgr->PrecomputeGwRoutes();

        std::cout << "\n[CASE] sat2gw | gwId=" << cfg.gwId
                  << " | feederlink_dn only | isl_cost=N/A\n";
        return;
    }

    routingMgr->SetGwElevationThreshold(elevMinDeg);
    AddGatewayOrAbort(routingMgr, cfg.gwId);
    routingMgr->AddUserTerminal(cfg.utId, cfg.utLatDeg, cfg.utLonDeg, cfg.utName);
    routingMgr->AddGwUtPair(cfg.gwId, cfg.utId);
    ConfigureGwFeederSetsFromRtnConf(
        routingMgr,
        cfg.rtnConfFilePath,
        std::set<uint32_t>{cfg.gwId},
        cfg.numSats);

    std::cout << "\n[CASE] gw2ut_e2e | gwId=" << cfg.gwId
              << " utId=" << cfg.utId
              << " utLatDeg=" << cfg.utLatDeg
              << " utLonDeg=" << cfg.utLonDeg << "\n";
    routingMgr->PrecomputeGwUtRoutes();
    routingMgr->PrintGwUtRouteReport();
}

static void
PrintPacketPathTrace(Ptr<IslRoutingManager> routingMgr,
                     const E2EConfig&       cfg,
                     uint32_t               numSlots)
{
    std::cout << "\n=== Packet Path Trace (planned forwarding path) ===\n";
    std::cout << "pathType=" << cfg.pathType
              << " trafficUtUserId=" << cfg.trafficUtUserId << "\n";

    for (uint32_t slot = 0; slot < numSlots; ++slot)
    {
        double t = slot * routingMgr->GetTimeSlotInterval();
        std::cout << "[PKT_PATH] slot=" << slot
                  << " t=" << std::fixed << std::setprecision(1) << t << "s ";

        if (cfg.pathType == "gw2gw_e2e")
        {
            GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
            if (!r.valid)
            {
                std::cout << "GW" << cfg.gwSrc << "->GW" << cfg.gwDst
                          << " INVALID\n";
                continue;
            }
            std::cout << "GW" << cfg.gwSrc
                      << " -> feeder_up:" << RouteNodeToString(r.entrySatId)
                      << " -> isl_transit:" << SatPathToString(r.satPath)
                      << " -> feeder_down:GW" << cfg.gwDst
                      << " obs_isl_keys=";
            for (size_t i = 0; i + 1 < r.satPath.size(); ++i)
            {
                if (i > 0) std::cout << "|";
                std::cout << MakeIslKey(r.satPath[i], r.satPath[i + 1]);
            }
            std::cout << "\n";
        }
        else if (cfg.pathType == "gw2ut_e2e")
        {
            GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
            if (!r.valid)
            {
                std::cout << "GW" << cfg.gwId << "->UT" << cfg.utId
                          << " INVALID\n";
                continue;
            }
            std::cout << "GW" << cfg.gwId
                      << " -> feeder_up:" << RouteNodeToString(r.entrySatId)
                      << " -> isl_transit:" << SatPathToString(r.satPath)
                      << " -> service_down:" << RouteNodeToString(r.servingSatId)
                      << " -> UT" << cfg.utId
                      << " (trafficUtUserId=" << cfg.trafficUtUserId << ")\n";
        }
        else if (cfg.pathType == "sat2sat")
        {
            std::vector<uint32_t> path =
                routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot);
            std::cout << "SAT" << cfg.satSrc
                      << " -> isl_transit:" << SatPathToString(path)
                      << " -> SAT" << cfg.satDst << "\n";
        }
        else if (cfg.pathType == "sat2ut")
        {
            GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
            if (!r.valid)
            {
                std::cout << "SAT(route_serving)->UT" << cfg.utId
                          << " INVALID\n";
                continue;
            }
            std::cout << "SAT" << r.servingSatId
                      << " -> service_down:" << RouteNodeToString(r.servingSatId)
                      << " -> UT" << cfg.utId
                      << " (trafficUtUserId=" << cfg.trafficUtUserId << ")\n";
        }
        else if (cfg.pathType == "gw2sat")
        {
            const auto& visible = routingMgr->GetGwVisibleSats(cfg.gwId, slot);
            std::cout << "GW" << cfg.gwId
                      << " -> feeder_up:visible_sats=" << SetToString(visible) << "\n";
        }
        else if (cfg.pathType == "sat2gw")
        {
            const auto& visible = routingMgr->GetGwVisibleSats(cfg.gwId, slot);
            std::cout << "visible_sats=" << SetToString(visible)
                      << " -> feeder_down:GW" << cfg.gwId << "\n";
        }
    }
    std::cout << "=================================================\n\n";
}

static void
PrintPropagationDelayAudit(Ptr<IslRoutingManager> routingMgr,
                           const E2EConfig&       cfg,
                           uint32_t               numSlots)
{
    std::cout << "\n=== Propagation Delay Audit ===\n";
    std::cout << "[PROP] ISL routing cost uses dist/c. PointToPointIslChannel also "
              << "computes distance/PropagationSpeed at packet transmit time; "
              << "PropagationDelayTrace verifies the actual scheduled channel delay.\n";

    for (uint32_t slot = 0; slot < numSlots; ++slot)
    {
        DelayMatrixRow row;
        row.pathType = cfg.pathType;
        FillTheoreticalDelay(routingMgr, cfg, slot, row);

        std::vector<uint32_t> path;
        if (cfg.pathType == "gw2gw_e2e")
        {
            GwToGwRoute r = routingMgr->GetGwRoute(cfg.gwSrc, cfg.gwDst, slot);
            if (r.valid) path = r.satPath;
        }
        else if (cfg.pathType == "gw2ut_e2e")
        {
            GwToUtRoute r = routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, slot);
            if (r.valid) path = r.satPath;
        }
        else if (cfg.pathType == "sat2sat")
        {
            path = routingMgr->TracePath(cfg.satSrc, cfg.satDst, slot);
        }

        if (!path.empty())
        {
            ComputeIslPropagationDelayMs(routingMgr, path, slot, true);
        }

        std::cout << "[PROP][TOTAL] slot=" << slot
                  << " feeder_ms=" << std::fixed << std::setprecision(3)
                  << row.theoreticalFeederDelayMs
                  << " isl_ms=" << row.theoreticalIslDelayMs
                  << " service_ms=" << row.theoreticalServiceDelayMs
                  << " total_theory_ms=" << row.theoreticalTotalDelayMs
                  << "\n";
    }

    std::cout << "===============================\n\n";
}

static void
PrintRerouteSummary(Ptr<IslRoutingManager> routingMgr)
{
    const auto& stats = routingMgr->GetSlotStats();
    uint32_t rerouteCount = 0;

    std::cout << "\n=== ISL Rerouting Summary ===\n"
              << std::left
              << std::setw(6)  << "slot"
              << std::setw(9)  << "t(s)"
              << std::setw(12) << "recomputed"
              << std::setw(10) << "changed"
              << "apply_ms\n"
              << std::string(47, '-') << "\n";

    for (const auto& s : stats)
    {
        if (s.significantChange || s.recomputedSources > 0)
        {
            if (s.significantChange)
            {
                ++rerouteCount;
            }
            std::cout << std::left
                      << std::setw(6)  << s.slotIndex
                      << std::setw(9)  << std::fixed << std::setprecision(1)
                                       << s.simTimeSec
                      << std::setw(12) << s.recomputedSources
                      << std::setw(10) << (s.significantChange ? "YES" : "no")
                      << s.applyWallMs << "ms\n";
        }
    }

    std::cout << std::string(47, '-') << "\n"
              << "Total rerouting events: " << rerouteCount
              << " / " << stats.size() << " applied slots\n";
    if (rerouteCount == 0)
    {
        std::cout << "[RESULT] No load-triggered rerouting was observed. "
                  << "Queueing/delay may still occur without a path change.\n";
    }
    else
    {
        std::cout << "[RESULT] Load-triggered rerouting occurred; compare "
                  << "[PKT_PATH] and [OBS][SCOPE_UPDATE] lines by slot.\n";
    }
    std::cout << "============================\n\n";
}

// ── PredictGw2GwRequiredBeams ─────────────────────────────────────────────
//
// Resolves the chicken-and-egg dependency between beam selection and routing:
//   - Beam selection must happen before CreateSatScenario()
//   - Routing computation (which determines entry/exit sats) needs the scenario
//
// Solution: two-phase bootstrap.
//   Phase 1 (this function): create a minimal scenario (1 beam per GW, fast),
//     run routing precomputation, collect the unique entry/exit satellite IDs
//     actually chosen across all slots, then destroy the simulator.
//   Phase 2 (main): create the real scenario using only those beams + 1 per GW,
//     replacing the original includeAllFeederBeams=true (~26 beams → ~4–8 beams).
//
// Return value: set of SNS3 beam IDs (= satId + 1) needed for all entry/exit sats
//   plus 1 representative beam per GW (to register physical GW nodes in SatTopology).
//
// Requires Simulator::Destroy() at the end to reset NS-3 state for phase 2.
// Config::SetDefault values persist across Destroy() and do not need re-applying.
static std::set<uint32_t>
PredictGw2GwRequiredBeams(const std::string& scenarioName,
                           const std::string& ns3BasePath,
                           const std::string& rtnConfFilePath,
                           const std::string& islsFilePath,
                           uint32_t           gwSrc,
                           uint32_t           gwDst,
                           const std::set<uint32_t>& targetGwIds,
                           uint32_t           numSats,
                           uint32_t           numSlots,
                           double             slotInterval,
                           double             islMaxDistKm,
                           double             islRateBps,
                           double             emaAlpha,
                           double             changeThresh,
                           double             cooldownSec,
                           double             elevMinDeg,
                           uint32_t           beamId,
                           uint32_t           requiredGwNodeCount,
                           const std::string& regenerationMode)
{
    std::cout << "\n[PREDICT] Phase 1: minimal-beam pre-routing to determine "
              << "required feeder beams for gw2gw_e2e\n";

    // Apply Config defaults required for SNS3 scenario creation.
    // These persist across Simulator::Destroy() so phase 2 does not need to re-apply them.
    SatEnums::RegenerationMode_t parsedRegen = ParseRegenerationMode(regenerationMode);
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(parsedRegen));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(parsedRegen));
    Config::SetDefault("ns3::PointToPointIslHelper::IslDataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(islRateBps))));
    Config::SetDefault("ns3::PointToPointIslChannel::PropagationSpeed",
                       DoubleValue(3.0e8));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionEachRequest",
                       BooleanValue(false));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionPeriod",
                       TimeValue(Seconds(slotInterval)));
    Config::SetDefault("ns3::SatHelper::GwUsers",
                       UintegerValue(requiredGwNodeCount));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatPhy::EnableStatisticsTags", BooleanValue(true));
    Config::SetDefault("ns3::SatNetDevice::EnableStatisticsTags", BooleanValue(true));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    // Minimal beam set: 1 representative beam per GW (not all feeder beams).
    // Fast to create — MAC/DAMA events scale with beam count.
    const std::set<uint32_t> minimalBeams =
        BuildGatewayBootstrapBeamSet(rtnConfFilePath, targetGwIds, beamId,
                                     false /* includeAllFeederBeams */);
    std::cout << "[PREDICT] minimalBeams={" << FormatBeamSet(minimalBeams) << "}\n";

    Ptr<SimulationHelper> preHelper =
        CreateObject<SimulationHelper>("test-iridium-3segment-e2e-predict");
    preHelper->LoadScenario(scenarioName);
    preHelper->SetSimulationTime(Seconds(1.0));  // just long enough to init nodes
    preHelper->SetBeamSet(minimalBeams);
    preHelper->SetUserCountPerUt(1);
    preHelper->CreateSatScenario();

    // Build a temporary routing manager using the scenario's satellite positions.
    Ptr<IslRoutingManager> preRouting = CreateObject<IslRoutingManager>();
    preRouting->SetAttribute("NumSatellites",    UintegerValue(numSats));
    preRouting->SetAttribute("NumTimeSlots",     UintegerValue(numSlots));
    preRouting->SetAttribute("TimeSlotInterval", DoubleValue(slotInterval));
    preRouting->SetAttribute("IslMaxDistanceKm", DoubleValue(islMaxDistKm));
    preRouting->SetAttribute("IslsFilePath",     StringValue(islsFilePath));
    preRouting->SetAttribute("EmaAlpha",         DoubleValue(emaAlpha));
    preRouting->SetAttribute("ChangeThreshold",  DoubleValue(changeThresh));
    preRouting->SetAttribute("CooldownSeconds",  DoubleValue(cooldownSec));
    preRouting->SetAttribute("IslLinkRateBps",   DoubleValue(islRateBps));

    preRouting->Initialize(islsFilePath);
    preRouting->PrecomputeAllTables();

    // Configure GW routing: feeder sat filter from rtnConf + elevation threshold.
    preRouting->SetGwElevationThreshold(elevMinDeg);
    AddGatewayOrAbort(preRouting, gwSrc);
    AddGatewayOrAbort(preRouting, gwDst);
    preRouting->AddGwPair(gwSrc, gwDst);

    // Read rtnConf feeder-sat mapping and configure Phase 1 routing.
    // feederSatsMap is kept in scope so Phase 2 beam selection can reuse it.
    std::map<uint32_t, std::set<uint32_t>> feederSatsMap;   // gwIdx (0-indexed) → {satIds}
    {
        std::ifstream rtnIn(rtnConfFilePath);
        if (rtnIn.is_open())
        {
            uint32_t beam, userChannel, gwIdFromFile, feederChannel;
            while (rtnIn >> beam >> userChannel >> gwIdFromFile >> feederChannel)
            {
                if (gwIdFromFile == 0 || beam == 0) continue;
                uint32_t satId = beam - 1;
                uint32_t gwIdx = gwIdFromFile - 1;
                if (satId < numSats)
                    feederSatsMap[gwIdx].insert(satId);
            }
        }
        for (uint32_t gwIdx : targetGwIds)
        {
            auto it = feederSatsMap.find(gwIdx);
            preRouting->SetGwFeederSats(gwIdx,
                (it != feederSatsMap.end()) ? it->second : std::set<uint32_t>{});
        }
    }

    preRouting->PrecomputeGwRoutes();

    // Collect actual entry/exit satellites chosen across all slots.
    std::set<uint32_t> requiredSats = preRouting->GetRequiredGwFeederSats(gwSrc, gwDst);
    std::cout << "[PREDICT] requiredSats (" << requiredSats.size() << "):";
    for (uint32_t s : requiredSats)
        std::cout << " sat" << s << "(beam" << (s + 1) << ")";
    std::cout << "\n";

    // Build Phase 2 beam set from entry/exit sats only — do NOT include minimalBeams
    // wholesale. minimalBeams may contain beams from GWs other than gwSrc and gwDst
    // (e.g. the primary beamId=72 could belong to a 3rd GW). Including those beams
    // creates extra physical GW nodes in Phase 2, which shifts gws.Get(gwDst) to the
    // wrong SNS3 node and causes dstAddr to resolve to the wrong IP.
    //
    // Build a reverse mapping: SNS3 beam (1-indexed) → gwIdx (0-indexed).
    std::map<uint32_t, uint32_t> beamToGwIdx;
    for (const auto& kv : feederSatsMap)
        for (uint32_t satId : kv.second)
            beamToGwIdx[satId + 1] = kv.first;

    std::set<uint32_t> requiredBeams;
    std::set<uint32_t> coveredGwIdxs;

    // Add beams for entry/exit sats — these are guaranteed to belong to gwSrc or gwDst
    // because feederFilter restricts candidates to each GW's rtnConf feeder set.
    for (uint32_t satId : requiredSats)
    {
        uint32_t beam = satId + 1;
        requiredBeams.insert(beam);
        auto it = beamToGwIdx.find(beam);
        if (it != beamToGwIdx.end())
            coveredGwIdxs.insert(it->second);
    }

    // Ensure at least one beam from gwSrc and one from gwDst is included.
    // If all requiredSats happen to belong to the same GW, add one bootstrap beam
    // for the other GW so Phase 2 creates exactly 2 physical GW nodes.
    const uint32_t targetGwIdxs[] = {gwSrc, gwDst};
    for (uint32_t targetGwIdx : targetGwIdxs)
    {
        if (coveredGwIdxs.count(targetGwIdx))
            continue;  // already covered by an entry/exit sat beam
        // Find the first beam in minimalBeams that belongs to targetGwIdx.
        for (uint32_t b : minimalBeams)
        {
            auto it = beamToGwIdx.find(b);
            if (it != beamToGwIdx.end() && it->second == targetGwIdx)
            {
                requiredBeams.insert(b);
                coveredGwIdxs.insert(targetGwIdx);
                std::cout << "[PREDICT] added bootstrap beam " << b
                          << " for gwIdx=" << targetGwIdx << " (no entry/exit sat covered it)\n";
                break;
            }
        }
    }

    // Diagnostic: log any beams from minimalBeams that were excluded (belong to a 3rd GW).
    for (uint32_t b : minimalBeams)
    {
        if (!requiredBeams.count(b))
        {
            auto it = beamToGwIdx.find(b);
            uint32_t ownerGw = (it != beamToGwIdx.end()) ? it->second : UINT32_MAX;
            std::cout << "[PREDICT] excluded beam " << b << " from Phase 2"
                      << " (belongs to gwIdx=" << ownerGw
                      << " which is not gwSrc=" << gwSrc << " or gwDst=" << gwDst << ")\n";
        }
    }

    std::cout << "[PREDICT] requiredBeams={" << FormatBeamSet(requiredBeams) << "} "
              << "(was ~" << (targetGwIds.size() * 13) << " with includeAllFeederBeams;"
              << " physicalGwNodes in Phase 2 = " << coveredGwIdxs.size() << ")\n";
    std::cout << "[PREDICT] Phase 1 complete. Destroying simulator for phase 2.\n\n";

    // Reset simulator state so phase 2 can create a fresh scenario.
    // SatTopology singleton is re-populated by CreateSatScenario() in phase 2.
    Simulator::Destroy();

    return requiredBeams;
}

} // namespace

int
main(int argc, char* argv[])
{
    const std::string ns3BasePath = "/home/wenj/workspace/ns-3.43";
    std::string       scenarioName = "constellation-iridium-66-sats-fixed";
    uint32_t          numSats = 66;

    std::string islsFilePath;
    std::string rtnConfFilePath;

    std::string pathType = "gw2gw_e2e";
    bool        rbdcVerbose = false;
    bool        obsDebug = false;
    bool        satStats = false;   // enable SNS3 native SatStatsHelperContainer output
    bool        endpointProbe = false;
    bool        packetHopTrace = false;
    uint32_t    packetHopTraceMaxPackets = 8;
    uint32_t    endpointProbePort = 9100;
    double      islDropThreshPct = 1.0;

    double      simTime = 630.0;
    double      slotInterval = 60.0;
    uint32_t    beamId = 72;

    double      islMaxDistKm = 5000.0;
    double      islRateMbps = 10.0;
    double      emaAlpha = 0.3;
    double      changeThresh = 0.1;
    double      cooldownRatio = 0.5;
    double      elevMinDeg = 5.0;

    TrafficConfig trafficCfg;

    uint32_t satSrc = 0;
    uint32_t satDst = 10;

    uint32_t gwSrc = 0;
    uint32_t gwDst = 1;
    std::string regenerationMode = "network";
    bool        delayMatrix = false;
    std::string delayCsvPath = "real_delay_matrix.csv";
    bool        strict3gppScenarioD = false;

    uint32_t    gwId = 0;
    uint32_t    utId = 0;
    double      utLatDeg = 25.0330;
    double      utLonDeg = 121.5654;
    std::string utName = "UT-Taipei";

    // === Link Observability CLI parameters ===
    std::string obsLogPath      = "e2e_link_obs.csv";  // CSV log output path
    double      obsIntervalSec  = 10.0;                // snapshot interval (s)
    double      obsDropAlertPct = 50.0;                // drop rate alert threshold (%)

    CommandLine cmd;
    cmd.AddValue("pathType",
                 "Path type: gw2sat | sat2sat | sat2ut | sat2gw | gw2ut_e2e | gw2gw_e2e "
                 "(aliases: gw2ut, gw2gw)",
                 pathType);
    cmd.AddValue("rbdcVerbose", "Print each RBDC request (1=on, 0=off, default=0)", rbdcVerbose);
    cmd.AddValue("obsDebug",
                 "Enable verbose OBS debug output (GW device list and rx_hits) (0/1)",
                 obsDebug);
    cmd.AddValue("satStats",
                 "Enable SNS3 native SatStatsHelperContainer output (scatter files per GW/SAT/UT) "
                 "(0/1, default=0). Useful for diagnostic cross-checks outside the main verdict.",
                 satStats);
    cmd.AddValue("endpointProbe",
                 "Enable endpoint delivery probe for all pathType values (0/1, default=0)",
                 endpointProbe);
    cmd.AddValue("packetHopTrace",
                 "Enable per-packet hop-by-hop trace for tagged app traffic (0/1, default=0)",
                 packetHopTrace);
    cmd.AddValue("packetHopTraceMaxPackets",
                 "Maximum number of packets to trace hop-by-hop when packetHopTrace=1",
                 packetHopTraceMaxPackets);
    cmd.AddValue("endpointProbePort",
                 "UDP port for diagnostic endpoint PacketSink probes (default=9100)",
                 endpointProbePort);
    cmd.AddValue("islDropThreshPct", "ISL overall drop rate PASS threshold (%, default=1.0)", islDropThreshPct);

    cmd.AddValue("simTime", "Simulation duration (s)", simTime);
    cmd.AddValue("slotInterval", "Routing slot interval (s)", slotInterval);
    cmd.AddValue("beamId",
                 "Primary SNS3 beam ID to activate; GW bootstrap beams are added automatically",
                 beamId);
    cmd.AddValue("islMaxDistKm", "ISL activation distance threshold (km)", islMaxDistKm);
    cmd.AddValue("islRateMbps", "ISL link rate (Mbps)", islRateMbps);
    cmd.AddValue("emaAlpha", "EMA weight for load-cost smoothing", emaAlpha);
    cmd.AddValue("changeThresh", "Load-cost change ratio for recompute", changeThresh);
    cmd.AddValue("cooldownRatio", "Cooldown = slotInterval * cooldownRatio", cooldownRatio);
    cmd.AddValue("elevMinDeg", "Minimum GW/UT elevation angle (deg)", elevMinDeg);

    cmd.AddValue("satSrc", "Source satellite ID for sat2sat", satSrc);
    cmd.AddValue("satDst", "Destination satellite ID for sat2sat", satDst);

    cmd.AddValue("gwSrc", "Source gateway preset ID for gw2gw_e2e", gwSrc);
    cmd.AddValue("gwDst", "Destination gateway preset ID for gw2gw_e2e", gwDst);
    cmd.AddValue("delayMatrix",
                 "Write real delay matrix CSV summary (0/1). Current path is measured; other paths are listed as not_run.",
                 delayMatrix);
    cmd.AddValue("delayCsvPath",
                 "Output CSV path for delay matrix/summary",
                 delayCsvPath);
    cmd.AddValue("regenerationMode",
                 "Satellite regeneration mode: network | phy | transparent",
                 regenerationMode);
    cmd.AddValue("scenarioName",
                 "SNS3 scenario folder name under contrib/satellite/data/scenarios",
                 scenarioName);
    cmd.AddValue("numSats",
                 "Number of satellites in the selected scenario",
                 numSats);
    cmd.AddValue("strict3gppScenarioD",
                 "Force 3GPP Scenario D semantics: sat2ut + regenerative payload + service-link-only traffic (0/1)",
                 strict3gppScenarioD);

    cmd.AddValue("gwId", "Gateway preset ID for gw2sat/sat2gw/gw2ut_e2e", gwId);
    cmd.AddValue("utId", "User terminal ID for sat2ut/gw2ut_e2e", utId);
    cmd.AddValue("utLatDeg", "UT latitude (deg) for sat2ut/gw2ut_e2e", utLatDeg);
    cmd.AddValue("utLonDeg", "UT longitude (deg) for sat2ut/gw2ut_e2e", utLonDeg);
    cmd.AddValue("utName", "UT name for sat2ut/gw2ut_e2e", utName);

    cmd.AddValue("fwd", "Enable FWD link (GW->UT) CBR traffic (0/1)", trafficCfg.enableFwd);
    cmd.AddValue("rtn", "Enable RTN link (UT->GW) CBR traffic (0/1)", trafficCfg.enableRtn);
    cmd.AddValue("fwdIntervalMs", "FWD CBR packet interval (ms)", trafficCfg.fwdIntervalMs);
    cmd.AddValue("rtnIntervalMs", "RTN CBR packet interval (ms)", trafficCfg.rtnIntervalMs);
    cmd.AddValue("fwdPktBytes", "FWD CBR packet size (bytes)", trafficCfg.fwdPktBytes);
    cmd.AddValue("rtnPktBytes", "RTN CBR packet size (bytes)", trafficCfg.rtnPktBytes);
    cmd.AddValue("trafficStart", "Traffic start time (s)", trafficCfg.startSec);
    cmd.AddValue("trafficStop", "Traffic stop time (s), 0 = simTime-1", trafficCfg.stopSec);

    cmd.AddValue("obsLogPath",
                 "Link observability log file path (CSV, default=e2e_link_obs.csv)",
                 obsLogPath);
    cmd.AddValue("obsInterval",
                 "Link observability snapshot interval (s, default=10.0)",
                 obsIntervalSec);
    cmd.AddValue("obsDropAlertPct",
                 "Drop-rate threshold for stdout event alert (%, default=50.0)",
                 obsDropAlertPct);

    cmd.Parse(argc, argv);

    // Apply observer config after cmd.Parse so overridden trafficCfg.startSec is used.
    g_obsCfg.snapshotIntervalSec = obsIntervalSec;
    g_obsCfg.dropAlertThreshPct  = obsDropAlertPct;
    g_obsCfg.trafficStartSec     = trafficCfg.startSec;
    g_obsCfg.logFilePath         = obsLogPath;
    g_obsDebug                   = obsDebug;
    g_packetHopTraceCfg.enabled  = packetHopTrace;
    g_packetHopTraceCfg.maxPackets = packetHopTraceMaxPackets;
    g_nextPacketHopTraceId       = 1;
    g_packetHopTraceActiveIds.clear();
    g_packetHopTraceFlowLabels.clear();

    if (satStats)
    {
        std::cout << "[SATSTATS] enabled — scatter files will be written after simulation\n";
    }

    if (packetHopTrace)
    {
        std::cout << "[PKT_HOP] enabled: tracing first "
                  << packetHopTraceMaxPackets
                  << " tagged packets hop-by-hop\n";
    }

    NS_ABORT_MSG_IF(slotInterval <= 0.0, "slotInterval must be > 0");
    NS_ABORT_MSG_IF(simTime <= 0.0, "simTime must be > 0");
    NS_ABORT_MSG_IF(endpointProbePort == 0 || endpointProbePort > 65535,
                    "endpointProbePort must be in [1, 65535]");
    NS_ABORT_MSG_IF(satSrc >= numSats || satDst >= numSats,
                    "satSrc/satDst must be < " << numSats);
    {
        double resolvedStop =
            (trafficCfg.stopSec > 0.0) ? trafficCfg.stopSec : (simTime - 1.0);
        NS_ABORT_MSG_IF(trafficCfg.startSec >= resolvedStop,
                        "trafficStart (" << trafficCfg.startSec
                        << "s) must be < trafficStop (" << resolvedStop << "s)");
    }

    pathType = NormalizePathType(pathType);

    if (strict3gppScenarioD)
    {
        pathType = "sat2ut";
        regenerationMode = "network";
        trafficCfg.enableFwd = true;
        trafficCfg.enableRtn = false;
    }

    islsFilePath = ns3BasePath + "/contrib/satellite/data/scenarios/" +
                   scenarioName + "/positions/isls.txt";
    rtnConfFilePath = ns3BasePath + "/contrib/satellite/data/scenarios/" +
                      scenarioName + "/beams/rtnConf.txt";

    E2EConfig e2eCfg;
    e2eCfg.pathType = pathType;
    e2eCfg.simTimeSec = simTime;
    e2eCfg.satSrc = satSrc;
    e2eCfg.satDst = satDst;
    e2eCfg.gwSrc = gwSrc;
    e2eCfg.gwDst = gwDst;
    e2eCfg.gwId = gwId;
    e2eCfg.numSats = numSats;
    e2eCfg.regenerationMode = regenerationMode;
    e2eCfg.utId = utId;
    e2eCfg.trafficUtUserId = utId;
    e2eCfg.utLatDeg = utLatDeg;
    e2eCfg.utLonDeg = utLonDeg;
    e2eCfg.utName = utName;
    e2eCfg.rtnConfFilePath = rtnConfFilePath;

    e2eCfg.feederlink.traffic = trafficCfg;
    e2eCfg.isl.traffic = trafficCfg;
    e2eCfg.servicelink.traffic = trafficCfg;

    PathTypePlan e2ePlan = BuildPathTypePlan(e2eCfg);

    const double   islRateBps = islRateMbps * 1.0e6;
    const double   cooldownSec = slotInterval * cooldownRatio;
    const uint32_t numSlots =
        static_cast<uint32_t>(std::floor(simTime / slotInterval)) + 1;
    const std::set<uint32_t> targetGwIds =
        DetermineTargetGwIds(pathType, gwId, gwSrc, gwDst);
    const uint32_t requiredGwNodeCount =
        DetermineRequiredGwNodeCount(targetGwIds);
    // Only activate beams for the GWs actually used in this path (1 for single-GW paths,
    // 2 for gw2gw). Activating all 5 GW beams multiplies satellite MAC/DAMA/RBDC events
    // by ~5x and inflates wall time from ~12 min to ~3 hours.
    // For gw2gw_e2e, use PredictGw2GwRequiredBeams() (phase 1) to determine exactly which
    // feeder beams are needed before creating the main scenario.
    // Previously used includeAllFeederBeams=true (~26 beams), causing massive MAC/DAMA event
    // burst at t=0. Phase 1 runs a fast minimal-beam pre-routing, collects the 4–8 unique
    // entry/exit satellites actually chosen across all slots, then Simulator::Destroy().
    // Phase 2 (below) creates the real scenario with only those beams, cutting t=0 event
    // count from ~26×N to ~6×N and reducing wall time proportionally.
    std::set<uint32_t> enabledBeamSet;
    if (pathType == "gw2gw_e2e")
    {
        // Use fixed beam set {1, 2, 72} to create two separate physical GW nodes:
        // beam 1 and 2 belong to Tokyo's GW physical location (gwSrc=0),
        // beam 72 belongs to New Delhi's GW physical location (gwDst=1).
        // The previous approach (PredictGw2GwRequiredBeams) returned {4,5,6} which
        // all mapped to the same physical GW node (nodeId=66), causing both gwSrc
        // and gwDst to resolve to the same ns-3 node and all packets to be
        // delivered locally without traversing the satellite network.
        enabledBeamSet = {1, 2, 72};
        std::cout << "[PREDICT] beamOverride={1,2,72} (fixed: two separate GW nodes for gwSrc/gwDst)\n";
    }
    else
    {
        enabledBeamSet = BuildGatewayBootstrapBeamSet(
            rtnConfFilePath, targetGwIds, beamId,
            false /* includeAllFeederBeams */);
    }

    std::cout << "[CFG] pathType=" << pathType
              << " obsFeederMode=" << ToString(e2ePlan.spec.obsFeederMode)
              << " regenerationMode=" << regenerationMode
              << " strict3gppScenarioD=" << (strict3gppScenarioD ? 1 : 0)
              << " scenarioName=" << scenarioName
              << " simTime=" << simTime
              << " slotInterval=" << slotInterval
              << " numSlots=" << numSlots
              << " lastSlotTime=" << ((numSlots - 1) * slotInterval)
              << "\n";
    std::cout << "[TOPO_BOOTSTRAP] enabledBeams={" << FormatBeamSet(enabledBeamSet)
              << "} gatewayPresets=" << GetGatewayPresets().size()
              << " primaryBeamId=" << beamId << "\n";
    g_traceSlotIntervalSec = slotInterval;

    SatEnums::RegenerationMode_t parsedRegen = ParseRegenerationMode(regenerationMode);
    if (regenerationMode == "transparent" &&
        (pathType == "gw2gw_e2e" || pathType == "gw2ut_e2e" || pathType == "sat2sat"))
    {
        std::cout << "[REGEN] WARNING: transparent mode may not support constellation ISL/network "
                  << "routing for pathType=" << pathType
                  << "; unsupported cases will fail explicitly instead of being treated as validated.\n";
    }
    if (strict3gppScenarioD && scenarioName == "constellation-iridium-66-sats-fixed")
    {
        std::cout << "[3GPP][ScenarioD] WARNING: traffic semantics are now service-link-only, "
                  << "but scenario geometry is still Iridium-66 (~780 km), not a 600 km 3GPP reference orbit.\n";
    }
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(parsedRegen));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(parsedRegen));
    Config::SetDefault("ns3::PointToPointIslHelper::IslDataRate",
                       DataRateValue(DataRate(static_cast<uint64_t>(islRateBps))));
    Config::SetDefault("ns3::PointToPointIslChannel::PropagationSpeed",
                       DoubleValue(3.0e8));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionEachRequest",
                       BooleanValue(false));
    Config::SetDefault("ns3::SatSGP4MobilityModel::UpdatePositionPeriod",
                       TimeValue(Seconds(slotInterval)));
    Config::SetDefault("ns3::SatHelper::GwUsers",
                       UintegerValue(requiredGwNodeCount));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatPhy::EnableStatisticsTags", BooleanValue(true));
    Config::SetDefault("ns3::SatNetDevice::EnableStatisticsTags", BooleanValue(true));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    ConfigureQoS();

    Ptr<SimulationHelper> simHelper =
        CreateObject<SimulationHelper>("test-iridium-3segment-e2e");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));
    simHelper->SetBeamSet(enabledBeamSet);
    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();
    RefreshLogicalGwNodeMap(rtnConfFilePath, targetGwIds);

    {
        auto topo = Singleton<SatTopology>::Get();
        NodeContainer physicalGwNodes = topo->GetGwNodes();
        NodeContainer utUserNodes = topo->GetUtUserNodes();
        std::set<uint32_t> logicalGwIds;
        const PathTypeSpec spec = GetPathTypeSpec(e2eCfg.pathType);
        if (spec.needsGwPair)
        {
            logicalGwIds.insert(e2eCfg.gwSrc);
            logicalGwIds.insert(e2eCfg.gwDst);
        }
        else if (spec.needsGwId)
        {
            logicalGwIds.insert(e2eCfg.gwId);
        }

        std::cout << "[TOPO] physicalGwNodes=" << physicalGwNodes.GetN()
                  << " utUserNodes=" << utUserNodes.GetN() << "\n";
        for (uint32_t gwIdForLog : logicalGwIds)
        {
            std::cout << "[TOPO] logicalGwId=" << gwIdForLog
                      << " physicalGwNode="
                      << (gwIdForLog < physicalGwNodes.GetN() ? "present" : "not_present")
                      << "\n";
        }
        if (e2eCfg.pathType == "sat2gw" && e2eCfg.gwId >= physicalGwNodes.GetN())
        {
            std::cout << "[OBS][GW] WARNING: sat2gw GW-side PHY observer requires a physical GW node; "
                      << "gwId=" << e2eCfg.gwId
                      << " physicalGwNodes=" << physicalGwNodes.GetN()
                      << "\n";
        }
    }

    // === SNS3 Native Statistics (optional, --satStats=1) ===
    //
    // Hooks into SNS3's internal Dev/MAC/PHY-facing statistics path.
    // Use this as a native cross-check for custom OBS, especially service
    // FWD user-link traffic (SAT -> UT) and feeder_dn visibility limits.
    //
    // Output files: <pathType>-stats-fwd-feeder-dev-throughput-per-gw.dat, etc.
    // Use --satStats=1 on gw2ut_e2e/sat2ut to cross-check FWD user-link
    // per-sat/per-UT throughput against SERVICE_LAYER detail.
    if (satStats)
    {
        Ptr<SatStatsHelperContainer> nativeStats = simHelper->GetStatisticsContainer();
        // Use pathType as prefix so output files are distinguishable across runs.
        nativeStats->SetName(pathType + "-stats");

        // --- Feeder link (GW ↔ SAT) ---
        // FWD feeder: GW → SAT (feeder_up direction)
        nativeStats->AddPerSatFwdFeederDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerGwFwdFeederDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);

        // RTN feeder: SAT → GW (feeder_dn direction).
        // Key target: verify this yields non-zero counts for gw2gw_e2e.
        nativeStats->AddPerSatRtnFeederDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerGwRtnFeederDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);

        // --- User link (SAT ↔ UT) ---
        // FWD user: SAT → UT (service link downlink)
        nativeStats->AddPerUtFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerGwFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerSatFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerSatRtnAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerGwFwdUserMacThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerUtFwdUserPhyThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerUtFwdUserMacThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerSatFwdUserDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerUtFwdUserDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);

        // RTN user: UT → SAT (service link uplink)
        nativeStats->AddPerUtRtnAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerUtRtnUserPhyThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerUtRtnUserMacThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerSatRtnUserDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
        nativeStats->AddPerUtRtnUserDevThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);

        std::cout << "[SATSTATS] registered:"
                  << " per-sat/gw fwd+rtn feeder dev throughput"
                  << " | endpoint app throughput"
                  << " | per-gw fwd user mac throughput"
                  << " | per-ut fwd+rtn user phy/mac/dev throughput"
                  << "\n"
                  << "[SATSTATS] output prefix: " << pathType << "-stats-*\n";
    }

    uint32_t islConnected = ConnectIslDropTrace();

    // === Initialize Link Observer ===
    // Open CSV log file and write header row.
    g_obsLog.open(g_obsCfg.logFilePath);
    if (g_obsLog.is_open())
    {
        g_obsLog << "time_s,link_type,link_id,rx_pkts,rx_bytes,"
                    "tx_pkts,drop_pkts,drop_rate_pct,throughput_kbps,avg_delay_ms\n";
        std::cout << "[OBS] log opened: " << g_obsCfg.logFilePath << "\n";
    }
    else
    {
        std::cout << "[OBS] WARNING: cannot open log: " << g_obsCfg.logFilePath << "\n";
    }

    // Connect feeder / service / ISL traces (must run after ConnectIslDropTrace).
    const PathTypeSpec traceSpec = GetPathTypeSpec(e2eCfg.pathType);
    //
    // useOrbiterFeeder: enables SatOrbiterNetDevice::RxFeeder on all satellites.
    //   Captures feeder_up (GW→SAT uplink) for: gw2sat, sat2ut, gw2ut_e2e, gw2gw_e2e.
    //   gw2gw_e2e has obsFeederMode=PHY; its FEEDER_LAYER verdict uses OrbiterRxFeederCb
    //   counts on the entry satellite. useOrbiterFeeder=true for gw2gw_e2e.
    //
    // useGwFeeder: enables SatNetDevice::Rx on GW nodes.
    //   Captures feeder_dn (SAT→GW downlink) for sat2gw.
    //   gw2gw_e2e scopes MakeGwKey(dstGW) in feederKeys but does NOT connect
    //   GatewayRxFeederCb for it here; only the entry-satellite OrbiterRxFeeder
    //   contributes to the gw2gw_e2e FEEDER_LAYER verdict.
    bool useOrbiterFeeder = (traceSpec.obsFeederMode == ObsFeederMode::PHY &&
                             e2eCfg.pathType != "sat2gw");
    bool useGwFeeder      = (traceSpec.obsFeederMode == ObsFeederMode::PHY &&
                             e2eCfg.pathType == "sat2gw");
    // sat2ut and gw2ut_e2e: service link is FWD (SAT→UT).
    // OrbiterRxUserCb fires on RTN only; UT Rx must be observed instead.
    bool useUtService     = (e2eCfg.pathType == "sat2ut" ||
                             e2eCfg.pathType == "gw2ut_e2e");
    ConnectLinkObserverTraces(useOrbiterFeeder, useGwFeeder, useUtService);

    // Schedule the first periodic snapshot event.
    Simulator::Schedule(Seconds(g_obsCfg.snapshotIntervalSec), &TakeObsSnapshot);
    std::cout << "[OBS] snapshot interval=" << g_obsCfg.snapshotIntervalSec
              << "s  dropAlertThresh=" << g_obsCfg.dropAlertThreshPct << "%\n";

    if (rbdcVerbose)
    {
        Config::ConnectWithoutContext(
            "/NodeList/*/DeviceList/*/SatLlc/SatRequestManager/RbdcTrace",
            MakeCallback(&RbdcTraceCallback));
        std::cout << "[RBDC] trace connected: SatLlc/SatRequestManager/RbdcTrace\n";
    }
    else
    {
        std::cout << "[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)\n";
    }

    auto wallStart = std::chrono::steady_clock::now();

    // Initialize routing manager first so ConfigureRoutingCase and ConfigureObsScope
    // can consume precomputed route tables before traffic applications are installed.
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
    e2eCfg.trafficUtUserId = ResolveTrafficUtUserId(routingMgr, e2eCfg, numSlots);
    e2eCfg.trafficUtUserIdResolved = (e2eCfg.trafficUtUserId != e2eCfg.utId);
    ConfigureObsScope(routingMgr, e2eCfg, numSlots);
    PrintPacketPathTrace(routingMgr, e2eCfg, numSlots);
    PrintPropagationDelayAudit(routingMgr, e2eCfg, numSlots);

    // Install traffic applications after routing is fully configured so the
    // execution order reflects the logical dependency: route first, then traffic.
    PrintE2ERunBanner(e2eCfg, e2ePlan);
    InstallE2ETraffic(simHelper, routingMgr, e2eCfg, e2ePlan);
    InstallEndpointProbe(simHelper,
                         e2eCfg,
                         endpointProbe,
                         static_cast<uint16_t>(endpointProbePort));

    routingMgr->ScheduleRoutingUpdates();

    // Schedule dynamic scope updates for slot 1..numSlots-1 at the same time as
    // ApplyRoutingTable. Since these are enqueued after ScheduleRoutingUpdates,
    // NS-3's FIFO scheduler guarantees they fire after ApplyRoutingTable at each
    // slot boundary, so g_obsScope always reflects the freshly applied route.
    // Slot 0 is already covered by ConfigureObsScope() above.
    for (uint32_t k = 1; k < numSlots; ++k)
    {
        double slotTime = slotInterval * k;
        if (slotTime >= simTime)
        {
            continue;
        }
        Simulator::Schedule(Seconds(slotTime),
                            &UpdateObsScopeForSlot,
                            routingMgr,
                            e2eCfg,
                            k);
    }

    simHelper->RunSimulation();

    PrintObsFinalSummary();
    PrintE2EFinalVerdict(routingMgr, e2eCfg, numSlots);
    PrintEndpointProbeSummary();
    WriteDelayMatrix(delayCsvPath,
                     BuildDelayMatrixRows(simHelper, routingMgr, e2eCfg, delayMatrix));
    routingMgr->PrintStats();
    routingMgr->PrintLoadStats();
    PrintRerouteSummary(routingMgr);
    PrintIslDropStats(islDropThreshPct, islConnected);

    auto wallEnd = std::chrono::steady_clock::now();
    auto wallMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(wallEnd - wallStart).count();

    std::cout << "Total wall time: " << wallMs / 1000.0 << " s\n";
    std::cout << "Event count:     " << Simulator::GetEventCount() << "\n";

    Simulator::Destroy();
    return 0;
}
