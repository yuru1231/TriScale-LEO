#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"

#include <chrono>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
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
                      << std::setw(14)
                      << (std::to_string(static_cast<int>(r.dropRate * 1000) / 1000.0).substr(0, 6))
                      << std::fixed << std::setprecision(3) << r.successRate << "\n";
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
// 觀測三段鏈路的封包收發、drop rate、throughput、延遲。
// 事件（drop rate 超閾值、throughput 歸零）立即輸出 stdout；
// 定期快照（每 obsInterval 秒）寫入 CSV log 檔案。
//
// 依賴關係：ConnectLinkObserverTraces() 必須在 ConnectIslDropTrace() 之後呼叫，
// 因為 ISL callback 使用 g_nodeToSatId（由 ConnectIslDropTrace 填入）。

struct SegLinkStats
{
    // 累積計數器
    uint64_t rxPkts{0};     // 成功收到的封包數
    uint64_t rxBytes{0};    // 成功收到的 bytes（用於 throughput）
    uint64_t txPkts{0};     // 總嘗試次數（rx + drop）
    uint64_t dropPkts{0};   // 被 drop 的封包數

    // 延遲累加器（用於平均計算）
    double   sumDelayMs{0.0};
    uint64_t delaySamples{0};

    // window 狀態（每次 snapshot 後重設）
    uint64_t rxBytesWin{0};  // window 起始時的 rxBytes
    double   tWin{0.0};      // window 起始的模擬時間

    double DropRate() const
    {
        return (txPkts > 0) ? 100.0 * dropPkts / txPkts : 0.0;
    }

    double AvgDelayMs() const
    {
        return (delaySamples > 0) ? sumDelayMs / static_cast<double>(delaySamples) : 0.0;
    }

    // 當前 window 的吞吐量（kbps）
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

struct ObsConfig
{
    double      snapshotIntervalSec{10.0};  // 寫入 log 的週期（秒）
    double      dropAlertThreshPct{50.0};   // drop rate % 超過此值觸發 stdout 事件
    double      trafficStartSec{1.0};       // 流量開始前不觸發 throughput=0 警示
    std::string logFilePath{"e2e_link_obs.csv"};
};

// --- 全域 observer 狀態 ---
static std::map<std::string, SegLinkStats> g_feederObsStats;   // key: "sat<nodeIdx>"
static std::map<std::string, SegLinkStats> g_serviceObsStats;  // key: "sat<nodeIdx>"
static std::map<std::string, SegLinkStats> g_islObsStats2;     // key: "<srcIdx>-<dstIdx>"
static ObsConfig                           g_obsCfg;
static std::ofstream                       g_obsLog;
static std::map<std::string, double>       g_prevObsDropRate;  // 用於事件邊緣偵測

// --- Orbiter feeder-link callbacks ---

static void
OrbiterRxFeederCb(std::string key, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    // feeder link RX：衛星從 GW 收到封包（FWD 方向）
    auto& s  = g_feederObsStats[key];
    s.txPkts++;
    s.rxPkts++;
    s.rxBytes += pkt->GetSize();
}

static void
OrbiterFeederDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // feeder link 傳播延遲累加
    auto& s = g_feederObsStats[key];
    s.sumDelayMs += delay.GetMilliSeconds();
    s.delaySamples++;
}

// --- Orbiter service-link callbacks ---

static void
OrbiterRxUserCb(std::string key, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    // service link RX：衛星從 UT 收到封包（RTN 方向）
    auto& s  = g_serviceObsStats[key];
    s.txPkts++;
    s.rxPkts++;
    s.rxBytes += pkt->GetSize();
}

static void
OrbiterUserDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // service link 傳播延遲累加
    auto& s = g_serviceObsStats[key];
    s.sumDelayMs += delay.GetMilliSeconds();
    s.delaySamples++;
}

// --- ISL observer callback ---
// 與既有 IslPacketDropCallback 並行掛在同一個 PacketDropRateTrace 上。
// NS3 TracedCallback 是 multicast，允許同一 trace source 掛多個 callback。
// 使用 g_nodeToSatId（由 ConnectIslDropTrace 填入）推導 key。

static void
IslObsCb(uint32_t pktSize, Ptr<Node> srcNode, Ptr<Node> dstNode, bool dropped)
{
    auto srcIt = g_nodeToSatId.find(srcNode);
    auto dstIt = g_nodeToSatId.find(dstNode);
    if (srcIt == g_nodeToSatId.end() || dstIt == g_nodeToSatId.end())
    {
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

// --- 事件偵測（stdout 輸出） ---

static void
CheckAndAlertObs(const std::string& linkType,
                 const std::string& key,
                 const SegLinkStats& stats,
                 double              nowSec)
{
    double dr     = stats.DropRate();
    auto   prevIt = g_prevObsDropRate.find(key);
    double prevDr = (prevIt != g_prevObsDropRate.end()) ? prevIt->second : -1.0;

    // drop rate 上升超過閾值
    if (dr >= g_obsCfg.dropAlertThreshPct && prevDr < g_obsCfg.dropAlertThreshPct)
    {
        std::cout << "[OBS][EVENT] t=" << std::fixed << std::setprecision(1) << nowSec
                  << "s  [" << linkType << "] " << key
                  << "  drop_rate=" << std::setprecision(1) << dr << "%"
                  << "  => LINK DEGRADED\n";
    }
    // drop rate 回復低於閾值
    else if (prevDr >= g_obsCfg.dropAlertThreshPct && dr < g_obsCfg.dropAlertThreshPct)
    {
        std::cout << "[OBS][EVENT] t=" << std::fixed << std::setprecision(1) << nowSec
                  << "s  [" << linkType << "] " << key
                  << "  drop_rate=" << std::setprecision(1) << dr << "%"
                  << "  => LINK RECOVERED\n";
    }

    // throughput 在流量啟動後歸零（可能鏈路斷線）
    if (nowSec > g_obsCfg.trafficStartSec + g_obsCfg.snapshotIntervalSec)
    {
        double tput = stats.WindowThroughputKbps(nowSec);
        if (stats.rxPkts > 0 && tput < 1e-9)
        {
            std::cout << "[OBS][EVENT] t=" << std::fixed << std::setprecision(1) << nowSec
                      << "s  [" << linkType << "] " << key
                      << "  window_throughput=0 kbps  => POSSIBLE LINK FAILURE\n";
        }
    }

    g_prevObsDropRate[key] = dr;
}

// --- CSV log row writer ---

static void
WriteObsLogRow(const std::string&  linkType,
               const std::string&  key,
               const SegLinkStats& stats,
               double              nowSec)
{
    if (!g_obsLog.is_open())
    {
        return;
    }
    // 欄位：time_s, link_type, link_id, rx_pkts, rx_bytes,
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

// --- 定期快照（每 snapshotIntervalSec 秒） ---

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

    // 排程下一次快照
    Simulator::Schedule(Seconds(g_obsCfg.snapshotIntervalSec), &TakeObsSnapshot);
}

// --- 掛載所有鏈路 trace ---
// 必須在 ConnectIslDropTrace() 之後呼叫（ISL callback 依賴 g_nodeToSatId）。

static void
ConnectLinkObserverTraces()
{
    NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();
    uint32_t      connFeeder{0}, connService{0}, connIsl{0};

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

            // feeder link：GW→SAT 方向（FWD feeder RX）
            if (orbDev->TraceConnectWithoutContext(
                    "RxFeeder",
                    MakeBoundCallback(&OrbiterRxFeederCb, satKey)))
            {
                ++connFeeder;
            }
            orbDev->TraceConnectWithoutContext(
                "RxFeederLinkDelay",
                MakeBoundCallback(&OrbiterFeederDelayCb, satKey));

            // service link：UT→SAT 方向（RTN user RX）
            if (orbDev->TraceConnectWithoutContext(
                    "RxUser",
                    MakeBoundCallback(&OrbiterRxUserCb, satKey)))
            {
                ++connService;
            }
            orbDev->TraceConnectWithoutContext(
                "RxUserLinkDelay",
                MakeBoundCallback(&OrbiterUserDelayCb, satKey));

            // ISL：並行掛第二個 callback 在 PacketDropRateTrace 上
            auto islDevices = orbDev->GetIslsNetDevices();
            for (auto& islDev : islDevices)
            {
                if (islDev->TraceConnectWithoutContext(
                        "PacketDropRateTrace",
                        MakeCallback(&IslObsCb)))
                {
                    ++connIsl;
                }
            }

            // 初始化各 satellite 的 window 起始點
            g_feederObsStats[satKey].BeginWindow(0.0);
            g_serviceObsStats[satKey].BeginWindow(0.0);
            break;  // 每個 satellite node 只有一個 SatOrbiterNetDevice
        }
    }

    std::cout << "[OBS] traces connected:"
              << "  feeder=" << connFeeder
              << "  service=" << connService
              << "  isl=" << connIsl
              << "\n";
}

// --- 模擬結束後印出總覽 ---

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
            std::string id = label + ":" + kv.first;
            std::cout << std::left
                      << std::setw(24) << id
                      << std::setw(10) << kv.second.rxPkts
                      << std::setw(13) << kv.second.rxBytes
                      << std::setw(11) << kv.second.dropPkts
                      << std::setw(14) << std::fixed << std::setprecision(2)
                      << kv.second.DropRate()
                      << std::setprecision(2) << kv.second.AvgDelayMs() << "\n";
        }
    };

    printSection("feeder",  g_feederObsStats);
    printSection("service", g_serviceObsStats);
    printSection("isl",     g_islObsStats2);

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
    std::string    mode{"gw2gw"};
    TrafficProfile legacyProfile{TrafficProfile::NONE};
    bool           explicitSegments{false};
    double         simTimeSec{0.0};

    E2ESegmentConfig feederlink{};
    E2ESegmentConfig isl{};
    E2ESegmentConfig servicelink{};

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

    if (cfg.mode == "gw2gw")
    {
        NS_ABORT_MSG_IF(cfg.gwSrc == cfg.gwDst,
                        "gwSrc and gwDst must be different in gw2gw mode");
        NS_ABORT_MSG_IF(FindGatewayPreset(cfg.gwSrc) == nullptr,
                        "Unknown gwSrc=" << cfg.gwSrc);
        NS_ABORT_MSG_IF(FindGatewayPreset(cfg.gwDst) == nullptr,
                        "Unknown gwDst=" << cfg.gwDst);
    }

    if (cfg.mode != "gw2gw" || HasEdgeSegmentEnabled(cfg))
    {
        uint32_t gwCheckId = (cfg.mode == "gw2gw") ? cfg.gwSrc : cfg.gwId;
        NS_ABORT_MSG_IF(FindGatewayPreset(gwCheckId) == nullptr,
                        "Unknown gwId=" << gwCheckId);
    }

    if (cfg.feederlink.enabled && cfg.isl.enabled && cfg.servicelink.enabled && cfg.mode == "gw2ut")
    {
        NS_ABORT_MSG_IF(cfg.utName.empty(), "utName must not be empty for full gw2ut e2e");
    }
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
              << " segments={"
              << "feederlink=" << (cfg.feederlink.enabled ? "on" : "off") << ", "
              << "isl=" << (cfg.isl.enabled ? "on" : "off") << ", "
              << "servicelink=" << (cfg.servicelink.enabled ? "on" : "off") << "}"
              << " traffic={"
              << "sharedEdge=" << (plan.installSharedEdgeTraffic ? "on" : "off") << ", "
              << "islBg=" << (plan.installIslBackgroundTraffic ? "on" : "off") << ", "
              << "gw2gwBg=" << (plan.installGw2GwBackgroundTraffic ? "on" : "off") << ", "
              << "gw2gwDirect=" << (plan.installGw2GwDirectTraffic ? "on" : "off") << "}"
              << "\n";
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
                  << " utUsers=" << utUsers.GetN()
                  << ", skipping traffic installation.\n";
        return;
    }

    double stopSec = ResolveTrafficStopSec(cfg, simTimeSec);

    std::cout << "[TRAFFIC][" << segmentLabel << "] gwId=" << gwId
              << " gwUsers=" << gwUsers.GetN()
              << " utUsers=" << utUsers.GetN()
              << " start=" << cfg.startSec << "s"
              << " stop=" << stopSec << "s\n";

    Ptr<SatTrafficHelper> trafficHelper = simHelper->GetTrafficHelper();

    if (cfg.enableFwd)
    {
        trafficHelper->AddCbrTraffic(
            SatTrafficHelper::FWD_LINK,
            SatTrafficHelper::UDP,
            MilliSeconds(cfg.fwdIntervalMs),
            cfg.fwdPktBytes,
            gwUsers,
            utUsers,
            Seconds(cfg.startSec),
            Seconds(stopSec),
            Seconds(0));

        std::cout << "[TRAFFIC][" << segmentLabel << "] FWD installed:"
                  << " interval=" << cfg.fwdIntervalMs << "ms"
                  << " pktSize=" << cfg.fwdPktBytes << "B"
                  << " rate~"
                  << (cfg.fwdPktBytes * 8.0 * 1000.0 / cfg.fwdIntervalMs / 1000.0)
                  << " kbps/flow\n";
    }

    if (cfg.enableRtn)
    {
        trafficHelper->AddCbrTraffic(
            SatTrafficHelper::RTN_LINK,
            SatTrafficHelper::UDP,
            MilliSeconds(cfg.rtnIntervalMs),
            cfg.rtnPktBytes,
            gwUsers,
            utUsers,
            Seconds(cfg.startSec),
            Seconds(stopSec),
            Seconds(0));

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
              << gwAnchorId << " <-> all UTs\n";

    InstallGwUtSegmentTrafficBase(simHelper, bg, simTimeSec, gwAnchorId, "isl");
}

static void
InstallGw2GwBackgroundLoad(Ptr<SimulationHelper> simHelper,
                           double                simTimeSec,
                           uint32_t              gwSrc,
                           uint32_t              gwDst)
{
    TrafficConfig srcCfg;
    srcCfg.enableFwd = true;
    srcCfg.enableRtn = true;
    srcCfg.fwdIntervalMs = 60;
    srcCfg.rtnIntervalMs = 60;
    srcCfg.fwdPktBytes = 1500;
    srcCfg.rtnPktBytes = 1024;
    srcCfg.startSec = 1.0;
    srcCfg.stopSec = simTimeSec - 1.0;

    TrafficConfig dstCfg = srcCfg;

    std::cout << "[TRAFFIC][gw2gw] install GW-side background load for gwSrc="
              << gwSrc << " and gwDst=" << gwDst << "\n";

    InstallGwUtSegmentTrafficBase(simHelper, srcCfg, simTimeSec, gwSrc, "feederlink");
    InstallGwUtSegmentTrafficBase(simHelper, dstCfg, simTimeSec, gwDst, "servicelink");
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

    NS_ABORT_MSG_IF(!srcUser, "[GW2GW_DIRECT] GetGwUserNode(" << gwSrc << ") returned null");
    NS_ABORT_MSG_IF(!dstUser, "[GW2GW_DIRECT] GetGwUserNode(" << gwDst << ") returned null");

    Ipv4Address srcAddr = satHelper->GetUserAddress(srcUser);
    Ipv4Address dstAddr = satHelper->GetUserAddress(dstUser);

    std::cout << "[GW2GW_DIRECT] GW" << gwSrc << "_user=" << srcAddr
              << " -> GW" << gwDst << "_user=" << dstAddr
              << " start=" << startSec << "s"
              << " stop=" << stopSec << "s\n";

    const uint16_t port = 9001;

    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(dstAddr, port));
    ApplicationContainer sinkApps = sink.Install(dstUser);
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec + 2.0));

    OnOffHelper sender("ns3::UdpSocketFactory",
                       InetSocketAddress(dstAddr, port));
    sender.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    sender.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    sender.SetAttribute("DataRate", DataRateValue(DataRate("40960bps")));
    sender.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer senderApps = sender.Install(srcUser);
    senderApps.Start(Seconds(startSec));
    senderApps.Stop(Seconds(stopSec));

    Simulator::Schedule(
        Seconds(stopSec + 1.0),
        [sinkApps, gwSrc, gwDst, srcAddr, dstAddr]()
        {
            auto sinkApp = DynamicCast<PacketSink>(sinkApps.Get(0));
            uint64_t rxBytes = sinkApp ? sinkApp->GetTotalRx() : 0;
            uint64_t estPkts = (rxBytes > 0) ? (rxBytes / 512) : 0;

            std::cout << "\n[GW2GW_DIRECT] === Delivery summary ===\n"
                      << "  src: GW" << gwSrc << " (" << srcAddr << ")\n"
                      << "  dst: GW" << gwDst << " (" << dstAddr << ")\n"
                      << "  received: " << rxBytes << " bytes (~" << estPkts << " pkts)\n";

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

static bool
InstallFeederlinkTraffic(Ptr<SimulationHelper>   simHelper,
                         const E2EConfig&        cfg,
                         const E2EExecutionPlan& plan)
{
    PrintSegmentBanner(E2ESegment::FEEDERLINK, cfg.feederlink.enabled);
    if (!cfg.feederlink.enabled)
    {
        return false;
    }

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

    std::cout << "[E2E][feederlink] no dedicated traffic generator selected\n";
    return false;
}

static void
InstallIslTraffic(Ptr<SimulationHelper>   simHelper,
                  const E2EConfig&        cfg,
                  const E2EExecutionPlan& plan)
{
    PrintSegmentBanner(E2ESegment::ISL, cfg.isl.enabled);
    if (!cfg.isl.enabled)
    {
        return;
    }

    if (plan.installIslBackgroundTraffic)
    {
        uint32_t gwAnchorId = (cfg.mode == "gw2gw") ? cfg.gwSrc : cfg.gwId;
        InstallSat2SatBackgroundLoad(simHelper, cfg.simTimeSec, gwAnchorId);
        return;
    }

    std::cout << "[E2E][isl] routing/transit enabled without extra ISL-only load generator\n";
}

static bool
InstallServicelinkTraffic(Ptr<SimulationHelper>   simHelper,
                          const E2EConfig&        cfg,
                          const E2EExecutionPlan& plan)
{
    PrintSegmentBanner(E2ESegment::SERVICELINK, cfg.servicelink.enabled);
    if (!cfg.servicelink.enabled)
    {
        return false;
    }

    if (plan.installSharedEdgeTraffic)
    {
        InstallGwUtSegmentTrafficBase(simHelper,
                                      cfg.servicelink.traffic,
                                      cfg.simTimeSec,
                                      plan.edgeGatewayId,
                                      "servicelink");
        return true;
    }

    if (plan.installGw2GwDirectTraffic || plan.installGw2GwBackgroundTraffic)
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
                  const E2EConfig&        cfg,
                  const E2EExecutionPlan& plan)
{
    bool edgeTrafficInstalled = InstallFeederlinkTraffic(simHelper, cfg, plan);
    InstallIslTraffic(simHelper, cfg, plan);

    if (!edgeTrafficInstalled)
    {
        InstallServicelinkTraffic(simHelper, cfg, plan);
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

// === Routing / Case Execution ===============================================

static void
ConfigureRoutingCase(Ptr<IslRoutingManager> routingMgr,
                     const E2EConfig&       cfg,
                     double                 elevMinDeg)
{
    if (cfg.mode == "sat2sat")
    {
        std::cout << "\n[CASE] sat2sat | src=" << cfg.satSrc
                  << " dst=" << cfg.satDst << "\n";
        routingMgr->PrintRouteReport({{cfg.satSrc, cfg.satDst}});
        return;
    }

    if (cfg.mode == "gw2gw")
    {
        routingMgr->SetGwElevationThreshold(elevMinDeg);
        AddGatewayOrAbort(routingMgr, cfg.gwSrc);
        AddGatewayOrAbort(routingMgr, cfg.gwDst);
        routingMgr->AddGwPair(cfg.gwSrc, cfg.gwDst);

        std::cout << "\n[CASE] gw2gw | gwSrc=" << cfg.gwSrc
                  << " gwDst=" << cfg.gwDst << "\n";
        routingMgr->PrecomputeGwRoutes();
        routingMgr->PrintGwRouteReport();
        return;
    }

    routingMgr->SetGwElevationThreshold(elevMinDeg);
    AddGatewayOrAbort(routingMgr, cfg.gwId);
    routingMgr->AddUserTerminal(cfg.utId, cfg.utLatDeg, cfg.utLonDeg, cfg.utName);
    routingMgr->AddGwUtPair(cfg.gwId, cfg.utId);

    std::cout << "\n[CASE] gw2ut | gwId=" << cfg.gwId
              << " utId=" << cfg.utId
              << " utLatDeg=" << cfg.utLatDeg
              << " utLonDeg=" << cfg.utLonDeg << "\n";
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
        ns3BasePath + "/contrib/satellite/data/scenarios/" +
        scenarioName + "/positions/isls.txt";

    std::string mode = "gw2gw";
    std::string trafficProf = "none";
    bool        enableFeederlink = false;
    bool        enableIsl = false;
    bool        enableServicelink = false;
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

    uint32_t satSrc = 0;
    uint32_t satDst = 10;

    uint32_t gwSrc = 0;
    uint32_t gwDst = 1;

    uint32_t    gwId = 0;
    uint32_t    utId = 0;
    double      utLatDeg = 25.0330;
    double      utLonDeg = 121.5654;
    std::string utName = "UT-Taipei";

    // === Link Observability CLI 參數 ===
    std::string obsLogPath      = "e2e_link_obs.csv";  // CSV log 輸出路徑
    double      obsIntervalSec  = 10.0;                // 快照寫入週期
    double      obsDropAlertPct = 50.0;                // drop rate 事件觸發門檻

    CommandLine cmd;
    cmd.AddValue("mode", "Routing case: sat2sat | gw2gw | gw2ut", mode);
    cmd.AddValue("trafficProfile", "Legacy profile: none | gw2ut | sat2sat | gw2gw | gw2gw_direct", trafficProf);
    cmd.AddValue("enableFeederlink", "Enable feederlink segment for e2e orchestration (0/1)", enableFeederlink);
    cmd.AddValue("enableIsl", "Enable ISL segment for e2e orchestration (0/1)", enableIsl);
    cmd.AddValue("enableServicelink", "Enable servicelink segment for e2e orchestration (0/1)", enableServicelink);
    cmd.AddValue("rbdcVerbose", "Print each RBDC request (1=on, 0=off, default=0)", rbdcVerbose);
    cmd.AddValue("islDropThreshPct", "ISL overall drop rate PASS threshold (%, default=1.0)", islDropThreshPct);

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

    // 設定 observer 參數（在 cmd.Parse 後，trafficCfg.startSec 已解析完成）
    g_obsCfg.snapshotIntervalSec = obsIntervalSec;
    g_obsCfg.dropAlertThreshPct  = obsDropAlertPct;
    g_obsCfg.trafficStartSec     = trafficCfg.startSec;
    g_obsCfg.logFilePath         = obsLogPath;

    NS_ABORT_MSG_IF(slotInterval <= 0.0, "slotInterval must be > 0");
    NS_ABORT_MSG_IF(simTime < 0.0, "simTime must be >= 0");
    NS_ABORT_MSG_IF(satSrc >= numSats || satDst >= numSats,
                    "satSrc/satDst must be < " << numSats);

    TrafficProfile profile = ParseTrafficProfile(trafficProf);

    E2EConfig e2eCfg;
    e2eCfg.mode = mode;
    e2eCfg.legacyProfile = profile;
    e2eCfg.explicitSegments = enableFeederlink || enableIsl || enableServicelink;
    e2eCfg.simTimeSec = simTime;
    e2eCfg.satSrc = satSrc;
    e2eCfg.satDst = satDst;
    e2eCfg.gwSrc = gwSrc;
    e2eCfg.gwDst = gwDst;
    e2eCfg.gwId = gwId;
    e2eCfg.utId = utId;
    e2eCfg.utLatDeg = utLatDeg;
    e2eCfg.utLonDeg = utLonDeg;
    e2eCfg.utName = utName;

    e2eCfg.feederlink.enabled = enableFeederlink;
    e2eCfg.isl.enabled = enableIsl;
    e2eCfg.servicelink.enabled = enableServicelink;

    e2eCfg.feederlink.traffic = trafficCfg;
    e2eCfg.isl.traffic = trafficCfg;
    e2eCfg.servicelink.traffic = trafficCfg;

    ApplyLegacySegmentDefaults(e2eCfg);
    E2EExecutionPlan e2ePlan = BuildE2EPlan(e2eCfg);

    const double   islRateBps = islRateMbps * 1.0e6;
    const double   cooldownSec = slotInterval * cooldownRatio;
    const uint32_t numSlots =
        static_cast<uint32_t>(std::floor(simTime / slotInterval)) + 1;

    std::cout << "[CFG] mode=" << mode
              << " trafficProfile=" << trafficProf
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
    Config::SetDefault("ns3::SatHelper::GwUsers", UintegerValue(3));
    Config::SetDefault("ns3::SatGwMac::SendNcrBroadcast", BooleanValue(false));
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    ConfigureQoS();

    Ptr<SimulationHelper> simHelper =
        CreateObject<SimulationHelper>("test-iridium-3segment-e2e");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));
    simHelper->SetBeamSet(std::set<uint32_t>{beamId});
    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    uint32_t islConnected = ConnectIslDropTrace();

    // === 初始化 Link Observer ===
    // 開啟 CSV log 並寫入 header
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

    // 掛載 feeder / service / ISL trace（必須在 ConnectIslDropTrace 之後）
    ConnectLinkObserverTraces();

    // 排程第一次快照
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

    PrintObsFinalSummary();
    routingMgr->PrintStats();
    routingMgr->PrintLoadStats();
    PrintIslDropStats(islDropThreshPct, islConnected);

    auto wallEnd = std::chrono::steady_clock::now();
    auto wallMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(wallEnd - wallStart).count();

    std::cout << "Total wall time: " << wallMs / 1000.0 << " s\n";
    std::cout << "Event count:     " << Simulator::GetEventCount() << "\n";

    Simulator::Destroy();
    return 0;
}
