#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <map>

using namespace ns3;

namespace
{

static void
ConfigureQoS()
{
    // 保留空函式，避免不同 SNS3 版本 attribute 名稱不一致時觸發 NS_FATAL。
}

// ── RBDC Trace 觀察回調 ────────────────────────────────────────────────────
//
// 觀察 SatRequestManager 的 RBDC（Rate-Based Dynamic Capacity）容量請求。
//
// RBDC ：
//   - 方向：RTN link（UT → GW），UT 向 NCC 申請傳輸容量
//   - trig：SatRequestManager::DoRbdcLegacy()
//             根據 SatQueue::QueueStats_t（incomingRateKbps + queueSizeBytes）
//             計算需求速率後呼叫 m_rbdcTrace(rbdcRateKbps)
//   - period：per time/frame（~26ms）
//
// Trace source （sat-rtn-system-test-example.cc）：
//   /NodeList/*/DeviceList/*/SatLlc/SatRequestManager/RbdcTrace
//   注意：是 SatLlc，不是 SatMac
//
// callback 簽名（SatRequestManager::RbdcTraceCallback）：
//   typedef void (*RbdcTraceCallback)(uint32_t requestSize);
//   使用 ConnectWithoutContext，無 context 字串前綴
//
// 過濾：requestKbps == 0 代表佇列空、不需額外容量，濾掉避免大量雜訊。
static void
RbdcTraceCallback(uint32_t requestKbps)
{
    if (requestKbps > 0)
    {
        std::cout << "[RBDC] t=" << Simulator::Now().GetSeconds() << "s"
                  << " request=" << requestKbps << " kbps\n";
    }
}

// ── ISL 封包丟棄率追蹤 ─────────────────────────────────────────────────────
//
// 追蹤每條 ISL 上的封包丟棄率，提供比received > 0更精確的驗證指標。
//
// 實作依據：
//   - TraceSource：PointToPointIslNetDevice::PacketDropRateTrace
//     (model/satellite-point-to-point-isl-net-device.cc)
//   - Callback 簽名：void(uint32_t pktSize, Ptr<Node> src, Ptr<Node> dst, bool dropped)
//     dropped=true  → 封包因 queue full 或 error model 被丟棄
//     dropped=false → 封包成功入列（注意：這裡計的是「嘗試入列」，不是「成功送達」）
//   - ISL key 格式：satSrcId-satDstId，與 SatStatsHelper::GetIdentifierForIsl 一致
//
// Pass/Fail 標準：整體丟棄率 < islDropThreshPct（預設 1.0%）

struct IslDropStats
{
    uint64_t total{0};    // 嘗試傳送的封包總數（含成功與丟棄）
    uint64_t dropped{0};  // 被丟棄的封包數
};

// 全域查找表：Ptr<Node> → satellite index（避免 callback 內 O(n) 遍歷）
static std::map<Ptr<Node>, uint32_t>       g_nodeToSatId;
// 全域統計：key = "satSrcId-satDstId"
static std::map<std::string, IslDropStats> g_islDropStats;

// PacketDropRateTrace callback
// 每次 PointToPointIslNetDevice 嘗試入列封包時觸發（包含成功與丟棄兩種情況）
static void
IslPacketDropCallback(uint32_t /*pktSize*/, Ptr<Node> srcNode, Ptr<Node> dstNode, bool dropped)
{
    auto srcIt = g_nodeToSatId.find(srcNode);
    auto dstIt = g_nodeToSatId.find(dstNode);
    if (srcIt == g_nodeToSatId.end() || dstIt == g_nodeToSatId.end())
        return;

    std::string key = std::to_string(srcIt->second) + "-" + std::to_string(dstIt->second);
    g_islDropStats[key].total++;
    if (dropped)
        g_islDropStats[key].dropped++;
}

// 在 CreateSatScenario() 之後呼叫：建立 node→satId 查找表，並掛接所有 ISL 的 trace
// 回傳成功掛接的介面數（每條雙向 link 有 2 個介面，unique link 數 = 回傳值 / 2）
static uint32_t
ConnectIslDropTrace()
{
    NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();

    // 預建查找表
    g_nodeToSatId.clear();
    for (uint32_t i = 0; i < sats.GetN(); ++i)
        g_nodeToSatId[sats.Get(i)] = i;

    uint32_t connected = 0;
    for (uint32_t i = 0; i < sats.GetN(); ++i)
    {
        Ptr<Node> satNode = sats.Get(i);
        for (uint32_t d = 0; d < satNode->GetNDevices(); ++d)
        {
            Ptr<SatOrbiterNetDevice> orbDev =
                DynamicCast<SatOrbiterNetDevice>(satNode->GetDevice(d));
            if (!orbDev) continue;

            // 掛接此衛星上每個 ISL 介面的 PacketDropRateTrace
            for (auto& islDev : orbDev->GetIslsNetDevices())
            {
                if (islDev->TraceConnectWithoutContext(
                        "PacketDropRateTrace",
                        MakeCallback(&IslPacketDropCallback)))
                    ++connected;
            }
            break;  // 每個 node 只有一個 SatOrbiterNetDevice
        }
    }
    // connected 為有向介面數；每條雙向 link = 2 介面，故 unique link = connected / 2
    std::cout << "[ISL_DROP] trace connected: " << connected
              << " ISL interfaces (" << connected / 2 << " unique links)\n";
    return connected;
}

// 輸出 ISL 丟棄率與傳輸成功率統計，並依 threshPct 判定 PASS / FAIL
// connectedInterfaces：ConnectIslDropTrace() 的回傳值，用於無事件時的 FAIL 判斷
// 只輸出有丟棄的 ISL 明細，所有 ISL 的合計納入整體統計
static void
PrintIslDropStats(double threshPct, uint32_t connectedInterfaces)
{
    std::cout << "\n=== ISL Packet Drop Rate Summary ===\n";

    // 無事件：可能是 trace 未連接，或流量完全沒通過 ISL，兩者都是異常 → FAIL
    if (g_islDropStats.empty())
    {
        if (connectedInterfaces == 0)
            std::cout << "  [FAIL] trace connection failed (0 interfaces connected)\n";
        else
            std::cout << "  [FAIL] " << connectedInterfaces
                      << " interfaces connected but 0 events recorded\n"
                      << "         (traffic may not have reached ISL layer)\n";
        std::cout << "=====================================\n\n";
        return;
    }

    // 統計有丟棄的 ISL 明細
    struct DropRow
    {
        std::string isl;
        uint64_t    total;
        uint64_t    dropped;
        double      dropRate;
        double      successRate;
    };
    std::vector<DropRow> rows;
    uint64_t sumTotal = 0, sumDropped = 0;

    for (const auto& kv : g_islDropStats)
    {
        sumTotal   += kv.second.total;
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
        // 欄寬：ISL(14) total_pkts(12) dropped(10) drop_rate(14) success_rate
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
                      << std::setw(14) << (std::to_string(static_cast<int>(r.dropRate * 1000) / 1000.0).substr(0, 6))
                      << std::fixed << std::setprecision(3) << r.successRate << "\n";
        }
        std::cout << std::string(62, '-') << "\n";
    }
    else
    {
        std::cout << "  (all ISLs: 0 drops)\n";
    }

    double overallDrop    = (sumTotal > 0) ? (100.0 * sumDropped / sumTotal) : 0.0;
    double overallSuccess = 100.0 - overallDrop;
    std::cout << "TOTAL: " << sumTotal << " pkts, "
              << sumDropped << " dropped | "
              << "drop_rate=" << std::fixed << std::setprecision(3) << overallDrop << "% | "
              << "success_rate=" << std::fixed << std::setprecision(3) << overallSuccess << "%\n";

    if (overallDrop < threshPct)
        std::cout << "[PASS] overall ISL drop rate < " << threshPct << "%\n";
    else
        std::cout << "[FAIL] overall ISL drop rate = " << overallDrop
                  << "% >= threshold " << threshPct << "%\n";

    std::cout << "=====================================\n\n";
}

// ── 流量配置 ──────────────────────────────────────────────────────────────
// trafficProfile ：
//   none     : 不裝流量
//   gw2ut    :  GW→UT / UT→GW 
//   sat2sat  : 強背景流量製造 ISL queue 壓力，驗證 Arbiter / load-aware reroute
//   gw2gw    : 用 GW 端背景流量驅動 queue / capacity-request 決策，再觀察 gw2gw 路由報告
//
// 並沒有硬做一個未經證實的 direct GW↔GW 或 SAT↔SAT app endpoint。
struct TrafficConfig
{
    bool     enableFwd{true};      // 是否安裝 FWD link (GW→UT) 流量
    bool     enableRtn{true};      // 是否安裝 RTN link (UT→GW) 流量
    uint32_t fwdIntervalMs{100};   // FWD CBR 封包間隔（毫秒）
    uint32_t rtnIntervalMs{500};   // RTN CBR 封包間隔（毫秒）
    uint32_t fwdPktBytes{1500};    // FWD 封包大小（bytes）
    uint32_t rtnPktBytes{512};     // RTN 封包大小（bytes）
    double   startSec{1.0};        // 流量開始時間（s）
    double   stopSec{0.0};         // 流量結束時間（0 = simTime - 1）
};

enum class TrafficProfile
{
    NONE,
    GW2UT_APP,      // 正常 GW↔UT 業務流
    SAT2SAT_BG,     // 用重背景流量製造 ISL queue load
    GW2GW_BG,       // 用 GW 端背景流量驅動 queue / capacity-request 行為
    GW2GW_DIRECT    // 真實 GW_user→GW_user 端到端資料平面驗證
                    // 若 received=0：GW unicast forward rule 未設定，需補 static route
                    // 若 received>0：端到端 ISL 路徑完整驗證成功
};

static TrafficProfile
ParseTrafficProfile(const std::string& s)
{
    if (s == "none")         return TrafficProfile::NONE;
    if (s == "gw2ut")        return TrafficProfile::GW2UT_APP;
    if (s == "sat2sat")      return TrafficProfile::SAT2SAT_BG;
    if (s == "gw2gw")        return TrafficProfile::GW2GW_BG;
    if (s == "gw2gw_direct") return TrafficProfile::GW2GW_DIRECT;

    NS_ABORT_MSG("trafficProfile must be one of: none | gw2ut | sat2sat | gw2gw | gw2gw_direct");
    return TrafficProfile::NONE;
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
                    "Unknown gwId=" << gwId
                    << ". Supported presets: 0(Taipei), 1(Tokyo), 2(SanFrancisco)");
    routingMgr->AddGateway(gw->id, gw->latDeg, gw->lonDeg, gw->name);
}

// ── Helper：取得指定 GW user node ────────────────────────────────────────
static NodeContainer
GetGwUsers(uint32_t gwId)
{
    auto topo = Singleton<SatTopology>::Get();
    return NodeContainer(topo->GetGwUserNode(gwId));
}

// ── Helper：取得所有 UT user nodes ────────────────────────────────────────
static NodeContainer
GetUtUsers()
{
    return Singleton<SatTopology>::Get()->GetUtUserNodes();
}

// ── 最基本的 CBR 安裝器（對應目前可見 SatTrafficHelper API）───────────────
// FWD_LINK : gwUsers -> utUsers
// RTN_LINK : utUsers -> gwUsers
static void
InstallGwUtCbrTraffic(Ptr<SimulationHelper> simHelper,
                      const TrafficConfig&  cfg,
                      double                simTimeSec,
                      uint32_t              gwId)
{
    if (!cfg.enableFwd && !cfg.enableRtn)
    {
        std::cout << "[TRAFFIC] Both FWD and RTN disabled, skipping traffic installation.\n";
        return;
    }

    NodeContainer gwUsers = GetGwUsers(gwId);
    NodeContainer utUsers = GetUtUsers();

    if (gwUsers.GetN() == 0 || utUsers.GetN() == 0)
    {
        std::cout << "[TRAFFIC] WARNING: gwUsers=" << gwUsers.GetN()
                  << " utUsers=" << utUsers.GetN()
                  << ", skipping traffic installation.\n";
        return;
    }

    double stopSec = (cfg.stopSec > 0.0) ? cfg.stopSec : (simTimeSec - 1.0);

    std::cout << "[TRAFFIC] gwId=" << gwId
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

        std::cout << "[TRAFFIC] FWD installed:"
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
            gwUsers,   // RTN 下此參數為接收端（GW）
            utUsers,   // RTN 下此參數為發送端（UT）
            Seconds(cfg.startSec),
            Seconds(stopSec),
            Seconds(0));

        std::cout << "[TRAFFIC] RTN installed:"
                  << " interval=" << cfg.rtnIntervalMs << "ms"
                  << " pktSize=" << cfg.rtnPktBytes << "B"
                  << " rate~"
                  << (cfg.rtnPktBytes * 8.0 * 1000.0 / cfg.rtnIntervalMs / 1000.0)
                  << " kbps/flow\n";
    }
}

// ── sat2sat profile：用較強背景流量壓出 ISL queue ───────────────────────
// 核心目的：不是做真正 sat endpoint app，而是讓 DropTail queue / EMA load cost 動起來。
// 這樣可驗證：
//   GetLinkQueueDelay() -> UpdateLoadCosts() -> HasSignificantChange() -> RecomputeAffectedRoutes()
static void
InstallSat2SatBackgroundLoad(Ptr<SimulationHelper> simHelper,
                             double                simTimeSec,
                             uint32_t              gwAnchorId)
{
    TrafficConfig bg;
    bg.enableFwd      = true;
    bg.enableRtn      = true;
    bg.fwdIntervalMs  = 30;    // 比 normal 更密
    bg.rtnIntervalMs  = 30;
    bg.fwdPktBytes    = 1500;
    bg.rtnPktBytes    = 1500;
    bg.startSec       = 1.0;
    bg.stopSec        = simTimeSec - 1.0;

    std::cout << "[TRAFFIC][sat2sat] install aggressive background load via GW="
              << gwAnchorId << " <-> all UTs\n";

    InstallGwUtCbrTraffic(simHelper, bg, simTimeSec, gwAnchorId);
}

// ── gw2gw profile：用 GW 端背景流量去驅動 GW 端 queue / request 決策 ───────
// 注意：基於目前可見 API，這裡採兩端 gateway 分別對 UT 群灌流，目的在於：
//   1. 讓 GW 端 queue / request logic 活化
//   2. 讓網路中產生足夠背景負載
//   3. 保留 gw2gw route report 作為 entry/exit/path 觀察對象
//
// 這不是「直接 GW-user -> GW-user app」，但在目前可見 API 下是最穩妥可實作版本。
static void
InstallGw2GwBackgroundLoad(Ptr<SimulationHelper> simHelper,
                           double                simTimeSec,
                           uint32_t              gwSrc,
                           uint32_t              gwDst)
{
    TrafficConfig srcCfg;
    srcCfg.enableFwd      = true;
    srcCfg.enableRtn      = true;
    srcCfg.fwdIntervalMs  = 60;
    srcCfg.rtnIntervalMs  = 60;
    srcCfg.fwdPktBytes    = 1500;
    srcCfg.rtnPktBytes    = 1024;
    srcCfg.startSec       = 1.0;
    srcCfg.stopSec        = simTimeSec - 1.0;

    TrafficConfig dstCfg = srcCfg;

    std::cout << "[TRAFFIC][gw2gw] install GW-side background load for gwSrc="
              << gwSrc << " and gwDst=" << gwDst << "\n";

    InstallGwUtCbrTraffic(simHelper, srcCfg, simTimeSec, gwSrc);
    InstallGwUtCbrTraffic(simHelper, dstCfg, simTimeSec, gwDst);
}

// ── GW-to-GW 直接資料平面驗證 ─────────────────────────────────────────────
//
// 目的：驗證 GW0_user → ISL 網格 → GW1_user 端到端封包是否真正到達。
//
// 使用標準 ns3 OnOffHelper（非 SatTrafficHelper），直接裝在 GW user 節點上：
//   發送端：OnOffHelper（constant-on = CBR 效果）→ 安裝於 GW_src 的 user node
//   接收端：PacketSinkHelper → 安裝於 GW_dst 的 user node
//
// IP 結構（已確認）：
//   所有 GW users 共用 90.1.0.0/255.255.0.0（同一 /16）
//   GW0_user 送往 GW1_user → connected route 自動送到 GW0 → 再往衛星
//
// 判斷基準：
//   received > 0 bytes → GW 節點有 unicast forward rule，端到端路徑有效
//   received = 0 bytes → GW 節點缺少 forward rule，需補 Ipv4StaticRouting 條目
//
// 函式依賴：
//   GetGwUserNode(uint32_t) : satellite-topology.h:358
//   GetUserAddress(Ptr<Node>) : satellite-helper.h:166
//   GetSatelliteHelper() : simulation-helper.h:483
static void
InstallGw2GwDirectApp(Ptr<SimulationHelper> simHelper,
                      uint32_t              gwSrc,
                      uint32_t              gwDst,
                      double                startSec,
                      double                stopSec)
{
    auto topo      = Singleton<SatTopology>::Get();
    auto satHelper = simHelper->GetSatelliteHelper();

    // 取得 GW user 節點（gwId 對應 GW 序號：0=TW, 1=JP, 2=US）
    Ptr<Node> srcUser = topo->GetGwUserNode(gwSrc);
    Ptr<Node> dstUser = topo->GetGwUserNode(gwDst);

    NS_ABORT_MSG_IF(!srcUser, "[GW2GW_DIRECT] GetGwUserNode(" << gwSrc << ") returned null");
    NS_ABORT_MSG_IF(!dstUser, "[GW2GW_DIRECT] GetGwUserNode(" << gwDst << ") returned null");

    // 取得 IP 位址（皆在 90.1.0.0/16 內）
    Ipv4Address srcAddr = satHelper->GetUserAddress(srcUser);
    Ipv4Address dstAddr = satHelper->GetUserAddress(dstUser);

    std::cout << "[GW2GW_DIRECT] GW" << gwSrc << "_user=" << srcAddr
              << "  ->  GW" << gwDst << "_user=" << dstAddr
              << "  start=" << startSec << "s  stop=" << stopSec << "s\n";

    const uint16_t port = 9001;

    // ── 接收端：PacketSinkHelper（UDP） ──────────────────────────────────
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(dstAddr, port));
    ApplicationContainer sinkApps = sink.Install(dstUser);
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(stopSec + 2.0));

    // ── 發送端：OnOffHelper 固定 on（模擬 CBR）────────────────────────────
    // DataRate = 512 bytes * 8 bits / 0.1s = 40960 bps
    // OnTime=1（永遠 on），OffTime=0（從不 off）
    OnOffHelper sender("ns3::UdpSocketFactory",
                       InetSocketAddress(dstAddr, port));
    sender.SetAttribute("OnTime",    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    sender.SetAttribute("OffTime",   StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    sender.SetAttribute("DataRate",  DataRateValue(DataRate("40960bps")));
    sender.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer senderApps = sender.Install(srcUser);
    senderApps.Start(Seconds(startSec));
    senderApps.Stop(Seconds(stopSec));

    // ── 模擬結束前列印收包統計 ────────────────────────────────────────────
    // 以 stopSec + 1.0 觸發，確保最後一批封包已處理完成
    Simulator::Schedule(Seconds(stopSec + 1.0),
        [sinkApps, gwSrc, gwDst, srcAddr, dstAddr]()
        {
            auto sinkApp  = DynamicCast<PacketSink>(sinkApps.Get(0));
            uint64_t rxBytes = sinkApp ? sinkApp->GetTotalRx() : 0;
            uint64_t estPkts = (rxBytes > 0) ? (rxBytes / 512) : 0;

            std::cout << "\n[GW2GW_DIRECT] === 資料平面驗證結果 ===\n"
                      << "  src: GW" << gwSrc << " (" << srcAddr << ")\n"
                      << "  dst: GW" << gwDst << " (" << dstAddr << ")\n"
                      << "  received: " << rxBytes << " bytes (~" << estPkts << " pkts)\n";

            if (rxBytes == 0)
            {
                std::cout << "  [FAIL] received=0 → GW node 缺少 unicast forward rule\n"
                          << "         下一步：補 Ipv4StaticRouting 條目\n";
            }
            else
            {
                std::cout << "  [PASS] received>0 → 端到端 ISL 路徑驗證成功！\n";
            }
            std::cout << "==========================================\n\n";
        });
}

// ── 依 trafficProfile 決定裝流方式 ────────────────────────────────────────
static void
InstallTrafficByProfile(Ptr<SimulationHelper> simHelper,
                        TrafficProfile        profile,
                        const TrafficConfig&  cfg,
                        double                simTimeSec,
                        uint32_t              gwSrc,
                        uint32_t              gwDst,
                        uint32_t              gwIdForGwUt)
{
    switch (profile)
    {
    case TrafficProfile::NONE:
        std::cout << "[TRAFFIC] profile=none (baseline, no traffic)\n";
        break;

    case TrafficProfile::GW2UT_APP:
        std::cout << "[TRAFFIC] profile=gw2ut (normal service traffic)\n";
        InstallGwUtCbrTraffic(simHelper, cfg, simTimeSec, gwIdForGwUt);
        break;

    case TrafficProfile::SAT2SAT_BG:
        std::cout << "[TRAFFIC] profile=sat2sat (ISL background-load mode)\n";
        InstallSat2SatBackgroundLoad(simHelper, simTimeSec, gwSrc);
        break;

    case TrafficProfile::GW2GW_BG:
        std::cout << "[TRAFFIC] profile=gw2gw (GW-side queue/request background-load mode)\n";
        InstallGw2GwBackgroundLoad(simHelper, simTimeSec, gwSrc, gwDst);
        break;

    case TrafficProfile::GW2GW_DIRECT:
        std::cout << "[TRAFFIC] profile=gw2gw_direct (GW_user→GW_user 端到端資料平面驗證)\n";
        InstallGw2GwDirectApp(simHelper, gwSrc, gwDst,
                              cfg.startSec,
                              (cfg.stopSec > 0.0) ? cfg.stopSec : (simTimeSec - 1.0));
        break;
    }
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

    std::string mode         = "gw2gw";  // sat2sat | gw2gw | gw2ut
    std::string trafficProf  = "none";   // none | gw2ut | sat2sat | gw2gw
    bool        rbdcVerbose     = false;  // true = 輸出每筆 RBDC request；false = 靜默（避免大量 log）
    double      islDropThreshPct = 1.0;  // ISL 整體丟棄率 PASS 門檻（%），預設 1%

    double      simTime      = 630.0;
    double      slotInterval = 60.0;
    uint32_t    beamId       = 1;

    double      islMaxDistKm  = 5000.0;
    double      islRateMbps   = 10.0;
    double      emaAlpha      = 0.3;
    double      changeThresh  = 0.1;
    double      cooldownRatio = 0.5;
    double      elevMinDeg    = 5.0;

    TrafficConfig trafficCfg;

    // sat2sat mode
    uint32_t satSrc = 0;
    uint32_t satDst = 10;

    // gw2gw mode
    uint32_t gwSrc = 0;
    uint32_t gwDst = 1;

    // gw2ut mode
    uint32_t gwId      = 0;
    uint32_t utId      = 0;
    double   utLatDeg  = 25.0330;
    double   utLonDeg  = 121.5654;
    std::string utName = "UT-Taipei";

    CommandLine cmd;
    cmd.AddValue("mode",          "Routing case: sat2sat | gw2gw | gw2ut", mode);
    cmd.AddValue("trafficProfile","none | gw2ut | sat2sat | gw2gw | gw2gw_direct", trafficProf);
    cmd.AddValue("rbdcVerbose",      "Print each RBDC request (1=on, 0=off, default=0)", rbdcVerbose);
    cmd.AddValue("islDropThreshPct", "ISL overall drop rate PASS threshold (%, default=1.0)", islDropThreshPct);

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

    cmd.AddValue("fwd",           "Enable FWD link (GW→UT) CBR traffic (0/1)",  trafficCfg.enableFwd);
    cmd.AddValue("rtn",           "Enable RTN link (UT→GW) CBR traffic (0/1)",  trafficCfg.enableRtn);
    cmd.AddValue("fwdIntervalMs", "FWD CBR packet interval (ms)",               trafficCfg.fwdIntervalMs);
    cmd.AddValue("rtnIntervalMs", "RTN CBR packet interval (ms)",               trafficCfg.rtnIntervalMs);
    cmd.AddValue("fwdPktBytes",   "FWD CBR packet size (bytes)",                trafficCfg.fwdPktBytes);
    cmd.AddValue("rtnPktBytes",   "RTN CBR packet size (bytes)",                trafficCfg.rtnPktBytes);
    cmd.AddValue("trafficStart",  "Traffic start time (s)",                     trafficCfg.startSec);
    cmd.AddValue("trafficStop",   "Traffic stop time (s), 0 = simTime-1",       trafficCfg.stopSec);

    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(slotInterval <= 0.0, "slotInterval must be > 0");
    NS_ABORT_MSG_IF(simTime < 0.0, "simTime must be >= 0");
    NS_ABORT_MSG_IF(mode != "sat2sat" && mode != "gw2gw" && mode != "gw2ut",
                    "mode must be one of: sat2sat, gw2gw, gw2ut");
    NS_ABORT_MSG_IF(satSrc >= numSats || satDst >= numSats,
                    "satSrc/satDst must be < " << numSats);

    TrafficProfile profile = ParseTrafficProfile(trafficProf);

    const double   islRateBps  = islRateMbps * 1.0e6;
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
        CreateObject<SimulationHelper>("test-iridium-3modes");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));
    simHelper->SetBeamSet(std::set<uint32_t>{beamId});
    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    // ── ISL Drop Rate Trace 連接 ───────────────────────────────────────────
    // 必須在 CreateSatScenario() 之後，ISL net device 才存在
    // 回傳值傳給 PrintIslDropStats，用於無事件時的 FAIL 診斷
    uint32_t islConnected = ConnectIslDropTrace();

    // ── RBDC Trace 連接 ─────────────────────────────────────────────────────
    // 必須在 CreateSatScenario() 之後掛接，否則 UT 設備尚未存在，
    // ConnectWithoutContext 會連到 0 個物件（不報錯但永遠沒有輸出）。
    // 路徑來自官方範例 sat-rtn-system-test-example.cc:354。
    //
    // rbdcVerbose=false（預設）時跳過連接，避免 91 UT × 26ms 週期產生大量 log。
    // 需要觀察 RBDC 行為時，加 --rbdcVerbose=1 重新執行即可。
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

    // ── 流量安裝：由 trafficProfile 決定，不再硬綁 mode ───────────────────
    InstallTrafficByProfile(simHelper,
                            profile,
                            trafficCfg,
                            simTime,
                            gwSrc,
                            gwDst,
                            gwId);

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
        NS_ABORT_MSG_IF(gwSrc == gwDst,
                        "gwSrc and gwDst must be different in gw2gw mode");

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

    // 輸出最終 EMA queue delay（ms）
    // 用來驗證流量是否真的形成 ISL queue load
    routingMgr->PrintLoadStats();

    // 輸出 ISL 封包丟棄率與傳輸成功率統計，並依 islDropThreshPct 判定 PASS / FAIL
    PrintIslDropStats(islDropThreshPct, islConnected);

    auto wallEnd = std::chrono::steady_clock::now();
    auto wallMs  = std::chrono::duration_cast<std::chrono::milliseconds>(
        wallEnd - wallStart).count();

    std::cout << "Total wall time: " << wallMs / 1000.0 << " s\n";
    std::cout << "Event count:     " << Simulator::GetEventCount() << "\n";

    Simulator::Destroy();
    return 0;
}