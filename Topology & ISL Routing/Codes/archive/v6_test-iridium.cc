// v6_test-iridium.cc
//
// v6 相較 v5 的核心變更：
//   - 移除 FtVisibilityFilter（其功能已整合至 IslRoutingManager）
//   - 改用 IslRoutingManager 內建的 GW routing API：
//       AddGateway / AddGwPair / SetGwElevationThreshold / PrecomputeGwRoutes
//   - PrintGwRouteReport：輸出所有 slot 的 GW-to-GW 完整路由，
//     含 entry sat、完整 ISL 路徑、exit sat、isl cost，並標記路由變化
//
// Fixed constants:
//   scenarioName = "constellation-iridium-66-sats-fixed"
//   ns3BasePath  = "/home/wenj/workspace/ns-3.43"
//   numSats      = 66
//   GW positions = Taipei / Tokyo / SanFrancisco（合約地面站）
//
// 所有可調整參數均透過 CommandLine 暴露。

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"
#include "ns3/isl-graph.h"           // v6：包含 GwDef / GwToGwRoute / GW routing API
#include "ns3/beam-hopping-manager.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

using namespace ns3;

// ── Layer 3: QoS 設定 ─────────────────────────────────────────────────────
// 必須在 CreateSatScenario() 之前呼叫。
// Class A (CRA): 保證速率，最高優先
// Class B (RBDC): 彈性速率，中優先
// Class C (VBDC): 盡力轉送，最低優先
//
// TODO SNS3_QOS_ATTR: 在正式啟用前，需先驗證 Config key 字串是否與實際安裝的
// SNS3 版本一致（key 錯誤會觸發 NS_FATAL）。驗證方式：
//   grep -r "AddAttribute" contrib/satellite/model/satellite-lower-layer-service.cc \
//        | grep -i "daservice"
static void
ConfigureQoS()
{
    // Class A – Constant Rate Assignment（VoIP / 控制流量）
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService0_ConstantAssignmentProvided",
    //                    BooleanValue(true));
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService0_ConstantServiceRateStream0",
    //                    UintegerValue(50)); // 50 kbps 保證頻寬

    // Class B – Rate Based Dynamic Capacity（串流）
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService1_RbdcAllowed",
    //                    BooleanValue(true));
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService1_MaximumServiceRateStream0",
    //                    UintegerValue(2000)); // 2 Mbps 上限

    // Class C – Volume Based Dynamic Capacity（盡力轉送）
    // Config::SetDefault("ns3::SatLowerLayerServiceConf::DaService2_VbdcAllowed",
    //                    BooleanValue(true));
}

// ── Main ──────────────────────────────────────────────────────────────────

int
main(int argc, char* argv[])
{
    // ── Fixed constants ───────────────────────────────────────────────────
    // 代表特定部署環境與合約地面站，模擬執行間不預期變動。
    const std::string ns3BasePath   = "/home/wenj/workspace/ns-3.43";
    const std::string scenarioName  = "constellation-iridium-66-sats-fixed";
    const uint32_t    numSats       = 66;

    const std::string islsFilePath  =
        ns3BasePath + "/contrib/satellite/data/scenarios/"
        + scenarioName + "/positions/isls.txt";

    // ── CommandLine parameters ────────────────────────────────────────────
    double   simTime         = 630.0;  // 模擬總時長（秒）
    double   slotInterval    = 60.0;   // routing / BH slot 間隔（秒）
    uint32_t beamId          = 1;      // SNS3 beam ID

    // Layer 1 – ISL routing 可調參數
    double   islMaxDistKm    = 5000.0; // ISL 啟用距離門檻（km）
    double   islRateMbps     = 10.0;   // ISL link rate（Mbps）
    double   emaAlpha        = 0.3;    // load cost EMA 更新權重
    double   changeThresh    = 0.1;    // 觸發局部重算的 load cost 變化比例
    double   cooldownRatio   = 0.5;    // cooldownSeconds = slotInterval * cooldownRatio
    double   elevMinDeg      = 5.0;    // GW 可見衛星最低仰角（度）

    // Layer 2 – Beam Hopping
    double   bhSuperframeSec = 0.25;   // BH superframe 長度（秒）

    CommandLine cmd;
    cmd.AddValue("simTime",         "Simulation duration (s)",                 simTime);
    cmd.AddValue("slotInterval",    "Routing and BH slot interval (s)",        slotInterval);
    cmd.AddValue("beamId",          "SNS3 beam ID to activate",                beamId);
    cmd.AddValue("islMaxDistKm",    "ISL activation distance threshold (km)",  islMaxDistKm);
    cmd.AddValue("islRateMbps",     "ISL link rate (Mbps)",                    islRateMbps);
    cmd.AddValue("emaAlpha",        "EMA weight for load-cost smoothing",      emaAlpha);
    cmd.AddValue("changeThresh",    "Load-cost change ratio for recompute",    changeThresh);
    cmd.AddValue("cooldownRatio",   "Cooldown = slotInterval * cooldownRatio", cooldownRatio);
    cmd.AddValue("elevMinDeg",      "Minimum GW elevation angle (deg)",        elevMinDeg);
    cmd.AddValue("bhSuperframeSec", "BH superframe duration (s)",              bhSuperframeSec);
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

    // Layer 3：QoS class 設定（必須在 CreateSatScenario 前）
    ConfigureQoS();

    // ── Scenario setup ────────────────────────────────────────────────────
    Ptr<SimulationHelper> simHelper =
        CreateObject<SimulationHelper>("test-iridium-v6");
    simHelper->LoadScenario(scenarioName);
    simHelper->SetSimulationTime(Seconds(simTime));

    // TODO BH: 待確認 beam ID 後擴展到台灣覆蓋範圍
    simHelper->SetBeamSet({beamId});

    simHelper->SetUserCountPerUt(1);
    simHelper->CreateSatScenario();

    auto wallStart = std::chrono::steady_clock::now();

    // ── Layer 1: ISL Routing ──────────────────────────────────────────────
    // SetAttribute 必須在 Initialize() 前呼叫
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

    // v6：在 Initialize 前設定 GW 定義（不依賴 NS3 場景）
    // 合約地面站：台北、東京、舊金山
    routingMgr->SetGwElevationThreshold(elevMinDeg);
    routingMgr->AddGateway(0, 25.0,  121.5, "TW-Taipei");       // 台灣
    routingMgr->AddGateway(1, 35.7,  139.7, "JP-Tokyo");        // 日本
    routingMgr->AddGateway(2, 37.8, -122.4, "US-SanFrancisco"); // 美國西岸

    // 合約路由對（雙向）
    routingMgr->AddGwPair(0, 1); // Taiwan ↔ Japan
    routingMgr->AddGwPair(0, 2); // Taiwan ↔ US

    routingMgr->Initialize(islsFilePath);
    routingMgr->PrecomputeAllTables();

    // ── Layer 1 v6：GW-to-GW 路由預計算與報告 ────────────────────────────
    //
    // 必須在 PrecomputeAllTables() 之後呼叫。
    // 流程：
    //   1. 每個 slot 計算每個 GW 的可見衛星集合
    //   2. 對每個 contracted GW pair，枚舉 (entry, exit) 找最小 ISL cost
    //   3. TracePath(entry, exit) 重建完整衛星路徑
    //   4. 結果儲存於 m_gwRoutes[slot][(srcGwId, dstGwId)]
    routingMgr->PrecomputeGwRoutes();

    // 輸出 GW-to-GW 路由報告：
    //   正向（TW→JP、TW→US）與反向各自顯示
    //   欄位：slot | time(s) | entry_sat | ISL_path | exit_sat | isl_cost(s)
    //   路徑與前一 slot 不同時標記 <ROUTE CHANGED>
    routingMgr->PrintGwRouteReport();

    routingMgr->ScheduleRoutingUpdates();

    // ── Layer 2: Beam Hopping Manager（stub）─────────────────────────────
    // BH 邏輯獨立於 ISL routing，作為背景 stub 驗證不干擾 Layer 1
    Ptr<BeamHoppingManager> bhMgr = CreateObject<BeamHoppingManager>();
    bhMgr->SetAttribute("NumSatellites",         UintegerValue(numSats));
    bhMgr->SetAttribute("SuperframeDurationSec", DoubleValue(bhSuperframeSec));
    bhMgr->SetAttribute("ElevationThresholdDeg", DoubleValue(elevMinDeg));

    // 目標區域：台灣主要城市
    bhMgr->AddCell(0, 25.0, 121.5, "Taipei");
    bhMgr->AddCell(1, 24.1, 120.7, "Taichung");
    bhMgr->AddCell(2, 22.6, 120.3, "Kaohsiung");
    bhMgr->AddCell(3, 24.8, 121.0, "Hsinchu");

    // 均勻需求 stub（正式 Layer 2 工作中替換為真實 provider）
    bhMgr->SetDemandProvider(std::make_shared<UniformDemandProvider>(1.0));

    // SNS3 注入 callback（stub，僅 logging）
    // TODO SNS3_BH_INJECT: 替換為實際 beam switch API
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
