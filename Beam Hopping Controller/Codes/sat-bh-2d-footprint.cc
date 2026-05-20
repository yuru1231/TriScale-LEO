/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-2d-footprint.cc
 *
 * 單顆 LEO 衛星對地投影 2D 場景 — 獨立幾何驗證
 *
 * 目的：
 *   在整合 BH 排程器之前，先驗證 footprint 對地投影可以被明確編號。
 *   本檔案與 BH 排程器完全獨立，不使用 SatBhHelper / SatBhScheduler /
 *   SatBhObc / SatResourceManager 等元件。
 *
 * 設計：
 *   footprint 圓形半徑 r_footprint → 最大內接正方形邊長 L = r * sqrt(2)
 *   將正方形分成 Nx 欄 × Ny 列；beam_id = row × Nx + col + 1（1-indexed）
 *   格子中心經緯度由直角座標轉換（與 Python utils.get_positions_in_lat_long_coordinates 一致）
 *
 * USAGE
 * -----
 *   步驟一：只驗證幾何（simTime=0 會跳過 SNS3 模擬）
 *     ./ns3 run "sat-bh-2d-footprint --Nx=4 --Ny=3 --rFootprint=300000"
 *
 *   步驟二：完整模擬（60 秒，Ka-band 25 MHz）
 *     ./ns3 run "sat-bh-2d-footprint --Nx=4 --Ny=3 --rFootprint=300000 --simTime=60" \
 *       2>&1 | tee footprint_run.log
 *
 *   步驟三：較大矩陣（5×4 = 20 beams）
 *     ./ns3 run "sat-bh-2d-footprint --Nx=5 --Ny=4 --rFootprint=300000 --simTime=60"
 *
 * 限制：
 *   Nx × Ny ≤ 72（Iridium-66 每顆衛星最多 72 個 beam slot）
 *
 * 輸出：
 *   footprint-geometry.csv  — SNS3 初始化之前寫出（幾何驗證用）
 *   footprint-results.csv   — 模擬結束後寫出（UT 位置對照用）
 *   data/sat-stats-per-ut-fwd-composite-sinr-scatter-*.dat — 每顆 UT 的 SINR
 */

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/satellite-module.h"
#include "ns3/traffic-module.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("sat-bh-2d-footprint");

// ─────────────────────────────────────────────────────────────────────────────
// FootprintConfig: 所有實驗參數集中管理，避免 hard-code
// ─────────────────────────────────────────────────────────────────────────────
struct FootprintConfig
{
    double   latCenter_deg  = 35.67619190;   // 中心緯度（東京，與 Python 框架一致）
    double   lonCenter_deg  = 139.65031060;  // 中心經度
    double   rFootprint_m   = 300000.0;      // footprint 半徑（公尺）
    uint32_t Nx             = 4;             // 欄數（東西方向）
    uint32_t Ny             = 3;             // 列數（南北方向）
    uint32_t satId          = 0;             // 衛星索引（0–65，Iridium-66）
    double   simTimeSec     = 60.0;          // 模擬時長（秒），0 = 只驗證幾何
    double   warmUpSec      = 5.0;           // 暖機時間（秒）
    std::string geoCsvFile    = "footprint-geometry.csv";
    std::string resultCsvFile = "footprint-results.csv";
};

// ─────────────────────────────────────────────────────────────────────────────
// BeamCell: 單一格子的完整資訊（beam ID、直角座標、經緯度）
// ─────────────────────────────────────────────────────────────────────────────
struct BeamCell
{
    uint32_t beamId;   // 1-indexed: row × Nx + col + 1
    uint32_t row;      // 0-indexed（row 0 = 最南列）
    uint32_t col;      // 0-indexed（col 0 = 最西欄）
    double   cx_m;     // footprint 直角座標 x（東，公尺）
    double   cy_m;     // footprint 直角座標 y（北，公尺）
    double   lat_deg;  // 格子中心緯度
    double   lon_deg;  // 格子中心經度
};

// ─────────────────────────────────────────────────────────────────────────────
// ParseConfig: 將所有 FootprintConfig 欄位對應到 CommandLine 參數
// ─────────────────────────────────────────────────────────────────────────────
static FootprintConfig
ParseConfig(int argc, char* argv[])
{
    FootprintConfig cfg;
    CommandLine cmd;

    cmd.AddValue("latCenter",   "Footprint center latitude [deg]",   cfg.latCenter_deg);
    cmd.AddValue("lonCenter",   "Footprint center longitude [deg]",  cfg.lonCenter_deg);
    cmd.AddValue("rFootprint",  "Footprint circle radius [m]",       cfg.rFootprint_m);
    cmd.AddValue("Nx",          "Number of columns (East-West)",      cfg.Nx);
    cmd.AddValue("Ny",          "Number of rows (North-South)",       cfg.Ny);
    cmd.AddValue("satId",       "Satellite index (0-65 for Iridium)", cfg.satId);
    cmd.AddValue("simTime",     "Simulation duration [s] (0=geometry only)", cfg.simTimeSec);
    cmd.AddValue("warmUp",      "Warm-up duration [s]",              cfg.warmUpSec);
    cmd.AddValue("geoCsv",      "Geometry CSV output filename",       cfg.geoCsvFile);
    cmd.AddValue("resultCsv",   "Results CSV output filename",        cfg.resultCsvFile);

    cmd.Parse(argc, argv);
    return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
// ComputeGrid: 計算所有格子的 beam_id、直角座標、經緯度
//
// 步驟：
//   1. 最大內接正方形邊長 L = r_footprint × sqrt(2)
//   2. Nx × Ny 格子（row=0 為最南，col=0 為最西）
//   3. beam_id = row × Nx + col + 1
//   4. 直角座標（公尺）→ 經緯度（與 Python utils 公式一致）
// ─────────────────────────────────────────────────────────────────────────────
static std::vector<BeamCell>
ComputeGrid(const FootprintConfig& cfg)
{
    const double R_earth = 6371000.0;  // 地球半徑（公尺），與 Python params.py 一致

    // 最大內接正方形邊長
    const double L = cfg.rFootprint_m * std::sqrt(2.0);

    // 格子寬高（公尺）
    const double cellW = L / cfg.Nx;
    const double cellH = L / cfg.Ny;

    // 中心點緯度（弧度）—— 用於 Δlon 計算
    const double latC_rad = cfg.latCenter_deg * M_PI / 180.0;

    std::vector<BeamCell> cells;
    cells.reserve(cfg.Nx * cfg.Ny);

    for (uint32_t row = 0; row < cfg.Ny; ++row)
    {
        for (uint32_t col = 0; col < cfg.Nx; ++col)
        {
            BeamCell c;
            c.beamId = row * cfg.Nx + col + 1;
            c.row    = row;
            c.col    = col;

            // footprint 直角座標中心（公尺）
            //   x = 東方向，y = 北方向，原點 = footprint 中心
            c.cx_m = -L / 2.0 + (col + 0.5) * cellW;
            c.cy_m = -L / 2.0 + (row + 0.5) * cellH;

            // 直角座標 → 經緯度（弧度差 → 度數差）
            double deltaLat_rad = c.cy_m / R_earth;
            double deltaLon_rad = c.cx_m / (R_earth * std::cos(latC_rad));

            c.lat_deg = cfg.latCenter_deg + deltaLat_rad * (180.0 / M_PI);
            c.lon_deg = cfg.lonCenter_deg + deltaLon_rad * (180.0 / M_PI);

            cells.push_back(c);
        }
    }

    return cells;
}

// ─────────────────────────────────────────────────────────────────────────────
// PrintMatrix: SNS3 初始化之前，在終端機列印格子矩陣供人工驗證
//   列印順序：由北（高列）往南（低列）、由西往東，符合視覺直觀
// ─────────────────────────────────────────────────────────────────────────────
static void
PrintMatrix(const std::vector<BeamCell>& cells, const FootprintConfig& cfg)
{
    const double L = cfg.rFootprint_m * std::sqrt(2.0);

    std::cout << "\n[Footprint Matrix]"
              << "  satId=" << cfg.satId
              << "  Nx=" << cfg.Nx
              << "  Ny=" << cfg.Ny
              << "  rFootprint=" << cfg.rFootprint_m << " m"
              << "  L=" << std::fixed << std::setprecision(1) << L << " m"
              << "\n";

    std::cout << std::setw(8)  << "beam_id"
              << std::setw(6)  << "row"
              << std::setw(6)  << "col"
              << std::setw(12) << "lat_deg"
              << std::setw(13) << "lon_deg"
              << std::setw(12) << "cx_m"
              << std::setw(12) << "cy_m"
              << "\n";
    std::cout << std::string(69, '-') << "\n";

    // 由北往南列印（row 由高到低）
    for (int32_t row = static_cast<int32_t>(cfg.Ny) - 1; row >= 0; --row)
    {
        for (uint32_t col = 0; col < cfg.Nx; ++col)
        {
            const BeamCell& c = cells[row * cfg.Nx + col];
            std::cout << std::setw(8)  << c.beamId
                      << std::setw(6)  << c.row
                      << std::setw(6)  << c.col
                      << std::setw(12) << std::fixed << std::setprecision(5) << c.lat_deg
                      << std::setw(13) << std::fixed << std::setprecision(5) << c.lon_deg
                      << std::setw(12) << std::fixed << std::setprecision(0) << c.cx_m
                      << std::setw(12) << std::fixed << std::setprecision(0) << c.cy_m
                      << "\n";
        }
        if (row > 0)
            std::cout << "\n";  // 每列之間空一行，方便閱讀
    }
    std::cout << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// ExportGeometryCsv: SNS3 初始化之前寫出幾何 CSV
//   欄位：beam_id, row, col, lat_deg, lon_deg, cx_m, cy_m
// ─────────────────────────────────────────────────────────────────────────────
static void
ExportGeometryCsv(const std::vector<BeamCell>& cells, const std::string& path)
{
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        std::cerr << "[ExportGeometryCsv] ERROR: cannot open " << path << "\n";
        return;
    }

    ofs << "beam_id,row,col,lat_deg,lon_deg,cx_m,cy_m\n";
    for (const auto& c : cells)
    {
        ofs << c.beamId << ","
            << c.row    << ","
            << c.col    << ","
            << std::fixed << std::setprecision(6) << c.lat_deg << ","
            << std::fixed << std::setprecision(6) << c.lon_deg << ","
            << std::fixed << std::setprecision(1) << c.cx_m    << ","
            << std::fixed << std::setprecision(1) << c.cy_m    << "\n";
    }

    std::cout << "[ExportGeometryCsv] Written: " << path
              << "  (" << cells.size() << " rows)\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// ExportResultsCsv: 模擬結束後，輸出格子設計中心（供對照驗證）
//   注意：SatBeamUserInfo(1,1) 讓 SNS3 自動放置 UT，實際位置可能略偏離中心。
//         本 CSV 記錄的是「設計中心座標」，實際 UT 位置在 sat-stats scatter 檔中。
// ─────────────────────────────────────────────────────────────────────────────
static void
ExportResultsCsv(const std::vector<BeamCell>& cells,
                 const FootprintConfig&        cfg,
                 const std::string&            path)
{
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        std::cerr << "[ExportResultsCsv] ERROR: cannot open " << path << "\n";
        return;
    }

    ofs << "beam_id,row,col,sat_id,lat_deg,lon_deg,cx_m,cy_m\n";
    for (const auto& c : cells)
    {
        ofs << c.beamId  << ","
            << c.row     << ","
            << c.col     << ","
            << cfg.satId << ","
            << std::fixed << std::setprecision(6) << c.lat_deg << ","
            << std::fixed << std::setprecision(6) << c.lon_deg << ","
            << std::fixed << std::setprecision(1) << c.cx_m    << ","
            << std::fixed << std::setprecision(1) << c.cy_m    << "\n";
    }

    std::cout << "[ExportResultsCsv] Written: " << path
              << "  (" << cells.size() << " rows)\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int
main(int argc, char* argv[])
{
    // ── 1. 解析參數 ──────────────────────────────────────────────────────────
    FootprintConfig cfg = ParseConfig(argc, argv);

    // ── 2. 幾何計算 ──────────────────────────────────────────────────────────
    std::vector<BeamCell> cells = ComputeGrid(cfg);

    // ── 3. 列印矩陣（SNS3 初始化之前，供人工驗證）────────────────────────────
    PrintMatrix(cells, cfg);

    // ── 4. 匯出幾何 CSV（SNS3 初始化之前）───────────────────────────────────
    ExportGeometryCsv(cells, cfg.geoCsvFile);

    // ── 5. 邊界條件檢查 ──────────────────────────────────────────────────────
    //   Iridium-66 每顆衛星最多 72 個 beam slot（beam ID 1–72）
    NS_ASSERT_MSG(cfg.Nx * cfg.Ny <= 72,
                  "Nx × Ny must not exceed 72 (Iridium-66 beam limit)");
    NS_ASSERT_MSG(cfg.satId <= 65,
                  "satId must be 0–65 for Iridium-66 constellation");

    std::cout << "[sat-bh-2d-footprint] Geometry verified: "
              << cfg.Nx << "×" << cfg.Ny << " = " << cfg.Nx * cfg.Ny << " beams"
              << "  satId=" << cfg.satId
              << "\n";

    // simTime=0 → 只驗證幾何，不跑 SNS3
    if (cfg.simTimeSec <= 0.0)
    {
        std::cout << "[sat-bh-2d-footprint] simTime=0 → geometry-only mode, exiting.\n";
        return 0;
    }

    // ── 6. SNS3 全域設定 ──────────────────────────────────────────────────────
    // Regeneration mode（與 sat-bh-example.cc 一致）
    Config::SetDefault("ns3::SatConf::ForwardLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));
    Config::SetDefault("ns3::SatConf::ReturnLinkRegenerationMode",
                       EnumValue(SatEnums::REGENERATION_NETWORK));

    // 本場景為靜態幾何驗證，不需要 handover
    Config::SetDefault("ns3::SatHelper::HandoversEnabled", BooleanValue(false));

    // 允許覆寫輸出目錄，避免每次都需要手動清除
    Config::SetDefault("ns3::SatEnvVariables::EnableSimulationOutputOverwrite",
                       BooleanValue(true));

    // Ka-band 頻寬覆寫（SNS3 預設已是 Ka-band；只需覆寫頻寬為 25 MHz，與 Python 框架一致）
    Config::SetDefault("ns3::SatConf::FwdUserLinkBandwidth",     DoubleValue(25.0e6));
    Config::SetDefault("ns3::SatConf::RtnUserLinkBandwidth",     DoubleValue(25.0e6));
    // 將前向鏈路基頻對齊到 30 GHz（Ka-band，與 params.py 中心頻率一致）
    Config::SetDefault("ns3::SatConf::FwdUserLinkBaseFrequency", DoubleValue(30.0e9));

    // ── 7. 建立 SimulationHelper ──────────────────────────────────────────────
    Ptr<SimulationHelper> simHelper = CreateObject<SimulationHelper>("sat-bh-2d-footprint");
    simHelper->SetSimulationTime(Seconds(cfg.simTimeSec));
    simHelper->SetGwUserCount(1);
    simHelper->SetUserCountPerUt(1);

    // ── 8. 載入 Iridium-66 constellation ─────────────────────────────────────
    simHelper->LoadScenario("constellation-iridium-66-sats");

    // ── 9. 建立 BeamUserInfoMap：每個格子對應一顆 UT ─────────────────────────
    //   {satId, beamId} → SatBeamUserInfo(utCount=1, userPerUt=1)
    //   所有格子都掛在同一顆衛星（cfg.satId）上
    SatHelper::BeamUserInfoMap_t beamInfos;
    std::set<uint32_t>           beamSet;

    for (const auto& c : cells)
    {
        beamSet.insert(c.beamId);
        beamInfos[{cfg.satId, c.beamId}] = SatBeamUserInfo(1, 1);
    }

    simHelper->SetBeamUserInfo(beamInfos);  // 指定每顆 UT 掛在哪個 beam
    simHelper->SetBeamSet(beamSet);         // 必須呼叫，否則 SetBeamUserInfo 無效

    // ── 10. 建立 SNS3 場景 ────────────────────────────────────────────────────
    simHelper->CreateSatScenario(SatHelper::NONE);

    std::cout << "\n[Topology]\n";
    Singleton<SatTopology>::Get()->PrintTopology(std::cout);

    // ── 11. 設定 FWD link CBR 流量 ───────────────────────────────────────────
    //   最小流量：觸發 link 建立，讓 SINR 統計可以被收集
    simHelper->GetTrafficHelper()->AddCbrTraffic(
        SatTrafficHelper::FWD_LINK,
        SatTrafficHelper::UDP,
        MilliSeconds(20),                    // 間隔 20 ms → ~50 pkt/s
        1500,                                // packet size（bytes）
        NodeContainer(Singleton<SatTopology>::Get()->GetGwUserNode(0)),
        Singleton<SatTopology>::Get()->GetUtUserNodes(),
        Seconds(cfg.warmUpSec),              // 暖機後才開始
        Seconds(cfg.simTimeSec),
        Seconds(0));

    // ── 12. 設定統計收集器 ───────────────────────────────────────────────────
    //   收集每顆 UT 的 FWD composite SINR（scatter 格式，後處理用）
    Ptr<SatStatsHelperContainer> stats = simHelper->GetStatisticsContainer();
    stats->AddPerUtFwdCompositeSinr(SatStatsHelper::OUTPUT_SCATTER_FILE);

    // 補充：吞吐量統計（per-beam / per-sat）供整體驗證
    stats->AddPerBeamFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);
    stats->AddPerSatFwdAppThroughput(SatStatsHelper::OUTPUT_SCATTER_FILE);

    // ── 13. 執行模擬 ─────────────────────────────────────────────────────────
    std::cout << "\n[sat-bh-2d-footprint] Starting simulation"
              << "  time=" << cfg.simTimeSec << "s"
              << "  warmUp=" << cfg.warmUpSec << "s"
              << "  satId=" << cfg.satId
              << "  beams=" << cfg.Nx * cfg.Ny
              << "  Nx=" << cfg.Nx
              << "  Ny=" << cfg.Ny
              << "\n\n";

    simHelper->EnableProgressLogs();
    simHelper->RunSimulation();

    // ── 14. 匯出結果 CSV ─────────────────────────────────────────────────────
    ExportResultsCsv(cells, cfg, cfg.resultCsvFile);

    std::cout << "\n[sat-bh-2d-footprint] Simulation complete.\n"
              << "  Geometry CSV : " << cfg.geoCsvFile    << "\n"
              << "  Results CSV  : " << cfg.resultCsvFile << "\n"
              << "  SINR scatter : data/sat-stats-per-ut-fwd-composite-sinr-scatter-*.dat\n"
              << "  Throughput   : data/sat-stats-per-beam-fwd-app-throughput-scatter-*.dat\n\n";

    return 0;
}
