/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-simulation.cc  [orbit-sgp4 / Phase 3]
 *
 * 單一用途入口：對固定 ROI 掃描 Iridium-NEXT 66 顆衛星的覆蓋與 SNR。
 * 工作邏輯已驗證正確，不需其他模式（Phase 1/2 模式保留在 phase2/code/ 歷史版本）。
 *
 * 流程：
 *   1. 讀取 tles.txt + fwdConf.txt，初始化 66 顆衛星的 SGP4 record
 *   2. 對每顆衛星做粗篩（elevation > minElevDeg）
 *   3. 通過篩選的衛星做細算：per-cell SNR（dtSnrS 間隔）
 *   4. 輸出 constellation_status.json + sat_XXXXX_cells.csv
 *
 * 執行指令：
 *   ./ns3 run "sat-multi-beam-simulation \
 *     --constellation-dir=scratch/constellation-iridium-next-66-sats \
 *     --lat=35.676 --lon=139.65 --d=5 \
 *     --window-s=3600 --dt-screen-s=10 --dt-snr-s=1 \
 *     --min-elevation-deg=5 \
 *     --out-dir=scratch/constellation_out" 2>&1 | tee constellation.log
 *
 * CMakeLists.txt SOURCE_FILES:
 *   sat-multi-beam-geometry.cc   sat-multi-beam-channel.cc
 *   sat-roi-grid.cc
 *   sat-tle-reader.cc            sat-constellation-scanner.cc
 *   sgp4unit.cpp  sgp4ext.cpp  sgp4io.cpp
 *   (從 hypatia-master/.../model/ 複製 sgp4*.cpp/h 到 scratch)
 */

#include "sat-multi-beam-config.h"
#include "sat-multi-beam-geometry.h"
#include "sat-tle-reader.h"
#include "sat-constellation-scanner.h"

#include "ns3/core-module.h"

#include <filesystem>
#include <iostream>
#include <string>

using namespace ns3;

int
main(int argc, char* argv[])
{
    // -----------------------------------------------------------------------
    // 參數
    // -----------------------------------------------------------------------
    SimConfig cfg;

    std::string constellationDir{""};
    std::string outDir{"scratch/constellation_out"};
    int         gridD{5};
    double      windowS{3600.0};
    double      dtScreenS{10.0};
    double      dtSnrS{1.0};
    double      minElevDeg{5.0};
    uint32_t    seed{42};

    CommandLine cmd(__FILE__);
    cmd.AddValue("constellation-dir",
                 "constellation 資料夾路徑，須含 positions/tles.txt 與 beams/fwdConf.txt",
                 constellationDir);
    cmd.AddValue("lat",
                 "ROI 中心緯度（degrees N）。預設：35.676（東京）",
                 cfg.latitudeCenterDeg);
    cmd.AddValue("lon",
                 "ROI 中心經度（degrees E）。預設：139.650（東京）",
                 cfg.longitudeCenterDeg);
    cmd.AddValue("d",
                 "觀測格點維度（d×d）。預設：5。獨立於 beam 數，代表 ROI 內觀測位置數",
                 gridD);
    // ---- beam configuration ------------------------------------------------
    // n-beams-x × n-beams-y = 同時開啟的 beam 數目（along-track × cross-track）。
    //   1×1 = 1 beam（BH 通道特性化基準，SINR = SNR，無 inter-beam 干擾）
    //   3×3 = 9 beams（中規模 BH 驗證）
    //   5×5 = 25 beams（全覆蓋模式，每格一個 beam）
    // cfg.nBeams 由 nBeamsX × nBeamsY 自動計算，不需手動設定。
    cmd.AddValue("n-beams-x",
                 "Along-track beam 數（1 = BH 基準，5 = 全覆蓋）。預設：1",
                 cfg.nBeamsX);
    cmd.AddValue("n-beams-y",
                 "Cross-track beam 數（1 = BH 基準，5 = 全覆蓋）。預設：1",
                 cfg.nBeamsY);
    // ------------------------------------------------------------------------
    cmd.AddValue("window-s",
                 "掃描時間窗口（秒）。預設：3600",
                 windowS);
    cmd.AddValue("dt-screen-s",
                 "粗篩仰角時間步長（秒）。預設：10",
                 dtScreenS);
    cmd.AddValue("dt-snr-s",
                 "細算 SNR 時間步長（秒）。預設：1",
                 dtSnrS);
    cmd.AddValue("min-elevation-deg",
                 "仰角門檻（度）。低於此值跳過。預設：5",
                 minElevDeg);
    cmd.AddValue("r-footprint",
                 "衛星覆蓋半徑 override（m）。0 = 依 nBeamsX 自動計算。預設：0",
                 cfg.rFootprintM);
    cmd.AddValue("h-km",
                 "衛星高度（km）。預設：634",
                 cfg.hSatelliteM);
    cmd.AddValue("tx-power",
                 "發射功率（W）。預設：63",
                 cfg.transmitPowerW);
    cmd.AddValue("out-dir",
                 "輸出目錄。預設：scratch/constellation_out",
                 outDir);
    cmd.AddValue("seed", "RNG 種子。預設：42", seed);
    cmd.Parse(argc, argv);

    // ---- validation --------------------------------------------------------
    NS_ABORT_MSG_IF(constellationDir.empty(),
                    "--constellation-dir 為必要參數\n"
                    "  範例: --constellation-dir=scratch/constellation-iridium-next-66-sats");
    NS_ABORT_MSG_IF(gridD < 1,        "--d 必須 >= 1");
    NS_ABORT_MSG_IF(cfg.nBeamsX < 1,  "--n-beams-x 必須 >= 1");
    NS_ABORT_MSG_IF(cfg.nBeamsY < 1,  "--n-beams-y 必須 >= 1");
    NS_ABORT_MSG_IF(cfg.nBeamsX > gridD || cfg.nBeamsY > gridD,
                    "--n-beams-x/y 不可超過 --d（beam 數不可多於觀測格點數）");

    // nBeams 由 nBeamsX × nBeamsY 自動派生，確保三者一致
    cfg.nBeams = cfg.nBeamsX * cfg.nBeamsY;

    // hSatelliteM 允許以 km 輸入
    if (cfg.hSatelliteM <= 2000.0)
    {
        cfg.hSatelliteM *= 1000.0;
    }

    std::filesystem::create_directories(outDir);

    // -----------------------------------------------------------------------
    // 執行
    // -----------------------------------------------------------------------
    const std::string tlesPath    = constellationDir + "/positions/tles.txt";
    const std::string fwdConfPath = constellationDir + "/beams/fwdConf.txt";

    // Pre-compute grid footprint radius for display (uses gridD as nBeamsX/Y)
    SimConfig gridDisplayCfg = cfg;
    gridDisplayCfg.nBeamsX   = gridD;
    gridDisplayCfg.nBeamsY   = gridD;
    const int beamFootprintKm = static_cast<int>(cfg.GetFootprintRadius()              / 1e3);
    const int gridFootprintKm = static_cast<int>(gridDisplayCfg.GetFootprintRadius()   / 1e3);

    std::cout << "[constellation scan]\n"
              << "  tles       : " << tlesPath     << "\n"
              << "  fwdConf    : " << fwdConfPath  << "\n"
              << "  ROI        : lat=" << cfg.latitudeCenterDeg
              <<                " lon=" << cfg.longitudeCenterDeg << "\n"
              << "  beams      : " << cfg.nBeamsX << "x" << cfg.nBeamsY
              <<                   " = " << cfg.nBeams << " beam(s)"
              <<                   "  footprint_r~" << beamFootprintKm << " km"
              <<                   "  [1x1=BH baseline / 5x5=full cover]\n"
              << "  obs grid   : " << gridD << "x" << gridD
              <<                   " = " << gridD * gridD << " cells"
              <<                   "  footprint_r~" << gridFootprintKm << " km\n"
              << "  window     : " << windowS    << " s\n"
              << "  dt_screen  : " << dtScreenS  << " s\n"
              << "  dt_snr     : " << dtSnrS     << " s\n"
              << "  min_elev   : " << minElevDeg << " deg\n"
              << "  out        : " << outDir     << "\n\n";

    SatTleReader            tleReader(tlesPath, fwdConfPath);
    SatConstellationScanner scanner(tleReader);

    ConstellationScanConfig scanCfg;
    scanCfg.cfg        = cfg;
    scanCfg.roiLatDeg  = cfg.latitudeCenterDeg;
    scanCfg.roiLonDeg  = cfg.longitudeCenterDeg;
    scanCfg.gridD      = gridD;
    scanCfg.windowS    = windowS;
    scanCfg.dtScreenS  = dtScreenS;
    scanCfg.dtSnrS     = dtSnrS;
    scanCfg.minElevDeg = minElevDeg;
    scanCfg.outDir     = outDir;
    scanCfg.seed       = seed;

    scanner.Run(scanCfg);
    return 0;
}
