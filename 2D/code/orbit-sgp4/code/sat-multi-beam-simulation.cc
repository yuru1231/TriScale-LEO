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
                 "ROI 格點維度（d×d）。預設：5",
                 gridD);
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
                 "衛星覆蓋半徑（m）。預設：100000",
                 cfg.rFootprintM);
    cmd.AddValue("h-km",
                 "衛星高度（km）。預設：780",
                 cfg.hSatelliteM);
    cmd.AddValue("tx-power",
                 "發射功率（W）。預設：63",
                 cfg.transmitPowerW);
    cmd.AddValue("out-dir",
                 "輸出目錄。預設：scratch/constellation_out",
                 outDir);
    cmd.AddValue("seed", "RNG 種子。預設：42", seed);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(constellationDir.empty(),
                    "--constellation-dir 為必要參數\n"
                    "  範例: --constellation-dir=scratch/constellation-iridium-next-66-sats");
    NS_ABORT_MSG_IF(gridD < 1, "--d 必須 >= 1");

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

    std::cout << "[constellation scan]\n"
              << "  tles       : " << tlesPath     << "\n"
              << "  fwdConf    : " << fwdConfPath  << "\n"
              << "  ROI        : lat=" << cfg.latitudeCenterDeg
              <<                " lon=" << cfg.longitudeCenterDeg << "\n"
              << "  grid       : " << gridD << "x" << gridD << "\n"
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
