/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-simulation.cc  [orbit-sgp4 / Phase 3]
 *
 * Entry point: scans 66 Iridium-NEXT satellites over a fixed ROI and
 * reports per-cell SNR for each qualifying pass.
 *
 * Flow:
 *   1. Parse command-line arguments into SimConfig + scan parameters.
 *   2. CreateObject<SatTleReader>  → Load(tles, fwdConf)
 *   3. CreateObject<SatConstellationScanner> → SetTleReader → Run()
 *   4. Outputs: constellation_status.json + sat_XXXXX_cells.csv
 *
 * Run command:
 *   ./ns3 run "sat-multi-beam-simulation \
 *     --constellation-dir=contrib/satellite/data/scenarios/constellation-iridium-next-66-sats \
 *     --lat=35.676 --lon=139.65 --d=5 \
 *     --window-s=3600 --dt-screen-s=10 --dt-snr-s=1 \
 *     --min-elevation-deg=5 \
 *     --out-dir=scratch/constellation_out" 2>&1 | tee constellation.log
 *
 * CMakeLists.txt SOURCE_FILES:
 *   sat-multi-beam-geometry.cc   sat-multi-beam-channel.cc
 *   sat-roi-grid.cc
 *   sat-tle-reader.cc            sat-constellation-scanner.cc
 *   sat-antenna-pattern-reader.cc
 *   sgp4unit.cpp  sgp4ext.cpp  sgp4io.cpp
 *   (copy sgp4*.cpp/h from hypatia-master/.../model/ to scratch)
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
    // Logging — enable info-level output for all satellite components
    // -----------------------------------------------------------------------
    LogComponentEnable("SatTleReader",             LOG_LEVEL_INFO);
    LogComponentEnable("SatConstellationScanner",  LOG_LEVEL_INFO);
    LogComponentEnable("SatAntennaPatternReader",  LOG_LEVEL_WARN);
    LogComponentEnable("SatMultiBeamGeometry",     LOG_LEVEL_WARN);

    // -----------------------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------------------
    SimConfig cfg;

    std::string constellationDir;
    std::string outDir{"scratch/constellation_out"};
    int         gridD{5};
    double      windowS{3600.0};
    double      dtScreenS{10.0};
    double      dtSnrS{1.0};
    double      minElevDeg{5.0};
    uint32_t    seed{42};

    CommandLine cmd(__FILE__);
    cmd.AddValue("constellation-dir",
                 "Path to constellation folder containing positions/tles.txt "
                 "and beams/fwdConf.txt",
                 constellationDir);
    cmd.AddValue("lat",
                 "ROI centre latitude (degrees N). Default: 35.676 (Tokyo)",
                 cfg.latitudeCenterDeg);
    cmd.AddValue("lon",
                 "ROI centre longitude (degrees E). Default: 139.650 (Tokyo)",
                 cfg.longitudeCenterDeg);
    cmd.AddValue("d",
                 "ROI grid dimension (d x d). Default: 5",
                 gridD);
    cmd.AddValue("window-s",
                 "Scan time window (seconds). Default: 3600",
                 windowS);
    cmd.AddValue("dt-screen-s",
                 "Coarse elevation screen time step (seconds). Default: 10",
                 dtScreenS);
    cmd.AddValue("dt-snr-s",
                 "Fine SNR computation time step (seconds). Default: 1",
                 dtSnrS);
    cmd.AddValue("min-elevation-deg",
                 "Elevation threshold (degrees). Passes below this are skipped. Default: 5",
                 minElevDeg);
    cmd.AddValue("r-footprint",
                 "Satellite footprint radius (m). Default: 100000",
                 cfg.rFootprintM);
    cmd.AddValue("h-km",
                 "Satellite altitude (km). Default: 780",
                 cfg.hSatelliteM);
    cmd.AddValue("tx-power",
                 "Transmit power (W). Default: 63",
                 cfg.transmitPowerW);
    cmd.AddValue("out-dir",
                 "Output directory. Default: scratch/constellation_out",
                 outDir);
    cmd.AddValue("seed", "RNG seed. Default: 42", seed);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(constellationDir.empty(),
                    "--constellation-dir is required\n"
                    "  Example: --constellation-dir=contrib/satellite/data/scenarios/constellation-iridium-next-66-sats");
    NS_ABORT_MSG_IF(gridD < 1, "--d must be >= 1");

    // Allow altitude input in km (values <= 2000 are treated as km)
    if (cfg.hSatelliteM <= 2000.0)
    {
        cfg.hSatelliteM *= 1000.0;
    }

    std::filesystem::create_directories(outDir);

    // -----------------------------------------------------------------------
    // Execute
    // -----------------------------------------------------------------------
    const std::string tlesPath    = constellationDir + "/positions/tles.txt";
    const std::string fwdConfPath = constellationDir + "/beams/fwdConf.txt";

    NS_LOG_UNCOND("[constellation scan]");
    NS_LOG_UNCOND("  tles       : " << tlesPath);
    NS_LOG_UNCOND("  fwdConf    : " << fwdConfPath);
    NS_LOG_UNCOND("  ROI        : lat=" << cfg.latitudeCenterDeg
                  << " lon=" << cfg.longitudeCenterDeg);
    NS_LOG_UNCOND("  grid       : " << gridD << "x" << gridD);
    NS_LOG_UNCOND("  window     : " << windowS    << " s");
    NS_LOG_UNCOND("  dt_screen  : " << dtScreenS  << " s");
    NS_LOG_UNCOND("  dt_snr     : " << dtSnrS     << " s");
    NS_LOG_UNCOND("  min_elev   : " << minElevDeg << " deg");
    NS_LOG_UNCOND("  out        : " << outDir);

    // Instantiate via ns3 object system (Ptr<T> / CreateObject<T>)
    Ptr<SatTleReader> tleReader = CreateObject<SatTleReader>();
    tleReader->Load(tlesPath, fwdConfPath);

    Ptr<SatConstellationScanner> scanner = CreateObject<SatConstellationScanner>();
    scanner->SetTleReader(tleReader);

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

    scanner->Run(scanCfg);
    return 0;
}
