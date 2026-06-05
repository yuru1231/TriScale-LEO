/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-simulation.cc
 *
 * [Historical record — arc mode preserved]
 * This file retains Phase 1.2.2 arc mode (analytical arc trajectory) for
 * experiment reproducibility.  The production version with arc mode removed
 * and only SGP4-orbit-driven modes (grid, dual) is in:
 *   2D/code/orbit-sgp4/sat-multi-beam-simulation.cc
 *
 * Multi-Beam LEO Communication Satellite Simulation — SNS3 C++ reproduction.
 *
 * Nadir-shadow design (redesigned from frame-based):
 *   At each simulation time T, the satellite sub-satellite point (nadir) is
 *   used as the footprint centre.  The local frame origin is always the nadir,
 *   and the satellite position in that frame is always {0, 0, h_satellite}.
 *   This mirrors the SNS3 create_basic_ground_station_for_satellite_shadow()
 *   ground station (gid=-1) which tracks the satellite shadow in real time.
 *
 * Layers:
 *   1. Geometry  — nadir position at time T, user positions around nadir,
 *                  19 hex beam centres
 *   2. Channel   — FSPL + atmospheric loss + Gaussian beam gain + Rician fading
 *   3. Simulation— per-snapshot SINR/SNR computation, beam association
 *
 * Simulation modes:
 *   macro  — static snapshot, concentric-grid users, no fading (Phase 1.0)
 *   rician — static snapshot, random users, Rician fading  (Phase 1.0)
 *   nadir  — ns3 event-driven, nadir-shadow, no fading      (Phase 1.2.1)
 *   arc    — ns3 event-driven, full arc pass, no fading     (Phase 1.2.2)
 *   grid   — ns3 event-driven, d×d ROI + SGP4 orbit         (Phase 2.0, sat-phase2-grid.cc)
 *   dual   — ns3 event-driven, two-satellite ROI + overlap   (Phase 2.2/2.3, sat-phase2-dual.cc)
 *
 * Phase 2 modes (grid, dual) are implemented in separate translation units:
 *   sat-phase2-grid.cc  — Phase 2.0 single-satellite grid ROI
 *   sat-phase2-dual.cc  — Phase 2.2/2.3 dual-satellite grid ROI
 *
 * Usage examples:
 *   ./ns3 run "sat-multi-beam-simulation --mode=macro --time-s=385.37,330.90,239.32"
 *   ./ns3 run "sat-multi-beam-simulation --mode=rician --r-footprint=100000 --time-s=385.37"
 *   ./ns3 run "sat-multi-beam-simulation --mode=grid --d=5 \
 *              --orbit-csv=scratch/orbit_sat_i.csv --out-dir=scratch/grid_out"
 *   ./ns3 run "sat-multi-beam-simulation --mode=dual --d=5 \
 *              --orbit-csv-i=scratch/orbit_sat_i.csv \
 *              --orbit-csv-i1=scratch/orbit_sat_i1.csv \
 *              --out-dir=scratch/dual_out"
 *
 * CMakeLists.txt SOURCE_FILES must include:
 *   sat-multi-beam-geometry.cc  sat-multi-beam-channel.cc
 *   sat-orbit-reader.cc         sat-roi-grid.cc
 *   sat-phase2-grid.cc          sat-phase2-dual.cc
 */

#include "sat-multi-beam-channel.h"
#include "sat-multi-beam-config.h"
#include "sat-multi-beam-geometry.h"
#include "sat-orbit-reader.h"
#include "sat-roi-grid.h"
#include "sat-phase2-dual.h"
#include "sat-phase2-grid.h"

#include "ns3/core-module.h"
#include "ns3/satellite-utils.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace
{

/**
 * ParseTimeList
 *
 * Parse a comma-separated list of simulation times (seconds), e.g.
 * "385.37,330.90,239.32".  Returns an empty vector for empty/"-1" input
 * (caller should fall back to a default).
 */
std::vector<double>
ParseTimeList(const std::string& s)
{
    std::vector<double> out;
    if (s.empty() || s == "-1")
    {
        return out;
    }
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        if (!token.empty())
        {
            out.push_back(std::stod(token));
        }
    }
    return out;
}

/** Format a double array as a JSON array string. */
std::string
JsonDoubleArray(const std::vector<double>& v)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(9) << "[";
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        os << v[i];
        if (i + 1 < v.size())
        {
            os << ",";
        }
    }
    os << "]";
    return os.str();
}

/** Format an int array as a JSON array string. */
std::string
JsonIntArray(const std::vector<int>& v)
{
    std::ostringstream os;
    os << "[";
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        os << v[i];
        if (i + 1 < v.size())
        {
            os << ",";
        }
    }
    os << "]";
    return os.str();
}

/** Format a Vec3 as a JSON triplet. */
std::string
JsonVec3(const Vec3& v)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(9)
       << "[" << v.x << "," << v.y << "," << v.z << "]";
    return os.str();
}

/** Compute mean of a vector. */
double
Mean(const std::vector<double>& v)
{
    if (v.empty())
    {
        return 0.0;
    }
    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

// ---------------------------------------------------------------------------
// JSON writer — schema matches simulation.py / plotResults.py expectations
// ---------------------------------------------------------------------------

/**
 * WriteJson — nadir-shadow version.
 *
 * Filename uses simulation time in milliseconds (integer) instead of
 * frame index: results_{time_ms}_{footprint_km}km.json.
 * The satellite_position field always holds {0, 0, h_satellite} because
 * the local frame is nadir-centred.  lat_nadir/lon_nadir are written as
 * additional fields so plotting scripts can geo-reference results.
 */
void
WriteJson(const std::string&               outDir,
          double                           timeS,
          double                           latNadirDeg,
          double                           lonNadirDeg,
          double                           footprintM,
          const Vec3&                      satPos,
          const std::vector<Vec3>&         userPos,
          const std::array<Vec3, 19>&      beamCenters,
          const std::vector<UserLinkResult>& results)
{
    const int footKm   = static_cast<int>(footprintM / 1000.0);
    const int timeMs   = static_cast<int>(timeS * 1000.0);
    const std::string fname =
        outDir + "/results_" + std::to_string(timeMs) + "ms_" +
        std::to_string(footKm) + "km.json";

    const int nUser = static_cast<int>(userPos.size());
    std::vector<double> snr(nUser), sinr(nUser), centGain(nUser);
    std::vector<int>    beamIndex(nUser);
    for (int u = 0; u < nUser; ++u)
    {
        snr[u]      = results[u].snrDb;
        sinr[u]     = results[u].sinrDb;
        centGain[u] = results[u].centerBeamGainDb;
        beamIndex[u] = results[u].beamId;
    }

    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open JSON output: " << fname);

    out << std::fixed << std::setprecision(9);
    out << "{\n";
    out << "  \"time_s\": " << timeS << ",\n";
    out << "  \"nadir_lat_deg\": " << latNadirDeg << ",\n";
    out << "  \"nadir_lon_deg\": " << lonNadirDeg << ",\n";

    out << "  \"user_positions\": [";
    for (int u = 0; u < nUser; ++u)
    {
        out << JsonVec3(userPos[u]);
        if (u + 1 < nUser)
        {
            out << ",";
        }
    }
    out << "],\n";

    out << "  \"n_user\": " << nUser << ",\n";
    out << "  \"satellite_position\": " << JsonVec3(satPos) << ",\n";
    out << "  \"snr\": " << JsonDoubleArray(snr) << ",\n";
    out << "  \"sinr\": " << JsonDoubleArray(sinr) << ",\n";
    out << "  \"beam_index\": " << JsonIntArray(beamIndex) << ",\n";
    out << "  \"center_beam_gain_dB\": " << JsonDoubleArray(centGain) << ",\n";

    out << "  \"beam_centers\": [";
    for (int j = 0; j < 19; ++j)
    {
        out << JsonVec3(beamCenters[j]);
        if (j + 1 < 19)
        {
            out << ",";
        }
    }
    out << "],\n";

    out << "  \"r_footprint\": " << footprintM << ",\n";
    out << "  \"elevation_angle\": 90.0\n";
    out << "}\n";

    std::cout << "  wrote " << fname << std::endl;
}

// ---------------------------------------------------------------------------
// CSV writers
// ---------------------------------------------------------------------------

void
WriteUserPositionsCsv(const std::string& outDir,
                      const std::vector<Vec3>& userPos)
{
    const std::string fname = outDir + "/user_positions.csv";
    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open: " << fname);

    out << std::fixed << std::setprecision(9);
    out << "user_id,x_m,y_m,z_m\n";
    for (std::size_t u = 0; u < userPos.size(); ++u)
    {
        out << u << "," << userPos[u].x << "," << userPos[u].y << ","
            << userPos[u].z << "\n";
    }
}

/**
 * AppendChannelCsvRows — nadir-shadow version.
 *
 * Replaces frame_id with time_s.  Elevation column is removed (always 90°
 * at footprint centre in nadir-shadow design); nadir lat/lon added instead.
 */
void
AppendChannelCsvRows(std::ofstream&                    csvOut,
                     double                            timeS,
                     double                            latNadirDeg,
                     double                            lonNadirDeg,
                     const std::vector<UserLinkResult>& results)
{
    for (const auto& r : results)
    {
        csvOut << std::fixed << std::setprecision(9)
               << timeS << "," << latNadirDeg << "," << lonNadirDeg << ","
               << r.userId << "," << r.beamId << ","
               << r.pathLossDb << "," << r.sinrDb << ","
               << r.snrDb << "," << r.beamGainDb << "\n";
    }
}

// ---------------------------------------------------------------------------
// Phase 1.2.1 — Nadir-Shadow event-driven state and callback
// ---------------------------------------------------------------------------

/**
 * NadirSimState — all state needed by NadirUpdateStep.
 *
 * The satellite is always at {0, 0, h} in the nadir-centred frame.
 * The footprint centre tracks the sub-satellite point (nadir) at each step.
 * This mirrors the SNS3 shadow ground station (gid=-1).
 */
struct NadirSimState
{
    SimConfig             cfg;
    std::vector<Vec3>     userPos;
    std::array<Vec3, 19>  beamCenters;
    std::ofstream         satCsv;
    std::ofstream         chanCsv;
    std::mt19937          rng{42};
    double                updateMs{100.0};
    double                endTimeS{770.0};
    bool                  withFading{false};
};

/**
 * NadirUpdateStep — ns3 event callback for nadir-shadow mode.
 *
 * Called at every updateMs interval.  At each step:
 *   1. Compute nadir position for the current simulation time.
 *   2. Run ComputeFrameResults with satPos = {0, 0, h}.
 *   3. Write satellite CSV and channel CSV rows.
 *   4. Reschedule the next step (unless the pass has ended).
 */
void
NadirUpdateStep(NadirSimState* s)
{
    const double t = Simulator::Now().GetSeconds();
    if (t > s->endTimeS)
    {
        return;
    }

    double latNadir, lonNadir;
    GetNadirFromTime(t, s->cfg, latNadir, lonNadir);

    SimConfig localCfg            = s->cfg;
    localCfg.latitudeCenterDeg    = latNadir;
    localCfg.longitudeCenterDeg   = lonNadir;

    const Vec3 satPos{0.0, 0.0, s->cfg.hSatelliteM};

    s->satCsv << std::fixed << std::setprecision(9)
              << t << "," << latNadir << "," << lonNadir << "\n";

    const auto results = ComputeFrameResults(
        satPos, s->userPos, s->beamCenters, localCfg, s->rng, s->withFading);

    AppendChannelCsvRows(s->chanCsv, t, latNadir, lonNadir, results);

    Simulator::Schedule(MilliSeconds(s->updateMs), &NadirUpdateStep, s);
}

// ---------------------------------------------------------------------------
// Phase 1.2.2 — Arc-mode event-driven state and callbacks
// ---------------------------------------------------------------------------

/**
 * ArcSimState — all state needed by ArcUpdateStep.
 *
 * The footprint centre is fixed at a geographic location (cfg.latitudeCenterDeg,
 * cfg.longitudeCenterDeg).  The satellite moves through a full arc pass,
 * entering from low elevation, reaching 90° overhead, then descending.
 */
struct ArcSimState
{
    SimConfig             cfg;
    std::vector<Vec3>     userPos;
    std::array<Vec3, 19>  beamCenters;
    std::ofstream         satCsv;
    std::ofstream         chanCsv;
    std::mt19937          rng{42};
    double                updateMs{100.0};
    double                minElevDeg{5.0};
    double                peakElevDeg{0.0};
    double                peakElevTimeS{0.0};
    double                arcStartS{-1.0};
    double                arcEndS{0.0};
    uint32_t              nFramesLogged{0};
    bool                  withFading{false};
    std::string           outDir;
};

/**
 * AppendArcChannelCsvRows — writes arc-mode channel results.
 *
 * Unlike the nadir version, the arc schema includes elevation_deg (not
 * nadir lat/lon) because the footprint centre is fixed and elevation changes.
 */
void
AppendArcChannelCsvRows(std::ofstream&                    csvOut,
                        double                            timeS,
                        double                            elevDeg,
                        const std::vector<UserLinkResult>& results)
{
    for (const auto& r : results)
    {
        csvOut << std::fixed << std::setprecision(9)
               << timeS << "," << elevDeg << ","
               << r.userId << "," << r.beamId << ","
               << r.pathLossDb << "," << r.sinrDb << ","
               << r.snrDb << "," << r.beamGainDb << "\n";
    }
}

/**
 * WriteArcSummaryJson — writes arc_summary.json after the pass completes.
 */
void
WriteArcSummaryJson(const ArcSimState* s)
{
    const std::string fname = s->outDir + "/arc_summary.json";
    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open arc_summary.json: " << fname);
    out << std::fixed << std::setprecision(3);
    out << "{\n";
    out << "  \"arc_start_s\": " << s->arcStartS << ",\n";
    out << "  \"arc_end_s\": " << s->arcEndS << ",\n";
    out << "  \"peak_elevation_s\": " << s->peakElevTimeS << ",\n";
    out << "  \"peak_elevation_deg\": " << s->peakElevDeg << ",\n";
    out << "  \"n_frames_logged\": " << s->nFramesLogged << ",\n";
    out << "  \"update_ms\": " << s->updateMs << ",\n";
    out << "  \"min_elevation_deg\": " << s->minElevDeg << "\n";
    out << "}\n";
    std::cout << "  wrote " << fname << "\n";
}

/**
 * ArcUpdateStep — ns3 event callback for arc mode.
 *
 * Called at every updateMs interval over the full satellite pass.  At each step:
 *   1. Compute satellite position from time via GetSatellitePositionAtTime().
 *   2. Compute elevation angle.
 *   3. Write satellite position row (always, for full arc visualisation).
 *   4. If elevation >= minElevDeg: run ComputeFrameResults and write channel CSV.
 *   5. Reschedule until pass end.
 */
void
ArcUpdateStep(ArcSimState* s)
{
    const double t       = Simulator::Now().GetSeconds();
    const double passEnd = s->cfg.GetTotalFrames() * s->cfg.tFrameS;

    if (t > passEnd)
    {
        return;
    }

    const Vec3   satPos  = GetSatellitePositionAtTime(t, s->cfg);
    const double elevDeg = GetElevationAngleDeg(satPos);

    s->satCsv << std::fixed << std::setprecision(9)
              << t << "," << satPos.x << "," << satPos.y << ","
              << satPos.z << "," << elevDeg << "\n";

    if (elevDeg >= s->minElevDeg)
    {
        if (s->arcStartS < 0.0)
        {
            s->arcStartS = t;
        }
        s->arcEndS = t;

        if (elevDeg > s->peakElevDeg)
        {
            s->peakElevDeg    = elevDeg;
            s->peakElevTimeS  = t;
        }

        const auto results = ComputeFrameResults(
            satPos, s->userPos, s->beamCenters, s->cfg, s->rng, s->withFading);

        AppendArcChannelCsvRows(s->chanCsv, t, elevDeg, results);
        s->nFramesLogged++;
    }

    Simulator::Schedule(MilliSeconds(s->updateMs), &ArcUpdateStep, s);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------

int
main(int argc, char* argv[])
{
    // -----------------------------------------------------------------------
    // Command-line arguments
    // -----------------------------------------------------------------------
    SimConfig cfg;

    std::string mode{"macro"};
    std::string timeStr{"385.37,330.90,239.32"};
    std::string outDir{"scratch/multi_beam_results"};
    double      userSpacingM{500.0};
    uint32_t    seed{42};
    // Phase 1.2 event-driven parameters
    double      updateMs{100.0};
    double      durationS{-1.0};
    double      minElevDeg{5.0};
    // Phase 2.0 grid mode parameters
    int         gridD{5};
    std::string orbitCsv{""};
    // Phase 2.2 dual-satellite mode parameters
    std::string orbitCsvI{""};
    std::string orbitCsvI1{""};
    double      snrThreshDb{0.0};

    CommandLine cmd(__FILE__);
    cmd.AddValue("mode",
                 "Simulation mode: macro (grid users, no fading), rician "
                 "(random users, Rician fading), nadir (Phase 1.2.1 ns3 "
                 "event-driven nadir-shadow), arc (Phase 1.2.2 full arc pass), "
                 "grid (Phase 2.0 d×d ROI with SGP4 orbit), "
                 "dual (Phase 2.2 dual-satellite d×d ROI with two orbit CSVs)",
                 mode);
    cmd.AddValue("time-s",
                 "Comma-separated simulation times (seconds) to snapshot. "
                 "Used by macro/rician modes only.  Default: 385.37,330.90,239.32.",
                 timeStr);
    cmd.AddValue("r-footprint", "Footprint radius (m)", cfg.rFootprintM);
    cmd.AddValue("n-user",      "Number of users",     cfg.nUser);
    cmd.AddValue("out-dir",     "Output directory",    outDir);
    cmd.AddValue("user-spacing","Grid spacing for macro mode (m)", userSpacingM);
    cmd.AddValue("seed",        "RNG seed",            seed);
    cmd.AddValue("h-km",        "Satellite altitude (km)", cfg.hSatelliteM);
    cmd.AddValue("tx-power",    "Transmit power (W)",     cfg.transmitPowerW);
    cmd.AddValue("rician-k",    "Rician K-factor",        cfg.ricianK);
    cmd.AddValue("lat",
                 "Footprint / ROI centre latitude  (degrees N). "
                 "Default: 35.676 (Tokyo).  Override for grid mode observer.",
                 cfg.latitudeCenterDeg);
    cmd.AddValue("lon",
                 "Footprint / ROI centre longitude (degrees E). "
                 "Default: 139.650 (Tokyo).  Override for grid mode observer.",
                 cfg.longitudeCenterDeg);
    cmd.AddValue("update-ms",
                 "Event step size for nadir/arc modes (milliseconds). "
                 "Default: 100 ms.",
                 updateMs);
    cmd.AddValue("duration-s",
                 "Override simulation duration for nadir mode (seconds). "
                 "-1 = full pass duration (default).",
                 durationS);
    cmd.AddValue("min-elevation-deg",
                 "Minimum elevation angle for arc/grid mode to log a frame (degrees). "
                 "Frames below this threshold are skipped.  Default: 5°.",
                 minElevDeg);
    cmd.AddValue("d",
                 "Grid dimension for grid mode: d×d rectangular ROI cells. "
                 "Phase 2.0 inscribed square: L = W = r_footprint × √2. "
                 "Default: 5.",
                 gridD);
    cmd.AddValue("orbit-csv",
                 "Path to satellite_orbit.csv produced by run_sgp4.py. "
                 "Required for grid mode.",
                 orbitCsv);
    cmd.AddValue("orbit-csv-i",
                 "Path to orbit_sat_i.csv (sat[i], from run_sgp4.py --mode=sequence). "
                 "Required for dual mode.",
                 orbitCsvI);
    cmd.AddValue("orbit-csv-i1",
                 "Path to orbit_sat_i1.csv (sat[i+1], same time axis as orbit-csv-i). "
                 "Required for dual mode.",
                 orbitCsvI1);
    cmd.AddValue("snr-thresh-db",
                 "SNR threshold (dB) for Phase 2.3 coverage counting: a cell is "
                 "'covered' by sat[i+1] when its SNR >= this value.  Default: 0 dB.",
                 snrThreshDb);
    cmd.Parse(argc, argv);

    if (cfg.hSatelliteM <= 2000.0)
    {
        cfg.hSatelliteM *= 1000.0;
    }

    std::filesystem::create_directories(outDir);

    // -----------------------------------------------------------------------
    // Phase 2 early dispatch — grid and dual modes do not need userPos or
    // beamCenters from main(); they build their own cell-level state.
    // RunGridMode / RunDualMode call Simulator::Destroy() internally.
    // -----------------------------------------------------------------------
    if (mode == "grid")
    {
        NS_ABORT_MSG_IF(orbitCsv.empty(),
                        "--orbit-csv is required for grid mode. "
                        "Run: python run_sgp4.py --tle-file <tle> --output satellite_orbit.csv");
        NS_ABORT_MSG_IF(gridD < 1, "--d must be >= 1 for grid mode");

        RunGridConfig gridCfg;
        gridCfg.cfg        = cfg;
        gridCfg.orbitCsv   = orbitCsv;
        gridCfg.gridD      = gridD;
        gridCfg.updateMs   = updateMs;
        gridCfg.minElevDeg = minElevDeg;
        gridCfg.outDir     = outDir;
        gridCfg.seed       = seed;
        RunGridMode(gridCfg);
        return 0;
    }

    if (mode == "dual")
    {
        NS_ABORT_MSG_IF(orbitCsvI.empty(),
                        "--orbit-csv-i is required for dual mode. "
                        "Run: python run_sgp4.py --mode=sequence ... --output-sat-i orbit_sat_i.csv");
        NS_ABORT_MSG_IF(orbitCsvI1.empty(),
                        "--orbit-csv-i1 is required for dual mode. "
                        "Run: python run_sgp4.py --mode=sequence ... --output-sat-i1 orbit_sat_i1.csv");
        NS_ABORT_MSG_IF(gridD < 1, "--d must be >= 1 for dual mode");

        RunDualConfig dualCfg;
        dualCfg.cfg         = cfg;
        dualCfg.orbitCsvI   = orbitCsvI;
        dualCfg.orbitCsvI1  = orbitCsvI1;
        dualCfg.gridD       = gridD;
        dualCfg.updateMs    = updateMs;
        dualCfg.minElevDeg  = minElevDeg;
        dualCfg.snrThreshDb = snrThreshDb;
        dualCfg.outDir      = outDir;
        dualCfg.seed        = seed;
        RunDualMode(dualCfg);
        return 0;
    }

    // -----------------------------------------------------------------------
    // Phase 1 modes — print configuration banner
    // -----------------------------------------------------------------------
    std::cout << "\n" << std::string(65, '=') << "\n";
    std::cout << "Multi-Beam LEO Simulation — SNS3 C++ reproduction\n";
    std::cout << std::string(65, '=') << "\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  mode        = " << mode << "\n";
    std::cout << "  h           = " << cfg.hSatelliteM / 1e3 << " km\n";
    std::cout << "  r_footprint = " << cfg.rFootprintM / 1e3 << " km\n";
    std::cout << "  n_user      = " << cfg.nUser << "\n";
    std::cout << "  n_beams     = " << cfg.nBeams << "\n";
    std::cout << "  f_c         = " << cfg.centerFreqHz / 1e9 << " GHz\n";
    std::cout << "  HPBW        = " << cfg.GetHpbwRad() * 180.0 / M_PI << " deg\n";
    std::cout << "  noise power = " << SatUtils::WToDbW(cfg.GetNoisePower()) + 30.0
              << " dBm\n";
    std::cout << "  pass duration (s)  = " << cfg.GetTotalFrames() * cfg.tFrameS << "\n";
    std::cout << "  nadir design       = satellite shadow centred\n";
    std::cout << std::string(65, '=') << "\n\n";

    // -----------------------------------------------------------------------
    // Parse simulation times (macro / rician modes)
    // -----------------------------------------------------------------------
    std::vector<double> timesToRun = ParseTimeList(timeStr);
    if (timesToRun.empty())
    {
        timesToRun = {385.37, 330.90, 239.32};
    }
    const double passEndS = cfg.GetTotalFrames() * cfg.tFrameS;
    timesToRun.erase(
        std::remove_if(timesToRun.begin(), timesToRun.end(),
                       [passEndS](double t) { return t < 0.0 || t > passEndS; }),
        timesToRun.end());
    NS_ABORT_MSG_IF(timesToRun.empty(), "No valid simulation times to evaluate.");

    // -----------------------------------------------------------------------
    // Build user positions
    // -----------------------------------------------------------------------
    const bool     withFading     = (mode == "rician");
    const uint32_t requestedNUser = cfg.nUser;

    std::cout << "Generating user positions (" << mode << " mode)... " << std::flush;
    std::vector<Vec3> userPos;
    if (withFading)
    {
        userPos = GetRandomUserPositions(cfg, seed);
    }
    else
    {
        userPos = GetGridUserPositions(userSpacingM, cfg);
        if (requestedNUser < static_cast<uint32_t>(userPos.size()))
        {
            std::cout << "  (capped from " << userPos.size()
                      << " to " << requestedNUser << " users via --n-user)\n";
            userPos.resize(requestedNUser);
        }
        cfg.nUser = static_cast<int>(userPos.size());
    }
    std::cout << userPos.size() << " users\n";

    // -----------------------------------------------------------------------
    // Build 19 hex beam centres
    // -----------------------------------------------------------------------
    const auto beamCenters = GetHexBeamCenters(cfg);
    std::cout << "Hex beam centres built (19 beams, 2-ring layout)\n";

    // -----------------------------------------------------------------------
    // Write user positions CSV once
    // -----------------------------------------------------------------------
    WriteUserPositionsCsv(outDir, userPos);
    std::cout << "  wrote " << outDir << "/user_positions.csv\n";

    // -----------------------------------------------------------------------
    // Phase 1.2.1 — Nadir-shadow ns3 event-driven mode
    // -----------------------------------------------------------------------
    if (mode == "nadir")
    {
        const double passEnd = cfg.GetTotalFrames() * cfg.tFrameS;
        const double endTime = (durationS > 0.0) ? durationS : passEnd;

        auto* s = new NadirSimState();
        s->cfg         = cfg;
        s->userPos     = userPos;
        s->beamCenters = beamCenters;
        s->rng         = std::mt19937(seed);
        s->updateMs    = updateMs;
        s->endTimeS    = endTime;
        s->withFading  = false;

        s->satCsv.open(outDir + "/satellite_positions.csv",
                       std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!s->satCsv.is_open(),
                        "Cannot open satellite_positions.csv");
        s->satCsv << "time_s,nadir_lat_deg,nadir_lon_deg\n";

        s->chanCsv.open(outDir + "/channel_results.csv",
                        std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!s->chanCsv.is_open(),
                        "Cannot open channel_results.csv");
        s->chanCsv << "time_s,nadir_lat_deg,nadir_lon_deg,user_id,beam_id,"
                      "path_loss_dB,sinr_dB,snr_dB,beam_gain_dB\n";

        std::cout << "\n[nadir mode] duration=" << endTime << "s"
                  << "  step=" << updateMs << "ms"
                  << "  users=" << s->userPos.size() << "\n";

        Simulator::Schedule(Seconds(0.0), &NadirUpdateStep, s);
        Simulator::Run();

        s->satCsv.close();
        s->chanCsv.close();
        std::cout << "  wrote " << outDir << "/satellite_positions.csv\n";
        std::cout << "  wrote " << outDir << "/channel_results.csv\n";

        delete s;
        Simulator::Destroy();
        std::cout << "\nDone.  Output in: " << outDir << "\n";
        return 0;
    }

    // -----------------------------------------------------------------------
    // Phase 1.2.2 — Arc ns3 event-driven mode
    // -----------------------------------------------------------------------
    if (mode == "arc")
    {
        auto* s = new ArcSimState();
        s->cfg         = cfg;
        s->userPos     = userPos;
        s->beamCenters = beamCenters;
        s->rng         = std::mt19937(seed);
        s->updateMs    = updateMs;
        s->minElevDeg  = minElevDeg;
        s->withFading  = false;
        s->outDir      = outDir;

        s->satCsv.open(outDir + "/arc_satellite_positions.csv",
                       std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!s->satCsv.is_open(),
                        "Cannot open arc_satellite_positions.csv");
        s->satCsv << "time_s,sat_x_m,sat_y_m,sat_z_m,elevation_deg\n";

        s->chanCsv.open(outDir + "/arc_channel_results.csv",
                        std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!s->chanCsv.is_open(),
                        "Cannot open arc_channel_results.csv");
        s->chanCsv << "time_s,elevation_deg,user_id,beam_id,"
                      "path_loss_dB,sinr_dB,snr_dB,beam_gain_dB\n";

        const double passEnd = s->cfg.GetTotalFrames() * s->cfg.tFrameS;
        std::cout << "\n[arc mode] pass_duration=" << passEnd << "s"
                  << "  step=" << updateMs << "ms"
                  << "  min_elev=" << minElevDeg << "°"
                  << "  users=" << s->userPos.size() << "\n";

        Simulator::Schedule(Seconds(0.0), &ArcUpdateStep, s);
        Simulator::Run();

        s->satCsv.close();
        s->chanCsv.close();
        std::cout << "  wrote " << outDir << "/arc_satellite_positions.csv\n";
        std::cout << "  wrote " << outDir << "/arc_channel_results.csv\n";

        WriteArcSummaryJson(s);

        delete s;
        Simulator::Destroy();
        std::cout << "\nDone.  Output in: " << outDir << "\n";
        return 0;
    }

    // -----------------------------------------------------------------------
    // Phase 1.0 — macro / rician snapshot mode
    //
    // Open output files for the main simulation loop.
    // -----------------------------------------------------------------------
    std::ofstream satCsv(outDir + "/satellite_positions.csv",
                         std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!satCsv.is_open(),
                    "Cannot open: " << outDir << "/satellite_positions.csv");
    satCsv << "time_s,nadir_lat_deg,nadir_lon_deg\n";

    std::ofstream chanCsv(outDir + "/channel_results.csv",
                          std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!chanCsv.is_open(),
                    "Cannot open: " << outDir << "/channel_results.csv");
    chanCsv << "time_s,nadir_lat_deg,nadir_lon_deg,user_id,beam_id,"
               "path_loss_dB,sinr_dB,snr_dB,beam_gain_dB\n";

    std::mt19937 rng(seed);

    // -----------------------------------------------------------------------
    // Main simulation loop — one iteration per simulation time (snapshot)
    //
    // Nadir-shadow design:
    //   At each time T, compute the satellite sub-satellite point (nadir).
    //   The local frame is centred on the nadir; the satellite is always at
    //   {0, 0, h_satellite}.  Users placed at origin are directly below the
    //   satellite.  This mirrors create_basic_ground_station_for_satellite_shadow()
    //   in SNS3/Hypatia (gid=-1, elevation_m_float=0).
    // -----------------------------------------------------------------------
    std::cout << "\nRunning " << timesToRun.size() << " snapshot(s)...\n";

    for (double timeS : timesToRun)
    {
        double latNadir, lonNadir;
        GetNadirFromTime(timeS, cfg, latNadir, lonNadir);

        const Vec3 satPos{0.0, 0.0, cfg.hSatelliteM};

        SimConfig localCfg = cfg;
        localCfg.latitudeCenterDeg  = latNadir;
        localCfg.longitudeCenterDeg = lonNadir;

        satCsv << std::fixed << std::setprecision(9)
               << timeS << "," << latNadir << "," << lonNadir << "\n";

        const auto results = ComputeFrameResults(
            satPos, userPos, beamCenters, localCfg, rng, withFading);

        std::vector<double> sinrVec(results.size()), snrVec(results.size());
        for (std::size_t u = 0; u < results.size(); ++u)
        {
            sinrVec[u] = results[u].sinrDb;
            snrVec[u]  = results[u].snrDb;
        }
        const double sinrMean = Mean(sinrVec);
        const double snrMean  = Mean(snrVec);

        std::cout << std::fixed << std::setprecision(3)
                  << "  [t=" << timeS << "s]"
                  << std::fixed << std::setprecision(4)
                  << "  nadir=(" << latNadir << "°N, " << lonNadir << "°E)"
                  << std::fixed << std::setprecision(1)
                  << "  SINR mean=" << sinrMean << " dB"
                  << "  SNR mean=" << snrMean << " dB"
                  << "  users=" << results.size() << "\n";

        AppendChannelCsvRows(chanCsv, timeS, latNadir, lonNadir, results);

        WriteJson(outDir, timeS, latNadir, lonNadir,
                  localCfg.rFootprintM, satPos, userPos, beamCenters, results);
    }

    satCsv.close();
    chanCsv.close();
    std::cout << "  wrote " << outDir << "/satellite_positions.csv\n";
    std::cout << "  wrote " << outDir << "/channel_results.csv\n";

    std::cout << "\nDone.  Output in: " << outDir << "\n";

    Simulator::Destroy();
    return 0;
}
