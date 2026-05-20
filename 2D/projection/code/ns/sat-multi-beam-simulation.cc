/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-simulation.cc
 *
 * Multi-Beam LEO Communication Satellite Simulation — SNS3 C++ reproduction
 * of the Python framework:
 *   2D/Multi-Beam LEO Communication Satellite Simulation Framework/
 *
 * Reproduces three layers:
 *   1. Geometry  — satellite arc trajectory, user positions, 19 hex beam centres
 *   2. Channel   — FSPL + atmospheric loss + Gaussian beam gain + Rician fading
 *   3. Simulation— frame-by-frame SINR/SNR computation, beam association
 *
 * Outputs (written to --out-dir):
 *   satellite_positions.csv  — one row per frame processed
 *   user_positions.csv       — fixed user layout
 *   channel_results.csv      — per (frame, user) SINR/SNR/path-loss
 *   results_{frame}_{footprint_km}km.json — JSON matching plotResults.py schema
 *
 * Usage examples:
 *   ./ns3 run "sat-multi-beam-simulation --mode=macro --frame=38537,33090,23932"
 *   ./ns3 run "sat-multi-beam-simulation --mode=rician --r-footprint=100000"
 *   ./ns3 run "sat-multi-beam-simulation --mode=rician --r-footprint=50000 --n-user=100000"
 *
 * Test command:
 *   ./ns3 run "sat-multi-beam-simulation --mode=macro --frame=38537 --n-user=500 \
 *              --out-dir=scratch/mbleo_test"
 *   Then verify scratch/mbleo_test/results_38537_100km.json exists and
 *   contains sinr values in the [-15, 15] dB range.
 */

#include "sat-multi-beam-channel.h"
#include "sat-multi-beam-config.h"
#include "sat-multi-beam-geometry.h"

#include "ns3/core-module.h"
#include "ns3/satellite-utils.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace
{

/** Parse a comma-separated list of ints, e.g. "38537,33090,23932". */
std::vector<int>
ParseFrameList(const std::string& s)
{
    std::vector<int> out;
    if (s.empty() || s == "-1")
    {
        return out; // empty → use all frames
    }
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        if (!token.empty())
        {
            out.push_back(std::stoi(token));
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

void
WriteJson(const std::string&               outDir,
          int                              frameIdx,
          double                           footprintM,
          double                           elevationDeg,
          const Vec3&                      satPos,
          const std::vector<Vec3>&         userPos,
          const std::array<Vec3, 19>&      beamCenters,
          const std::vector<UserLinkResult>& results)
{
    const int footKm = static_cast<int>(footprintM / 1000.0);
    const std::string fname =
        outDir + "/results_" + std::to_string(frameIdx) + "_" +
        std::to_string(footKm) + "km.json";

    // Collect arrays from results
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

    // user_positions: list of [x,y,z]
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

    // beam_centers: list of [x,y,z]
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
    out << "  \"elevation_angle\": " << elevationDeg << "\n";
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

/** Append one frame's results to channel_results.csv (already open). */
void
AppendChannelCsvRows(std::ofstream&                    csvOut,
                     int                               frameIdx,
                     double                            timeS,
                     double                            elevDeg,
                     const std::vector<UserLinkResult>& results)
{
    for (const auto& r : results)
    {
        csvOut << std::fixed << std::setprecision(9)
               << timeS << "," << frameIdx << "," << elevDeg << ","
               << r.userId << "," << r.beamId << ","
               << r.pathLossDb << "," << r.sinrDb << ","
               << r.snrDb << "," << r.beamGainDb << "\n";
    }
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

    std::string mode{"macro"};          // "macro" or "rician"
    std::string frameStr{"38537,33090,23932"}; // comma-list or "-1" for all
    std::string outDir{"scratch/multi_beam_results"};
    double      userSpacingM{500.0};    // grid spacing for macro mode (m)
    uint32_t    seed{42};               // RNG seed

    CommandLine cmd(__FILE__);
    cmd.AddValue("mode",
                 "Simulation mode: macro (grid users, no fading) or rician "
                 "(random users, Rician fading)",
                 mode);
    cmd.AddValue("frame",
                 "Comma-separated frame indices to evaluate, or -1 for all frames "
                 "(default: 38537,33090,23932 → 90°/55°/25° elevation)",
                 frameStr);
    cmd.AddValue("r-footprint", "Footprint radius (m)", cfg.rFootprintM);
    cmd.AddValue("n-user",      "Number of users",     cfg.nUser);
    cmd.AddValue("out-dir",     "Output directory",    outDir);
    cmd.AddValue("user-spacing","Grid spacing for macro mode (m)", userSpacingM);
    cmd.AddValue("seed",        "RNG seed",            seed);
    // Expose key RF parameters for sensitivity studies
    cmd.AddValue("h-km",        "Satellite altitude (km)", cfg.hSatelliteM);
    cmd.AddValue("tx-power",    "Transmit power (W)",     cfg.transmitPowerW);
    cmd.AddValue("rician-k",    "Rician K-factor",        cfg.ricianK);
    cmd.Parse(argc, argv);

    // h-km was provided in km by convention — convert if needed
    // (value ≤ 2000 is treated as km, otherwise already metres)
    if (cfg.hSatelliteM <= 2000.0)
    {
        cfg.hSatelliteM *= 1000.0;
    }

    const bool withFading = (mode == "rician");

    std::filesystem::create_directories(outDir);

    // -----------------------------------------------------------------------
    // Print configuration summary (mirrors Python startup print)
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
    std::cout << "  total frames (full pass) = " << cfg.GetTotalFrames() << "\n";
    std::cout << std::string(65, '=') << "\n\n";

    // -----------------------------------------------------------------------
    // Build satellite arc positions
    // -----------------------------------------------------------------------
    std::cout << "Building satellite arc positions... " << std::flush;
    const auto arcPos = GetSatelliteArcPositions(cfg);
    const int  nArc   = static_cast<int>(arcPos.size());
    std::cout << nArc << " frames\n";

    // -----------------------------------------------------------------------
    // Determine which frames to evaluate
    // -----------------------------------------------------------------------
    std::vector<int> framesToRun = ParseFrameList(frameStr);
    if (framesToRun.empty())
    {
        // All frames
        framesToRun.resize(nArc);
        std::iota(framesToRun.begin(), framesToRun.end(), 0);
    }
    // Clamp to valid range
    framesToRun.erase(
        std::remove_if(framesToRun.begin(), framesToRun.end(),
                       [nArc](int f) { return f < 0 || f >= nArc; }),
        framesToRun.end());
    NS_ABORT_MSG_IF(framesToRun.empty(), "No valid frames to evaluate.");

    // -----------------------------------------------------------------------
    // Build user positions
    // -----------------------------------------------------------------------
    std::cout << "Generating user positions (" << mode << " mode)... " << std::flush;
    std::vector<Vec3> userPos;
    if (withFading)
    {
        // Rician mode: random uniform sphere-cap (matches simulation.py rician)
        userPos = GetRandomUserPositions(cfg, seed);
    }
    else
    {
        // Macro mode: concentric grid (matches simulation.py macro)
        userPos = GetGridUserPositions(userSpacingM, cfg);
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
    // Open satellite_positions.csv and channel_results.csv
    // -----------------------------------------------------------------------
    std::ofstream satCsv(outDir + "/satellite_positions.csv",
                         std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!satCsv.is_open(),
                    "Cannot open: " << outDir << "/satellite_positions.csv");
    satCsv << "frame_id,time_s,elevation_deg,x_m,y_m,z_m\n";

    std::ofstream chanCsv(outDir + "/channel_results.csv",
                          std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!chanCsv.is_open(),
                    "Cannot open: " << outDir << "/channel_results.csv");
    chanCsv << "time_s,frame_id,elevation_deg,user_id,beam_id,"
               "path_loss_dB,sinr_dB,snr_dB,beam_gain_dB\n";

    // -----------------------------------------------------------------------
    // RNG
    // -----------------------------------------------------------------------
    std::mt19937 rng(seed);

    // -----------------------------------------------------------------------
    // Main simulation loop — one iteration per selected frame
    // -----------------------------------------------------------------------
    std::cout << "\nRunning " << framesToRun.size() << " frame(s)...\n";

    for (int frameIdx : framesToRun)
    {
        const Vec3&  satPos  = arcPos[frameIdx];
        const double timeS   = frameIdx * cfg.tFrameS;
        const double elevDeg = GetElevationAngleDeg(satPos);

        // Write satellite position row
        satCsv << std::fixed << std::setprecision(9)
               << frameIdx << "," << timeS << "," << elevDeg << ","
               << satPos.x << "," << satPos.y << "," << satPos.z << "\n";

        // Compute all user link results for this frame
        const auto results = ComputeFrameResults(
            satPos, userPos, beamCenters, cfg, rng, withFading);

        // Collect SINR/SNR for console summary
        std::vector<double> sinrVec(results.size()), snrVec(results.size());
        for (std::size_t u = 0; u < results.size(); ++u)
        {
            sinrVec[u] = results[u].sinrDb;
            snrVec[u]  = results[u].snrDb;
        }
        const double sinrMean = Mean(sinrVec);
        const double snrMean  = Mean(snrVec);

        std::cout << std::fixed << std::setprecision(1)
                  << "  [frame " << frameIdx << "]"
                  << "  elev=" << elevDeg << "°"
                  << "  SINR mean=" << sinrMean << " dB"
                  << "  SNR mean=" << snrMean << " dB"
                  << "  users=" << results.size() << "\n";

        // Append to channel CSV
        AppendChannelCsvRows(chanCsv, frameIdx, timeS, elevDeg, results);

        // Write JSON for this frame (matches plotResults.py schema)
        WriteJson(outDir, frameIdx, cfg.rFootprintM, elevDeg,
                  satPos, userPos, beamCenters, results);
    }

    satCsv.close();
    chanCsv.close();
    std::cout << "  wrote " << outDir << "/satellite_positions.csv\n";
    std::cout << "  wrote " << outDir << "/channel_results.csv\n";

    std::cout << "\nDone.  Output in: " << outDir << "\n";

    Simulator::Destroy();
    return 0;
}
