/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-simulation.cc
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
 * Outputs (written to --out-dir):
 *   satellite_positions.csv  — one row per snapshot processed (time, lat, lon)
 *   user_positions.csv       — user layout (relative to nadir origin)
 *   channel_results.csv      — per (snapshot, user) SINR/SNR/path-loss
 *   results_{time_ms}_{footprint_km}km.json — JSON matching plotResults.py schema
 *
 * Usage examples:
 *   ./ns3 run "sat-multi-beam-simulation --mode=macro --time-s=385.37,330.90,239.32"
 *   ./ns3 run "sat-multi-beam-simulation --mode=rician --r-footprint=100000 --time-s=385.37"
 *   ./ns3 run "sat-multi-beam-simulation --mode=macro --time-s=0,100,200,300"
 *
 * Note: time-s=385.37 ≈ old frame 38537 (nadir/90°), 330.90 ≈ frame 33090 (55°),
 *       239.32 ≈ frame 23932 (25°) — but now footprint is always centred on
 *       the nadir, so elevation at footprint centre is always 90°.
 *
 * Test command:
 *   ./ns3 run "sat-multi-beam-simulation --mode=macro --time-s=385.37 --n-user=500 \
 *              --out-dir=scratch/mbleo_test"
 *   Then verify scratch/mbleo_test/results_385370ms_100km.json exists and
 *   contains sinr values in the [-15, 15] dB range.
 */

#include "sat-multi-beam-channel.h"
#include "sat-multi-beam-config.h"
#include "sat-multi-beam-geometry.h"
#include "sat-orbit-reader.h"
#include "sat-roi-grid.h"

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

    // Nadir geographic position — used by plotting scripts for geo-referencing.
    out << "  \"time_s\": " << timeS << ",\n";
    out << "  \"nadir_lat_deg\": " << latNadirDeg << ",\n";
    out << "  \"nadir_lon_deg\": " << lonNadirDeg << ",\n";

    // user_positions: list of [x,y,z] — relative to nadir origin
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
    // satellite_position is always {0, 0, h_satellite} in nadir-centred frame.
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
    // Elevation at footprint centre is always 90° in the nadir-shadow design.
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
        return; // pass finished — do not reschedule, Simulator::Run() returns
    }

    double latNadir, lonNadir;
    GetNadirFromTime(t, s->cfg, latNadir, lonNadir);

    // Build a per-step config with the current nadir as footprint centre
    SimConfig localCfg            = s->cfg;
    localCfg.latitudeCenterDeg    = latNadir;
    localCfg.longitudeCenterDeg   = lonNadir;

    // In nadir-centred frame the satellite is always directly overhead
    const Vec3 satPos{0.0, 0.0, s->cfg.hSatelliteM};

    s->satCsv << std::fixed << std::setprecision(9)
              << t << "," << latNadir << "," << lonNadir << "\n";

    const auto results = ComputeFrameResults(
        satPos, s->userPos, s->beamCenters, localCfg, s->rng, s->withFading);

    AppendChannelCsvRows(s->chanCsv, t, latNadir, lonNadir, results);

    // Reschedule the next update
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
    double                arcStartS{-1.0};  // set on first in-coverage frame
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
 *
 * Summarises arc start/end, peak elevation time, and frame count so that
 * post-processing scripts can identify the visible window without parsing
 * all per-frame CSV rows.
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
        return; // pass finished — Simulator::Run() returns
    }

    const Vec3   satPos  = GetSatellitePositionAtTime(t, s->cfg);
    const double elevDeg = GetElevationAngleDeg(satPos);

    // Write satellite position for every step (below threshold included)
    s->satCsv << std::fixed << std::setprecision(9)
              << t << "," << satPos.x << "," << satPos.y << ","
              << satPos.z << "," << elevDeg << "\n";

    if (elevDeg >= s->minElevDeg)
    {
        // Record arc window start on the first in-coverage frame
        if (s->arcStartS < 0.0)
        {
            s->arcStartS = t;
        }
        s->arcEndS = t;

        // Track peak elevation
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

// ---------------------------------------------------------------------------
// Phase 2.0 — Grid ROI event-driven state and callbacks
// ---------------------------------------------------------------------------

/**
 * CellStats — per-grid-cell running statistics for the pass summary.
 * Indexed by (row * d + col) across all d*d cells.
 */
struct CellStats
{
    double coverageS{0.0};  // total simulated time with elevation ≥ threshold (s)
    double sumSnrDb{0.0};   // cumulative SNR for mean computation
    double minSnrDb{1e9};   // minimum observed SNR
    double maxSnrDb{-1e9};  // maximum observed SNR
    int    nSamples{0};     // number of in-coverage samples
};

/**
 * GridSimState — all state needed by GridUpdateStep.
 *
 * The observation point (ROI centre) is fixed at cfg.latitudeCenterDeg /
 * cfg.longitudeCenterDeg.  The satellite position is read from a pre-computed
 * SGP4 orbit CSV (output of run_sgp4.py).  Each in-footprint RoiCell centre
 * is treated as a user position passed to ComputeFrameResults().
 *
 * All positions (satellite, beam centres, cell centres) are in local ENU:
 *   x → East,  y → North,  z → Up
 */
struct GridSimState
{
    SimConfig              cfg;
    RoiGrid                grid;
    std::vector<Vec3>      cellPositions;  // in-footprint ENU positions (user list)
    std::vector<size_t>    cellIdxMap;     // cellPositions[i] → grid.cells[] index
    std::array<Vec3, 19>   beamCentersEnu; // beam centres in ENU (rotated from ECEF-offset)
    std::vector<OrbitPoint> orbit;         // full pre-computed SGP4 orbit
    size_t                 orbitIdx{0};    // current lookup position in orbit
    std::ofstream          cellCsv;        // cell_results.csv
    std::vector<CellStats> stats;          // per-cell accumulation (all d*d cells)
    std::mt19937           rng{42};
    double                 updateMs{100.0};
    double                 minElevDeg{5.0};
    std::string            outDir;
};

/**
 * AppendGridCellCsvRows — write one timestep of per-cell channel results.
 *
 * results[i] corresponds to cellPositions[i] which maps to grid.cells[cellIdxMap[i]].
 */
void
AppendGridCellCsvRows(std::ofstream&                    csvOut,
                      double                            timeS,
                      double                            elevDeg,
                      const GridSimState*               s,
                      const std::vector<UserLinkResult>& results)
{
    for (std::size_t i = 0; i < results.size(); ++i)
    {
        const RoiCell&        cell = s->grid.cells[s->cellIdxMap[i]];
        const UserLinkResult& r    = results[i];
        csvOut << std::fixed << std::setprecision(6)
               << timeS << ","
               << cell.row << "," << cell.col << ","
               << std::setprecision(3) << cell.cx_m << "," << cell.cy_m << ","
               << std::setprecision(4) << elevDeg << ","
               << "1,"  // in_footprint: always 1 for processed cells
               << r.pathLossDb << "," << r.beamGainDb << ","
               << r.snrDb << "," << r.sinrDb << "\n";
    }
}

/**
 * WriteGridSummaryCsv — write per-cell pass statistics after simulation ends.
 *
 * All d*d cells are written; out-of-footprint cells have coverage_s = 0.
 */
void
WriteGridSummaryCsv(const GridSimState* s)
{
    const std::string fname = s->outDir + "/cell_summary.csv";
    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open cell_summary.csv: " << fname);

    out << "row,col,cx_m,cy_m,in_footprint,coverage_s,"
           "mean_snr_dB,min_snr_dB,max_snr_dB\n";

    const int d = s->grid.d;
    for (int row = 0; row < d; ++row)
    {
        for (int col = 0; col < d; ++col)
        {
            const int         idx  = row * d + col;
            const RoiCell&    cell = s->grid.cells[idx];
            const CellStats&  st   = s->stats[idx];

            const double meanSnr = (st.nSamples > 0)
                                   ? st.sumSnrDb / st.nSamples : 0.0;
            const double minSnr  = (st.nSamples > 0) ? st.minSnrDb : 0.0;
            const double maxSnr  = (st.nSamples > 0) ? st.maxSnrDb : 0.0;

            out << std::fixed << std::setprecision(3)
                << row << "," << col << ","
                << cell.cx_m << "," << cell.cy_m << ","
                << (cell.inFootprint ? 1 : 0) << ","
                << st.coverageS << ","
                << std::setprecision(4) << meanSnr << ","
                << minSnr << "," << maxSnr << "\n";
        }
    }

    std::cout << "  wrote " << fname << "\n";
}

/**
 * GridUpdateStep — ns3 event callback for Phase 2.0 grid ROI mode.
 *
 * Called at every updateMs interval.  At each step:
 *   1. Advance orbit pointer to the last point with timeS ≤ current time.
 *   2. Read elevation_deg from the CSV (PyEphem, accurate).
 *   3. If elevation ≥ minElevDeg and cells exist:
 *      a. Convert satellite geodetic position to local ENU Vec3.
 *      b. Run ComputeFrameResults for all in-footprint cell positions.
 *      c. Write cell_results.csv rows.
 *      d. Accumulate per-cell coverage and SNR statistics.
 *   4. Reschedule until the last orbit data point.
 */
void
GridUpdateStep(GridSimState* s)
{
    const double t = Simulator::Now().GetSeconds();

    // Advance orbit pointer: find the last point with timeS <= t
    while (s->orbitIdx + 1 < s->orbit.size() &&
           s->orbit[s->orbitIdx + 1].timeS <= t)
    {
        ++s->orbitIdx;
    }

    const OrbitPoint& pt     = s->orbit[s->orbitIdx];
    const double      elevDeg = pt.elevationDeg; // PyEphem value, use directly

    if (elevDeg >= s->minElevDeg && !s->cellPositions.empty())
    {
        // Convert satellite geodetic position to local ENU Vec3
        const Vec3 satEnu = OrbitPointToEnuVec3(
            pt,
            s->cfg.latitudeCenterDeg,
            s->cfg.longitudeCenterDeg,
            s->cfg.rEarthM);

        // Compute macro channel for all in-footprint cells (no Rician fading)
        const auto results = ComputeFrameResults(
            satEnu,
            s->cellPositions,
            s->beamCentersEnu,
            s->cfg,
            s->rng,
            false); // withFading = false: grid mode uses macro SNR

        AppendGridCellCsvRows(s->cellCsv, t, elevDeg, s, results);

        // Accumulate statistics
        const double dtS = s->updateMs / 1000.0;
        for (std::size_t i = 0; i < results.size(); ++i)
        {
            const int    gIdx = static_cast<int>(s->cellIdxMap[i]);
            CellStats&   st   = s->stats[gIdx];
            const double snr  = results[i].snrDb;
            st.coverageS += dtS;
            st.sumSnrDb  += snr;
            if (snr < st.minSnrDb) { st.minSnrDb = snr; }
            if (snr > st.maxSnrDb) { st.maxSnrDb = snr; }
            st.nSamples++;
        }
    }

    // Reschedule while there are future orbit data points
    const double nextT = t + s->updateMs / 1000.0;
    if (nextT <= s->orbit.back().timeS)
    {
        Simulator::Schedule(MilliSeconds(s->updateMs), &GridUpdateStep, s);
    }
}

// ---------------------------------------------------------------------------
// Phase 2.2 — Dual-Satellite Grid ROI state and callbacks
// ---------------------------------------------------------------------------

/**
 * DualCellStats — per-grid-cell statistics accumulated for both sat[i] and
 * sat[i+1] across the full simulation window.
 *
 * Greedy SNR is computed by selecting the better satellite at every step,
 * which is the Phase 2.4 baseline assignment policy.
 */
struct DualCellStats
{
    // sat[i] accumulation
    double coverageSI{0.0};
    double sumSnrSI{0.0};
    double maxSnrSI{-1e9};
    int    nSamplesSI{0};
    // sat[i+1] accumulation
    double coverageSI1{0.0};
    double sumSnrSI1{0.0};
    double maxSnrSI1{-1e9};
    int    nSamplesSI1{0};
    // Greedy (best-of-two at each step) accumulation
    double sumGreedySnr{0.0};
    int    nGreedy{0};
};

/**
 * DualSimState — all state needed by DualUpdateStep.
 *
 * Both orbit CSVs must share the same 0-based time_s axis, which is
 * guaranteed by run_sgp4.py --mode=sequence output (propagate_window
 * sets time_s = i * step_s for both CSVs over the same window).
 *
 * Phase 2.3 overlap detection (inline):
 *   At each step, count in-footprint cells where sat[i+1] SNR >= snrThreshDb.
 *   Record the first time this count reaches 10/25/50/75/90% of all
 *   in-footprint cells.  Written to dual_overlap.json after simulation ends.
 */
struct DualSimState
{
    SimConfig                cfg;
    RoiGrid                  grid;
    std::vector<Vec3>        cellPositions;   // in-footprint ENU cell centres
    std::vector<size_t>      cellIdxMap;      // cellPositions[i] → grid.cells index
    std::array<Vec3, 19>     beamCentersEnu;  // static beam centres (elevation=90° approx)
    std::vector<OrbitPoint>  orbitI;          // sat[i] — full pre-computed orbit
    std::vector<OrbitPoint>  orbitI1;         // sat[i+1] — same time axis
    size_t                   idxI{0};
    size_t                   idxI1{0};
    std::ofstream            dualCsv;         // dual_cell_results.csv
    std::vector<DualCellStats> stats;         // all d*d cells, indexed row*d+col
    // Phase 2.3 coverage threshold detection
    std::array<double, 5>    thresholdsPct{10.0, 25.0, 50.0, 75.0, 90.0};
    std::array<double, 5>    overlapTimesS{-1.0, -1.0, -1.0, -1.0, -1.0};
    int                      nInFootprint{0};
    double                   snrThreshDb{0.0};  // SNR ≥ 0 dB ≡ "covered" by sat[i+1]
    std::mt19937             rng{42};
    double                   updateMs{100.0};
    double                   minElevDeg{5.0};
    std::string              outDir;
};

/**
 * AppendDualCellCsvRows — write one timestep of dual-satellite per-cell SNR.
 *
 * For each in-footprint cell one row is written.  If a satellite is below
 * minElevDeg its SNR/SINR columns are set to -999.0 (sentinel = "not visible").
 *
 * best_sat column:
 *    0  → sat[i] wins (higher SNR) or sat[i+1] not visible
 *    1  → sat[i+1] wins or sat[i] not visible
 *   -1  → neither satellite visible this step
 */
void
AppendDualCellCsvRows(std::ofstream&                     csvOut,
                      double                             timeS,
                      double                             elevI,
                      double                             elevI1,
                      bool                               visI,
                      bool                               visI1,
                      const DualSimState*                s,
                      const std::vector<UserLinkResult>& resI,
                      const std::vector<UserLinkResult>& resI1)
{
    for (std::size_t i = 0; i < s->cellPositions.size(); ++i)
    {
        const RoiCell& cell = s->grid.cells[s->cellIdxMap[i]];

        const double snrI   = visI  ? resI[i].snrDb   : -999.0;
        const double sinrI  = visI  ? resI[i].sinrDb  : -999.0;
        const double snrI1  = visI1 ? resI1[i].snrDb  : -999.0;
        const double sinrI1 = visI1 ? resI1[i].sinrDb : -999.0;

        int bestSat = -1;
        if (visI && visI1)      { bestSat = (snrI1 >= snrI) ? 1 : 0; }
        else if (visI)          { bestSat = 0; }
        else if (visI1)         { bestSat = 1; }

        csvOut << std::fixed << std::setprecision(6)
               << timeS << ","
               << cell.row << "," << cell.col << ","
               << std::setprecision(3) << cell.cx_m << "," << cell.cy_m << ","
               << std::setprecision(4) << elevI  << "," << snrI  << "," << sinrI  << ","
               <<                         elevI1 << "," << snrI1 << "," << sinrI1 << ","
               << bestSat << "\n";
    }
}

/**
 * WriteDualSummaryCsv — per-cell statistics for both satellites after
 * the full simulation window.
 *
 * greedy_mean_snr_dB is the per-cell mean SNR when always assigning the
 * satellite with higher SNR at each step — this is the Phase 2.4 baseline.
 */
void
WriteDualSummaryCsv(const DualSimState* s)
{
    const std::string fname = s->outDir + "/dual_cell_summary.csv";
    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open: " << fname);

    out << "row,col,cx_m,cy_m,in_footprint,"
           "coverage_i_s,mean_snr_i_dB,max_snr_i_dB,"
           "coverage_i1_s,mean_snr_i1_dB,max_snr_i1_dB,"
           "greedy_mean_snr_dB\n";

    const int d = s->grid.d;
    for (int row = 0; row < d; ++row)
    {
        for (int col = 0; col < d; ++col)
        {
            const int            idx  = row * d + col;
            const RoiCell&       cell = s->grid.cells[idx];
            const DualCellStats& st   = s->stats[idx];

            const double meanI   = (st.nSamplesSI  > 0) ? st.sumSnrSI  / st.nSamplesSI  : 0.0;
            const double meanI1  = (st.nSamplesSI1 > 0) ? st.sumSnrSI1 / st.nSamplesSI1 : 0.0;
            const double maxI    = (st.nSamplesSI  > 0) ? st.maxSnrSI  : 0.0;
            const double maxI1   = (st.nSamplesSI1 > 0) ? st.maxSnrSI1 : 0.0;
            const double greedy  = (st.nGreedy     > 0) ? st.sumGreedySnr / st.nGreedy   : 0.0;

            out << std::fixed << std::setprecision(3)
                << row << "," << col << ","
                << cell.cx_m << "," << cell.cy_m << ","
                << (cell.inFootprint ? 1 : 0) << ","
                << st.coverageSI  << ","
                << std::setprecision(4) << meanI  << "," << maxI  << ","
                << std::setprecision(3) << st.coverageSI1 << ","
                << std::setprecision(4) << meanI1 << "," << maxI1 << ","
                << greedy << "\n";
        }
    }
    std::cout << "  wrote " << fname << "\n";
}

/**
 * WriteDualOverlapJson — Phase 2.3 overlap stage detection results.
 *
 * Records the first simulation time (seconds, 0-based) at which sat[i+1]
 * provides SNR >= snrThreshDb to at least X% of all in-footprint cells.
 * A value of -1.0 means the threshold was not reached within the window.
 */
void
WriteDualOverlapJson(const DualSimState* s)
{
    const std::string fname = s->outDir + "/dual_overlap.json";
    std::ofstream out(fname, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open: " << fname);

    const std::array<std::string, 5> keys{"10pct", "25pct", "50pct", "75pct", "90pct"};

    out << std::fixed << std::setprecision(3);
    out << "{\n";
    out << "  \"n_in_footprint\": "    << s->nInFootprint  << ",\n";
    out << "  \"snr_threshold_dB\": "  << s->snrThreshDb   << ",\n";
    out << "  \"thresholds_pct\": [10, 25, 50, 75, 90],\n";
    out << "  \"sat_i1_coverage_times_s\": {\n";
    for (int k = 0; k < 5; ++k)
    {
        out << "    \"" << keys[k] << "\": " << s->overlapTimesS[k];
        if (k < 4) { out << ","; }
        out << "\n";
    }
    out << "  }\n";
    out << "}\n";
    std::cout << "  wrote " << fname << "\n";

    std::cout << "\n[Phase 2.3] sat[i+1] coverage thresholds"
              << "  (SNR >= " << s->snrThreshDb << " dB"
              << ", n_in_fp = " << s->nInFootprint << "):\n";
    for (int k = 0; k < 5; ++k)
    {
        std::cout << "  " << std::setw(3) << static_cast<int>(s->thresholdsPct[k])
                  << "% : ";
        if (s->overlapTimesS[k] >= 0.0)
            std::cout << "t = " << s->overlapTimesS[k] << " s\n";
        else
            std::cout << "NOT REACHED in window\n";
    }
}

/**
 * DualUpdateStep — ns3 event callback for Phase 2.2 dual-satellite grid mode.
 *
 * Called at every updateMs interval.  At each step:
 *   1. Advance both orbit pointers to the last point with timeS <= t.
 *   2. Compute SNR for sat[i] if elevation >= minElevDeg.
 *   3. Compute SNR for sat[i+1] if elevation >= minElevDeg.
 *   4. Write dual_cell_results.csv rows.
 *   5. Accumulate DualCellStats (coverage, SNR sums, greedy SNR).
 *   6. Phase 2.3: count cells with sat[i+1] SNR >= snrThreshDb and
 *      record first-crossing times for 10/25/50/75/90% thresholds.
 *   7. Reschedule until the end of the longer orbit stream.
 */
void
DualUpdateStep(DualSimState* s)
{
    const double t = Simulator::Now().GetSeconds();

    // Advance both orbit pointers
    while (s->idxI  + 1 < s->orbitI.size()  && s->orbitI[s->idxI   + 1].timeS <= t)
        ++s->idxI;
    while (s->idxI1 + 1 < s->orbitI1.size() && s->orbitI1[s->idxI1 + 1].timeS <= t)
        ++s->idxI1;

    const OrbitPoint& ptI  = s->orbitI[s->idxI];
    const OrbitPoint& ptI1 = s->orbitI1[s->idxI1];

    const double elevI  = ptI.elevationDeg;
    const double elevI1 = ptI1.elevationDeg;

    const bool visI  = (elevI  >= s->minElevDeg) && !s->cellPositions.empty();
    const bool visI1 = (elevI1 >= s->minElevDeg) && !s->cellPositions.empty();

    // Compute per-cell results for each visible satellite
    std::vector<UserLinkResult> resI, resI1;

    if (visI)
    {
        const Vec3 satEnuI = OrbitPointToEnuVec3(
            ptI, s->cfg.latitudeCenterDeg, s->cfg.longitudeCenterDeg, s->cfg.rEarthM);
        resI = ComputeFrameResults(
            satEnuI, s->cellPositions, s->beamCentersEnu, s->cfg, s->rng, false);
    }
    if (visI1)
    {
        const Vec3 satEnuI1 = OrbitPointToEnuVec3(
            ptI1, s->cfg.latitudeCenterDeg, s->cfg.longitudeCenterDeg, s->cfg.rEarthM);
        resI1 = ComputeFrameResults(
            satEnuI1, s->cellPositions, s->beamCentersEnu, s->cfg, s->rng, false);
    }

    // Write CSV when at least one satellite is visible
    if (visI || visI1)
    {
        // Provide empty result vectors for non-visible satellite — CSV writer
        // uses visI/visI1 flags and writes -999 sentinels instead.
        if (!visI)  { resI.resize(s->cellPositions.size());  }
        if (!visI1) { resI1.resize(s->cellPositions.size()); }

        AppendDualCellCsvRows(
            s->dualCsv, t, elevI, elevI1, visI, visI1, s, resI, resI1);
    }

    // Accumulate per-cell stats and Phase 2.3 threshold detection
    const double dtS = s->updateMs / 1000.0;
    int cellsI1Covered = 0;

    for (std::size_t i = 0; i < s->cellPositions.size(); ++i)
    {
        const int gIdx = static_cast<int>(s->cellIdxMap[i]);
        DualCellStats& st = s->stats[gIdx];

        if (visI)
        {
            const double snr = resI[i].snrDb;
            st.coverageSI += dtS;
            st.sumSnrSI   += snr;
            if (snr > st.maxSnrSI) { st.maxSnrSI = snr; }
            st.nSamplesSI++;
        }
        if (visI1)
        {
            const double snr = resI1[i].snrDb;
            st.coverageSI1 += dtS;
            st.sumSnrSI1   += snr;
            if (snr > st.maxSnrSI1) { st.maxSnrSI1 = snr; }
            st.nSamplesSI1++;
            if (snr >= s->snrThreshDb) { ++cellsI1Covered; }
        }

        // Greedy: pick the higher-SNR satellite for this cell at this step
        if (visI || visI1)
        {
            double greedySnr;
            if (visI && visI1) { greedySnr = std::max(resI[i].snrDb, resI1[i].snrDb); }
            else if (visI)     { greedySnr = resI[i].snrDb; }
            else               { greedySnr = resI1[i].snrDb; }
            st.sumGreedySnr += greedySnr;
            st.nGreedy++;
        }
    }

    // Phase 2.3: record first-crossing times for coverage fraction thresholds
    if (s->nInFootprint > 0)
    {
        const double pct = 100.0 * cellsI1Covered / s->nInFootprint;
        for (int k = 0; k < 5; ++k)
        {
            if (s->overlapTimesS[k] < 0.0 && pct >= s->thresholdsPct[k])
            {
                s->overlapTimesS[k] = t;
            }
        }
    }

    // Reschedule while there is data remaining in either orbit stream
    const double nextT = t + s->updateMs / 1000.0;
    const double winEnd = std::max(s->orbitI.back().timeS, s->orbitI1.back().timeS);
    if (nextT <= winEnd)
    {
        Simulator::Schedule(MilliSeconds(s->updateMs), &DualUpdateStep, s);
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

    std::string mode{"macro"};
    // Default times (seconds) correspond to old frame defaults for reference:
    //   385.37s ≈ frame 38537 (nadir/90°),
    //   330.90s ≈ frame 33090 (55° from old centre),
    //   239.32s ≈ frame 23932 (25° from old centre).
    // In nadir-shadow design, elevation at footprint centre is always 90°;
    // these three times differ in which ground track point is covered.
    std::string timeStr{"385.37,330.90,239.32"};
    std::string outDir{"scratch/multi_beam_results"};
    double      userSpacingM{500.0};
    uint32_t    seed{42};
    // Phase 1.2 event-driven parameters
    double      updateMs{100.0};    // ns3 event step (milliseconds)
    double      durationS{-1.0};    // override pass duration (-1 = auto)
    double      minElevDeg{5.0};    // arc/grid minimum elevation threshold
    // Phase 2.0 grid mode parameters
    int         gridD{5};           // d×d grid dimension
    std::string orbitCsv{""};       // path to satellite_orbit.csv (run_sgp4.py output)
    // Phase 2.2 dual-satellite mode parameters
    std::string orbitCsvI{""};      // path to orbit_sat_i.csv (run_sgp4.py --mode=sequence)
    std::string orbitCsvI1{""};     // path to orbit_sat_i1.csv
    double      snrThreshDb{0.0};   // SNR threshold for Phase 2.3 coverage counting (dB)

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
    // Phase 1.2 event-driven options
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
    // Phase 2.0 grid mode
    cmd.AddValue("d",
                 "Grid dimension for grid mode: d×d rectangular ROI cells. "
                 "Phase 2.0 inscribed square: L = W = r_footprint × √2. "
                 "Default: 5.",
                 gridD);
    cmd.AddValue("orbit-csv",
                 "Path to satellite_orbit.csv produced by run_sgp4.py. "
                 "Required for grid mode.",
                 orbitCsv);
    // Phase 2.2 dual-satellite mode
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

    const bool withFading = (mode == "rician");
    const uint32_t requestedNUser = cfg.nUser;

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
    std::cout << "  pass duration (s)  = " << cfg.GetTotalFrames() * cfg.tFrameS << "\n";
    std::cout << "  nadir design       = satellite shadow centred\n";
    std::cout << std::string(65, '=') << "\n\n";

    // -----------------------------------------------------------------------
    // Parse simulation times
    // -----------------------------------------------------------------------
    std::vector<double> timesToRun = ParseTimeList(timeStr);
    if (timesToRun.empty())
    {
        // Default: three representative snapshots.
        timesToRun = {385.37, 330.90, 239.32};
    }
    // Clamp to valid pass duration.
    const double passEndS = cfg.GetTotalFrames() * cfg.tFrameS;
    timesToRun.erase(
        std::remove_if(timesToRun.begin(), timesToRun.end(),
                       [passEndS](double t) { return t < 0.0 || t > passEndS; }),
        timesToRun.end());
    NS_ABORT_MSG_IF(timesToRun.empty(), "No valid simulation times to evaluate.");

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
        // Macro mode: concentric grid (matches simulation.py macro).
        // --n-user caps the grid for ALL non-rician modes.  Without a cap the
        // default spacing (500 m) over a 100 km footprint yields ~122k users,
        // which is correct for full Phase-1 validation but far too large for
        // nadir/arc event-driven runs.
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
        s->withFading  = false; // nadir mode uses macro (no fading) by default

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
        s->withFading  = false; // arc mode uses macro (no fading) by default
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
    // Phase 2.0 — Grid ROI mode: d×d rectangular ROI + SGP4 orbit
    // -----------------------------------------------------------------------
    if (mode == "grid")
    {
        NS_ABORT_MSG_IF(orbitCsv.empty(),
                        "--orbit-csv is required for grid mode. "
                        "Run: python run_sgp4.py --tle-file <tle> --output satellite_orbit.csv");
        NS_ABORT_MSG_IF(gridD < 1, "--d must be >= 1 for grid mode");

        // Load pre-computed SGP4 orbit
        std::cout << "[grid mode] Loading orbit CSV: " << orbitCsv << "\n";
        const auto orbit = LoadOrbit(orbitCsv);
        std::cout << "  " << orbit.size() << " orbit points"
                  << "  t_start=" << orbit.front().timeS << "s"
                  << "  t_end="   << orbit.back().timeS  << "s\n";

        // Build d×d ROI grid (Phase 2.0: inscribed square, L = W = r × √2)
        const RoiGrid grid    = GenerateRoiGrid(gridD, cfg.rFootprintM);
        const int     nInFP   = static_cast<int>(
            std::count_if(grid.cells.begin(), grid.cells.end(),
                          [](const RoiCell& c) { return c.inFootprint; }));

        std::cout << "[grid mode] " << gridD << "×" << gridD
                  << " grid = " << (gridD * gridD) << " cells"
                  << "  in-footprint = " << nInFP << "\n"
                  << "  L = W = " << grid.L_m / 1e3 << " km"
                  << "  cell = " << grid.cellL / 1e3 << " km × "
                  << grid.cellW / 1e3 << " km\n";

        // In-footprint cell positions and index map for result attribution
        const auto cellPos    = GetRoiCellPositions(grid);
        const auto cellIdxMap = GetRoiCellIndexMap(grid);

        // Beam centres: convert ECEF-offset → ENU so all positions share the
        // same coordinate system as OrbitPointToEnuVec3() output.
        const auto beamCentersEcef = GetHexBeamCenters(cfg);
        std::array<Vec3, 19> beamCentersEnu;
        for (int i = 0; i < 19; ++i)
        {
            beamCentersEnu[i] = EcefOffsetToEnu(
                beamCentersEcef[i],
                cfg.latitudeCenterDeg,
                cfg.longitudeCenterDeg);
        }

        auto* s = new GridSimState();
        s->cfg            = cfg;
        s->grid           = grid;
        s->cellPositions  = cellPos;
        s->cellIdxMap     = cellIdxMap;
        s->beamCentersEnu = beamCentersEnu;
        s->orbit          = orbit;
        s->rng            = std::mt19937(seed);
        s->updateMs       = updateMs;
        s->minElevDeg     = minElevDeg;
        s->outDir         = outDir;
        s->stats.assign(gridD * gridD, CellStats{});

        s->cellCsv.open(outDir + "/cell_results.csv",
                        std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!s->cellCsv.is_open(),
                        "Cannot open cell_results.csv in: " << outDir);
        s->cellCsv << "time_s,row,col,cx_m,cy_m,elevation_deg,in_footprint,"
                      "path_loss_dB,beam_gain_dB,snr_dB,sinr_dB\n";

        std::cout << "\n[grid mode]"
                  << "  obs=(" << cfg.latitudeCenterDeg << "°N, "
                  << cfg.longitudeCenterDeg << "°E)"
                  << "  step=" << updateMs << "ms"
                  << "  min_elev=" << minElevDeg << "°"
                  << "  in-FP cells=" << nInFP << "\n";

        Simulator::Schedule(Seconds(0.0), &GridUpdateStep, s);
        Simulator::Run();

        s->cellCsv.close();
        std::cout << "  wrote " << outDir << "/cell_results.csv\n";

        WriteGridSummaryCsv(s);

        delete s;
        Simulator::Destroy();
        std::cout << "\nDone.  Output in: " << outDir << "\n";
        return 0;
    }

    // -----------------------------------------------------------------------
    // Phase 2.2 — Dual-satellite grid ROI mode
    //
    // Loads two orbit CSVs (sat[i] and sat[i+1]) that share a 0-based time_s
    // axis (produced by run_sgp4.py --mode=sequence).  At each timestep, per-
    // cell SNR is computed for both satellites simultaneously.
    //
    // Outputs:
    //   dual_cell_results.csv  — per (time, cell) SNR for both satellites
    //   dual_cell_summary.csv  — per-cell pass statistics + greedy SNR
    //   dual_overlap.json      — Phase 2.3 overlap threshold timestamps
    // -----------------------------------------------------------------------
    if (mode == "dual")
    {
        NS_ABORT_MSG_IF(orbitCsvI.empty(),
                        "--orbit-csv-i is required for dual mode. "
                        "Run: python run_sgp4.py --mode=sequence ... --output-sat-i orbit_sat_i.csv");
        NS_ABORT_MSG_IF(orbitCsvI1.empty(),
                        "--orbit-csv-i1 is required for dual mode. "
                        "Run: python run_sgp4.py --mode=sequence ... --output-sat-i1 orbit_sat_i1.csv");
        NS_ABORT_MSG_IF(gridD < 1, "--d must be >= 1 for dual mode");

        // Load both orbit CSVs
        std::cout << "[dual mode] Loading sat[i]   orbit: " << orbitCsvI  << "\n";
        const auto orbitI  = LoadOrbit(orbitCsvI);
        std::cout << "  " << orbitI.size() << " points"
                  << "  t=[" << orbitI.front().timeS << "s, " << orbitI.back().timeS << "s]\n";

        std::cout << "[dual mode] Loading sat[i+1] orbit: " << orbitCsvI1 << "\n";
        const auto orbitI1 = LoadOrbit(orbitCsvI1);
        std::cout << "  " << orbitI1.size() << " points"
                  << "  t=[" << orbitI1.front().timeS << "s, " << orbitI1.back().timeS << "s]\n";

        NS_ABORT_MSG_IF(orbitI.size() != orbitI1.size(),
                        "orbit-csv-i and orbit-csv-i1 must have the same row count "
                        "(same time axis).  Got " << orbitI.size()
                        << " vs " << orbitI1.size() << ".  "
                        "Re-run run_sgp4.py --mode=sequence to regenerate.");

        // Build d×d ROI grid
        const RoiGrid grid  = GenerateRoiGrid(gridD, cfg.rFootprintM);
        const int     nInFP = static_cast<int>(
            std::count_if(grid.cells.begin(), grid.cells.end(),
                          [](const RoiCell& c) { return c.inFootprint; }));

        std::cout << "[dual mode] " << gridD << "×" << gridD
                  << " grid = " << (gridD * gridD) << " cells"
                  << "  in-footprint = " << nInFP << "\n"
                  << "  L = W = " << grid.L_m / 1e3 << " km"
                  << "  cell = " << grid.cellL / 1e3 << " km × "
                  << grid.cellW / 1e3 << " km\n";

        const auto cellPos    = GetRoiCellPositions(grid);
        const auto cellIdxMap = GetRoiCellIndexMap(grid);

        // Static beam centres (elevation=90° approximation, same as Phase 2.0)
        const auto beamCentersEcef = GetHexBeamCenters(cfg);
        std::array<Vec3, 19> beamCentersEnu;
        for (int i = 0; i < 19; ++i)
        {
            beamCentersEnu[i] = EcefOffsetToEnu(
                beamCentersEcef[i],
                cfg.latitudeCenterDeg,
                cfg.longitudeCenterDeg);
        }

        auto* s = new DualSimState();
        s->cfg             = cfg;
        s->grid            = grid;
        s->cellPositions   = cellPos;
        s->cellIdxMap      = cellIdxMap;
        s->beamCentersEnu  = beamCentersEnu;
        s->orbitI          = orbitI;
        s->orbitI1         = orbitI1;
        s->rng             = std::mt19937(seed);
        s->updateMs        = updateMs;
        s->minElevDeg      = minElevDeg;
        s->snrThreshDb     = snrThreshDb;
        s->nInFootprint    = nInFP;
        s->outDir          = outDir;
        s->stats.assign(gridD * gridD, DualCellStats{});

        s->dualCsv.open(outDir + "/dual_cell_results.csv",
                        std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!s->dualCsv.is_open(),
                        "Cannot open dual_cell_results.csv in: " << outDir);
        s->dualCsv << "time_s,row,col,cx_m,cy_m,"
                      "elev_i_deg,snr_i_dB,sinr_i_dB,"
                      "elev_i1_deg,snr_i1_dB,sinr_i1_dB,"
                      "best_sat\n";

        std::cout << "\n[dual mode]"
                  << "  obs=(" << cfg.latitudeCenterDeg << "°N, "
                  << cfg.longitudeCenterDeg << "°E)"
                  << "  step=" << updateMs << "ms"
                  << "  min_elev=" << minElevDeg << "°"
                  << "  in-FP cells=" << nInFP
                  << "  snr_thresh=" << snrThreshDb << " dB\n";

        Simulator::Schedule(Seconds(0.0), &DualUpdateStep, s);
        Simulator::Run();

        s->dualCsv.close();
        std::cout << "  wrote " << outDir << "/dual_cell_results.csv\n";

        WriteDualSummaryCsv(s);
        WriteDualOverlapJson(s);

        delete s;
        Simulator::Destroy();
        std::cout << "\nDone.  Output in: " << outDir << "\n";
        return 0;
    }

    // -----------------------------------------------------------------------
    // Open satellite_positions.csv and channel_results.csv
    // -----------------------------------------------------------------------
    std::ofstream satCsv(outDir + "/satellite_positions.csv",
                         std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!satCsv.is_open(),
                    "Cannot open: " << outDir << "/satellite_positions.csv");
    // Nadir-shadow schema: time + nadir lat/lon.  Satellite x/y/z in local
    // frame is always 0,0,h_satellite so those columns are omitted.
    satCsv << "time_s,nadir_lat_deg,nadir_lon_deg\n";

    std::ofstream chanCsv(outDir + "/channel_results.csv",
                          std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!chanCsv.is_open(),
                    "Cannot open: " << outDir << "/channel_results.csv");
    chanCsv << "time_s,nadir_lat_deg,nadir_lon_deg,user_id,beam_id,"
               "path_loss_dB,sinr_dB,snr_dB,beam_gain_dB\n";

    // -----------------------------------------------------------------------
    // RNG
    // -----------------------------------------------------------------------
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
        // Compute nadir lat/lon for this simulation time.
        double latNadir, lonNadir;
        GetNadirFromTime(timeS, cfg, latNadir, lonNadir);

        // In the nadir-centred local frame, the satellite is always directly
        // overhead.  No arc position look-up needed.
        const Vec3 satPos{0.0, 0.0, cfg.hSatelliteM};

        // Update config centre to the current nadir so that GetLatLon() inside
        // the channel model converts local user coordinates correctly.
        SimConfig localCfg = cfg;
        localCfg.latitudeCenterDeg  = latNadir;
        localCfg.longitudeCenterDeg = lonNadir;

        // Write nadir position row.
        satCsv << std::fixed << std::setprecision(9)
               << timeS << "," << latNadir << "," << lonNadir << "\n";

        // Compute all user link results for this snapshot.
        const auto results = ComputeFrameResults(
            satPos, userPos, beamCenters, localCfg, rng, withFading);

        // Collect SINR/SNR for console summary.
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

        // Append to channel CSV.
        AppendChannelCsvRows(chanCsv, timeS, latNadir, lonNadir, results);

        // Write JSON for this snapshot.
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
