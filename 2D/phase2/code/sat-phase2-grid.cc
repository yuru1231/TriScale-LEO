/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-phase2-grid.cc
 *
 * Phase 2.0 — Grid ROI mode implementation.
 *
 * Implements RunGridMode() declared in sat-phase2-grid.h.
 * All internal types (CellStats, GridSimState) and callbacks (GridUpdateStep,
 * AppendGridCellCsvRows, WriteGridSummaryCsv) are confined to the anonymous
 * namespace and not visible outside this translation unit.
 *
 * Design notes:
 *   - The d×d ROI grid uses an inscribed square: L = W = r_footprint × √2.
 *   - Only cells whose centre lies within the circular footprint (cx²+cy²≤r²)
 *     are processed; out-of-footprint cells get coverage_s = 0 in the summary.
 *   - Satellite elevation is read from the SGP4 orbit CSV (PyEphem value).
 *   - Beam centres are fixed at the elevation=90° approximation (same as
 *     Phase 2.0 spec); the Phase 2.5 z-rotation correction in channel.cc
 *     handles the azimuth error at lower elevations.
 */

#include "sat-phase2-grid.h"

#include "sat-multi-beam-channel.h"
#include "sat-multi-beam-geometry.h"
#include "sat-orbit-reader.h"
#include "sat-roi-grid.h"

#include "ns3/core-module.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace ns3
{

// ---------------------------------------------------------------------------
// Internal types and helpers (not exported)
// ---------------------------------------------------------------------------

namespace
{

/**
 * CellStats — per-grid-cell running statistics for the pass summary.
 * Indexed by (row * d + col) across all d*d cells.
 */
struct CellStats
{
    double coverageS{0.0};   // total simulated time with elevation >= threshold (s)
    double sumSnrDb{0.0};    // cumulative SNR for mean computation
    double minSnrDb{1e9};    // minimum observed SNR
    double maxSnrDb{-1e9};   // maximum observed SNR
    int    nSamples{0};      // number of in-coverage samples
};

/**
 * GridSimState — all state needed by GridUpdateStep.
 *
 * The observation point (ROI centre) is fixed at cfg.latitudeCenterDeg /
 * cfg.longitudeCenterDeg.  The satellite position is read from the pre-computed
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
    std::vector<Vec3>      cellPositions;   // in-footprint ENU positions (user list)
    std::vector<size_t>    cellIdxMap;      // cellPositions[i] → grid.cells[] index
    std::array<Vec3, 19>   beamCentersEnu;  // beam centres in ENU (elevation=90° approx)
    std::vector<OrbitPoint> orbit;          // full pre-computed SGP4 orbit
    size_t                 orbitIdx{0};     // current lookup position in orbit
    std::ofstream          cellCsv;         // cell_results.csv
    std::vector<CellStats> stats;           // per-cell accumulation (all d*d cells)
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
AppendGridCellCsvRows(std::ofstream&                     csvOut,
                      double                             timeS,
                      double                             elevDeg,
                      const GridSimState*                s,
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
            const int        idx  = row * d + col;
            const RoiCell&   cell = s->grid.cells[idx];
            const CellStats& st   = s->stats[idx];

            const double meanSnr = (st.nSamples > 0) ? st.sumSnrDb / st.nSamples : 0.0;
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
 *   3. If elevation >= minElevDeg and cells exist:
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

    const OrbitPoint& pt      = s->orbit[s->orbitIdx];
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

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void
RunGridMode(const RunGridConfig& config)
{
    std::filesystem::create_directories(config.outDir);

    // Load pre-computed SGP4 orbit
    std::cout << "[grid mode] Loading orbit CSV: " << config.orbitCsv << "\n";
    const auto orbit = LoadOrbit(config.orbitCsv);
    NS_ABORT_MSG_IF(orbit.empty(), "Orbit CSV is empty: " << config.orbitCsv);
    std::cout << "  " << orbit.size() << " orbit points"
              << "  t_start=" << orbit.front().timeS << "s"
              << "  t_end="   << orbit.back().timeS  << "s\n";

    // Build d×d ROI grid (Phase 2.0: inscribed square, L = W = r × √2)
    const RoiGrid grid  = GenerateRoiGrid(config.gridD, config.cfg.rFootprintM);
    const int     nInFP = static_cast<int>(
        std::count_if(grid.cells.begin(), grid.cells.end(),
                      [](const RoiCell& c) { return c.inFootprint; }));

    std::cout << "[grid mode] " << config.gridD << "×" << config.gridD
              << " grid = " << (config.gridD * config.gridD) << " cells"
              << "  in-footprint = " << nInFP << "\n"
              << "  L = W = " << grid.L_m / 1e3 << " km"
              << "  cell = " << grid.cellL / 1e3 << " km × "
              << grid.cellW / 1e3 << " km\n";

    // In-footprint cell positions and index map for result attribution
    const auto cellPos    = GetRoiCellPositions(grid);
    const auto cellIdxMap = GetRoiCellIndexMap(grid);

    // Beam centres: convert ECEF-offset → ENU so all positions share the
    // same coordinate system as OrbitPointToEnuVec3() output.
    const auto beamCentersEcef = GetHexBeamCenters(config.cfg);
    std::array<Vec3, 19> beamCentersEnu;
    for (int i = 0; i < 19; ++i)
    {
        beamCentersEnu[i] = EcefOffsetToEnu(
            beamCentersEcef[i],
            config.cfg.latitudeCenterDeg,
            config.cfg.longitudeCenterDeg);
    }

    auto* s = new GridSimState();
    s->cfg            = config.cfg;
    s->grid           = grid;
    s->cellPositions  = cellPos;
    s->cellIdxMap     = cellIdxMap;
    s->beamCentersEnu = beamCentersEnu;
    s->orbit          = orbit;
    s->rng            = std::mt19937(config.seed);
    s->updateMs       = config.updateMs;
    s->minElevDeg     = config.minElevDeg;
    s->outDir         = config.outDir;
    s->stats.assign(config.gridD * config.gridD, CellStats{});

    s->cellCsv.open(config.outDir + "/cell_results.csv",
                    std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!s->cellCsv.is_open(),
                    "Cannot open cell_results.csv in: " << config.outDir);
    s->cellCsv << "time_s,row,col,cx_m,cy_m,elevation_deg,in_footprint,"
                  "path_loss_dB,beam_gain_dB,snr_dB,sinr_dB\n";

    std::cout << "\n[grid mode]"
              << "  obs=(" << config.cfg.latitudeCenterDeg << "°N, "
              << config.cfg.longitudeCenterDeg << "°E)"
              << "  step=" << config.updateMs << "ms"
              << "  min_elev=" << config.minElevDeg << "°"
              << "  in-FP cells=" << nInFP << "\n";

    Simulator::Schedule(Seconds(0.0), &GridUpdateStep, s);
    Simulator::Run();

    s->cellCsv.close();
    std::cout << "  wrote " << config.outDir << "/cell_results.csv\n";

    WriteGridSummaryCsv(s);

    delete s;
    Simulator::Destroy();
    std::cout << "\nDone.  Output in: " << config.outDir << "\n";
}

} // namespace ns3
