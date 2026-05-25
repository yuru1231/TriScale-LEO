/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-phase2-dual.cc
 *
 * Phase 2.2 / 2.3 — Dual-Satellite Grid ROI mode implementation.
 *
 * Implements RunDualMode() declared in sat-phase2-dual.h.
 * All internal types and callbacks are confined to the anonymous namespace.
 *
 * Design notes:
 *   - Both orbit CSVs must share the same 0-based time_s axis (same row count).
 *     Violation triggers NS_ABORT_MSG in RunDualMode().
 *   - Greedy SNR: at each step, for each in-footprint cell, the satellite with
 *     the higher SNR is selected.  This is the Phase 2.4 baseline assignment.
 *   - Phase 2.3 coverage detection: at each step, count cells where
 *     sat[i+1] SNR >= snrThreshDb.  Record the first crossing time for
 *     10/25/50/75/90% thresholds.  Written to dual_overlap.json.
 *   - Sentinel value -999.0 in dual_cell_results.csv marks "satellite not
 *     visible this step" (elevation < minElevDeg).
 */

#include "sat-phase2-dual.h"

#include "sat-multi-beam-channel.h"
#include "sat-multi-beam-geometry.h"
#include "sat-orbit-reader.h"
#include "sat-roi-grid.h"

#include "ns3/core-module.h"

#include <algorithm>
#include <array>
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
    double                   snrThreshDb{0.0}; // SNR >= 0 dB ≡ "covered" by sat[i+1]
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
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open dual_cell_summary.csv: " << fname);

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

            const double meanI  = (st.nSamplesSI  > 0) ? st.sumSnrSI  / st.nSamplesSI  : 0.0;
            const double meanI1 = (st.nSamplesSI1 > 0) ? st.sumSnrSI1 / st.nSamplesSI1 : 0.0;
            const double maxI   = (st.nSamplesSI  > 0) ? st.maxSnrSI  : 0.0;
            const double maxI1  = (st.nSamplesSI1 > 0) ? st.maxSnrSI1 : 0.0;
            const double greedy = (st.nGreedy     > 0) ? st.sumGreedySnr / st.nGreedy   : 0.0;

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
    NS_ABORT_MSG_IF(!out.is_open(), "Cannot open dual_overlap.json: " << fname);

    const std::array<std::string, 5> keys{"10pct", "25pct", "50pct", "75pct", "90pct"};

    out << std::fixed << std::setprecision(3);
    out << "{\n";
    out << "  \"n_in_footprint\": "   << s->nInFootprint  << ",\n";
    out << "  \"snr_threshold_dB\": " << s->snrThreshDb   << ",\n";
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
        // Provide zero-filled result vectors for non-visible satellite; CSV
        // writer uses visI/visI1 flags and writes -999 sentinels instead.
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
        const int      gIdx = static_cast<int>(s->cellIdxMap[i]);
        DualCellStats& st   = s->stats[gIdx];

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
    const double nextT  = t + s->updateMs / 1000.0;
    const double winEnd = std::max(s->orbitI.back().timeS, s->orbitI1.back().timeS);
    if (nextT <= winEnd)
    {
        Simulator::Schedule(MilliSeconds(s->updateMs), &DualUpdateStep, s);
    }
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void
RunDualMode(const RunDualConfig& config)
{
    std::filesystem::create_directories(config.outDir);

    // Load both orbit CSVs
    std::cout << "[dual mode] Loading sat[i]   orbit: " << config.orbitCsvI  << "\n";
    const auto orbitI  = LoadOrbit(config.orbitCsvI);
    NS_ABORT_MSG_IF(orbitI.empty(), "orbit-csv-i is empty: " << config.orbitCsvI);
    std::cout << "  " << orbitI.size() << " points"
              << "  t=[" << orbitI.front().timeS << "s, " << orbitI.back().timeS << "s]\n";

    std::cout << "[dual mode] Loading sat[i+1] orbit: " << config.orbitCsvI1 << "\n";
    const auto orbitI1 = LoadOrbit(config.orbitCsvI1);
    NS_ABORT_MSG_IF(orbitI1.empty(), "orbit-csv-i1 is empty: " << config.orbitCsvI1);
    std::cout << "  " << orbitI1.size() << " points"
              << "  t=[" << orbitI1.front().timeS << "s, " << orbitI1.back().timeS << "s]\n";

    NS_ABORT_MSG_IF(orbitI.size() != orbitI1.size(),
                    "orbit-csv-i and orbit-csv-i1 must have the same row count "
                    "(same time axis).  Got " << orbitI.size()
                    << " vs " << orbitI1.size() << ".  "
                    "Re-run run_sgp4.py --mode=sequence to regenerate.");

    // Build d×d ROI grid
    const RoiGrid grid  = GenerateRoiGrid(config.gridD, config.cfg.rFootprintM);
    const int     nInFP = static_cast<int>(
        std::count_if(grid.cells.begin(), grid.cells.end(),
                      [](const RoiCell& c) { return c.inFootprint; }));

    std::cout << "[dual mode] " << config.gridD << "×" << config.gridD
              << " grid = " << (config.gridD * config.gridD) << " cells"
              << "  in-footprint = " << nInFP << "\n"
              << "  L = W = " << grid.L_m / 1e3 << " km"
              << "  cell = " << grid.cellL / 1e3 << " km × "
              << grid.cellW / 1e3 << " km\n";

    const auto cellPos    = GetRoiCellPositions(grid);
    const auto cellIdxMap = GetRoiCellIndexMap(grid);

    // Static beam centres (elevation=90° approximation; Phase 2.5 z-rotation
    // in channel.cc corrects azimuth error at lower elevations)
    const auto beamCentersEcef = GetHexBeamCenters(config.cfg);
    std::array<Vec3, 19> beamCentersEnu;
    for (int i = 0; i < 19; ++i)
    {
        beamCentersEnu[i] = EcefOffsetToEnu(
            beamCentersEcef[i],
            config.cfg.latitudeCenterDeg,
            config.cfg.longitudeCenterDeg);
    }

    auto* s = new DualSimState();
    s->cfg             = config.cfg;
    s->grid            = grid;
    s->cellPositions   = cellPos;
    s->cellIdxMap      = cellIdxMap;
    s->beamCentersEnu  = beamCentersEnu;
    s->orbitI          = orbitI;
    s->orbitI1         = orbitI1;
    s->rng             = std::mt19937(config.seed);
    s->updateMs        = config.updateMs;
    s->minElevDeg      = config.minElevDeg;
    s->snrThreshDb     = config.snrThreshDb;
    s->nInFootprint    = nInFP;
    s->outDir          = config.outDir;
    s->stats.assign(config.gridD * config.gridD, DualCellStats{});

    s->dualCsv.open(config.outDir + "/dual_cell_results.csv",
                    std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!s->dualCsv.is_open(),
                    "Cannot open dual_cell_results.csv in: " << config.outDir);
    s->dualCsv << "time_s,row,col,cx_m,cy_m,"
                  "elev_i_deg,snr_i_dB,sinr_i_dB,"
                  "elev_i1_deg,snr_i1_dB,sinr_i1_dB,"
                  "best_sat\n";

    std::cout << "\n[dual mode]"
              << "  obs=(" << config.cfg.latitudeCenterDeg << "°N, "
              << config.cfg.longitudeCenterDeg << "°E)"
              << "  step=" << config.updateMs << "ms"
              << "  min_elev=" << config.minElevDeg << "°"
              << "  in-FP cells=" << nInFP
              << "  snr_thresh=" << config.snrThreshDb << " dB\n";

    Simulator::Schedule(Seconds(0.0), &DualUpdateStep, s);
    Simulator::Run();

    s->dualCsv.close();
    std::cout << "  wrote " << config.outDir << "/dual_cell_results.csv\n";

    WriteDualSummaryCsv(s);
    WriteDualOverlapJson(s);

    delete s;
    Simulator::Destroy();
    std::cout << "\nDone.  Output in: " << config.outDir << "\n";
}

} // namespace ns3
