/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-phase2-dual.h
 *
 * Phase 2.2 / 2.3 — Dual-Satellite Grid ROI mode public API.
 *
 * Declares RunDualConfig (all parameters for a dual-satellite d×d ROI run)
 * and RunDualMode() which encapsulates:
 *   - Two SGP4 orbit CSV loads (sat[i] and sat[i+1], same time axis)
 *   - Per-cell SNR computation for both satellites simultaneously
 *   - Greedy best-of-two SNR baseline (Phase 2.4 comparison anchor)
 *   - Phase 2.3 overlap threshold detection (10/25/50/75/90% of ROI cells
 *     reached by sat[i+1] at SNR >= snrThreshDb)
 *
 * Outputs written by RunDualMode():
 *   dual_cell_results.csv  — per (time, cell) SNR for both satellites
 *   dual_cell_summary.csv  — per-cell mean/max SNR + greedy mean SNR
 *   dual_overlap.json      — Phase 2.3 first-crossing timestamps
 *
 * Internal state (DualCellStats, DualSimState, DualUpdateStep) is confined to
 * sat-phase2-dual.cc and not visible to callers.
 *
 * CMakeLists.txt: add sat-phase2-dual.cc to SOURCE_FILES of build_exec.
 *
 * Test command (run from ns3 root on VMware):
 *   ./ns3 run "sat-multi-beam-simulation \
 *     --mode=dual --d=5 \
 *     --orbit-csv-i=scratch/orbit_sat_i.csv \
 *     --orbit-csv-i1=scratch/orbit_sat_i1.csv \
 *     --out-dir=scratch/dual_d5_verify"
 */

#ifndef SAT_PHASE2_DUAL_H
#define SAT_PHASE2_DUAL_H

#include "sat-multi-beam-config.h"

#include <cstdint>
#include <string>

namespace ns3
{

/**
 * RunDualConfig — all parameters needed for a Phase 2.2/2.3 dual-satellite run.
 *
 * Both orbit CSVs must share the same 0-based time_s axis, which is guaranteed
 * by run_sgp4.py --mode=sequence output (both CSVs cover the same window with
 * the same step size).
 */
struct RunDualConfig
{
    SimConfig   cfg;              // satellite / channel / RF parameters
    std::string orbitCsvI;        // path to orbit_sat_i.csv (run_sgp4.py --mode=sequence)
    std::string orbitCsvI1;       // path to orbit_sat_i1.csv (same time axis)
    int         gridD{5};         // d×d grid dimension
    double      updateMs{100.0};  // ns3 event step (ms)
    double      minElevDeg{5.0};  // minimum elevation to process a frame (degrees)
    double      snrThreshDb{0.0}; // Phase 2.3: SNR threshold to count a cell as "covered"
    std::string outDir;           // output directory (created if absent)
    uint32_t    seed{42};         // RNG seed
};

/**
 * RunDualMode — execute the Phase 2.2/2.3 dual-satellite grid simulation.
 *
 * Loads both SGP4 orbit CSVs, builds the inscribed-square ROI grid, runs
 * the ns3 event loop (DualUpdateStep), and writes the three output files.
 * Phase 2.3 overlap detection is integrated inline in DualUpdateStep.
 *
 * Calls Simulator::Destroy() before returning.  Caller must not call it again.
 *
 * @param config  All Phase 2.2/2.3 run parameters.
 */
void RunDualMode(const RunDualConfig& config);

} // namespace ns3

#endif // SAT_PHASE2_DUAL_H
