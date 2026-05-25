/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-phase2-grid.h
 *
 * Phase 2.0 — Grid ROI mode public API.
 *
 * Declares RunGridConfig (all parameters needed to run a d×d ROI simulation
 * with a single SGP4 orbit CSV) and RunGridMode() which encapsulates the
 * full Phase 2.0 ns3 event-driven simulation, including:
 *   - Orbit CSV loading
 *   - RoiGrid construction (inscribed square, L = W = r_footprint × √2)
 *   - Per-cell SNR accumulation via GridUpdateStep
 *   - cell_results.csv and cell_summary.csv output
 *
 * Internal implementation details (CellStats, GridSimState, GridUpdateStep)
 * are kept in sat-phase2-grid.cc to avoid exposing simulation state to main.
 *
 * CMakeLists.txt: add sat-phase2-grid.cc to SOURCE_FILES of build_exec.
 *
 * Test command (run from ns3 root on VMware):
 *   ./ns3 run "sat-multi-beam-simulation \
 *     --mode=grid --d=5 \
 *     --orbit-csv=scratch/orbit_sat_i.csv \
 *     --out-dir=scratch/grid_verify"
 */

#ifndef SAT_PHASE2_GRID_H
#define SAT_PHASE2_GRID_H

#include "sat-multi-beam-config.h"

#include <cstdint>
#include <string>

namespace ns3
{

/**
 * RunGridConfig — all parameters needed for a Phase 2.0 grid mode run.
 *
 * Populated by main() from CommandLine values and passed to RunGridMode().
 * Keeping parameters here (instead of inlining into RunGridMode's signature)
 * allows main() to remain readable when both grid and dual configs are built
 * in the same dispatch block.
 */
struct RunGridConfig
{
    SimConfig   cfg;              // satellite / channel / RF parameters
    std::string orbitCsv;         // path to satellite_orbit.csv (run_sgp4.py output)
    int         gridD{5};         // d×d grid dimension
    double      updateMs{100.0};  // ns3 event step (ms)
    double      minElevDeg{5.0};  // minimum elevation angle to log a frame (degrees)
    std::string outDir;           // output directory (created if absent)
    uint32_t    seed{42};         // RNG seed
};

/**
 * RunGridMode — execute the Phase 2.0 d×d ROI grid simulation.
 *
 * Loads the SGP4 orbit CSV, builds the inscribed-square ROI grid, runs the
 * ns3 event loop (GridUpdateStep), and writes:
 *   cell_results.csv   — per (time, cell) SNR/SINR rows
 *   cell_summary.csv   — per-cell mean/min/max SNR + coverage
 *
 * Calls Simulator::Destroy() before returning.  Caller must not call it again.
 *
 * @param config  All Phase 2.0 run parameters.
 */
void RunGridMode(const RunGridConfig& config);

} // namespace ns3

#endif // SAT_PHASE2_GRID_H
