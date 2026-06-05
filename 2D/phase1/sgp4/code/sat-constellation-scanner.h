/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-constellation-scanner.h
 *
 * ns3::Object that scans all 66 Iridium-NEXT satellites over a fixed ROI
 * and reports:
 *   (1) Which satellites pass above an elevation threshold.
 *   (2) Per-cell SNR for each qualifying pass.
 *
 * Algorithm (two-pass per satellite):
 *   Pass A — coarse elevation screen at dtScreenS intervals (pure C++).
 *             Finds which satellites qualify and their pass windows.
 *   Pass B — fine SNR scan via ns-3 Simulator::Schedule.
 *             Each qualifying satellite schedules one event per dtSnrS
 *             second within its pass window.  Simulator::Run() drives
 *             all satellites concurrently.
 *
 * Output files written to outDir:
 *   constellation_status.json   — index of all passes (name, window, peak elev)
 *   sat_XXXXX_cells.csv         — per-cell SNR time-series for each qualifying sat
 *                                  (XXXXX = zero-padded satIndex)
 *
 * Coordinate chain (ECI → ENU):
 *   1. sgp4(satrec, tsince_min) → r[3] (km, TEME/ECI)
 *   2. EciToEcef(r, jdUT1)      → ecefKm
 *   3. EcefDeltaToEnu(delta)    → enuM  (East, North, Up in metres)
 *   4. GetElevationAngleDeg_3D(enuM) → elevDeg
 */

#ifndef SAT_CONSTELLATION_SCANNER_H
#define SAT_CONSTELLATION_SCANNER_H

#include "sat-multi-beam-config.h"
#include "sat-multi-beam-geometry.h"
#include "sat-tle-reader.h"

#include "ns3/object.h"
#include "ns3/ptr.h"

#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace ns3
{

// ---------------------------------------------------------------------------
// Public data structures (plain structs — value types)
// ---------------------------------------------------------------------------

/** Summary of one qualifying satellite pass — written to constellation_status.json. */
struct SatPassInfo
{
    int         satIndex{0};
    std::string satName;
    double      windowStartS{0.0};
    double      windowEndS{0.0};
    double      peakElevDeg{0.0};
    double      peakElevTimeS{0.0};
    int         nCellsCovered{0};
    int         nSnrSamples{0};
};

/** All parameters for one Run() call. */
struct ConstellationScanConfig
{
    SimConfig   cfg;
    double      roiLatDeg{35.676};
    double      roiLonDeg{139.65};
    int         gridD{5};
    double      windowS{3600.0};
    double      dtScreenS{10.0};
    double      dtSnrS{1.0};
    double      minElevDeg{5.0};
    std::string outDir;
    uint32_t    seed{42};
};

// ---------------------------------------------------------------------------
// MRC accumulator — cross-satellite SNR combining (shared across all satellites)
// ---------------------------------------------------------------------------

/**
 * MrcCellSample — accumulated linear SNR for one (quantised-time, cell) pair.
 *
 * linearSnrSum = Σ_i SNR_i  (linear, unitless — sum before converting back to dB)
 * nSats        = number of satellites that contributed in this timestep
 */
struct MrcCellSample
{
    double linearSnrSum{0.0};
    int    nSats{0};
};

/**
 * MrcAccumulator — shared MRC state written by ScanSnrCallback for every sat.
 *
 * Key: timeKey = round(time_s × 10)  → avoids float map-key collisions for 0.1 s steps.
 * Value: per-cell vector indexed by cell_idx.
 *
 * Allocated in SatConstellationScanner::Run() and passed by raw pointer to
 * each SatScanState.  Lifetime spans the full Simulator::Run() call.
 */
struct MrcAccumulator
{
    // time_key → per-cell sample vector (indexed 0..nCells-1)
    std::map<int64_t, std::vector<MrcCellSample>> data;
    int nCells{0};   // total grid cells; set before scheduling
};

// ---------------------------------------------------------------------------
// Per-satellite runtime state (one instance per qualifying satellite)
// ---------------------------------------------------------------------------

/**
 * SatScanState — mutable state shared across all ns-3 events for one satellite.
 *
 * Allocated on the heap in Run(); pointer passed to each scheduled event.
 * Lifetime covers the entire Simulator::Run() call.
 *
 * Not an ns3::Object (allocated/owned by SatConstellationScanner, not by the
 * ns3 object system; raw pointer passed to event callbacks is intentional).
 */
struct SatScanState
{
    // Satellite identity and propagator (pointer into SatTleReader's vector)
    const SatTleEntry*   sat{nullptr};
    double               epochJd{0.0};

    // Fixed geometry (same for every event of this satellite)
    Vec3                 obsEcef{};
    double               roiLatDeg{0.0};
    double               roiLonDeg{0.0};
    double               minElevDeg{0.0};
    SimConfig            cfg{};
    std::vector<Vec3>    cellPos{};
    // beamCentersEnu removed: recomputed per-timestep in ScanSnrCallback
    // via GetBeamCentersFromSatPos(satEnu, cfg) for elliptical correction.

    // Output
    std::ofstream        csv{};
    int                  nWritten{0};

    // MRC: pointer to the shared accumulator owned by SatConstellationScanner.
    // Non-owning; valid for the duration of Simulator::Run().
    MrcAccumulator*      mrcAcc{nullptr};

    // RNG (per-satellite seed for channel fading)
    std::mt19937         rng{};
};

// ---------------------------------------------------------------------------
// Scanner class — ns3::Object
// ---------------------------------------------------------------------------

/**
 * SatConstellationScanner — ns3::Object wrapping the two-pass constellation scan.
 *
 * Usage:
 *   Ptr<SatConstellationScanner> scanner = CreateObject<SatConstellationScanner>();
 *   scanner->SetTleReader(tleReader);   // Ptr<SatTleReader>
 *   auto passes = scanner->Run(scanCfg);
 */
class SatConstellationScanner : public Object
{
public:
    /**
     * GetTypeId — ns3 TypeId registration.
     * Group: Satellite.
     */
    static TypeId GetTypeId();

    SatConstellationScanner()           = default;
    ~SatConstellationScanner() override = default;

    /**
     * SetTleReader — bind the TLE dataset to this scanner.
     *
     * Must be called before Run().
     * @param reader  Ptr to an already-loaded SatTleReader.
     */
    void SetTleReader(Ptr<SatTleReader> reader);

    /**
     * Run — execute the full constellation scan.
     *
     * Internally runs:
     *   1. Pure-C++ coarse screen  (finds qualifying satellites + windows)
     *   2. Simulator::Schedule     (schedules fine SNR events)
     *   3. Simulator::Run()        (fires all events)
     *   4. Simulator::Destroy()
     *
     * Writes constellation_status.json and per-sat CSVs to config.outDir.
     *
     * @param config  Scan parameters.
     * @return        Vector of SatPassInfo for each qualifying satellite.
     */
    std::vector<SatPassInfo> Run(const ConstellationScanConfig& config);

private:
    Ptr<SatTleReader> m_tleReader;       // bound TLE dataset
    MrcAccumulator    m_mrcAccumulator;  // shared MRC state; reset at each Run()

    /** Elevation angle (deg) from satellite to ROI centre at tsinceMin. */
    double ComputeElevation(const SatTleEntry& sat,
                             double             tsinceMin,
                             double             epochJd,
                             double             roiLatDeg,
                             double             roiLonDeg,
                             double             rEarthM) const;

    /** Write constellation_status.json. */
    void WriteStatusJson(const std::string&              outDir,
                         const ConstellationScanConfig&  config,
                         const std::vector<SatPassInfo>& passes) const;

    /**
     * WriteMrcCsv — flush the MRC accumulator to mrc_combined.csv.
     *
     * Called once after Simulator::Run() when all ScanSnrCallback events have fired.
     * Output columns: time_s, cell_idx, snr_mrc_dB, n_sats
     *   snr_mrc_dB = 10·log10(Σ SNR_i)   (MRC: linear sum of per-satellite SNRs)
     *   n_sats     = number of simultaneously visible satellites at that timestep
     */
    void WriteMrcCsv(const std::string& outDir) const;
};

} // namespace ns3

#endif // SAT_CONSTELLATION_SCANNER_H
