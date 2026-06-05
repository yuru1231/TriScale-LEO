/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-constellation-scanner.h
 *
 * Scans all 66 Iridium-NEXT satellites over a fixed ROI and reports:
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
#include "ns3/type-id.h"

#include <array>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace ns3
{

// ---------------------------------------------------------------------------
// Public data structures
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
// Per-satellite runtime state (one instance per qualifying satellite)
// ---------------------------------------------------------------------------

/**
 * SatScanState — mutable state shared across all ns-3 events for one satellite.
 *
 * Allocated on the heap in Run(); pointer passed to each scheduled event.
 * Lifetime covers the entire Simulator::Run() call.
 */
struct SatScanState
{
    // Satellite identity and propagator
    const SatTleEntry*   sat{nullptr};
    double               epochJd{0.0};

    // Fixed geometry (same for every event of this satellite)
    Vec3                 obsEcef{};            // observer ECEF (m)
    double               roiLatDeg{0.0};
    double               roiLonDeg{0.0};
    double               minElevDeg{0.0};
    SimConfig            cfg{};
    std::vector<Vec3>    cellPos{};            // unused after Phase 2.1; beam centers recomputed per-tick

    // Phase 2.1: along-track direction from consecutive satellite positions
    Vec3                 prevSatEnu{};         // satellite ENU at previous tick
    bool                 hasPrevSat{false};    // true after first valid tick

    // Output
    std::ofstream        csv{};
    int                  nWritten{0};

    // RNG (per-satellite seed)
    std::mt19937         rng{};
};

// ---------------------------------------------------------------------------
// Scanner class
// ---------------------------------------------------------------------------

class SatConstellationScanner : public Object
{
public:
    /** ns-3 TypeId registration — required by NS_OBJECT_ENSURE_REGISTERED. */
    static TypeId GetTypeId();

    SatConstellationScanner() = default;

    /** Inject the TLE reader before calling Run(). */
    void SetTleReader(Ptr<SatTleReader> tleReader);

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
     */
    std::vector<SatPassInfo> Run(const ConstellationScanConfig& config);

private:
    Ptr<SatTleReader> m_tleReader{nullptr};

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
};

} // namespace ns3

#endif // SAT_CONSTELLATION_SCANNER_H
