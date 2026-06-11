/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-constellation-scanner.cc
 *
 * Pass A  — pure C++ loop  : coarse elevation screen for all 66 satellites.
 * Pass B  — ns-3 event-driven: per-satellite SNR events via Simulator::Schedule;
 *            all qualifying satellites run concurrently within one
 *            Simulator::Run() call.
 *
 * Design rationale for event-driven Pass B:
 *   - Consistent with ns-3 philosophy (time is managed by the scheduler).
 *   - All satellites are naturally concurrent: each schedules its own events
 *     independently within its pass window.
 *   - Easily extended to add ns-3 network nodes, packet flows, or mobility
 *     models alongside the channel computation.
 *   - Pass A stays pure C++ because it is a pre-processing step (fast,
 *     O(66 × 360) SGP4 calls) that determines which satellites qualify before
 *     any events are scheduled.
 */

#include "sat-constellation-scanner.h"

#include "sat-multi-beam-channel.h"

#include "sgp4unit.h"

#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/simulator.h"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatConstellationScanner");

NS_OBJECT_ENSURE_REGISTERED(SatConstellationScanner);

TypeId
SatConstellationScanner::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatConstellationScanner")
            .SetParent<Object>()
            .SetGroupName("Scratch")
            .AddConstructor<SatConstellationScanner>();
    return tid;
}

SatConstellationScanner::SatConstellationScanner(const SatTleReader& tleReader)
    : m_tleReader(&tleReader)
{
}

void
SatConstellationScanner::SetTleReader(const SatTleReader& tleReader)
{
    m_tleReader = &tleReader;
}

// ---------------------------------------------------------------------------
// ComputeElevation — ECI → ENU → elevation (used by Pass A)
// ---------------------------------------------------------------------------

double
SatConstellationScanner::ComputeElevation(const SatTleEntry& sat,
                                           double             tsinceMin,
                                           double             epochJd,
                                           double             roiLatDeg,
                                           double             roiLonDeg,
                                           double             rEarthM) const
{
    double r[3], v[3];
    elsetrec satrec = sat.satrec;
    if (!sgp4(wgs72, satrec, tsinceMin, r, v) || satrec.error != 0)
    {
        return -90.0;
    }

    const double jdNow = epochJd + tsinceMin / 1440.0;
    const Vec3 eciKm{r[0], r[1], r[2]};
    const Vec3 ecefKm = EciToEcef(eciKm, jdNow);
    const Vec3 ecefM{ecefKm.x * 1e3, ecefKm.y * 1e3, ecefKm.z * 1e3};

    const double latR = roiLatDeg * M_PI / 180.0;
    const double lonR = roiLonDeg * M_PI / 180.0;
    const Vec3 obsEcef{
        rEarthM * std::cos(latR) * std::cos(lonR),
        rEarthM * std::cos(latR) * std::sin(lonR),
        rEarthM * std::sin(latR)
    };
    const Vec3 delta{ecefM.x - obsEcef.x, ecefM.y - obsEcef.y, ecefM.z - obsEcef.z};
    return GetElevationAngleDeg_3D(EcefOffsetToEnu(delta, roiLatDeg, roiLonDeg));
}

// ---------------------------------------------------------------------------
// ScanSnrCallback — ns-3 event handler (Pass B)
//
// Fired once per (satellite, timestep) by the ns-3 scheduler.
// Computes satellite ENU, checks elevation, runs UPA channel model,
// appends one row per cell to the satellite's CSV.
// ---------------------------------------------------------------------------

static void
ScanSnrCallback(SatScanState* state, double timeS)
{
    const double tsinceMin = timeS / 60.0;
    const double jdNow     = state->epochJd + tsinceMin / 1440.0;

    double r[3], v[3];
    elsetrec satrec = state->sat->satrec;
    if (!sgp4(wgs72, satrec, tsinceMin, r, v) || satrec.error != 0) { return; }

    const Vec3 eciKm{r[0], r[1], r[2]};
    const Vec3 ecefKm = EciToEcef(eciKm, jdNow);
    const Vec3 ecefM{ecefKm.x * 1e3, ecefKm.y * 1e3, ecefKm.z * 1e3};

    const Vec3 delta{
        ecefM.x - state->obsEcef.x,
        ecefM.y - state->obsEcef.y,
        ecefM.z - state->obsEcef.z
    };
    const Vec3 satEnu = EcefOffsetToEnu(delta, state->roiLatDeg, state->roiLonDeg);

    const double elevDeg = GetElevationAngleDeg_3D(satEnu);
    if (elevDeg < state->minElevDeg) { return; }
    // Invariant: elevation >= minElevDeg; GetEllipticBeamCenters must not receive
    // elevations below minElevDeg.  ConstellationScanConfig::minElevDeg is the
    // single true source — no floor inside GetEllipticBeamCenters.
    NS_ASSERT(elevDeg >= state->minElevDeg);

    // 25-cell 5×5 elliptic beam centers (along-track/cross-track aligned).
    // GetEllipticBeamCenters computes the max inscribed rectangle in the elliptic
    // footprint and divides it into a 5×5 grid.  cellPos = beamCenters → ΔΦ = 0
    // → peak DirichletKernel gain for all 25 cells.
    const auto beamCentersEnu = GetEllipticBeamCenters(
        satEnu,
        state->hasPrevSat ? state->prevSatEnu : satEnu,
        state->cfg);

    state->prevSatEnu = satEnu;
    state->hasPrevSat = true;

    const std::vector<Vec3> tickCellPos(beamCentersEnu.begin(), beamCentersEnu.end());

    const auto results = ComputeFrameResults(
        satEnu, tickCellPos, beamCentersEnu,
        state->cfg, state->rng, false);

    for (std::size_t i = 0; i < results.size(); ++i)
    {
        // Convert beam-centre ENU position to geographic lat/lon so that
        // check_coverage.py can group co-located beams from different satellites
        // by actual position rather than by cell_idx (which is satellite-relative).
        double cellLat, cellLon;
        GetLatLon(tickCellPos[i], state->cfg, cellLat, cellLon);

        state->csv << std::fixed << std::setprecision(6)
                   << timeS << "," << i << ","
                   << cellLat << "," << cellLon << ","
                   << std::setprecision(4) << elevDeg << ","
                   << results[i].pathLossDb << ","
                   << results[i].beamGainDb << ","
                   << results[i].snrDb << ","
                   << results[i].sinrDb << "\n";
    }
    ++state->nWritten;
}

// ---------------------------------------------------------------------------
// Run
// ---------------------------------------------------------------------------

std::vector<SatPassInfo>
SatConstellationScanner::Run(const ConstellationScanConfig& config)
{
    NS_ABORT_MSG_IF(m_tleReader == nullptr, "SatTleReader must be set before Run()");

    std::filesystem::create_directories(config.outDir);

    // Cell positions are computed per-tick in ScanSnrCallback via
    // GetEllipticBeamCenters(satEnu, prevSatEnu, cfg).  The 5×5 elliptic grid
    // adapts to the satellite's elevation and along-track direction at each second.
    // No static pre-computation needed.
    const int nInFP = 25;  // 5×5 elliptic grid, always 25 cells

    const double epochJd = m_tleReader->GetEpochJd();
    const double rEarthM = config.cfg.rEarthM;
    const double latR    = config.roiLatDeg * M_PI / 180.0;
    const double lonR    = config.roiLonDeg * M_PI / 180.0;
    const Vec3 obsEcef{
        rEarthM * std::cos(latR) * std::cos(lonR),
        rEarthM * std::cos(latR) * std::sin(lonR),
        rEarthM * std::sin(latR)
    };

    std::cout << "[Constellation] 5x5 elliptic grid  n_cells=" << nInFP
              << "  centre lat=" << config.roiLatDeg
              << " lon=" << config.roiLonDeg << "\n";

    // -----------------------------------------------------------------------
    // Pass A — coarse elevation screen (pure C++)
    // -----------------------------------------------------------------------
    std::vector<SatPassInfo>        passes;
    std::vector<const SatTleEntry*> qualifyingSats;

    for (const auto& sat : m_tleReader->GetSatellites())
    {
        double peakElev = -90.0, peakElevT = 0.0;
        double winStart = -1.0,  winEnd    = -1.0;

        for (double t = 0.0; t <= config.windowS; t += config.dtScreenS)
        {
            const double elev = ComputeElevation(
                sat, t / 60.0, epochJd,
                config.roiLatDeg, config.roiLonDeg, rEarthM);

            if (elev >= config.minElevDeg)
            {
                if (winStart < 0.0) { winStart = t; }
                winEnd = t;
                if (elev > peakElev) { peakElev = elev; peakElevT = t; }
            }
        }

        if (winStart < 0.0) { continue; }

        std::cout << "[Constellation] " << sat.name
                  << "  elev_peak=" << std::fixed << std::setprecision(1) << peakElev
                  << "°  window=[" << winStart << "," << winEnd << "]s\n";

        SatPassInfo info;
        info.satIndex      = sat.satIndex;
        info.satName       = sat.name;
        info.windowStartS  = winStart;
        info.windowEndS    = winEnd;
        info.peakElevDeg   = peakElev;
        info.peakElevTimeS = peakElevT;
        info.nCellsCovered = nInFP;
        passes.push_back(info);
        qualifyingSats.push_back(&sat);
    }

    // -----------------------------------------------------------------------
    // Compute actual satellite altitude for each qualifying satellite.
    //
    // For each qualifying satellite, call SGP4 at its peak elevation time to
    // get the ECI position, convert to ECEF, and compute:
    //   h = |r_ecef| − R_earth
    //
    // This derives the true orbital altitude from TLE data so that the Python
    // link budget analysis (link_budget.py) can use the correct value instead
    // of a hardcoded constant.  The mean altitude is written to
    // constellation_status.json as "h_satellite_km".
    // -----------------------------------------------------------------------
    for (std::size_t idx = 0; idx < passes.size(); ++idx)
    {
        const double tPeakMin = passes[idx].peakElevTimeS / 60.0;
        const double jdPeak   = epochJd + tPeakMin / 1440.0;

        double r[3], v[3];
        elsetrec satrec = qualifyingSats[idx]->satrec;
        if (!sgp4(wgs72, satrec, tPeakMin, r, v) || satrec.error != 0) { continue; }

        // |r_ecef| in metres → subtract Earth radius
        const Vec3  ecefKm = EciToEcef(Vec3{r[0], r[1], r[2]}, jdPeak);
        const double rMagM = std::sqrt(ecefKm.x * ecefKm.x +
                                        ecefKm.y * ecefKm.y +
                                        ecefKm.z * ecefKm.z) * 1e3;
        passes[idx].altitudeM = rMagM - rEarthM;
    }

    // -----------------------------------------------------------------------
    // Pass B — fine SNR scan via Simulator::Schedule
    //
    // One SatScanState per qualifying satellite; all states live on the heap
    // and are kept alive for the duration of Simulator::Run().
    // -----------------------------------------------------------------------
    std::vector<std::unique_ptr<SatScanState>> states;
    states.reserve(passes.size());

    for (std::size_t idx = 0; idx < passes.size(); ++idx)
    {
        const SatPassInfo& passInfo = passes[idx];  // renamed: avoids #define pi collision

        // CSV path
        std::ostringstream ss;
        ss << config.outDir << "/sat_"
           << std::setw(5) << std::setfill('0') << passInfo.satIndex
           << "_cells.csv";

        // Per-satellite state (heap-allocated)
        auto state = std::make_unique<SatScanState>();
        state->sat            = qualifyingSats[idx];
        state->epochJd        = epochJd;
        state->obsEcef        = obsEcef;
        state->roiLatDeg      = config.roiLatDeg;
        state->roiLonDeg      = config.roiLonDeg;
        state->minElevDeg     = config.minElevDeg;
        state->cfg            = config.cfg;
        // cellPos unused; beam centers recomputed per-tick in ScanSnrCallback.
        state->hasPrevSat     = false;
        state->rng            = std::mt19937(config.seed + static_cast<uint32_t>(idx));

        state->csv.open(ss.str(), std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!state->csv.is_open(), "Cannot open: " << ss.str());
        // sinr_allbeams_dB: SINR assuming all 25 beams active simultaneously.
        // In BH mode only 1 beam per slot is active, so SINR ≈ SNR.
        // Use snr_dB for BH scheduling decisions; sinr_allbeams_dB is diagnostic.
        // cell_lat_deg / cell_lon_deg: geographic position of the beam centre (WGS84).
        // Used by check_coverage.py to group co-located beams across satellites
        // for physically correct MRC combining (fixes cell_idx ambiguity).
        state->csv << "time_s,cell_idx,cell_lat_deg,cell_lon_deg,elevation_deg,path_loss_dB,"
                      "beam_gain_dB,snr_dB,sinr_allbeams_dB\n";

        // Pre-compute prevSatEnu at (windowStart − dtSnrS) so that the first
        // scheduled tick has a valid along-track direction without any East fallback.
        // This eliminates the degenerate direction in GetEllipticBeamCenters.
        {
            const double tPrev    = passInfo.windowStartS - config.dtSnrS;
            const double tPrevMin = tPrev / 60.0;
            const double jdPrev   = epochJd + tPrevMin / 1440.0;
            double rp[3], vp[3];
            elsetrec satrecPrev = qualifyingSats[idx]->satrec;
            if (tPrev >= 0.0 &&
                sgp4(wgs72, satrecPrev, tPrevMin, rp, vp) &&
                satrecPrev.error == 0)
            {
                const Vec3 eciPrev{rp[0], rp[1], rp[2]};
                const Vec3 ecefPrev  = EciToEcef(eciPrev, jdPrev);
                const Vec3 ecefPrevM{ecefPrev.x * 1e3, ecefPrev.y * 1e3, ecefPrev.z * 1e3};
                const Vec3 deltaPrev{
                    ecefPrevM.x - obsEcef.x,
                    ecefPrevM.y - obsEcef.y,
                    ecefPrevM.z - obsEcef.z
                };
                state->prevSatEnu = EcefOffsetToEnu(deltaPrev, config.roiLatDeg, config.roiLonDeg);
                state->hasPrevSat = true;
            }
        }

        // Schedule one event per dtSnrS second within the padded window.
        // All satellites schedule concurrently; the ns-3 scheduler fires them
        // in chronological order across all satellites.
        SatScanState* rawPtr = state.get();
        const double tScanStart = std::max(0.0, passInfo.windowStartS - config.dtScreenS);
        const double tScanEnd   = passInfo.windowEndS + config.dtScreenS;
        for (double t = tScanStart; t <= tScanEnd; t += config.dtSnrS)
        {
            Simulator::Schedule(Seconds(t), &ScanSnrCallback, rawPtr, t);
        }

        states.push_back(std::move(state));
    }

    // Run all events then clean up the ns-3 scheduler.
    // Stop after the last possible padded event: windowS + dtScreenS.
    Simulator::Stop(Seconds(config.windowS + config.dtScreenS));
    Simulator::Run();
    Simulator::Destroy();

    // Collect results and report
    for (std::size_t idx = 0; idx < passes.size(); ++idx)
    {
        passes[idx].nSnrSamples = states[idx]->nWritten;
        std::cout << "  wrote sat_"
                  << std::setw(5) << std::setfill('0') << passes[idx].satIndex
                  << "_cells.csv"
                  << "  (" << passes[idx].nSnrSamples
                  << " timesteps × " << passes[idx].nCellsCovered << " cells)\n";
    }

    WriteStatusJson(config.outDir, config, passes);

    std::cout << "[Constellation] Done. " << passes.size()
              << " satellites qualified (elevation > " << config.minElevDeg << "°).\n";

    return passes;
}

// ---------------------------------------------------------------------------
// WriteStatusJson
// ---------------------------------------------------------------------------

void
SatConstellationScanner::WriteStatusJson(
    const std::string&              outDir,
    const ConstellationScanConfig&  config,
    const std::vector<SatPassInfo>& passes) const
{
    const std::string path = outDir + "/constellation_status.json";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        std::cerr << "[Constellation] Cannot write: " << path << "\n";
        return;
    }

    // Compute mean satellite altitude across all qualifying passes
    double altSum = 0.0;
    int    altCnt = 0;
    for (const auto& p : passes)
    {
        if (p.altitudeM > 0.0) { altSum += p.altitudeM; ++altCnt; }
    }
    const double meanAltKm = (altCnt > 0) ? altSum / altCnt / 1e3 : 0.0;

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"roi_lat_deg\": "   << config.roiLatDeg  << ",\n";
    out << "  \"roi_lon_deg\": "   << config.roiLonDeg  << ",\n";
    out << "  \"grid_d\": "        << config.gridD       << ",\n";
    out << "  \"window_s\": "      << config.windowS     << ",\n";
    out << "  \"dt_screen_s\": "   << config.dtScreenS   << ",\n";
    out << "  \"dt_snr_s\": "      << config.dtSnrS      << ",\n";
    out << "  \"min_elev_deg\": "  << config.minElevDeg  << ",\n";
    out << "  \"n_qualifying\": "  << passes.size()      << ",\n";
    out << "  \"h_satellite_km\": " << std::setprecision(3) << meanAltKm << ",\n";
    out << "  \"passes\": [\n";

    out << std::setprecision(6);
    for (std::size_t i = 0; i < passes.size(); ++i)
    {
        const auto& p = passes[i];
        out << "    {\n";
        out << "      \"sat_index\": "        << p.satIndex             << ",\n";
        out << "      \"sat_name\": \""        << p.satName              << "\",\n";
        out << "      \"csv_file\": \"sat_";
        out << std::setw(5) << std::setfill('0') << p.satIndex;
        out << std::setfill(' ') << "_cells.csv\",\n";
        out << "      \"window_start_s\": "   << p.windowStartS         << ",\n";
        out << "      \"window_end_s\": "     << p.windowEndS           << ",\n";
        out << "      \"peak_elev_deg\": "    << p.peakElevDeg          << ",\n";
        out << "      \"peak_elev_time_s\": " << p.peakElevTimeS        << ",\n";
        out << "      \"altitude_km\": "
            << std::setprecision(3) << p.altitudeM / 1e3               << ",\n";
        out << std::setprecision(6);
        out << "      \"n_cells_covered\": "  << p.nCellsCovered        << ",\n";
        out << "      \"n_snr_samples\": "    << p.nSnrSamples          << "\n";
        out << "    }";
        if (i + 1 < passes.size()) { out << ","; }
        out << "\n";
    }

    out << "  ]\n}\n";
    std::cout << "[Constellation] wrote " << path << "\n";
}

} // namespace ns3 fix
