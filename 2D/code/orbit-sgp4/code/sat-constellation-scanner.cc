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

void
SatConstellationScanner::SetTleReader(Ptr<SatTleReader> tleReader)
{
    m_tleReader = tleReader;
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

    // Phase 2.1 — 25-cell 5×5 elliptic beam centers (along-track/cross-track aligned).
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
        state->csv << std::fixed << std::setprecision(6)
                   << timeS << "," << i << ","
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
    std::filesystem::create_directories(config.outDir);

    // Phase 2.1: cell positions are computed per-tick in ScanSnrCallback via
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

    std::cout << "[Constellation] Phase 2.1 — 5x5 elliptic grid  n_cells=" << nInFP
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
        // Phase 2.1: cellPos unused; beam centers recomputed per-tick in ScanSnrCallback.
        state->hasPrevSat     = false;
        state->rng            = std::mt19937(config.seed + static_cast<uint32_t>(idx));

        state->csv.open(ss.str(), std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!state->csv.is_open(), "Cannot open: " << ss.str());
        state->csv << "time_s,cell_idx,elevation_deg,path_loss_dB,"
                      "beam_gain_dB,snr_dB,sinr_dB\n";

        // Schedule one event per dtSnrS second within this satellite's window.
        // All satellites schedule concurrently; the ns-3 scheduler fires them
        // in chronological order across all satellites.
        SatScanState* rawPtr = state.get();
        for (double t = passInfo.windowStartS; t <= passInfo.windowEndS; t += config.dtSnrS)
        {
            Simulator::Schedule(Seconds(t), &ScanSnrCallback, rawPtr, t);
        }

        states.push_back(std::move(state));
    }

    // Run all events then clean up the ns-3 scheduler
    Simulator::Stop(Seconds(config.windowS));
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
    out << "  \"passes\": [\n";

    for (std::size_t i = 0; i < passes.size(); ++i)
    {
        const auto& p = passes[i];
        out << "    {\n";
        out << "      \"sat_index\": "        << p.satIndex      << ",\n";
        out << "      \"sat_name\": \""        << p.satName       << "\",\n";
        out << "      \"csv_file\": \"sat_";
        out << std::setw(5) << std::setfill('0') << p.satIndex;
        out << std::setfill(' ') << "_cells.csv\",\n";
        out << "      \"window_start_s\": "   << p.windowStartS  << ",\n";
        out << "      \"window_end_s\": "     << p.windowEndS    << ",\n";
        out << "      \"peak_elev_deg\": "    << p.peakElevDeg   << ",\n";
        out << "      \"peak_elev_time_s\": " << p.peakElevTimeS << ",\n";
        out << "      \"n_cells_covered\": "  << p.nCellsCovered << ",\n";
        out << "      \"n_snr_samples\": "    << p.nSnrSamples   << "\n";
        out << "    }";
        if (i + 1 < passes.size()) { out << ","; }
        out << "\n";
    }

    out << "  ]\n}\n";
    std::cout << "[Constellation] wrote " << path << "\n";
}

} // namespace ns3
