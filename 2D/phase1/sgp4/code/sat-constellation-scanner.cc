/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-constellation-scanner.cc
 *
 * ns3::Object implementation of SatConstellationScanner.
 * See sat-constellation-scanner.h for API documentation.
 *
 * Pass A — pure C++ coarse screen : elevation filter for all 66 satellites.
 * Pass B — ns-3 event-driven       : per-satellite SNR events via
 *           Simulator::Schedule; all qualifying satellites run concurrently
 *           within one Simulator::Run() call.
 *
 * Design rationale for event-driven Pass B:
 *   - Consistent with ns-3 philosophy (time managed by the scheduler).
 *   - All satellites schedule events independently; concurrency is implicit.
 *   - Easily extended to add ns-3 network nodes or mobility models later.
 *   - Pass A stays pure C++ (fast pre-processing, O(66×360) SGP4 calls).
 *
 * Logging component: SatConstellationScanner
 *   Enable with: LogComponentEnable("SatConstellationScanner", LOG_LEVEL_INFO)
 */

#include "sat-constellation-scanner.h"

#include "sat-multi-beam-channel.h"
#include "sat-roi-grid.h"

#include "sgp4unit.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatConstellationScanner");
NS_OBJECT_ENSURE_REGISTERED(SatConstellationScanner);

// ---------------------------------------------------------------------------
// TypeId registration
// ---------------------------------------------------------------------------

TypeId
SatConstellationScanner::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatConstellationScanner")
            .SetParent<Object>()
            .SetGroupName("Satellite")
            .AddConstructor<SatConstellationScanner>();
    return tid;
}

// ---------------------------------------------------------------------------
// SetTleReader
// ---------------------------------------------------------------------------

void
SatConstellationScanner::SetTleReader(Ptr<SatTleReader> reader)
{
    NS_LOG_FUNCTION(this << reader);
    NS_ABORT_MSG_IF(!reader, "SatConstellationScanner::SetTleReader: reader is null");
    m_tleReader = reader;
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
    elsetrec satrec = sat.satrec;   // local copy — sgp4() mutates satrec
    if (!sgp4(wgs72, satrec, tsinceMin, r, v) || satrec.error != 0)
    {
        NS_LOG_DEBUG("SGP4 propagation error for " << sat.name
                     << "  error=" << satrec.error);
        return -90.0;
    }

    const double jdNow  = epochJd + tsinceMin / 1440.0;
    const Vec3   eciKm  = Vec3{r[0], r[1], r[2]};
    const Vec3   ecefKm = EciToEcef(eciKm, jdNow);
    const Vec3   ecefM  = Vec3{ecefKm.x * 1e3, ecefKm.y * 1e3, ecefKm.z * 1e3};

    const double latR = roiLatDeg * M_PI / 180.0;
    const double lonR = roiLonDeg * M_PI / 180.0;
    const Vec3   obsEcef{
        rEarthM * std::cos(latR) * std::cos(lonR),
        rEarthM * std::cos(latR) * std::sin(lonR),
        rEarthM * std::sin(latR)
    };
    const Vec3 delta{ecefM.x - obsEcef.x,
                     ecefM.y - obsEcef.y,
                     ecefM.z - obsEcef.z};
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
    NS_LOG_FUNCTION(state->sat->name << timeS);

    const double tsinceMin = timeS / 60.0;
    const double jdNow     = state->epochJd + tsinceMin / 1440.0;

    double r[3], v[3];
    elsetrec satrec = state->sat->satrec;   // local copy
    if (!sgp4(wgs72, satrec, tsinceMin, r, v) || satrec.error != 0)
    {
        NS_LOG_WARN("ScanSnrCallback: SGP4 error for " << state->sat->name
                    << " at t=" << timeS);
        return;
    }

    const Vec3 eciKm  = Vec3{r[0], r[1], r[2]};
    const Vec3 ecefKm = EciToEcef(eciKm, jdNow);
    const Vec3 ecefM  = Vec3{ecefKm.x * 1e3, ecefKm.y * 1e3, ecefKm.z * 1e3};

    const Vec3 delta{
        ecefM.x - state->obsEcef.x,
        ecefM.y - state->obsEcef.y,
        ecefM.z - state->obsEcef.z
    };
    const Vec3   satEnu  = EcefOffsetToEnu(delta, state->roiLatDeg, state->roiLonDeg);
    const double elevDeg = GetElevationAngleDeg_3D(satEnu);

    if (elevDeg < state->minElevDeg)
    {
        NS_LOG_DEBUG("Skip: elev=" << elevDeg << " < minElev=" << state->minElevDeg);
        return;
    }

    // Elliptical correction: recompute beam centres for actual satellite position.
    // At nadir this matches the old pre-computed value; at low elevation the
    // pattern elongates correctly along the slant direction.
    const auto beamCentersEnu = GetBeamCentersFromSatPos(satEnu, state->cfg);

    const auto results = ComputeFrameResults(
        satEnu, state->cellPos, beamCentersEnu,
        state->cfg, state->rng, false);

    for (std::size_t i = 0; i < results.size(); ++i)
    {
        state->csv << std::fixed << std::setprecision(6)
                   << timeS << "," << i << ","
                   << std::setprecision(4) << elevDeg << ","
                   << results[i].pathLossDb << ","
                   << results[i].beamGainDb << ","
                   << results[i].snrDb      << ","
                   << results[i].sinrDb     << "\n";
    }
    ++state->nWritten;

    // MRC accumulation: add this satellite's linear SNR contribution per cell.
    // Cells that fail the elevation gate never reach this point, so only
    // qualifying satellites accumulate into the shared MrcAccumulator.
    // timeKey = round(time_s × 10) avoids floating-point map-key collisions
    // when dtSnrS is a multiple of 0.1 s (the common case).
    if (state->mrcAcc)
    {
        const int64_t timeKey = static_cast<int64_t>(std::llround(timeS * 10.0));
        auto& cellVec = state->mrcAcc->data[timeKey];
        if (cellVec.empty())
            cellVec.resize(static_cast<std::size_t>(state->mrcAcc->nCells));

        for (std::size_t i = 0; i < results.size(); ++i)
        {
            // Convert dB to linear before summing: MRC SNR = Σ SNR_i (linear)
            cellVec[i].linearSnrSum += std::pow(10.0, results[i].snrDb / 10.0);
            cellVec[i].nSats        += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Run
// ---------------------------------------------------------------------------

std::vector<SatPassInfo>
SatConstellationScanner::Run(const ConstellationScanConfig& config)
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(!m_tleReader,
                    "SatConstellationScanner::Run: SetTleReader() must be called first");

    std::filesystem::create_directories(config.outDir);

    // Pre-compute fixed geometry shared across all satellites
    const RoiGrid          grid   = GenerateRoiGrid(config.gridD, config.cfg.rFootprintM);
    const std::vector<Vec3> cellPos = GetRoiCellPositions(grid);
    const int              nInFP  = static_cast<int>(cellPos.size());

    // Initialise the MRC accumulator: reset from any previous Run() call and
    // record the cell count so ScanSnrCallback can size the per-timestep vector.
    m_mrcAccumulator        = MrcAccumulator{};
    m_mrcAccumulator.nCells = nInFP;

    // Note: beam centres are NOT pre-computed here anymore.
    // ScanSnrCallback calls GetBeamCentersFromSatPos(satEnu, cfg) each second
    // so the elliptical ground projection is correct at every elevation angle.

    const double epochJd = m_tleReader->GetEpochJd();
    const double rEarthM = config.cfg.rEarthM;
    const double latR    = config.roiLatDeg * M_PI / 180.0;
    const double lonR    = config.roiLonDeg * M_PI / 180.0;
    const Vec3 obsEcef{
        rEarthM * std::cos(latR) * std::cos(lonR),
        rEarthM * std::cos(latR) * std::sin(lonR),
        rEarthM * std::sin(latR)
    };

    NS_LOG_INFO("ROI grid " << config.gridD << "x" << config.gridD
                << "  in-footprint=" << nInFP
                << "  lat=" << config.roiLatDeg
                << "  lon=" << config.roiLonDeg);

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

        if (winStart < 0.0) { continue; }   // satellite never above threshold

        NS_LOG_INFO(sat.name
                    << "  peak=" << std::fixed << std::setprecision(1) << peakElev << " deg"
                    << "  window=[" << winStart << "," << winEnd << "] s");

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
    // One SatScanState per qualifying satellite; states live on the heap for
    // the duration of Simulator::Run().
    // -----------------------------------------------------------------------
    std::vector<std::unique_ptr<SatScanState>> states;
    states.reserve(passes.size());

    for (std::size_t idx = 0; idx < passes.size(); ++idx)
    {
        const SatPassInfo& passInfo = passes[idx];

        std::ostringstream ss;
        ss << config.outDir << "/sat_"
           << std::setw(5) << std::setfill('0') << passInfo.satIndex
           << "_cells.csv";

        auto state = std::make_unique<SatScanState>();
        state->sat            = qualifyingSats[idx];
        state->epochJd        = epochJd;
        state->obsEcef        = obsEcef;
        state->roiLatDeg      = config.roiLatDeg;
        state->roiLonDeg      = config.roiLonDeg;
        state->minElevDeg     = config.minElevDeg;
        state->cfg            = config.cfg;
        state->cellPos        = cellPos;
        // beamCentersEnu omitted: computed per-timestep in ScanSnrCallback.
        state->mrcAcc         = &m_mrcAccumulator;   // shared MRC accumulator
        state->rng            = std::mt19937(config.seed + static_cast<uint32_t>(idx));

        state->csv.open(ss.str(), std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!state->csv.is_open(), "Cannot open CSV: " << ss.str());
        state->csv << "time_s,cell_idx,elevation_deg,path_loss_dB,"
                      "beam_gain_dB,snr_dB,sinr_dB\n";

        // Schedule one event per dtSnrS second within this satellite's window.
        // Satellites schedule independently; ns-3 fires them in time order.
        SatScanState* rawPtr = state.get();
        for (double t = passInfo.windowStartS; t <= passInfo.windowEndS; t += config.dtSnrS)
        {
            Simulator::Schedule(Seconds(t), &ScanSnrCallback, rawPtr, t);
        }

        NS_LOG_DEBUG("Scheduled events for " << passInfo.satName
                     << "  window=[" << passInfo.windowStartS << "," << passInfo.windowEndS << "] s"
                     << "  csvPath=" << ss.str());

        states.push_back(std::move(state));
    }

    Simulator::Stop(Seconds(config.windowS));
    Simulator::Run();
    Simulator::Destroy();

    // Collect per-satellite result counts and report
    for (std::size_t idx = 0; idx < passes.size(); ++idx)
    {
        passes[idx].nSnrSamples = states[idx]->nWritten;
        NS_LOG_INFO("sat_" << std::setw(5) << std::setfill('0') << passes[idx].satIndex
                    << "_cells.csv"
                    << "  " << passes[idx].nSnrSamples
                    << " timesteps x " << passes[idx].nCellsCovered << " cells");
    }

    // Flush MRC combined CSV (all satellites have written into m_mrcAccumulator)
    WriteMrcCsv(config.outDir);

    WriteStatusJson(config.outDir, config, passes);

    NS_LOG_INFO("Done: " << passes.size()
                << " satellites qualified (elevation > " << config.minElevDeg << " deg)");

    return passes;
}

// ---------------------------------------------------------------------------
// WriteMrcCsv
// ---------------------------------------------------------------------------

void
SatConstellationScanner::WriteMrcCsv(const std::string& outDir) const
{
    NS_LOG_FUNCTION(this << outDir);

    const std::string path = outDir + "/mrc_combined.csv";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!out.is_open(), "WriteMrcCsv: cannot open " << path);

    // Header: four columns.
    // snr_mrc_dB = 10·log10(Σ SNR_i)  — MRC combines linear SNRs before log.
    out << "time_s,cell_idx,snr_mrc_dB,n_sats\n";

    for (const auto& [timeKey, cellVec] : m_mrcAccumulator.data)
    {
        const double timeS = static_cast<double>(timeKey) / 10.0;
        for (std::size_t i = 0; i < cellVec.size(); ++i)
        {
            const auto& s = cellVec[i];
            if (s.nSats == 0)
                continue;

            // Guard: linearSnrSum > 0 before log (always true for valid SNR)
            const double snrMrcDb = (s.linearSnrSum > 0.0)
                                        ? 10.0 * std::log10(s.linearSnrSum)
                                        : -999.0;

            out << std::fixed << std::setprecision(6) << timeS << ","
                << i                                           << ","
                << std::setprecision(4) << snrMrcDb            << ","
                << s.nSats                                     << "\n";
        }
    }

    NS_LOG_INFO("WriteMrcCsv: " << m_mrcAccumulator.data.size()
                << " timesteps written → " << path);
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
    NS_LOG_FUNCTION(this << outDir);

    const std::string path = outDir + "/constellation_status.json";
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        NS_LOG_WARN("WriteStatusJson: cannot write " << path);
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
    NS_LOG_INFO("Wrote " << path);
}

} // namespace ns3
