#include "beam-hopping-manager.h"

#include "ns3/double.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(BeamHoppingManager);

TypeId
BeamHoppingManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::BeamHoppingManager")
            .SetParent<Object>()
            .AddConstructor<BeamHoppingManager>()
            .AddAttribute("NumSatellites",
                          "Number of satellites",
                          UintegerValue(66),
                          MakeUintegerAccessor(&BeamHoppingManager::m_numSatellites),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SuperframeDurationSec",
                          "DVB superframe duration in seconds",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(&BeamHoppingManager::m_superframeSec),
                          MakeDoubleChecker<double>(0.001, 10.0))
            .AddAttribute("ElevationThresholdDeg",
                          "Minimum elevation angle for cell visibility",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&BeamHoppingManager::m_elevThreshDeg),
                          MakeDoubleChecker<double>(0.0, 90.0));
    return tid;
}

BeamHoppingManager::BeamHoppingManager()
{
    // Default: uniform demand, log-only switch callback
    m_demandProvider = std::make_shared<UniformDemandProvider>(1.0);
    m_switchCallback = [](uint32_t satId, uint32_t cellId, Time t) {
        std::cout << "[BH] t=" << t.GetSeconds() << "s sat=" << satId
                  << " → cell=" << cellId << "\n";
    };
}

// ── Configuration ─────────────────────────────────────────────────────────

void
BeamHoppingManager::AddCell(uint32_t id, double latDeg, double lonDeg, const std::string& name)
{
    m_cells.push_back({id, latDeg, lonDeg, name});
}

void
BeamHoppingManager::SetElevationThreshold(double deg)
{
    m_elevThreshDeg = deg;
}

void
BeamHoppingManager::SetDemandProvider(std::shared_ptr<TrafficDemandProvider> provider)
{
    m_demandProvider = provider;
}

void
BeamHoppingManager::SetBhSwitchCallback(BhSwitchCallback cb)
{
    m_switchCallback = cb;
}

// ── Offline schedule computation ──────────────────────────────────────────

void
BeamHoppingManager::ComputeBhSchedule(uint32_t numSlots, double slotIntervalSec)
{
    LoadOrbiterNodes();
    m_schedule.clear();

    const auto sfPerSlot =
        static_cast<uint32_t>(slotIntervalSec / m_superframeSec);

    for (uint32_t k = 0; k < numSlots; k++)
    {
        const double tauSec = k * slotIntervalSec;
        const Time   tau    = Seconds(tauSec);

        for (uint32_t s = 0; s < m_numSatellites; s++)
        {
            Ptr<SatSGP4MobilityModel> mob =
                m_orbNodes[s]->GetObject<SatSGP4MobilityModel>();
            NS_ASSERT_MSG(mob, "BeamHoppingManager: no SGP4 model on sat " << s);
            const Vector satPos = mob->GetGeoPositionAt(tau).ToVector();

            const std::vector<uint32_t> visCells = ComputeVisibleCells(satPos);
            if (visCells.empty()) continue;

            // Gather demand per visible cell and sort highest-first
            std::vector<std::pair<double, uint32_t>> demandOrder;
            double totalDemand = 0.0;
            for (uint32_t cellId : visCells)
            {
                double d = m_demandProvider->GetDemandMbps(cellId, tau);
                demandOrder.push_back({d, cellId});
                totalDemand += d;
            }
            std::sort(demandOrder.rbegin(), demandOrder.rend()); // highest demand first

            // Allocate superframe slots proportional to demand
            double cursorSec = tauSec;
            const double slotEndSec = tauSec + slotIntervalSec;

            for (const auto& [demand, cellId] : demandOrder)
            {
                if (cursorSec >= slotEndSec) break;

                const double fraction =
                    (totalDemand > 0.0) ? demand / totalDemand
                                        : 1.0 / static_cast<double>(visCells.size());
                const uint32_t numSF =
                    std::max(1u, static_cast<uint32_t>(std::round(fraction * sfPerSlot)));

                m_schedule.push_back({Seconds(cursorSec), s, cellId});
                cursorSec += numSF * m_superframeSec;
            }
        }
    }

    // Sort all events by time
    std::sort(m_schedule.begin(), m_schedule.end(),
              [](const BhEvent& a, const BhEvent& b) { return a.t < b.t; });

    std::cout << "[BeamHoppingManager] ComputeBhSchedule: " << m_schedule.size()
              << " events across " << numSlots << " slots\n";
}

// ── Runtime ───────────────────────────────────────────────────────────────

void
BeamHoppingManager::ScheduleBhUpdates()
{
    for (const auto& ev : m_schedule)
    {
        Simulator::Schedule(ev.t,
                            &BeamHoppingManager::ApplyBhEvent,
                            this,
                            ev.satId,
                            ev.cellId);
    }
    std::cout << "[BeamHoppingManager] ScheduleBhUpdates: "
              << m_schedule.size() << " events registered\n";
}

void
BeamHoppingManager::ApplyBhEvent(uint32_t satId, uint32_t cellId)
{
    m_currentCell[satId] = cellId;

    // ── SNS3_BH_INJECT ────────────────────────────────────────────────────
    // TODO: call the appropriate SNS3 API to switch beam here.
    // Candidate (to be verified against SNS3 source):
    //   m_orbDevs[satId]->SetActiveBeam(cellId);
    // or
    //   Singleton<SatTopology>::Get()
    //       ->GetBeamScheduler(satId)->EnableBeam(cellId);
    // ─────────────────────────────────────────────────────────────────────

    if (m_switchCallback)
        m_switchCallback(satId, cellId, Simulator::Now());
}

// ── Layer 3 hook ──────────────────────────────────────────────────────────

uint32_t
BeamHoppingManager::GetCurrentCell(uint32_t satId) const
{
    auto it = m_currentCell.find(satId);
    return (it != m_currentCell.end()) ? it->second : UINT32_MAX;
}

// ── Print ─────────────────────────────────────────────────────────────────

void
BeamHoppingManager::PrintSchedule() const
{
    std::cout << "\n=== BeamHoppingManager Schedule ("
              << m_schedule.size() << " events) ===\n"
              << std::left << std::setw(12) << "time(s)"
              << std::setw(8)  << "satId"
              << std::setw(8)  << "cellId"
              << "\n";
    for (const auto& ev : m_schedule)
    {
        std::cout << std::left << std::setw(12) << ev.t.GetSeconds()
                  << std::setw(8)  << ev.satId
                  << std::setw(8)  << ev.cellId
                  << "\n";
    }
    std::cout << "===========================================\n\n";
}

// ── Private helpers ───────────────────────────────────────────────────────

std::vector<uint32_t>
BeamHoppingManager::ComputeVisibleCells(const Vector& satEcef) const
{
    std::vector<uint32_t> result;
    for (const auto& cell : m_cells)
    {
        double elev = FtVisibilityFilter::ComputeElevationDeg(cell.latDeg, cell.lonDeg, satEcef);
        if (elev >= m_elevThreshDeg)
            result.push_back(cell.id);
    }
    return result;
}

void
BeamHoppingManager::LoadOrbiterNodes()
{
    m_orbNodes.resize(m_numSatellites);
    for (uint32_t i = 0; i < m_numSatellites; i++)
    {
        m_orbNodes[i] = Singleton<SatTopology>::Get()->GetOrbiterNode(i);
        NS_ASSERT_MSG(m_orbNodes[i], "BeamHoppingManager: null orbiter node " << i);
    }
    std::cout << "[BeamHoppingManager] LoadOrbiterNodes: " << m_numSatellites << " nodes loaded\n";
}

} // namespace ns3
