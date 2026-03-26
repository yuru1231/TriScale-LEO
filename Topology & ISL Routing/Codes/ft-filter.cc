#include "ft-filter.h"

#include "ns3/satellite-topology.h"
#include "ns3/singleton.h"
#include "ns3/type-id.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(FtVisibilityFilter);

static const double EARTH_RADIUS_M = 6371000.0;

TypeId
FtVisibilityFilter::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::FtVisibilityFilter")
            .SetParent<Object>()
            .AddConstructor<FtVisibilityFilter>();
    return tid;
}

FtVisibilityFilter::FtVisibilityFilter() {}

void
FtVisibilityFilter::SetRoutingManager(Ptr<IslRoutingManager> mgr)
{
    m_routingMgr = mgr;
}

void
FtVisibilityFilter::SetElevationThreshold(double deg)
{
    m_elevThreshDeg = deg;
}

void
FtVisibilityFilter::AddFt(uint32_t id, double latDeg, double lonDeg, const std::string& name)
{
    m_ftIdToIdx[id] = static_cast<uint32_t>(m_fts.size());
    m_fts.push_back({id, latDeg, lonDeg, name});
}

void
FtVisibilityFilter::AddContractedPair(uint32_t ftI, uint32_t ftJ)
{
    m_contractedPairs.insert({ftI, ftJ});
    m_contractedPairs.insert({ftJ, ftI}); // bidirectional
}

// ── Offline pre-computation ───────────────────────────────────────────────

void
FtVisibilityFilter::PrecomputeVisibility()
{
    NS_ASSERT_MSG(m_routingMgr, "FtVisibilityFilter: call SetRoutingManager first");

    m_numSlots      = m_routingMgr->GetNumTimeSlots();
    m_numSatellites = m_routingMgr->GetNumSatellites();
    m_visibility.resize(m_numSlots);

    const double slotInterval = m_routingMgr->GetTimeSlotInterval();

    for (uint32_t k = 0; k < m_numSlots; k++)
    {
        Time                tau    = Seconds(k * slotInterval);
        std::vector<Vector> satPos = m_routingMgr->GetPositionsAt(tau);

        for (const auto& ft : m_fts)
        {
            std::set<uint32_t>& vis = m_visibility[k][ft.id];
            for (uint32_t s = 0; s < m_numSatellites; s++)
            {
                if (ComputeElevationDeg(ft.latDeg, ft.lonDeg, satPos[s]) >= m_elevThreshDeg)
                    vis.insert(s);
            }
        }

        // Progress log
        std::cout << "[FtFilter] slot=" << k << " t=" << tau.GetSeconds() << "s";
        for (const auto& ft : m_fts)
            std::cout << " FT" << ft.id << "=" << m_visibility[k][ft.id].size() << "sats";
        std::cout << "\n";
    }
}

// ── Query ─────────────────────────────────────────────────────────────────

const std::set<uint32_t>&
FtVisibilityFilter::GetAccessSats(uint32_t ftId, uint32_t slotIndex) const
{
    if (slotIndex >= m_visibility.size())
        return m_emptySet;
    auto it = m_visibility[slotIndex].find(ftId);
    return (it != m_visibility[slotIndex].end()) ? it->second : m_emptySet;
}

FtTransitRoute
FtVisibilityFilter::GetBestTransit(uint32_t ftI, uint32_t ftJ, uint32_t slotIndex) const
{
    FtTransitRoute result;
    result.ftSrc = ftI;
    result.ftDst = ftJ;

    if (!m_contractedPairs.count({ftI, ftJ}))
        return result; // not contracted

    const auto& srcSats = GetAccessSats(ftI, slotIndex);
    const auto& dstSats = GetAccessSats(ftJ, slotIndex);

    if (srcSats.empty() || dstSats.empty())
        return result; // no visibility

    for (uint32_t entry : srcSats)
    {
        for (uint32_t exit : dstSats)
        {
            // Same satellite can see both FTs: zero ISL cost
            double cost = (entry == exit)
                              ? 0.0
                              : m_routingMgr->GetRouteCost(entry, exit, slotIndex);

            if (cost < result.islCost)
            {
                result.entrySatId = entry;
                result.exitSatId  = exit;
                result.islCost    = cost;
                result.valid      = true;
            }
        }
    }

    return result;
}

// ── Report ────────────────────────────────────────────────────────────────

void
FtVisibilityFilter::PrintVisibilityReport() const
{
    std::cout << "\n=== FtVisibilityFilter Report ===\n";
    for (uint32_t k = 0; k < m_numSlots; k++)
    {
        std::cout << "Slot " << k << " (" << (k * m_routingMgr->GetTimeSlotInterval()) << "s):\n";
        for (const auto& ft : m_fts)
        {
            const auto& vis = GetAccessSats(ft.id, k);
            std::cout << "  FT" << ft.id << "[" << ft.name << "] lat=" << ft.latDeg
                      << " lon=" << ft.lonDeg << " → " << vis.size() << " visible sats\n";
        }
    }

    std::cout << "\nBest transit routes (contracted pairs, slot 0):\n";
    for (const auto& p : m_contractedPairs)
    {
        if (p.first >= p.second) continue;
        auto r = GetBestTransit(p.first, p.second, 0);
        std::cout << "  FT" << p.first << " → FT" << p.second << ": ";
        if (r.valid)
            std::cout << "entry=sat" << r.entrySatId << " exit=sat" << r.exitSatId
                      << " cost=" << std::fixed << std::setprecision(4) << r.islCost << "s";
        else
            std::cout << "NO ROUTE";
        std::cout << "\n";
    }
    std::cout << "=================================\n\n";
}

// ── Utility ───────────────────────────────────────────────────────────────

// static
double
FtVisibilityFilter::ComputeElevationDeg(double obsLatDeg,
                                         double        obsLonDeg,
                                         const Vector& satEcef)
{
    const double latRad = obsLatDeg * M_PI / 180.0;
    const double lonRad = obsLonDeg * M_PI / 180.0;

    // Observer ECEF (spherical Earth, alt=0)
    const double ox = EARTH_RADIUS_M * std::cos(latRad) * std::cos(lonRad);
    const double oy = EARTH_RADIUS_M * std::cos(latRad) * std::sin(lonRad);
    const double oz = EARTH_RADIUS_M * std::sin(latRad);

    // Observer → satellite vector
    const double dx   = satEcef.x - ox;
    const double dy   = satEcef.y - oy;
    const double dz   = satEcef.z - oz;
    const double dLen = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dLen < 1.0) return -90.0;

    // Local "up" unit vector at observer
    const double oLen   = std::sqrt(ox * ox + oy * oy + oz * oz);
    const double dotProd = (dx * ox + dy * oy + dz * oz) / (dLen * oLen);

    return std::asin(std::max(-1.0, std::min(1.0, dotProd))) * 180.0 / M_PI;
}

} // namespace ns3
