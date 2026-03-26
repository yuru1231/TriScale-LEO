#ifndef FT_FILTER_H
#define FT_FILTER_H

#include "isl-graph.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/satellite-sgp4-mobility-model.h"
#include "ns3/vector.h"

#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ns3
{

// ── Data structures ───────────────────────────────────────────────────────

struct FtDef
{
    uint32_t    id;
    double      latDeg;
    double      lonDeg;
    std::string name;
};

struct FtTransitRoute
{
    uint32_t ftSrc{0};
    uint32_t ftDst{0};
    uint32_t entrySatId{0};   // first sat visible from ftSrc on this path
    uint32_t exitSatId{0};    // last sat visible to ftDst on this path
    double   islCost{std::numeric_limits<double>::infinity()};
    bool     valid{false};
};

// ── FtVisibilityFilter ────────────────────────────────────────────────────
//
// Layer 1 extension: filters IslRoutingManager results to only consider
// paths through satellites visible from contracted FT pairs.
//
// Usage:
//   1. SetRoutingManager(mgr)
//   2. AddFt(id, lat, lon)  for each FT
//   3. AddContractedPair(ftI, ftJ)
//   4. PrecomputeVisibility()  ← call AFTER IslRoutingManager::PrecomputeAllTables()
//   5. GetBestTransit(ftI, ftJ, slotIndex)  during simulation
//
class FtVisibilityFilter : public Object
{
  public:
    static TypeId GetTypeId();
    FtVisibilityFilter();

    // Configuration
    void SetRoutingManager(Ptr<IslRoutingManager> mgr);
    void SetElevationThreshold(double deg);
    void AddFt(uint32_t id, double latDeg, double lonDeg, const std::string& name = "");
    void AddContractedPair(uint32_t ftI, uint32_t ftJ);

    // Offline pre-computation (call after IslRoutingManager::PrecomputeAllTables)
    void PrecomputeVisibility();

    // Query interface
    const std::set<uint32_t>& GetAccessSats(uint32_t ftId, uint32_t slotIndex) const;
    FtTransitRoute             GetBestTransit(uint32_t ftI, uint32_t ftJ, uint32_t slotIndex) const;

    void PrintVisibilityReport() const;

    // Utility: also used by BeamHoppingManager
    static double ComputeElevationDeg(double obsLatDeg, double obsLonDeg,
                                      const Vector& satEcef);

  private:
    Ptr<IslRoutingManager>                              m_routingMgr;
    std::vector<FtDef>                                  m_fts;
    std::map<uint32_t, uint32_t>                        m_ftIdToIdx; // ftId → index
    std::set<std::pair<uint32_t, uint32_t>>             m_contractedPairs;
    double                                              m_elevThreshDeg{5.0};

    // m_visibility[slotIndex][ftId] = set of visible satIds
    std::vector<std::map<uint32_t, std::set<uint32_t>>> m_visibility;
    std::set<uint32_t>                                  m_emptySet;

    uint32_t m_numSlots{0};
    uint32_t m_numSatellites{0};
};

} // namespace ns3
#endif // FT_FILTER_H
