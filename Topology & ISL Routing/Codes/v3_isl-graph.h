#pragma once

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/satellite-orbiter-net-device.h"
#include "ns3/satellite-sgp4-mobility-model.h"
#include "ns3/satellite-topology.h"
#include "ns3/singleton.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

// ─── POD types ────────────────────────────────────────────────────────────────

struct ISLDef
{
    uint32_t nodeA, nodeB;
};

struct ISLEdge
{
    uint32_t nodeB;
    double   propagation_cost;   // seconds (prop only; load added separately)
    uint32_t islIfIndexOnA;
    uint32_t islIfIndexOnB;
};

struct RouteEntry
{
    uint32_t destSatId;
    uint32_t nextHopSatId;
    uint32_t islIfIndexOnA;
    double   cost;               // total cost used by Dijkstra
};

using ISLGraph     = std::vector<std::vector<ISLEdge>>;
using RoutingTable = std::vector<std::vector<RouteEntry>>;

// ─── IslRoutingManager ────────────────────────────────────────────────────────

class IslRoutingManager : public Object
{
public:
    static TypeId GetTypeId();
    IslRoutingManager();

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void Initialize(const std::string& islsFilePath);
    void PrecomputeAllTables();
    void ScheduleRoutingUpdates();

    // Called by Simulator at each time slot.
    // For slot > 0: also runs UpdateLoadCosts → HasSignificantChange →
    //               RecomputeAffectedRoutes (selective partial Dijkstra).
    void ApplyRoutingTable(uint32_t slotIndex);

private:
    // ── Setup helpers ─────────────────────────────────────────────────────────
    void LoadISLDefs(const std::string& islsFilePath);
    void InitOrbiterDevices();

    // ── Graph construction ────────────────────────────────────────────────────

    // Compute all satellite positions at tau in one SGP4 pass (N calls).
    std::vector<Vector> GetPositionsAt(Time tau) const;

    // Build ISLGraph from pre-cached positions (prop-delay edges only).
    ISLGraph BuildISLGraph(const std::vector<Vector>& pos) const;

    // Build ISLGraph from pre-cached positions with load-adjusted edge weights.
    ISLGraph BuildISLGraphWithLoad(const std::vector<Vector>& pos) const;

    // ── Routing computation ───────────────────────────────────────────────────

    // Full all-pairs routing (offline).
    RoutingTable ComputeBaseRoutes(const ISLGraph& graph) const;

    // Single-source Dijkstra; shared by full and partial recompute.
    // FIX: tracks first-hop directly — O(E log V) vs old O(N²) back-trace.
    std::vector<RouteEntry> ComputeRoutesForSrc(uint32_t              src,
                                                const ISLGraph&       graph) const;

    // Prefer next-hops that survive into the next time slot.
    RoutingTable ApplyTiebreaker(const RoutingTable& routes,
                                 const ISLGraph&     graphNext) const;

    // ── Runtime load management ───────────────────────────────────────────────

    // Query each ISL queue and EMA-smooth into m_loadCosts.
    void UpdateLoadCosts();

    // Returns true if any load cost changed beyond threshold AND cooldown passed.
    bool HasSignificantChange() const;

    // Partial Dijkstra: only sources with changed adjacent links are rerun.
    void RecomputeAffectedRoutes(uint32_t slotIndex);

    // Return estimated queuing delay (seconds) for sat `satId`, interface `ifIdx`.
    // Fill in with satellite-module queue API when available.
    double GetLinkQueueDelay(uint32_t satId, uint32_t ifIdx) const;

    // ── ns3 Attributes ────────────────────────────────────────────────────────
    uint32_t    m_numSatellites;
    double      m_islMaxDistanceKm;
    uint32_t    m_numTimeSlots;
    double      m_timeSlotInterval;
    std::string m_islsFilePath;

    // EMA alpha for load smoothing (Attribute)
    double      m_emaAlpha;
    // Fractional change threshold to trigger partial recompute (Attribute)
    double      m_changeThreshold;
    // Minimum seconds between partial recomputes — cooldown (Attribute)
    double      m_cooldownSeconds;
    // Assumed ISL link rate in bps — used to convert queue bytes → delay
    double      m_islLinkRateBps;

    // ── Persistent state ──────────────────────────────────────────────────────
    std::vector<ISLDef>                          m_islDefs;
    std::vector<std::map<uint32_t, uint32_t>>    m_perSatISLOrder;
    std::vector<Ptr<SatOrbiterNetDevice>>        m_orbDevs;
    std::vector<Ptr<Node>>                       m_orbNodes;
    std::vector<RoutingTable>                    m_tables;

    // Flat N×N arrays for O(1) load lookup: index = a*N + b
    std::vector<double> m_loadCosts;       // current EMA values
    std::vector<double> m_prevLoadCosts;   // snapshot before last update

    Time m_lastRecomputeTime;
};

} // namespace ns3
