#ifndef ISL_GRAPH_H
#define ISL_GRAPH_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/satellite-isl-arbiter-unicast.h"
#include "ns3/satellite-orbiter-net-device.h"
#include "ns3/satellite-sgp4-mobility-model.h"
#include "ns3/satellite-topology.h"
#include "ns3/singleton.h"
#include "ns3/vector.h"

#include <chrono>
#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ns3
{

struct ISLEdge
{
    uint32_t nodeB;
    double propagation_cost;   // dist/c，單位：秒
    uint32_t islIfIndexOnA;
    uint32_t islIfIndexOnB;
};

struct ISLDef
{
    uint32_t nodeA;
    uint32_t nodeB;
};

struct RouteEntry
{
    uint32_t destSatId;
    uint32_t nextHopSatId;
    uint32_t islIfIndexOnA;
    double cost;
};

using ISLGraph = std::vector<std::vector<ISLEdge>>;
using RoutingTable = std::vector<std::vector<RouteEntry>>;

struct SlotStats
{
    uint32_t slotIndex;
    double simTimeSec;
    long applyWallMs;         // ApplyRoutingTable 執行時間
    long recomputeWallMs;     // RecomputeAffectedRoutes 執行時間（0 若未觸發）
    uint32_t recomputedSources;
    bool significantChange;
};

class IslRoutingManager : public Object
{
  public:
    static TypeId GetTypeId();

    IslRoutingManager();
    ~IslRoutingManager() override = default;

    // Lifecycle
    void Initialize(const std::string& islsFilePath);
    void PrecomputeAllTables();
    void ScheduleRoutingUpdates();
    void ApplyRoutingTable(uint32_t slotIndex);

    // Graph
    std::vector<Vector> GetPositionsAt(Time tau) const;
    ISLGraph BuildISLGraph(const std::vector<Vector>& pos) const;
    ISLGraph BuildISLGraphWithLoad(const std::vector<Vector>& pos) const;

    // Stats
    void PrintStats() const;

    // Accessors for Layer 2/3
    uint32_t GetNumTimeSlots()    const { return m_numTimeSlots; }
    uint32_t GetNumSatellites()   const { return m_numSatellites; }
    double   GetTimeSlotInterval()const { return m_timeSlotInterval; }
    double   GetRouteCost(uint32_t src, uint32_t dst, uint32_t slotIndex) const;

    // ── Verification / diagnostics (pure offline, no NS3 event interaction) ──
    //
    // TracePath: reconstruct full hop sequence src→…→dst by following next-hop
    //   entries in the stored routing table.  Returns {UINT32_MAX} as last
    //   element if destination is unreachable.
    std::vector<uint32_t> TracePath(uint32_t src,
                                    uint32_t dst,
                                    uint32_t slotIndex) const;

    // PrintRouteReport: for each (src,dst) pair, print one row per slot:
    //   time(s) | src | dst | full_path | route_cost | applied_slot
    //   Rows where the path changed from the previous slot are tagged <CHANGED>.
    void PrintRouteReport(
        const std::vector<std::pair<uint32_t, uint32_t>>& pairs) const;

    // BlockISL / UnblockISL: temporarily mark an ISL as unavailable so that
    //   BuildISLGraph and BuildISLGraphWithLoad skip it.  Both directions are
    //   blocked/unblocked simultaneously.
    void BlockISL(uint32_t nodeA, uint32_t nodeB);
    void UnblockISL(uint32_t nodeA, uint32_t nodeB);

    // RunAvoidanceTest: at slotIndex, find the path from testSrc to testDst,
    //   block the first ISL on that path, recompute the routing table offline,
    //   print the new path, and verify:
    //     (a) the blocked ISL does not appear in the new path, and
    //     (b) no stale next-hop entry remains in the routing table.
    //   Restores the original routing table and clears the block on exit.
    void RunAvoidanceTest(uint32_t testSrc,
                          uint32_t testDst,
                          uint32_t slotIndex);

  private:
    // Routing
    std::vector<RouteEntry> ComputeRoutesForSrc(uint32_t src,
                                                const ISLGraph& graph) const;
    RoutingTable ComputeBaseRoutes(const ISLGraph& graph) const;
    RoutingTable ApplyTiebreaker(const RoutingTable& routes,
                                 const ISLGraph& graphNext) const;

    // Runtime
    void UpdateLoadCosts();
    bool HasSignificantChange() const;
    uint32_t RecomputeAffectedRoutes(uint32_t slotIndex);
    void RebuildIslSources(uint32_t slotIndex);
    double GetLinkQueueDelay(uint32_t satId, uint32_t ifIdx) const;

    // Setup
    void LoadISLDefs(const std::string& islsFilePath);
    void InitOrbiterDevices();

  private:
    // Attributes
    uint32_t m_numSatellites;
    double   m_islMaxDistanceKm;
    uint32_t m_numTimeSlots;
    double m_timeSlotInterval = 60.0;
    std::string m_islsFilePath;
    double m_emaAlpha = 0.5;
    double m_changeThreshold = 0.2;
    double m_cooldownSeconds = 60.0;
    double m_islLinkRateBps = 10e6;

    // Internal data
    // Blocked ISL set: populated only during RunAvoidanceTest.
    // Both (a,b) and (b,a) are inserted so BuildISLGraph needs only one lookup.
    std::set<std::pair<uint32_t, uint32_t>> m_blockedEdges;

    std::vector<ISLDef> m_islDefs;
    std::vector<std::map<uint32_t, uint32_t>> m_perSatISLOrder;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> m_edgeOfPair;

    std::vector<Ptr<SatOrbiterNetDevice>> m_orbDevs;
    std::vector<Ptr<Node>> m_orbNodes;
    std::vector<Ptr<SatIslArbiterUnicast>> m_arbiters;

    std::vector<RoutingTable> m_tables;
    std::vector<double> m_loadCosts;
    std::vector<double> m_prevLoadCosts;
    std::vector<std::unordered_set<uint32_t>> m_islSources;

    Time m_lastRecomputeTime;

    // Stats
    std::vector<SlotStats> m_stats;
};

} // namespace ns3

#endif // ISL_GRAPH_H
