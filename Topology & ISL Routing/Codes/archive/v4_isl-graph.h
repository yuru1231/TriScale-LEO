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
// header 改成：
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
