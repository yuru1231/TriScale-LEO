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
#include <limits>
#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ns3
{

// ── Layer 1 基本資料結構 ──────────────────────────────────────────────────

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

using ISLGraph    = std::vector<std::vector<ISLEdge>>;
using RoutingTable = std::vector<std::vector<RouteEntry>>;

struct SlotStats
{
    uint32_t slotIndex;
    double   simTimeSec;
    long     applyWallMs;       // ApplyRoutingTable 執行時間
    long     recomputeWallMs;   // RecomputeAffectedRoutes 執行時間（0 若未觸發）
    uint32_t recomputedSources;
    bool     significantChange;
};

// ── v6 新增：GW-to-GW 路由資料結構 ────────────────────────────────────────

// 代表一個 Gateway（地面站）的基本定義
struct GwDef
{
    uint32_t    gwId;
    double      latDeg;   // 緯度（度）
    double      lonDeg;   // 經度（度）
    std::string name;
};

// 代表一條 GW→GW 的端到端路由（含完整衛星路徑）
// 路由路徑：src_GW → entrySatId → [ISL path] → exitSatId → dst_GW
struct GwToGwRoute
{
    uint32_t              srcGwId{0};
    uint32_t              dstGwId{0};
    uint32_t              entrySatId{UINT32_MAX};  // src GW 可見、作為上鏈入口的衛星
    uint32_t              exitSatId{UINT32_MAX};   // dst GW 可見、作為下鏈出口的衛星
    std::vector<uint32_t> satPath;                 // 完整衛星路徑（entry → … → exit）
    double                islCost{std::numeric_limits<double>::infinity()};
    bool                  valid{false};
};

// ── v7 新增：GW-to-UT 路由資料結構 ────────────────────────────────────────

// 代表一個 User Terminal（用戶終端）的基本定義
struct UtDef
{
    uint32_t    utId;
    double      latDeg;   // 緯度（度）
    double      lonDeg;   // 經度（度）
    std::string name;
};

// 代表一條 GW→UT 的端到端路由（含完整衛星路徑）
// 路由路徑：GW → entrySatId → [ISL path] → servingSatId → UT
struct GwToUtRoute
{
    uint32_t              gwId{0};
    uint32_t              utId{0};
    uint32_t              entrySatId{UINT32_MAX};    // GW 可見、作為上鏈入口的衛星
    uint32_t              servingSatId{UINT32_MAX};  // UT 可見、作為下鏈服務的衛星
    std::vector<uint32_t> satPath;                   // 完整衛星路徑（entry → … → serving）
    double                islCost{std::numeric_limits<double>::infinity()};
    bool                  valid{false};
};

// ── IslRoutingManager ─────────────────────────────────────────────────────

class IslRoutingManager : public Object
{
  public:
    static TypeId GetTypeId();

    IslRoutingManager();
    ~IslRoutingManager() override = default;

    // ── Lifecycle ──────────────────────────────────────────────────────────
    void Initialize(const std::string& islsFilePath);
    void PrecomputeAllTables();
    void ScheduleRoutingUpdates();
    void ApplyRoutingTable(uint32_t slotIndex);

    // ── Graph ──────────────────────────────────────────────────────────────
    std::vector<Vector> GetPositionsAt(Time tau) const;
    ISLGraph BuildISLGraph(const std::vector<Vector>& pos) const;
    ISLGraph BuildISLGraphWithLoad(const std::vector<Vector>& pos) const;

    // ── Stats ──────────────────────────────────────────────────────────────
    void PrintStats() const;

    // ── Accessors for Layer 2/3 ────────────────────────────────────────────
    uint32_t GetNumTimeSlots()     const { return m_numTimeSlots; }
    uint32_t GetNumSatellites()    const { return m_numSatellites; }
    double   GetTimeSlotInterval() const { return m_timeSlotInterval; }
    double   GetRouteCost(uint32_t src, uint32_t dst, uint32_t slotIndex) const;

    // ── Verification / diagnostics ─────────────────────────────────────────
    //
    // TracePath: 依 routing table 重建 src→…→dst 完整跳數序列。
    //   若目的不可達，最後一個元素為 UINT32_MAX。
    std::vector<uint32_t> TracePath(uint32_t src,
                                    uint32_t dst,
                                    uint32_t slotIndex) const;

    // PrintRouteReport: 針對每個 (src,dst) pair，逐 slot 輸出路徑、代價，
    //   若路徑相較前一 slot 有變化則標記 <PATH CHANGED>。
    void PrintRouteReport(
        const std::vector<std::pair<uint32_t, uint32_t>>& pairs) const;

    // BlockISL / UnblockISL: 暫時標記某條 ISL 為不可用（供 RunAvoidanceTest 使用）。
    void BlockISL(uint32_t nodeA, uint32_t nodeB);
    void UnblockISL(uint32_t nodeA, uint32_t nodeB);

    // RunAvoidanceTest: 封鎖 (testSrc→testDst) 路徑上第一條 ISL，
    //   離線重算路由，驗證：(a) 封鎖 ISL 不出現於新路徑，(b) 無殘留舊 next-hop。
    void RunAvoidanceTest(uint32_t testSrc,
                          uint32_t testDst,
                          uint32_t slotIndex);

    // ── v6：GW-to-GW 路由 API ─────────────────────────────────────────────
    //
    // 使用流程：
    //   1. AddGateway(...)          ← 設定地面站
    //   2. AddGwPair(...)           ← 設定允許路由的 GW pair
    //   3. SetGwElevationThreshold  ← 設定仰角門檻（可選，預設 5°）
    //   4. Initialize(...)
    //   5. PrecomputeAllTables()
    //   6. PrecomputeGwRoutes()     ← 必須在 PrecomputeAllTables 之後
    //   7. PrintGwRouteReport() / GetGwRoute()

    void AddGateway(uint32_t gwId, double latDeg, double lonDeg,
                    const std::string& name = "");
    void AddGwPair(uint32_t gwA, uint32_t gwB);
    void SetGwElevationThreshold(double deg);
    void PrecomputeGwRoutes();
    GwToGwRoute GetGwRoute(uint32_t srcGwId, uint32_t dstGwId,
                           uint32_t slotIndex) const;
    const std::set<uint32_t>& GetGwVisibleSats(uint32_t gwId,
                                               uint32_t slotIndex) const;
    void PrintGwRouteReport() const;

    // ── v7：GW-to-UT 路由 API ─────────────────────────────────────────────
    //
    // 使用流程：
    //   1. AddGateway(...)          ← 同 v6，設定地面站（可與 GW-to-GW 共用）
    //   2. AddUserTerminal(...)     ← 設定 UT 位置
    //   3. AddGwUtPair(...)         ← 設定 (GW, UT) 路由對
    //   4. Initialize(...)
    //   5. PrecomputeAllTables()
    //   6. PrecomputeGwUtRoutes()   ← 必須在 PrecomputeAllTables 之後
    //   7. PrintGwUtRouteReport() / GetGwUtRoute()

    void AddUserTerminal(uint32_t utId, double latDeg, double lonDeg,
                         const std::string& name = "");
    void AddGwUtPair(uint32_t gwId, uint32_t utId);
    void PrecomputeGwUtRoutes();
    GwToUtRoute GetGwUtRoute(uint32_t gwId, uint32_t utId,
                             uint32_t slotIndex) const;
    const std::set<uint32_t>& GetUtVisibleSats(uint32_t utId,
                                               uint32_t slotIndex) const;
    void PrintGwUtRouteReport() const;

  private:
    // ── Routing ────────────────────────────────────────────────────────────
    std::vector<RouteEntry> ComputeRoutesForSrc(uint32_t src,
                                                const ISLGraph& graph) const;
    RoutingTable ComputeBaseRoutes(const ISLGraph& graph) const;
    RoutingTable ApplyTiebreaker(const RoutingTable& routes,
                                 const ISLGraph& graphNext) const;

    // ── Runtime ────────────────────────────────────────────────────────────
    void UpdateLoadCosts();
    bool HasSignificantChange() const;
    uint32_t RecomputeAffectedRoutes(uint32_t slotIndex);
    void RebuildIslSources(uint32_t slotIndex);
    double GetLinkQueueDelay(uint32_t satId, uint32_t ifIdx) const;

    // ── Setup ──────────────────────────────────────────────────────────────
    void LoadISLDefs(const std::string& islsFilePath);
    void InitOrbiterDevices();

    // ── 仰角計算（GW / UT 共用）────────────────────────────────────────────
    // 計算從地面觀測點 (obsLatDeg, obsLonDeg) 到衛星 ECEF 位置的仰角（度）
    static double ComputeElevationDeg(double obsLatDeg, double obsLonDeg,
                                      const Vector& satEcef);

  private:
    // ── Attributes ─────────────────────────────────────────────────────────
    uint32_t m_numSatellites;
    double   m_islMaxDistanceKm;
    uint32_t m_numTimeSlots;
    double   m_timeSlotInterval{60.0};
    std::string m_islsFilePath;
    double   m_emaAlpha{0.5};
    double   m_changeThreshold{0.2};
    double   m_cooldownSeconds{60.0};
    double   m_islLinkRateBps{10e6};

    // ── Internal sat routing data ──────────────────────────────────────────
    std::set<std::pair<uint32_t, uint32_t>> m_blockedEdges;

    std::vector<ISLDef>                                 m_islDefs;
    std::vector<std::map<uint32_t, uint32_t>>           m_perSatISLOrder;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t>   m_edgeOfPair;

    std::vector<Ptr<SatOrbiterNetDevice>>               m_orbDevs;
    std::vector<Ptr<Node>>                              m_orbNodes;
    std::vector<Ptr<SatIslArbiterUnicast>>              m_arbiters;

    std::vector<RoutingTable>                           m_tables;
    std::vector<double>                                 m_loadCosts;
    std::vector<double>                                 m_prevLoadCosts;
    std::vector<std::unordered_set<uint32_t>>           m_islSources;

    Time m_lastRecomputeTime;

    std::vector<SlotStats> m_stats;

    // ── v6：GW routing 內部資料 ────────────────────────────────────────────
    std::vector<GwDef>             m_gws;
    std::map<uint32_t, uint32_t>   m_gwIdToIdx;
    std::set<std::pair<uint32_t, uint32_t>> m_gwPairs;

    double m_gwElevThreshDeg{5.0};  // GW / UT 共用仰角門檻（度）

    // m_gwVisibility[slotIndex][gwId] = set of visible satIds
    std::vector<std::map<uint32_t, std::set<uint32_t>>>              m_gwVisibility;

    // m_gwRoutes[slotIndex][(srcGwId, dstGwId)] = GwToGwRoute
    std::vector<std::map<std::pair<uint32_t, uint32_t>, GwToGwRoute>> m_gwRoutes;

    std::set<uint32_t> m_emptyGwSatSet;

    // ── v7：UT routing 內部資料 ────────────────────────────────────────────
    std::vector<UtDef>             m_uts;
    std::map<uint32_t, uint32_t>   m_utIdToIdx;
    // m_gwUtPairs：(gwId, utId) 的有序 pair，單向（GW→UT）
    std::set<std::pair<uint32_t, uint32_t>> m_gwUtPairs;

    // m_utVisibility[slotIndex][utId] = set of visible satIds
    std::vector<std::map<uint32_t, std::set<uint32_t>>>               m_utVisibility;

    // m_gwUtRoutes[slotIndex][(gwId, utId)] = GwToUtRoute
    std::vector<std::map<std::pair<uint32_t, uint32_t>, GwToUtRoute>> m_gwUtRoutes;

    std::set<uint32_t> m_emptyUtSatSet;
};

} // namespace ns3

#endif // ISL_GRAPH_H
