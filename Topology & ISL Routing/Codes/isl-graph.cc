// isl-graph.cc
//
// 統一版本（無版本號）：整合 v6 GW-to-GW + v7 GW-to-UT 路由能力。
//
// v6 新增（相較 v5）：
//   將 GW-to-GW 路由內建至 IslRoutingManager，
//   src GW → entry sat → [ISL path] → exit sat → dst GW。
//
// v7 新增（相較 v6）：
//   GW-to-UT 路由，
//   GW → entry sat → [ISL path] → serving sat → UT。
//
// 主要函式：
//   AddGateway / AddGwPair / SetGwElevationThreshold
//   PrecomputeGwRoutes / GetGwRoute / PrintGwRouteReport
//   AddUserTerminal / AddGwUtPair
//   PrecomputeGwUtRoutes / GetGwUtRoute / PrintGwUtRouteReport
//   ComputeElevationDeg（GW / UT 共用）

#include "isl-graph.h"

#include "ns3/double.h"
#include "ns3/drop-tail-queue.h"
#include "ns3/packet.h"
#include "ns3/satellite-isl-arbiter-unicast.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(IslRoutingManager);

namespace
{

bool
HasIslTransitPath(const std::vector<uint32_t>& satPath)
{
    return satPath.size() > 1;
}

} // namespace

// ── 偵錯巨集 ──────────────────────────────────────────────────────────────

#define CHKPT(msg)                                                              \
    do                                                                          \
    {                                                                           \
        std::cout << "[CHKPT] " << Simulator::Now().GetSeconds() << "s | "      \
                  << msg << std::endl;                                          \
    } while (0)

#define WALL_START(var)                                                         \
    auto _wall_##var = std::chrono::steady_clock::now()

#define WALL_END_MS(var)                                                        \
    std::chrono::duration_cast<std::chrono::milliseconds>(                      \
        std::chrono::steady_clock::now() - _wall_##var)                         \
        .count()

// ── TypeId ────────────────────────────────────────────────────────────────

TypeId
IslRoutingManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::IslRoutingManager")
            .SetParent<Object>()
            .AddConstructor<IslRoutingManager>()
            .AddAttribute("NumSatellites",
                          "Number of satellites in the constellation",
                          UintegerValue(66),
                          MakeUintegerAccessor(&IslRoutingManager::m_numSatellites),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("IslMaxDistanceKm",
                          "Maximum ISL distance in km",
                          DoubleValue(5000.0),
                          MakeDoubleAccessor(&IslRoutingManager::m_islMaxDistanceKm),
                          MakeDoubleChecker<double>())
            .AddAttribute("NumTimeSlots",
                          "Number of precomputed time slots",
                          UintegerValue(10),
                          MakeUintegerAccessor(&IslRoutingManager::m_numTimeSlots),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("TimeSlotInterval",
                          "Interval between time slots in seconds",
                          DoubleValue(60.0),
                          MakeDoubleAccessor(&IslRoutingManager::m_timeSlotInterval),
                          MakeDoubleChecker<double>())
            .AddAttribute("IslsFilePath",
                          "Full path to isls.txt",
                          StringValue(""),
                          MakeStringAccessor(&IslRoutingManager::m_islsFilePath),
                          MakeStringChecker())
            .AddAttribute("EmaAlpha",
                          "EMA new-sample weight (0 < alpha <= 1)",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&IslRoutingManager::m_emaAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("ChangeThreshold",
                          "Fractional load-cost change that triggers recompute",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&IslRoutingManager::m_changeThreshold),
                          MakeDoubleChecker<double>())
            .AddAttribute("CooldownSeconds",
                          "Minimum seconds between partial recomputes",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&IslRoutingManager::m_cooldownSeconds),
                          MakeDoubleChecker<double>())
            .AddAttribute("IslLinkRateBps",
                          "ISL link rate in bps",
                          DoubleValue(10.0e6),
                          MakeDoubleAccessor(&IslRoutingManager::m_islLinkRateBps),
                          MakeDoubleChecker<double>());
    return tid;
}

// ── Constructor ───────────────────────────────────────────────────────────

IslRoutingManager::IslRoutingManager()
    : m_numSatellites(66),
      m_islMaxDistanceKm(5000.0),
      m_numTimeSlots(10),
      m_timeSlotInterval(60.0),
      m_islsFilePath(""),
      m_emaAlpha(0.3),
      m_changeThreshold(0.1),
      m_cooldownSeconds(30.0),
      m_islLinkRateBps(10.0e6),
      m_lastRecomputeTime(Seconds(0.0))
{
}

// ── Lifecycle ─────────────────────────────────────────────────────────────

void
IslRoutingManager::Initialize(const std::string& islsFilePath)
{
    CHKPT("Initialize: start");

    m_islsFilePath = islsFilePath;
    LoadISLDefs(m_islsFilePath);
    InitOrbiterDevices();

    const uint32_t n2 = m_numSatellites * m_numSatellites;
    m_loadCosts.assign(n2, 0.0);
    m_prevLoadCosts.assign(n2, 0.0);
    m_islSources.assign(m_islDefs.size(), std::unordered_set<uint32_t>());

    CHKPT("Initialize: done | satellites=" << m_numSatellites
                                           << " isls=" << m_islDefs.size());
}

void
IslRoutingManager::PrecomputeAllTables()
{
    CHKPT("PrecomputeAllTables: start | slots=" << m_numTimeSlots);
    WALL_START(total);

    m_tables.resize(m_numTimeSlots);

    std::vector<Vector> posCurr = GetPositionsAt(Seconds(0.0));
    ISLGraph graphCurr = BuildISLGraph(posCurr);

    for (uint32_t k = 0; k < m_numTimeSlots; k++)
    {
        WALL_START(slot);

        RoutingTable routes = ComputeBaseRoutes(graphCurr);
        long dijkstraMs = WALL_END_MS(slot);

        // NOTE（dead code 清理）：
        //   ApplyTiebreaker 呼叫已移除。原因：
        //   ComputeRoutesForSrc 對每個 dest 只 push_back 一筆 RouteEntry，
        //   導致 destToIndices[dest].size() 永遠 == 1，
        //   tie-breaking 分支（indices.size() <= 1 continue）永遠跳過，
        //   整個函式是 no-op 且會白建一張 graphNext 圖。
        //
        //   若未來要啟用 stability-preference tiebreaking，需：
        //     1. 改造 Dijkstra，在 nd == dist[e.nodeB] 時也記錄 alternative first-hop
        //     2. entries 允許同一 dest 出現多筆（不同 nextHopSatId）
        //     3. ApplyTiebreaker 才能從多筆中選出在 graphNext 仍有效的那筆
        //
        //   目前仍推進 graphCurr，確保下一 slot 從正確的衛星位置圖出發。
        if (k < m_numTimeSlots - 1)
        {
            Time tauNext = Seconds(m_timeSlotInterval * (k + 1));
            std::vector<Vector> posNext = GetPositionsAt(tauNext);
            graphCurr = BuildISLGraph(posNext); // advance for next slot
        }

        m_tables[k] = std::move(routes);

        long slotMs = WALL_END_MS(slot);
        std::cout << "[CHKPT] PrecomputeAllTables: slot=" << k
                  << " t=" << (m_timeSlotInterval * k)
                  << "s | SAT0_routes="
                  << (m_tables[k].empty() ? 0 : m_tables[k][0].size())
                  << " dijkstra=" << dijkstraMs << "ms"
                  << " total=" << slotMs << "ms"
                  << std::endl;
    }

    long totalMs = WALL_END_MS(total);
    CHKPT("PrecomputeAllTables: complete | wall=" << totalMs << "ms");
}

void
IslRoutingManager::ScheduleRoutingUpdates()
{
    for (uint32_t k = 0; k < m_numTimeSlots; k++)
    {
        Time t = Seconds(m_timeSlotInterval * k);
        Simulator::Schedule(t, &IslRoutingManager::ApplyRoutingTable, this, k);
    }

    CHKPT("ScheduleRoutingUpdates: " << m_numTimeSlots << " events scheduled");
}

void
IslRoutingManager::ApplyRoutingTable(uint32_t slotIndex)
{
    WALL_START(apply);

    NS_ASSERT_MSG(slotIndex < m_tables.size(),
                  "slotIndex out of range: " << slotIndex);

    bool     didRecompute      = false;
    long     recomputeMs       = 0;
    uint32_t recomputedSources = 0;

    if (slotIndex > 0)
    {
        UpdateLoadCosts();
        bool changed = HasSignificantChange();

        CHKPT("ApplyRoutingTable: slot=" << slotIndex
                                         << " HasSignificantChange="
                                         << (changed ? "YES" : "NO"));

        if (changed)
        {
            WALL_START(recompute);
            recomputedSources = RecomputeAffectedRoutes(slotIndex);
            recomputeMs       = WALL_END_MS(recompute);
            didRecompute      = true;

            CHKPT("RecomputeAffectedRoutes: slot=" << slotIndex
                                                   << " recomputed="
                                                   << recomputedSources
                                                   << "/" << m_numSatellites
                                                   << " wall="
                                                   << recomputeMs << "ms");
        }
    }

    const RoutingTable& table = m_tables[slotIndex];

    NS_ASSERT_MSG(m_arbiters.size() == m_numSatellites, "m_arbiters size mismatch");
    NS_ASSERT_MSG(m_orbDevs.size() == m_numSatellites, "m_orbDevs size mismatch");

    for (uint32_t satId = 0; satId < m_numSatellites; satId++)
    {
        NS_ASSERT_MSG(m_arbiters[satId], "Null arbiter at satId=" << satId);
        NS_ASSERT_MSG(m_orbDevs[satId], "Null orbiter net device at satId=" << satId);

        m_arbiters[satId]->ClearNextHopEntries();

        for (const auto& entry : table[satId])
        {
            m_arbiters[satId]->AddNextHopEntry(entry.destSatId, entry.islIfIndexOnA);
        }

        m_orbDevs[satId]->SetArbiter(m_arbiters[satId]);
    }

    RebuildIslSources(slotIndex);

    long applyMs = WALL_END_MS(apply);

    SlotStats s;
    s.slotIndex        = slotIndex;
    s.simTimeSec       = Simulator::Now().GetSeconds();
    s.applyWallMs      = applyMs;
    s.recomputeWallMs  = recomputeMs;
    s.recomputedSources = recomputedSources;
    s.significantChange = didRecompute;
    m_stats.push_back(s);

    CHKPT("ApplyRoutingTable: slot=" << slotIndex
                                     << " t=" << s.simTimeSec << "s"
                                     << " | apply=" << applyMs << "ms"
                                     << " recompute=" << recomputeMs << "ms"
                                     << " recomputedSrc=" << recomputedSources);
}

// ── Graph 建構 ────────────────────────────────────────────────────────────

std::vector<Vector>
IslRoutingManager::GetPositionsAt(Time tau) const
{
    std::vector<Vector> pos(m_numSatellites);

    for (uint32_t i = 0; i < m_numSatellites; i++)
    {
        Ptr<SatSGP4MobilityModel> mob =
            m_orbNodes[i]->GetObject<SatSGP4MobilityModel>();

        NS_ASSERT_MSG(mob, "Sat " << i << " has no SatSGP4MobilityModel");
        pos[i] = mob->GetGeoPositionAt(tau).ToVector();
    }

    return pos;
}

ISLGraph
IslRoutingManager::BuildISLGraph(const std::vector<Vector>& pos) const
{
    const double C       = 3e8;
    const double maxDist = m_islMaxDistanceKm * 1e3;

    ISLGraph graph(m_numSatellites);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double dx   = pos[a].x - pos[b].x;
        double dy   = pos[a].y - pos[b].y;
        double dz   = pos[a].z - pos[b].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > maxDist)
            continue;

        if (m_blockedEdges.count({a, b}))
            continue;

        double   propCost = dist / C;
        uint32_t ifIdxOnA = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifIdxOnB = m_perSatISLOrder[b].at(edgeIdx);

        graph[a].push_back({b, propCost, ifIdxOnA, ifIdxOnB});
        graph[b].push_back({a, propCost, ifIdxOnB, ifIdxOnA});
    }

    return graph;
}

ISLGraph
IslRoutingManager::BuildISLGraphWithLoad(const std::vector<Vector>& pos) const
{
    const double   C       = 3e8;
    const double   maxDist = m_islMaxDistanceKm * 1e3;
    const uint32_t N       = m_numSatellites;

    ISLGraph graph(N);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double dx   = pos[a].x - pos[b].x;
        double dy   = pos[a].y - pos[b].y;
        double dz   = pos[a].z - pos[b].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > maxDist)
            continue;

        if (m_blockedEdges.count({a, b}))
            continue;

        double   propCost = dist / C;
        uint32_t ifIdxOnA = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifIdxOnB = m_perSatISLOrder[b].at(edgeIdx);

        double loadAb = m_loadCosts[a * N + b];
        double loadBa = m_loadCosts[b * N + a];

        graph[a].push_back({b, propCost + loadAb, ifIdxOnA, ifIdxOnB});
        graph[b].push_back({a, propCost + loadBa, ifIdxOnB, ifIdxOnA});
    }

    return graph;
}

// ── Dijkstra 路由計算 ─────────────────────────────────────────────────────

std::vector<RouteEntry>
IslRoutingManager::ComputeRoutesForSrc(uint32_t src, const ISLGraph& graph) const
{
    const uint32_t N   = m_numSatellites;
    const double   INF = std::numeric_limits<double>::infinity();

    std::vector<double>   dist(N, INF);
    std::vector<uint32_t> firstHopNode(N, UINT32_MAX);
    std::vector<uint32_t> firstHopIf(N, UINT32_MAX);

    dist[src] = 0.0;

    using P = std::pair<double, uint32_t>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0, src});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u])
            continue;

        for (const auto& e : graph[u])
        {
            double nd = dist[u] + e.propagation_cost;
            if (nd < dist[e.nodeB])
            {
                dist[e.nodeB] = nd;

                if (u == src)
                {
                    firstHopNode[e.nodeB] = e.nodeB;
                    firstHopIf[e.nodeB]   = e.islIfIndexOnA;
                }
                else
                {
                    firstHopNode[e.nodeB] = firstHopNode[u];
                    firstHopIf[e.nodeB]   = firstHopIf[u];
                }

                pq.push({nd, e.nodeB});
            }
        }
    }

    std::vector<RouteEntry> entries;
    entries.reserve(N > 0 ? N - 1 : 0);

    for (uint32_t dest = 0; dest < N; dest++)
    {
        if (dest == src || dist[dest] == INF)
            continue;

        entries.push_back({dest, firstHopNode[dest], firstHopIf[dest], dist[dest]});
    }

    return entries;
}

RoutingTable
IslRoutingManager::ComputeBaseRoutes(const ISLGraph& graph) const
{
    RoutingTable result(m_numSatellites);
    for (uint32_t src = 0; src < m_numSatellites; src++)
        result[src] = ComputeRoutesForSrc(src, graph);
    return result;
}

RoutingTable
IslRoutingManager::ApplyTiebreaker(const RoutingTable& routes,
                                   const ISLGraph& graphNext) const
{
    struct PairHash
    {
        size_t operator()(const std::pair<uint32_t, uint32_t>& p) const
        {
            return std::hash<uint64_t>{}(
                (static_cast<uint64_t>(p.first) << 32) | p.second);
        }
    };

    std::unordered_set<std::pair<uint32_t, uint32_t>, PairHash> nextEligible;
    nextEligible.reserve(graphNext.size() * 4);

    for (uint32_t u = 0; u < graphNext.size(); u++)
        for (const auto& e : graphNext[u])
            nextEligible.insert({u, e.nodeB});

    RoutingTable result = routes;

    for (uint32_t src = 0; src < routes.size(); src++)
    {
        std::map<uint32_t, std::vector<size_t>> destToIndices;
        for (size_t i = 0; i < routes[src].size(); i++)
            destToIndices[routes[src][i].destSatId].push_back(i);

        for (auto& [dest, indices] : destToIndices)
        {
            (void)dest;
            if (indices.size() <= 1)
                continue;

            double minCost = routes[src][indices[0]].cost;

            std::vector<size_t> tied;
            for (size_t idx : indices)
                if (std::abs(routes[src][idx].cost - minCost) < 1e-9)
                    tied.push_back(idx);

            if (tied.size() <= 1)
                continue;

            for (size_t idx : tied)
            {
                uint32_t nh = routes[src][idx].nextHopSatId;
                if (nextEligible.count({src, nh}))
                {
                    for (size_t other : tied)
                        if (other != idx)
                            result[src][other].cost =
                                std::numeric_limits<double>::infinity();
                    break;
                }
            }
        }

        auto& vec = result[src];
        vec.erase(std::remove_if(vec.begin(), vec.end(),
                                 [](const RouteEntry& e) {
                                     return e.cost ==
                                            std::numeric_limits<double>::infinity();
                                 }),
                  vec.end());
    }

    return result;
}

// ── Load cost 管理 ────────────────────────────────────────────────────────

double
IslRoutingManager::GetLinkQueueDelay(uint32_t satId, uint32_t ifIdx) const
{
    if (satId >= m_orbDevs.size() || !m_orbDevs[satId])
        return 0.0;

    auto islDevs = m_orbDevs[satId]->GetIslsNetDevices();

    if (ifIdx >= islDevs.size() || !islDevs[ifIdx])
        return 0.0;

    Ptr<DropTailQueue<Packet>> q = islDevs[ifIdx]->GetQueue();
    if (!q)
        return 0.0;

    // 使用佇列實際持有的 byte 數，而非封包數 × 固定 1500 bytes。
    // 原先 nPackets * 1500 的估算在 SNS3 控制平面封包（通常遠小於 1500B）
    // 下會嚴重高估 queue delay，導致 trafficProfile=none 時出現大量假性非零值。
    uint32_t nBytes = q->GetNBytes();
    double   bits   = static_cast<double>(nBytes) * 8.0;

    if (m_islLinkRateBps <= 0.0)
        return 0.0;

    return bits / m_islLinkRateBps;
}

void
IslRoutingManager::UpdateLoadCosts()
{
    m_prevLoadCosts    = m_loadCosts;
    const uint32_t N   = m_numSatellites;

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a    = m_islDefs[edgeIdx].nodeA;
        uint32_t b    = m_islDefs[edgeIdx].nodeB;
        uint32_t ifA  = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifB  = m_perSatISLOrder[b].at(edgeIdx);

        double sampleAb = GetLinkQueueDelay(a, ifA);
        double sampleBa = GetLinkQueueDelay(b, ifB);

        m_loadCosts[a * N + b] =
            m_emaAlpha * sampleAb + (1.0 - m_emaAlpha) * m_loadCosts[a * N + b];
        m_loadCosts[b * N + a] =
            m_emaAlpha * sampleBa + (1.0 - m_emaAlpha) * m_loadCosts[b * N + a];
    }
}

bool
IslRoutingManager::HasSignificantChange() const
{
    double elapsed = (Simulator::Now() - m_lastRecomputeTime).GetSeconds();

    if (elapsed < m_cooldownSeconds)
    {
        CHKPT("HasSignificantChange: cooldown active, elapsed="
              << elapsed << "s < " << m_cooldownSeconds << "s");
        return false;
    }

    const uint32_t N = m_numSatellites;

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double prevAb = m_prevLoadCosts[a * N + b];
        double currAb = m_loadCosts[a * N + b];
        double prevBa = m_prevLoadCosts[b * N + a];
        double currBa = m_loadCosts[b * N + a];

        double refAb = std::max(prevAb, currAb);
        double refBa = std::max(prevBa, currBa);

        bool changed =
            (refAb > 1e-12 &&
             std::abs(currAb - prevAb) / refAb > m_changeThreshold) ||
            (refBa > 1e-12 &&
             std::abs(currBa - prevBa) / refBa > m_changeThreshold);

        if (changed)
            return true;
    }

    return false;
}

uint32_t
IslRoutingManager::RecomputeAffectedRoutes(uint32_t slotIndex)
{
    Time               now   = Simulator::Now();
    std::vector<Vector> pos  = GetPositionsAt(now);
    ISLGraph            graph = BuildISLGraphWithLoad(pos);

    const uint32_t      N       = m_numSatellites;
    std::vector<bool>   affected(N, false);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double prevAb = m_prevLoadCosts[a * N + b];
        double currAb = m_loadCosts[a * N + b];
        double prevBa = m_prevLoadCosts[b * N + a];
        double currBa = m_loadCosts[b * N + a];

        double refAb = std::max(prevAb, currAb);
        double refBa = std::max(prevBa, currBa);

        bool changed =
            (refAb > 1e-12 &&
             std::abs(currAb - prevAb) / refAb > m_changeThreshold) ||
            (refBa > 1e-12 &&
             std::abs(currBa - prevBa) / refBa > m_changeThreshold);

        if (changed)
        {
            for (uint32_t src : m_islSources[edgeIdx])
                affected[src] = true;
        }
    }

    uint32_t      recomputedCount = 0;
    RoutingTable& table           = m_tables[slotIndex];

    for (uint32_t src = 0; src < N; src++)
    {
        if (!affected[src])
            continue;
        table[src] = ComputeRoutesForSrc(src, graph);
        ++recomputedCount;
    }

    m_lastRecomputeTime = now;
    RebuildIslSources(slotIndex);

    // Bug 4 fix：m_tables[slotIndex] 已被部分重算，同步更新 GW/UT 路由報表，
    // 避免 PrintGwRouteReport / GetGwRoute 與實際 forwarding path 脫節。
    if (recomputedCount > 0)
        RefreshGwRoutesForSlot(slotIndex);

    return recomputedCount;
}

void
IslRoutingManager::RebuildIslSources(uint32_t slotIndex)
{
    // 重建每條 ISL edge 對應的「以此 edge 為第一跳的 source 衛星集合」
    m_islSources.assign(m_islDefs.size(), std::unordered_set<uint32_t>());

    const RoutingTable& table = m_tables[slotIndex];

    for (uint32_t src = 0; src < m_numSatellites; src++)
    {
        std::unordered_set<uint32_t> seenNh;

        for (const auto& entry : table[src])
        {
            if (!seenNh.insert(entry.nextHopSatId).second)
                continue;

            auto it = m_edgeOfPair.find({src, entry.nextHopSatId});
            if (it != m_edgeOfPair.end())
                m_islSources[it->second].insert(src);
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────

void
IslRoutingManager::LoadISLDefs(const std::string& islsFilePath)
{
    CHKPT("LoadISLDefs: start | path=" << islsFilePath);

    m_islDefs.clear();
    m_perSatISLOrder.assign(m_numSatellites, std::map<uint32_t, uint32_t>());
    m_edgeOfPair.clear();

    std::ifstream file(islsFilePath);
    NS_ASSERT_MSG(file.is_open(), "Cannot open isls.txt: " << islsFilePath);

    std::string line;
    std::getline(file, line); // 跳過 header

    std::vector<uint32_t> counter(m_numSatellites, 0);

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::istringstream iss(line);
        uint32_t a = 0;
        uint32_t b = 0;
        iss >> a >> b;

        // 防呆 1：解析失敗（格式錯誤 / 非數字）→ 跳過並警告
        if (iss.fail())
        {
            std::cerr << "[LoadISLDefs] WARN: failed to parse line '"
                      << line << "', skipping\n";
            continue;
        }

        // 防呆 2：衛星 ID 超界 → 立即 fatal，避免 out-of-bounds UB
        // 常見原因：isls.txt 與 NumSatellites 屬性不一致
        if (a >= m_numSatellites)
            NS_FATAL_ERROR("LoadISLDefs: node a=" << a
                           << " >= numSatellites=" << m_numSatellites
                           << " (line: '" << line << "')");
        if (b >= m_numSatellites)
            NS_FATAL_ERROR("LoadISLDefs: node b=" << b
                           << " >= numSatellites=" << m_numSatellites
                           << " (line: '" << line << "')");

        uint32_t edgeIdx = static_cast<uint32_t>(m_islDefs.size());
        m_islDefs.push_back({a, b});

        m_perSatISLOrder[a][edgeIdx] = counter[a]++;
        m_perSatISLOrder[b][edgeIdx] = counter[b]++;

        m_edgeOfPair[{a, b}] = edgeIdx;
        m_edgeOfPair[{b, a}] = edgeIdx;
    }

    CHKPT("LoadISLDefs: done | loaded=" << m_islDefs.size() << " ISLs");
}

void
IslRoutingManager::InitOrbiterDevices()
{
    CHKPT("InitOrbiterDevices: start");

    m_orbDevs.resize(m_numSatellites);
    m_orbNodes.resize(m_numSatellites);
    m_arbiters.resize(m_numSatellites);

    for (uint32_t i = 0; i < m_numSatellites; i++)
    {
        Ptr<Node> sat = Singleton<SatTopology>::Get()->GetOrbiterNode(i);
        m_orbNodes[i] = sat;

        for (uint32_t d = 0; d < sat->GetNDevices(); d++)
        {
            Ptr<SatOrbiterNetDevice> dev =
                DynamicCast<SatOrbiterNetDevice>(sat->GetDevice(d));
            if (dev)
            {
                m_orbDevs[i] = dev;
                break;
            }
        }

        NS_ASSERT_MSG(m_orbDevs[i], "no SatOrbiterNetDevice on sat=" << i);

        m_arbiters[i] = CreateObject<SatIslArbiterUnicast>(sat);
    }

    CHKPT("InitOrbiterDevices: done | satellites=" << m_numSatellites);
}

// ── Stats ─────────────────────────────────────────────────────────────────

void
IslRoutingManager::PrintStats() const
{
    std::cout << "\n=== IslRoutingManager Stats ===" << std::endl;
    std::cout << std::left
              << std::setw(6)  << "slot"
              << std::setw(10) << "simTime"
              << std::setw(12) << "apply(ms)"
              << std::setw(16) << "recompute(ms)"
              << std::setw(16) << "recompSrc"
              << std::setw(12) << "changed"
              << std::endl;

    for (const auto& s : m_stats)
    {
        std::cout << std::left
                  << std::setw(6)  << s.slotIndex
                  << std::setw(10) << s.simTimeSec
                  << std::setw(12) << s.applyWallMs
                  << std::setw(16) << s.recomputeWallMs
                  << std::setw(16) << s.recomputedSources
                  << std::setw(12) << (s.significantChange ? "YES" : "NO")
                  << std::endl;
    }

    std::cout << "==============================\n" << std::endl;
}

// ── PrintLoadStats ────────────────────────────────────────────────────────
// 輸出每條 ISL 的最終 EMA load cost（佇列延遲，單位 ms）。
// 僅印出 loadAB 或 loadBA 非零的鏈路（有流量才有非零值）。
// 用途：
//   1. 驗證 CBR 流量是否確實流過 ISL 層
//   2. 確認 UpdateLoadCosts + EMA 平滑機制在運行
//   3. 觀察負載分布，作為後續 QoS 調整依據
void
IslRoutingManager::PrintLoadStats() const
{
    const uint32_t N = m_numSatellites;

    std::cout << "\n=== ISL Load Cost Summary (EMA queue delay) ===" << std::endl;
    std::cout << std::left
              << std::setw(8)  << "edgeIdx"
              << std::setw(8)  << "satA"
              << std::setw(8)  << "satB"
              << std::setw(16) << "loadAB(ms)"
              << std::setw(16) << "loadBA(ms)"
              << std::endl;

    uint32_t nonZeroLinks = 0;

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        // load cost 單位為秒（queue bits / link rate），轉換為 ms 方便閱讀
        double loadAbMs = m_loadCosts[a * N + b] * 1000.0;
        double loadBaMs = m_loadCosts[b * N + a] * 1000.0;

        // 僅印出有流量的鏈路（threshold 1e-6 ms 避免浮點誤差干擾）
        if (loadAbMs > 1e-6 || loadBaMs > 1e-6)
        {
            std::cout << std::left
                      << std::setw(8)  << edgeIdx
                      << std::setw(8)  << a
                      << std::setw(8)  << b
                      << std::setw(16) << std::fixed << std::setprecision(4) << loadAbMs
                      << std::setw(16) << loadBaMs
                      << std::endl;
            nonZeroLinks++;
        }
    }

    if (nonZeroLinks == 0)
    {
        // 可能原因：
        //   - 流量未通過 ISL 層（僅在單顆衛星 feeder link 內完成）
        //   - simTime 太短，流量尚未到達 ISL
        //   - TrafficConfig::enableFwd / enableRtn 均為 false
        std::cout << "(no ISL load detected — traffic may not have reached ISL layer)\n";
    }

    std::cout << "Loaded ISL links: " << nonZeroLinks
              << " / " << m_islDefs.size()
              << " total ISL edges\n";
    std::cout << "================================================\n" << std::endl;
}

// ── Accessors ─────────────────────────────────────────────────────────────

double
IslRoutingManager::GetRouteCost(uint32_t src, uint32_t dst, uint32_t slotIndex) const
{
    if (slotIndex >= m_tables.size() || src >= m_tables[slotIndex].size())
        return std::numeric_limits<double>::infinity();
    for (const auto& e : m_tables[slotIndex][src])
        if (e.destSatId == dst)
            return e.cost;
    return std::numeric_limits<double>::infinity();
}

// ── Verification helpers ──────────────────────────────────────────────────

std::vector<uint32_t>
IslRoutingManager::TracePath(uint32_t src, uint32_t dst, uint32_t slotIndex) const
{
    std::vector<uint32_t> path;

    if (slotIndex >= m_tables.size())
        return path;

    path.push_back(src);
    uint32_t curr = src;

    for (uint32_t hop = 0; hop < m_numSatellites; ++hop)
    {
        if (curr == dst)
            break;

        bool found = false;
        for (const auto& e : m_tables[slotIndex][curr])
        {
            if (e.destSatId == dst)
            {
                path.push_back(e.nextHopSatId);
                curr  = e.nextHopSatId;
                found = true;
                break;
            }
        }

        if (!found)
        {
            path.push_back(UINT32_MAX);
            break;
        }
    }

    return path;
}

void
IslRoutingManager::PrintRouteReport(
    const std::vector<std::pair<uint32_t, uint32_t>>& pairs) const
{
    std::cout << "\n=== Route Report: Full Paths Across Slots ===\n";
    std::cout << std::left
              << std::setw(10) << "time(s)"
              << std::setw(6)  << "src"
              << std::setw(6)  << "dst"
              << std::setw(44) << "full_path"
              << std::setw(14) << "route_cost"
              << std::setw(6)  << "slot"
              << "\n"
              << std::string(86, '-') << "\n";

    for (const auto& [src, dst] : pairs)
    {
        std::string prevPathStr;

        for (uint32_t k = 0; k < m_numTimeSlots; ++k)
        {
            double                timeSec = k * m_timeSlotInterval;
            double                cost    = GetRouteCost(src, dst, k);
            std::vector<uint32_t> path    = TracePath(src, dst, k);

            std::ostringstream pathSs;
            for (size_t i = 0; i < path.size(); ++i)
            {
                if (i > 0) pathSs << "->";
                pathSs << (path[i] == UINT32_MAX ? "?" : std::to_string(path[i]));
            }
            std::string pathStr = pathSs.str();

            bool changed = (k > 0 && pathStr != prevPathStr);

            std::cout << std::left
                      << std::setw(10) << timeSec
                      << std::setw(6)  << src
                      << std::setw(6)  << dst
                      << std::setw(44) << pathStr
                      << std::setw(14) << std::fixed << std::setprecision(6) << cost
                      << std::setw(6)  << k;

            if (changed)
                std::cout << "  <-- PATH CHANGED";

            std::cout << "\n";
            prevPathStr = pathStr;
        }

        std::cout << "\n";
    }

    std::cout << "==============================================\n";
}

void
IslRoutingManager::BlockISL(uint32_t nodeA, uint32_t nodeB)
{
    m_blockedEdges.insert({nodeA, nodeB});
    m_blockedEdges.insert({nodeB, nodeA});
}

void
IslRoutingManager::UnblockISL(uint32_t nodeA, uint32_t nodeB)
{
    m_blockedEdges.erase({nodeA, nodeB});
    m_blockedEdges.erase({nodeB, nodeA});
}

void
IslRoutingManager::RunAvoidanceTest(uint32_t testSrc,
                                     uint32_t testDst,
                                     uint32_t slotIndex)
{
    std::cout << "\n=== ISL Avoidance Test =========================\n";

    std::vector<uint32_t> baselinePath = TracePath(testSrc, testDst, slotIndex);
    double                baselineCost = GetRouteCost(testSrc, testDst, slotIndex);

    auto printPath = [](const std::vector<uint32_t>& p) {
        for (size_t i = 0; i < p.size(); ++i)
        {
            if (i > 0) std::cout << "->";
            if (p[i] == UINT32_MAX) std::cout << "?";
            else std::cout << p[i];
        }
    };

    std::cout << "Pair      : " << testSrc << " -> " << testDst
              << "  (slot " << slotIndex << ", t="
              << slotIndex * m_timeSlotInterval << "s)\n";
    std::cout << "Baseline  : ";
    printPath(baselinePath);
    std::cout << "  cost=" << std::fixed << std::setprecision(6) << baselineCost << "\n";

    if (baselinePath.size() < 2 || baselinePath.back() == UINT32_MAX)
    {
        std::cout << "  [SKIP] No reachable path — cannot perform avoidance test.\n";
        std::cout << "================================================\n";
        return;
    }

    uint32_t blockA = baselinePath[0];
    uint32_t blockB = baselinePath[1];

    std::cout << "Block ISL : " << blockA << " <-> " << blockB << "\n";

    RoutingTable savedTable = m_tables[slotIndex];

    BlockISL(blockA, blockB);

    Time             tau          = Seconds(m_timeSlotInterval * slotIndex);
    std::vector<Vector> pos       = GetPositionsAt(tau);
    ISLGraph         graphBlocked = BuildISLGraph(pos);
    m_tables[slotIndex]           = ComputeBaseRoutes(graphBlocked);

    std::vector<uint32_t> newPath = TracePath(testSrc, testDst, slotIndex);
    double                newCost = GetRouteCost(testSrc, testDst, slotIndex);

    std::cout << "Post-block: ";
    printPath(newPath);
    if (newPath.empty() || newPath.back() == UINT32_MAX)
        std::cout << "  (unreachable)";
    else
        std::cout << "  cost=" << std::fixed << std::setprecision(6) << newCost;
    std::cout << "\n";

    bool pathAvoidsISL = true;
    for (size_t i = 0; i + 1 < newPath.size(); ++i)
    {
        if ((newPath[i] == blockA && newPath[i + 1] == blockB) ||
            (newPath[i] == blockB && newPath[i + 1] == blockA))
        {
            pathAvoidsISL = false;
            break;
        }
    }

    bool noStaleNextHop = true;
    if (testSrc == blockA)
    {
        for (const auto& e : m_tables[slotIndex][testSrc])
        {
            if (e.destSatId == testDst && e.nextHopSatId == blockB)
            {
                noStaleNextHop = false;
                break;
            }
        }
    }

    std::cout << "Check (a) ISL absent from path : "
              << (pathAvoidsISL ? "PASS" : "FAIL") << "\n";
    std::cout << "Check (b) No stale next-hop    : "
              << (noStaleNextHop ? "PASS" : "FAIL") << "\n";

    UnblockISL(blockA, blockB);
    m_tables[slotIndex] = std::move(savedTable);

    std::cout << "Routing table restored.\n";
    std::cout << "================================================\n";
}

// ═══════════════════════════════════════════════════════════════════════════
// v6：GW-to-GW 路由
// ═══════════════════════════════════════════════════════════════════════════

// ── 仰角計算（GW / UT 共用）──────────────────────────────────────────────
//
// 計算從地面觀測點 (obsLatDeg, obsLonDeg, alt=0) 到衛星 ECEF 位置的仰角（度）。
// 使用球形地球模型（半徑 6371 km）。
// 回傳值範圍：-90° ~ 90°；-90° 表示計算失敗（衛星距離 < 1 m）。
//
// static
double
IslRoutingManager::ComputeElevationDeg(double        obsLatDeg,
                                        double        obsLonDeg,
                                        const Vector& satEcef)
{
    static const double EARTH_RADIUS_M = 6371000.0;

    const double latRad = obsLatDeg * M_PI / 180.0;
    const double lonRad = obsLonDeg * M_PI / 180.0;

    const double ox = EARTH_RADIUS_M * std::cos(latRad) * std::cos(lonRad);
    const double oy = EARTH_RADIUS_M * std::cos(latRad) * std::sin(lonRad);
    const double oz = EARTH_RADIUS_M * std::sin(latRad);

    const double dx   = satEcef.x - ox;
    const double dy   = satEcef.y - oy;
    const double dz   = satEcef.z - oz;
    const double dLen = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dLen < 1.0)
        return -90.0;

    const double oLen    = std::sqrt(ox * ox + oy * oy + oz * oz);
    const double dotProd = (dx * ox + dy * oy + dz * oz) / (dLen * oLen);

    return std::asin(std::max(-1.0, std::min(1.0, dotProd))) * 180.0 / M_PI;
}

// ── GW 設定 API ────────────────────────────────────────────────────────────

void
IslRoutingManager::AddGateway(uint32_t           gwId,
                               double             latDeg,
                               double             lonDeg,
                               const std::string& name)
{
    NS_ASSERT_MSG(!m_gwIdToIdx.count(gwId),
                  "AddGateway: gwId=" << gwId << " already exists");
    m_gwIdToIdx[gwId] = static_cast<uint32_t>(m_gws.size());
    m_gws.push_back({gwId, latDeg, lonDeg, name});
}

void
IslRoutingManager::AddGwPair(uint32_t gwA, uint32_t gwB)
{
    // 防呆：未透過 AddGateway 註冊的 ID → 立即 fatal，避免在報表時靜默失敗
    NS_ASSERT_MSG(m_gwIdToIdx.count(gwA),
                  "AddGwPair: gwA=" << gwA << " not registered — call AddGateway() first");
    NS_ASSERT_MSG(m_gwIdToIdx.count(gwB),
                  "AddGwPair: gwB=" << gwB << " not registered — call AddGateway() first");

    // 雙向插入，正反向各自獨立計算最佳路由
    m_gwPairs.insert({gwA, gwB});
    m_gwPairs.insert({gwB, gwA});
}

void
IslRoutingManager::SetGwElevationThreshold(double deg)
{
    // 同時作用於 GW 與 UT 的仰角門檻
    m_gwElevThreshDeg = deg;
}

const std::set<uint32_t>&
IslRoutingManager::GetGwVisibleSats(uint32_t gwId, uint32_t slotIndex) const
{
    if (slotIndex >= m_gwVisibility.size())
        return m_emptyGwSatSet;
    auto it = m_gwVisibility[slotIndex].find(gwId);
    return (it != m_gwVisibility[slotIndex].end()) ? it->second : m_emptyGwSatSet;
}

// ── PrecomputeGwRoutes ────────────────────────────────────────────────────
//
// 預計算所有 time slot 的 GW-to-GW 最佳路由：
//   1. 計算每個 GW 的可見衛星集合（仰角 >= m_gwElevThreshDeg）
//   2. 對每個 contracted pair (A, B)：
//      ‧ 枚舉 (entry ∈ vis(A)) × (exit ∈ vis(B))，選最小 ISL cost
//      ‧ TracePath(entry, exit) 重建路徑
//      ‧ 雙向各自獨立計算
//
// 必須在 PrecomputeAllTables() 之後呼叫。
void
IslRoutingManager::PrecomputeGwRoutes()
{
    NS_ASSERT_MSG(!m_tables.empty(),
                  "PrecomputeGwRoutes: call PrecomputeAllTables() first");
    NS_ASSERT_MSG(!m_gws.empty(),
                  "PrecomputeGwRoutes: no gateways — call AddGateway() first");

    CHKPT("PrecomputeGwRoutes: start"
          << " | gws=" << m_gws.size()
          << " pairs=" << (m_gwPairs.size() / 2)
          << " slots=" << m_numTimeSlots);

    WALL_START(gw);

    m_gwVisibility.resize(m_numTimeSlots);
    m_gwRoutes.resize(m_numTimeSlots);

    std::set<std::pair<uint32_t, uint32_t>> processedPairs;

    for (uint32_t k = 0; k < m_numTimeSlots; ++k)
    {
        Time                tau    = Seconds(k * m_timeSlotInterval);
        std::vector<Vector> satPos = GetPositionsAt(tau);

        // Step 1：計算每個 GW 的可見衛星
        for (const auto& gw : m_gws)
        {
            std::set<uint32_t>& vis = m_gwVisibility[k][gw.gwId];
            for (uint32_t s = 0; s < m_numSatellites; ++s)
            {
                if (ComputeElevationDeg(gw.latDeg, gw.lonDeg, satPos[s]) >=
                    m_gwElevThreshDeg)
                    vis.insert(s);
            }
        }

        // Step 2：對每個 contracted pair 計算最佳 GW-to-GW 路由
        processedPairs.clear();
        for (const auto& p : m_gwPairs)
        {
            uint32_t gwA = p.first;
            uint32_t gwB = p.second;

            uint32_t lo = std::min(gwA, gwB);
            uint32_t hi = std::max(gwA, gwB);
            if (!processedPairs.insert({lo, hi}).second)
                continue;

            const auto& srcSatsAB = GetGwVisibleSats(gwA, k);
            const auto& dstSatsAB = GetGwVisibleSats(gwB, k);
            const auto& srcSatsBA = dstSatsAB;
            const auto& dstSatsBA = srcSatsAB;

            GwToGwRoute bestAB;
            bestAB.srcGwId = gwA;
            bestAB.dstGwId = gwB;

            for (uint32_t entry : srcSatsAB)
            {
                for (uint32_t exit : dstSatsAB)
                {
                    double cost = (entry == exit)
                                      ? 0.0
                                      : GetRouteCost(entry, exit, k);

                    if (cost < bestAB.islCost)
                    {
                        bestAB.entrySatId = entry;
                        bestAB.exitSatId  = exit;
                        bestAB.islCost    = cost;
                        bestAB.satPath    = (entry == exit)
                                                ? std::vector<uint32_t>{entry}
                                                : TracePath(entry, exit, k);
                        bestAB.valid      = true;
                    }
                }
            }

            GwToGwRoute bestBA;
            bestBA.srcGwId = gwB;
            bestBA.dstGwId = gwA;

            for (uint32_t entry : srcSatsBA)
            {
                for (uint32_t exit : dstSatsBA)
                {
                    double cost = (entry == exit)
                                      ? 0.0
                                      : GetRouteCost(entry, exit, k);

                    if (cost < bestBA.islCost)
                    {
                        bestBA.entrySatId = entry;
                        bestBA.exitSatId  = exit;
                        bestBA.islCost    = cost;
                        bestBA.satPath    = (entry == exit)
                                                ? std::vector<uint32_t>{entry}
                                                : TracePath(entry, exit, k);
                        bestBA.valid      = true;
                    }
                }
            }

            m_gwRoutes[k][{gwA, gwB}] = bestAB;
            m_gwRoutes[k][{gwB, gwA}] = bestBA;
        }

        std::cout << "[GwRouting] slot=" << k
                  << " t=" << tau.GetSeconds() << "s";
        for (const auto& gw : m_gws)
            std::cout << " GW" << gw.gwId << "[" << gw.name << "]="
                      << GetGwVisibleSats(gw.gwId, k).size() << "sats";
        std::cout << "\n";
    }

    long gwMs = WALL_END_MS(gw);
    CHKPT("PrecomputeGwRoutes: done | wall=" << gwMs << "ms");
}

GwToGwRoute
IslRoutingManager::GetGwRoute(uint32_t srcGwId,
                               uint32_t dstGwId,
                               uint32_t slotIndex) const
{
    GwToGwRoute invalid;
    invalid.srcGwId = srcGwId;
    invalid.dstGwId = dstGwId;

    if (slotIndex >= m_gwRoutes.size())
        return invalid;

    auto it = m_gwRoutes[slotIndex].find({srcGwId, dstGwId});
    return (it != m_gwRoutes[slotIndex].end()) ? it->second : invalid;
}

void
IslRoutingManager::PrintGwRouteReport() const
{
    auto nameOf = [&](uint32_t id) -> std::string {
        auto it = m_gwIdToIdx.find(id);
        if (it == m_gwIdToIdx.end())
            return "GW" + std::to_string(id);
        const std::string& n = m_gws[it->second].name;
        return n.empty() ? "GW" + std::to_string(id) : n;
    };

    auto pathToStr = [](const std::vector<uint32_t>& path) -> std::string {
        std::ostringstream ss;
        for (size_t i = 0; i < path.size(); ++i)
        {
            if (i > 0) ss << "->";
            ss << (path[i] == UINT32_MAX ? "?" : std::to_string(path[i]));
        }
        return ss.str();
    };

    std::cout << "\n=== GW-to-GW Route Report (v6) ===\n";

    std::set<std::pair<uint32_t, uint32_t>> reported;
    for (const auto& p : m_gwPairs)
    {
        uint32_t lo = std::min(p.first, p.second);
        uint32_t hi = std::max(p.first, p.second);
        if (!reported.insert({lo, hi}).second)
            continue;

        uint32_t gwA = lo, gwB = hi;

        // 正向：gwA → gwB
        std::cout << "\n  [" << nameOf(gwA) << " → " << nameOf(gwB) << "]\n";
        std::cout << std::left
                  << std::setw(6)  << "slot"
                  << std::setw(10) << "time(s)"
                  << std::setw(8)  << "entry"
                  << std::setw(50) << "ISL_path"
                  << std::setw(8)  << "exit"
                  << std::setw(16) << "isl_cost(s)"
                  << "\n"
                  << std::string(98, '-') << "\n";

        {
            std::string prevPathStr;
            for (uint32_t k = 0; k < m_numTimeSlots; ++k)
            {
                GwToGwRoute r = GetGwRoute(gwA, gwB, k);
                double      t = k * m_timeSlotInterval;

                std::string pathStr = r.valid ? pathToStr(r.satPath) : "(no route)";
                bool        changed = (k > 0 && pathStr != prevPathStr);

                std::ostringstream costSs;
                if (r.valid && HasIslTransitPath(r.satPath))
                    costSs << std::fixed << std::setprecision(6) << r.islCost;
                else if (r.valid)
                    costSs << "N/A";
                else
                    costSs << "-";

                std::cout << std::left
                          << std::setw(6)  << k
                          << std::setw(10) << t
                          << std::setw(8)  << (r.valid ? std::to_string(r.entrySatId) : "-")
                          << std::setw(50) << pathStr
                          << std::setw(8)  << (r.valid ? std::to_string(r.exitSatId) : "-")
                          << std::setw(16) << costSs.str();

                if (changed)
                    std::cout << "  <-- ROUTE CHANGED";
                std::cout << "\n";
                prevPathStr = pathStr;
            }
        }

        // 反向：gwB → gwA
        std::cout << "\n  [" << nameOf(gwB) << " → " << nameOf(gwA) << "]\n";
        std::cout << std::left
                  << std::setw(6)  << "slot"
                  << std::setw(10) << "time(s)"
                  << std::setw(8)  << "entry"
                  << std::setw(50) << "ISL_path"
                  << std::setw(8)  << "exit"
                  << std::setw(16) << "isl_cost(s)"
                  << "\n"
                  << std::string(98, '-') << "\n";

        {
            std::string prevPathStr;
            for (uint32_t k = 0; k < m_numTimeSlots; ++k)
            {
                GwToGwRoute r = GetGwRoute(gwB, gwA, k);
                double      t = k * m_timeSlotInterval;

                std::string pathStr = r.valid ? pathToStr(r.satPath) : "(no route)";
                bool        changed = (k > 0 && pathStr != prevPathStr);

                std::ostringstream costSs;
                if (r.valid && HasIslTransitPath(r.satPath))
                    costSs << std::fixed << std::setprecision(6) << r.islCost;
                else if (r.valid)
                    costSs << "N/A";
                else
                    costSs << "-";

                std::cout << std::left
                          << std::setw(6)  << k
                          << std::setw(10) << t
                          << std::setw(8)  << (r.valid ? std::to_string(r.entrySatId) : "-")
                          << std::setw(50) << pathStr
                          << std::setw(8)  << (r.valid ? std::to_string(r.exitSatId) : "-")
                          << std::setw(16) << costSs.str();

                if (changed)
                    std::cout << "  <-- ROUTE CHANGED";
                std::cout << "\n";
                prevPathStr = pathStr;
            }
        }
    }

    std::cout << "\n===================================\n\n";
}

// ═══════════════════════════════════════════════════════════════════════════
// v7：GW-to-UT 路由
// ═══════════════════════════════════════════════════════════════════════════

// ── UT 設定 API ────────────────────────────────────────────────────────────

// AddUserTerminal: 新增一個 User Terminal 定義（id 必須唯一）
void
IslRoutingManager::AddUserTerminal(uint32_t           utId,
                                    double             latDeg,
                                    double             lonDeg,
                                    const std::string& name)
{
    NS_ASSERT_MSG(!m_utIdToIdx.count(utId),
                  "AddUserTerminal: utId=" << utId << " already exists");
    m_utIdToIdx[utId] = static_cast<uint32_t>(m_uts.size());
    m_uts.push_back({utId, latDeg, lonDeg, name});
}

// AddGwUtPair: 設定一組 (GW, UT) 路由對（單向：GW→UT）
void
IslRoutingManager::AddGwUtPair(uint32_t gwId, uint32_t utId)
{
    // 防呆：未透過 AddGateway / AddUserTerminal 註冊的 ID → 立即 fatal
    NS_ASSERT_MSG(m_gwIdToIdx.count(gwId),
                  "AddGwUtPair: gwId=" << gwId << " not registered — call AddGateway() first");
    NS_ASSERT_MSG(m_utIdToIdx.count(utId),
                  "AddGwUtPair: utId=" << utId << " not registered — call AddUserTerminal() first");

    m_gwUtPairs.insert({gwId, utId});
}

// GetUtVisibleSats: 查詢 utId 在 slotIndex 的可見衛星集合
const std::set<uint32_t>&
IslRoutingManager::GetUtVisibleSats(uint32_t utId, uint32_t slotIndex) const
{
    if (slotIndex >= m_utVisibility.size())
        return m_emptyUtSatSet;
    auto it = m_utVisibility[slotIndex].find(utId);
    return (it != m_utVisibility[slotIndex].end()) ? it->second : m_emptyUtSatSet;
}

// ── PrecomputeGwUtRoutes ──────────────────────────────────────────────────
//
// 預計算所有 time slot 的 GW-to-UT 最佳路由：
//   對每個 slot：
//     1. 計算 GW 可見衛星（若 PrecomputeGwRoutes 未呼叫則自行計算）
//     2. 計算每個 UT 的可見衛星集合（仰角 >= m_gwElevThreshDeg）
//     3. 對每個 contracted (gwId, utId) pair：
//        ‧ 枚舉 (entry ∈ vis(GW)) × (serving ∈ vis(UT))
//        ‧ 選出 ISL cost 最小的 (entry, serving)
//        ‧ TracePath(entry, serving) 重建完整衛星路徑
//
// 必須在 PrecomputeAllTables() 之後呼叫。
// 若已呼叫 PrecomputeGwRoutes()，共用 m_gwVisibility；
// 否則本函式自行計算 GW 可見性。
void
IslRoutingManager::PrecomputeGwUtRoutes()
{
    NS_ASSERT_MSG(!m_tables.empty(),
                  "PrecomputeGwUtRoutes: call PrecomputeAllTables() first");
    NS_ASSERT_MSG(!m_gws.empty(),
                  "PrecomputeGwUtRoutes: no gateways — call AddGateway() first");
    NS_ASSERT_MSG(!m_uts.empty(),
                  "PrecomputeGwUtRoutes: no user terminals — call AddUserTerminal() first");

    CHKPT("PrecomputeGwUtRoutes: start"
          << " | gws=" << m_gws.size()
          << " uts=" << m_uts.size()
          << " pairs=" << m_gwUtPairs.size()
          << " slots=" << m_numTimeSlots);

    WALL_START(gwut);

    m_utVisibility.resize(m_numTimeSlots);
    m_gwUtRoutes.resize(m_numTimeSlots);

    // 若 GW visibility 尚未計算（未呼叫 PrecomputeGwRoutes），先分配空間
    bool needGwVis = m_gwVisibility.empty();
    if (needGwVis)
        m_gwVisibility.resize(m_numTimeSlots);

    for (uint32_t k = 0; k < m_numTimeSlots; ++k)
    {
        Time                tau    = Seconds(k * m_timeSlotInterval);
        std::vector<Vector> satPos = GetPositionsAt(tau);

        // Step 1：若需要，補算 GW 可見衛星
        if (needGwVis)
        {
            for (const auto& gw : m_gws)
            {
                std::set<uint32_t>& vis = m_gwVisibility[k][gw.gwId];
                for (uint32_t s = 0; s < m_numSatellites; ++s)
                {
                    if (ComputeElevationDeg(gw.latDeg, gw.lonDeg, satPos[s]) >=
                        m_gwElevThreshDeg)
                        vis.insert(s);
                }
            }
        }

        // Step 2：計算每個 UT 的可見衛星
        for (const auto& ut : m_uts)
        {
            std::set<uint32_t>& vis = m_utVisibility[k][ut.utId];
            for (uint32_t s = 0; s < m_numSatellites; ++s)
            {
                if (ComputeElevationDeg(ut.latDeg, ut.lonDeg, satPos[s]) >=
                    m_gwElevThreshDeg)
                    vis.insert(s);
            }
        }

        // Step 3：對每個 (gwId, utId) pair 找最佳 entry + serving
        for (const auto& p : m_gwUtPairs)
        {
            uint32_t gwId = p.first;
            uint32_t utId = p.second;

            const auto& entrySats   = GetGwVisibleSats(gwId, k);
            const auto& servingSats = GetUtVisibleSats(utId, k);

            GwToUtRoute best;
            best.gwId = gwId;
            best.utId = utId;

            for (uint32_t entry : entrySats)
            {
                for (uint32_t serving : servingSats)
                {
                    // 同一顆衛星同時覆蓋 GW 和 UT：ISL cost = 0
                    double cost = (entry == serving)
                                      ? 0.0
                                      : GetRouteCost(entry, serving, k);

                    if (cost < best.islCost)
                    {
                        best.entrySatId   = entry;
                        best.servingSatId = serving;
                        best.islCost      = cost;
                        best.satPath      = (entry == serving)
                                                ? std::vector<uint32_t>{entry}
                                                : TracePath(entry, serving, k);
                        best.valid        = true;
                    }
                }
            }

            m_gwUtRoutes[k][{gwId, utId}] = best;
        }

        // Progress log
        std::cout << "[GwUtRouting] slot=" << k
                  << " t=" << tau.GetSeconds() << "s";
        for (const auto& ut : m_uts)
            std::cout << " UT" << ut.utId << "[" << ut.name << "]="
                      << GetUtVisibleSats(ut.utId, k).size() << "sats";
        std::cout << "\n";
    }

    long gwutMs = WALL_END_MS(gwut);
    CHKPT("PrecomputeGwUtRoutes: done | wall=" << gwutMs << "ms");
}

// ── RefreshGwRoutesForSlot ────────────────────────────────────────────────
//
// 設計目的：
//   RecomputeAffectedRoutes() 更新 m_tables[slotIndex] 後，
//   重新以最新 GetRouteCost() 計算 m_gwRoutes / m_gwUtRoutes，
//   確保路由報表與實際 forwarding path 不脫節。
//
// 前提：m_gwVisibility / m_utVisibility 在 precompute 階段計算完畢，
//   執行期不因 load recompute 改變（衛星位置不動），故不重算。
// no-op 條件：
//   - m_gwRoutes 尚未初始化（PrecomputeGwRoutes 未呼叫）
//   - slotIndex 超出範圍
void
IslRoutingManager::RefreshGwRoutesForSlot(uint32_t slotIndex)
{
    // ── GW-to-GW 路由刷新 ─────────────────────────────────────────────────
    if (slotIndex < m_gwRoutes.size())
    {
        for (const auto& p : m_gwPairs)
        {
            uint32_t gwA = p.first;
            uint32_t gwB = p.second;

            // 只處理 gwA < gwB，避免重複計算（gwPairs 含正反向）
            if (gwA >= gwB)
                continue;

            const auto& srcSatsAB = GetGwVisibleSats(gwA, slotIndex);
            const auto& dstSatsAB = GetGwVisibleSats(gwB, slotIndex);

            // A → B
            GwToGwRoute bestAB;
            bestAB.srcGwId = gwA;
            bestAB.dstGwId = gwB;

            for (uint32_t entry : srcSatsAB)
            {
                for (uint32_t exit : dstSatsAB)
                {
                    double cost = (entry == exit)
                                      ? 0.0
                                      : GetRouteCost(entry, exit, slotIndex);
                    if (cost < bestAB.islCost)
                    {
                        bestAB.entrySatId = entry;
                        bestAB.exitSatId  = exit;
                        bestAB.islCost    = cost;
                        bestAB.satPath    = (entry == exit)
                                                ? std::vector<uint32_t>{entry}
                                                : TracePath(entry, exit, slotIndex);
                        bestAB.valid      = true;
                    }
                }
            }

            // B → A
            GwToGwRoute bestBA;
            bestBA.srcGwId = gwB;
            bestBA.dstGwId = gwA;

            for (uint32_t entry : dstSatsAB)
            {
                for (uint32_t exit : srcSatsAB)
                {
                    double cost = (entry == exit)
                                      ? 0.0
                                      : GetRouteCost(entry, exit, slotIndex);
                    if (cost < bestBA.islCost)
                    {
                        bestBA.entrySatId = entry;
                        bestBA.exitSatId  = exit;
                        bestBA.islCost    = cost;
                        bestBA.satPath    = (entry == exit)
                                                ? std::vector<uint32_t>{entry}
                                                : TracePath(entry, exit, slotIndex);
                        bestBA.valid      = true;
                    }
                }
            }

            m_gwRoutes[slotIndex][{gwA, gwB}] = bestAB;
            m_gwRoutes[slotIndex][{gwB, gwA}] = bestBA;
        }
    }

    // ── GW-to-UT 路由刷新 ─────────────────────────────────────────────────
    if (slotIndex < m_gwUtRoutes.size())
    {
        for (const auto& p : m_gwUtPairs)
        {
            uint32_t gwId = p.first;
            uint32_t utId = p.second;

            const auto& entrySats   = GetGwVisibleSats(gwId, slotIndex);
            const auto& servingSats = GetUtVisibleSats(utId, slotIndex);

            GwToUtRoute best;
            best.gwId = gwId;
            best.utId = utId;

            for (uint32_t entry : entrySats)
            {
                for (uint32_t serving : servingSats)
                {
                    double cost = (entry == serving)
                                      ? 0.0
                                      : GetRouteCost(entry, serving, slotIndex);
                    if (cost < best.islCost)
                    {
                        best.entrySatId   = entry;
                        best.servingSatId = serving;
                        best.islCost      = cost;
                        best.satPath      = (entry == serving)
                                                ? std::vector<uint32_t>{entry}
                                                : TracePath(entry, serving, slotIndex);
                        best.valid        = true;
                    }
                }
            }

            m_gwUtRoutes[slotIndex][{gwId, utId}] = best;
        }
    }
}

// ── GetGwUtRoute ──────────────────────────────────────────────────────────

GwToUtRoute
IslRoutingManager::GetGwUtRoute(uint32_t gwId,
                                 uint32_t utId,
                                 uint32_t slotIndex) const
{
    GwToUtRoute invalid;
    invalid.gwId = gwId;
    invalid.utId = utId;

    if (slotIndex >= m_gwUtRoutes.size())
        return invalid;

    auto it = m_gwUtRoutes[slotIndex].find({gwId, utId});
    return (it != m_gwUtRoutes[slotIndex].end()) ? it->second : invalid;
}

// ── PrintGwUtRouteReport ──────────────────────────────────────────────────
//
// 輸出所有 contracted (GW, UT) pair 的跨 slot 路由報告。
// 欄位：slot | time(s) | entry_sat | ISL_path | serving_sat | isl_cost(s)
// 路徑與前一 slot 不同時標記 <-- ROUTE CHANGED。
void
IslRoutingManager::PrintGwUtRouteReport() const
{
    auto gwNameOf = [&](uint32_t id) -> std::string {
        auto it = m_gwIdToIdx.find(id);
        if (it == m_gwIdToIdx.end())
            return "GW" + std::to_string(id);
        const std::string& n = m_gws[it->second].name;
        return n.empty() ? "GW" + std::to_string(id) : n;
    };

    auto utNameOf = [&](uint32_t id) -> std::string {
        auto it = m_utIdToIdx.find(id);
        if (it == m_utIdToIdx.end())
            return "UT" + std::to_string(id);
        const std::string& n = m_uts[it->second].name;
        return n.empty() ? "UT" + std::to_string(id) : n;
    };

    auto pathToStr = [](const std::vector<uint32_t>& path) -> std::string {
        std::ostringstream ss;
        for (size_t i = 0; i < path.size(); ++i)
        {
            if (i > 0) ss << "->";
            ss << (path[i] == UINT32_MAX ? "?" : std::to_string(path[i]));
        }
        return ss.str();
    };

    std::cout << "\n=== GW-to-UT Route Report (v7) ===\n";

    for (const auto& p : m_gwUtPairs)
    {
        uint32_t gwId = p.first;
        uint32_t utId = p.second;

        std::cout << "\n  [" << gwNameOf(gwId) << " → " << utNameOf(utId) << "]\n";
        std::cout << std::left
                  << std::setw(6)  << "slot"
                  << std::setw(10) << "time(s)"
                  << std::setw(8)  << "entry"
                  << std::setw(50) << "ISL_path"
                  << std::setw(10) << "serving"
                  << std::setw(16) << "isl_cost(s)"
                  << "\n"
                  << std::string(100, '-') << "\n";

        std::string prevPathStr;
        for (uint32_t k = 0; k < m_numTimeSlots; ++k)
        {
            GwToUtRoute r = GetGwUtRoute(gwId, utId, k);
            double      t = k * m_timeSlotInterval;

            std::string pathStr = r.valid ? pathToStr(r.satPath) : "(no route)";
            bool        changed = (k > 0 && pathStr != prevPathStr);

            std::ostringstream costSs;
            if (r.valid && HasIslTransitPath(r.satPath))
                costSs << std::fixed << std::setprecision(6) << r.islCost;
            else if (r.valid)
                costSs << "N/A";
            else
                costSs << "-";

            std::cout << std::left
                      << std::setw(6)  << k
                      << std::setw(10) << t
                      << std::setw(8)  << (r.valid ? std::to_string(r.entrySatId) : "-")
                      << std::setw(50) << pathStr
                      << std::setw(10) << (r.valid ? std::to_string(r.servingSatId) : "-")
                      << std::setw(16) << costSs.str();

            if (changed)
                std::cout << "  <-- ROUTE CHANGED";
            std::cout << "\n";
            prevPathStr = pathStr;
        }
    }

    std::cout << "\n===================================\n\n";
}

// ── HolDelayObserver Stub 實作 ────────────────────────────────────────────
//
// 以下三個方法均為 no-op stub，保留完整的實作路線圖於註解中。
// 待使用者確認允許修改 SNS3 satellite-queue.h/.cc 後，再依路線圖展開實作。

void
HolDelayObserver::Enable()
{
    // ── 解鎖後的實作步驟（確認許可後展開）────────────────────────────────
    //
    // Step 1. 在 satellite-queue.h 定義 EnqueueTimestampTag：
    //   class EnqueueTimestampTag : public Tag {
    //     Time m_enqueueTime;
    //     void Serialize / Deserialize / GetSerializedSize ...
    //   };
    //
    // Step 2. 在 SatQueue::Enqueue() 加入：
    //   pkt->AddPacketTag(EnqueueTimestampTag(Simulator::Now()));
    //
    // Step 3. 在 SatQueue::Dequeue() 加入：
    //   EnqueueTimestampTag tag;
    //   if (pkt->RemovePacketTag(tag))
    //       m_holDelayTrace(( Simulator::Now() - tag.m_enqueueTime ).GetMilliSeconds());
    //
    // Step 4. 在 SatQueue::GetTypeId() 新增 TraceSource：
    //   .AddTraceSource("HolDelayTrace", "HOL delay in ms",
    //                   MakeTraceSourceAccessor(&SatQueue::m_holDelayTrace),
    //                   "ns3::TracedValueCallback::Double")
    //
    // Step 5. 在 test-iridium.cc CreateSatScenario() 後連接：
    //   Config::ConnectWithoutContext(
    //       "/NodeList/*/DeviceList/*/SatLlc/SatQueue/HolDelayTrace",
    //       MakeCallback(&HolDelayTraceCallback));

    std::cout << "[HOL] HolDelayObserver::Enable() — Stub only, not yet implemented.\n"
              << "      Requires SNS3 modification (satellite-queue.h/.cc).\n"
              << "      See isl-graph.h HolDelayObserver for full implementation roadmap.\n";
    m_enabled = false;
}

double
HolDelayObserver::GetHolDelayMs(uint32_t /* utNodeId */) const
{
    // Stub：回傳 -1.0 代表「未實作」
    // 實作後：從 EMA 統計表依 utNodeId 查詢最近一次 HOL delay
    return -1.0;
}

void
HolDelayObserver::PrintReport() const
{
    if (!m_enabled)
    {
        std::cout << "[HOL] HolDelayObserver is not yet implemented.\n"
                  << "      Call Enable() after confirming SNS3 modification permission.\n"
                  << "      See isl-graph.h HolDelayObserver for implementation roadmap.\n";
        return;
    }
    // 實作後：輸出所有 UT 的 HOL delay（min / max / avg）
}

} // namespace ns3
