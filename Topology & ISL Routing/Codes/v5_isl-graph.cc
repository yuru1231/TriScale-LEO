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

        if (k < m_numTimeSlots - 1)
        {
            Time tauNext = Seconds(m_timeSlotInterval * (k + 1));
            std::vector<Vector> posNext = GetPositionsAt(tauNext);
            ISLGraph graphNext = BuildISLGraph(posNext);
            routes = ApplyTiebreaker(routes, graphNext);
            graphCurr = std::move(graphNext);
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

    bool didRecompute = false;
    long recomputeMs = 0;
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
            recomputeMs = WALL_END_MS(recompute);
            didRecompute = true;

            CHKPT("RecomputeAffectedRoutes: slot=" << slotIndex
                                                   << " recomputed="
                                                   << recomputedSources
                                                   << "/" << m_numSatellites
                                                   << " wall="
                                                   << recomputeMs << "ms");
        }
    }

    const RoutingTable& table = m_tables[slotIndex];

    NS_ASSERT_MSG(m_arbiters.size() == m_numSatellites,
                  "m_arbiters size mismatch");
    NS_ASSERT_MSG(m_orbDevs.size() == m_numSatellites,
                  "m_orbDevs size mismatch");

    for (uint32_t satId = 0; satId < m_numSatellites; satId++)
    {
        NS_ASSERT_MSG(m_arbiters[satId], "Null arbiter at satId=" << satId);
        NS_ASSERT_MSG(m_orbDevs[satId], "Null orbiter net device at satId=" << satId);

        m_arbiters[satId]->ClearNextHopEntries();

        for (const auto& entry : table[satId])
        {
            m_arbiters[satId]->AddNextHopEntry(entry.destSatId,
                                               entry.islIfIndexOnA);
        }

        m_orbDevs[satId]->SetArbiter(m_arbiters[satId]);
    }

    RebuildIslSources(slotIndex);

    long applyMs = WALL_END_MS(apply);

    SlotStats s;
    s.slotIndex = slotIndex;
    s.simTimeSec = Simulator::Now().GetSeconds();
    s.applyWallMs = applyMs;
    s.recomputeWallMs = recomputeMs;
    s.recomputedSources = recomputedSources;
    s.significantChange = didRecompute;
    m_stats.push_back(s);

    CHKPT("ApplyRoutingTable: slot=" << slotIndex
                                     << " t=" << s.simTimeSec << "s"
                                     << " | apply=" << applyMs << "ms"
                                     << " recompute=" << recomputeMs << "ms"
                                     << " recomputedSrc="
                                     << recomputedSources);
}

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
    const double C = 3e8;
    const double maxDist = m_islMaxDistanceKm * 1e3;

    ISLGraph graph(m_numSatellites);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double dx = pos[a].x - pos[b].x;
        double dy = pos[a].y - pos[b].y;
        double dz = pos[a].z - pos[b].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > maxDist)
        {
            continue;
        }

        // Skip ISLs that are explicitly blocked (used by RunAvoidanceTest)
        if (m_blockedEdges.count({a, b}))
        {
            continue;
        }

        double propCost = dist / C;
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
    const double C = 3e8;
    const double maxDist = m_islMaxDistanceKm * 1e3;
    const uint32_t N = m_numSatellites;

    ISLGraph graph(N);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double dx = pos[a].x - pos[b].x;
        double dy = pos[a].y - pos[b].y;
        double dz = pos[a].z - pos[b].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > maxDist)
        {
            continue;
        }

        // Skip ISLs that are explicitly blocked (used by RunAvoidanceTest)
        if (m_blockedEdges.count({a, b}))
        {
            continue;
        }

        double propCost = dist / C;
        uint32_t ifIdxOnA = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifIdxOnB = m_perSatISLOrder[b].at(edgeIdx);

        double loadAb = m_loadCosts[a * N + b];
        double loadBa = m_loadCosts[b * N + a];

        graph[a].push_back({b, propCost + loadAb, ifIdxOnA, ifIdxOnB});
        graph[b].push_back({a, propCost + loadBa, ifIdxOnB, ifIdxOnA});
    }

    return graph;
}

std::vector<RouteEntry>
IslRoutingManager::ComputeRoutesForSrc(uint32_t src, const ISLGraph& graph) const
{
    const uint32_t N = m_numSatellites;
    const double INF = std::numeric_limits<double>::infinity();

    std::vector<double> dist(N, INF);
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
        {
            continue;
        }

        for (const auto& e : graph[u])
        {
            double nd = dist[u] + e.propagation_cost;
            if (nd < dist[e.nodeB])
            {
                dist[e.nodeB] = nd;

                if (u == src)
                {
                    firstHopNode[e.nodeB] = e.nodeB;
                    firstHopIf[e.nodeB] = e.islIfIndexOnA;
                }
                else
                {
                    firstHopNode[e.nodeB] = firstHopNode[u];
                    firstHopIf[e.nodeB] = firstHopIf[u];
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
        {
            continue;
        }

        entries.push_back(
            {dest, firstHopNode[dest], firstHopIf[dest], dist[dest]});
    }

    return entries;
}

RoutingTable
IslRoutingManager::ComputeBaseRoutes(const ISLGraph& graph) const
{
    RoutingTable result(m_numSatellites);

    for (uint32_t src = 0; src < m_numSatellites; src++)
    {
        result[src] = ComputeRoutesForSrc(src, graph);
    }

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
    {
        for (const auto& e : graphNext[u])
        {
            nextEligible.insert({u, e.nodeB});
        }
    }

    RoutingTable result = routes;

    for (uint32_t src = 0; src < routes.size(); src++)
    {
        std::map<uint32_t, std::vector<size_t>> destToIndices;

        for (size_t i = 0; i < routes[src].size(); i++)
        {
            destToIndices[routes[src][i].destSatId].push_back(i);
        }

        for (auto& [dest, indices] : destToIndices)
        {
            (void)dest;

            if (indices.size() <= 1)
            {
                continue;
            }

            double minCost = routes[src][indices[0]].cost;

            std::vector<size_t> tied;
            for (size_t idx : indices)
            {
                if (std::abs(routes[src][idx].cost - minCost) < 1e-9)
                {
                    tied.push_back(idx);
                }
            }

            if (tied.size() <= 1)
            {
                continue;
            }

            for (size_t idx : tied)
            {
                uint32_t nh = routes[src][idx].nextHopSatId;
                if (nextEligible.count({src, nh}))
                {
                    for (size_t other : tied)
                    {
                        if (other != idx)
                        {
                            result[src][other].cost =
                                std::numeric_limits<double>::infinity();
                        }
                    }
                    break;
                }
            }
        }

        auto& vec = result[src];
        vec.erase(std::remove_if(vec.begin(),
                                 vec.end(),
                                 [](const RouteEntry& e)
                                 {
                                     return e.cost ==
                                            std::numeric_limits<double>::infinity();
                                 }),
                  vec.end());
    }

    return result;
}

double
IslRoutingManager::GetLinkQueueDelay(uint32_t satId, uint32_t ifIdx) const
{
    if (satId >= m_orbDevs.size() || !m_orbDevs[satId])
    {
        return 0.0;
    }

    auto islDevs = m_orbDevs[satId]->GetIslsNetDevices();

    if (ifIdx >= islDevs.size() || !islDevs[ifIdx])
    {
        return 0.0;
    }

    Ptr<DropTailQueue<Packet>> q = islDevs[ifIdx]->GetQueue();

    if (!q)
    {
        return 0.0;
    }

    uint32_t nPackets = q->GetNPackets();
    double bits = static_cast<double>(nPackets) * 1500.0 * 8.0;

    if (m_islLinkRateBps <= 0.0)
    {
        return 0.0;
    }

    return bits / m_islLinkRateBps;
}

void
IslRoutingManager::UpdateLoadCosts()
{
    m_prevLoadCosts = m_loadCosts;
    const uint32_t N = m_numSatellites;

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;
        uint32_t ifA = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifB = m_perSatISLOrder[b].at(edgeIdx);

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
        {
            return true;
        }
    }

    return false;
}

uint32_t
IslRoutingManager::RecomputeAffectedRoutes(uint32_t slotIndex)
{
    Time now = Simulator::Now();
    std::vector<Vector> pos = GetPositionsAt(now);
    ISLGraph graph = BuildISLGraphWithLoad(pos);

    const uint32_t N = m_numSatellites;
    std::vector<bool> affected(N, false);

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
            {
                affected[src] = true;
            }
        }
    }

    uint32_t recomputedCount = 0;
    RoutingTable& table = m_tables[slotIndex];

    for (uint32_t src = 0; src < N; src++)
    {
        if (!affected[src])
        {
            continue;
        }

        table[src] = ComputeRoutesForSrc(src, graph);
        ++recomputedCount;
    }

    m_lastRecomputeTime = now;
    RebuildIslSources(slotIndex);

    return recomputedCount;
}

void
IslRoutingManager::RebuildIslSources(uint32_t slotIndex)
{
    m_islSources.assign(m_islDefs.size(), std::unordered_set<uint32_t>());

    const RoutingTable& table = m_tables[slotIndex];

    for (uint32_t src = 0; src < m_numSatellites; src++)
    {
        std::unordered_set<uint32_t> seenNh;

        for (const auto& entry : table[src])
        {
            if (!seenNh.insert(entry.nextHopSatId).second)
            {
                continue;
            }

            auto it = m_edgeOfPair.find({src, entry.nextHopSatId});
            if (it != m_edgeOfPair.end())
            {
                m_islSources[it->second].insert(src);
            }
        }
    }
}

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
    std::getline(file, line); // skip header

    std::vector<uint32_t> counter(m_numSatellites, 0);

    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::istringstream iss(line);
        uint32_t a = 0;
        uint32_t b = 0;
        iss >> a >> b;

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

void
IslRoutingManager::PrintStats() const
{
    std::cout << "\n=== IslRoutingManager Stats ===" << std::endl;
    std::cout << std::left
              << std::setw(6) << "slot"
              << std::setw(10) << "simTime"
              << std::setw(12) << "apply(ms)"
              << std::setw(16) << "recompute(ms)"
              << std::setw(16) << "recompSrc"
              << std::setw(12) << "changed"
              << std::endl;

    for (const auto& s : m_stats)
    {
        std::cout << std::left
                  << std::setw(6) << s.slotIndex
                  << std::setw(10) << s.simTimeSec
                  << std::setw(12) << s.applyWallMs
                  << std::setw(16) << s.recomputeWallMs
                  << std::setw(16) << s.recomputedSources
                  << std::setw(12) << (s.significantChange ? "YES" : "NO")
                  << std::endl;
    }

    std::cout << "==============================\n" << std::endl;
}

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

// TracePath: reconstruct the full hop sequence from src to dst at a given slot
// by following each satellite's next-hop entry for the destination.
// This uses the already-computed routing table; no Dijkstra re-run needed.
std::vector<uint32_t>
IslRoutingManager::TracePath(uint32_t src,
                              uint32_t dst,
                              uint32_t slotIndex) const
{
    std::vector<uint32_t> path;

    if (slotIndex >= m_tables.size())
    {
        return path;
    }

    path.push_back(src);
    uint32_t curr = src;

    // Guard: a valid path can have at most (numSatellites - 1) hops.
    for (uint32_t hop = 0; hop < m_numSatellites; ++hop)
    {
        if (curr == dst)
        {
            break;
        }

        // Look up the next-hop entry for 'dst' in curr's routing table.
        bool found = false;
        for (const auto& e : m_tables[slotIndex][curr])
        {
            if (e.destSatId == dst)
            {
                path.push_back(e.nextHopSatId);
                curr = e.nextHopSatId;
                found = true;
                break;
            }
        }

        if (!found)
        {
            // Destination unreachable from curr; mark and stop.
            path.push_back(UINT32_MAX);
            break;
        }
    }

    return path;
}

// PrintRouteReport: for each (src, dst) pair, print one row per time slot.
// Format: time(s) | src | dst | full_path | route_cost | applied_slot
// Rows where the full path changed from the previous slot are tagged <CHANGED>.
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
            double timeSec   = k * m_timeSlotInterval;
            double cost      = GetRouteCost(src, dst, k);
            std::vector<uint32_t> path = TracePath(src, dst, k);

            // Build human-readable path string
            std::ostringstream pathSs;
            for (size_t i = 0; i < path.size(); ++i)
            {
                if (i > 0)
                {
                    pathSs << "->";
                }
                if (path[i] == UINT32_MAX)
                {
                    pathSs << "?";
                }
                else
                {
                    pathSs << path[i];
                }
            }
            std::string pathStr = pathSs.str();

            bool changed = (k > 0 && pathStr != prevPathStr);

            std::cout << std::left
                      << std::setw(10) << timeSec
                      << std::setw(6)  << src
                      << std::setw(6)  << dst
                      << std::setw(44) << pathStr
                      << std::setw(14) << std::fixed << std::setprecision(6)
                      << cost
                      << std::setw(6)  << k;

            if (changed)
            {
                std::cout << "  <-- PATH CHANGED";
            }

            std::cout << "\n";
            prevPathStr = pathStr;
        }

        std::cout << "\n";
    }

    std::cout << "==============================================\n";
}

// BlockISL: mark an ISL as unavailable.
// Both directions are inserted so BuildISLGraph only needs one lookup.
void
IslRoutingManager::BlockISL(uint32_t nodeA, uint32_t nodeB)
{
    m_blockedEdges.insert({nodeA, nodeB});
    m_blockedEdges.insert({nodeB, nodeA});
}

// UnblockISL: remove the block on an ISL.
void
IslRoutingManager::UnblockISL(uint32_t nodeA, uint32_t nodeB)
{
    m_blockedEdges.erase({nodeA, nodeB});
    m_blockedEdges.erase({nodeB, nodeA});
}

// RunAvoidanceTest
// 1. Trace the path from testSrc to testDst at slotIndex (baseline).
// 2. Block the first ISL on that path.
// 3. Recompute the routing table for slotIndex without the blocked ISL.
// 4. Trace the new path and verify:
//    (a) the blocked ISL does not appear anywhere in the new path, and
//    (b) the routing table at testSrc has no next-hop entry pointing to
//        the blocked neighbour for destination testDst.
// 5. Restore the original routing table and unblock the ISL.
void
IslRoutingManager::RunAvoidanceTest(uint32_t testSrc,
                                     uint32_t testDst,
                                     uint32_t slotIndex)
{
    std::cout << "\n=== ISL Avoidance Test =========================\n";

    // ── Step 1: baseline path ─────────────────────────────────────────────
    std::vector<uint32_t> baselinePath = TracePath(testSrc, testDst, slotIndex);
    double baselineCost = GetRouteCost(testSrc, testDst, slotIndex);

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
    std::cout << "  cost=" << std::fixed << std::setprecision(6)
              << baselineCost << "\n";

    if (baselinePath.size() < 2 || baselinePath.back() == UINT32_MAX)
    {
        std::cout << "  [SKIP] No reachable path — cannot perform avoidance test.\n";
        std::cout << "================================================\n";
        return;
    }

    // ── Step 2: pick the first ISL on the baseline path to block ──────────
    uint32_t blockA = baselinePath[0];
    uint32_t blockB = baselinePath[1];

    std::cout << "Block ISL : " << blockA << " <-> " << blockB << "\n";

    // ── Step 3: save original table, block ISL, recompute slot offline ────
    RoutingTable savedTable = m_tables[slotIndex];

    BlockISL(blockA, blockB);

    Time tau = Seconds(m_timeSlotInterval * slotIndex);
    std::vector<Vector> pos = GetPositionsAt(tau);
    ISLGraph graphBlocked   = BuildISLGraph(pos);        // blocked edge skipped here
    m_tables[slotIndex]     = ComputeBaseRoutes(graphBlocked);

    // ── Step 4a: new path ─────────────────────────────────────────────────
    std::vector<uint32_t> newPath = TracePath(testSrc, testDst, slotIndex);
    double newCost = GetRouteCost(testSrc, testDst, slotIndex);

    std::cout << "Post-block: ";
    printPath(newPath);
    if (newPath.empty() || newPath.back() == UINT32_MAX)
    {
        std::cout << "  (unreachable)";
    }
    else
    {
        std::cout << "  cost=" << std::fixed << std::setprecision(6) << newCost;
    }
    std::cout << "\n";

    // ── Step 4b: verify (a) — blocked ISL absent from new path ───────────
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

    // ── Step 4c: verify (b) — no stale next-hop in routing table ─────────
    // Specifically: m_tables[slotIndex][testSrc] must not point to blockB
    // as the next hop for testDst (when testSrc == blockA).
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

    // ── Step 5: restore ───────────────────────────────────────────────────
    UnblockISL(blockA, blockB);
    m_tables[slotIndex] = std::move(savedTable);

    std::cout << "Routing table restored.\n";
    std::cout << "================================================\n";
}

} // namespace ns3
