#include "isl-graph.h"

#include "ns3/double.h"
#include "ns3/satellite-isl-arbiter-unicast.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>

namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED(IslRoutingManager);

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
                          "EMA smoothing factor for load costs (0 < alpha <= 1)",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&IslRoutingManager::m_emaAlpha),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("ChangeThreshold",
                          "Fractional load-cost change that triggers partial recompute",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&IslRoutingManager::m_changeThreshold),
                          MakeDoubleChecker<double>())
            .AddAttribute("CooldownSeconds",
                          "Minimum seconds between partial recomputes",
                          DoubleValue(30.0),
                          MakeDoubleAccessor(&IslRoutingManager::m_cooldownSeconds),
                          MakeDoubleChecker<double>())
            .AddAttribute("IslLinkRateBps",
                          "ISL link rate in bps (for queue delay estimation)",
                          DoubleValue(1.0e9),
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
      m_islLinkRateBps(1.0e9),
      m_lastRecomputeTime(Seconds(0.0))
{
}

// ─── Lifecycle ─────────────────────────────────────────────────────────────────

void
IslRoutingManager::Initialize(const std::string& islsFilePath)
{
    m_islsFilePath = islsFilePath;
    LoadISLDefs(m_islsFilePath);
    InitOrbiterDevices();

    // Initialise flat load-cost arrays to zero
    const uint32_t n2 = m_numSatellites * m_numSatellites;
    m_loadCosts.assign(n2, 0.0);
    m_prevLoadCosts.assign(n2, 0.0);
}

void
IslRoutingManager::PrecomputeAllTables()
{
    m_tables.resize(m_numTimeSlots);
    std::cout << "PrecomputeAllTables: start" << std::endl;

    // FIX 1 (rolling cache): build graphNext once and reuse as graphCurr next
    // iteration — eliminates duplicate BuildISLGraph calls.
    // Total BuildISLGraph calls: m_numTimeSlots  (was 2*m_numTimeSlots - 1)
    std::vector<Vector> posCurr = GetPositionsAt(Seconds(0.0));
    ISLGraph            graphCurr = BuildISLGraph(posCurr);

    for (uint32_t k = 0; k < m_numTimeSlots; k++)
    {
        RoutingTable routes = ComputeBaseRoutes(graphCurr);

        if (k < m_numTimeSlots - 1)
        {
            Time                tau_next  = Seconds(m_timeSlotInterval * (k + 1));
            std::vector<Vector> posNext   = GetPositionsAt(tau_next);
            ISLGraph            graphNext = BuildISLGraph(posNext);

            routes    = ApplyTiebreaker(routes, graphNext);
            graphCurr = std::move(graphNext);   // roll forward; no extra SGP4 call
        }

        m_tables[k] = std::move(routes);
        std::cout << "  t=" << m_timeSlotInterval * k
                  << "s: done, SAT0 routes=" << m_tables[k][0].size()
                  << std::endl;
    }

    std::cout << "PrecomputeAllTables: complete" << std::endl;
}

void
IslRoutingManager::ScheduleRoutingUpdates()
{
    for (uint32_t k = 0; k < m_numTimeSlots; k++)
    {
        Time t = Seconds(m_timeSlotInterval * k);
        Simulator::Schedule(t, &IslRoutingManager::ApplyRoutingTable, this, k);
    }
    std::cout << "ScheduleRoutingUpdates: "
              << m_numTimeSlots << " events scheduled" << std::endl;
}

void
IslRoutingManager::ApplyRoutingTable(uint32_t slotIndex)
{
    NS_ASSERT_MSG(slotIndex < m_tables.size(),
                  "slotIndex out of range: " << slotIndex);

    // ── Runtime load management (skip slot 0 — no history yet) ────────────────
    if (slotIndex > 0)
    {
        UpdateLoadCosts();
        if (HasSignificantChange())
            RecomputeAffectedRoutes(slotIndex);
    }

    // ── Install routing table into each satellite's arbiter ───────────────────
    const RoutingTable& table = m_tables[slotIndex];

    for (uint32_t satId = 0; satId < m_numSatellites; satId++)
    {
        Ptr<SatIslArbiterUnicast> arbiter =
            CreateObject<SatIslArbiterUnicast>(m_orbNodes[satId]);
        for (const auto& entry : table[satId])
            arbiter->AddNextHopEntry(entry.destSatId, entry.islIfIndexOnA);
        m_orbDevs[satId]->SetArbiter(arbiter);
    }

    std::cout << "ApplyRoutingTable: slot=" << slotIndex
              << " t=" << Simulator::Now().GetSeconds() << "s done" << std::endl;
}

// ─── Graph construction ─────────────────────────────────────────────────────────

std::vector<Vector>
IslRoutingManager::GetPositionsAt(Time tau) const
{
    // FIX 2 (position cache): call SGP4 exactly once per satellite per time
    // instead of once per edge (which repeats each satellite up to degree times).
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
    const double C        = 3e8;
    const double MAX_DIST = m_islMaxDistanceKm * 1e3;

    ISLGraph graph(m_numSatellites);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double dx   = pos[a].x - pos[b].x;
        double dy   = pos[a].y - pos[b].y;
        double dz   = pos[a].z - pos[b].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > MAX_DIST)
            continue;

        double   prop_cost = dist / C;
        uint32_t ifIdxOnA  = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifIdxOnB  = m_perSatISLOrder[b].at(edgeIdx);

        graph[a].push_back({b, prop_cost, ifIdxOnA, ifIdxOnB});
        graph[b].push_back({a, prop_cost, ifIdxOnB, ifIdxOnA});
    }

    return graph;
}

ISLGraph
IslRoutingManager::BuildISLGraphWithLoad(const std::vector<Vector>& pos) const
{
    // Same as BuildISLGraph but adds per-direction load cost to edge weight.
    const double C        = 3e8;
    const double MAX_DIST = m_islMaxDistanceKm * 1e3;
    const uint32_t N      = m_numSatellites;

    ISLGraph graph(N);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        double dx   = pos[a].x - pos[b].x;
        double dy   = pos[a].y - pos[b].y;
        double dz   = pos[a].z - pos[b].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > MAX_DIST)
            continue;

        double   prop_cost = dist / C;
        uint32_t ifIdxOnA  = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifIdxOnB  = m_perSatISLOrder[b].at(edgeIdx);

        double load_ab = m_loadCosts[a * N + b];
        double load_ba = m_loadCosts[b * N + a];

        graph[a].push_back({b, prop_cost + load_ab, ifIdxOnA, ifIdxOnB});
        graph[b].push_back({a, prop_cost + load_ba, ifIdxOnB, ifIdxOnA});
    }

    return graph;
}

// ─── Routing computation ────────────────────────────────────────────────────────

std::vector<RouteEntry>
IslRoutingManager::ComputeRoutesForSrc(uint32_t        src,
                                       const ISLGraph& graph) const
{
    // FIX 3 (first-hop tracking): record first-hop node and interface during
    // Dijkstra relaxation — eliminates the O(N) back-trace per destination.
    const uint32_t N   = m_numSatellites;
    const double   INF = std::numeric_limits<double>::infinity();

    std::vector<double>   dist(N, INF);
    std::vector<uint32_t> firstHopNode(N, UINT32_MAX);
    std::vector<uint32_t> firstHopIf  (N, UINT32_MAX);

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
                // Propagate first-hop info: if we just left src, the neighbour
                // itself is the first hop; otherwise inherit from u.
                if (u == src)
                {
                    firstHopNode[e.nodeB] = e.nodeB;
                    firstHopIf  [e.nodeB] = e.islIfIndexOnA;
                }
                else
                {
                    firstHopNode[e.nodeB] = firstHopNode[u];
                    firstHopIf  [e.nodeB] = firstHopIf  [u];
                }
                pq.push({nd, e.nodeB});
            }
        }
    }

    std::vector<RouteEntry> entries;
    entries.reserve(N - 1);
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
                                    const ISLGraph&     graphNext) const
{
    // FIX 4: use unordered_set for O(1) eligible-link lookup instead of O(log N).
    struct PairHash
    {
        size_t operator()(const std::pair<uint32_t, uint32_t>& p) const
        {
            return std::hash<uint64_t>{}((uint64_t)p.first << 32 | p.second);
        }
    };
    std::unordered_set<std::pair<uint32_t, uint32_t>, PairHash> nextEligible;
    nextEligible.reserve(graphNext.size() * 4);
    for (uint32_t u = 0; u < graphNext.size(); u++)
        for (const auto& e : graphNext[u])
            nextEligible.insert({u, e.nodeB});

    RoutingTable result = routes;   // copy once; entries modified in-place below

    for (uint32_t src = 0; src < routes.size(); src++)
    {
        // Group indices by destination
        std::map<uint32_t, std::vector<size_t>> destToIndices;
        for (size_t i = 0; i < routes[src].size(); i++)
            destToIndices[routes[src][i].destSatId].push_back(i);

        for (auto& [dest, indices] : destToIndices)
        {
            if (indices.size() <= 1)
                continue;

            double minCost = routes[src][indices[0]].cost;

            std::vector<size_t> tied;
            for (size_t idx : indices)
                if (std::abs(routes[src][idx].cost - minCost) < 1e-9)
                    tied.push_back(idx);

            if (tied.size() <= 1)
                continue;

            // Keep the first tied entry whose next-hop survives into tau_k+1
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

// ─── Runtime load management ────────────────────────────────────────────────────

double
IslRoutingManager::GetLinkQueueDelay(uint32_t satId, uint32_t ifIdx) const
{
    // TODO: replace with actual satellite-module queue API, e.g.:
    //   Ptr<Queue<Packet>> q = m_orbDevs[satId]->GetIslQueue(ifIdx);
    //   uint32_t bytes = q->GetCurrentSize().GetValue();
    //   return bytes * 8.0 / m_islLinkRateBps;
    (void)satId;
    (void)ifIdx;
    return 0.0;
}

void
IslRoutingManager::UpdateLoadCosts()
{
    m_prevLoadCosts = m_loadCosts;   // snapshot before EMA update

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a      = m_islDefs[edgeIdx].nodeA;
        uint32_t b      = m_islDefs[edgeIdx].nodeB;
        uint32_t ifA    = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifB    = m_perSatISLOrder[b].at(edgeIdx);
        uint32_t N      = m_numSatellites;

        double sample_ab = GetLinkQueueDelay(a, ifA);
        double sample_ba = GetLinkQueueDelay(b, ifB);

        // EMA: new = alpha * sample + (1 - alpha) * old
        m_loadCosts[a * N + b] =
            m_emaAlpha * sample_ab + (1.0 - m_emaAlpha) * m_loadCosts[a * N + b];
        m_loadCosts[b * N + a] =
            m_emaAlpha * sample_ba + (1.0 - m_emaAlpha) * m_loadCosts[b * N + a];
    }
}

bool
IslRoutingManager::HasSignificantChange() const
{
    // Cooldown guard: do not recompute more often than m_cooldownSeconds
    double elapsed = (Simulator::Now() - m_lastRecomputeTime).GetSeconds();
    if (elapsed < m_cooldownSeconds)
        return false;

    // Hysteresis: flag true only when fractional change exceeds threshold
    for (uint32_t i = 0; i < m_numSatellites * m_numSatellites; i++)
    {
        double prev = m_prevLoadCosts[i];
        double curr = m_loadCosts[i];
        double ref  = std::max(prev, curr);
        if (ref > 1e-12 && std::abs(curr - prev) / ref > m_changeThreshold)
            return true;
    }
    return false;
}

void
IslRoutingManager::RecomputeAffectedRoutes(uint32_t slotIndex)
{
    // Cache current satellite positions with one SGP4 call per satellite
    Time                now = Simulator::Now();
    std::vector<Vector> pos = GetPositionsAt(now);

    // Build load-aware graph at current simulation time
    ISLGraph graph = BuildISLGraphWithLoad(pos);

    // Identify affected sources: any satellite with an adjacent load-changed link
    const uint32_t N = m_numSatellites;
    std::vector<bool> affected(N, false);

    for (uint32_t a = 0; a < N; a++)
    {
        for (uint32_t b = 0; b < N; b++)
        {
            double prev = m_prevLoadCosts[a * N + b];
            double curr = m_loadCosts    [a * N + b];
            double ref  = std::max(prev, curr);
            if (ref > 1e-12 && std::abs(curr - prev) / ref > m_changeThreshold)
            {
                affected[a] = true;
                break;
            }
        }
    }

    // Partial Dijkstra: rerun only for affected sources
    uint32_t recomputedCount = 0;
    RoutingTable& table = m_tables[slotIndex];

    for (uint32_t src = 0; src < N; src++)
    {
        if (!affected[src])
            continue;
        table[src] = ComputeRoutesForSrc(src, graph);
        ++recomputedCount;
    }

    m_lastRecomputeTime = now;
    std::cout << "RecomputeAffectedRoutes: slot=" << slotIndex
              << " t=" << now.GetSeconds() << "s"
              << " recomputed=" << recomputedCount << "/" << N << " sources"
              << std::endl;
}

// ─── Private setup ──────────────────────────────────────────────────────────────

void
IslRoutingManager::LoadISLDefs(const std::string& islsFilePath)
{
    m_islDefs.clear();
    m_perSatISLOrder.assign(m_numSatellites, std::map<uint32_t, uint32_t>());

    std::ifstream file(islsFilePath);
    NS_ASSERT_MSG(file.is_open(), "Cannot open isls.txt: " << islsFilePath);

    std::string line;
    std::getline(file, line);   // skip header

    std::vector<uint32_t> counter(m_numSatellites, 0);

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::istringstream iss(line);
        uint32_t a, b;
        iss >> a >> b;

        uint32_t edgeIdx = m_islDefs.size();
        m_islDefs.push_back({a, b});

        m_perSatISLOrder[a][edgeIdx] = counter[a]++;
        m_perSatISLOrder[b][edgeIdx] = counter[b]++;
    }

    std::cout << "LoadISLDefs: loaded " << m_islDefs.size() << " ISLs" << std::endl;
}

void
IslRoutingManager::InitOrbiterDevices()
{
    m_orbDevs.resize(m_numSatellites);
    m_orbNodes.resize(m_numSatellites);

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
    }

    std::cout << "InitOrbiterDevices: done" << std::endl;
}

} // namespace ns3
