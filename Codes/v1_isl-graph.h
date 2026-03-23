//not oop
#include "isl-graph.h"

#include "ns3/satellite-isl-arbiter-unicast.h"
#include "ns3/satellite-orbiter-net-device.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <set>
#include <sstream>

namespace ns3
{

static std::vector<ISLDef>                       islDefs;
static std::vector<std::map<uint32_t, uint32_t>> perSatISLOrder;
static std::vector<Ptr<SatOrbiterNetDevice>>     g_orbDevs;
static std::vector<Ptr<Node>>                    g_orbNodes;

void
LoadISLDefs(const std::string& islsFilePath)
{
    islDefs.clear();
    perSatISLOrder.assign(66, std::map<uint32_t, uint32_t>());

    std::ifstream file(islsFilePath);
    NS_ASSERT_MSG(file.is_open(), "Cannot open isls.txt: " << islsFilePath);

    std::string line;
    std::getline(file, line);  // 跳過 header

    std::vector<uint32_t> counter(66, 0);

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        std::istringstream iss(line);
        uint32_t a, b;
        iss >> a >> b;

        uint32_t edgeIdx = islDefs.size();
        islDefs.push_back({a, b});

        perSatISLOrder[a][edgeIdx] = counter[a]++;
        perSatISLOrder[b][edgeIdx] = counter[b]++;
    }

    std::cout << "LoadISLDefs: loaded " << islDefs.size() << " ISLs"
              << std::endl;
}

void
InitOrbiterDevices()
{
    g_orbDevs.resize(66);
    g_orbNodes.resize(66);

    for (uint32_t i = 0; i < 66; i++)
    {
        Ptr<Node> sat = Singleton<SatTopology>::Get()->GetOrbiterNode(i);
        g_orbNodes[i] = sat;

        for (uint32_t d = 0; d < sat->GetNDevices(); d++)
        {
            Ptr<SatOrbiterNetDevice> dev =
                DynamicCast<SatOrbiterNetDevice>(sat->GetDevice(d));
            if (dev)
            {
                g_orbDevs[i] = dev;
                break;
            }
        }
        NS_ASSERT_MSG(g_orbDevs[i], "no SatOrbiterNetDevice on sat=" << i);
    }

    std::cout << "InitOrbiterDevices: done" << std::endl;
}

ISLGraph
BuildISLGraph(Time tau_k)
{
    const double C        = 3e8;
    const double MAX_DIST = 5000e3;

    ISLGraph graph(66);

    for (uint32_t edgeIdx = 0; edgeIdx < islDefs.size(); edgeIdx++)
    {
        uint32_t a = islDefs[edgeIdx].nodeA;
        uint32_t b = islDefs[edgeIdx].nodeB;

        Ptr<SatSGP4MobilityModel> mobA =
            g_orbNodes[a]->GetObject<SatSGP4MobilityModel>();
        Ptr<SatSGP4MobilityModel> mobB =
            g_orbNodes[b]->GetObject<SatSGP4MobilityModel>();

        NS_ASSERT_MSG(mobA, "Sat " << a << " has no SatSGP4MobilityModel");
        NS_ASSERT_MSG(mobB, "Sat " << b << " has no SatSGP4MobilityModel");

        Vector posA = mobA->GetGeoPositionAt(tau_k).ToVector();
        Vector posB = mobB->GetGeoPositionAt(tau_k).ToVector();

        double dx   = posA.x - posB.x;
        double dy   = posA.y - posB.y;
        double dz   = posA.z - posB.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > MAX_DIST)
            continue;

        double   prop_cost = dist / C;
        uint32_t ifIdxOnA  = perSatISLOrder[a][edgeIdx];
        uint32_t ifIdxOnB  = perSatISLOrder[b][edgeIdx];

        graph[a].push_back({b, prop_cost, ifIdxOnA, ifIdxOnB});
        graph[b].push_back({a, prop_cost, ifIdxOnB, ifIdxOnA});
    }

    return graph;
}

RoutingTable
ComputeBaseRoutes(const ISLGraph& graph)
{
    const uint32_t n   = 66;
    const double   INF = std::numeric_limits<double>::infinity();

    RoutingTable result(n);

    for (uint32_t src = 0; src < n; src++)
    {
        std::vector<double>   dist(n, INF);
        std::vector<uint32_t> prevHop(n, UINT32_MAX);
        std::vector<uint32_t> prevIfIdx(n, UINT32_MAX);

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
                    dist[e.nodeB]      = nd;
                    prevHop[e.nodeB]   = u;
                    prevIfIdx[e.nodeB] = e.islIfIndexOnA;
                    pq.push({nd, e.nodeB});
                }
            }
        }

        for (uint32_t dest = 0; dest < n; dest++)
        {
            if (dest == src || dist[dest] == INF)
                continue;

            uint32_t cur   = dest;
            uint32_t steps = 0;
            while (prevHop[cur] != src)
            {
                cur = prevHop[cur];
                NS_ASSERT_MSG(++steps < n,
                              "Routing loop: src=" << src
                              << " dest=" << dest);
            }

            result[src].push_back({dest, cur, prevIfIdx[cur], dist[dest]});
        }
    }

    return result;
}

RoutingTable
ApplyTiebreaker(const RoutingTable& routes,
                const ISLGraph&     graphNext)
{
    std::set<std::pair<uint32_t, uint32_t>> nextEligible;
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
        vec.erase(std::remove_if(vec.begin(),
                                 vec.end(),
                                 [](const RouteEntry& e) {
                                     return e.cost ==
                                            std::numeric_limits<double>::infinity();
                                 }),
                  vec.end());
    }

    return result;
}

void
PrecomputeAllTables(PrecomputedTables& tables)
{
    std::cout << "PrecomputeAllTables: start" << std::endl;

    for (uint32_t k = 0; k < 10; k++)
    {
        Time     tau_k     = Seconds(60.0 * k);
        ISLGraph graphCurr = BuildISLGraph(tau_k);
        RoutingTable routes = ComputeBaseRoutes(graphCurr);

        if (k < 9)
        {
            Time     tau_next  = Seconds(60.0 * (k + 1));
            ISLGraph graphNext = BuildISLGraph(tau_next);
            routes = ApplyTiebreaker(routes, graphNext);
        }

        tables[k] = routes;
        std::cout << "  t=" << 60 * k
                  << "s: done, SAT0 routes=" << tables[k][0].size()
                  << std::endl;
    }

    std::cout << "PrecomputeAllTables: complete" << std::endl;
}

void
ApplyRoutingTable(uint32_t slotIndex, const PrecomputedTables& tables)
{
    NS_ASSERT_MSG(slotIndex < tables.size(),
                  "slotIndex out of range: " << slotIndex);

    const RoutingTable& table = tables[slotIndex];

    for (uint32_t satId = 0; satId < 66; satId++)
    {
        Ptr<SatIslArbiterUnicast> arbiter =
            CreateObject<SatIslArbiterUnicast>(g_orbNodes[satId]);
        for (const auto& entry : table[satId])
            arbiter->AddNextHopEntry(entry.destSatId, entry.islIfIndexOnA);
        g_orbDevs[satId]->SetArbiter(arbiter);
    }

    std::cout << "ApplyRoutingTable: slot=" << slotIndex
              << " t=" << Simulator::Now().GetSeconds() << "s done"
              << std::endl;
}

void
ScheduleRoutingUpdates(const PrecomputedTables& tables)
{
    for (uint32_t k = 0; k < 10; k++)
    {
        Time t = Seconds(60.0 * k);
        Simulator::Schedule(t, [k, &tables]() {
            ApplyRoutingTable(k, tables);
        });
    }
    std::cout << "ScheduleRoutingUpdates: 10 events scheduled" << std::endl;
}

} // namespace ns3
