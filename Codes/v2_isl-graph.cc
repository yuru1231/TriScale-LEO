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
#include <set>
#include <sstream>

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
                          MakeStringChecker());
    return tid;
}

IslRoutingManager::IslRoutingManager()
    : m_numSatellites(66),
      m_islMaxDistanceKm(5000.0),
      m_numTimeSlots(10),
      m_timeSlotInterval(60.0),
      m_islsFilePath("")
{
}

// ─── public ──────────────────────────────────────────────────

void
IslRoutingManager::Initialize(const std::string& islsFilePath)
{
    m_islsFilePath = islsFilePath;
    LoadISLDefs(m_islsFilePath);
    InitOrbiterDevices();
}

void
IslRoutingManager::PrecomputeAllTables()
{
    m_tables.resize(m_numTimeSlots);
    std::cout << "PrecomputeAllTables: start" << std::endl;

    for (uint32_t k = 0; k < m_numTimeSlots; k++)
    {
        Time     tau_k     = Seconds(m_timeSlotInterval * k);
        ISLGraph graphCurr = BuildISLGraph(tau_k);
        RoutingTable routes = ComputeBaseRoutes(graphCurr);

        if (k < m_numTimeSlots - 1)
        {
            Time     tau_next  = Seconds(m_timeSlotInterval * (k + 1));
            ISLGraph graphNext = BuildISLGraph(tau_next);
            routes = ApplyTiebreaker(routes, graphNext);
        }

        m_tables[k] = routes;
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
        Simulator::Schedule(t,
                            &IslRoutingManager::ApplyRoutingTable,
                            this,
                            k);
    }
    std::cout << "ScheduleRoutingUpdates: "
              << m_numTimeSlots << " events scheduled" << std::endl;
}

void
IslRoutingManager::ApplyRoutingTable(uint32_t slotIndex)
{
    NS_ASSERT_MSG(slotIndex < m_tables.size(),
                  "slotIndex out of range: " << slotIndex);

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
              << " t=" << Simulator::Now().GetSeconds() << "s done"
              << std::endl;
}

ISLGraph
IslRoutingManager::BuildISLGraph(Time tau_k) const
{
    const double C        = 3e8;
    const double MAX_DIST = m_islMaxDistanceKm * 1e3;

    ISLGraph graph(m_numSatellites);

    for (uint32_t edgeIdx = 0; edgeIdx < m_islDefs.size(); edgeIdx++)
    {
        uint32_t a = m_islDefs[edgeIdx].nodeA;
        uint32_t b = m_islDefs[edgeIdx].nodeB;

        Ptr<SatSGP4MobilityModel> mobA =
            m_orbNodes[a]->GetObject<SatSGP4MobilityModel>();
        Ptr<SatSGP4MobilityModel> mobB =
            m_orbNodes[b]->GetObject<SatSGP4MobilityModel>();

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
        uint32_t ifIdxOnA  = m_perSatISLOrder[a].at(edgeIdx);
        uint32_t ifIdxOnB  = m_perSatISLOrder[b].at(edgeIdx);

        graph[a].push_back({b, prop_cost, ifIdxOnA, ifIdxOnB});
        graph[b].push_back({a, prop_cost, ifIdxOnB, ifIdxOnA});
    }

    return graph;
}

// ─── private ─────────────────────────────────────────────────

void
IslRoutingManager::LoadISLDefs(const std::string& islsFilePath)
{
    m_islDefs.clear();
    m_perSatISLOrder.assign(m_numSatellites, std::map<uint32_t, uint32_t>());

    std::ifstream file(islsFilePath);
    NS_ASSERT_MSG(file.is_open(), "Cannot open isls.txt: " << islsFilePath);

    std::string line;
    std::getline(file, line);  // 跳過 header

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

    std::cout << "LoadISLDefs: loaded " << m_islDefs.size() << " ISLs"
              << std::endl;
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

RoutingTable
IslRoutingManager::ComputeBaseRoutes(const ISLGraph& graph) const
{
    const double INF = std::numeric_limits<double>::infinity();

    RoutingTable result(m_numSatellites);

    for (uint32_t src = 0; src < m_numSatellites; src++)
    {
        std::vector<double>   dist(m_numSatellites, INF);
        std::vector<uint32_t> prevHop(m_numSatellites, UINT32_MAX);
        std::vector<uint32_t> prevIfIdx(m_numSatellites, UINT32_MAX);

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

        for (uint32_t dest = 0; dest < m_numSatellites; dest++)
        {
            if (dest == src || dist[dest] == INF)
                continue;

            uint32_t cur   = dest;
            uint32_t steps = 0;
            while (prevHop[cur] != src)
            {
                cur = prevHop[cur];
                NS_ASSERT_MSG(++steps < m_numSatellites,
                              "Routing loop: src=" << src
                              << " dest=" << dest);
            }

            result[src].push_back({dest, cur, prevIfIdx[cur], dist[dest]});
        }
    }

    return result;
}

RoutingTable
IslRoutingManager::ApplyTiebreaker(const RoutingTable& routes,
                                    const ISLGraph&     graphNext) const
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

} // namespace ns3
