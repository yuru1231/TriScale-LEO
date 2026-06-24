/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-user-associator.cc
 *
 * SatUserAssociator implementation.
 * See sat-bh-user-associator.h for design rationale and full documentation.
 */

#include "sat-bh-user-associator.h"

#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatUserAssociator");
NS_OBJECT_ENSURE_REGISTERED(SatUserAssociator);

TypeId
SatUserAssociator::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatUserAssociator")
            .SetParent<Object>()
            .AddConstructor<SatUserAssociator>()
            .AddAttribute("SchedulingMode",
                          "0=WFQ, 1=Priority, 2=RoundRobin (default WFQ)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&SatUserAssociator::SetSchedulingModeAttr,
                                               &SatUserAssociator::GetSchedulingModeAttr),
                          MakeUintegerChecker<uint8_t>(0, 2))
            .AddAttribute("MaxReassignmentPerFrame",
                          "Max MoveUtBetweenBeams calls issued per frame (deadline UTs bypass)",
                          UintegerValue(5),
                          MakeUintegerAccessor(&SatUserAssociator::m_maxReassignment),
                          MakeUintegerChecker<uint32_t>(0, 1000))
            .AddAttribute("MinRateKbps",
                          "Minimum rate threshold [kbps]; UTs below get reduced WFQ weight",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&SatUserAssociator::m_minRateKbps),
                          MakeDoubleChecker<double>(0.0, 1e6))
            .AddAttribute("MaxDelayMs",
                          "HOL delay limit [ms]; UTs beyond this get forced reassignment",
                          DoubleValue(530.0),
                          MakeDoubleAccessor(&SatUserAssociator::m_maxDelayMs),
                          MakeDoubleChecker<double>(0.0, 1e5));
    return tid;
}

SatUserAssociator::SatUserAssociator()
    : m_schedulingMode(SchedulingMode::WFQ),
      m_maxReassignment(5),
      m_minRateKbps(100.0),
      m_maxDelayMs(530.0),
      m_i(0),
      m_rrIndex(0),
      m_lastReassignCount(0)
{
    NS_LOG_INFO("SatUserAssociator: constructed (default mode=WFQ)");
}

// ── Configuration ─────────────────────────────────────────────────────────

void
SatUserAssociator::SetBeamCapacityMap(const std::map<uint32_t, double>& capacityKbps)
{
    m_beamCapacityKbps = capacityKbps;
    NS_LOG_DEBUG("SatUserAssociator::SetBeamCapacityMap: "
                 << capacityKbps.size() << " beams updated");
}

// ── Core operations ───────────────────────────────────────────────────────

AssignmentMap
SatUserAssociator::Associate(const std::vector<UtInfo>& utList,
                             const std::vector<uint32_t>& beamIds)
{
    NS_LOG_INFO("SatUserAssociator::Associate t="
                << Simulator::Now().GetSeconds() << "s"
                << " utCount=" << utList.size()
                << " beams=" << beamIds.size()
                << " mode=" << static_cast<int>(m_schedulingMode));

    if (utList.empty() || beamIds.empty())
        return {};

    AssignmentMap assignment;
    switch (m_schedulingMode)
    {
        case SchedulingMode::WFQ:
            assignment = RunWfq(utList, beamIds);
            break;
        case SchedulingMode::PRIORITY:
            assignment = RunPriority(utList, beamIds);
            break;
        case SchedulingMode::ROUND_ROBIN:
            assignment = RunRoundRobin(utList, beamIds);
            break;
        default:
            assignment = RunWfq(utList, beamIds);
            break;
    }

    // Deadline enforcement: override assignment for HOL-critical UTs
    uint32_t forced = EnforceDeadlineProtection(assignment, utList, beamIds);
    if (forced > 0)
        NS_LOG_INFO("SatUserAssociator: forced " << forced
                    << " deadline-violating UTs to high-capacity beams");

    return assignment;
}

void
SatUserAssociator::ApplyAssociation(const AssignmentMap& assignment,
                                    const std::vector<UtInfo>& utList)
{
    // Build lookup: utId → current beam from the UtInfo list
    std::map<uint32_t, uint32_t> currentBeam;
    for (const auto& ut : utList)
        currentBeam[ut.utId] = ut.currentBeamId;

    // Identify UTs that need to move (new beam differs from current)
    std::vector<std::pair<uint32_t, uint32_t>> moves; // (utId, newBeamId)
    for (const auto& [utId, newBeam] : assignment)
    {
        auto it = currentBeam.find(utId);
        if (it == currentBeam.end() || it->second != newBeam)
            moves.push_back({utId, newBeam});
    }

    // Separate deadline-violated UTs (they bypass the reassignment cap)
    std::map<uint32_t, double> holDelay;
    for (const auto& ut : utList)
        holDelay[ut.utId] = ut.headOfLineDelayMs;

    std::vector<std::pair<uint32_t, uint32_t>> urgentMoves, normalMoves;
    for (const auto& [utId, newBeam] : moves)
    {
        double delay = holDelay.count(utId) ? holDelay[utId] : 0.0;
        if (delay >= m_maxDelayMs)
            urgentMoves.push_back({utId, newBeam});
        else
            normalMoves.push_back({utId, newBeam});
    }

    // Apply urgent moves first (deadline-protected, bypass cap)
    uint32_t count = 0;
    for (const auto& [utId, newBeam] : urgentMoves)
    {
        uint32_t srcBeam = currentBeam.count(utId) ? currentBeam[utId] : 0;
        NS_LOG_INFO("SatUserAssociator: URGENT move utId=" << utId
                    << " beam " << srcBeam << "→" << newBeam);
        if (m_moveUtCb)
            m_moveUtCb(utId, m_i, srcBeam, m_i, newBeam);
        else
            NS_LOG_WARN("SatUserAssociator: MoveUtCallback not wired — move utId="
                        << utId << " beam " << srcBeam << "→" << newBeam
                        << " is logged only (Phase E will wire this)");
        count++;
    }

    // Apply normal moves up to MaxReassignmentPerFrame limit
    for (const auto& [utId, newBeam] : normalMoves)
    {
        if (count - static_cast<uint32_t>(urgentMoves.size()) >= m_maxReassignment)
        {
            NS_LOG_DEBUG("SatUserAssociator: MaxReassignmentPerFrame=" << m_maxReassignment
                         << " reached; deferring remaining " << normalMoves.size()
                         << " moves to next frame");
            break;
        }
        uint32_t srcBeam = currentBeam.count(utId) ? currentBeam[utId] : 0;
        NS_LOG_INFO("SatUserAssociator: move utId=" << utId
                    << " beam " << srcBeam << "→" << newBeam);
        if (m_moveUtCb)
            m_moveUtCb(utId, m_i, srcBeam, m_i, newBeam);
        else
            NS_LOG_WARN("SatUserAssociator: MoveUtCallback not wired — move utId="
                        << utId << " beam " << srcBeam << "→" << newBeam
                        << " is logged only (Phase E will wire this)");
        count++;
    }

    m_lastReassignCount = count;
    m_prevAssignment    = assignment;

    NS_LOG_INFO("SatUserAssociator::ApplyAssociation: issued " << count
                << " MoveUtBetweenBeams calls ("
                << urgentMoves.size() << " urgent, "
                << (count - static_cast<uint32_t>(urgentMoves.size())) << " normal)");
}

// ── WFQ helpers ───────────────────────────────────────────────────────────

double
SatUserAssociator::ComputeWfqWeight(const UtInfo& ut) const
{
    // weight = (priorityClass+1) × (1 + bufferFill + demandFraction)
    // bufferFill   = bufferBytes / 1500 (normalised by MTU-sized packet)
    // demandFraction = max(demandKbps, minRateKbps) / minRateKbps
    //
    // Rationale: higher priority class multiplies the weight; buffer fill
    // captures HOL urgency; demand fraction ensures high-demand UTs get
    // proportionally more capacity in the WFQ allocation.
    double bufferFill    = ut.bufferBytes / 1500.0;
    double demandFrac    = std::max(ut.demandKbps, m_minRateKbps) / m_minRateKbps;
    return static_cast<double>(ut.priorityClass + 1) * (1.0 + bufferFill + demandFrac);
}

bool
SatUserAssociator::IsDeadlineViolated(const UtInfo& ut) const
{
    return ut.headOfLineDelayMs >= m_maxDelayMs;
}

// ── Scheduling: WFQ ───────────────────────────────────────────────────────

AssignmentMap
SatUserAssociator::RunWfq(const std::vector<UtInfo>& utList,
                          const std::vector<uint32_t>& beamIds)
{
    AssignmentMap assignment;
    if (beamIds.empty())
        return assignment;

    std::set<uint32_t> activeBeams(beamIds.begin(), beamIds.end());

    // Per-beam remaining capacity in Kbps.
    // Initialise from m_beamCapacityKbps; default to large value if not set
    // (meaning beam can absorb any number of UTs).
    std::map<uint32_t, double> remaining;
    for (uint32_t bid : beamIds)
    {
        auto it = m_beamCapacityKbps.find(bid);
        remaining[bid] = (it != m_beamCapacityKbps.end()) ? it->second : 1e9;
    }

    // Sort UTs by WFQ weight descending (highest weight served first)
    std::vector<size_t> order(utList.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) {
                  return ComputeWfqWeight(utList[a]) > ComputeWfqWeight(utList[b]);
              });

    for (size_t idx : order)
    {
        const UtInfo& ut = utList[idx];

        // First preference: stay on current beam if it is in the active set
        // (avoids unnecessary TIM-U handover overhead)
        uint32_t chosen = 0;
        if (activeBeams.count(ut.currentBeamId) &&
            remaining[ut.currentBeamId] >= ut.demandKbps)
        {
            chosen = ut.currentBeamId;
        }
        else
        {
            // Find the active beam with the most remaining capacity
            double    bestCap  = -1.0;
            for (uint32_t bid : beamIds)
            {
                if (remaining[bid] >= ut.demandKbps && remaining[bid] > bestCap)
                {
                    bestCap = remaining[bid];
                    chosen  = bid;
                }
            }
            // If no beam has enough remaining capacity, pick the least-loaded one
            if (chosen == 0 && !beamIds.empty())
            {
                double    maxRem = -1.0;
                for (uint32_t bid : beamIds)
                    if (remaining[bid] > maxRem)
                    {
                        maxRem = remaining[bid];
                        chosen = bid;
                    }
            }
        }

        if (chosen != 0)
        {
            assignment[ut.utId]  = chosen;
            remaining[chosen]   -= ut.demandKbps;
            NS_LOG_DEBUG("SatUserAssociator WFQ: utId=" << ut.utId
                         << " → beam " << chosen
                         << " weight=" << ComputeWfqWeight(ut)
                         << " remaining=" << remaining[chosen]);
        }
    }

    return assignment;
}

// ── Scheduling: Priority ──────────────────────────────────────────────────

AssignmentMap
SatUserAssociator::RunPriority(const std::vector<UtInfo>& utList,
                               const std::vector<uint32_t>& beamIds)
{
    AssignmentMap assignment;
    if (beamIds.empty())
        return assignment;

    // Sort by priorityClass descending, then by demand descending within class
    std::vector<size_t> order(utList.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](size_t a, size_t b) {
                  if (utList[a].priorityClass != utList[b].priorityClass)
                      return utList[a].priorityClass > utList[b].priorityClass;
                  return utList[a].demandKbps > utList[b].demandKbps;
              });

    std::map<uint32_t, double> remaining;
    for (uint32_t bid : beamIds)
    {
        auto it = m_beamCapacityKbps.find(bid);
        remaining[bid] = (it != m_beamCapacityKbps.end()) ? it->second : 1e9;
    }

    std::set<uint32_t> activeBeams(beamIds.begin(), beamIds.end());

    for (size_t idx : order)
    {
        const UtInfo& ut = utList[idx];
        uint32_t chosen = 0;

        // Prefer current beam; fallback to highest-capacity active beam
        if (activeBeams.count(ut.currentBeamId) &&
            remaining[ut.currentBeamId] >= ut.demandKbps)
        {
            chosen = ut.currentBeamId;
        }
        else
        {
            double maxRem = -1.0;
            for (uint32_t bid : beamIds)
                if (remaining[bid] > maxRem)
                {
                    maxRem = remaining[bid];
                    chosen = bid;
                }
        }

        if (chosen != 0)
        {
            assignment[ut.utId] = chosen;
            remaining[chosen]  -= ut.demandKbps;
        }
    }

    return assignment;
}

// ── Scheduling: Round-Robin ───────────────────────────────────────────────

AssignmentMap
SatUserAssociator::RunRoundRobin(const std::vector<UtInfo>& utList,
                                 const std::vector<uint32_t>& beamIds)
{
    AssignmentMap assignment;
    if (beamIds.empty() || utList.empty())
        return assignment;

    // Rotate the UT list starting from m_rrIndex, assigning beams cyclically.
    // m_rrIndex persists across frames so each frame starts from where the
    // previous one left off, ensuring fairness over time.
    uint32_t numBeams = static_cast<uint32_t>(beamIds.size());
    uint32_t numUts   = static_cast<uint32_t>(utList.size());

    for (uint32_t i = 0; i < numUts; i++)
    {
        uint32_t utPos   = (m_rrIndex + i) % numUts;
        uint32_t beamPos = i % numBeams;
        assignment[utList[utPos].utId] = beamIds[beamPos];
    }

    // Advance cursor by one for next frame
    m_rrIndex = (m_rrIndex + 1) % numUts;

    return assignment;
}

// ── Deadline enforcement ──────────────────────────────────────────────────

uint32_t
SatUserAssociator::EnforceDeadlineProtection(AssignmentMap& assignment,
                                              const std::vector<UtInfo>& utList,
                                              const std::vector<uint32_t>& beamIds)
{
    if (beamIds.empty())
        return 0;

    // Find the beam with the highest remaining capacity after current assignment.
    // Compute remaining = total capacity - sum of demand of assigned UTs.
    std::map<uint32_t, double> usedKbps;
    for (uint32_t bid : beamIds)
        usedKbps[bid] = 0.0;

    std::map<uint32_t, double> utDemand;
    for (const auto& ut : utList)
        utDemand[ut.utId] = ut.demandKbps;

    for (const auto& [utId, beam] : assignment)
        usedKbps[beam] += utDemand.count(utId) ? utDemand[utId] : 0.0;

    // Best beam: highest (capacity − used)
    uint32_t bestBeam    = beamIds.front();
    double   bestFreeKbps = -1e9;
    for (uint32_t bid : beamIds)
    {
        auto capIt  = m_beamCapacityKbps.find(bid);
        double cap  = (capIt != m_beamCapacityKbps.end()) ? capIt->second : 1e9;
        double free = cap - usedKbps[bid];
        if (free > bestFreeKbps)
        {
            bestFreeKbps = free;
            bestBeam     = bid;
        }
    }

    uint32_t forcedCount = 0;
    for (const auto& ut : utList)
    {
        if (IsDeadlineViolated(ut))
        {
            uint32_t oldBeam = assignment.count(ut.utId) ? assignment[ut.utId] : 0;
            assignment[ut.utId] = bestBeam;
            if (oldBeam != bestBeam)
            {
                forcedCount++;
                NS_LOG_INFO("SatUserAssociator::EnforceDeadlineProtection: utId="
                            << ut.utId << " HOL=" << ut.headOfLineDelayMs
                            << "ms >= maxDelay=" << m_maxDelayMs
                            << "ms → forced to beam " << bestBeam);
            }
        }
    }

    return forcedCount;
}

} // namespace ns3
