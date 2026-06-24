/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-resource-manager.cc
 *
 * SatResourceManager implementation.
 * See sat-bh-resource-manager.h for design rationale and full documentation.
 */

#include "sat-bh-resource-manager.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatResourceManager");
NS_OBJECT_ENSURE_REGISTERED(SatResourceManager);

TypeId
SatResourceManager::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatResourceManager")
            .SetParent<Object>()
            .AddConstructor<SatResourceManager>()
            .AddAttribute("FrameDuration",
                          "T_frame: self-scheduling loop period in milliseconds",
                          DoubleValue(80.0),  // T_p project fixed: 8 × 10 ms
                          MakeDoubleAccessor(&SatResourceManager::SetFrameDurationMs,
                                             &SatResourceManager::GetFrameDurationMs),
                          MakeDoubleChecker<double>(1.0, 10000.0))
            .AddAttribute("EnableUserAssociation",
                          "Run SatUserAssociator each frame (false = static association)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&SatResourceManager::m_enableUserAssoc),
                          MakeBooleanChecker())
            .AddAttribute("EnablePatternSelection",
                          "Run SatBeamPatternSelector at setup (feature flag, optional)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&SatResourceManager::m_enablePatternSel),
                          MakeBooleanChecker())
            .AddAttribute("NominalKbpsPerSlot",
                          "Nominal link capacity per active T_slot [kbps] for capacity hint",
                          DoubleValue(50000.0),
                          MakeDoubleAccessor(&SatResourceManager::m_nominalKbpsPerSlot),
                          MakeDoubleChecker<double>(1.0, 1e9))
            .AddAttribute("SatId",
                          "i: satellite index in the formal BH model (single-satellite scenario = 0)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&SatResourceManager::m_i),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("EnablePowerAllocation",
                          "Run SatPowerAllocator each frame (Phase D)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&SatResourceManager::m_enablePowerAlloc),
                          MakeBooleanChecker());
    return tid;
}

SatResourceManager::SatResourceManager()
    : m_frameDuration(MilliSeconds(80.0)),  // T_p project fixed: 8 × 10 ms
      m_enableUserAssoc(true),
      m_enablePatternSel(false),
      m_enablePowerAlloc(false),
      m_nominalKbpsPerSlot(50000.0),
      m_i(0),
      m_n(0)
{
    NS_LOG_INFO("SatResourceManager: constructed (T_frame="
                << m_frameDuration.GetMilliSeconds() << "ms)");
}

// ── Object lifecycle ──────────────────────────────────────────────────────

void
SatResourceManager::DoInitialize()
{
    // Chain up to Object::DoInitialize so ns-3 sub-object initialization runs
    Object::DoInitialize();

    NS_LOG_INFO("SatResourceManager::DoInitialize: scheduling first RunFrameOptimization"
                << " at t=0");

    // Self-scheduling loop: first call at t=0 (before any slot events fire).
    // Each call re-schedules itself at the end, so the period is exact.
    Simulator::Schedule(Seconds(0.0),
                        &SatResourceManager::RunFrameOptimization, this);
}

// ── UT state management ───────────────────────────────────────────────────

void
SatResourceManager::UpdateUtState(const UtInfo& ut)
{
    m_utStateMap[ut.utId] = ut;
    NS_LOG_DEBUG("SatResourceManager::UpdateUtState utId=" << ut.utId
                 << " beam=" << ut.currentBeamId
                 << " demand=" << ut.demandKbps << "kbps"
                 << " buffer=" << ut.bufferBytes << "B"
                 << " prio=" << ut.priorityClass);
}

void
SatResourceManager::RemoveUt(uint32_t utId)
{
    m_utStateMap.erase(utId);
    NS_LOG_INFO("SatResourceManager::RemoveUt utId=" << utId << " removed from managed set");
}

// ── Query ─────────────────────────────────────────────────────────────────

BeamConfig
SatResourceManager::GetBeamConfig(uint32_t beamId) const
{
    auto it = m_beamConfigs.find(beamId);
    if (it != m_beamConfigs.end())
        return it->second;

    // Return default config for beams not yet optimised
    BeamConfig def;
    def.beamId = beamId;
    return def;
}

std::vector<UtInfo>
SatResourceManager::GetUtList() const
{
    std::vector<UtInfo> utList;
    utList.reserve(m_utStateMap.size());
    for (const auto& [id, ut] : m_utStateMap)
        utList.push_back(ut);
    return utList;
}

// ── Frame optimization ────────────────────────────────────────────────────

void
SatResourceManager::RunFrameOptimization()
{
    m_n++;
    NS_LOG_INFO("SatResourceManager::RunFrameOptimization frameId=" << m_n
                << " t=" << Simulator::Now().GetSeconds() << "s"
                << " utCount=" << m_utStateMap.size());

    // ── Step 1: Retrieve active beam IDs from BHTP ────────────────────────
    std::vector<uint32_t> activeBeams = GetActiveBeamIds();
    if (activeBeams.empty())
    {
        NS_LOG_WARN("SatResourceManager: no active beams in current BHTP;"
                    " skipping user association for frameId=" << m_n);
    }

    // ── Step 2: Build per-beam capacity map ───────────────────────────────
    std::map<uint32_t, double> capacityMap = ComputeBeamCapacityMap(activeBeams);

    // ── Step 3: L1 path protection (optional, stub always returns empty) ──
    // Remove beams carrying active ISL routes from the reassignment pool.
    // In stub mode this has no effect; Phase E will activate this.
    if (m_l1Interface && m_l1Interface->IsEnabled())
    {
        // TODO Phase E: query L1 for protected beams and reduce reassignment
        // candidates accordingly.
        NS_LOG_DEBUG("SatResourceManager: L1 interface enabled but path protection"
                     " deferred to Phase E");
    }

    // ── Step 4: User association ──────────────────────────────────────────
    if (m_enableUserAssoc && m_associator && !activeBeams.empty() && !m_utStateMap.empty())
    {
        // Pass updated capacity map to associator
        m_associator->SetBeamCapacityMap(capacityMap);

        std::vector<UtInfo> utList = GetUtList();

        // Associate() computes frame N+1 assignment (pre-planned schedule)
        AssignmentMap assignment = m_associator->Associate(utList, activeBeams);

        // ApplyAssociation() issues MoveUtBetweenBeams for changed assignments.
        // TIM-U delay means the assignment takes effect at frame N+1 start.
        m_associator->ApplyAssociation(assignment, utList);

        // Update BeamConfig with UT assignments
        // Initialise all active beams with empty UT lists
        for (uint32_t bid : activeBeams)
        {
            BeamConfig& cfg = m_beamConfigs[bid];
            cfg.beamId       = bid;
            cfg.assignedUtIds.clear();
        }

        // Populate assignedUtIds from the just-computed assignment
        for (const auto& [utId, beamId] : assignment)
        {
            auto it = m_beamConfigs.find(beamId);
            if (it != m_beamConfigs.end())
                it->second.assignedUtIds.push_back(utId);
        }
    }
    else if (!m_enableUserAssoc)
    {
        NS_LOG_DEBUG("SatResourceManager: user association disabled (static mode)");
    }

    // ── Step 5: Set dwell times from BHTP ────────────────────────────────
    if (m_scheduler)
    {
        Ptr<SatBhTimePlan> plan = m_scheduler->GetCurrentPlan();
        if (plan)
        {
            for (uint32_t bid : activeBeams)
            {
                BeamConfig& cfg = m_beamConfigs[bid];
                cfg.beamId  = bid;
                cfg.dwellMs = plan->GetBeamDwellMs(bid);
            }
            // Stamp the frame ID into the plan for traceability
            plan->SetFrameN(m_n);
        }
    }

    // ── Step 5.5: Power allocation (Phase D) ─────────────────────────────
    // Build channel gains (stub: 1.0 per beam; Phase E injects real CSI).
    // Build demand map by summing assigned UTs' demandKbps per beam.
    if (m_enablePowerAlloc && m_powerAllocator && !activeBeams.empty())
    {
        std::map<uint32_t, double> channelGains;
        std::map<uint32_t, double> demandKbps;

        for (uint32_t bid : activeBeams)
        {
            channelGains[bid] = 1.0; // Phase D stub; Phase E: real G_tx*G_rx/FSL

            double demand = 0.0;
            auto   cfgIt  = m_beamConfigs.find(bid);
            if (cfgIt != m_beamConfigs.end())
            {
                for (uint32_t utId : cfgIt->second.assignedUtIds)
                {
                    auto utIt = m_utStateMap.find(utId);
                    if (utIt != m_utStateMap.end())
                        demand += utIt->second.demandKbps;
                }
            }
            demandKbps[bid] = demand;
        }

        PowerMap powers = m_powerAllocator->Allocate(activeBeams, channelGains, demandKbps);
        m_powerAllocator->ApplyPower(powers);

        // Write allocated power back into BeamConfig for FrameConfigCallback consumers
        for (const auto& [bid, powerDbw] : powers)
        {
            auto it = m_beamConfigs.find(bid);
            if (it != m_beamConfigs.end())
                it->second.allocatedPowerDbw = powerDbw;
        }

        NS_LOG_INFO("SatResourceManager: power allocation done for frameId=" << m_n
                    << " beams=" << powers.size());
    }
    else if (m_enablePowerAlloc && !m_powerAllocator)
    {
        NS_LOG_WARN("SatResourceManager: EnablePowerAllocation=true but"
                    " SatPowerAllocator not wired — skipping (frameId=" << m_n << ")");
    }

    // ── Step 6: Fire FrameConfigCallback ─────────────────────────────────
    if (m_frameConfigCb)
    {
        m_frameConfigCb(m_n, m_beamConfigs);
        NS_LOG_DEBUG("SatResourceManager: FrameConfigCallback fired frameId=" << m_n
                     << " beams=" << m_beamConfigs.size());
    }

    // ── Step 7: Schedule next frame ───────────────────────────────────────
    // Anchored to current fire time to prevent drift accumulation over many frames.
    Simulator::Schedule(m_frameDuration,
                        &SatResourceManager::RunFrameOptimization, this);

    NS_LOG_INFO("SatResourceManager: frame " << m_n << " done;"
                << " next optimization at t="
                << (Simulator::Now() + m_frameDuration).GetSeconds() << "s");
}

// ── Internal helpers ──────────────────────────────────────────────────────

std::vector<uint32_t>
SatResourceManager::GetActiveBeamIds() const
{
    if (m_scheduler)
    {
        Ptr<SatBhTimePlan> plan = m_scheduler->GetCurrentPlan();
        if (plan)
        {
            std::vector<uint32_t> beams = plan->GetAllBeamIds();
            NS_LOG_DEBUG("SatResourceManager::GetActiveBeamIds: "
                         << beams.size() << " beams from current BHTP plan "
                         << plan->GetPlanId());
            return beams;
        }
    }

    // Fallback: use the static list set by SatBhHelper (if any)
    if (!m_fallbackBeamIds.empty())
    {
        NS_LOG_WARN("SatResourceManager::GetActiveBeamIds: no BHTP available;"
                    " using fallback beam list (" << m_fallbackBeamIds.size() << " beams)");
        return m_fallbackBeamIds;
    }

    NS_LOG_WARN("SatResourceManager::GetActiveBeamIds: neither BHTP nor fallback available;"
                " returning empty beam list");
    return {};
}

std::map<uint32_t, double>
SatResourceManager::ComputeBeamCapacityMap(const std::vector<uint32_t>& beamIds) const
{
    std::map<uint32_t, double> capacityMap;
    if (beamIds.empty())
        return capacityMap;

    // Capacity hint = (dwellMs / T_slot) × nominalKbpsPerSlot
    // Without BHTP: assume equal dwell across all beams (M × T_s / numBeams).
    const double T_slot_ms = 10.0; // project fixed T_s

    Ptr<SatBhTimePlan> plan;
    if (m_scheduler)
        plan = m_scheduler->GetCurrentPlan();

    for (uint32_t bid : beamIds)
    {
        double dwellMs = plan ? plan->GetBeamDwellMs(bid)
                               : (80.0 / static_cast<double>(beamIds.size())); // T_p = 80 ms project
        double numSlots    = dwellMs / T_slot_ms;
        capacityMap[bid]   = numSlots * m_nominalKbpsPerSlot;
        NS_LOG_DEBUG("SatResourceManager::ComputeBeamCapacityMap beam=" << bid
                     << " dwellMs=" << dwellMs
                     << " capacity=" << capacityMap[bid] << "kbps");
    }

    return capacityMap;
}

} // namespace ns3
