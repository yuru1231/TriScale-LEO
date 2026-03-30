/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-obc.cc
 *
 * SatBhObc — STUB implementation (Phase 2, not yet active).
 *
 * All method bodies log their invocation but perform no state transitions.
 * Interfaces are final; only this .cc changes when Phase 2 is implemented.
 *
 * Implementation checklist (Layer2.md Step 4):
 *   [ ] ReceiveNewPlan: store plan, schedule EnterSlot(0) at plan start time
 *   [ ] EnterSlot: emit BeamActivateCallback, schedule OnSlotServiceEnd at T_s−T_sw
 *   [ ] OnSlotServiceEnd: emit BeamDeactivateCallback, transition to SWITCHING
 *   [ ] OnSwitchingDone: advance slot index or → WAIT_PLAN if last slot
 *   [ ] WAIT_PLAN → ACTIVE: immediate activation when pending plan arrives
 *   [ ] GetActiveBeams / GetCurrentSlotIndex returning live state
 */

#include "sat-bh-obc.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatBhObc");
NS_OBJECT_ENSURE_REGISTERED(SatBhObc);

std::string
ObcStateToString(ObcState s)
{
    switch (s)
    {
        case ObcState::IDLE:      return "IDLE";
        case ObcState::ACTIVE:    return "ACTIVE";
        case ObcState::SWITCHING: return "SWITCHING";
        case ObcState::WAIT_PLAN: return "WAIT_PLAN";
        default:                  return "UNKNOWN";
    }
}

TypeId
SatBhObc::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatBhObc")
            .SetParent<Object>()
            .AddConstructor<SatBhObc>()
            .AddAttribute("SatId",
                          "Satellite node index this OBC is attached to",
                          UintegerValue(0),
                          MakeUintegerAccessor(&SatBhObc::m_satId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SwitchingTimeMs",
                          "T_sw: beam switching dead-time in milliseconds",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&SatBhObc::SetSwitchingTimeMs,
                                             &SatBhObc::GetSwitchingTimeMs),
                          MakeDoubleChecker<double>(0.0, 100.0));
    return tid;
}

SatBhObc::SatBhObc()
    : m_state(ObcState::IDLE),
      m_satId(0),
      m_switchingTime(MilliSeconds(2.0)),
      m_currentSlotIdx(-1)
{
    NS_LOG_INFO("SatBhObc: constructed (Phase 2 stub — not yet implemented)");
}

// ── Plan reception ────────────────────────────────────────────────────────

void
SatBhObc::ReceiveNewPlan(Ptr<SatBhTimePlan> plan, Time /*propagationDelay*/)
{
    // TODO Phase 2:
    //   if IDLE or WAIT_PLAN → store as m_activePlan, schedule EnterSlot(0)
    //   if ACTIVE/SWITCHING  → store as m_pendingPlan (activate when current ends)
    NS_LOG_INFO("SatBhObc::ReceiveNewPlan planId=" << (plan ? plan->GetPlanId() : 0)
                << " state=" << ObcStateToString(m_state) << " [STUB]");
}

// ── Callback registration ─────────────────────────────────────────────────

void
SatBhObc::SetBeamActivateCallback(BeamActivateCallback cb)
{
    m_activateCb = cb;
}

void
SatBhObc::SetBeamDeactivateCallback(BeamDeactivateCallback cb)
{
    m_deactivateCb = cb;
}

// ── Identity ──────────────────────────────────────────────────────────────

void
SatBhObc::SetSatId(uint32_t satId)
{
    m_satId = satId;
}

// ── State query ───────────────────────────────────────────────────────────

const std::vector<uint32_t>&
SatBhObc::GetActiveBeams() const
{
    return m_activeBeams;
}

int32_t
SatBhObc::GetCurrentSlotIndex() const
{
    return m_currentSlotIdx;
}

// ── Private state machine (stubs) ─────────────────────────────────────────

void
SatBhObc::EnterSlot(int32_t slotIdx)
{
    // TODO Phase 2:
    //   m_currentSlotIdx = slotIdx
    //   m_activeBeams = m_activePlan->GetSlots()[slotIdx].beamIds
    //   for each beam: m_activateCb(m_satId, beamId, T_s − T_sw)
    //   Simulator::Schedule(T_s − T_sw, OnSlotServiceEnd, slotIdx)
    NS_LOG_DEBUG("SatBhObc::EnterSlot " << slotIdx << " [STUB]");
}

void
SatBhObc::OnSlotServiceEnd(int32_t slotIdx)
{
    // TODO Phase 2:
    //   for each beam: m_deactivateCb(m_satId, beamId, usedDuration)
    //   m_state = SWITCHING
    //   Simulator::Schedule(T_sw, OnSwitchingDone, slotIdx+1)
    NS_LOG_DEBUG("SatBhObc::OnSlotServiceEnd slot=" << slotIdx << " [STUB]");
}

void
SatBhObc::OnSwitchingDone(int32_t nextSlotIdx)
{
    // TODO Phase 2:
    //   if nextSlotIdx < m_activePlan->GetNumSlots() → EnterSlot(nextSlotIdx)
    //   else if m_pendingPlan → m_activePlan = m_pendingPlan; EnterSlot(0)
    //   else → m_state = WAIT_PLAN
    NS_LOG_DEBUG("SatBhObc::OnSwitchingDone nextSlot=" << nextSlotIdx << " [STUB]");
}

} // namespace ns3
