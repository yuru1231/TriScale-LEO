/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-obc.cc
 *
 * SatBhObc — Phase 2 full implementation.
 *
 * Implements spec Section 7.2 (SatBhObc) and Layer2.md Section 7.2.
 *
 * State machine summary:
 *   IDLE      → ACTIVE     : ReceiveNewPlan() called; EnterSlot(0) scheduled
 *   ACTIVE    → SWITCHING  : slot service window ends (T_s − T_sw elapsed)
 *   SWITCHING → ACTIVE     : T_sw dead-time ends; next slot begins
 *   ACTIVE    → WAIT_PLAN  : last slot of BHTP ends and no pending plan queued
 *   WAIT_PLAN → ACTIVE     : ReceiveNewPlan() called while waiting
 *
 * Timing per slot (spec Section 5.1):
 *   T_s   = 26.5 ms  total slot window
 *   T_sw  = 2 ms     beam switching dead-time (no data)
 *   Usable = T_s − T_sw = 24.5 ms  (emitted as BeamActivateCallback duration)
 *
 * Callback chain (connected by SatBhHelper::SetupObc):
 *   EnterSlot        → BeamActivateCallback   → SatBhMetrics::OnSlotActivated
 *   OnSlotServiceEnd → BeamDeactivateCallback → SatBhMetrics::OnSlotDeactivated
 *   (Phase 3) BeamActivateCallback also → SatGwCacheQueue::DequeueAll
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

// ── ObcStateToString ──────────────────────────────────────────────────────────

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

// ── TypeId ────────────────────────────────────────────────────────────────────

TypeId
SatBhObc::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatBhObc")
            .SetParent<Object>()
            .AddConstructor<SatBhObc>()
            .AddAttribute("SatId",
                          "i: satellite index (formal model parameter i)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&SatBhObc::m_i),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("SwitchingTimeMs",
                          "T_sw: beam switching dead-time in milliseconds",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&SatBhObc::SetSwitchingTimeMs,
                                             &SatBhObc::GetSwitchingTimeMs),
                          MakeDoubleChecker<double>(0.0, 100.0));
    return tid;
}

// ── Constructor ───────────────────────────────────────────────────────────────

SatBhObc::SatBhObc()
    : m_state(ObcState::IDLE),
      m_i(0),
      m_switchingTime(MilliSeconds(2.0)),
      m_currentSlotIdx(-1)
{
    NS_LOG_INFO("SatBhObc: constructed (Phase 2 active)");
}

// ── Plan reception ────────────────────────────────────────────────────────────

void
SatBhObc::ReceiveNewPlan(Ptr<SatBhTimePlan> plan, Time /*propagationDelay*/)
{
    // The propagationDelay parameter is informational only; it was already
    // incorporated by SatBhScheduler::BuildPlan into plan->GetPeriodStart().
    // We schedule EnterSlot(0) at the plan's absolute period start time.

    NS_LOG_INFO("SatBhObc::ReceiveNewPlan planId=" << (plan ? plan->GetPlanId() : 0)
                << " state=" << ObcStateToString(m_state));

    if (!plan || plan->GetNumSlots() == 0)
    {
        NS_LOG_WARN("SatBhObc::ReceiveNewPlan: null or empty plan; ignored");
        return;
    }

    if (m_state == ObcState::IDLE || m_state == ObcState::WAIT_PLAN)
    {
        // Directly activate: store plan, transition to ACTIVE, schedule first slot
        m_activePlan      = plan;
        m_pendingPlan     = nullptr;
        m_state           = ObcState::ACTIVE;
        m_currentSlotIdx  = -1; // Will be updated in EnterSlot(0)

        // Compute delay until the plan's first slot starts.
        // plan->GetPeriodStart() = Simulator::Now() + T_prop (set by scheduler).
        // delay should be > 0 because the scheduler added T_prop to periodStart.
        Time now      = Simulator::Now();
        Time planStart = plan->GetPeriodStart();
        Time delay    = (planStart > now) ? (planStart - now) : Seconds(0.0);

        NS_LOG_INFO("SatBhObc: scheduling EnterSlot(0) in "
                    << delay.GetMilliSeconds() << "ms"
                    << " (planStart=" << planStart.GetSeconds() << "s)");

        Ptr<SatBhObc> self(this);
        Simulator::Schedule(delay, &SatBhObc::EnterSlot, self, 0);
    }
    else
    {
        // ACTIVE or SWITCHING: the OBC is currently executing a plan.
        // Queue the new plan; it will be activated when the current plan exhausts.
        // If a pending plan was already queued, it is replaced by the latest one.
        NS_LOG_INFO("SatBhObc: queuing plan " << plan->GetPlanId()
                    << " as pending (current state=" << ObcStateToString(m_state) << ")");
        m_pendingPlan = plan;
    }
}

// ── Callback registration ─────────────────────────────────────────────────────

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

// ── Identity ──────────────────────────────────────────────────────────────────

void
SatBhObc::SetSatId(uint32_t satId)
{
    m_i = satId;
}

// ── State query ───────────────────────────────────────────────────────────────

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

// ── State machine: EnterSlot ─────────────────────────────────────────────────

void
SatBhObc::EnterSlot(int32_t slotIdx)
{
    // Begin executing slot slotIdx of the active plan:
    //   1. Update state to ACTIVE and record active beams.
    //   2. Emit BeamActivateCallback for each beam (triggers Metrics + CacheQueue).
    //   3. Schedule OnSlotServiceEnd after the usable window (T_s − T_sw).

    if (!m_activePlan)
    {
        NS_LOG_WARN("SatBhObc::EnterSlot: no active plan (slot=" << slotIdx << ")");
        return;
    }

    if (slotIdx < 0 ||
        static_cast<uint32_t>(slotIdx) >= m_activePlan->GetNumSlots())
    {
        NS_LOG_WARN("SatBhObc::EnterSlot: invalid slotIdx=" << slotIdx
                    << " numSlots=" << m_activePlan->GetNumSlots());
        return;
    }

    m_state          = ObcState::ACTIVE;
    m_currentSlotIdx = slotIdx;

    const BhSlotEntry& slot = m_activePlan->GetSlots()[slotIdx];
    m_activeBeams            = slot.beamIds; // Copy 1-indexed beam IDs

    // Usable service window = T_s − T_sw
    // T_sw is the dead-time needed for beam steering / PA settling.
    Time usableDur = slot.duration - m_switchingTime;
    if (usableDur < Seconds(0.0))
    {
        NS_LOG_WARN("SatBhObc::EnterSlot: T_sw >= T_s; usable window is zero");
        usableDur = Seconds(0.0);
    }

    NS_LOG_INFO("SatBhObc::EnterSlot slot=" << slotIdx
                << " beams=" << slot.beamIds.size()
                << " usable=" << usableDur.GetMilliSeconds() << "ms"
                << " satId=" << m_i);

    // Notify all downstream consumers (Metrics, CacheQueue) that beams are active
    if (m_activateCb)
    {
        for (uint32_t beamId : slot.beamIds)
            m_activateCb(m_i, beamId, usableDur);
    }

    // Schedule the service window end event; OBC transitions to SWITCHING after this
    Ptr<SatBhObc> self(this);
    Simulator::Schedule(usableDur, &SatBhObc::OnSlotServiceEnd, self, slotIdx);
}

// ── State machine: OnSlotServiceEnd ─────────────────────────────────────────

void
SatBhObc::OnSlotServiceEnd(int32_t slotIdx)
{
    // Usable service window has elapsed for slot slotIdx:
    //   1. Emit BeamDeactivateCallback for each beam (Metrics records dwell time).
    //   2. Transition to SWITCHING state.
    //   3. Schedule OnSwitchingDone after T_sw (beam steering dead-time).

    // Guard against stale events (e.g., if a new plan pre-empted the current slot)
    if (slotIdx != m_currentSlotIdx)
    {
        NS_LOG_DEBUG("SatBhObc::OnSlotServiceEnd: stale event slot="
                     << slotIdx << " current=" << m_currentSlotIdx << "; ignored");
        return;
    }

    if (!m_activePlan)
    {
        NS_LOG_WARN("SatBhObc::OnSlotServiceEnd: no active plan");
        return;
    }

    const BhSlotEntry& slot  = m_activePlan->GetSlots()[slotIdx];
    Time               usedDur = slot.duration - m_switchingTime;
    if (usedDur < Seconds(0.0))
        usedDur = Seconds(0.0);

    NS_LOG_INFO("SatBhObc::OnSlotServiceEnd slot=" << slotIdx
                << " usedDur=" << usedDur.GetMilliSeconds() << "ms");

    // Emit deactivate event: Metrics records actual dwell time for KPI calculation
    if (m_deactivateCb)
    {
        for (uint32_t beamId : slot.beamIds)
            m_deactivateCb(m_i, beamId, usedDur);
    }

    // Clear active beam list (beams are inactive during T_sw)
    m_activeBeams.clear();

    // Enter SWITCHING state for the T_sw dead-time
    m_state = ObcState::SWITCHING;

    // After T_sw, advance to the next slot (or WAIT_PLAN if this was the last slot)
    Ptr<SatBhObc> self(this);
    Simulator::Schedule(m_switchingTime,
                        &SatBhObc::OnSwitchingDone, self,
                        slotIdx + 1); // nextSlotIdx = slotIdx + 1
}

// ── State machine: OnSwitchingDone ───────────────────────────────────────────

void
SatBhObc::OnSwitchingDone(int32_t nextSlotIdx)
{
    // T_sw dead-time has elapsed.  Decide next action:
    //   (a) More slots in current plan → EnterSlot(nextSlotIdx)
    //   (b) Plan exhausted AND pending plan queued → activate pending plan
    //   (c) Plan exhausted AND no pending plan → enter WAIT_PLAN

    NS_LOG_DEBUG("SatBhObc::OnSwitchingDone nextSlot=" << nextSlotIdx
                 << " state=" << ObcStateToString(m_state));

    if (m_activePlan &&
        nextSlotIdx >= 0 &&
        static_cast<uint32_t>(nextSlotIdx) < m_activePlan->GetNumSlots())
    {
        // (a) Continue with the next slot in the current BHTP
        EnterSlot(nextSlotIdx);
    }
    else if (m_pendingPlan)
    {
        // (b) Current plan exhausted; activate the queued next plan.
        // The pending plan's period start was set by the scheduler when it was
        // generated, so we schedule the first slot at the plan's absolute start time.
        NS_LOG_INFO("SatBhObc::OnSwitchingDone: activating pending plan "
                    << m_pendingPlan->GetPlanId());

        m_activePlan  = m_pendingPlan;
        m_pendingPlan = nullptr;

        Time now      = Simulator::Now();
        Time planStart = m_activePlan->GetPeriodStart();
        Time delay    = (planStart > now) ? (planStart - now) : Seconds(0.0);

        Ptr<SatBhObc> self(this);
        Simulator::Schedule(delay, &SatBhObc::EnterSlot, self, 0);
    }
    else
    {
        // (c) No more work; wait for the next BHTP from NCC
        m_state          = ObcState::WAIT_PLAN;
        m_currentSlotIdx = -1;
        m_activeBeams.clear();
        NS_LOG_INFO("SatBhObc: entering WAIT_PLAN (satId=" << m_i << ")");
    }
}

} // namespace ns3
