/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-obc.h
 *
 * SatBhObc — On-Board Computer: satellite-side BHTP executor.
 *
 * Implements spec Section 7.2 (SatBhObc) and Layer2.md Section 7.2.
 * Implementation phase: Phase 2 (Layer2.md Section 10).
 *
 * Responsibilities (satellite side):
 *   - Receive new BHTP from NCC (after T_prop propagation delay)
 *   - Execute beam switching at each slot boundary per the plan
 *   - Manage the OBC state machine (IDLE / ACTIVE / SWITCHING / WAIT_PLAN)
 *   - Emit BeamSwitchCallback for each slot start/end event
 *     (consumed by SatBhMetrics and SatGwCacheQueue)
 *
 * State machine (spec Section 7.2):
 *   IDLE      → ACTIVE     : new BHTP received (ReceiveNewPlan called)
 *   ACTIVE    → SWITCHING  : current slot ends; T_sw timer starts
 *   SWITCHING → ACTIVE     : T_sw (2 ms) elapses; next slot begins
 *   ACTIVE    → WAIT_PLAN  : last slot of BHTP ends; awaiting next plan
 *   WAIT_PLAN → ACTIVE     : new BHTP received while waiting
 *
 * Timing constraints (spec Section 5.1):
 *   T_s  = 26.5 ms   slot service window
 *   T_sw = 2 ms      dead-time during beam switching (no data)
 *   Usable window = T_s − T_sw = 24.5 ms
 *
 * Callbacks emitted (connected by SatBhHelper to Metrics + CacheQueue):
 *   BeamActivateCallback  (satId, beamId, usableDuration)  ← slot start
 *   BeamDeactivateCallback(satId, beamId, actualUsed)      ← slot end
 *
 * Non-invasive constraint (Layer2.md Principle A/B):
 *   OBC does not modify SNS3 MAC internals.
 *   Beam switching is signalled via BeamActivateCallback to external modules.
 *   Hook to actual RF chain switching deferred to Hook feasibility check
 *   (Layer2.md Step 0 / Section 8).
 */

#ifndef SAT_BH_OBC_H
#define SAT_BH_OBC_H

#include "sat-bh-time-plan.h"

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <functional>

namespace ns3
{

// ── OBC state machine states ──────────────────────────────────────────────

enum class ObcState : uint8_t
{
    IDLE      = 0, ///< No BHTP loaded; OBC passive
    ACTIVE    = 1, ///< Executing current slot; beam(s) active
    SWITCHING = 2, ///< T_sw dead-time between consecutive slots
    WAIT_PLAN = 3, ///< Last slot finished; waiting for next BHTP from NCC
};

std::string ObcStateToString(ObcState s);

// ── Beam event callbacks ──────────────────────────────────────────────────
//
// Connected by SatBhHelper to:
//   SatBhMetrics::OnSlotActivated   / OnSlotDeactivated
//   SatGwCacheQueue::OnBeamActive   / OnBeamInactive
//
using BeamActivateCallback   = std::function<void(uint32_t satId, uint32_t beamId, Time usableDur)>;
using BeamDeactivateCallback = std::function<void(uint32_t satId, uint32_t beamId, Time usedDur)>;

// ── SatBhObc ─────────────────────────────────────────────────────────────

class SatBhObc : public Object
{
  public:
    static TypeId GetTypeId();
    SatBhObc();

    // ── Plan reception ────────────────────────────────────────────────────

    /// Accept a new BHTP from NCC (called after T_prop delay by SatBhScheduler
    /// via the BhPlanReadyCallback registered in SatBhHelper).
    /// Transitions: IDLE → ACTIVE  or  WAIT_PLAN → ACTIVE
    void ReceiveNewPlan(Ptr<SatBhTimePlan> plan, Time propagationDelay);

    // ── Callback registration ─────────────────────────────────────────────

    /// Register callback for slot-start event (beam activated)
    void SetBeamActivateCallback(BeamActivateCallback cb);

    /// Register callback for slot-end event (beam deactivated)
    void SetBeamDeactivateCallback(BeamDeactivateCallback cb);

    // ── Satellite identity ────────────────────────────────────────────────

    /// Set the satellite ID this OBC is attached to
    void SetSatId(uint32_t satId);
    uint32_t GetSatId() const { return m_satId; }

    // ── State query ───────────────────────────────────────────────────────

    ObcState GetState() const { return m_state; }

    /// Return currently active beam IDs (empty if IDLE / SWITCHING / WAIT_PLAN)
    const std::vector<uint32_t>& GetActiveBeams() const;

    /// Return current slot index within active plan (-1 if no plan active)
    int32_t GetCurrentSlotIndex() const;

  private:
    // ── Attribute accessor (double ms ↔ Time, required by MakeDoubleAccessor) ──
    void   SetSwitchingTimeMs(double ms) { m_switchingTime = MilliSeconds(ms); }
    double GetSwitchingTimeMs() const    { return m_switchingTime.GetMilliSeconds(); }

    // ── State machine transitions ──────────────────────────────────────────

    /// Start executing slot slotIdx: emit BeamActivateCallback for all beams,
    /// schedule slot-end event at startTime + usableDuration.
    void EnterSlot(int32_t slotIdx);

    /// Slot service window ended: emit BeamDeactivateCallback,
    /// enter SWITCHING state, schedule T_sw timer.
    void OnSlotServiceEnd(int32_t slotIdx);

    /// T_sw dead-time over: advance to next slot or WAIT_PLAN if last.
    void OnSwitchingDone(int32_t nextSlotIdx);

    // ── State ─────────────────────────────────────────────────────────────

    ObcState            m_state;            ///< Current FSM state
    uint32_t            m_satId;            ///< Satellite this OBC is attached to
    Time                m_switchingTime;    ///< T_sw (attr: SwitchingTimeMs)
    Ptr<SatBhTimePlan>  m_activePlan;       ///< Currently executing plan (nullptr if IDLE)
    Ptr<SatBhTimePlan>  m_pendingPlan;      ///< Received but not yet active plan
    int32_t             m_currentSlotIdx;   ///< Slot index currently being executed
    std::vector<uint32_t> m_activeBeams;    ///< Beams active in current slot

    BeamActivateCallback   m_activateCb;    ///< Emitted at slot start
    BeamDeactivateCallback m_deactivateCb;  ///< Emitted at slot end
};

} // namespace ns3

#endif // SAT_BH_OBC_H
