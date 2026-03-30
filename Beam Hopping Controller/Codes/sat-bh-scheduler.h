/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-scheduler.h
 *
 * SatBhScheduler — NCC-side demand-driven BHTP scheduler.
 *
 * Implements spec Section 7.1 (SatBhScheduler) and Layer2.md Section 7.1.
 * Implementation phase: Phase 2 (Layer2.md Section 10).
 *
 * Responsibilities (NCC side):
 *   1. Collect per-beam traffic demand via OnDemandReceived()
 *   2. Run EM algorithm to estimate required slot counts d_n per beam
 *   3. Compute virtual traffic A_n for delay-sensitive scheduling
 *   4. Perform beam scheduling (hotspot/non-hotspot split, K-limited packing)
 *   5. Group beams into interference clusters (ω_ij ≥ κ → merge)
 *   6. Output SatBhTimePlan to SatBhObc (after T_prop propagation delay)
 *
 * EM algorithm (spec Section 6.1):
 *   E-step : Q(λ|λ_old) = Σ_n Σ_t [ x_n,t × log(λ_n) − λ_n × T_s ]
 *   M-step : λ_n_new = (1 / W×M) × Σ_t x_n,t
 *   Termination: ||λ_new − λ_old||₂ < ε  OR  iterations ≥ EmMaxIterations
 *   d_n = max(1, round(λ_n / Σ λ_n × M × K)); corrected to Σd_n = M×K
 *
 * Virtual traffic (spec Section 6.2):
 *   A_{p,j} = L_{p,j} × α × (1 + 1/T_{p,j})
 *   A_j     = Σ_p A_{p,j}
 *
 * Scheduling strategy (spec Section 6.3):
 *   Phase A: non-hotspot beams (d_n ≤ 25th pct) → LARGE beam, greedy cluster
 *   Phase B: hotspot beams (sorted by A_n) → dynamic radius, K-limited packing
 *
 * Cluster grouping (spec Section 6.4):
 *   ω_{i,j} = G_i(θ_{i→j}) / G_j(0°)
 *   if ω_{i,j} ≥ κ → merge clusters
 *
 * Trigger conditions:
 *   - Periodic: every T_p (scheduled by ScheduleNextCycle)
 *   - Early: demand change > DemandChangeThreshold over 2 consecutive slots
 *
 * Non-invasive constraint (Layer2.md Principle A/B):
 *   All demand data arrives via OnDemandReceived() callback.
 *   No direct calls to SNS3 internal scheduler or MAC.
 */

#ifndef SAT_BH_SCHEDULER_H
#define SAT_BH_SCHEDULER_H

#include "sat-bh-time-plan.h"

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace ns3
{

// ── Callback: called by Scheduler when a new BHTP is ready ───────────────
//
// Signature: (newPlan, propagationDelay)
// SatBhHelper connects this to SatBhObc::ReceiveNewPlan().
//
using BhPlanReadyCallback =
    std::function<void(Ptr<SatBhTimePlan> plan, Time propagationDelay)>;

// ── SatBhScheduler ────────────────────────────────────────────────────────

class SatBhScheduler : public Object
{
  public:
    static TypeId GetTypeId();
    SatBhScheduler();

    // ── Demand input (trace sink, called by SatBhHelper) ─────────────────

    /// Called each time a DA-Request or MAC Rx event reports new demand.
    /// @param beamId  Beam index (0-based)
    /// @param bytes   Bytes demanded in the current observation window
    void OnDemandReceived(uint32_t beamId, uint32_t bytes);

    // ── Scheduling control ────────────────────────────────────────────────

    /// Run one full scheduling cycle: EM → virtual traffic → plan generation.
    /// Called internally by the periodic timer and on early-trigger.
    void RunSchedulingCycle(Time now);

    /// Schedule the next RunSchedulingCycle() call at now + T_p.
    /// Called once by SatBhHelper after Install(); self-perpetuating thereafter.
    void ScheduleNextCycle();

    // ── Plan output hook ──────────────────────────────────────────────────

    /// Register the callback invoked when a new BHTP is ready.
    /// SatBhHelper connects this to SatBhObc::ReceiveNewPlan().
    void SetPlanReadyCallback(BhPlanReadyCallback cb);

    // ── Query ─────────────────────────────────────────────────────────────

    /// Return the most recently generated plan (nullptr if no cycle run yet)
    Ptr<SatBhTimePlan> GetCurrentPlan() const;

    /// Return latest d_n vector (slot count per beam), for diagnostics
    const std::vector<uint32_t>& GetSlotAllocation() const;

  private:
    // ── EM algorithm (spec Section 6.1) ──────────────────────────────────

    /// Run EM iterations; updates m_lambda from m_demandHistory.
    /// Returns true if converged within EmMaxIterations.
    bool RunEM();

    /// Convert EM output λ_n to integer slot counts d_n.
    /// Applies global correction to ensure Σ d_n = M × K.
    void ComputeSlotAllocation();

    // ── Virtual traffic (spec Section 6.2) ───────────────────────────────

    /// Compute virtual traffic A_n for each beam from m_demandHistory.
    void ComputeVirtualTraffic();

    // ── Scheduling (spec Section 6.3) ────────────────────────────────────

    /// Build a SatBhTimePlan from d_n and A_n.
    /// Applies hotspot/non-hotspot split and K-limited slot packing.
    Ptr<SatBhTimePlan> BuildPlan(Time periodStart);

    // ── Cluster grouping (spec Section 6.4) ──────────────────────────────

    /// Compute interference factor ω_{i,j} for a beam pair.
    double ComputeInterferenceFactor(uint32_t beamI, uint32_t beamJ) const;

    /// Group beams into clusters; fills m_clusterMap.
    void GroupClusters(const std::vector<uint32_t>& activeBeams);

    // ── Attribute accessors (double ms ↔ Time, required by MakeDoubleAccessor) ──
    //
    // ns-3 MakeDoubleAccessor only accepts member function pointers, not lambdas.
    // These wrappers convert the double-ms attribute value to/from the Time member.
    // Pattern mirrors SatBhMetrics::SetReportIntervalMs / GetReportIntervalMs.

    void   SetBhtpPeriodMs(double ms)       { m_bhtpPeriod = MilliSeconds(ms); }
    double GetBhtpPeriodMs() const          { return m_bhtpPeriod.GetMilliSeconds(); }

    void   SetSlotDurationMs(double ms)     { m_slotDuration = MilliSeconds(ms); }
    double GetSlotDurationMs() const        { return m_slotDuration.GetMilliSeconds(); }

    void   SetPropagationDelayMs(double ms) { m_propagationDelay = MilliSeconds(ms); }
    double GetPropagationDelayMs() const    { return m_propagationDelay.GetMilliSeconds(); }

    // ── State ─────────────────────────────────────────────────────────────

    uint32_t m_numBeams;                ///< Total beam count (attr: NumBeams)
    uint32_t m_maxActiveBeams;          ///< K: max simultaneous beams (attr: MaxActiveBeams)
    Time     m_bhtpPeriod;             ///< T_p (attr: BhtpPeriod)
    Time     m_slotDuration;           ///< T_s (attr: SlotDuration)
    Time     m_propagationDelay;       ///< T_prop (attr: PropagationDelay)

    uint32_t m_emMaxIterations;        ///< EM iteration cap (attr: EmMaxIterations)
    double   m_emConvergenceEps;       ///< EM convergence threshold ε (attr: EmConvergenceEps)
    double   m_demandChangeThreshold;  ///< Early-trigger threshold (attr: DemandChangeThreshold)
    double   m_alphaDelaySensitivity;  ///< α for virtual traffic (attr: AlphaDelaySensitivity)
    double   m_interferenceKappa;      ///< κ for cluster grouping (attr: InterferenceKappa)
    double   m_nonHotspotPercentile;   ///< Hotspot split threshold (attr: NonHotspotPercentile)
    uint32_t m_observationWindow;      ///< W: EM window in periods (attr: ObservationWindowPeriods)

    /// Demand observation history: m_demandHistory[beamId] = bytes observed in each slot
    /// Outer index = beam, inner index = slot within observation window
    std::vector<std::vector<double>> m_demandHistory;

    /// Previous window demand per beam (for early-trigger detection)
    std::vector<double> m_prevDemand;

    /// EM output: Poisson rate estimates λ_n per beam
    std::vector<double> m_lambda;

    /// Slot allocation d_n (output of ComputeSlotAllocation)
    std::vector<uint32_t> m_slotAlloc;

    /// Virtual traffic A_n per beam (output of ComputeVirtualTraffic)
    std::vector<double> m_virtualTraffic;

    /// Cluster assignment: beamId → clusterId
    std::map<uint32_t, uint32_t> m_clusterMap;

    /// Monotonically increasing plan ID
    uint32_t m_nextPlanId;

    /// Most recently generated plan
    Ptr<SatBhTimePlan> m_currentPlan;

    /// Callback invoked after each successful scheduling cycle
    BhPlanReadyCallback m_planReadyCb;
};

} // namespace ns3

#endif // SAT_BH_SCHEDULER_H
