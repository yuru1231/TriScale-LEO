/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-scheduler.cc
 *
 * SatBhScheduler — STUB implementation (Phase 2, not yet active).
 *
 * All method bodies log their invocation but perform no computation.
 * Interfaces are final; only this .cc changes when Phase 2 is implemented.
 *
 * Implementation checklist (Layer2.md Step 3):
 *   [ ] EM E-step / M-step / convergence loop (RunEM)
 *   [ ] d_n integer allocation with global correction (ComputeSlotAllocation)
 *   [ ] Virtual traffic A_n with α factor (ComputeVirtualTraffic)
 *   [ ] Hotspot/non-hotspot split at 25th percentile (BuildPlan)
 *   [ ] Dynamic beam radius selection per beam (BuildPlan)
 *   [ ] K-limited slot packing with interference check (BuildPlan)
 *   [ ] Cluster grouping via ω_ij ≥ κ merge (GroupClusters)
 *   [ ] Early-trigger on demand change > 20% (OnDemandReceived)
 *   [ ] Self-perpetuating scheduling loop (ScheduleNextCycle)
 */

#include "sat-bh-scheduler.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatBhScheduler");
NS_OBJECT_ENSURE_REGISTERED(SatBhScheduler);

TypeId
SatBhScheduler::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatBhScheduler")
            .SetParent<Object>()
            .AddConstructor<SatBhScheduler>()
            // ── Attributes from spec Section 7.1 ────────────────────────
            .AddAttribute("NumBeams",
                          "Total number of beams managed by this scheduler",
                          UintegerValue(19),
                          MakeUintegerAccessor(&SatBhScheduler::m_numBeams),
                          MakeUintegerChecker<uint32_t>(1, 1000))
            .AddAttribute("MaxActiveBeams",
                          "K: maximum simultaneously active beams per slot",
                          UintegerValue(3),
                          MakeUintegerAccessor(&SatBhScheduler::m_maxActiveBeams),
                          MakeUintegerChecker<uint32_t>(1, 10))
            .AddAttribute("BhtpPeriodMs",
                          "T_p: BHTP period in milliseconds",
                          DoubleValue(503.0),
                          MakeDoubleAccessor(&SatBhScheduler::SetBhtpPeriodMs,
                                             &SatBhScheduler::GetBhtpPeriodMs),
                          MakeDoubleChecker<double>(1.0, 10000.0))
            .AddAttribute("SlotDurationMs",
                          "T_s: BH time slot duration in milliseconds (= DVB-S2X super-frame)",
                          DoubleValue(26.5),
                          MakeDoubleAccessor(&SatBhScheduler::SetSlotDurationMs,
                                             &SatBhScheduler::GetSlotDurationMs),
                          MakeDoubleChecker<double>(1.0, 1000.0))
            .AddAttribute("PropagationDelayMs",
                          "T_prop: BHTP command propagation delay NCC→OBC in milliseconds",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&SatBhScheduler::SetPropagationDelayMs,
                                             &SatBhScheduler::GetPropagationDelayMs),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddAttribute("EmMaxIterations",
                          "EM algorithm maximum iteration count",
                          UintegerValue(50),
                          MakeUintegerAccessor(&SatBhScheduler::m_emMaxIterations),
                          MakeUintegerChecker<uint32_t>(1, 1000))
            .AddAttribute("EmConvergenceEps",
                          "EM convergence threshold ε (||λ_new - λ_old||₂ < ε to stop)",
                          DoubleValue(0.001),
                          MakeDoubleAccessor(&SatBhScheduler::m_emConvergenceEps),
                          MakeDoubleChecker<double>(1e-9, 1.0))
            .AddAttribute("DemandChangeThreshold",
                          "Early-trigger: reschedule if demand changes > this fraction",
                          DoubleValue(0.20),
                          MakeDoubleAccessor(&SatBhScheduler::m_demandChangeThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("AlphaDelaySensitivity",
                          "α: delay sensitivity factor for virtual traffic A_n",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&SatBhScheduler::m_alphaDelaySensitivity),
                          MakeDoubleChecker<double>(1.0, 10.0))
            .AddAttribute("InterferenceKappa",
                          "κ: interference factor threshold for cluster merging",
                          DoubleValue(0.08),
                          MakeDoubleAccessor(&SatBhScheduler::m_interferenceKappa),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("NonHotspotPercentile",
                          "Beams with d_n below this fraction of max are non-hotspot",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(&SatBhScheduler::m_nonHotspotPercentile),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("ObservationWindowPeriods",
                          "W: number of BHTP periods in the EM observation window",
                          UintegerValue(5),
                          MakeUintegerAccessor(&SatBhScheduler::m_observationWindow),
                          MakeUintegerChecker<uint32_t>(1, 100));
    return tid;
}

SatBhScheduler::SatBhScheduler()
    : m_numBeams(19),
      m_maxActiveBeams(3),
      m_bhtpPeriod(MilliSeconds(503.0)),
      m_slotDuration(MilliSeconds(26.5)),
      m_propagationDelay(MilliSeconds(10.0)),
      m_emMaxIterations(50),
      m_emConvergenceEps(0.001),
      m_demandChangeThreshold(0.20),
      m_alphaDelaySensitivity(2.0),
      m_interferenceKappa(0.08),
      m_nonHotspotPercentile(0.25),
      m_observationWindow(5),
      m_nextPlanId(1)
{
    NS_LOG_INFO("SatBhScheduler: constructed (Phase 2 stub — not yet implemented)");
}

// ── Demand input ──────────────────────────────────────────────────────────

void
SatBhScheduler::OnDemandReceived(uint32_t beamId, uint32_t bytes)
{
    // TODO Phase 2:
    //   1. Append bytes to m_demandHistory[beamId]
    //   2. Compute demand change vs m_prevDemand[beamId]
    //   3. If change > m_demandChangeThreshold for 2 consecutive slots → RunSchedulingCycle()
    NS_LOG_DEBUG("SatBhScheduler::OnDemandReceived beam=" << beamId
                 << " bytes=" << bytes << " [STUB]");
}

// ── Scheduling control ────────────────────────────────────────────────────

void
SatBhScheduler::RunSchedulingCycle(Time now)
{
    // TODO Phase 2: RunEM() → ComputeSlotAllocation() → ComputeVirtualTraffic()
    //               → BuildPlan(now) → m_planReadyCb(plan, m_propagationDelay)
    NS_LOG_INFO("SatBhScheduler::RunSchedulingCycle t=" << now.GetSeconds()
                << "s [STUB — no plan generated]");
}

void
SatBhScheduler::ScheduleNextCycle()
{
    // TODO Phase 2: Simulator::Schedule(m_bhtpPeriod, &SatBhScheduler::RunAndReschedule, this)
    NS_LOG_INFO("SatBhScheduler::ScheduleNextCycle [STUB — not scheduling]");
}

// ── Plan output hook ──────────────────────────────────────────────────────

void
SatBhScheduler::SetPlanReadyCallback(BhPlanReadyCallback cb)
{
    m_planReadyCb = cb;
}

// ── Query ─────────────────────────────────────────────────────────────────

Ptr<SatBhTimePlan>
SatBhScheduler::GetCurrentPlan() const
{
    return m_currentPlan; // nullptr until Phase 2 implemented
}

const std::vector<uint32_t>&
SatBhScheduler::GetSlotAllocation() const
{
    return m_slotAlloc;
}

// ── Private stubs ─────────────────────────────────────────────────────────

bool
SatBhScheduler::RunEM()
{
    // TODO Phase 2: implement EM E-step / M-step / convergence loop
    return false;
}

void
SatBhScheduler::ComputeSlotAllocation()
{
    // TODO Phase 2: d_n = max(1, round(λ_n / Σλ × M × K)); global correction
}

void
SatBhScheduler::ComputeVirtualTraffic()
{
    // TODO Phase 2: A_n = Σ_p [ L_p × α × (1 + 1/T_p) ] per beam
}

Ptr<SatBhTimePlan>
SatBhScheduler::BuildPlan(Time periodStart)
{
    // TODO Phase 2: hotspot split → dynamic radius → K-limited packing → BHTP
    return nullptr;
}

double
SatBhScheduler::ComputeInterferenceFactor(uint32_t /*beamI*/, uint32_t /*beamJ*/) const
{
    // TODO Phase 2: ω_{i,j} = G_i(θ_{i→j}) / G_j(0°)
    return 0.0;
}

void
SatBhScheduler::GroupClusters(const std::vector<uint32_t>& activeBeams)
{
    // TODO Phase 2: union-find merge when ω_ij ≥ κ
    // Phase 1 stub: each beam is its own cluster
    for (uint32_t bid : activeBeams)
        m_clusterMap[bid] = bid;
}

} // namespace ns3
