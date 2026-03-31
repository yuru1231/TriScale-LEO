/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-scheduler.cc
 *
 * SatBhScheduler — Phase 2 full implementation.
 *
 * Implements spec Section 7.1 (SatBhScheduler) and Layer2.md Section 7.1.
 *
 * Pipeline per scheduling cycle:
 *   OnDemandReceived() → m_demandHistory
 *   RunSchedulingCycle()
 *     → RunEM()               : estimate λ_n (Poisson demand rates)
 *     → ComputeSlotAllocation(): d_n = max(1, round(λ_n/Σλ × M × K))
 *     → ComputeVirtualTraffic(): A_n = demand_n × α × (1 + 1/W)
 *     → BuildPlan()           : hotspot split → radius → cluster → slot packing
 *     → m_planReadyCb(plan, T_prop) → OBC::ReceiveNewPlan
 *   ScheduleNextCycle() → Simulator::Schedule(T_p, RunAndReschedule)
 *
 * Interference model:
 *   Gaussian beam: G_i(d) = G_max_i × exp(-2.77 × (d / r_i)²)
 *   ω_{i,j} = G_i(d_ij) / G_j(0) ≈ exp(-2.77 × (d_ij / r_middle)²)
 *   Beams where ω_{i,j} ≥ κ are merged into the same cluster.
 *
 * Beam layout:
 *   Hexagonal grid with 70 km inter-beam spacing (suitable for MIDDLE r=40 km).
 *   Ring 0: 1 beam at origin; Ring 1: 6 beams; Ring 2: 12 beams (total 19).
 */

#include "sat-bh-scheduler.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <numeric>

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
    NS_LOG_INFO("SatBhScheduler: constructed (Phase 2 active)");
}

// ── Private helpers ──────────────────────────────────────────────────────────

uint32_t
SatBhScheduler::ComputeM() const
{
    // M = number of BH time slots per BHTP period = round(T_p / T_s)
    // For default params: round(503 / 26.5) = round(18.98) = 19
    return static_cast<uint32_t>(
        std::round(m_bhtpPeriod.GetMilliSeconds() / m_slotDuration.GetMilliSeconds()));
}

void
SatBhScheduler::InitBeamPositions()
{
    // Pre-compute hex-grid ground positions (x, y) in km for numBeams beams.
    // Hexagonal lattice with pointy-top orientation, inter-beam spacing d = 70 km.
    // This spacing ensures adjacent MIDDLE beams (r=40 km) have moderate overlap
    // without excessive interference (ω ≈ 0.02 for adjacent beams at 70 km).
    //
    // Ring 0: 1 beam (center)
    // Ring 1: 6 beams at distance d, angles 0°, 60°, 120°, 180°, 240°, 300°
    // Ring 2: 12 beams at distances 2d (corners) and d√3 (edges)
    // Ring 3: 18 beams for numBeams > 19 (simple circular layout)

    const double d     = 70.0;              // inter-beam spacing in km
    const double sqrt3 = std::sqrt(3.0);
    const double pi    = std::acos(-1.0);

    m_beamPositions.resize(m_numBeams, {0.0, 0.0});

    if (m_numBeams == 0)
        return;

    // Ring 0: center beam (index 0 → beam 1 in SNS3)
    m_beamPositions[0] = {0.0, 0.0};

    // Ring 1: 6 beams at angles k×60°, k=0..5 (indices 1..6 → beams 2..7)
    for (uint32_t k = 0; k < 6 && (k + 1) < m_numBeams; k++)
    {
        double angle         = k * 60.0 * pi / 180.0;
        m_beamPositions[k + 1] = {d * std::cos(angle), d * std::sin(angle)};
    }

    // Ring 2: 12 beams forming the second hexagonal shell (indices 7..18 → beams 8..19)
    // Positions from hex-grid arithmetic: 6 corner beams at distance 2d,
    // 6 edge beams at distance d√3, alternating at 60°/30° offsets
    if (m_numBeams > 7)
    {
        // 12 ring-2 positions in counter-clockwise order
        const double ring2[][2] = {
            { 2.0 * d,          0.0         },
            { 1.5 * d,          d * sqrt3 / 2.0 },
            { d,                d * sqrt3   },
            { 0.0,              d * sqrt3   },
            {-d,                d * sqrt3   },
            {-1.5 * d,          d * sqrt3 / 2.0 },
            {-2.0 * d,          0.0         },
            {-1.5 * d,         -d * sqrt3 / 2.0 },
            {-d,               -d * sqrt3   },
            { 0.0,             -d * sqrt3   },
            { d,               -d * sqrt3   },
            { 1.5 * d,         -d * sqrt3 / 2.0 },
        };
        for (uint32_t k = 0; k < 12 && (k + 7) < m_numBeams; k++)
            m_beamPositions[k + 7] = {ring2[k][0], ring2[k][1]};
    }

    // Ring 3: simple circular layout for any remaining beams beyond 19
    for (uint32_t n = 19; n < m_numBeams; n++)
    {
        double angle      = (n - 19) * (2.0 * pi / 18.0);
        m_beamPositions[n] = {3.0 * d * std::cos(angle), 3.0 * d * std::sin(angle)};
    }

    NS_LOG_INFO("SatBhScheduler::InitBeamPositions: " << m_numBeams
                << " beams positioned on hex grid (spacing=" << d << " km)");
}

// ── Demand input ──────────────────────────────────────────────────────────────

void
SatBhScheduler::OnDemandReceived(uint32_t beamId, uint32_t bytes)
{
    // Lazy-initialise per-beam data structures on first demand event.
    // This avoids construction-time dependency on m_numBeams (which may
    // be set later via SetAttribute before Install() is called).
    if (m_demandHistory.empty())
    {
        m_demandHistory.assign(m_numBeams, std::vector<double>());
        m_prevDemand.assign(m_numBeams, 0.0);
        m_lambda.assign(m_numBeams, 1.0);
        m_slotAlloc.assign(m_numBeams, 1);
        m_virtualTraffic.assign(m_numBeams, 0.0);
        m_consecutiveChanges.assign(m_numBeams, 0);
    }

    if (beamId >= m_numBeams)
    {
        NS_LOG_WARN("SatBhScheduler::OnDemandReceived: beamId="
                    << beamId << " >= numBeams=" << m_numBeams << "; ignored");
        return;
    }

    // Append current observation to history ring buffer
    m_demandHistory[beamId].push_back(static_cast<double>(bytes));

    // Enforce observation window limit: keep at most W × M entries per beam
    uint32_t maxHistory = m_observationWindow * ComputeM();
    while (m_demandHistory[beamId].size() > maxHistory)
        m_demandHistory[beamId].erase(m_demandHistory[beamId].begin());

    // ── Early-trigger detection (spec Section 6.1) ────────────────────────
    // If demand changes by > DemandChangeThreshold for 2 consecutive observations,
    // immediately run a new scheduling cycle rather than waiting for T_p.
    double prev = m_prevDemand[beamId];
    double curr = static_cast<double>(bytes);
    if (prev > 0.0)
    {
        double changeRatio = std::abs(curr - prev) / prev;
        if (changeRatio > m_demandChangeThreshold)
        {
            m_consecutiveChanges[beamId]++;
            if (m_consecutiveChanges[beamId] >= 2)
            {
                NS_LOG_INFO("SatBhScheduler::OnDemandReceived: early-trigger beam="
                            << beamId << " change=" << changeRatio);
                m_consecutiveChanges[beamId] = 0; // Reset; avoid repeated triggers
                RunSchedulingCycle(Simulator::Now());
            }
        }
        else
        {
            // Change within threshold; reset consecutive counter
            m_consecutiveChanges[beamId] = 0;
        }
    }
    m_prevDemand[beamId] = curr;

    NS_LOG_DEBUG("SatBhScheduler::OnDemandReceived beam=" << beamId
                 << " bytes=" << bytes
                 << " historySize=" << m_demandHistory[beamId].size());
}

// ── Scheduling control ────────────────────────────────────────────────────────

void
SatBhScheduler::RunSchedulingCycle(Time now)
{
    NS_LOG_INFO("SatBhScheduler::RunSchedulingCycle t=" << now.GetSeconds() << "s");

    // Ensure data structures are initialised even if no demand has arrived yet
    if (m_demandHistory.empty())
    {
        m_demandHistory.assign(m_numBeams, std::vector<double>());
        m_prevDemand.assign(m_numBeams, 0.0);
        m_lambda.assign(m_numBeams, 1.0);
        m_slotAlloc.assign(m_numBeams, 1);
        m_virtualTraffic.assign(m_numBeams, 0.0);
        m_consecutiveChanges.assign(m_numBeams, 0);
    }

    // Initialise beam positions if not yet done
    if (m_beamPositions.empty())
        InitBeamPositions();

    // ── EM → slot allocation ───────────────────────────────────────────────
    bool emConverged = RunEM();
    NS_LOG_INFO("SatBhScheduler: EM "
                << (emConverged ? "converged" : "hit iteration cap"));
    ComputeSlotAllocation();

    // ── Virtual traffic ────────────────────────────────────────────────────
    ComputeVirtualTraffic();

    // ── Build BHTP ─────────────────────────────────────────────────────────
    // Plan period starts at now + T_prop (the plan takes effect on the OBC
    // after the propagation delay; the OBC schedules slot events at absolute
    // time = plan->GetPeriodStart()).
    Time periodStart = now + m_propagationDelay;
    Ptr<SatBhTimePlan> plan = BuildPlan(periodStart);

    if (!plan || plan->GetNumSlots() == 0)
    {
        NS_LOG_WARN("SatBhScheduler::RunSchedulingCycle: BuildPlan returned empty plan;"
                    " cycle skipped");
        return;
    }

    m_currentPlan = plan;

    // Validate plan against structural invariants
    if (!plan->Validate(m_maxActiveBeams))
        NS_LOG_WARN("SatBhScheduler: generated plan " << plan->GetPlanId()
                    << " failed validation");

    // Deliver plan to OBC via registered callback.
    // The callback is wired by SatBhHelper::SetupScheduler →
    //   SatBhObc::ReceiveNewPlan(plan, propagationDelay).
    // The OBC schedules actual slot events at plan->GetPeriodStart().
    if (m_planReadyCb)
    {
        NS_LOG_INFO("SatBhScheduler: emitting plan " << plan->GetPlanId()
                    << " slots=" << plan->GetNumSlots()
                    << " T_prop=" << m_propagationDelay.GetMilliSeconds() << "ms");
        m_planReadyCb(plan, m_propagationDelay);
    }
    else
    {
        NS_LOG_WARN("SatBhScheduler: no plan-ready callback registered;"
                    " plan " << plan->GetPlanId() << " not delivered");
    }
}

void
SatBhScheduler::ScheduleNextCycle()
{
    // Schedule RunAndReschedule() after one BHTP period.
    // RunAndReschedule calls RunSchedulingCycle then re-calls ScheduleNextCycle,
    // creating a self-perpetuating loop (spec Section 7.1 / Layer2.md Step 3).
    Ptr<SatBhScheduler> self(this);
    Simulator::Schedule(m_bhtpPeriod, &SatBhScheduler::RunAndReschedule, self);
    NS_LOG_DEBUG("SatBhScheduler::ScheduleNextCycle: next cycle in "
                 << m_bhtpPeriod.GetMilliSeconds() << "ms");
}

void
SatBhScheduler::RunAndReschedule()
{
    // Called by Simulator; runs scheduling cycle then re-arms the timer.
    RunSchedulingCycle(Simulator::Now());
    ScheduleNextCycle();
}

// ── Plan output hook ──────────────────────────────────────────────────────────

void
SatBhScheduler::SetPlanReadyCallback(BhPlanReadyCallback cb)
{
    m_planReadyCb = cb;
}

// ── Query ─────────────────────────────────────────────────────────────────────

Ptr<SatBhTimePlan>
SatBhScheduler::GetCurrentPlan() const
{
    return m_currentPlan;
}

const std::vector<uint32_t>&
SatBhScheduler::GetSlotAllocation() const
{
    return m_slotAlloc;
}

// ── EM algorithm (spec Section 6.1) ──────────────────────────────────────────

bool
SatBhScheduler::RunEM()
{
    // Poisson EM for multi-beam traffic estimation.
    //
    // Model: demand from beam n in slot t follows Poisson(λ_n × T_s).
    // Given observed demand x_{n,t} across N beams × (W×M) slots:
    //   E-step: Q(λ|λ_old) = Σ_n Σ_t [ x_{n,t} × log(λ_n) − λ_n × T_s ]
    //   M-step: λ_n_new = (1/(W×M)) × Σ_t x_{n,t}   ← sample mean per beam per slot
    //   Stop:   ||λ_new − λ_old||₂ < ε  OR  iterations ≥ EmMaxIterations
    //
    // Note: the M-step formula is the global MLE regardless of λ_old, so
    // convergence typically occurs in one iteration. The loop exists for
    // model extensions (e.g., mixture models) and for consistency with spec.

    uint32_t M         = ComputeM();
    double   windowSz  = static_cast<double>(m_observationWindow * M);

    bool converged = false;

    for (uint32_t iter = 0; iter < m_emMaxIterations && !converged; iter++)
    {
        std::vector<double> lambdaNew(m_numBeams, 0.0);

        // M-step: λ_n_new = (1 / W×M) × Σ_t x_{n,t}
        for (uint32_t n = 0; n < m_numBeams; n++)
        {
            if (m_demandHistory[n].empty())
            {
                // No demand data yet; keep a nominal positive rate (1 byte/slot)
                lambdaNew[n] = 1.0;
                continue;
            }
            double sum = 0.0;
            for (double x : m_demandHistory[n])
                sum += x;
            // Normalise by full window size even if history is shorter,
            // to avoid over-estimating rate when buffer is still filling up.
            lambdaNew[n] = std::max(1.0, sum / windowSz);
        }

        // Convergence check: ||λ_new − λ_old||₂ < ε
        double norm2 = 0.0;
        for (uint32_t n = 0; n < m_numBeams; n++)
        {
            double diff = lambdaNew[n] - m_lambda[n];
            norm2 += diff * diff;
        }
        m_lambda  = lambdaNew;
        converged = (std::sqrt(norm2) < m_emConvergenceEps);

        NS_LOG_DEBUG("SatBhScheduler::RunEM iter=" << iter
                     << " norm=" << std::sqrt(norm2));
    }

    return converged;
}

// ── Slot allocation (spec Section 6.1 d_n conversion) ───────────────────────

void
SatBhScheduler::ComputeSlotAllocation()
{
    // d_n = max(1, round(λ_n / Σλ × M × K))
    // After rounding, apply global correction to ensure Σ d_n = M × K exactly.
    // Correction adjusts the beam(s) with the highest/lowest allocation first.

    uint32_t M     = ComputeM();
    uint32_t K     = m_maxActiveBeams;
    uint32_t total = M * K; // Target: Σ d_n must equal this

    m_slotAlloc.resize(m_numBeams, 1);

    // Normalisation denominator
    double lambdaSum = 0.0;
    for (uint32_t n = 0; n < m_numBeams; n++)
        lambdaSum += m_lambda[n];
    if (lambdaSum <= 0.0)
        lambdaSum = 1.0;

    // Initial proportional allocation with floor at 1
    int32_t allocated = 0;
    for (uint32_t n = 0; n < m_numBeams; n++)
    {
        double frac = m_lambda[n] / lambdaSum * static_cast<double>(total);
        m_slotAlloc[n] = std::max(1u, static_cast<uint32_t>(std::round(frac)));
        allocated += static_cast<int32_t>(m_slotAlloc[n]);
    }

    // Global correction: reduce high-allocation beams if surplus, increase low ones if deficit
    int32_t surplus = allocated - static_cast<int32_t>(total);

    while (surplus > 0)
    {
        // Reduce the beam with the largest d_n (that is still > 1)
        uint32_t maxIdx = 0;
        for (uint32_t n = 1; n < m_numBeams; n++)
            if (m_slotAlloc[n] > m_slotAlloc[maxIdx])
                maxIdx = n;
        if (m_slotAlloc[maxIdx] <= 1)
            break; // Cannot reduce further; accept residual surplus
        m_slotAlloc[maxIdx]--;
        surplus--;
    }

    while (surplus < 0)
    {
        // Increase the beam with the smallest d_n
        uint32_t minIdx = 0;
        for (uint32_t n = 1; n < m_numBeams; n++)
            if (m_slotAlloc[n] < m_slotAlloc[minIdx])
                minIdx = n;
        m_slotAlloc[minIdx]++;
        surplus++;
    }

    NS_LOG_INFO("SatBhScheduler::ComputeSlotAllocation: M=" << M
                << " K=" << K << " total=" << total
                << " allocated=" << (allocated - surplus));
    for (uint32_t n = 0; n < m_numBeams; n++)
        NS_LOG_DEBUG("  beam[" << n << "] λ=" << m_lambda[n]
                     << " d_n=" << m_slotAlloc[n]);
}

// ── Virtual traffic (spec Section 6.2) ──────────────────────────────────────

void
SatBhScheduler::ComputeVirtualTraffic()
{
    // A_j = Σ_p [ L_{p,j} × α × (1 + 1/T_{p,j}) ]
    //
    // Without packet-level tracking, we approximate:
    //   ΣL_{p,j} ≈ demand_j (most recent period demand in bytes)
    //   T_{p,j}  ≈ W (average remaining TTL = observation window depth)
    //
    // This gives: A_j = demand_j × α × (1 + 1/W)
    // The relative ordering of A_j values across beams matches the spec intent:
    //   beams with high recent demand get higher virtual traffic.

    m_virtualTraffic.resize(m_numBeams, 0.0);
    double urgency = 1.0 + 1.0 / static_cast<double>(m_observationWindow);

    for (uint32_t n = 0; n < m_numBeams; n++)
    {
        // Use most recent observation as the "current period demand"
        double demand = 0.0;
        if (!m_demandHistory[n].empty())
            demand = m_demandHistory[n].back();

        m_virtualTraffic[n] = demand * m_alphaDelaySensitivity * urgency;
    }

    NS_LOG_DEBUG("SatBhScheduler::ComputeVirtualTraffic: α=" << m_alphaDelaySensitivity
                 << " urgency=" << urgency);
}

// ── Interference model (spec Section 6.4) ───────────────────────────────────

double
SatBhScheduler::ComputeInterferenceFactor(uint32_t beamI, uint32_t beamJ) const
{
    // ω_{i,j} = G_i(θ_{i→j}) / G_j(0°)
    //
    // Using Gaussian beam gain model:
    //   G_i(d) = G_max_i × exp(-2.77 × (d / r_i)²)
    //   G_j(0) = G_max_j  (peak gain toward beam j centre)
    //
    // Simplification: assume MIDDLE beam radius (40 km) as reference for both beams.
    // This is a conservative estimate; narrower beams would have lower ω_{i,j}.
    //
    // First-layer check: if d_ij > 1.5 × (r_i + r_j), interference is negligible.
    // (spec Section 6.4 first-layer condition)

    if (beamI >= m_beamPositions.size() || beamJ >= m_beamPositions.size())
        return 0.0; // Positions not initialised; assume no interference

    const double rMiddle    = 40.0; // km — MIDDLE beam 3dB radius (spec Section 4.2)
    const double firstLayer = 1.5 * (rMiddle + rMiddle); // = 120 km

    // Euclidean distance between beam centre positions in km
    double dx  = m_beamPositions[beamI].first  - m_beamPositions[beamJ].first;
    double dy  = m_beamPositions[beamI].second - m_beamPositions[beamJ].second;
    double dij = std::sqrt(dx * dx + dy * dy);

    // First-layer spatial isolation check
    if (dij > firstLayer)
        return 0.0;

    // Gaussian gain model: ω_{i,j} = exp(-2.77 × (d_ij / r)²)
    // Factor 2.77 ≈ ln(2) × 4 comes from the 3dB half-power beamwidth definition.
    double omega = std::exp(-2.77 * (dij / rMiddle) * (dij / rMiddle));
    return omega;
}

// ── Cluster grouping (spec Section 6.4) ─────────────────────────────────────

void
SatBhScheduler::GroupClusters(const std::vector<uint32_t>& activeBeams)
{
    // Union-Find (iterative merge) for interference cluster formation.
    // Two beams i, j are merged into the same cluster if ω_{i,j} ≥ κ.
    // Repeat until no further merges occur (transitive closure).
    //
    // Initial state: each beam is its own cluster (cluster ID = beam ID).
    // After merging: beams in the same cluster share the lowest beam ID as cluster ID.

    // Initialise: each beam is its own cluster
    for (uint32_t bid : activeBeams)
        m_clusterMap[bid] = bid;

    // Iteratively merge until stable
    bool merged;
    do
    {
        merged = false;
        for (uint32_t i = 0; i < activeBeams.size(); i++)
        {
            for (uint32_t j = i + 1; j < activeBeams.size(); j++)
            {
                uint32_t bi = activeBeams[i];
                uint32_t bj = activeBeams[j];

                if (m_clusterMap[bi] == m_clusterMap[bj])
                    continue; // Already in same cluster; no action needed

                double omega = ComputeInterferenceFactor(bi, bj);
                if (omega >= m_interferenceKappa)
                {
                    // Merge cluster of bj into cluster of bi
                    uint32_t oldCluster = m_clusterMap[bj];
                    uint32_t newCluster = m_clusterMap[bi];
                    for (uint32_t bk : activeBeams)
                    {
                        if (m_clusterMap[bk] == oldCluster)
                            m_clusterMap[bk] = newCluster;
                    }
                    merged = true;
                    NS_LOG_DEBUG("SatBhScheduler::GroupClusters: merged beam "
                                 << bj << " → cluster " << newCluster
                                 << " (ω=" << omega << " ≥ κ=" << m_interferenceKappa << ")");
                }
            }
        }
    } while (merged);

    // Count distinct clusters for diagnostic logging
    std::map<uint32_t, uint32_t> clusterSizes;
    for (uint32_t bid : activeBeams)
        clusterSizes[m_clusterMap[bid]]++;
    NS_LOG_INFO("SatBhScheduler::GroupClusters: " << clusterSizes.size()
                << " clusters for " << activeBeams.size() << " beams");
}

// ── Plan builder (spec Sections 6.3, 6.4) ────────────────────────────────────

Ptr<SatBhTimePlan>
SatBhScheduler::BuildPlan(Time periodStart)
{
    Ptr<SatBhTimePlan> plan = CreateObject<SatBhTimePlan>();
    plan->SetPlanId(m_nextPlanId++);

    uint32_t M = ComputeM();
    plan->SetPeriodBounds(periodStart, periodStart + m_bhtpPeriod);

    // ── Step 1: categorise beams into hotspot / non-hotspot ───────────────
    // Non-hotspot: d_n ≤ 25th-percentile threshold (spec Section 6.3 Phase A)
    // Hotspot:     d_n >  25th-percentile threshold (spec Section 6.3 Phase B)

    std::vector<uint32_t> sortedAlloc(m_slotAlloc.begin(),
                                      m_slotAlloc.begin() + m_numBeams);
    std::sort(sortedAlloc.begin(), sortedAlloc.end());
    uint32_t p25Idx       = static_cast<uint32_t>(m_nonHotspotPercentile *
                                                  (m_numBeams > 1 ? m_numBeams - 1 : 0));
    uint32_t p25Threshold = sortedAlloc[p25Idx];

    // Collect hotspot beams sorted by virtual traffic (highest first)
    std::vector<uint32_t> hotspotBeams, nonHotspotBeams;
    for (uint32_t n = 0; n < m_numBeams; n++)
    {
        if (m_slotAlloc[n] <= p25Threshold)
            nonHotspotBeams.push_back(n);
        else
            hotspotBeams.push_back(n);
    }
    std::sort(hotspotBeams.begin(), hotspotBeams.end(),
              [this](uint32_t a, uint32_t b) {
                  return m_virtualTraffic[a] > m_virtualTraffic[b];
              });

    // ── Step 2: assign beam radii (spec Section 6.3 Phase A/B) ──────────
    // Non-hotspot → LARGE (wide coverage, lower gain)
    // Top 1/3 hotspot by virtual traffic → SMALL (concentrated gain, high SINR)
    // Remaining hotspot → MIDDLE (general purpose)
    std::vector<BeamRadiusType> beamRadius(m_numBeams, BeamRadiusType::MIDDLE);

    for (uint32_t n : nonHotspotBeams)
        beamRadius[n] = BeamRadiusType::LARGE;

    if (!hotspotBeams.empty())
    {
        uint32_t smallCount =
            std::max(1u, static_cast<uint32_t>(hotspotBeams.size() / 3));
        for (uint32_t i = 0; i < smallCount; i++)
            beamRadius[hotspotBeams[i]] = BeamRadiusType::SMALL;
    }

    // ── Step 3: interference cluster grouping (spec Section 6.4) ─────────
    // All numBeams beams are grouped; cluster IDs stored in m_clusterMap.
    std::vector<uint32_t> allBeams(m_numBeams);
    std::iota(allBeams.begin(), allBeams.end(), 0); // {0, 1, ..., N-1}
    GroupClusters(allBeams);

    // ── Step 4: K-limited greedy slot assignment ───────────────────────────
    // slotBeams[s] = 0-indexed beam IDs assigned to slot s.
    // Assignment order: non-hotspot first (large-beam coverage priority),
    // then hotspot by virtual traffic (high-demand served first).
    // Within each slot, at most K beams, no interfering beam pair (ω ≥ κ).

    std::vector<std::vector<uint32_t>> slotBeams(M);

    std::vector<uint32_t> beamOrder;
    beamOrder.insert(beamOrder.end(), nonHotspotBeams.begin(), nonHotspotBeams.end());
    beamOrder.insert(beamOrder.end(), hotspotBeams.begin(), hotspotBeams.end());

    // Track remaining slot budget per beam (initialised from d_n)
    std::vector<int32_t> remaining(m_numBeams);
    for (uint32_t n = 0; n < m_numBeams; n++)
        remaining[n] = static_cast<int32_t>(m_slotAlloc[n]);

    for (uint32_t n : beamOrder)
    {
        while (remaining[n] > 0)
        {
            // Find the slot with the fewest beams that:
            //   (a) still has capacity (< K beams),
            //   (b) does not already contain beam n,
            //   (c) contains no beam that interferes with n (ω ≥ κ).
            int32_t  chosen  = -1;
            uint32_t minSize = m_maxActiveBeams + 1;

            for (uint32_t s = 0; s < M; s++)
            {
                // Capacity check
                if (static_cast<uint32_t>(slotBeams[s].size()) >= m_maxActiveBeams)
                    continue;

                // Duplicate check: beam must not appear twice in the same slot
                bool duplicate = false;
                for (uint32_t b : slotBeams[s])
                    if (b == n)
                    {
                        duplicate = true;
                        break;
                    }
                if (duplicate)
                    continue;

                // Interference check: avoid co-scheduling beams in the same cluster
                // (they would require MMSE precoding, which is Phase 3)
                bool interferenceOk = true;
                for (uint32_t b : slotBeams[s])
                {
                    if (ComputeInterferenceFactor(n, b) >= m_interferenceKappa)
                    {
                        interferenceOk = false;
                        break;
                    }
                }

                // Prefer the slot with the fewest current assignments (load balancing)
                if (interferenceOk &&
                    static_cast<uint32_t>(slotBeams[s].size()) < minSize)
                {
                    chosen  = static_cast<int32_t>(s);
                    minSize = static_cast<uint32_t>(slotBeams[s].size());
                }
            }

            // Fallback: if no interference-free slot found, accept any under-capacity slot.
            // This can occur when κ is very small and all slots contain interfering beams.
            // The resulting co-scheduled beams will produce a note in the log.
            if (chosen < 0)
            {
                for (uint32_t s = 0; s < M; s++)
                {
                    if (static_cast<uint32_t>(slotBeams[s].size()) < m_maxActiveBeams)
                    {
                        bool dup = false;
                        for (uint32_t b : slotBeams[s])
                            if (b == n)
                            {
                                dup = true;
                                break;
                            }
                        if (!dup)
                        {
                            chosen = static_cast<int32_t>(s);
                            NS_LOG_WARN("SatBhScheduler::BuildPlan: beam "
                                        << n << " placed in slot " << s
                                        << " with potential interference (no clean slot)");
                            break;
                        }
                    }
                }
            }

            if (chosen >= 0)
            {
                slotBeams[chosen].push_back(n);
                remaining[n]--;
            }
            else
            {
                // All M slots full or already contain this beam; cannot assign
                NS_LOG_WARN("SatBhScheduler::BuildPlan: beam " << n
                            << " could not be assigned a slot; remaining="
                            << remaining[n]);
                break;
            }
        }
    }

    // ── Step 5: convert slot assignments to BhSlotEntry objects ──────────
    // Slots are added in ascending startTime order (asserted by SatBhTimePlan::AddSlot).
    // Empty slots (no beams assigned) are skipped to avoid zero-beam entries.

    for (uint32_t s = 0; s < M; s++)
    {
        if (slotBeams[s].empty())
            continue;

        BhSlotEntry slot;
        slot.startTime = s * m_slotDuration;   // Offset from period start
        slot.duration  = m_slotDuration;        // T_s = 26.5 ms by default

        // Determine slot beam radius: use the most restrictive (smallest = highest gain)
        // among all beams in this slot.  SMALL (0) < MIDDLE (1) < LARGE (2).
        slot.beamRadius = BeamRadiusType::LARGE;
        for (uint32_t bIdx : slotBeams[s])
        {
            if (static_cast<uint8_t>(beamRadius[bIdx]) <
                static_cast<uint8_t>(slot.beamRadius))
                slot.beamRadius = beamRadius[bIdx];
        }

        // MODCOD index proportional to beam gain (spec placeholder values)
        switch (slot.beamRadius)
        {
            case BeamRadiusType::SMALL:  slot.modcod = 15; break;
            case BeamRadiusType::MIDDLE: slot.modcod = 10; break;
            case BeamRadiusType::LARGE:  slot.modcod = 5;  break;
        }

        // Convert 0-indexed beam IDs to 1-indexed (SNS3 convention)
        // and attach cluster IDs (also 1-indexed)
        for (uint32_t bIdx : slotBeams[s])
        {
            uint32_t beamId1  = bIdx + 1;
            uint32_t clustId1 = m_clusterMap.count(bIdx) ? m_clusterMap[bIdx] + 1 : beamId1;
            slot.beamIds.push_back(beamId1);
            slot.clusterIds.push_back(clustId1);
        }

        plan->AddSlot(slot);
    }

    NS_LOG_INFO("SatBhScheduler::BuildPlan: planId=" << plan->GetPlanId()
                << " slots=" << plan->GetNumSlots()
                << " M=" << M
                << " hotspot=" << hotspotBeams.size()
                << " nonHotspot=" << nonHotspotBeams.size());

    return plan;
}

} // namespace ns3
