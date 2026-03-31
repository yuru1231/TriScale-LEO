/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-helper.cc
 *
 * SatBhHelper implementation.
 *
 * Phase 1 is FULLY OPERATIONAL in this file.
 * Phase 2 and 3 setup methods are stubs (log + no-op) until implemented.
 *
 * Phase 1 provides:
 *   - Static BHTP construction (BuildStaticBhtp)
 *   - SatBhMetrics with periodic CSV output
 *   - Synthetic slot driver (ApplySyntheticSlot) simulating OBC events
 *     when m_cfg.enableObc == false
 */

#include "sat-bh-helper.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatBhHelper");
NS_OBJECT_ENSURE_REGISTERED(SatBhHelper);

TypeId
SatBhHelper::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatBhHelper")
            .SetParent<Object>()
            .AddConstructor<SatBhHelper>();
    return tid;
}

SatBhHelper::SatBhHelper()
    : m_syntheticSlotIdx(0),
      m_installed(false)
{
    // Default config is Phase 1 only (basic validation scenario)
    m_cfg.numBeams         = 7;
    m_cfg.maxActiveBeams   = 2;   // K=2 for basic scenario
    m_cfg.numHotspotBeams  = 3;
    m_cfg.satId            = 0;
    m_cfg.warmUpSec        = 10.0;
    m_cfg.bhtpPeriodMs     = 503.0;
    m_cfg.slotDurationMs   = 26.5;
    m_cfg.enableScheduler  = false;
    m_cfg.enableObc        = false;
    m_cfg.enableCacheQueue = false;
    m_cfg.enablePrecoder   = false;
}

// ── Configuration ─────────────────────────────────────────────────────────

void
SatBhHelper::Configure(const BhExperimentConfig& cfg)
{
    NS_ASSERT_MSG(!m_installed, "SatBhHelper::Configure must be called before Install()");
    m_cfg = cfg;
}

void
SatBhHelper::EnableMMSEPrecoding(bool enable)
{
    m_cfg.enablePrecoder = enable;
}

void
SatBhHelper::SetAlpha(double alpha)
{
    m_cfg.alphaDelaySensitivity = alpha;
}

void
SatBhHelper::SetMaxActiveBeams(uint32_t k)
{
    m_cfg.maxActiveBeams = k;
}

void
SatBhHelper::SetNumBeams(uint32_t n)
{
    m_cfg.numBeams = n;
}

void
SatBhHelper::SetSatId(uint32_t satId)
{
    m_cfg.satId = satId;
}

void
SatBhHelper::SetMetricsOutputFile(const std::string& path)
{
    m_cfg.metricsOutputFile = path;
}

void
SatBhHelper::SetSchedulerEnabled(bool enable)
{
    m_cfg.enableScheduler = enable;
}

void
SatBhHelper::SetObcEnabled(bool enable)
{
    m_cfg.enableObc = enable;
}

void
SatBhHelper::SetCacheQueueEnabled(bool enable)
{
    m_cfg.enableCacheQueue = enable;
}

// ── Installation ──────────────────────────────────────────────────────────

void
SatBhHelper::Install()
{
    NS_ASSERT_MSG(!m_installed, "SatBhHelper::Install() called more than once");
    m_installed = true;

    NS_LOG_INFO("SatBhHelper::Install() — configuring BH system\n"
                << "  Phase 1 (TimePlan + Metrics): ACTIVE\n"
                << "  Phase 2 (Scheduler + OBC):   "
                << (m_cfg.enableScheduler || m_cfg.enableObc ? "ACTIVE" : "disabled") << "\n"
                << "  Phase 3 (CacheQueue + MMSE): "
                << (m_cfg.enableCacheQueue || m_cfg.enablePrecoder ? "ACTIVE (stub)" : "disabled"));

    // ── Phase 1: always ───────────────────────────────────────────────────

    // Build static BHTP for Phase 1 (also used as fallback when Scheduler stub)
    m_staticPlan = BuildStaticBhtp(Seconds(0.0));
    bool valid = m_staticPlan->Validate(m_cfg.maxActiveBeams);
    if (!valid)
        NS_LOG_WARN("SatBhHelper: static BHTP validation failed — check config");

    // Pretty-print plan to console
    m_staticPlan->PrettyPrint(std::cout);

    // Export BHTP slot table to CSV
    {
        std::ofstream planCsv(m_cfg.timePlanCsvFile);
        planCsv << m_staticPlan->ToCsv();
        NS_LOG_INFO("SatBhHelper: BHTP slot table written to " << m_cfg.timePlanCsvFile);
    }

    // Create and configure SatBhMetrics
    m_metrics = CreateObject<SatBhMetrics>();
    m_metrics->SetOutputFile(m_cfg.metricsOutputFile);
    m_metrics->SetReportInterval(MilliSeconds(m_cfg.bhtpPeriodMs));
    m_metrics->SetWarmUpTime(Seconds(m_cfg.warmUpSec));

    // ── Phase 2 setup ─────────────────────────────────────────────────────
    // OBC must be created BEFORE Scheduler so SetupScheduler() can wire its
    // plan-ready callback directly to the already-constructed m_obc object.

    if (m_cfg.enableObc)
        SetupObc();

    if (m_cfg.enableScheduler)
        SetupScheduler();

    // ── Phase 3 setup ─────────────────────────────────────────────────────

    if (m_cfg.enableCacheQueue)
        SetupCacheQueue();

    if (m_cfg.enablePrecoder)
        SetupPrecoder();

    // ── Connect traces (Phase 4; currently a stub) ────────────────────────
    ConnectTraces();

    // ── Start metrics flush loop ──────────────────────────────────────────
    // Begin periodic flushing after warm-up period
    Simulator::Schedule(Seconds(m_cfg.warmUpSec),
                        &SatBhMetrics::ScheduleNextFlush, m_metrics);

    // ── Start synthetic slot driver (when OBC is not yet active) ─────────
    if (!m_cfg.enableObc)
    {
        NS_LOG_INFO("SatBhHelper: OBC not enabled — using synthetic slot driver");
        m_syntheticSlotIdx = 0;
        // Start at t=0; warm-up suppression is handled inside SatBhMetrics
        Simulator::Schedule(Seconds(0.0), &SatBhHelper::ApplySyntheticSlot, this);
    }

    // ── Start scheduler cycle loop (Phase 2) ─────────────────────────────
    if (m_cfg.enableScheduler && m_scheduler)
    {
        Simulator::Schedule(Seconds(m_cfg.warmUpSec),
                            &SatBhScheduler::ScheduleNextCycle, m_scheduler);
    }

    // ── Start synthetic demand driver (Phase 2, until ConnectTraces is done) ──
    // Feeds Scheduler::OnDemandReceived() with per-beam sinusoidal demand every T_s.
    // Hotspot beams get ~3× higher base demand than non-hotspot beams, with different
    // phase offsets per beam so EM sees decorrelated inputs.
    // This driver is a placeholder until Phase 4 wires real SNS3 DaRequestReceived
    // trace hooks in ConnectTraces().
    if (m_cfg.enableScheduler && m_scheduler)
    {
        NS_LOG_INFO("SatBhHelper: starting synthetic demand driver (Phase 2)");
        Simulator::Schedule(Seconds(0.0), &SatBhHelper::ApplySyntheticDemand, this);
    }

    NS_LOG_INFO("SatBhHelper::Install() complete");
}

// ── Module access ─────────────────────────────────────────────────────────

Ptr<SatBhTimePlan>
SatBhHelper::GetCurrentPlan() const
{
    if (m_scheduler)
        return m_scheduler->GetCurrentPlan();
    return m_staticPlan;
}

// ── Phase 1: static BHTP builder ─────────────────────────────────────────
//
// Slot layout (for basic scenario numBeams=7, K=2, M=19):
//   Hotspot beams (beamId 1..numHotspotBeams): 3 slots each, SMALL radius
//   Non-hotspot   (beamId numHotspotBeams+1..numBeams): ≤2 slots each, LARGE
//   Each slot activates K=2 beams: one hotspot + one non-hotspot (round-robin)
//
// For general (numBeams > 7): same principle, round-robin pairing.
//
Ptr<SatBhTimePlan>
SatBhHelper::BuildStaticBhtp(Time periodStart)
{
    Ptr<SatBhTimePlan> plan = CreateObject<SatBhTimePlan>();
    plan->SetPlanId(1);

    const Time T_s = MilliSeconds(m_cfg.slotDurationMs);
    const Time T_p = MilliSeconds(m_cfg.bhtpPeriodMs);
    plan->SetPeriodBounds(periodStart, periodStart + T_p);

    const uint32_t K = m_cfg.maxActiveBeams;
    const uint32_t N = m_cfg.numBeams;
    const uint32_t H = std::min(m_cfg.numHotspotBeams, N);

    // M = ceil(T_p / T_s) = 19 for default parameters
    const uint32_t M = static_cast<uint32_t>(
        std::ceil(m_cfg.bhtpPeriodMs / m_cfg.slotDurationMs));

    // Collect hotspot and non-hotspot beam IDs (1-indexed per SNS3 convention)
    std::vector<uint32_t> hotspot, nonHotspot;
    for (uint32_t i = 1; i <= N; i++)
    {
        if (i <= H) hotspot.push_back(i);
        else        nonHotspot.push_back(i);
    }

    // Fill M slots round-robin: pair hotspot + non-hotspot beams
    // If K > 2, extend to fill remaining beam slots with additional pairings
    uint32_t hotIdx    = 0;
    uint32_t nonHotIdx = 0;

    for (uint32_t slotIdx = 0; slotIdx < M; slotIdx++)
    {
        BhSlotEntry slot;
        slot.startTime  = slotIdx * T_s;
        slot.duration   = T_s;
        slot.modcod     = 5;  // placeholder MODCOD

        // Fill up to K beams: alternate hotspot / non-hotspot
        for (uint32_t k = 0; k < K; k++)
        {
            if (k % 2 == 0 && !hotspot.empty())
            {
                uint32_t bid = hotspot[hotIdx % hotspot.size()];
                slot.beamIds.push_back(bid);
                slot.clusterIds.push_back(bid);
                slot.beamRadius = BeamRadiusType::SMALL;
                if (k == 0) hotIdx++;
            }
            else if (!nonHotspot.empty())
            {
                uint32_t bid = nonHotspot[nonHotIdx % nonHotspot.size()];
                slot.beamIds.push_back(bid);
                slot.clusterIds.push_back(bid);
                // Use LARGE for non-hotspot; first beam in slot sets dominant radius
                if (slot.beamRadius != BeamRadiusType::SMALL)
                    slot.beamRadius = BeamRadiusType::LARGE;
                nonHotIdx++;
            }
        }

        if (!slot.beamIds.empty())
            plan->AddSlot(slot);
    }

    NS_LOG_INFO("SatBhHelper::BuildStaticBhtp: created plan with "
                << plan->GetNumSlots() << " slots"
                << " (M=" << M << ", K=" << K
                << ", N=" << N << ", hotspot=" << H << ")");

    return plan;
}

// ── Phase 1: synthetic slot driver ────────────────────────────────────────
//
// Simulates one slot's worth of OBC + CacheQueue events for SatBhMetrics.
// Fires every T_s and wraps around the static plan.
// Replaced by real OBC callbacks when Phase 2 is enabled.
//
void
SatBhHelper::ApplySyntheticSlot()
{
    if (!m_staticPlan || !m_metrics)
        return;

    // Wrap around at end of plan
    if (m_syntheticSlotIdx >= m_staticPlan->GetNumSlots())
        m_syntheticSlotIdx = 0;

    const BhSlotEntry& slot   = m_staticPlan->GetSlots()[m_syntheticSlotIdx];
    const Time         T_s    = slot.duration;
    const Time         T_sw   = MilliSeconds(m_cfg.switchingTimeMs);
    const Time         usable = T_s - T_sw;  // Effective slot window

    for (uint32_t beamId : slot.beamIds)
    {
        // Notify Metrics: slot started for this beam
        m_metrics->OnSlotActivated(m_cfg.satId, beamId, usable);

        // Simulate packet receptions
        bool     isHotspot = (beamId <= m_cfg.numHotspotBeams);
        uint32_t numPkts   = isHotspot ? 5 : 2;
        uint32_t pktSize   = isHotspot ? 1500 : 1024;  // bytes
        double   delayMs   = isHotspot ? 10.0 : 20.0;

        for (uint32_t p = 0; p < numPkts; p++)
        {
            Ptr<Packet> pkt = Create<Packet>(pktSize);
            m_metrics->OnPacketReceived(m_cfg.satId, beamId, pkt, delayMs);
        }

        // Simulate occasional tail-drop (hotspot, ~5% rate)
        if (isHotspot && (m_syntheticSlotIdx % 20 == 0))
        {
            Ptr<Packet> dropped = Create<Packet>(pktSize);
            m_metrics->OnPacketDropped(m_cfg.satId, beamId, dropped);
        }

        // Notify Metrics: slot ended; utilization factor
        double utilFraction = isHotspot ? 0.80 : 0.40;
        Time   usedDur      = Time(usable.GetTimeStep() * utilFraction);
        m_metrics->OnSlotDeactivated(m_cfg.satId, beamId, usedDur);
    }

    m_syntheticSlotIdx++;

    // Re-schedule for next slot boundary
    Simulator::Schedule(T_s, &SatBhHelper::ApplySyntheticSlot, this);
}

// ── Phase 2: synthetic demand driver ─────────────────────────────────────────
//
// Injects per-beam demand into the Scheduler every T_s so the EM algorithm
// has non-trivial input data before real SNS3 trace hooks are connected.
//
// Demand model (per beam):
//   base      = hotspot ? 6 : 2   (abstract demand tokens; ratio matters, not absolute)
//   variation = amplitude × sin(2π × t / T_cycle + phase_b)
//   amplitude = hotspot ? 2.0 : 0.5
//   T_cycle   = 60 s  (one slow demand cycle per minute)
//   phase_b   = b × π/3  (60° offset per beam, prevents lock-step correlation)
//
// Result: λ_hotspot ≈ 6 >> λ_non-hotspot ≈ 2, so EM assigns proportionally
// more slots to hotspot beams, producing a BHTP that differs from Phase 1's
// static plan (which assigned slots based on hard-coded hotspot index alone).
//
// This driver is a placeholder until Phase 4 connects real SNS3 DaRequestReceived
// trace hooks in ConnectTraces().
//
void
SatBhHelper::ApplySyntheticDemand()
{
    // Safety check: stop if Scheduler was destroyed or not yet created
    if (!m_scheduler)
        return;

    double nowSec = Simulator::Now().GetSeconds();

    for (uint32_t b = 0; b < m_cfg.numBeams; b++)
    {
        uint32_t beamId  = b + 1;  // 1-indexed per SNS3 beam convention
        bool     hotspot = (beamId <= m_cfg.numHotspotBeams);

        // Base demand: hotspot beams carry ~3× the traffic load of non-hotspot
        double base      = hotspot ? 6.0 : 2.0;
        double amplitude = hotspot ? 2.0 : 0.5;

        // Per-beam phase offset (60° spacing) to decorrelate demand signals.
        // This prevents all beams from changing demand in the same direction
        // simultaneously, giving EM more informative input to work with.
        double phaseRad = static_cast<double>(b) * (M_PI / 3.0);

        // Slow sinusoidal variation: one cycle per 60 s
        double variation = amplitude * std::sin(2.0 * M_PI * nowSec / 60.0 + phaseRad);

        // Clamp to minimum 1 (EM also guards with max(1,...) but be explicit here)
        uint32_t demand = static_cast<uint32_t>(std::max(1.0, base + variation));

        m_scheduler->OnDemandReceived(beamId, demand);
    }

    // Re-schedule for the next slot boundary so every T_s produces a demand sample
    Simulator::Schedule(MilliSeconds(m_cfg.slotDurationMs),
                        &SatBhHelper::ApplySyntheticDemand,
                        this);
}

// ── Phase 2 setup ─────────────────────────────────────────────────────────────

void
SatBhHelper::SetupScheduler()
{
    // Phase 2: create SatBhScheduler, configure attributes from m_cfg,
    // and wire the plan-ready callback to SatBhObc::ReceiveNewPlan.
    NS_LOG_INFO("SatBhHelper::SetupScheduler (Phase 2 active)");

    m_scheduler = CreateObject<SatBhScheduler>();
    m_scheduler->SetAttribute("NumBeams",          UintegerValue(m_cfg.numBeams));
    m_scheduler->SetAttribute("MaxActiveBeams",    UintegerValue(m_cfg.maxActiveBeams));
    m_scheduler->SetAttribute("EmMaxIterations",   UintegerValue(m_cfg.emMaxIterations));
    m_scheduler->SetAttribute("EmConvergenceEps",  DoubleValue(m_cfg.emConvergenceEps));
    m_scheduler->SetAttribute("AlphaDelaySensitivity", DoubleValue(m_cfg.alphaDelaySensitivity));
    m_scheduler->SetAttribute("InterferenceKappa", DoubleValue(m_cfg.interferenceKappa));
    m_scheduler->SetAttribute("PropagationDelayMs", DoubleValue(m_cfg.propagationDelayMs));

    // Wire plan-ready callback to OBC (if OBC is also enabled)
    if (m_cfg.enableObc && m_obc)
    {
        m_scheduler->SetPlanReadyCallback(
            [this](Ptr<SatBhTimePlan> plan, Time propDelay) {
                m_obc->ReceiveNewPlan(plan, propDelay);
            });
    }
}

void
SatBhHelper::SetupObc()
{
    // Phase 2: create SatBhObc, configure attributes, and wire beam callbacks
    // to SatBhMetrics. Phase 3 will additionally wire to SatGwCacheQueue.
    NS_LOG_INFO("SatBhHelper::SetupObc (Phase 2 active)");

    m_obc = CreateObject<SatBhObc>();
    m_obc->SetSatId(m_cfg.satId);
    m_obc->SetAttribute("SwitchingTimeMs", DoubleValue(m_cfg.switchingTimeMs));

    // Wire OBC beam-activate → Metrics::OnSlotActivated
    m_obc->SetBeamActivateCallback(
        [this](uint32_t satId, uint32_t beamId, Time usableDur) {
            if (m_metrics)
                m_metrics->OnSlotActivated(satId, beamId, usableDur);
            // TODO Phase 3: also call m_cacheQueue->DequeueAll(beamId)
        });

    // Wire OBC beam-deactivate → Metrics::OnSlotDeactivated
    m_obc->SetBeamDeactivateCallback(
        [this](uint32_t satId, uint32_t beamId, Time usedDur) {
            if (m_metrics)
                m_metrics->OnSlotDeactivated(satId, beamId, usedDur);
        });
}

// ── Phase 3 setup (stubs) ─────────────────────────────────────────────────

void
SatBhHelper::SetupCacheQueue()
{
    // TODO Phase 3: create SatGwCacheQueue, connect drop→Metrics, dequeue→Metrics
    NS_LOG_INFO("SatBhHelper::SetupCacheQueue [STUB — Phase 3 not yet implemented]");

    m_cacheQueue = CreateObject<SatGwCacheQueue>();
    m_cacheQueue->SetSatId(m_cfg.satId);
    m_cacheQueue->SetAttribute("MaxQueueSizeMB", DoubleValue(m_cfg.maxQueueSizeMB));

    // Wire drop callback → Metrics::OnPacketDropped
    m_cacheQueue->SetDropCallback(
        [this](uint32_t satId, uint32_t beamId, Ptr<const Packet> pkt) {
            if (m_metrics)
                m_metrics->OnPacketDropped(satId, beamId, pkt);
        });

    // Wire dequeue callback → Metrics::OnPacketReceived
    m_cacheQueue->SetDequeueCallback(
        [this](uint32_t satId, uint32_t beamId, Ptr<const Packet> pkt, double delayMs) {
            if (m_metrics)
                m_metrics->OnPacketReceived(satId, beamId, pkt, delayMs);
        });

    // TODO Phase 3: wire OBC BeamActivateCallback → CacheQueue::DequeueAll
    if (m_obc)
    {
        // Override the existing BeamActivateCallback to also trigger dequeue
        m_obc->SetBeamActivateCallback(
            [this](uint32_t satId, uint32_t beamId, Time usableDur) {
                if (m_metrics)
                    m_metrics->OnSlotActivated(satId, beamId, usableDur);
                if (m_cacheQueue)
                    m_cacheQueue->DequeueAll(beamId);
            });
    }
}

void
SatBhHelper::SetupPrecoder()
{
    // TODO Phase 3: create SatBhPrecoder, connect CSI estimate hook
    NS_LOG_INFO("SatBhHelper::SetupPrecoder [STUB — Phase 3 not yet implemented]");

    m_precoder = CreateObject<SatBhPrecoder>();
    m_precoder->SetEnabled(m_cfg.enablePrecoder);
    m_precoder->SetAttribute("NoisePowerDb", DoubleValue(m_cfg.noisePowerDb));
}

// ── SNS3 trace connection (Phase 4 stub) ─────────────────────────────────

void
SatBhHelper::ConnectTraces()
{
    // TODO Phase 4 (after Hook feasibility check, Layer2.md Step 0):
    //
    //   Config::ConnectWithoutContext(
    //       "/NodeList/*/DeviceList/*/$ns3::SatNetDevice/SatGwMac/Rx",
    //       MakeCallback(&SatBhMetrics::OnPacketReceived, m_metrics));
    //
    //   Config::ConnectWithoutContext(
    //       "/NodeList/*/DeviceList/*/$ns3::SatNetDevice/SatGwMac/Tx",
    //       MakeCallback(&SatGwCacheQueue::Enqueue, m_cacheQueue));
    //
    //   Config::ConnectWithoutContext(
    //       "...DaRequestReceived...",
    //       MakeCallback(&SatBhScheduler::OnDemandReceived, m_scheduler));
    //
    //   Each hook path must be verified with Layer2.md Step 0 checklist.
    //   If a trace does not exist, use the fallback strategy listed in
    //   Layer2.md Section 8.

    NS_LOG_INFO("SatBhHelper::ConnectTraces [STUB — SNS3 traces not yet connected]"
                "\n  Pending Hook feasibility check (Layer2.md Step 0)");
}

} // namespace ns3
