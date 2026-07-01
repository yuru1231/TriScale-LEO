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
#include "sat-constellation-params.h"
#include "sat-greedy-bstp-provider.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

// Phase E: SNS3 module headers for real callback wiring
#include "ns3/satellite-beam-helper.h"         // SatBeamHelper::GetBeams(), GetNcc()
#include "ns3/satellite-beam-scheduler.h"      // SatBeamScheduler::BacklogRequestsTrace (Phase F)
#include "ns3/satellite-id-mapper.h"           // SatIdMapper::GetUtIdWithMac (Phase F)
#include "ns3/satellite-mac.h"                 // SatMac::GetBeamId() — used by BuildBeamToggleMap
#include "ns3/satellite-net-device.h"          // SatNetDevice: GetMac(), ToggleState()
#include "ns3/satellite-ncc.h"                 // SatNcc::MoveUtBetweenBeams, GetBeamScheduler
#include "ns3/satellite-orbiter-net-device.h"  // SatOrbiterNetDevice::GetUserPhy
#include "ns3/satellite-phy.h"                 // SatPhy::SetTxMaxPowerDbw + Initialize
#include "ns3/satellite-topology.h"            // SatTopology: GetUtNodes, GetGwNodes, GetUtBeamId
#include "ns3/simulation-helper.h"             // SimulationHelper::GetSatelliteHelper
#include "ns3/singleton.h"                     // Singleton<SatTopology/SatIdMapper>::Get()

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>   // std::istringstream for OnBacklogRequestTrace parsing

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
      m_installed(false),
      m_dynamicPlanId(0)
{
    // Default config: Phase 1, Starlink-1584, Tokyo 37°, K_b=2, 5×5 grid
    m_cfg.numBeams         = 7;   // leo2sat default; starlink25 sets 25 via Configure()
    m_cfg.maxActiveBeams   = 2;   // K_b=2 (confirmed viable at 37° SNR margin)
    m_cfg.numHotspotBeams  = 3;   // legacy fallback; overridden by hotCellIndices
    m_cfg.satId            = 0;
    m_cfg.warmUpSec        = 10.0;
    m_cfg.bhtpPeriodMs     = 80.0;   // 80 ms frame = 8 slots
    m_cfg.slotDurationMs   = 10.0;   // 10 ms per slot
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

void
SatBhHelper::SetResourceManagerEnabled(bool enable)
{
    m_cfg.enableResourceManager = enable;
}

void
SatBhHelper::SetSimulationHelper(Ptr<SimulationHelper> simHelper)
{
    // Store pointer so ConnectTracesPhaseE() can reach SatNcc and topology.
    // Must be called before Install() when enablePhaseE=true.
    m_simHelper = simHelper;
}

void
SatBhHelper::SetFrameConfigCallback(FrameConfigCallback cb)
{
    m_frameConfigCb = cb;
    // Forward immediately if ResourceManager already exists
    if (m_resourceManager)
        m_resourceManager->SetFrameConfigCallback(cb);
}

void
SatBhHelper::OpenTrafficTrace()
{
    if (m_cfg.trafficTraceFile.empty() || m_trafficTrace.is_open())
    {
        return;
    }

    std::filesystem::path tracePath(m_cfg.trafficTraceFile);
    if (tracePath.has_parent_path())
    {
        std::error_code ec;
        std::filesystem::create_directories(tracePath.parent_path(), ec);
        NS_ASSERT_MSG(!ec,
                      "SatBhHelper: cannot create traffic trace directory for "
                          << m_cfg.trafficTraceFile << ": " << ec.message());
    }

    bool writeHeader =
        !std::filesystem::exists(tracePath) || std::filesystem::file_size(tracePath) == 0;

    m_trafficTrace.open(m_cfg.trafficTraceFile, std::ios::out | std::ios::app);
    NS_ASSERT_MSG(m_trafficTrace.is_open(),
                  "SatBhHelper: cannot open traffic trace file: " << m_cfg.trafficTraceFile);

    if (writeHeader)
    {
        m_trafficTrace
            << "record_type,time_s,sat_id,beam_id,event,plan_id,slot_idx,"
            << "demand_kbps,duration_ms,active_beams,mapped,toggled\n";
        m_trafficTrace.flush();
    }

    NS_LOG_INFO("SatBhHelper: BH traffic/event trace -> " << m_cfg.trafficTraceFile
                                                          << " sat=" << m_cfg.satId);
}

void
SatBhHelper::EmitTrafficTrace(const std::string& recordType,
                              double timeSec,
                              uint32_t beamId,
                              const std::string& event,
                              uint32_t planId,
                              int32_t slotIdx,
                              double demandKbps,
                              double durationMs,
                              const std::string& activeBeams,
                              bool mapped,
                              bool toggled)
{
    if (!m_trafficTrace.is_open())
    {
        return;
    }

    m_trafficTrace << recordType << ","
                   << std::fixed << std::setprecision(6) << timeSec << ","
                   << m_cfg.satId << ","
                   << beamId << ","
                   << event << ","
                   << planId << ","
                   << slotIdx << ","
                   << std::setprecision(3) << demandKbps << ","
                   << durationMs << ","
                   << activeBeams << ","
                   << (mapped ? 1 : 0) << ","
                   << (toggled ? 1 : 0) << "\n";
    m_trafficTrace.flush();
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
        std::filesystem::path planPath(m_cfg.timePlanCsvFile);
        if (planPath.has_parent_path())
        {
            std::error_code ec;
            std::filesystem::create_directories(planPath.parent_path(), ec);
            NS_ASSERT_MSG(!ec,
                          "SatBhHelper: cannot create time-plan output directory for "
                              << m_cfg.timePlanCsvFile << ": " << ec.message());
        }

        std::ofstream planCsv(m_cfg.timePlanCsvFile);
        NS_ASSERT_MSG(planCsv.is_open(),
                      "SatBhHelper: cannot open time-plan output file: "
                          << m_cfg.timePlanCsvFile);
        planCsv << m_staticPlan->ToCsv();
        NS_LOG_INFO("SatBhHelper: BHTP slot table written to " << m_cfg.timePlanCsvFile);
    }

    // Create and configure SatBhMetrics
    m_metrics = CreateObject<SatBhMetrics>();
    m_metrics->SetOutputFile(m_cfg.metricsOutputFile);
    m_metrics->SetReportInterval(MilliSeconds(m_cfg.bhtpPeriodMs));
    m_metrics->SetWarmUpTime(Seconds(m_cfg.warmUpSec));

    OpenTrafficTrace();

    // ── Phase 2 setup ─────────────────────────────────────────────────────
    // OBC must be created BEFORE Scheduler so SetupScheduler() can wire its
    // plan-ready callback directly to the already-constructed m_obc object.

    if (m_cfg.enableObc)
    {
        SetupObc();
        // Build the {satId, beamId} → GW SatNetDevice map immediately after OBC is created.
        // OBC callbacks capture `this` and read m_beamToggleMap at call-time, so
        // the map just needs to be filled before simulation events fire (t=0).
        // This is independent of ResourceManager / Phase E — no guard needed.
        BuildBeamToggleMap();
    }

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
    // Phase G (enableDynamicBstp): flush is driven by RunDynamicBstpCycle()
    // so it fires exactly once per T_p at the cycle boundary.
    // Phase 1 / static plan: use periodic timer anchored to warmUpSec.
    if (!m_cfg.enableDynamicBstp)
    {
        Simulator::Schedule(Seconds(m_cfg.warmUpSec),
                            &SatBhMetrics::ScheduleNextFlush, m_metrics);
    }

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

    // ── Phase C setup ─────────────────────────────────────────────────────
    if (m_cfg.enableResourceManager)
        SetupPhaseC();

    // ── Phase D setup ─────────────────────────────────────────────────────
    // Requires Phase C (ResourceManager) to be active so the allocator can
    // be wired into m_resourceManager.
    if (m_cfg.enablePowerAllocation)
    {
        if (!m_resourceManager)
        {
            NS_LOG_WARN("SatBhHelper: enablePowerAllocation=true but"
                        " enableResourceManager=false — Phase D skipped."
                        " Add --enableResourceManager=1 to activate.");
        }
        else
        {
            SetupPhaseD();
        }
    }

    // ── Phase E wiring ────────────────────────────────────────────────────
    // Wires MoveUtCallback → SatNcc and ApplyPowerCallback → SatOrbiterUserPhy.
    // Requires Phase C (ResourceManager + UserAssociator) to be active.
    // Requires SetSimulationHelper() called before Install().
    if (m_cfg.enablePhaseE)
    {
        if (!m_resourceManager)
        {
            NS_LOG_WARN("SatBhHelper: enablePhaseE=true but enableResourceManager=false"
                        " — Phase E skipped. Add --enableResourceManager=1.");
        }
        else if (!m_simHelper)
        {
            NS_LOG_WARN("SatBhHelper: enablePhaseE=true but SetSimulationHelper() not called"
                        " — Phase E skipped. Call bhHelper->SetSimulationHelper(simHelper)"
                        " before Install().");
        }
        else
        {
            ConnectTracesPhaseE();
        }
    }

    // ── Phase F wiring ────────────────────────────────────────────────────
    // Connects BacklogRequestsTrace on every SatBeamScheduler so that real
    // RBDC demand is available.  Two activation paths:
    //
    //   (a) Phase E active (enablePhaseE=true + ResourceManager):
    //       BuildUtAddressMap() was already called in ConnectTracesPhaseE().
    //       Demand flows: trace → m_utDemandCache → PollUtStates() → ResourceManager.
    //
    //   (b) Phase G active (enableDynamicBstp=true), Phase E not required:
    //       BuildUtAddressMap() is called here if the map was not built by Phase E.
    //       Demand flows: trace → OnBacklogRequestTrace() → m_dynamicProvider->UpdateBeamDemand().
    //       ResourceManager and PollUtStates() are not involved.
    //
    // In both cases m_satMapperIdToContainerIdx must be populated before the trace fires.
    if (m_cfg.enablePhaseF)
    {
        // Phase F is valid when Phase E is fully active, or when Phase G is active
        // (Phase G feeds demand directly through m_dynamicProvider without ResourceManager).
        bool phaseEReady = m_cfg.enablePhaseE && (m_resourceManager != nullptr);
        bool phaseFForG  = m_cfg.enableDynamicBstp;

        if (!phaseEReady && !phaseFForG)
        {
            NS_LOG_WARN("SatBhHelper: enablePhaseF=true but neither Phase E nor Phase G active"
                        " — Phase F skipped."
                        " Enable with --enablePhaseE=1 --enableResourceManager=1 (full path)"
                        " or --enableDynamicBstp=1 (Phase G demand path).");
        }
        else if (!m_simHelper)
        {
            NS_LOG_WARN("SatBhHelper: enablePhaseF=true but SetSimulationHelper() not called"
                        " — Phase F skipped.");
        }
        else
        {
            // If Phase E did not run, BuildUtAddressMap was never called.
            // Build it now (best-effort — some UTs may not be in SatIdMapper yet).
            if (!phaseEReady && m_satMapperIdToContainerIdx.empty())
            {
                NS_LOG_INFO("SatBhHelper: Phase E not active — building UT address map"
                            " for Phase F demand parsing (Phase G path)");
                BuildUtAddressMap();
            }
            ConnectTracesPhaseF();

            // SatIdMapper registration is NOT guaranteed complete at Install() time.
            // UTs that have not yet called AttachMacAddress are absent from the map,
            // causing OnBacklogRequestTrace to silently drop their RBDC updates.
            // Rebuild the map at t=0.5 s (before the first BacklogRequestsTrace at ~1 s)
            // so all 25 UTs are present when demand starts arriving.
            Simulator::Schedule(MilliSeconds(500), &SatBhHelper::BuildUtAddressMap, this);
        }
    }

    // ── Phase G: Dynamic BSTP Provider ───────────────────────────────────
    // Demand-greedy top-K beam selector.  Mutually exclusive with Phase 2
    // Scheduler: if both are enabled, Scheduler wins and Phase G is skipped.
    if (m_cfg.enableDynamicBstp)
    {
        if (m_cfg.enableScheduler)
        {
            NS_LOG_WARN("SatBhHelper: enableDynamicBstp=true but enableScheduler=true"
                        " — Phase G skipped (Scheduler takes priority)."
                        " Set enableScheduler=false to use the dynamic provider.");
        }
        else
        {
            SetupDynamicBstp();
        }
    }

    NS_LOG_INFO("SatBhHelper::Install() complete");
}

// ── Module access ─────────────────────────────────────────────────────────

Ptr<SatBhTimePlan>
SatBhHelper::GetCurrentPlan() const
{
    if (m_scheduler)
        return m_scheduler->GetCurrentPlan();
    // Phase G: OBC receives plans via RunDynamicBstpCycle but m_staticPlan is NOT
    // updated when OBC is active — return the last plan delivered to the provider
    // instead so callers see the current dynamic plan rather than the stale static one.
    if (m_lastDynamicPlan)
        return m_lastDynamicPlan;
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

    // M = floor(T_p / T_s) slots per frame — e.g. 80 ms / 10 ms = 8
    const uint32_t M = static_cast<uint32_t>(
        std::ceil(m_cfg.bhtpPeriodMs / m_cfg.slotDurationMs));

    // Collect hotspot and non-hotspot beam IDs (1-indexed per SNS3 convention).
    // When hotCellIndices is set, use explicit 0-indexed cell positions; otherwise
    // fall back to legacy numHotspotBeams (first H beams are hotspot).
    std::vector<uint32_t> hotspot, nonHotspot;
    if (!m_cfg.hotCellIndices.empty())
    {
        // Build a lookup set from 0-indexed cell positions for O(1) membership test
        std::set<uint32_t> hotSet(m_cfg.hotCellIndices.begin(), m_cfg.hotCellIndices.end());
        for (uint32_t i = 1; i <= N; i++)
        {
            uint32_t cellIdx = i - 1;  // convert 1-indexed beamId to 0-indexed cell
            if (hotSet.count(cellIdx)) hotspot.push_back(i);
            else                       nonHotspot.push_back(i);
        }
    }
    else
    {
        // Legacy: first numHotspotBeams beams are hotspot
        const uint32_t H = std::min(m_cfg.numHotspotBeams, N);
        for (uint32_t i = 1; i <= N; i++)
        {
            if (i <= H) hotspot.push_back(i);
            else        nonHotspot.push_back(i);
        }
    }

    // Slot allocation strategy (Phase 1, all beams use MIDDLE 2.0° pattern):
    //   Standard slots: 1 hotspot beam + (K-1) non-hotspot beams, round-robin
    //   Overflow slots: K non-hotspot beams — used when nonHotspot.size() > M
    //
    // When numNonHotspot > M, standard allocation leaves the last
    // (numNonHotspot - M) beams unserved.  To cover them within the same T_p,
    // the first `overflowSlots` are filled with K non-hotspot beams instead of
    // the usual 1 hotspot + (K-1) non-hotspot, trading hotspot dwell for full coverage.
    //
    // Overflow slots handle the case where nonHotspot.size() > M with NO hotspot beams.
    // When hotspot beams exist, every slot uses 1 hotspot + (K-1) non-hotspot (round-robin),
    // so non-hotspot beams simply cycle across multiple frame periods — overflowSlots = 0.
    // Example (starlink25): N=25, K_b=2, M=8, hotspot=5, nonHotspot=20
    //   → overflowSlots=0: all 8 slots serve 1 hotspot + 1 non-hotspot (20 beams cycle in 2.5 frames)
    // Example (legacy, no hotspot): N=7, K_b=2, M=8, hotspot=0, nonHotspot=7
    //   → overflowSlots=0 (7 < 8), all slots serve 2 non-hotspot beams
    uint32_t overflowSlots = (!hotspot.empty()) ? 0U
        : (nonHotspot.size() > M)
            ? static_cast<uint32_t>(nonHotspot.size() - M)
            : 0U;

    uint32_t hotIdx    = 0;
    uint32_t nonHotIdx = 0;

    for (uint32_t slotIdx = 0; slotIdx < M; slotIdx++)
    {
        BhSlotEntry slot;
        slot.startTime  = slotIdx * T_s;
        slot.duration   = T_s;
        slot.modcod     = 5;  // placeholder MODCOD

        if (slotIdx < overflowSlots)
        {
            // Overflow slot: fill with K MIDDLE beams to cover non-hotspot overflow.
            // No hotspot beam in these slots — hotspot dwell reduced proportionally.
            for (uint32_t k = 0; k < K && !nonHotspot.empty(); k++)
            {
                uint32_t bid = nonHotspot[nonHotIdx % nonHotspot.size()];
                slot.beamIds.push_back(bid);
                slot.clusterIds.push_back(bid);
                slot.SetBeamPattern(bid, BeamRadiusType::MIDDLE);
                nonHotIdx++;
            }
        }
        else
        {
            // Standard slot: 1 hotspot + (K-1) non-hotspot, all MIDDLE pattern
            for (uint32_t k = 0; k < K; k++)
            {
                if (k % 2 == 0 && !hotspot.empty())
                {
                    uint32_t bid = hotspot[hotIdx % hotspot.size()];
                    slot.beamIds.push_back(bid);
                    slot.clusterIds.push_back(bid);
                    // Unified 2.0° MIDDLE pattern for all beams (Phase 1 simplification)
                    slot.SetBeamPattern(bid, BeamRadiusType::MIDDLE);
                    if (k == 0) hotIdx++;
                }
                else if (!nonHotspot.empty())
                {
                    uint32_t bid = nonHotspot[nonHotIdx % nonHotspot.size()];
                    slot.beamIds.push_back(bid);
                    slot.clusterIds.push_back(bid);
                    // Unified 2.0° MIDDLE pattern for all beams (Phase 1 simplification)
                    slot.SetBeamPattern(bid, BeamRadiusType::MIDDLE);
                    nonHotIdx++;
                }
            }
        }

        if (!slot.beamIds.empty())
            plan->AddSlot(slot);
    }

    NS_LOG_INFO("SatBhHelper::BuildStaticBhtp: created plan with "
                << plan->GetNumSlots() << " slots"
                << " (M=" << M << ", K=" << K
                << ", N=" << N << ", hotspots=" << hotspot.size()
                << ", overflowSlots=" << overflowSlots << ")");

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

        // Phase 3: release any buffered packets now that beam is active.
        // Mirrors what SetupCacheQueue() wires via OBC BeamActivateCallback.
        if (m_cacheQueue)
            m_cacheQueue->DequeueAll(beamId);

        // Simulate packet receptions — hotspot check uses hotCellIndices (0-indexed)
        const auto& hci = m_cfg.hotCellIndices;
        bool isHotspot = !hci.empty()
            ? std::find(hci.begin(), hci.end(), beamId - 1) != hci.end()
            : (beamId <= m_cfg.numHotspotBeams);
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
        // hotspot check: use hotCellIndices (0-indexed) when available
        const auto& hci = m_cfg.hotCellIndices;
        bool hotspot = !hci.empty()
            ? std::find(hci.begin(), hci.end(), b) != hci.end()
            : (beamId <= m_cfg.numHotspotBeams);

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

    // ── Inject altitude-derived constellation geometry ────────────────────
    // SetConstellationParams() updates rMiddle in ComputeInterferenceFactor().
    // SetBeamPositions() bypasses the internal hex-ring fallback so the
    // scheduler uses the same rectangular grid as the 2D footprint tool.
    m_scheduler->SetConstellationParams(m_cfg.altitudeKm, m_cfg.beamHalfAngleDeg);

    // Compute the Nside×Nside rectangular grid (Nside = sqrt(numBeams)).
    // For 25 beams this gives a 5×5 grid with cell spacing = 2 × r_beam.
    const uint32_t Nside = static_cast<uint32_t>(std::round(std::sqrt(
        static_cast<double>(m_cfg.numBeams))));
    if (Nside >= 2 && Nside * Nside == m_cfg.numBeams)
    {
        ConstellationParams cp{m_cfg.altitudeKm, m_cfg.minElevDeg};
        auto gridPos = cp.GridPositions(Nside, Nside, m_cfg.beamHalfAngleDeg);
        m_scheduler->SetBeamPositions(gridPos);
    }
    // Non-square counts fall back to the hex-ring layout in InitBeamPositions().
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

    // Wire OBC beam-activate → Metrics::OnSlotActivated + synthetic packet injection
    // + SatNetDevice::ToggleState(true).
    //
    // Synthetic injection mirrors ApplySyntheticSlot (used when enableObc=false).
    // It keeps throughput/delay KPIs non-zero until Phase F wires real SNS3 demand
    // traces (BacklogRequestsTrace → UpdateBeamDemand).  The guard on m_cfg.enablePhaseF
    // lets future code disable synthetic injection once real traffic is connected.
    //
    // m_beamToggleMap is populated by BuildBeamToggleMap() during Install().
    m_obc->SetBeamActivateCallback(
        [this](uint32_t satId, uint32_t beamId, Time usableDur) {
            if (m_metrics)
                m_metrics->OnSlotActivated(satId, beamId, usableDur);

            // Synthetic traffic: active only when real SNS3 demand is not yet wired.
            // Hotspot classification mirrors ApplySyntheticSlot and ApplySyntheticDemand.
            if (m_metrics && !m_cfg.enablePhaseF)
            {
                const auto& hci = m_cfg.hotCellIndices;
                bool isHot = !hci.empty()
                    ? std::find(hci.begin(), hci.end(), beamId - 1) != hci.end()
                    : (beamId <= m_cfg.numHotspotBeams);
                uint32_t numPkts  = isHot ? 5 : 2;
                uint32_t pktBytes = isHot ? 1500 : 1024;
                double   delayMs  = isHot ? 10.0 : 20.0;
                for (uint32_t p = 0; p < numPkts; ++p)
                {
                    Ptr<Packet> pkt = Create<Packet>(pktBytes);
                    m_metrics->OnPacketReceived(satId, beamId, pkt, delayMs);
                }
            }

            // Key = {satId, beamId} — flat beamId would mis-toggle beams on other sats
            auto toggleIt = m_beamToggleMap.find({satId, beamId});
            bool mapped = (toggleIt != m_beamToggleMap.end());
            bool toggled = false;
            if (mapped)
            {
                toggleIt->second->ToggleState(true);
                toggled = true;
            }
            else
            {
                std::cout << "[BH-TOGGLEMAP][MISS] action=activate"
                          << " sat=" << satId
                          << " beam=" << beamId
                          << " mappedCount=" << m_beamToggleMap.size()
                          << std::endl;
            }
            EmitTrafficTrace("EVENT",
                             Simulator::Now().GetSeconds(),
                             beamId,
                             "ACTIVATE",
                             m_lastDynamicPlan ? m_lastDynamicPlan->GetPlanId() : 0,
                             -1,
                             0.0,
                             usableDur.GetMilliSeconds(),
                             "",
                             mapped,
                             toggled);
        });

    // Wire OBC beam-deactivate → Metrics::OnSlotDeactivated + SatNetDevice::ToggleState(false).
    //
    // OBC always passes usedDur = T_s - T_sw (full usable window), which makes
    // slot_util_pct permanently 100%.  Apply a hotspot-based utilisation fraction
    // to produce differentiated KPI values until real per-packet accounting is connected.
    m_obc->SetBeamDeactivateCallback(
        [this](uint32_t satId, uint32_t beamId, Time usedDur) {
            if (m_metrics)
            {
                // Estimate actual slot utilisation from hotspot classification.
                // These fractions match ApplySyntheticSlot: hotspot=0.80, others=0.40.
                const auto& hci = m_cfg.hotCellIndices;
                bool isHot = !hci.empty()
                    ? std::find(hci.begin(), hci.end(), beamId - 1) != hci.end()
                    : (beamId <= m_cfg.numHotspotBeams);
                double utilFrac = isHot ? 0.80 : 0.40;
                // Compute actualUsed from config values (T_s - T_sw) so the
                // denominator (slotAllocMs, from OnSlotActivated) and numerator
                // (slotUsedMs) share the same basis.  Using usedDur.GetTimeStep()
                // risks int64_t × double truncation that shifts the ratio by ~0.5%.
                double usableMs   = m_cfg.slotDurationMs - m_cfg.switchingTimeMs;
                Time   actualUsed = MilliSeconds(usableMs * utilFrac);
                m_metrics->OnSlotDeactivated(satId, beamId, actualUsed);
            }
            auto toggleIt = m_beamToggleMap.find({satId, beamId});
            bool mapped = (toggleIt != m_beamToggleMap.end());
            bool toggled = false;
            if (mapped)
            {
                toggleIt->second->ToggleState(false);
                toggled = true;
            }
            else
            {
                std::cout << "[BH-TOGGLEMAP][MISS] action=deactivate"
                          << " sat=" << satId
                          << " beam=" << beamId
                          << " mappedCount=" << m_beamToggleMap.size()
                          << std::endl;
            }
            EmitTrafficTrace("EVENT",
                             Simulator::Now().GetSeconds(),
                             beamId,
                             "DEACTIVATE",
                             m_lastDynamicPlan ? m_lastDynamicPlan->GetPlanId() : 0,
                             -1,
                             0.0,
                             usedDur.GetMilliSeconds(),
                             "",
                             mapped,
                             toggled);
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

    // Override BeamActivateCallback to add CacheQueue dequeue while preserving the
    // ToggleState wiring that SetupObc() relies on.  The toggle map is checked at
    // call-time so this lambda works regardless of whether Phase E is active.
    if (m_obc)
    {
        m_obc->SetBeamActivateCallback(
            [this](uint32_t satId, uint32_t beamId, Time usableDur) {
                if (m_metrics)
                    m_metrics->OnSlotActivated(satId, beamId, usableDur);
                if (m_cacheQueue)
                    m_cacheQueue->DequeueAll(beamId);
                auto toggleIt = m_beamToggleMap.find({satId, beamId});
                if (toggleIt != m_beamToggleMap.end())
                {
                    toggleIt->second->ToggleState(true);
                }
                else
                {
                    std::cout << "[BH-TOGGLEMAP][MISS] action=cache-activate"
                              << " sat=" << satId
                              << " beam=" << beamId
                              << " mappedCount=" << m_beamToggleMap.size()
                              << std::endl;
                }
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

// ── Phase C setup ────────────────────────────────────────────────────────
//
// Wiring order:
//   1. SatL1RoutingInterface  — stub (Enabled=false); Phase E activates ISL queries
//   2. SatUserAssociator      — WFQ/Priority/RR; MoveUtCallback unwired until Phase E
//   3. SatResourceManager     — wired to scheduler + associator + L1;
//                               Initialize() fires DoInitialize() which schedules
//                               the first RunFrameOptimization at simulation time 0
//
void
SatBhHelper::SetupPhaseC()
{
    NS_LOG_INFO("SatBhHelper::SetupPhaseC (Phase C active)");

    // 1. L1 routing interface (stub)
    m_l1Interface = CreateObject<SatL1RoutingInterface>();
    m_l1Interface->SetAttribute("Enabled", BooleanValue(m_cfg.enablePatternSelection));
    // enablePatternSelection doubles as the L1-protection gate until Phase E
    // adds a dedicated flag; default false keeps it as a pure stub.
    m_l1Interface->Initialize();

    NS_LOG_DEBUG("SatBhHelper::SetupPhaseC: L1 interface created (stub)");

    // 2. User associator
    m_associator = CreateObject<SatUserAssociator>();
    m_associator->SetSatId(m_cfg.satId);
    m_associator->SetAttribute("SchedulingMode",
                               UintegerValue(m_cfg.schedulingMode));
    m_associator->SetAttribute("MaxReassignmentPerFrame",
                               UintegerValue(m_cfg.maxReassignmentPerFrame));
    m_associator->SetAttribute("MaxDelayMs",
                               DoubleValue(m_cfg.maxDelayMs));
    // MoveUtCallback is wired in Phase E when SatNcc pointer is available.
    // Until then, ApplyAssociation() logs intended moves without calling SNS3.
    m_associator->Initialize();

    NS_LOG_INFO("SatBhHelper::SetupPhaseC: UserAssociator created"
                << " mode=" << static_cast<int>(m_cfg.schedulingMode)
                << " maxReassign=" << m_cfg.maxReassignmentPerFrame);

    // 3. Resource manager — self-scheduling loop
    m_resourceManager = CreateObject<SatResourceManager>();
    m_resourceManager->SetAttribute("FrameDuration",
                                    DoubleValue(m_cfg.bhtpPeriodMs));
    m_resourceManager->SetAttribute("EnableUserAssociation",
                                    BooleanValue(m_cfg.enableUserAssociation));
    m_resourceManager->SetAttribute("EnablePatternSelection",
                                    BooleanValue(m_cfg.enablePatternSelection));
    m_resourceManager->SetAttribute("NominalKbpsPerSlot",
                                    DoubleValue(m_cfg.nominalKbpsPerSlot));
    m_resourceManager->SetAttribute("SatId",
                                    UintegerValue(m_cfg.satId));

    m_resourceManager->SetScheduler(m_scheduler);       // null if Phase 2 disabled; handled internally
    m_resourceManager->SetUserAssociator(m_associator);
    m_resourceManager->SetL1Interface(m_l1Interface);

    if (m_frameConfigCb)
        m_resourceManager->SetFrameConfigCallback(m_frameConfigCb);

    // Initialize triggers DoInitialize() → Simulator::Schedule(t=0, RunFrameOptimization)
    m_resourceManager->Initialize();

    NS_LOG_INFO("SatBhHelper::SetupPhaseC: ResourceManager initialized"
                << " T_frame=" << m_cfg.bhtpPeriodMs << "ms"
                << " enableUserAssoc=" << m_cfg.enableUserAssociation);
}

// ── Phase D setup ────────────────────────────────────────────────────────
//
// Wiring order:
//   1. Create SatPowerAllocator with attributes from BhExperimentConfig
//   2. Wire into SatResourceManager via SetPowerAllocator()
//   3. Enable power allocation loop via EnablePowerAllocation attribute
//   4. ApplyPowerCallback left unwired until Phase E (SatOrbiterUserPhy hook)
//
void
SatBhHelper::SetupPhaseD()
{
    NS_LOG_INFO("SatBhHelper::SetupPhaseD (Phase D active)");

    // 1. Create power allocator
    m_powerAllocator = CreateObject<SatPowerAllocator>();
    m_powerAllocator->SetAttribute("TotalPowerBudgetDbm",
                                   DoubleValue(m_cfg.totalPowerBudgetDbm));
    m_powerAllocator->SetAttribute("NoisePowerDbw",
                                   DoubleValue(m_cfg.noisePowerDbw));
    m_powerAllocator->SetAttribute("MaxIterations",
                                   UintegerValue(m_cfg.powerMaxIterations));
    m_powerAllocator->SetAttribute("ConvergenceEpsilon",
                                   DoubleValue(m_cfg.powerConvergenceEps));
    m_powerAllocator->SetAttribute("InterferenceFactor",
                                   DoubleValue(m_cfg.interferenceFactor));
    m_powerAllocator->Initialize();

    // ApplyPowerCallback is wired in Phase E when SatOrbiterUserPhy pointer
    // is available via SatTopology.  Until then, ApplyPower() logs only.

    // 2. Wire into ResourceManager
    m_resourceManager->SetPowerAllocator(m_powerAllocator);
    m_resourceManager->SetAttribute("EnablePowerAllocation", BooleanValue(true));

    NS_LOG_INFO("SatBhHelper::SetupPhaseD: PowerAllocator wired into ResourceManager"
                << " P_total=" << m_cfg.totalPowerBudgetDbm << "dBm"
                << " N0=" << m_cfg.noisePowerDbw << "dBW"
                << " maxIter=" << m_cfg.powerMaxIterations
                << " eps=" << m_cfg.powerConvergenceEps
                << " ici=" << m_cfg.interferenceFactor);
}

// ── Phase E: UT address map builder ──────────────────────────────────────
//
// For each UT node in SatTopology, find the first SatNetDevice and store its
// MAC address keyed by the 0-based container index.
// The same index is used as utId in PollUtStates() and in MoveUtCallback.
//
void
SatBhHelper::BuildUtAddressMap()
{
    m_utAddressMap.clear();
    m_satMapperIdToContainerIdx.clear(); // Phase F: rebuild together with address map

    NodeContainer utNodes = Singleton<SatTopology>::Get()->GetUtNodes();
    NS_LOG_INFO("SatBhHelper::BuildUtAddressMap: scanning " << utNodes.GetN() << " UT nodes");

    for (uint32_t i = 0; i < utNodes.GetN(); i++)
    {
        Ptr<Node> utNode = utNodes.Get(i);
        for (uint32_t d = 0; d < utNode->GetNDevices(); d++)
        {
            Ptr<SatNetDevice> satDev = DynamicCast<SatNetDevice>(utNode->GetDevice(d));
            if (satDev)
            {
                Address addr = satDev->GetAddress();
                m_utAddressMap[i] = addr;

                // Phase F: build reverse map SatIdMapper UT ID -> container index.
                // BacklogRequestsTrace embeds the SatIdMapper UT ID (GetUtIdWithMac),
                // which differs from the container index used by PollUtStates.
                int32_t satMapperId = Singleton<SatIdMapper>::Get()->GetUtIdWithMac(addr);
                if (satMapperId >= 0)
                {
                    m_satMapperIdToContainerIdx[satMapperId] = i;
                    NS_LOG_DEBUG("SatBhHelper::BuildUtAddressMap: utId=" << i
                                 << " satMapperId=" << satMapperId
                                 << " nodeId=" << utNode->GetId()
                                 << " addr=" << addr);
                }
                else
                {
                    NS_LOG_WARN("SatBhHelper::BuildUtAddressMap: utId=" << i
                                << " addr=" << addr
                                << " not yet registered in SatIdMapper"
                                << " — Phase F demand trace will miss this UT");
                }
                break; // one SatNetDevice per UT is sufficient
            }
        }
        if (m_utAddressMap.find(i) == m_utAddressMap.end())
            NS_LOG_WARN("SatBhHelper::BuildUtAddressMap: no SatNetDevice on UT node "
                        << utNode->GetId() << " (utId=" << i << ")");
    }

    NS_LOG_INFO("SatBhHelper::BuildUtAddressMap: mapped " << m_utAddressMap.size()
                << " of " << utNodes.GetN() << " UTs"
                << "; satMapperIdToContainerIdx entries=" << m_satMapperIdToContainerIdx.size());
}

// ── Phase E: orbiter device cache ─────────────────────────────────────────
//
// Iterates devices on the configured satellite node to find the
// SatOrbiterNetDevice, then caches it for use in ApplyPowerCallback.
//
void
SatBhHelper::CacheOrbiterDevice()
{
    m_orbiterDev = nullptr;

    Ptr<Node> satNode = Singleton<SatTopology>::Get()->GetOrbiterNode(m_cfg.satId);
    if (!satNode)
    {
        NS_LOG_WARN("SatBhHelper::CacheOrbiterDevice: orbiter node not found for satId="
                    << m_cfg.satId);
        return;
    }

    for (uint32_t d = 0; d < satNode->GetNDevices(); d++)
    {
        m_orbiterDev = DynamicCast<SatOrbiterNetDevice>(satNode->GetDevice(d));
        if (m_orbiterDev)
        {
            NS_LOG_INFO("SatBhHelper::CacheOrbiterDevice: found SatOrbiterNetDevice"
                        " on sat " << m_cfg.satId
                        << " (device index=" << d << ")");
            return;
        }
    }

    NS_LOG_WARN("SatBhHelper::CacheOrbiterDevice: SatOrbiterNetDevice not found on sat "
                << m_cfg.satId << " — power callback will be log-only");
}

// ── Phase E: beam toggle map ───────────────────────────────────────────────
//
// Iterates all GW nodes in SatTopology and collects the SatNetDevice for each
// beam served by this helper's satellite.  The same ToggleState(bool) callback that
// SatBstpController::AddNetDeviceCallback() stores is accessed directly here so
// that OBC BeamActivate/DeactivateCallbacks can replicate the original BH flow:
//
//   SatBstpController::DoBstpConfiguration() → ToggleState(true/false)
//          ↕ (replaced by)
//   SatBhObc::EnterSlot / OnSlotServiceEnd   → ToggleState(true/false)
//
// NOTE: Uses SatMac::GetSatId() and SatMac::GetBeamId() — SatNetDevice has no GetBeamId().
// NOTE: SatTopology::GetGwNodes() — confirmed present in satellite-topology.h.
//
void
SatBhHelper::BuildBeamToggleMap()
{
    m_beamToggleMap.clear();

    Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
    if (!topo)
    {
        NS_LOG_WARN("SatBhHelper::BuildBeamToggleMap: SatTopology unavailable"
                    " — ToggleState not wired; simulation runs in metrics-only mode");
        return;
    }

    NodeContainer gwNodes = topo->GetGwNodes();
    uint32_t mapped = 0;
    std::set<uint32_t> mappedBeams;

    for (uint32_t i = 0; i < gwNodes.GetN(); i++)
    {
        Ptr<Node> gwNode = gwNodes.Get(i);
        for (uint32_t d = 0; d < gwNode->GetNDevices(); d++)
        {
            Ptr<SatNetDevice> dev = DynamicCast<SatNetDevice>(gwNode->GetDevice(d));
            if (!dev)
                continue;

            // SatNetDevice does not expose GetBeamId() directly.
            // Both satId and beamId live on the MAC layer (SatMac).
            Ptr<SatMac> mac = dev->GetMac();
            if (!mac)
                continue;

            uint32_t satId  = mac->GetSatId();
            uint32_t beamId = mac->GetBeamId();
            if (beamId == 0)
                continue; // loopback or non-beam device
            if (satId != m_cfg.satId)
                continue; // this helper controls one satellite only

            // Key = {satId, beamId}: a flat beamId key would collide across satellites
            // (e.g. sat0/beam1 and sat1/beam1 are different physical links).
            auto key = std::make_pair(satId, beamId);
            if (m_beamToggleMap.count(key))
            {
                NS_LOG_DEBUG("SatBhHelper::BuildBeamToggleMap: duplicate GW device for"
                             " sat=" << satId << " beam=" << beamId << " — keeping first");
                continue;
            }

            m_beamToggleMap[key] = dev;
            mappedBeams.insert(beamId);
            mapped++;

            NS_LOG_INFO("SatBhHelper::BuildBeamToggleMap: sat=" << satId
                        << " beam=" << beamId
                        << " -> GW nodeId=" << gwNode->GetId()
                        << " deviceIdx=" << d);
        }
    }

    NS_LOG_INFO("SatBhHelper::BuildBeamToggleMap: " << mapped
                << " beam(s) mapped for ToggleState wiring"
                << " (total GW nodes scanned=" << gwNodes.GetN() << ")");

    std::ostringstream mappedOss;
    for (auto it = mappedBeams.begin(); it != mappedBeams.end(); ++it)
    {
        if (it != mappedBeams.begin())
            mappedOss << ",";
        mappedOss << *it;
    }

    std::ostringstream missingOss;
    uint32_t missing = 0;
    for (uint32_t beamId = 1; beamId <= m_cfg.numBeams; ++beamId)
    {
        if (mappedBeams.count(beamId) == 0)
        {
            if (missing > 0)
                missingOss << ",";
            missingOss << beamId;
            missing++;
        }
    }

    std::cout << "[BH-TOGGLEMAP] sat=" << m_cfg.satId
              << " gwNodesScanned=" << gwNodes.GetN()
              << " mappedCount=" << mappedBeams.size()
              << "/" << m_cfg.numBeams
              << " mappedBeams=[" << mappedOss.str() << "]"
              << " missingCount=" << missing
              << " missingBeams=[" << missingOss.str() << "]"
              << std::endl;

    // SatNetDevice defaults to enabled.  When we suppress SatBstpController,
    // there is no initial static pass to disable beams outside the active set.
    // Start all beams controlled by this helper in the disabled state; OBC
    // EnterSlot then enables only the selected beams and OnSlotServiceEnd
    // disables them again.
    for (auto& kv : m_beamToggleMap)
    {
        kv.second->ToggleState(false);
    }
    NS_LOG_INFO("SatBhHelper::BuildBeamToggleMap: initialized mapped beams to disabled"
                " for sat=" << m_cfg.satId);
}

// ── Phase E: callback wiring ──────────────────────────────────────────────
//
// Wiring order:
//   1. BuildUtAddressMap()  — container index → MAC Address
//   2. CacheOrbiterDevice() — satellite's SatOrbiterNetDevice
//      (BuildBeamToggleMap was already called in Install() when OBC was created)
//   3. Wire MoveUtCallback → SatNcc::MoveUtBetweenBeams
//   4. Wire ApplyPowerCallback → SatOrbiterUserPhy::SetTxMaxPowerDbw + Initialize
//   5. Schedule PollUtStates() at t=0 (every T_frame thereafter)
//
void
SatBhHelper::ConnectTracesPhaseE()
{
    NS_LOG_INFO("SatBhHelper::ConnectTracesPhaseE (Phase E active)");

    // 1 + 2: Build address map and cache orbiter device
    BuildUtAddressMap();
    CacheOrbiterDevice();
    // NOTE: BuildBeamToggleMap() was called earlier in Install() when OBC was created,
    // so m_beamToggleMap is already populated here.

    // 3. Wire MoveUtCallback → SatNcc::MoveUtBetweenBeams
    //    Address lookup: m_utAddressMap[utId] (container index) → MAC address
    if (m_associator && m_simHelper)
    {
        Ptr<SatNcc> ncc =
            m_simHelper->GetSatelliteHelper()->GetBeamHelper()->GetNcc();

        if (ncc)
        {
            m_associator->SetMoveUtCallback(
                [this, ncc](uint32_t utId,
                            uint32_t srcSatId, uint32_t srcBeamId,
                            uint32_t dstSatId, uint32_t dstBeamId) {
                    auto it = m_utAddressMap.find(utId);
                    if (it == m_utAddressMap.end())
                    {
                        NS_LOG_WARN("[PhaseE] MoveUtCallback: utId=" << utId
                                    << " not in address map — ignored");
                        return;
                    }
                    NS_LOG_INFO("[PhaseE] MoveUtBetweenBeams: ut=" << it->second
                                << " sat " << srcSatId << " beam " << srcBeamId
                                << " -> sat " << dstSatId << " beam " << dstBeamId);
                    ncc->MoveUtBetweenBeams(it->second,
                                            srcSatId, srcBeamId,
                                            dstSatId, dstBeamId);
                });

            NS_LOG_INFO("SatBhHelper::ConnectTracesPhaseE:"
                        " MoveUtCallback wired to SatNcc");
        }
        else
        {
            NS_LOG_WARN("SatBhHelper::ConnectTracesPhaseE:"
                        " SatNcc not available — MoveUtCallback stays log-only");
        }
    }

    // 5. Wire ApplyPowerCallback → SatPhy::SetTxMaxPowerDbw + Initialize
    //    SatPhy::Initialize() recomputes m_eirpWoGainW from the new TxMaxPowerDbw.
    //    The SatPhy::Initialize() method is NOT the ns-3 Object lifecycle
    //    Initialize(); it is SatPhy's own setup method that is safe to call
    //    at any simulation time to apply a new power level.
    if (m_powerAllocator && m_orbiterDev)
    {
        Ptr<SatOrbiterNetDevice> dev = m_orbiterDev;
        m_powerAllocator->SetApplyPowerCallback(
            [dev](uint32_t beamId, double txMaxDbw) {
                Ptr<SatPhy> phy = dev->GetUserPhy(beamId);
                if (!phy)
                {
                    NS_LOG_WARN("[PhaseE] ApplyPowerCallback: no UserPhy for beamId="
                                << beamId);
                    return;
                }
                phy->SetTxMaxPowerDbw(txMaxDbw);
                phy->Initialize(); // SatPhy::Initialize recomputes m_eirpWoGainW
                NS_LOG_INFO("[PhaseE] ApplyPower: beam=" << beamId
                            << " txMaxPowerDbw=" << txMaxDbw << "dBW");
            });

        NS_LOG_INFO("SatBhHelper::ConnectTracesPhaseE:"
                    " ApplyPowerCallback wired to SatOrbiterUserPhy");
    }
    else if (m_powerAllocator && !m_orbiterDev)
    {
        NS_LOG_WARN("SatBhHelper::ConnectTracesPhaseE:"
                    " ApplyPowerCallback not wired (orbiter device not found);"
                    " power allocation continues in log-only mode");
    }

    // 6. Start UT state polling loop at t=0
    if (m_resourceManager)
    {
        NS_LOG_INFO("SatBhHelper::ConnectTracesPhaseE: scheduling PollUtStates()"
                    " every T_frame=" << m_cfg.bhtpPeriodMs << "ms");
        Simulator::Schedule(Seconds(0.0), &SatBhHelper::PollUtStates, this);
    }
}

// ── Phase E: UT state polling ─────────────────────────────────────────────
//
// Runs every T_frame.  For each UT on the configured satellite, reads the
// current beam from SatTopology and updates ResourceManager state.
//
// Demand values are synthetic (1000 kbps) until Phase F wires real DAMA
// request queue sizes from SatGwMac trace sources.
//
void
SatBhHelper::PollUtStates()
{
    if (!m_resourceManager)
        return;

    auto* topo = Singleton<SatTopology>::Get();
    NodeContainer utNodes = topo->GetUtNodes();

    uint32_t updatedCount = 0;
    for (uint32_t i = 0; i < utNodes.GetN(); i++)
    {
        Ptr<Node> utNode = utNodes.Get(i);

        // Filter: only manage UTs currently on the configured satellite
        uint32_t utSatId = topo->GetUtSatId(utNode);
        if (utSatId != m_cfg.satId)
            continue;

        uint32_t beamId = topo->GetUtBeamId(utNode);

        UtInfo info;
        info.utId            = i;            // 0-based container index, matches m_utAddressMap
        info.currentBeamId   = beamId;
        // Phase F: use real RBDC demand from BacklogRequestsTrace cache.
        // Falls back to 0.0 when no trace has arrived yet (e.g., during warm-up or
        // when Phase F is disabled).  0.0 is intentionally conservative so the
        // ResourceManager does not over-allocate slots to idle UTs.
        {
            auto dit = m_utDemandCache.find(i);
            info.demandKbps = (dit != m_utDemandCache.end()) ? dit->second.totalRbdcKbps : 0.0;
        }
        info.bufferBytes     = 0.0;
        info.cno             = 0.0;
        info.priorityClass   = 1;
        info.headOfLineDelayMs = 0.0;

        m_resourceManager->UpdateUtState(info);
        updatedCount++;
    }

    NS_LOG_DEBUG("SatBhHelper::PollUtStates: updated " << updatedCount
                 << " UTs on sat " << m_cfg.satId
                 << " t=" << Simulator::Now().GetSeconds() << "s");

    // Reschedule for next frame
    Simulator::Schedule(MilliSeconds(m_cfg.bhtpPeriodMs),
                        &SatBhHelper::PollUtStates, this);
}

// ── Phase F: DAMA demand trace wiring ────────────────────────────────────
//
// ConnectTracesPhaseF() connects BacklogRequestsTrace on every SatBeamScheduler.
// The trace fires once per UT per Return Channel (RC) each DAMA scheduling cycle,
// with a record string: "time, beamId, satMapperUtId, typeEnum, value"
//   typeEnum: DA_RBDC=1 (kbps), DA_VBDC=2 (bytes)
//
// OnBacklogRequestTrace() accumulates RBDC kbps per containerIdx into
// m_utDemandCache.  When a new timestamp is seen for the same UT, the
// accumulator resets (new scheduling cycle), so multi-RC totals are correct.
//
// PollUtStates() reads m_utDemandCache instead of the synthetic 1000 kbps stub.
//
void
SatBhHelper::ConnectTracesPhaseF()
{
    NS_LOG_INFO("SatBhHelper::ConnectTracesPhaseF (Phase F active)");

    Ptr<SatBeamHelper> beamHelper =
        m_simHelper->GetSatelliteHelper()->GetBeamHelper();
    if (!beamHelper)
    {
        NS_LOG_WARN("SatBhHelper::ConnectTracesPhaseF: SatBeamHelper not available"
                    " — Phase F skipped");
        return;
    }

    Ptr<SatNcc> ncc = beamHelper->GetNcc();
    if (!ncc)
    {
        NS_LOG_WARN("SatBhHelper::ConnectTracesPhaseF: SatNcc not available"
                    " — Phase F skipped");
        return;
    }

    std::list<std::pair<uint32_t, uint32_t>> beams = beamHelper->GetBeams();
    uint32_t connected = 0;

    // Only connect beam schedulers belonging to this helper's satellite (m_cfg.satId).
    // GetBeams() returns all (satId, beamId) pairs across the entire constellation;
    // without filtering, every SatBhHelper would register on all 4752 beams and
    // OnBacklogRequestTrace would fire 66× per event, polluting each helper's cache.
    uint32_t totalBeams = 0;
    for (const auto& [satId, beamId] : beams)
    {
        if (satId != m_cfg.satId)
            continue; // skip beams belonging to other satellites

        totalBeams++;
        Ptr<SatBeamScheduler> sched = ncc->GetBeamScheduler(satId, beamId);
        if (!sched)
        {
            NS_LOG_WARN("SatBhHelper::ConnectTracesPhaseF: no scheduler for sat="
                        << satId << " beam=" << beamId);
            continue;
        }

        // TraceConnectWithoutContext: callback receives only the record string.
        // MakeCallback generates a ns3::Callback<void,std::string> compatible with
        // the BacklogRequestsTraceCallback typedef void(*)(std::string).
        bool ok = sched->TraceConnectWithoutContext(
            "BacklogRequestsTrace",
            MakeCallback(&SatBhHelper::OnBacklogRequestTrace, this));

        if (ok)
        {
            connected++;
            NS_LOG_DEBUG("SatBhHelper::ConnectTracesPhaseF: connected sat="
                         << satId << " beam=" << beamId);
        }
        else
        {
            NS_LOG_WARN("SatBhHelper::ConnectTracesPhaseF: TraceConnectWithoutContext"
                        " failed for sat=" << satId << " beam=" << beamId);
        }
    }

    NS_LOG_INFO("SatBhHelper::ConnectTracesPhaseF: sat=" << m_cfg.satId
                << " connected BacklogRequestsTrace on " << connected
                << " / " << totalBeams << " own-satellite beam schedulers"
                << " (total constellation beams=" << beams.size() << ")");
}

void
SatBhHelper::OnBacklogRequestTrace(std::string record)
{
    // In FWD offered-demand mode, scheduler demand comes from the configured
    // FWD offered load. RBDC is a separate RTN DAMA signal and would duplicate
    // the demand model for this experiment, so do not feed or trace it here.
    if (m_cfg.enableFwdOfferedDemand)
    {
        return;
    }

    // Record format: "time, beamId, satMapperUtId, typeEnum, value"
    //   typeEnum: DA_RBDC=1 (kbps), DA_VBDC=2 (bytes) — only RBDC is used here.
    double   timeS;
    uint32_t beamId;
    int32_t  satMapperUtId;
    int      typeEnum;
    double   value;
    char     comma;

    std::istringstream iss(record);
    if (!(iss >> timeS >> comma
              >> beamId >> comma
              >> satMapperUtId >> comma
              >> typeEnum >> comma
              >> value))
    {
        NS_LOG_WARN("SatBhHelper::OnBacklogRequestTrace: failed to parse record: "
                    << record);
        return;
    }

    // Skip VBDC entries (typeEnum=2); only accumulate RBDC (typeEnum=1).
    if (typeEnum != static_cast<int>(SatEnums::DA_RBDC))
        return;

    // Map SatIdMapper UT ID to container index (built in BuildUtAddressMap).
    auto it = m_satMapperIdToContainerIdx.find(satMapperUtId);
    if (it == m_satMapperIdToContainerIdx.end())
    {
        NS_LOG_DEBUG("SatBhHelper::OnBacklogRequestTrace: satMapperUtId=" << satMapperUtId
                     << " not in reverse map — UT may not be on managed satellite");
        return;
    }

    uint32_t containerIdx = it->second;
    auto& entry = m_utDemandCache[containerIdx];

    if (entry.lastTimestampS == timeS)
    {
        // Same scheduling cycle: sum across Return Channels.
        entry.totalRbdcKbps += value;
    }
    else
    {
        // New scheduling cycle: reset accumulator.
        entry.totalRbdcKbps  = value;
        entry.lastTimestampS = timeS;
    }

    NS_LOG_DEBUG("SatBhHelper::OnBacklogRequestTrace: containerIdx=" << containerIdx
                 << " t=" << timeS << "s beam=" << beamId
                 << " rbdc=" << entry.totalRbdcKbps << "kbps");

    EmitTrafficTrace("DEMAND",
                     timeS,
                     beamId,
                     "RBDC",
                     m_lastDynamicPlan ? m_lastDynamicPlan->GetPlanId() : 0,
                     -1,
                     entry.totalRbdcKbps,
                     0.0,
                     "",
                     false,
                     false);

    // Phase G: forward accumulated per-beam demand to the dynamic provider so
    // its scoring function reflects real RBDC values instead of the 0.0 fallback.
    // m_dynamicProvider is null when Phase G is not active — check before calling.
    if (m_dynamicProvider)
        m_dynamicProvider->UpdateBeamDemand(beamId, entry.totalRbdcKbps);
}

// ── Phase G: Dynamic BSTP Provider ───────────────────────────────────────

void
SatBhHelper::SetupDynamicBstp()
{
    NS_LOG_INFO("SatBhHelper::SetupDynamicBstp (Phase G active)");

    // Create the greedy provider and configure its scoring weights / limits.
    Ptr<SatGreedyBstpProvider> provider = CreateObject<SatGreedyBstpProvider>();
    provider->SetAttribute("MaxActiveBeams",
                           UintegerValue(m_cfg.maxActiveBeams));
    provider->SetAttribute("BacklogWeight",
                           DoubleValue(m_cfg.bhDemandBacklogWeight));
    provider->SetAttribute("FairnessWeight",
                           DoubleValue(m_cfg.bhFairnessWeight));
    provider->SetAttribute("ValidityInSuperframes",
                           UintegerValue(m_cfg.bhValiditySuperframes));
    provider->SetAttribute("StarvationThreshold",
                           UintegerValue(m_cfg.bhStarvationThreshold));

    m_dynamicProvider = provider;

    // Register all beams that belong to the configured satellite.
    // SatTopology is a Singleton populated by SimulationHelper::CreateSatScenario().
    // When SatTopology is not available (unit-test / standalone scenarios), fall back
    // to registering beams 1..numBeams with placeholder frequency/GW info.
    bool topologyAvailable = false;
    try
    {
        Ptr<SatTopology> topo = Singleton<SatTopology>::Get();
        if (topo)
        {
            // Iterate all beams on our satellite.
            // SatTopology does not expose a direct beam-info iterator with freq IDs,
            // so we iterate UT nodes and derive beam info from their assignment.
            // Freq IDs are set to 0 as placeholders (constraint checker still works
            // for gw/feeder-freq collisions when Phase E is active and real IDs known).
            NodeContainer utNodes = topo->GetUtNodes();
            std::set<uint32_t> registeredBeams;

            for (uint32_t i = 0; i < utNodes.GetN(); i++)
            {
                Ptr<Node> ut = utNodes.Get(i);
                if (topo->GetUtSatId(ut) != m_cfg.satId)
                    continue;

                uint32_t beamId = topo->GetUtBeamId(ut);
                if (registeredBeams.count(beamId))
                    continue; // already registered

                // Placeholder freq IDs — updated by SetupPhaseE if available
                provider->AddEnabledBeamInfo(beamId, 0, beamId, 0);
                registeredBeams.insert(beamId);
            }

            NS_LOG_INFO("SatBhHelper::SetupDynamicBstp: registered "
                        << registeredBeams.size()
                        << " beams from SatTopology for sat=" << m_cfg.satId);
            topologyAvailable = !registeredBeams.empty();
        }
    }
    catch (...)
    {
        topologyAvailable = false;
    }

    if (!topologyAvailable)
    {
        // Fallback: register beams 1..numBeams with sequential placeholder IDs.
        // Useful for standalone unit tests where SatTopology is not populated.
        NS_LOG_WARN("SatBhHelper::SetupDynamicBstp: SatTopology unavailable or empty"
                    " — registering beams 1.." << m_cfg.numBeams << " as placeholders");
        for (uint32_t b = 1; b <= m_cfg.numBeams; b++)
            provider->AddEnabledBeamInfo(b, 0, b, 0);
    }

    // ── FWD offered-demand injection (Phase G scheduler input) ────────────
    // FWD CBR offered load is the demand the BH scheduler should react to.
    // Packet Rx is the output/validation path, so do not feed Rx back into the
    // scheduler.  Inject before the first dynamic cycle so plan #1 can use it.
    if (m_cfg.enableFwdOfferedDemand && m_cfg.fwdOfferedDemandKbps > 0.0)
    {
        Simulator::Schedule(Seconds(0.0), &SatBhHelper::InjectFwdDemand, this);

        NS_LOG_INFO("SatBhHelper::SetupDynamicBstp: FWD offered-demand injection armed"
                    " — " << m_cfg.numBeams << " beams × "
                    << m_cfg.fwdOfferedDemandKbps
                    << " kbps, period=" << m_cfg.bhtpPeriodMs << "ms");
    }

    // Schedule first cycle immediately (t = 0).
    // Subsequent cycles self-schedule inside RunDynamicBstpCycle().
    Simulator::Schedule(Seconds(0.0),
                        &SatBhHelper::RunDynamicBstpCycle, this, Seconds(0.0));
}

void
SatBhHelper::RunDynamicBstpCycle(Time now)
{
    NS_ASSERT_MSG(m_dynamicProvider,
                  "SatBhHelper::RunDynamicBstpCycle called without a provider");

    // Flush KPIs for the just-completed T_p, delayed by T_prop.
    // OBC slot 7 runs from t=T_p to t=T_p+T_usable (8 ms after NCC cycle start).
    // Without the delay, FlushMetrics fires at the NCC boundary (t=T_p) before
    // BeamDeactivateCallback for slot 7, causing its 8 ms dwell to be silently
    // dropped (dwell=56ms instead of 64ms) and time_s to lag T_prop behind the
    // true period end. Delaying by T_prop ensures the flush fires at t=T_p+T_prop
    // (= the OBC period transition), capturing all 8 slots and aligning time_s
    // with the actual OBC execution boundary.
    if (m_metrics)
        Simulator::Schedule(MilliSeconds(m_cfg.propagationDelayMs),
                            &SatBhMetrics::FlushMetrics, m_metrics);

    SatDynamicBstpProvider::Conf conf = m_dynamicProvider->GetNextConf(now);

    // Validate before converting — use static plan as fallback on failure.
    if (!m_dynamicProvider->ValidateConf(conf))
    {
        NS_LOG_WARN("SatBhHelper::RunDynamicBstpCycle: ValidateConf failed at t="
                    << now.GetSeconds() << "s — keeping previous plan");
    }
    else if (conf.activeBeams.empty())
    {
        NS_LOG_WARN("SatBhHelper::RunDynamicBstpCycle: provider returned empty activeBeams"
                    " — keeping previous plan");
    }
    else
    {
        // When OBC is active, bake propagation delay into the plan's period start so
        // OBC::ReceiveNewPlan schedules EnterSlot(0) at now+T_prop, not immediately.
        // The original SatBhScheduler path does the same (BuildPlan adds T_prop to
        // periodStart); ConfToTimePlan was using bare `now`, which caused the dynamic
        // plan to fire with zero delay regardless of m_cfg.propagationDelayMs.
        Time periodStart = m_obc
            ? (now + MilliSeconds(m_cfg.propagationDelayMs))
            : now;

        Ptr<SatBhTimePlan> plan = ConfToTimePlan(conf, periodStart);

        if (plan)
        {
            // Track the most recent dynamic plan so GetCurrentPlan() returns it.
            // Without this, GetCurrentPlan() would return the stale m_staticPlan
            // even after Phase G has been running for many periods.
            m_lastDynamicPlan = plan;

            const auto& slots = plan->GetSlots();
            for (uint32_t slotIdx = 0; slotIdx < slots.size(); ++slotIdx)
            {
                const auto& slot = slots[slotIdx];
                std::ostringstream beams;
                for (uint32_t i = 0; i < slot.beamIds.size(); ++i)
                {
                    if (i > 0)
                    {
                        beams << "|";
                    }
                    beams << slot.beamIds[i];
                }

                EmitTrafficTrace("PLAN",
                                 periodStart.GetSeconds() + slot.startTime.GetSeconds(),
                                 0,
                                 "SLOT",
                                 plan->GetPlanId(),
                                 static_cast<int32_t>(slotIdx),
                                 0.0,
                                 slot.duration.GetMilliSeconds(),
                                 beams.str(),
                                 false,
                                 false);
            }

            if (m_obc)
            {
                // propagationDelay arg is now informational only (already in periodStart)
                m_obc->ReceiveNewPlan(plan, MilliSeconds(m_cfg.propagationDelayMs));
            }
            else
            {
                // No OBC: update static plan reference used by the synthetic driver.
                m_staticPlan = plan;
            }

            NS_LOG_DEBUG("SatBhHelper::RunDynamicBstpCycle: new plan id="
                         << plan->GetPlanId()
                         << " beams=" << conf.activeBeams.size()
                         << " validity=" << conf.validityInSuperframes
                         << " periodStart=" << periodStart.GetMilliSeconds() << "ms"
                         << " t=" << now.GetSeconds() << "s");
        }
    }

    // Self-schedule next cycle at now + T_p.
    Time nextT = now + MilliSeconds(m_cfg.bhtpPeriodMs);
    Simulator::Schedule(MilliSeconds(m_cfg.bhtpPeriodMs),
                        &SatBhHelper::RunDynamicBstpCycle, this, nextT);
}

void
SatBhHelper::InjectFwdDemand()
{
    if (!m_dynamicProvider)
        return;

    if (!m_cfg.enableFwdOfferedDemand || m_cfg.fwdOfferedDemandKbps <= 0.0)
    {
        return;
    }

    std::set<uint32_t> hotspotBeams(m_cfg.bhFwdHotspotBeamIds.begin(),
                                    m_cfg.bhFwdHotspotBeamIds.end());

    for (uint32_t bid = 1; bid <= m_cfg.numBeams; ++bid)
    {
        double demandKbps = m_cfg.fwdOfferedDemandKbps;
        if (hotspotBeams.find(bid) != hotspotBeams.end())
        {
            demandKbps += m_cfg.bhFwdHotspotBoostKbps;
        }

        m_dynamicProvider->UpdateBeamDemand(bid, demandKbps);
        EmitTrafficTrace("DEMAND",
                         Simulator::Now().GetSeconds(),
                         bid,
                         "OFFERED_FWD",
                         m_lastDynamicPlan ? m_lastDynamicPlan->GetPlanId() : 0,
                         -1,
                         demandKbps,
                         0.0,
                         "",
                         false,
                         false);

        NS_LOG_DEBUG("SatBhHelper::InjectFwdDemand: sat=" << m_cfg.satId
                     << " beam=" << bid
                     << " offered=" << demandKbps << " kbps");
    }

    Simulator::Schedule(MilliSeconds(m_cfg.bhtpPeriodMs),
                        &SatBhHelper::InjectFwdDemand, this);
}

Ptr<SatBhTimePlan>
SatBhHelper::ConfToTimePlan(const SatDynamicBstpProvider::Conf& conf, Time periodStart)
{
    // Validate inputs
    if (conf.activeBeams.empty())
        return nullptr;

    Ptr<SatBhTimePlan> plan = CreateObject<SatBhTimePlan>();
    plan->SetPlanId(++m_dynamicPlanId);

    const Time T_s = MilliSeconds(m_cfg.slotDurationMs);
    const Time T_p = MilliSeconds(m_cfg.bhtpPeriodMs) *
                     static_cast<double>(conf.validityInSuperframes);

    plan->SetPeriodBounds(periodStart, periodStart + T_p);

    // Number of BHTP slots in one period
    const uint32_t M = static_cast<uint32_t>(
        std::ceil(m_cfg.bhtpPeriodMs * conf.validityInSuperframes / m_cfg.slotDurationMs));
    const uint32_t K = m_cfg.maxActiveBeams;

    const std::vector<uint32_t>& beams = conf.activeBeams;
    const uint32_t               B     = static_cast<uint32_t>(beams.size());

    // Distribute activeBeams across M slots using round-robin.
    // Each slot gets min(K, B) beams so constraint |beamIds| ≤ K is always met.
    // Example: activeBeams={1,4,7}, K=2, M=3 →
    //   slot 0: {1,4}, slot 1: {7,1}, slot 2: {4,7}
    uint32_t beamOffset = 0;
    for (uint32_t s = 0; s < M; s++)
    {
        BhSlotEntry slot;
        slot.startTime = s * T_s;
        slot.duration  = T_s;
        slot.modcod    = 5; // default DVB-S2X MODCOD

        uint32_t count = std::min(K, B);
        for (uint32_t k = 0; k < count; k++)
        {
            uint32_t bid = beams[(beamOffset + k) % B];
            slot.beamIds.push_back(bid);
            slot.clusterIds.push_back(bid);
            // Default pattern: SMALL for first beam (likely hotspot), LARGE otherwise
            slot.SetBeamPattern(bid, (k == 0) ? BeamRadiusType::SMALL
                                              : BeamRadiusType::LARGE);
        }
        beamOffset = (beamOffset + 1) % B;

        plan->AddSlot(slot);
    }

    // Validate the generated plan — log a warning if it fails but return it anyway
    // so the caller can decide whether to use it.
    if (!plan->Validate(K))
        NS_LOG_WARN("SatBhHelper::ConfToTimePlan: Validate() failed for planId="
                    << plan->GetPlanId());

    return plan;
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
