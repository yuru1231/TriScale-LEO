/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-helper.h
 *
 * SatBhHelper — Unified installer and orchestrator for the BH system.
 *
 * Implements spec Section 7.5 (SatBhHelper) and Layer2.md Section 7.5.
 * Implementation phase: Phase 4 (unified), but Phase 1 is operational now.
 *
 * Responsibilities:
 *   - Single entry point for all BH module installation
 *   - Create and wire all modules (TimePlan, Metrics, Scheduler, OBC,
 *     CacheQueue, Precoder) based on which phases are enabled
 *   - Connect SNS3 trace sources to module callbacks (non-invasive, Principle A/B)
 *   - Provide feature flags to enable/disable individual phases
 *   - Provide global parameter setters (alpha, kappa, K, MMSE, etc.)
 *   - Expose SatBhMetrics for external KPI access
 *
 * Phase activation:
 *   Phase 1 (always active): SatBhTimePlan + SatBhMetrics
 *     → static BHTP used when Scheduler disabled
 *     → synthetic metrics driver used when OBC disabled
 *   Phase 2 (opt-in): SatBhScheduler + SatBhObc
 *     → dynamic BHTP from EM algorithm
 *     → real slot-switching state machine
 *   Phase 3 (opt-in): SatGwCacheQueue + SatBhPrecoder
 *     → packet buffering during inactive beam periods
 *     → MMSE precoding for interfering clusters
 *
 * Hook connection points (to be confirmed in Hook feasibility check):
 *   DaRequestReceived   → SatBhScheduler::OnDemandReceived
 *   SlotAllocated       → slot boundary synchronization
 *   GwMac::Tx           → SatGwCacheQueue::Enqueue
 *   GwMac::Rx           → SatBhMetrics::OnPacketReceived
 *   ChannelEstimation   → SatBhPrecoder::OnChannelEstimateReceived
 *   HandoverCompleted   → queue migration (LEO multi-sat)
 *
 * Non-invasive constraint (Layer2.md Principle A/B):
 *   All SNS3 connections are done via:
 *     Config::ConnectWithoutContext()
 *     Object::AggregateObject()
 *     Simulator::Schedule() / trace callbacks
 *   No SNS3 source code is modified.
 */

#ifndef SAT_BH_HELPER_H
#define SAT_BH_HELPER_H

#include "sat-bh-metrics.h"
#include "sat-bh-obc.h"
#include "sat-bh-precoder.h"
#include "sat-bh-scheduler.h"
#include "sat-bh-time-plan.h"
#include "sat-gw-cache-queue.h"

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>

namespace ns3
{

// ── BhExperimentConfig ────────────────────────────────────────────────────
//
// Aggregates all experiment-level parameters into one object.
// Passed to SatBhHelper::Configure() before Install().
// This is the single point of truth for the example file.
//
struct BhExperimentConfig
{
    // ── Constellation / scenario ──────────────────────────────────────────
    uint32_t numBeams{7};           ///< Number of beams (basic: 7, full: 19 per sat)
    uint32_t maxActiveBeams{2};     ///< K (basic scenario: 2; full: 3)
    uint32_t numHotspotBeams{3};    ///< Number of hotspot beams (for synthetic driver)
    uint32_t satId{0};             ///< Satellite index to install on

    // ── Timing ────────────────────────────────────────────────────────────
    double simTimeSec{300.0};       ///< Total simulation duration [s]
    double warmUpSec{10.0};         ///< Warm-up period to discard [s] (spec Section 9)
    double slotDurationMs{26.5};    ///< T_s [ms]
    double bhtpPeriodMs{503.0};     ///< T_p [ms]
    double switchingTimeMs{2.0};    ///< T_sw [ms]
    double propagationDelayMs{10.0};///< T_prop [ms]

    // ── EM algorithm ──────────────────────────────────────────────────────
    double   emConvergenceEps{0.001};
    uint32_t emMaxIterations{50};
    uint32_t observationWindowPeriods{5};

    // ── Scheduling ────────────────────────────────────────────────────────
    double alphaDelaySensitivity{2.0};  ///< α (virtual traffic delay factor)
    double interferenceKappa{0.08};     ///< κ (cluster merge threshold)
    double nonHotspotPercentile{0.25};  ///< Hotspot split threshold
    double demandChangeThreshold{0.20}; ///< Early-trigger threshold

    // ── Cache queue ────────────────────────────────────────────────────────
    double maxQueueSizeMB{40.0};        ///< Per-beam queue capacity [MB]

    // ── Precoder ──────────────────────────────────────────────────────────
    double noisePowerDb{-110.0};        ///< Default noise power for MMSE

    // ── Feature flags (which phases to activate) ──────────────────────────
    bool enableScheduler{false};    ///< Phase 2: use SatBhScheduler (EM-based BHTP)
    bool enableObc{false};          ///< Phase 2: use SatBhObc (real slot switching)
    bool enableCacheQueue{false};   ///< Phase 3: use SatGwCacheQueue
    bool enablePrecoder{false};     ///< Phase 3: use SatBhPrecoder (MMSE)

    // ── Output ────────────────────────────────────────────────────────────
    std::string metricsOutputFile{"bh-metrics.csv"};
    std::string timePlanCsvFile{"bh-timeplan.csv"};
};

// ── SatBhHelper ───────────────────────────────────────────────────────────

class SatBhHelper : public Object
{
  public:
    static TypeId GetTypeId();
    SatBhHelper();

    // ── Configuration (call before Install) ───────────────────────────────

    /// Apply all parameters from a BhExperimentConfig struct
    void Configure(const BhExperimentConfig& cfg);

    /// Shortcut setters (alternative to Configure)
    void EnableMMSEPrecoding(bool enable);       ///< spec Section 7.5
    void SetAlpha(double alpha);                 ///< spec Section 7.5
    void SetMaxActiveBeams(uint32_t k);
    void SetNumBeams(uint32_t n);
    void SetSatId(uint32_t satId);
    void SetMetricsOutputFile(const std::string& path);

    // ── Phase feature flags ────────────────────────────────────────────────
    void SetSchedulerEnabled(bool enable);
    void SetObcEnabled(bool enable);
    void SetCacheQueueEnabled(bool enable);

    // ── Installation ──────────────────────────────────────────────────────

    /// Create and wire all enabled modules.
    /// Accesses Singleton<SatTopology> internally (non-invasive).
    /// Must be called after SimulationHelper::CreateSatScenario().
    void Install();

    // ── Module access ──────────────────────────────────────────────────────

    Ptr<SatBhMetrics>     GetMetrics()     const { return m_metrics; }
    Ptr<SatBhScheduler>   GetScheduler()   const { return m_scheduler; }
    Ptr<SatBhObc>         GetObc()         const { return m_obc; }
    Ptr<SatGwCacheQueue>  GetCacheQueue()  const { return m_cacheQueue; }
    Ptr<SatBhPrecoder>    GetPrecoder()    const { return m_precoder; }
    Ptr<SatBhTimePlan>    GetCurrentPlan() const;

  private:
    // ── Phase 1 bootstrap (always runs) ──────────────────────────────────

    /// Build a static BHTP for Phase 1 (no Scheduler yet).
    /// hotspot beams = beamIds 1..m_numHotspotBeams (SMALL radius)
    /// non-hotspot   = beamIds (m_numHotspotBeams+1)..m_numBeams (LARGE radius)
    Ptr<SatBhTimePlan> BuildStaticBhtp(Time periodStart);

    /// Synthetic metrics driver: simulates one slot's OBC + CacheQueue events.
    /// Called every T_s when OBC is disabled (Phase 1 / 1.5).
    void ApplySyntheticSlot();

    // ── Phase 2 setup ─────────────────────────────────────────────────────

    /// Create SatBhScheduler, configure attributes, connect plan-ready callback.
    void SetupScheduler();

    /// Create SatBhObc, configure attributes, connect beam callbacks to Metrics
    /// and CacheQueue.
    void SetupObc();

    // ── Phase 3 setup ─────────────────────────────────────────────────────

    /// Create SatGwCacheQueue, configure attributes, connect drop callback to Metrics.
    void SetupCacheQueue();

    /// Create SatBhPrecoder, configure attributes.
    void SetupPrecoder();

    // ── SNS3 trace connection (Phase 4, Hook feasibility required) ────────

    /// Connect available SNS3 traces to module callbacks.
    /// Falls back to Simulator::Schedule polling for missing traces.
    void ConnectTraces();

    // ── State ─────────────────────────────────────────────────────────────

    BhExperimentConfig   m_cfg;           ///< All experiment parameters

    Ptr<SatBhMetrics>    m_metrics;       ///< Phase 1: always created
    Ptr<SatBhTimePlan>   m_staticPlan;    ///< Phase 1: static plan (used when Scheduler off)
    Ptr<SatBhScheduler>  m_scheduler;     ///< Phase 2: created when m_cfg.enableScheduler
    Ptr<SatBhObc>        m_obc;           ///< Phase 2: created when m_cfg.enableObc
    Ptr<SatGwCacheQueue> m_cacheQueue;    ///< Phase 3: created when m_cfg.enableCacheQueue
    Ptr<SatBhPrecoder>   m_precoder;      ///< Phase 3: created when m_cfg.enablePrecoder

    uint32_t             m_syntheticSlotIdx; ///< Tracks position in synthetic driver loop
    bool                 m_installed;        ///< True after Install() has been called
};

} // namespace ns3

#endif // SAT_BH_HELPER_H
