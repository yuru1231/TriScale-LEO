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
#include "sat-bh-resource-manager.h"
#include "sat-bh-scheduler.h"
#include "sat-bh-time-plan.h"
#include "sat-bh-user-associator.h"
#include "sat-dynamic-bstp-provider.h"
#include "sat-gw-cache-queue.h"
#include "sat-l1-routing-interface.h"
#include "sat-power-allocator.h"

#include "ns3/address.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <map>
#include <utility>   // std::pair — used by m_beamToggleMap key {satId, beamId}

// Forward declarations for Phase E/F SNS3 components (full includes in .cc)
namespace ns3
{
class SimulationHelper;
class SatOrbiterNetDevice;
class SatNetDevice;
}

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
    uint32_t numBeams{7};           ///< Beams per satellite (leo2sat default: 7; starlink25: 25)
    uint32_t maxActiveBeams{2};     ///< K_b: max simultaneous beams per slot (Phase 1: 2)
    uint32_t numHotspotBeams{3};    ///< Legacy fallback when hotCellIndices is empty
    uint32_t satId{0};             ///< Satellite index to install on

    /// Fixed hot cell indices (0-indexed, same convention as 5×5 grid in orbit-sgp4).
    /// When non-empty, BuildStaticBhtp() uses these instead of numHotspotBeams.
    /// Leave empty (default) for leo2sat / iridium-next scenarios.
    /// Starlink-1584 Phase 1 set: cell_idx 2 (row0-mid), 10 (row2-left), 12 (centre),
    ///                            18 (row3-right), 21 (row4-left).
    std::vector<uint32_t> hotCellIndices{};

    // ── Timing ────────────────────────────────────────────────────────────
    double   simTimeSec{300.0};       ///< Total simulation duration [s] (ignored when numPeriods>0)
    uint32_t numPeriods{0};           ///< Run exactly N BHTP periods after warmup (0 = use simTimeSec)
    double   warmUpSec{10.0};         ///< Warm-up period to discard [s]
    double   slotDurationMs{10.0};    ///< T_s [ms] — 10 ms per slot (project fixed)
    double   bhtpPeriodMs{80.0};      ///< T_p [ms] — 80 ms frame = 8 slots × T_s
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

    // ── Phase C feature flags ─────────────────────────────────────────────
    bool     enableResourceManager{false};  ///< Phase C: SatResourceManager self-scheduling loop
    bool     enableUserAssociation{true};   ///< Phase C: run SatUserAssociator each frame
    bool     enablePatternSelection{false}; ///< Phase C: beam pattern selection (optional, Phase E)

    // ── Phase C scheduling ────────────────────────────────────────────────
    uint8_t  schedulingMode{0};           ///< 0=WFQ (default), 1=Priority, 2=RoundRobin
    uint32_t maxReassignmentPerFrame{5};  ///< Max MoveUtBetweenBeams calls per frame
    double   nominalKbpsPerSlot{50000.0}; ///< Capacity hint per slot [kbps] for WFQ
    double   maxDelayMs{80.0};            ///< HOL deadline ~1 T_frame (80 ms) for deadline protection

    // ── Phase D feature flags ─────────────────────────────────────────────
    bool     enablePowerAllocation{false};  ///< Phase D: run SatPowerAllocator each frame

    // ── Phase D power parameters ──────────────────────────────────────────
    double   totalPowerBudgetDbm{43.0};    ///< Total TX power budget [dBm]
    double   noisePowerDbw{-126.47};       ///< Thermal noise floor [dBW]
    uint32_t powerMaxIterations{30};       ///< IWFA max iterations
    double   powerConvergenceEps{0.001};   ///< IWFA convergence threshold [W]
    double   interferenceFactor{0.01};     ///< Cross-beam ICI leakage fraction

    // ── Phase E feature flags ─────────────────────────────────────────────
    // Activates only when enableResourceManager=true AND SetSimulationHelper() called.
    // Wires MoveUtCallback -> SatNcc::MoveUtBetweenBeams and
    //       ApplyPowerCallback -> SatOrbiterUserPhy::SetTxMaxPowerDbw + Initialize.
    bool     enablePhaseE{false};           ///< Phase E: wire callbacks to real SNS3 APIs

    // ── Phase F feature flags ─────────────────────────────────────────────
    // Activates only when enablePhaseE=true (requires BuildUtAddressMap() called first).
    // Connects SatBeamScheduler::BacklogRequestsTrace on every beam scheduler and
    // replaces the synthetic 1000 kbps demand in PollUtStates() with real RBDC values.
    bool     enablePhaseF{false};           ///< Phase F: wire DAMA demand traces → ResourceManager

    // ── Phase G: Dynamic BSTP Provider ───────────────────────────────────
    // Demand-greedy, fairness-aware beam selection — lighter than the full EM
    // Scheduler (Phase 2) but smarter than the static round-robin (Phase 1).
    //
    // Activation: enableDynamicBstp=true AND enableScheduler=false.
    // If both flags are true, Scheduler takes priority and Phase G is skipped.
    //
    // Demand feeding: when enablePhaseF=true, OnBacklogRequestTrace() also calls
    // SatDynamicBstpProvider::UpdateBeamDemand() so the provider sees real RBDC.
    // Before Phase F is wired, all demands are 0 and the provider uses round-robin.
    bool     enableDynamicBstp{false};      ///< Phase G: use SatDynamicBstpProvider (greedy top-K)
    double   bhDemandBacklogWeight{1.0};    ///< Phase G: score weight for demand [kbps]
    double   bhFairnessWeight{0.5};         ///< Phase G: score weight for time-since-served [s]
    uint32_t bhValiditySuperframes{1};      ///< Phase G: Conf::validityInSuperframes
    uint32_t bhStarvationThreshold{5};      ///< Phase G: forced inclusion after N skipped cycles

    // ── Phase G synthetic FWD demand injection ────────────────────────────
    // RBDC (Phase F) only captures RTN demand, which is uniform across beams.
    // To make the greedy provider aware of FWD traffic hotspots, inject a
    // synthetic demand signal for designated FWD hotspot beams every T_p.
    // This signal is ADDED on top of any RBDC demand already received.
    //
    // Set bhFwdHotspotBeamIds to the beam IDs with elevated FWD traffic.
    // Set bhFwdHotspotBoostKbps to the extra kbps credited to hotspot beams.
    // Leave bhFwdHotspotBeamIds empty (default) to disable synthetic injection.
    std::vector<uint32_t> bhFwdHotspotBeamIds{};   ///< Beam IDs treated as FWD hotspot
    double   bhFwdHotspotBoostKbps{0.0};            ///< Extra demand credited to hotspot beams [kbps]

    // ── ROI / elevation filter (starlink-1584 scale-down) ──────────────────
    // Used to limit BH helper installation to a geographic window.
    // Only helpers whose satId falls in [satIdStart, satIdStart + maxHelperSats)
    // are created; all others are skipped — reducing ns-3 object count from
    // 1584 to maxHelperSats without changing the loaded constellation.
    // minElevDeg is logged for documentation; actual elevation filtering
    // requires the 2D orbit-sgp4 pre-scan to determine satIdStart.
    double   roiLat{35.676};         ///< ROI centre latitude  [°N]  (default: Tokyo)
    double   roiLon{139.650};        ///< ROI centre longitude [°E]
    double   roiRadiusDeg{5.0};      ///< ROI half-width radius [°] (approx great-circle)
    double   minElevDeg{37.0};       ///< Min elevation angle [°] — Starlink Tokyo threshold
    uint32_t maxHelperSats{10};      ///< Max BH helpers to install (0 = entire constellation)

    // ── Constellation geometry (shared with 2D footprint and BH scheduler) ────
    // The scheduler interference model (rMiddle) and the 2D footprint tool
    // (rFootprint_m) both derive their geometric constants from these two values.
    // Changing them here propagates automatically to all dependent calculations
    // via ConstellationParams in sat-constellation-params.h.
    double   altitudeKm{550.0};       ///< Satellite altitude [km] — Starlink Shell-1 default
    double   beamHalfAngleDeg{2.0};   ///< Beam half-angle [deg] — MIDDLE pattern (BeamRadiusType)
    uint32_t satIdStart{490};        ///< First satId of the helper monitoring window

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
    void SetResourceManagerEnabled(bool enable); ///< Phase C: enable ResourceManager loop

    /// Provide a SimulationHelper pointer so Phase E can reach SatNcc and
    /// SatOrbiterNetDevice.  Call before Install() when enablePhaseE=true.
    void SetSimulationHelper(Ptr<SimulationHelper> simHelper);

    // ── Installation ──────────────────────────────────────────────────────

    /// Create and wire all enabled modules.
    /// Accesses Singleton<SatTopology> internally (non-invasive).
    /// Must be called after SimulationHelper::CreateSatScenario().
    void Install();

    // ── Module access ──────────────────────────────────────────────────────

    Ptr<SatBhMetrics>          GetMetrics()          const { return m_metrics; }
    Ptr<SatBhScheduler>        GetScheduler()        const { return m_scheduler; }
    Ptr<SatBhObc>              GetObc()              const { return m_obc; }
    Ptr<SatGwCacheQueue>       GetCacheQueue()       const { return m_cacheQueue; }
    Ptr<SatBhPrecoder>         GetPrecoder()         const { return m_precoder; }
    Ptr<SatBhTimePlan>         GetCurrentPlan()      const;
    Ptr<SatResourceManager>    GetResourceManager()  const { return m_resourceManager; }
    Ptr<SatUserAssociator>     GetUserAssociator()   const { return m_associator; }
    Ptr<SatL1RoutingInterface> GetL1Interface()      const { return m_l1Interface; }

    /// Register callback fired once per frame after UT association completes.
    /// Used by SatPowerAllocator (Phase D).
    void SetFrameConfigCallback(FrameConfigCallback cb);

  private:
    // ── Phase 1 bootstrap (always runs) ──────────────────────────────────

    /// Build a static BHTP for Phase 1 (no Scheduler yet).
    /// hotspot beams = beamIds 1..m_numHotspotBeams (SMALL radius)   [j=1..J_hotspot]
    /// non-hotspot   = beamIds (m_numHotspotBeams+1)..J (LARGE radius)  [J = total beams]
    Ptr<SatBhTimePlan> BuildStaticBhtp(Time periodStart);

    /// Synthetic metrics driver: simulates one slot's OBC + CacheQueue events.
    /// Called every T_s when OBC is disabled (Phase 1 / 1.5).
    void ApplySyntheticSlot();

    /// Synthetic demand driver: injects per-beam demand into the Scheduler every T_s.
    /// Active when enableScheduler=true but ConnectTraces() has not yet wired real
    /// SNS3 trace hooks. Makes the EM algorithm work with non-trivial data.
    /// Replaced by real DaRequestReceived trace hook in Phase 4.
    void ApplySyntheticDemand();

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

    // ── Phase C setup ─────────────────────────────────────────────────────

    /// Create SatL1RoutingInterface (stub), SatUserAssociator, SatResourceManager.
    /// SatResourceManager::Initialize() starts the self-scheduling loop.
    void SetupPhaseC();

    // ── Phase D setup ─────────────────────────────────────────────────────

    /// Create SatPowerAllocator and wire it into SatResourceManager.
    /// ApplyPowerCallback left unwired until Phase E (SNS3 hook).
    /// Must be called after SetupPhaseC() so m_resourceManager exists.
    void SetupPhaseD();

    // ── Phase E wiring ────────────────────────────────────────────────────

    /// Build m_utAddressMap: maps UT container index (0-based) to MAC Address.
    /// Must be called after CreateSatScenario() so SatTopology is populated.
    void BuildUtAddressMap();

    /// Cache the SatOrbiterNetDevice for satellite m_cfg.satId.
    /// Required by ApplyPowerCallback so it can reach GetUserPhy(beamId).
    void CacheOrbiterDevice();

    /// Build m_beamToggleMap: maps {satId, beamId} to the GW-side SatNetDevice whose
    /// ToggleState(bool) physically enables/disables the forward link.
    /// Mirrors what SatBeamHelper::Install() registers with SatBstpController
    /// via AddNetDeviceCallback().  Called during Install() when OBC is enabled
    /// so that BeamActivate/DeactivateCallbacks can drive real link state.
    void BuildBeamToggleMap();

    /// Wire Phase E callbacks:
    ///   1. MoveUtCallback -> SatNcc::MoveUtBetweenBeams  (real handover)
    ///   2. ApplyPowerCallback -> SatOrbiterUserPhy::SetTxMaxPowerDbw + Initialize
    ///   3. PollUtStates() scheduling loop (every T_frame)
    /// Requires m_simHelper, m_associator, m_powerAllocator.
    void ConnectTracesPhaseE();

    /// Connect SatBeamScheduler::BacklogRequestsTrace on every beam scheduler so that
    /// real RBDC demand (kbps) is forwarded to OnBacklogRequestTrace() and cached in
    /// m_utDemandCache.  Must be called after ConnectTracesPhaseE() (which calls
    /// BuildUtAddressMap() and populates m_satMapperIdToContainerIdx).
    /// Requires m_simHelper.
    void ConnectTracesPhaseF();

    // ── Phase G setup ─────────────────────────────────────────────────────

    /// Create SatGreedyBstpProvider, register all enabled beams from SatTopology,
    /// set Attributes from m_cfg, and schedule first RunDynamicBstpCycle(t=0).
    /// Only called when m_cfg.enableDynamicBstp=true && !m_cfg.enableScheduler.
    void SetupDynamicBstp();

    /// Called every T_p: ask the provider for the next Conf, convert to
    /// SatBhTimePlan, and push to OBC (or update m_staticPlan when OBC off).
    /// Self-schedules the next call at now + T_p.
    void RunDynamicBstpCycle(Time now);

    /// Called every T_p when bhFwdHotspotBeamIds is non-empty.
    /// Injects a synthetic FWD demand boost into the greedy provider for each
    /// hotspot beam, then re-schedules itself at now + bhtpPeriodMs.
    /// This compensates for Phase F RBDC being RTN-only (uniform demand).
    void InjectFwdDemand();

    /// Convert a SatDynamicBstpProvider::Conf to a SatBhTimePlan.
    ///
    /// The Conf carries an unordered set of active beams for one BHTP window.
    /// To fit the time-slotted SatBhTimePlan model, the active beams are spread
    /// evenly across M slots (round-robin), at most K beams per slot.
    /// Validity maps to periodEnd = periodStart + validityInSuperframes × T_p.
    ///
    /// Returns nullptr if Validate() fails (caller falls back to m_staticPlan).
    Ptr<SatBhTimePlan> ConfToTimePlan(const SatDynamicBstpProvider::Conf& conf,
                                      Time periodStart);

    /// Callback fired by SatBeamScheduler::BacklogRequestsTrace.
    /// Parses the record string "time, beamId, satMapperUtId, typeEnum, value" and
    /// updates m_utDemandCache for DA_RBDC entries only.
    void OnBacklogRequestTrace(std::string record);

    /// Query SatTopology for UT beam assignments and update ResourceManager state.
    /// Runs every T_frame.  When Phase F is active, demand is read from
    /// m_utDemandCache (real RBDC); otherwise falls back to 0.0.
    void PollUtStates();

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

    // Phase C modules
    Ptr<SatResourceManager>    m_resourceManager; ///< Phase C: self-scheduling 80 ms loop (T_p)
    Ptr<SatUserAssociator>     m_associator;      ///< Phase C: WFQ/Priority/RR assignment
    Ptr<SatL1RoutingInterface> m_l1Interface;     ///< Phase C: ISL path protection (stub)
    FrameConfigCallback        m_frameConfigCb;   ///< Phase C/D: output sink for BeamConfig

    // Phase D modules
    Ptr<SatPowerAllocator>     m_powerAllocator;  ///< Phase D: IWFA TX power optimizer

    // Phase G modules
    Ptr<SatDynamicBstpProvider> m_dynamicProvider; ///< Phase G: greedy/reactive BSTP provider
    uint32_t                    m_dynamicPlanId;   ///< Monotonic plan ID for Phase G SatBhTimePlans
    Ptr<SatBhTimePlan>          m_lastDynamicPlan; ///< Phase G: most-recent plan from dynamic provider (for GetCurrentPlan)

    // Phase E state
    Ptr<SimulationHelper>           m_simHelper;       ///< Phase E: access to NCC + topology
    std::map<uint32_t, Address>     m_utAddressMap;    ///< Phase E: UT container idx -> MAC addr
    Ptr<SatOrbiterNetDevice>        m_orbiterDev;      ///< Phase E: cached orbiter net device

    // OBC real-toggle state
    // {satId, beamId} -> GW-side SatNetDevice.  Populated by BuildBeamToggleMap().
    // Keyed by (satId, beamId) pair because the same beamId may exist on multiple
    // satellites in a constellation — a flat beamId key would keep only the first
    // GW device found and mis-toggle beams on other satellites.
    // OBC callbacks drive ToggleState(true/false) to replicate SatBstpController.
    std::map<std::pair<uint32_t,uint32_t>, Ptr<SatNetDevice>> m_beamToggleMap;
    ///< OBC real-toggle: {satId, beamId} -> GW SatNetDevice

    // Phase F state
    // UtDemandEntry accumulates per-scheduling-cycle RBDC demand.
    // lastTimestampS is used to detect a new scheduling cycle (reset vs. sum RCs).
    struct UtDemandEntry
    {
        double totalRbdcKbps{0.0};   ///< sum of RBDC across all RCs in the latest cycle
        double lastTimestampS{-1.0}; ///< scheduling cycle timestamp (from trace string)
    };
    std::map<uint32_t, UtDemandEntry> m_utDemandCache;            ///< Phase F: containerIdx → latest RBDC demand
    std::map<int32_t,  uint32_t>      m_satMapperIdToContainerIdx; ///< Phase F: SatIdMapper UT ID → container index

    uint32_t             m_syntheticSlotIdx; ///< Tracks position in synthetic driver loop
    bool                 m_installed;        ///< True after Install() has been called
};

} // namespace ns3

#endif // SAT_BH_HELPER_H
