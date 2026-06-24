/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-resource-manager.h
 *
 * SatResourceManager — Frame-scale BH resource management controller.
 * (Layer2.md §5.3, Phase C)
 *
 * Responsibilities:
 *   1. Self-scheduling loop every T_frame = 503 ms (no external timer needed)
 *   2. Collect per-UT state (demand, buffer, C/N0) from callbacks
 *   3. Retrieve active beam list from SatBhScheduler's current BHTP
 *   4. Delegate UT-beam assignment to SatUserAssociator (WFQ/Priority/RR)
 *   5. Expose per-beam BeamConfig to SatBhHelper and SatPowerAllocator
 *   6. Optionally query SatL1RoutingInterface to protect route-bearing beams
 *
 * Self-scheduling design (Layer2.md §5.3):
 *   DoInitialize() schedules the first RunFrameOptimization().
 *   At the end of each RunFrameOptimization(), Simulator::Schedule(T_frame,...)
 *   queues the next call.  This avoids timer drift by anchoring each call to
 *   the previous call's actual fire time rather than a fixed-rate timer.
 *
 * Non-invasive constraint (Layer2.md §2.2):
 *   All SNS3 interactions go through callbacks registered by SatBhHelper.
 *   SatResourceManager never holds direct pointers to SNS3 internal objects.
 */

#ifndef SAT_BH_RESOURCE_MANAGER_H
#define SAT_BH_RESOURCE_MANAGER_H

#include "sat-bh-scheduler.h"
#include "sat-bh-user-associator.h"
#include "sat-l1-routing-interface.h"
#include "sat-power-allocator.h"

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace ns3
{

// ── BeamConfig ────────────────────────────────────────────────────────────
//
// Per-beam output of one RunFrameOptimization cycle.
// Consumed by SatPowerAllocator (Phase D) and SatBhHelper diagnostics.
//
struct BeamConfig
{
    uint32_t              beamId;            ///< Beam identifier (1-indexed)
    uint32_t              patternIndex;      ///< Beam pattern selected (0~4); default MIDDLE=2
    std::vector<uint32_t> assignedUtIds;     ///< UTs associated to this beam for the next frame
    double                allocatedPowerDbw; ///< Power assigned by SatPowerAllocator (default: equal share)
    double                dwellMs;           ///< Total dwell time in this frame [ms] (from BHTP)

    BeamConfig()
        : beamId(0), patternIndex(2), allocatedPowerDbw(43.0), dwellMs(0.0)
    {}
};

// ── Callback types ────────────────────────────────────────────────────────

// Callback fired once per frame after assignment is computed.
// Consumers (e.g., SatPowerAllocator) receive the full beam config map.
using FrameConfigCallback =
    std::function<void(uint32_t frameId, const std::map<uint32_t, BeamConfig>& configs)>;

// Callback invoked by SatBhHelper to inject per-UT state before each frame.
// SatResourceManager stores the state and uses it in the next Associate() call.
using UtStateUpdateCallback =
    std::function<void(const UtInfo& ut)>;

// ── SatResourceManager ────────────────────────────────────────────────────

class SatResourceManager : public Object
{
  public:
    static TypeId GetTypeId();
    SatResourceManager();

    // ── Wiring (called by SatBhHelper before Install()) ───────────────────

    /// Connect to the demand-driven scheduler for BHTP plan queries.
    void SetScheduler(Ptr<SatBhScheduler> scheduler) { m_scheduler = scheduler; }

    /// Connect optional L1 routing interface for path-beam protection.
    void SetL1Interface(Ptr<SatL1RoutingInterface> iface) { m_l1Interface = iface; }

    /// Connect the frame-config output sink (e.g., SatPowerAllocator or logging).
    void SetFrameConfigCallback(FrameConfigCallback cb) { m_frameConfigCb = cb; }

    /// Connect the user associator (WFQ/Priority/RR).
    void SetUserAssociator(Ptr<SatUserAssociator> assoc) { m_associator = assoc; }

    /// Connect the power allocator (Phase D).
    void SetPowerAllocator(Ptr<SatPowerAllocator> alloc) { m_powerAllocator = alloc; }

    // ── UT state injection (called by SatBhHelper trace hooks in Phase E) ─

    /// Update or insert the state for a single UT.
    /// Called periodically by SatBhHelper from SNS3 trace callbacks.
    void UpdateUtState(const UtInfo& ut);

    /// Remove a UT from the managed set (e.g., UT has left coverage area).
    void RemoveUt(uint32_t utId);

    // ── Query ─────────────────────────────────────────────────────────────

    /// Return the most recent BeamConfig for a given beam.
    /// Returns default BeamConfig if no frame has run yet.
    BeamConfig GetBeamConfig(uint32_t beamId) const;

    /// Return frame counter n (increments once per RunFrameOptimization call).
    /// Corresponds to parameter n in the formal BH model.
    uint32_t GetFrameN() const { return m_n; }

    /// Return a copy of the current UT state map (for diagnostics).
    std::vector<UtInfo> GetUtList() const;

  private:
    // ── Object lifecycle ──────────────────────────────────────────────────

    /// Schedule the first RunFrameOptimization() at simulation time 0.
    void DoInitialize() override;

    // ── Frame optimization ────────────────────────────────────────────────

    /// One frame cycle:
    ///   1. Collect active beams from current BHTP
    ///   2. Build per-beam capacity map from BHTP dwell times
    ///   3. Filter out L1-protected beams from reassignment pool (optional)
    ///   4. Call SatUserAssociator::Associate() + ApplyAssociation()
    ///   5. Update m_beamConfigs
    ///   6. Fire FrameConfigCallback
    ///   7. Schedule next RunFrameOptimization at now + T_frame
    void RunFrameOptimization();

    // ── Internal helpers ──────────────────────────────────────────────────

    /// Extract active beam IDs from the scheduler's current BHTP.
    /// Falls back to m_fallbackBeamIds if no plan is available.
    std::vector<uint32_t> GetActiveBeamIds() const;

    /// Compute per-beam service capacity [kbps] from BHTP dwell times and
    /// a nominal link rate (used by SatUserAssociator as capacity hint).
    /// capacity_b = dwellMs_b / T_slot × nominalKbpsPerSlot
    std::map<uint32_t, double> ComputeBeamCapacityMap(
        const std::vector<uint32_t>& beamIds) const;

    // ── Attribute accessors ───────────────────────────────────────────────

    void   SetFrameDurationMs(double ms) { m_frameDuration = MilliSeconds(ms); }
    double GetFrameDurationMs() const    { return m_frameDuration.GetMilliSeconds(); }

    // ── State ─────────────────────────────────────────────────────────────

    Time     m_frameDuration;      ///< T_frame (attr: FrameDuration, default 503 ms)
    bool     m_enableUserAssoc;    ///< Enable SatUserAssociator (attr: EnableUserAssociation)
    bool     m_enablePatternSel;   ///< Enable SatBeamPatternSelector (attr: EnablePatternSelection, optional)
    bool     m_enablePowerAlloc;   ///< Enable SatPowerAllocator Phase D (attr: EnablePowerAllocation)
    double   m_nominalKbpsPerSlot; ///< Nominal link capacity per active slot [kbps]

    uint32_t m_i;                  ///< i: satellite index (formal model parameter i)
    uint32_t m_n;                  ///< n: frame number, increments each RunFrameOptimization

    Ptr<SatBhScheduler>        m_scheduler;       ///< BHTP source (demand-driven scheduler)
    Ptr<SatUserAssociator>     m_associator;      ///< UT-beam assignment engine (Phase C)
    Ptr<SatL1RoutingInterface> m_l1Interface;     ///< Optional L1 path protection
    Ptr<SatPowerAllocator>     m_powerAllocator;  ///< Per-beam TX power optimizer (Phase D)

    FrameConfigCallback m_frameConfigCb; ///< Output sink for per-frame BeamConfig map

    std::map<uint32_t, UtInfo>      m_utStateMap;   ///< Current state of all managed UTs
    std::map<uint32_t, BeamConfig>  m_beamConfigs;  ///< Last computed BeamConfig per beam

    /// Fallback beam IDs when no BHTP is available (e.g., during warm-up).
    /// Set by SatBhHelper via SetFallbackBeams() if needed.
    std::vector<uint32_t> m_fallbackBeamIds;
};

} // namespace ns3

#endif // SAT_BH_RESOURCE_MANAGER_H
