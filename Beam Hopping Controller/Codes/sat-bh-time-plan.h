/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-time-plan.h
 *
 * SatBhTimePlan — Beam Hopping Time Plan (BHTP) data model.
 *
 * This is the central data structure of the formal BH architecture
 * defined in BH_SNS3_Specification_v1.pdf, and described in
 * Layer2.md Section 5 (Time Model & BHTP Structure).
 *
 * Key time constants (spec Section 5.1):
 *   T_s   = 26.5 ms   BH time slot duration (= DVB-S2X super-frame)
 *   T_p   = 503 ms    BHTP period  (M slots × T_s)
 *   T_sw  = 2 ms      beam switching dead-time (embedded in slot boundary)
 *   T_prop= 10 ms     command propagation delay (NCC → satellite OBC)
 *   K     = 2 ~ 4     max simultaneously active beams per slot (default 3)
 *
 * Lifecycle (spec Section 3.1):
 *   SatBhScheduler  → generates SatBhTimePlan each period
 *   SatBhObc        → receives plan (after T_prop), executes slot switching
 *   SatBhMetrics    → passively observes plan for KPI computation
 *
 * Implementation phase: Phase 1 (Layer2.md Section 10)
 *
 * Non-invasive constraint (Principle A/B, Layer2.md Section 2.2):
 *   This module is a pure data model — no direct SNS3 API calls.
 */

#ifndef SAT_BH_TIME_PLAN_H
#define SAT_BH_TIME_PLAN_H

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace ns3
{

// ── BeamRadiusType ────────────────────────────────────────────────────────
//
// Five beam pattern sizes, aligned with Layer2.md §4.3 (5-pattern table).
// Each value maps to a 3dB beamwidth and corresponding antenna gain,
// fed into the SNS3 antenna model non-invasively via attribute configuration.
//
// Index mapping matches SatBeamPatternSelector output (patternIndex 0~4):
//   XSMALL  → patternIndex 0
//   SMALL   → patternIndex 1
//   MIDDLE  → patternIndex 2 (default)
//   LARGE   → patternIndex 3
//   XLARGE  → patternIndex 4
//
enum class BeamRadiusType : uint8_t
{
    XSMALL = 0, ///< 1.0° beamwidth, ~10 km radius, 43.89 dBi — hotspot core
    SMALL  = 1, ///< 1.5° beamwidth, ~15 km radius, 41.39 dBi — high demand
    MIDDLE = 2, ///< 2.0° beamwidth, ~20 km radius, 37.89 dBi — default
    LARGE  = 3, ///< 2.5° beamwidth, ~25 km radius, 35.01 dBi — low demand
    XLARGE = 4, ///< 3.0° beamwidth, ~30 km radius, 31.89 dBi — edge / cold-spot
};

/// Convert BeamRadiusType to human-readable string (for logging / CSV)
std::string BeamRadiusTypeToString(BeamRadiusType r);

// ── ModcodIndex ───────────────────────────────────────────────────────────
//
// DVB-S2X MODCOD index placeholder.
// Maps to the MODCOD enumeration used by SatPhyRx / SatModcodDefs in SNS3.
// Exact mapping to be confirmed against SNS3 source during Hook feasibility
// check (Layer2.md Step 0).  0 = lowest rate (QPSK 1/4); higher = higher.
//
using ModcodIndex = uint32_t;

// ── BhSlotEntry ───────────────────────────────────────────────────────────
//
// Represents one time slot within a BHTP period.
// Each slot activates up to K beams simultaneously.
//
// Direct mapping to spec Section 5.2:
//   "each slot specifies the set of simultaneously active beams,
//    beam pattern (per-beam), MODCOD, cluster IDs, start time, and duration."
//
// Note: startTime is an offset from SatBhTimePlan::m_periodStart,
// NOT an absolute simulation time.  SatBhObc converts to absolute time
// by adding periodStart when scheduling Simulator events.
//
struct BhSlotEntry
{
    std::vector<uint32_t> beamIds;    ///< Simultaneously active beams (|beamIds| ≤ K)
    Time                  startTime;  ///< Offset from period start (e.g. slot 0 → 0 ms)
    Time                  duration;   ///< Slot service window (T_s = 26.5 ms by default)
    ModcodIndex           modcod;     ///< DVB-S2X MODCOD index for this slot
    std::vector<uint32_t> clusterIds; ///< Interference cluster ID per beam (one-to-one with beamIds)

    /// Per-beam antenna pattern, keyed by 1-indexed beamId.
    /// Replaces the former slot-wide beamRadius field (Q2/Q3 decision).
    /// Set by SatBhScheduler (BuildPlan) or SatBhHelper (BuildStaticBhtp).
    std::map<uint32_t, BeamRadiusType> beamPatterns;

    /// Scheduled UT IDs for this slot, assigned by SatUserAssociator.
    /// Empty until SatResourceManager populates it each frame.
    std::vector<uint32_t> scheduledUtIds;

    /// Power allocation per beam [dBW], assigned by SatPowerAllocator.
    /// Key = beamId (1-indexed), Value = allocated transmit power in dBW.
    /// Empty until SatPowerAllocator populates it each frame.
    std::map<uint32_t, double> allocatedPowerDbw;

    /// Default constructor — sets T_s = 26.5 ms, MODCOD = 0
    BhSlotEntry();

    /// Convenience constructor for single-beam slot (common in Phase 2 testing)
    BhSlotEntry(uint32_t beamId, Time start, Time dur,
                BeamRadiusType pattern = BeamRadiusType::MIDDLE,
                ModcodIndex mc         = 0);

    /// End time of this slot (startTime + duration)
    Time GetEndTime() const { return startTime + duration; }

    /// Return true if the given beam is active in this slot
    bool ContainsBeam(uint32_t beamId) const;

    /// Set the antenna pattern for a specific beam in this slot.
    /// beamId is 1-indexed (SNS3 convention).
    void SetBeamPattern(uint32_t beamId, BeamRadiusType pattern);

    /// Get the antenna pattern for a specific beam (returns defaultVal if not found).
    BeamRadiusType GetBeamPattern(uint32_t beamId,
                                  BeamRadiusType defaultVal = BeamRadiusType::MIDDLE) const;

    /// Debug pretty-print to any ostream
    void Print(std::ostream& os) const;
};

// ── SatBhTimePlan ─────────────────────────────────────────────────────────
//
// Container for a complete BHTP period.
// This is the data core of the formal BH architecture.
// (Spec Section 5.3, Layer2.md Section 5.3)
//
// Invariants enforced by Validate():
//   - planId > 0 (assigned by SatBhScheduler, monotonically increasing)
//   - periodEnd > periodStart
//   - all slot offsets fall within [0, periodEnd − periodStart)
//   - slots do not overlap in time
//   - each slot has 1 ≤ |beamIds| ≤ maxActiveBeams (K)
//   - clusterIds.size() == beamIds.size() (if clusterIds is non-empty)
//
class SatBhTimePlan : public Object
{
  public:
    static TypeId GetTypeId();

    SatBhTimePlan();

    // ── Setters / builders ────────────────────────────────────────────────

    /// Assign plan ID (called by SatBhScheduler; must be > 0)
    void SetPlanId(uint32_t id);

    /// Assign frame number n (monotonically increasing; set by SatResourceManager)
    /// Corresponds to parameter n in the formal BH model (n = frame number).
    void SetFrameN(uint32_t n);

    /// Set absolute simulation time bounds for this BHTP period
    void SetPeriodBounds(Time start, Time end);

    /// Append a slot to this plan.
    /// Slots MUST be added in ascending startTime order;
    /// this is asserted to catch out-of-order scheduling bugs early.
    void AddSlot(const BhSlotEntry& slot);

    // ── Getters ───────────────────────────────────────────────────────────

    uint32_t GetPlanId()      const { return m_planId; }
    uint32_t GetFrameN()      const { return m_n; }       ///< n: frame number (formal model)
    Time     GetPeriodStart() const { return m_periodStart; }
    Time     GetPeriodEnd()   const { return m_periodEnd; }
    uint32_t GetNumSlots()    const { return static_cast<uint32_t>(m_slots.size()); }

    const std::vector<BhSlotEntry>& GetSlots() const { return m_slots; }

    /// Return pointer to the slot whose window covers absolute simulation
    /// time t (i.e. periodStart + slot.startTime ≤ t < periodStart + slot.endTime).
    /// Returns nullptr if t is outside all slot windows.
    const BhSlotEntry* GetSlotAt(Time t) const;

    /// Return 0-based slot index active at time t, or -1 if none.
    int32_t GetSlotIndexAt(Time t) const;

    /// Return all unique beam IDs referenced anywhere in this plan
    std::vector<uint32_t> GetAllBeamIds() const;

    /// Total dwell time allocated to the given beam across this plan [ms]
    double GetBeamDwellMs(uint32_t beamId) const;

    /// Total period duration (= T_p = M × T_s)
    Time GetPeriodDuration() const { return m_periodEnd - m_periodStart; }

    // ── Validation ────────────────────────────────────────────────────────

    /// Validate plan structure against the invariants listed above.
    /// @param maxActiveBeams  K value (default 4; production default is 3)
    /// @return true if all checks pass; logs NS_LOG_WARN for each failure.
    bool Validate(uint32_t maxActiveBeams = 4) const;

    // ── Serialization / debug ─────────────────────────────────────────────

    /// Pretty-print the full plan to an ostream (for debug logs)
    void PrettyPrint(std::ostream& os) const;

    /// Export slot table as a CSV string.
    /// Columns: slotIdx, startMs, durationMs, beamIds, beamRadius, modcod, clusterIds
    /// (beamIds and clusterIds are semicolon-separated within their column)
    std::string ToCsv() const;

  private:
    uint32_t               m_planId;      ///< Monotonic plan ID (0 = unassigned)
    uint32_t               m_n;           ///< n: frame number, set by SatResourceManager (0 = unassigned)
    Time                   m_periodStart; ///< Absolute simulation start of this period
    Time                   m_periodEnd;   ///< Absolute simulation end of this period
    std::vector<BhSlotEntry> m_slots;     ///< Ordered list of slot entries
};

} // namespace ns3

#endif // SAT_BH_TIME_PLAN_H
