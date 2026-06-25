/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-user-associator.h
 *
 * SatUserAssociator — Frame-scale UT-to-beam association with integrated
 * WFQ / Priority / Round-Robin scheduling (Layer2.md §5.2).
 *
 * This module absorbs the removed SatUserScheduler (Layer2.md §5.4).
 * Scheduling is expressed as pre-planned UT-beam assignments computed at
 * frame N for execution at frame N+1, aligned with the BHTP pre-planned
 * schedule design.
 *
 * Pre-planned schedule flow (Layer2.md §5.2):
 *   Frame N end:  Associate() computes utId → beamId for frame N+1
 *                 ApplyAssociation() calls MoveUtBetweenBeams immediately
 *   Frame N+1:    TIM-U delay expires, UT completes beam switch → live
 *
 * The 1-frame TIM-U delay is intentional — the scheduler plans one frame
 * ahead so the association is already in effect when frame N+1 starts.
 * No race condition exists by design.
 *
 * SNS3 hook (trace confirmed, Layer2.md §6):
 *   SatNcc::MoveUtBetweenBeams(utId, srcSatId, srcBeamId, dstSatId, dstBeamId)
 *   Invoked via MoveUtCallback (non-invasive; wired by SatBhHelper in Phase E).
 *   TransferUtToBeam() is NOT used — it only moves scheduler state and leaves
 *   MAC/PHY/routing inconsistent.
 *
 * Implementation phase: Phase C (Layer2.md §7).
 */

#ifndef SAT_BH_USER_ASSOCIATOR_H
#define SAT_BH_USER_ASSOCIATOR_H

#include "ns3/object.h"

#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <vector>

namespace ns3
{

// ── AssignmentMap ─────────────────────────────────────────────────────────
//
// Output of Associate(): maps each UT ID to the beam ID it should be
// served by in the next frame.  Key = utId (1-indexed), Value = beamId.
//
using AssignmentMap = std::map<uint32_t, uint32_t>;

// ── UtInfo ────────────────────────────────────────────────────────────────
//
// State snapshot of a single UT as seen by SatUserAssociator.
// Populated by SatResourceManager before each Associate() call.
// Values come from SNS3 trace callbacks in Phase E; synthetic in Phase C.
//
struct UtInfo
{
    uint32_t utId;                ///< UT identifier (SNS3 1-indexed)
    uint32_t currentBeamId;       ///< Current beam ID (1-indexed)
    double   demandKbps;          ///< Requested data rate
    double   bufferBytes;         ///< TX buffer occupancy (proxy for HOL wait)
    double   cno;                 ///< C/N0 [dB] on current beam
    uint32_t priorityClass;       ///< 0=low, 1=medium, 2=high (from QoS marking)
    double   headOfLineDelayMs;   ///< Estimated HOL delay; used for deadline enforcement

    UtInfo()
        : utId(0), currentBeamId(0), demandKbps(0.0), bufferBytes(0.0),
          cno(0.0), priorityClass(1), headOfLineDelayMs(0.0)
    {}

    UtInfo(uint32_t id, uint32_t beam, double demand = 0.0,
           double buf = 0.0, double c = 0.0, uint32_t prio = 1)
        : utId(id), currentBeamId(beam), demandKbps(demand), bufferBytes(buf),
          cno(c), priorityClass(prio), headOfLineDelayMs(0.0)
    {}
};

// ── MoveUtCallback ────────────────────────────────────────────────────────
//
// Corresponds to SatNcc::MoveUtBetweenBeams(utId, srcSat, srcBeam, dstSat, dstBeam).
// Registered by SatBhHelper in Phase E via SatUserAssociator::SetMoveUtCallback().
// Until wired, ApplyAssociation() logs the intended move but does not call into SNS3.
//
using MoveUtCallback = std::function<void(uint32_t utId,
                                          uint32_t srcSatId, uint32_t srcBeamId,
                                          uint32_t dstSatId, uint32_t dstBeamId)>;

// ── SchedulingMode ────────────────────────────────────────────────────────

enum class SchedulingMode : uint8_t
{
    WFQ         = 0, ///< Weighted Fair Queuing — primary mode
    PRIORITY    = 1, ///< Strict priority (high priorityClass served first)
    ROUND_ROBIN = 2, ///< Equal rotation regardless of demand
};

// ── SatUserAssociator ─────────────────────────────────────────────────────

class SatUserAssociator : public Object
{
  public:
    static TypeId GetTypeId();
    SatUserAssociator();

    // ── Configuration ─────────────────────────────────────────────────────

    /// Set satellite ID used as both src and dst sat in MoveUtBetweenBeams.
    /// (Single-satellite scenario; multi-sat support deferred to Phase E.)
    void SetSatId(uint32_t satId) { m_i = satId; }

    /// Register callback wired to SatNcc::MoveUtBetweenBeams (Phase E).
    void SetMoveUtCallback(MoveUtCallback cb) { m_moveUtCb = cb; }

    /// Provide per-beam service capacity in Kbps (beamId → capacity).
    /// WFQ uses this to estimate how many UTs a beam can serve per frame.
    /// Derived from BHTP dwell time × link rate.
    void SetBeamCapacityMap(const std::map<uint32_t, double>& capacityKbps);

    // ── Core operations ───────────────────────────────────────────────────

    /// Compute the UT-beam assignment for the NEXT frame.
    /// Dispatches to WFQ / Priority / RR based on m_schedulingMode.
    /// @param utList    All UTs with current state (populated by ResourceManager)
    /// @param beamIds   Active beam IDs for the next frame (from BHTP)
    /// @return          AssignmentMap: utId → beamId
    AssignmentMap Associate(const std::vector<UtInfo>& utList,
                            const std::vector<uint32_t>& beamIds);

    /// Apply assignment: for each (ut, newBeam) pair that differs from the
    /// UT's current beam, invoke MoveUtCallback.
    /// Respects MaxReassignmentPerFrame limit; deadline-violated UTs bypass it.
    /// Called at the END of frame N so TIM-U takes effect at frame N+1 start.
    void ApplyAssociation(const AssignmentMap& assignment,
                          const std::vector<UtInfo>& utList);

    // ── WFQ helpers ───────────────────────────────────────────────────────

    /// Compute WFQ service weight for a UT.
    /// weight = (priorityClass+1) × (1 + bufferBytes/1500 + demandKbps/100)
    /// Higher weight → served first within this frame.
    double ComputeWfqWeight(const UtInfo& ut) const;

    /// Return true if ut.headOfLineDelayMs >= m_maxDelayMs.
    bool IsDeadlineViolated(const UtInfo& ut) const;

    // ── Query ─────────────────────────────────────────────────────────────

    /// Return the assignment computed in the most recent Associate() call.
    const AssignmentMap& GetLastAssignment() const { return m_prevAssignment; }

    uint32_t GetFrameReassignmentCount() const { return m_lastReassignCount; }

  private:
    // ── Scheduling mode implementations ───────────────────────────────────

    /// WFQ: sort UTs by weight, assign greedily to best-fit beam.
    /// Beam preference: keep on current beam if available (min TIM-U overhead).
    AssignmentMap RunWfq(const std::vector<UtInfo>& utList,
                         const std::vector<uint32_t>& beamIds);

    /// Priority: sort by priorityClass desc, assign to highest-capacity beam.
    AssignmentMap RunPriority(const std::vector<UtInfo>& utList,
                              const std::vector<uint32_t>& beamIds);

    /// Round-Robin: rotate UT-beam pairing across frames for equal service.
    AssignmentMap RunRoundRobin(const std::vector<UtInfo>& utList,
                                const std::vector<uint32_t>& beamIds);

    // ── Deadline enforcement ──────────────────────────────────────────────

    /// Override assignment for UTs whose HOL delay >= m_maxDelayMs.
    /// Forced UTs are moved to the beam with the highest available capacity.
    /// Returns the number of forced reassignments performed.
    uint32_t EnforceDeadlineProtection(AssignmentMap& assignment,
                                       const std::vector<UtInfo>& utList,
                                       const std::vector<uint32_t>& beamIds);

    // ── Attribute accessors ───────────────────────────────────────────────

    uint8_t GetSchedulingModeAttr() const
    { return static_cast<uint8_t>(m_schedulingMode); }
    void    SetSchedulingModeAttr(uint8_t v)
    { m_schedulingMode = static_cast<SchedulingMode>(v); }

    // ── State ─────────────────────────────────────────────────────────────

    SchedulingMode m_schedulingMode;   ///< Active scheduling strategy
    uint32_t       m_maxReassignment;  ///< Max MoveUtBetweenBeams calls per frame
    double         m_minRateKbps;      ///< Min rate threshold; UTs below get lower priority
    double         m_maxDelayMs;       ///< HOL delay limit for deadline enforcement

    uint32_t       m_i;                ///< i: satellite index (formal model parameter i)
    MoveUtCallback m_moveUtCb;         ///< Hook → SatNcc::MoveUtBetweenBeams (Phase E)

    std::map<uint32_t, double> m_beamCapacityKbps; ///< Per-beam service capacity

    uint32_t      m_rrIndex;           ///< Round-robin cursor (index into UT list)
    AssignmentMap m_prevAssignment;    ///< Previous frame result (delta detection)
    uint32_t      m_lastReassignCount; ///< Reassignments issued in last ApplyAssociation
};

} // namespace ns3

#endif // SAT_BH_USER_ASSOCIATOR_H
