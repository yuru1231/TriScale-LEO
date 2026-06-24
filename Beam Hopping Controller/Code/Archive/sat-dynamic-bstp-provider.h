/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-dynamic-bstp-provider.h
 *
 * SatDynamicBstpProvider — Abstract interface for dynamic BSTP (Beam Switching
 * Time Plan) providers that can replace the static BHTP in SatBhHelper.
 *
 * Motivation:
 *   SatBhHelper Phase 1 uses BuildStaticBhtp() which generates a fixed
 *   round-robin beam rotation.  Phase 2 uses SatBhScheduler (EM algorithm).
 *   This interface sits between the two: demand-aware, but lightweight—
 *   no EM convergence required, just a scoring function over beam backlogs.
 *
 * Integration point (Phase G in SatBhHelper):
 *   When BhExperimentConfig::enableDynamicBstp = true and enableScheduler = false,
 *   SatBhHelper creates a concrete provider, registers all enabled beams, and
 *   calls GetNextConf(now) every T_p (BHTP period) to obtain the next active
 *   beam set.  The result is converted to SatBhTimePlan and fed to SatBhObc.
 *
 * Demand feeding:
 *   SatBhHelper::OnBacklogRequestTrace() (Phase F) additionally calls
 *   UpdateBeamDemand() whenever real RBDC demand arrives from the network.
 *   Before Phase F is active the provider falls back to its own policy
 *   (e.g., round-robin) because all demands are 0.
 *
 * Conf format (mirrors original SNS3 static BSTP line format):
 *   validity, beamId1, beamId2, ...
 *   e.g. "1, 3, 7" → validityInSuperframes=1, activeBeams={3,7}
 *
 * Deployment note:
 *   This file is deployed to contrib/satellite/helper/ and listed in
 *   contrib/satellite/CMakeLists.txt.  It does not modify any pre-existing
 *   SNS3 class, but it does extend the contrib/satellite build graph.
 */

#ifndef SAT_DYNAMIC_BSTP_PROVIDER_H
#define SAT_DYNAMIC_BSTP_PROVIDER_H

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <vector>

namespace ns3
{

// ── SatDynamicBstpProvider ────────────────────────────────────────────────
//
// Pure-virtual base class.  Concrete implementations must override all four
// virtual methods.  SatBhHelper stores a Ptr<SatDynamicBstpProvider> and
// calls them through this interface, enabling policy swap without touching
// the helper.
//
class SatDynamicBstpProvider : public Object
{
  public:
    static TypeId GetTypeId();

    // ── Beam registry ─────────────────────────────────────────────────────
    //
    // Metadata per enabled beam (mirrors SatStaticBstp::AddEnabledBeamInfo).
    // The provider uses this to (a) validate that selected beams exist, and
    // (b) apply per-GW feederFreqId exclusion constraints.
    //
    struct BeamInfo
    {
        uint32_t beamId;       ///< 1-indexed beam ID (SNS3 convention)
        uint32_t userFreqId;   ///< User-link frequency band ID
        uint32_t feederFreqId; ///< Feeder-link frequency band ID
        uint32_t gwId;         ///< Gateway serving this beam
    };

    // ── Output configuration ──────────────────────────────────────────────
    //
    // Single BHTP window returned by GetNextConf().
    // Maps to one row of the original static BSTP file:
    //   validityInSuperframes, beamId1, beamId2, ...
    //
    struct Conf
    {
        uint32_t              validityInSuperframes; ///< ≥ 1; how many superframes this plan is valid
        std::vector<uint32_t> activeBeams;           ///< Beam IDs to activate; |set| ≤ MaxActiveBeams
    };

    // ── API ───────────────────────────────────────────────────────────────

    /// Register one enabled beam (call once per beam after scenario creation).
    /// Must be called before GetNextConf() to populate the valid-beam set.
    virtual void AddEnabledBeamInfo(uint32_t beamId,
                                    uint32_t userFreqId,
                                    uint32_t feederFreqId,
                                    uint32_t gwId) = 0;

    /// Update demand estimate for beamId [kbps].
    /// Called by SatBhHelper::OnBacklogRequestTrace() (Phase F) every DAMA cycle.
    /// Concrete implementations should accumulate or replace demand per cycle.
    virtual void UpdateBeamDemand(uint32_t beamId, double demandKbps) = 0;

    /// Update raw backlog queue depth for beamId [bytes].
    /// Optional: concrete implementations may ignore this if scoring uses only kbps.
    virtual void UpdateBeamBacklog(uint32_t beamId, uint64_t queueBytes) = 0;

    /// Compute and return the next BSTP configuration window.
    /// Called once every T_p by SatBhHelper::RunDynamicBstpCycle().
    /// @param now  Current simulation time (used for fairness / age computations).
    /// @return     Conf with validityInSuperframes ≥ 1 and activeBeams non-empty.
    virtual Conf GetNextConf(Time now) = 0;

    /// Validate a Conf against the registered enabled-beam set and constraints.
    /// Called by SatBhHelper after GetNextConf() before converting to SatBhTimePlan.
    /// Returns true if the conf is legal; false triggers a fallback or warning.
    virtual bool ValidateConf(const Conf& conf) const = 0;
};

} // namespace ns3

#endif // SAT_DYNAMIC_BSTP_PROVIDER_H
