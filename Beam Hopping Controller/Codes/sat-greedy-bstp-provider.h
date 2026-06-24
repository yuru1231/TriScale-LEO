/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-greedy-bstp-provider.h
 *
 * SatGreedyBstpProvider — Demand-greedy, fairness-aware implementation of
 * SatDynamicBstpProvider.
 *
 * Scheduling policy:
 *   Each BHTP period, select up to MaxActiveBeams beams using a composite score:
 *
 *     score(b) = BacklogWeight  × demandKbps(b)
 *              + FairnessWeight × timeSinceLastServed(b) [seconds]
 *
 *   Beams whose starvationCount ≥ StarvationThreshold are unconditionally
 *   included in the active set (forced override) to prevent indefinite starvation
 *   of low-demand beams.
 *
 *   Fallback (all demands == 0, e.g. during warm-up before Phase F wires traces):
 *     Round-robin over all enabled beams in ascending beamId order.
 *
 * Constraint enforcement (ValidateConf):
 *   1. All active beams must be in the registered enabled-beam set.
 *   2. No duplicate beam IDs within one Conf.
 *   3. |activeBeams| ≤ MaxActiveBeams.
 *   4. validityInSuperframes ≥ 1.
 *   5. No two beams on the same GW may share the same feederFreqId
 *      (hardware constraint: one feeder uplink per frequency per GW).
 *
 * Parameters (ns3 Attributes):
 *   MaxActiveBeams       — K, max simultaneous beams (default 2)
 *   BacklogWeight        — demand component weight (default 1.0)
 *   FairnessWeight       — age component weight   (default 0.5)
 *   ValidityInSuperframes— window length returned in Conf (default 1)
 *   StarvationThreshold  — cycles before forced inclusion (default 5)
 */

#ifndef SAT_GREEDY_BSTP_PROVIDER_H
#define SAT_GREEDY_BSTP_PROVIDER_H

#include "sat-dynamic-bstp-provider.h"

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

class SatGreedyBstpProvider : public SatDynamicBstpProvider
{
  public:
    static TypeId GetTypeId();
    SatGreedyBstpProvider();

    // ── SatDynamicBstpProvider API ────────────────────────────────────────

    void AddEnabledBeamInfo(uint32_t beamId,
                            uint32_t userFreqId,
                            uint32_t feederFreqId,
                            uint32_t gwId) override;

    void UpdateBeamDemand(uint32_t beamId, double demandKbps) override;
    void UpdateBeamBacklog(uint32_t beamId, uint64_t queueBytes) override;

    Conf GetNextConf(Time now) override;
    bool ValidateConf(const Conf& conf) const override;

  private:
    // ── Per-beam state ────────────────────────────────────────────────────

    struct BeamState
    {
        BeamInfo info;

        double   demandKbps{0.0};    ///< Latest RBDC demand [kbps] (from UpdateBeamDemand)
        uint64_t backlogBytes{0};    ///< Raw queue depth [bytes]  (from UpdateBeamBacklog)

        Time     lastServedTime;     ///< Simulation time when beam was last in active set
        uint32_t starvationCount{0}; ///< Consecutive cycles beam was NOT selected
    };

    // ── Helpers ───────────────────────────────────────────────────────────

    /// Score function: higher = more urgently needs service.
    double ComputeScore(const BeamState& s, Time now) const;

    /// Round-robin fallback: select first K beams in ascending beamId order,
    /// advancing the internal round-robin pointer.
    Conf FallbackRoundRobin();

    /// Build a Conf from the given list of selected beam IDs.
    Conf MakeConf(const std::vector<uint32_t>& selected) const;

    // ── State ─────────────────────────────────────────────────────────────

    std::map<uint32_t, BeamState> m_beams;      ///< Key = beamId
    uint32_t                      m_rrPointer;   ///< Round-robin offset (fallback)
    uint32_t                      m_cycleCount;  ///< Total GetNextConf() calls

    // ── Attributes ────────────────────────────────────────────────────────

    uint32_t m_maxActiveBeams;        ///< K
    double   m_backlogWeight;         ///< Score: demand component coefficient
    double   m_fairnessWeight;        ///< Score: age component coefficient
    uint32_t m_validityInSuperframes; ///< Conf::validityInSuperframes to return
    uint32_t m_starvationThreshold;   ///< Forced-inclusion after this many skipped cycles
};

} // namespace ns3

#endif // SAT_GREEDY_BSTP_PROVIDER_H
