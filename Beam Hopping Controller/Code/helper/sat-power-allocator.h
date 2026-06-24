/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-power-allocator.h
 *
 * SatPowerAllocator — Slot-scale iterative water-filling power optimizer.
 * (Layer2.md §5.5, Phase D)
 *
 * Responsibilities:
 *   Optimize per-beam TX power under a total budget constraint every T_slot
 *   to maximize sum-rate (Σ log2(1+SINR_k)) subject to Σ p_k ≤ P_total.
 *
 * Algorithm: Iterative Water-Filling Algorithm (IWFA)
 *   Each outer iteration fixes all other beams' power as interference and
 *   solves a single-beam water-filling problem via bisection.
 *   Converges when ||Δp||_2 < ConvergenceEpsilon or MaxIterations reached.
 *
 *   SINR model:
 *     SINR_k = (p_k × g_k) / (N0 + f × Σ_{j≠k} p_j × g_j)
 *   where
 *     g_k   = channel gain (G_tx × G_rx / FSL, linear; default 1.0 in Phase D)
 *     N0    = noise power [W]  (attr NoisePowerDbw, default −126.47 dBW)
 *     f     = inter-beam interference leakage factor (attr InterferenceFactor,
 *             default 0.01 for SMALL-radius beams)
 *
 * SNS3 hook (ApplyPower):
 *   ApplyPower() fires ApplyPowerCallback per beam:
 *     Ptr<SatOrbiterUserPhy> phy = dev->GetUserPhy(beamId);
 *     phy->SetTxMaxPowerDbw(txMaxPowerDbw);   // [dBW]
 *     phy->Initialize();                       // recomputes m_eirpWoGainW
 *   Callback wired by SatBhHelper in Phase E.
 *   In Phase D: callback absent → ApplyPower() logs only (same pattern as
 *   SatUserAssociator::MoveUtCallback in Phase C).
 *
 * Unit contract (Layer2.md §5.5):
 *   Input  : TotalPowerBudgetDbm [dBm] — total TX budget
 *   Output : PowerMap beam → txMaxPowerDbw [dBW]
 *            (TX max power before subtracting output/pointing/OBO/antenna losses)
 *   Internal optimisation is done in linear [W]; converted to dBW for output.
 *
 * Implementation phase: Phase D (Layer2.md §7).
 */

#ifndef SAT_BH_POWER_ALLOCATOR_H
#define SAT_BH_POWER_ALLOCATOR_H

#include "ns3/object.h"

#include <cstdint>
#include <functional>
#include <map>
#include <vector>

namespace ns3
{

// ── PowerMap ──────────────────────────────────────────────────────────────
//
// Output of Allocate(): maps each active beam ID to its allocated TX max
// power in dBW.  Written into BeamConfig::allocatedPowerDbw by
// SatResourceManager.
//
using PowerMap = std::map<uint32_t, double>; // beam → txMaxPowerDbw [dBW]

// ── ApplyPowerCallback ────────────────────────────────────────────────────
//
// Fired once per beam by ApplyPower().
// Phase E wires this to SatOrbiterUserPhy::SetTxMaxPowerDbw().
// Until then, ApplyPower() logs the intended action.
//
using ApplyPowerCallback =
    std::function<void(uint32_t beamId, double txMaxPowerDbw)>;

// ── SatPowerAllocator ─────────────────────────────────────────────────────

class SatPowerAllocator : public Object
{
  public:
    static TypeId GetTypeId();
    SatPowerAllocator();

    // ── Wiring (called by SatBhHelper before Install()) ───────────────────

    /// Register the SNS3 write-back callback.
    /// Called by SatBhHelper in Phase E when SatOrbiterUserPhy is available.
    void SetApplyPowerCallback(ApplyPowerCallback cb) { m_applyPowerCb = cb; }

    // ── Core API ──────────────────────────────────────────────────────────

    /// Run IWFA over active beams and return optimised power map.
    ///
    /// @param activeBeams  Beam IDs active in this slot.
    /// @param channelGains beam → G_tx × G_rx / FSL (linear).
    ///                     Defaults to 1.0 per beam in Phase D stub.
    /// @param demandKbps   beam → total UT demand [kbps].
    ///                     Used as tiebreaker when gains are equal.
    /// @return PowerMap    beam → txMaxPowerDbw [dBW].
    PowerMap Allocate(const std::vector<uint32_t>&   activeBeams,
                      const std::map<uint32_t, double>& channelGains,
                      const std::map<uint32_t, double>& demandKbps);

    /// Write back optimised powers to SNS3 via ApplyPowerCallback.
    /// No-op (log only) until callback is wired in Phase E.
    void ApplyPower(const PowerMap& powers);

  private:
    // ── Algorithm ─────────────────────────────────────────────────────────

    /// Iterative Water-Filling Algorithm (IWFA).
    /// Returns power map in linear [W]; caller converts to dBW.
    PowerMap RunIWFA(const std::vector<uint32_t>&    beams,
                     const std::map<uint32_t, double>& gains) const;

    /// Compute effective noise for each beam (thermal + inter-beam interference).
    /// N_k_eff = N0 + f × Σ_{j≠k} p_j × g_j
    std::map<uint32_t, double> ComputeEffectiveNoise(
        const std::vector<uint32_t>&    beams,
        const std::map<uint32_t, double>& gains,
        const PowerMap&                   powerW) const;

    /// Single water-filling step: given per-beam effective noise N_k and gain g_k,
    /// find p_k = (μ − N_k/g_k)^+ with water level μ s.t. Σ p_k = P_total.
    /// Uses bisection on μ (O(K log(1/ε))).
    PowerMap WaterFillStep(const std::vector<uint32_t>&    beams,
                           const std::map<uint32_t, double>& gains,
                           const std::map<uint32_t, double>& effNoise,
                           double                            totalPowerW) const;

    /// L2 norm of the difference between two power maps (convergence check).
    double ConvergenceNorm(const PowerMap& pOld, const PowerMap& pNew) const;

    // ── Attribute accessors ───────────────────────────────────────────────

    void   SetTotalPowerDbm(double dbm) { m_totalPowerBudgetDbm = dbm; }
    double GetTotalPowerDbm() const     { return m_totalPowerBudgetDbm; }

    // ── State ─────────────────────────────────────────────────────────────

    double   m_totalPowerBudgetDbm;  ///< Total TX budget [dBm]  (attr: TotalPowerBudgetDbm)
    double   m_noisePowerDbw;        ///< Thermal noise floor [dBW] (attr: NoisePowerDbw)
    uint32_t m_maxIterations;        ///< IWFA outer iteration cap (attr: MaxIterations)
    double   m_convergenceEps;       ///< Convergence threshold ||Δp||_2 (attr: ConvergenceEpsilon)
    double   m_interferenceFactor;   ///< Cross-beam leakage fraction (attr: InterferenceFactor)

    ApplyPowerCallback m_applyPowerCb; ///< SNS3 write-back; null until Phase E
};

} // namespace ns3

#endif // SAT_BH_POWER_ALLOCATOR_H
