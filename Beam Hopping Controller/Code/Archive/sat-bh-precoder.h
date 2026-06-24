/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-precoder.h
 *
 * SatBhPrecoder — GW-side MMSE precoder for interfering beam clusters.
 *
 * Implements spec Section 6.5 and Layer2.md Section 6.5 / 7.
 * Implementation phase: Phase 3 (Layer2.md Section 10).
 *
 * Responsibilities (GW side):
 *   - Activated only when a cluster contains ≥ 2 simultaneously active beams
 *   - Compute MMSE precoding matrix W for each active cluster
 *   - Apply W to transmitted signals (via SNS3 hook, to be confirmed)
 *   - Log numerical stability indicator (condition number) if |C_k| > 3
 *
 * MMSE formula (spec Section 6.5):
 *   W = H^H × (H × H^H + σ²_n × I)^{-1}
 *   H = channel matrix for the cluster (K_c × K_c, K_c = cluster size)
 *   σ²_n = noise variance
 *
 * CSI approximation:
 *   Per spec, CSI is approximated from a channel snapshot at each slot start.
 *   In ns-3, this maps to SatPhyRxCarrier::SNR (fallback) or
 *   ChannelEstimation trace (preferred — see Hook checklist, Layer2.md Step 0).
 *
 * Numerical stability:
 *   Matrix inversion uses Cholesky decomposition.
 *   If cluster size |C_k| > 3, log condition number to WARN.
 *
 * Activation condition:
 *   Only activated when cluster size ≥ 2.
 *   Single-beam clusters bypass MMSE entirely.
 *   Controlled globally by the m_enabled flag (set via SatBhHelper).
 *
 * Non-invasive constraint (Layer2.md Principle A/B):
 *   Precoding is applied at GW side, not on the satellite payload.
 *   Connection to SNS3 transmission pipeline deferred to Hook feasibility.
 */

#ifndef SAT_BH_PRECODER_H
#define SAT_BH_PRECODER_H

#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace ns3
{

// ── Channel matrix type ───────────────────────────────────────────────────
//
// Simplified representation: K_c × K_c complex matrix as a flat vector of
// real-valued channel magnitude coefficients.
// Full complex-valued CSI to be added when SNS3 channel estimation hook
// is confirmed (Layer2.md Step 0).
//
using ChannelMatrix = std::vector<std::vector<double>>;

// ── MMSE result ───────────────────────────────────────────────────────────

struct MmseResult
{
    ChannelMatrix W;             ///< Computed precoding matrix (K_c × K_c)
    double        conditionNumber; ///< Numerical stability indicator
    bool          valid;          ///< False if matrix was singular or cluster size < 2
};

// ── SatBhPrecoder ─────────────────────────────────────────────────────────

class SatBhPrecoder : public Object
{
  public:
    static TypeId GetTypeId();
    SatBhPrecoder();

    // ── Control ───────────────────────────────────────────────────────────

    /// Enable or disable MMSE precoding globally.
    /// When disabled, ComputeMMSE() returns immediately with valid=false.
    void SetEnabled(bool enable);
    bool IsEnabled() const { return m_enabled; }

    // ── MMSE computation ──────────────────────────────────────────────────

    /// Compute MMSE precoding matrix for the given cluster of beam IDs.
    /// @param clusterBeams  Beam IDs in the cluster (must have ≥ 2 elements)
    /// @param H             Channel matrix snapshot at slot start (K_c × K_c)
    /// @param noisePower    Noise variance σ²_n (linear scale)
    /// @return MmseResult containing W, condition number, and validity flag
    MmseResult ComputeMMSE(const std::vector<uint32_t>& clusterBeams,
                            const ChannelMatrix&          H,
                            double                        noisePower);

    // ── CSI update hook ───────────────────────────────────────────────────

    /// Called at each slot start with a new channel estimate for the cluster.
    /// In Phase 3 stub: stores H for use by ComputeMMSE().
    /// In Phase 3 impl: connected to ChannelEstimation trace via SatBhHelper.
    void OnChannelEstimateReceived(const std::vector<uint32_t>& clusterBeams,
                                   const ChannelMatrix&          H);

    // ── Query ─────────────────────────────────────────────────────────────

    /// Return number of MMSE computations performed (for diagnostics)
    uint64_t GetComputeCount() const { return m_computeCount; }

  private:
    // ── Matrix operations (implemented in Phase 3) ────────────────────────

    /// Compute condition number of matrix A (for numerical stability logging)
    double ComputeConditionNumber(const ChannelMatrix& A) const;

    /// Invert matrix using Cholesky decomposition.
    /// Returns false if matrix is not positive-definite (singular).
    bool CholeskyInvert(const ChannelMatrix& A, ChannelMatrix& result) const;

    /// Multiply matrices A × B (K × K)
    ChannelMatrix MatMul(const ChannelMatrix& A, const ChannelMatrix& B) const;

    /// Transpose matrix A
    ChannelMatrix Transpose(const ChannelMatrix& A) const;

    // ── State ─────────────────────────────────────────────────────────────

    bool     m_enabled;        ///< Global MMSE on/off (attr: Enabled)
    double   m_noisePowerDb;   ///< Default noise power in dB (attr: NoisePowerDb)
    uint64_t m_computeCount;   ///< Number of MMSE computations performed
    uint32_t m_condWarnSize;   ///< Cluster size threshold for condition number logging (attr: CondWarnSize)
};

} // namespace ns3

#endif // SAT_BH_PRECODER_H
