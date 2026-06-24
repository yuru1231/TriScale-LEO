/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-precoder.cc
 *
 * SatBhPrecoder — STUB implementation (Phase 3, not yet active).
 *
 * ComputeMMSE() returns MmseResult{.valid = false} in this stub.
 * Interfaces are final; only this .cc changes when Phase 3 is implemented.
 *
 * Implementation checklist (Layer2.md Step 7):
 *   [ ] Cholesky decomposition and inversion (CholeskyInvert)
 *   [ ] Matrix multiply and transpose helpers (MatMul, Transpose)
 *   [ ] Condition number computation (ComputeConditionNumber)
 *   [ ] Full MMSE W = H^H × (H × H^H + σ²I)^{-1} (ComputeMMSE)
 *   [ ] NS_LOG_WARN when |C_k| > m_condWarnSize (default 3)
 *   [ ] OnChannelEstimateReceived: store latest H per cluster
 *   [ ] Connect to ChannelEstimation trace in SatBhHelper
 */

#include "sat-bh-precoder.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatBhPrecoder");
NS_OBJECT_ENSURE_REGISTERED(SatBhPrecoder);

TypeId
SatBhPrecoder::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatBhPrecoder")
            .SetParent<Object>()
            .AddConstructor<SatBhPrecoder>()
            .AddAttribute("Enabled",
                          "Enable MMSE precoding (requires cluster size ≥ 2)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&SatBhPrecoder::m_enabled),
                          MakeBooleanChecker())
            .AddAttribute("NoisePowerDb",
                          "Default noise power σ²_n in dB (used when CSI not available)",
                          DoubleValue(-110.0),
                          MakeDoubleAccessor(&SatBhPrecoder::m_noisePowerDb),
                          MakeDoubleChecker<double>(-200.0, 0.0))
            .AddAttribute("CondWarnSize",
                          "Log NS_LOG_WARN for condition number when cluster size > this",
                          UintegerValue(3),
                          MakeUintegerAccessor(&SatBhPrecoder::m_condWarnSize),
                          MakeUintegerChecker<uint32_t>(1, 100));
    return tid;
}

SatBhPrecoder::SatBhPrecoder()
    : m_enabled(false),
      m_noisePowerDb(-110.0),
      m_computeCount(0),
      m_condWarnSize(3)
{
    NS_LOG_INFO("SatBhPrecoder: constructed (Phase 3 stub — not yet implemented)");
}

// ── Control ───────────────────────────────────────────────────────────────

void
SatBhPrecoder::SetEnabled(bool enable)
{
    m_enabled = enable;
    NS_LOG_INFO("SatBhPrecoder: MMSE " << (enable ? "ENABLED" : "DISABLED"));
}

// ── MMSE computation ──────────────────────────────────────────────────────

MmseResult
SatBhPrecoder::ComputeMMSE(const std::vector<uint32_t>& clusterBeams,
                             const ChannelMatrix&          /*H*/,
                             double                        /*noisePower*/)
{
    MmseResult result;
    result.conditionNumber = 0.0;
    result.valid           = false;

    if (!m_enabled)
    {
        NS_LOG_DEBUG("SatBhPrecoder::ComputeMMSE: MMSE disabled, skipping");
        return result;
    }

    if (clusterBeams.size() < 2)
    {
        NS_LOG_DEBUG("SatBhPrecoder::ComputeMMSE: cluster size "
                     << clusterBeams.size() << " < 2, MMSE not activated");
        return result;
    }

    // TODO Phase 3:
    //   1. H^H (transpose of H)
    //   2. H × H^H
    //   3. Add σ²_n × I
    //   4. CholeskyInvert → (H × H^H + σ²I)^{-1}
    //   5. W = H^H × (...)^{-1}
    //   6. Compute condition number; warn if cluster > m_condWarnSize
    //   7. result.W = W; result.valid = true

    NS_LOG_DEBUG("SatBhPrecoder::ComputeMMSE: cluster size=" << clusterBeams.size()
                 << " [STUB — no computation]");

    m_computeCount++;
    return result;
}

void
SatBhPrecoder::OnChannelEstimateReceived(const std::vector<uint32_t>& clusterBeams,
                                          const ChannelMatrix&          /*H*/)
{
    // TODO Phase 3: store H per cluster for next ComputeMMSE() call
    NS_LOG_DEBUG("SatBhPrecoder::OnChannelEstimateReceived cluster size="
                 << clusterBeams.size() << " [STUB]");
}

// ── Private matrix operations (stubs) ────────────────────────────────────

double
SatBhPrecoder::ComputeConditionNumber(const ChannelMatrix& /*A*/) const
{
    // TODO Phase 3: SVD-based condition number = σ_max / σ_min
    return 0.0;
}

bool
SatBhPrecoder::CholeskyInvert(const ChannelMatrix& /*A*/, ChannelMatrix& /*result*/) const
{
    // TODO Phase 3: Cholesky decomposition: A = L × L^T → A^{-1} via back-substitution
    return false;
}

ChannelMatrix
SatBhPrecoder::MatMul(const ChannelMatrix& /*A*/, const ChannelMatrix& /*B*/) const
{
    // TODO Phase 3: standard matrix multiply C[i][j] = Σ_k A[i][k] × B[k][j]
    return {};
}

ChannelMatrix
SatBhPrecoder::Transpose(const ChannelMatrix& A) const
{
    // TODO Phase 3: T[j][i] = A[i][j]
    if (A.empty()) return {};
    const size_t rows = A.size();
    const size_t cols = A[0].size();
    ChannelMatrix T(cols, std::vector<double>(rows, 0.0));
    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++)
            T[j][i] = A[i][j];
    return T; // Transpose is safe to implement even in stub
}

} // namespace ns3
