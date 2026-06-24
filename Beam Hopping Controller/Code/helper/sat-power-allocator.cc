/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-power-allocator.cc
 *
 * SatPowerAllocator implementation.
 * See sat-power-allocator.h for design rationale and full documentation.
 */

#include "sat-power-allocator.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatPowerAllocator");
NS_OBJECT_ENSURE_REGISTERED(SatPowerAllocator);

TypeId
SatPowerAllocator::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatPowerAllocator")
            .SetParent<Object>()
            .AddConstructor<SatPowerAllocator>()
            .AddAttribute("TotalPowerBudgetDbm",
                          "Total satellite TX power budget [dBm]",
                          DoubleValue(43.0),
                          MakeDoubleAccessor(&SatPowerAllocator::SetTotalPowerDbm,
                                             &SatPowerAllocator::GetTotalPowerDbm),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddAttribute("NoisePowerDbw",
                          "Thermal noise floor at receiver [dBW]",
                          DoubleValue(-126.47),
                          MakeDoubleAccessor(&SatPowerAllocator::m_noisePowerDbw),
                          MakeDoubleChecker<double>(-200.0, 0.0))
            .AddAttribute("MaxIterations",
                          "Maximum IWFA outer iterations before forced termination",
                          UintegerValue(30),
                          MakeUintegerAccessor(&SatPowerAllocator::m_maxIterations),
                          MakeUintegerChecker<uint32_t>(1, 1000))
            .AddAttribute("ConvergenceEpsilon",
                          "IWFA convergence threshold ||Δp||_2 [W]",
                          DoubleValue(0.001),
                          MakeDoubleAccessor(&SatPowerAllocator::m_convergenceEps),
                          MakeDoubleChecker<double>(1e-10, 1.0))
            .AddAttribute("InterferenceFactor",
                          "Cross-beam interference leakage fraction (0 = no ICI, 1 = full)",
                          DoubleValue(0.01),
                          MakeDoubleAccessor(&SatPowerAllocator::m_interferenceFactor),
                          MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

SatPowerAllocator::SatPowerAllocator()
    : m_totalPowerBudgetDbm(43.0),
      m_noisePowerDbw(-126.47),
      m_maxIterations(30),
      m_convergenceEps(0.001),
      m_interferenceFactor(0.01)
{
    NS_LOG_INFO("SatPowerAllocator: constructed"
                << " P_total=" << m_totalPowerBudgetDbm << "dBm"
                << " N0=" << m_noisePowerDbw << "dBW"
                << " maxIter=" << m_maxIterations
                << " eps=" << m_convergenceEps
                << " ici=" << m_interferenceFactor);
}

// ── Core API ──────────────────────────────────────────────────────────────

PowerMap
SatPowerAllocator::Allocate(const std::vector<uint32_t>&    activeBeams,
                             const std::map<uint32_t, double>& channelGains,
                             const std::map<uint32_t, double>& demandKbps)
{
    NS_LOG_INFO("SatPowerAllocator::Allocate"
                << " beams=" << activeBeams.size()
                << " P_total=" << m_totalPowerBudgetDbm << "dBm");

    if (activeBeams.empty())
        return {};

    // Resolve channel gains: use provided value; default to 1.0 if not set.
    // Phase E will inject real G_tx × G_rx / FSL from SNS3 channel estimates.
    std::map<uint32_t, double> gains;
    for (uint32_t bid : activeBeams)
    {
        auto it = channelGains.find(bid);
        gains[bid] = (it != channelGains.end() && it->second > 0.0) ? it->second : 1.0;
    }

    // Run IWFA in linear [W]
    PowerMap powerW = RunIWFA(activeBeams, gains);

    // Convert linear W → txMaxPowerDbw [dBW] for SNS3 output
    // txMaxPowerDbw is the value passed to SatOrbiterUserPhy::SetTxMaxPowerDbw().
    // It equals the EIRP-without-gain (m_eirpWoGainW) in dBW, which SNS3 maps
    // internally by subtracting output/pointing/OBO/antenna losses.
    // In Phase D we output raw power dBW; Phase E adds the loss offset if needed.
    PowerMap powerDbw;
    for (const auto& [bid, pW] : powerW)
    {
        // Guard: clamp to a safe minimum to avoid log(0)
        double safeW      = std::max(pW, 1e-12);
        powerDbw[bid]     = 10.0 * std::log10(safeW); // W → dBW
        NS_LOG_DEBUG("SatPowerAllocator beam=" << bid
                     << " p=" << safeW * 1000.0 << "mW"
                     << " → " << powerDbw[bid] << "dBW");
    }

    return powerDbw;
}

void
SatPowerAllocator::ApplyPower(const PowerMap& powers)
{
    if (powers.empty())
        return;

    if (!m_applyPowerCb)
    {
        // Phase D: callback not yet wired.  Log intended writes.
        for (const auto& [bid, dbw] : powers)
        {
            NS_LOG_INFO("SatPowerAllocator::ApplyPower (log-only, Phase E will wire)"
                        << " beamId=" << bid
                        << " txMaxPowerDbw=" << dbw);
        }
        return;
    }

    // Phase E+: fire callback to write-back into SatOrbiterUserPhy.
    for (const auto& [bid, dbw] : powers)
    {
        NS_LOG_INFO("SatPowerAllocator::ApplyPower beamId=" << bid
                    << " txMaxPowerDbw=" << dbw << "dBW");
        m_applyPowerCb(bid, dbw);
    }
}

// ── IWFA algorithm ────────────────────────────────────────────────────────

PowerMap
SatPowerAllocator::RunIWFA(const std::vector<uint32_t>&    beams,
                            const std::map<uint32_t, double>& gains) const
{
    // Convert total power budget: dBm → W
    // P_W = 10^((dBm - 30) / 10)
    const double totalPowerW = std::pow(10.0, (m_totalPowerBudgetDbm - 30.0) / 10.0);
    const double initPowerW  = totalPowerW / static_cast<double>(beams.size());

    NS_LOG_DEBUG("SatPowerAllocator::RunIWFA"
                 << " P_total=" << totalPowerW * 1000.0 << "mW"
                 << " initPerBeam=" << initPowerW * 1000.0 << "mW");

    // Initialise with equal power allocation
    PowerMap pW;
    for (uint32_t bid : beams)
        pW[bid] = initPowerW;

    for (uint32_t iter = 0; iter < m_maxIterations; iter++)
    {
        PowerMap pOld = pW;

        // For each beam, compute effective noise (thermal + ICI from all others),
        // then run one water-filling step.
        std::map<uint32_t, double> effNoise = ComputeEffectiveNoise(beams, gains, pW);
        pW = WaterFillStep(beams, gains, effNoise, totalPowerW);

        double norm = ConvergenceNorm(pOld, pW);
        NS_LOG_DEBUG("SatPowerAllocator::RunIWFA iter=" << iter + 1
                     << " ||Δp||=" << norm * 1000.0 << "mW");

        if (norm < m_convergenceEps)
        {
            NS_LOG_INFO("SatPowerAllocator: IWFA converged at iter=" << iter + 1
                        << " ||Δp||=" << norm * 1000.0 << "mW");
            break;
        }
    }

    return pW;
}

std::map<uint32_t, double>
SatPowerAllocator::ComputeEffectiveNoise(const std::vector<uint32_t>&    beams,
                                          const std::map<uint32_t, double>& gains,
                                          const PowerMap&                   powerW) const
{
    // Thermal noise in W: N0 = 10^(m_noisePowerDbw / 10)
    const double N0 = std::pow(10.0, m_noisePowerDbw / 10.0);

    // Total weighted power across all beams: Σ p_j × g_j
    double totalWeighted = 0.0;
    for (uint32_t bid : beams)
    {
        double g = gains.count(bid) ? gains.at(bid) : 1.0;
        double p = powerW.count(bid) ? powerW.at(bid) : 0.0;
        totalWeighted += p * g;
    }

    // N_k_eff = N0 + f × (totalWeighted − p_k × g_k)
    std::map<uint32_t, double> effNoise;
    for (uint32_t bid : beams)
    {
        double g   = gains.count(bid) ? gains.at(bid) : 1.0;
        double p   = powerW.count(bid) ? powerW.at(bid) : 0.0;
        double ici = m_interferenceFactor * (totalWeighted - p * g);
        effNoise[bid] = N0 + ici;
    }

    return effNoise;
}

PowerMap
SatPowerAllocator::WaterFillStep(const std::vector<uint32_t>&    beams,
                                  const std::map<uint32_t, double>& gains,
                                  const std::map<uint32_t, double>& effNoise,
                                  double                             totalPowerW) const
{
    // Water-filling: p_k = max(0, μ − N_k/g_k)
    // Water level μ satisfies Σ max(0, μ − N_k/g_k) = totalPowerW
    //
    // "thresholds" τ_k = N_k / g_k  (the point at which beam k is turned on)
    // Sort beams by τ_k ascending; any beam with τ_k ≥ μ gets p_k = 0.
    //
    // Bisection on μ: lo = 0, hi = totalPowerW + max(τ_k)

    // Build τ_k = N_k / g_k for each beam
    std::vector<std::pair<double, uint32_t>> thresholds; // (τ_k, beamId)
    for (uint32_t bid : beams)
    {
        double g = gains.count(bid) ? gains.at(bid) : 1.0;
        double N = effNoise.count(bid) ? effNoise.at(bid) : 1e-12;
        // Guard against g == 0 (should not happen but be safe)
        double tau = (g > 1e-30) ? N / g : 1e30;
        thresholds.push_back({tau, bid});
    }

    double maxTau = 0.0;
    for (const auto& [tau, bid] : thresholds)
        maxTau = std::max(maxTau, tau);

    double lo = 0.0;
    double hi = totalPowerW + maxTau + 1.0; // safe upper bound

    // Bisection: find μ such that f(μ) = Σ max(0, μ − τ_k) = totalPowerW
    // f is monotonically non-decreasing in μ
    for (int bisect = 0; bisect < 200; bisect++)
    {
        double mu   = 0.5 * (lo + hi);
        double sumP = 0.0;
        for (const auto& [tau, bid] : thresholds)
            sumP += std::max(0.0, mu - tau);

        if (sumP > totalPowerW)
            hi = mu;
        else
            lo = mu;

        if (hi - lo < 1e-12)
            break;
    }

    double mu = 0.5 * (lo + hi);

    // Compute final p_k and normalise to ensure Σ p_k = totalPowerW exactly.
    PowerMap pW;
    double   sumP = 0.0;
    for (const auto& [tau, bid] : thresholds)
    {
        double p = std::max(0.0, mu - tau);
        pW[bid]  = p;
        sumP    += p;
    }

    // Rescale to exact budget (compensates for bisection residual)
    if (sumP > 1e-30)
    {
        double scale = totalPowerW / sumP;
        for (auto& [bid, p] : pW)
            p *= scale;
    }

    return pW;
}

double
SatPowerAllocator::ConvergenceNorm(const PowerMap& pOld, const PowerMap& pNew) const
{
    double sumSq = 0.0;
    for (const auto& [bid, pn] : pNew)
    {
        double po   = pOld.count(bid) ? pOld.at(bid) : 0.0;
        double diff = pn - po;
        sumSq += diff * diff;
    }
    return std::sqrt(sumSq);
}

} // namespace ns3
