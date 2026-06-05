/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-channel.h
 *
 * Physical-layer channel model for Multi-Beam LEO satellite simulation.
 * Mirrors channel.py from the Python framework.
 *
 * Beam gain model (exact UPA — matches Python channel.py):
 *   Same coordinate transform as get_angles_to_satellite() in utils.py.
 *   Closed-form Dirichlet kernel replaces the full steering-vector inner product:
 *     |beam_gain[u,j]|² = AF_x²(ΔΦ_x) · AF_y²(ΔΦ_y) / (Nx² · Ny² · Nbeams)
 *   where AF²(ΔΦ, N) = sin²(N·π·ΔΦ/2) / sin²(π·ΔΦ/2)
 *   Peak at beam centre: |beam_gain|² = 1/Nbeams  (identical to Python).
 *
 * Atmospheric loss model (simplified ITU-R P.618-13):
 *   A(ε) ≈ A_zenith / sin(ε),  A_zenith = 0.55 dB (Ka-band, Tokyo, p=1%).
 *   Differs from Python's itur library at non-nadir passes by < 0.3 dB.
 *
 * Rician fading (mirrors channel.get_Rician_fading_coefficient()):
 *   h = N(mu, sigma) + j·N(mu, sigma),  mu=√(K/(2(K+1))), sigma=√(1/(2(K+1)))
 *   Phase terms (Doppler, propagation delay) cancel in |·|² and are omitted.
 */

#ifndef SAT_MULTI_BEAM_CHANNEL_H
#define SAT_MULTI_BEAM_CHANNEL_H

#include "sat-multi-beam-config.h"
#include "sat-multi-beam-geometry.h"

#include <array>
#include <random>
#include <vector>

namespace ns3
{

// ---------------------------------------------------------------------------
// Per-user link result
// ---------------------------------------------------------------------------

/** UserLinkResult — channel metrics for one user at one simulation frame. */
struct UserLinkResult
{
    int    userId{-1};
    int    beamId{-1};         // assigned beam index (0..nBeams-1)
    double pathLossDb{0.0};    // FSPL + atmospheric (dB)
    double beamGainDb{0.0};    // UPA beam gain of the serving beam: to_dB(|beam_gain|²) + G_max (dB)
    double sinrDb{0.0};        // Signal-to-interference-plus-noise ratio (dB)
    double snrDb{0.0};         // Signal-to-noise ratio (no interference) (dB)
    double centerBeamGainDb{0.0}; // Gain of centre beam (beam 9): to_dB(|beam_gain[u,9]|²) + G_max — paper metric
};

// ---------------------------------------------------------------------------
// Path loss
// ---------------------------------------------------------------------------

/**
 * ComputeFSPL_dB
 * Free-space path loss in dB.
 *   L_fspl = 20·log10(4π·d·f / c)
 *
 * @param distanceM  Slant range (m).
 * @param freqHz     Carrier frequency (Hz).
 * @return           FSPL (dB, positive).
 */
double ComputeFSPL_dB(double distanceM, double freqHz);

/**
 * ComputeAtmosphericLoss_dB
 * Simplified ITU-R P.618-13 atmospheric attenuation for Ka-band slant path.
 * Model: A(ε) = A_zenith / sin(ε),  A_zenith fitted for 30 GHz / Tokyo / p=1%.
 * Clamped to a maximum of 25 dB at very low elevation angles (ε < 5°).
 *
 * @param elevationDeg  Elevation angle (degrees, 0–90).
 * @return              Atmospheric attenuation (dB, positive).
 */
double ComputeAtmosphericLoss_dB(double elevationDeg);

/**
 * ComputePathLoss_dB
 * Total path loss = FSPL + atmospheric.  Mirrors channel.path_loss().
 *
 * @param userPos  User position in local Cartesian (m).
 * @param satPos   Satellite position in local Cartesian (m).
 * @param cfg      Simulation configuration.
 * @return         Total path loss (dB, positive).
 */
double ComputePathLoss_dB(const Vec3& userPos,
                          const Vec3& satPos,
                          const SimConfig& cfg);

// ---------------------------------------------------------------------------
// Beam gain — exact UPA model matching channel.py
// ---------------------------------------------------------------------------

/**
 * ComputeUPABeamGainPower
 *
 * Returns the normalised beam gain power |beam_gain[u,j]|² for user u
 * and beam j, using the same UPA steering-vector model as Python's
 * get_effective_channel() / fixed_beam_steering().
 *
 * Formula:
 *   |beam_gain|² = AF_x²(ΔΦ_x) · AF_y²(ΔΦ_y) / (Nx² · Ny² · Nbeams)
 *
 * where AF²(N, ΔΦ) = sin²(N·π·ΔΦ/2) / sin²(π·ΔΦ/2)
 * and   ΔΦ = Φ_x_user − Φ_x_beam  (spatial-frequency difference per axis)
 *
 * The spatial frequencies (Φ_x, Φ_y) are computed via the same
 * coordinate transform as Python's get_angles_to_satellite():
 *   shift satellite to array corner → translate target → rotate (y-axis)
 *   → cart2pol3D → Φ_x = cos(φ)·sin(θ),  Φ_y = sin(φ)·sin(θ)
 *
 * Peak at beam centre: |beam_gain|² = 1/Nbeams  (identical to Python).
 *
 * Corresponding full beam gain metric (dB) = to_dB(result) + antenna_gain_dB,
 * which is what Python's calculate_simulation_result() reports.
 *
 * @param satPos      Satellite position, local Cartesian (m).
 * @param userPos     User position, local Cartesian (m).
 * @param beamCentre  Beam centre position on Earth surface (m).
 * @param cfg         Simulation configuration.
 * @return            Normalised beam gain power (linear, ≥ 0).
 */
double ComputeUPABeamGainPower(const Vec3& satPos,
                                const Vec3& userPos,
                                const Vec3& beamCentre,
                                const SimConfig& cfg);

// ---------------------------------------------------------------------------
// Fading — mirrors channel.get_Rician_fading_coefficient()
// ---------------------------------------------------------------------------

/**
 * SampleRicianAmplitude
 * Draw one Rician fading amplitude |h| for the given K-factor.
 *   h = √(K/(K+1))·(1+j) / √2  +  √(1/(K+1))·CN(0,1/2)
 *   Returns |h| (non-negative real).
 *   E[|h|²] = 1.
 *
 * @param K    Rician K-factor.
 * @param rng  Mersenne-Twister RNG (caller owns).
 * @return     Fading amplitude (linear, ≥0).
 */
double SampleRicianAmplitude(double K, std::mt19937& rng);

// ---------------------------------------------------------------------------
// Full per-frame computation
// ---------------------------------------------------------------------------

/**
 * ComputeFrameResults
 *
 * For one satellite position, compute the link results (path loss, beam gain,
 * SINR, SNR) for all users.  Mirrors the inner loop of simulation.py.
 *
 * Beam association: each user is assigned to the beam with the highest
 * macroscopic received power (strongest beam, ignoring fading).  This matches
 * the Python assignment:
 *   beam_index = argmax(|macro_channel|²) over beams.
 *
 * SINR formula:
 *   P_desired[u]     = |effective_channel[u, beam_index[u]]|²
 *   P_interference[u] = Σ_{j≠beam[u]} |effective_channel[u, j]|²
 *   SINR[u]          = P_desired / (P_interference + noise_power)
 *   SNR[u]           = P_desired / noise_power
 *
 * effective_channel[u,j] = √(Tx·G_ant·/ loss_linear[u]) · |h_rician[u]| · beam_gain_lin[u,j]
 * macro_channel[u,j]     = same without |h_rician|  (for beam assignment)
 *
 * @param satPos       Satellite position (m).
 * @param userPos      User positions (m).
 * @param beamCenters  19 beam centre positions (m).
 * @param cfg          Simulation configuration.
 * @param rng          RNG for Rician sampling (caller owns).
 * @param withFading   true → include Rician fading; false → macro only.
 * @return             One UserLinkResult per user.
 */
std::vector<UserLinkResult>
ComputeFrameResults(const Vec3&                     satPos,
                    const std::vector<Vec3>&         userPos,
                    const std::array<Vec3, 19>&      beamCenters,
                    const SimConfig&                 cfg,
                    std::mt19937&                    rng,
                    bool                             withFading = true);

} // namespace ns3

#endif // SAT_MULTI_BEAM_CHANNEL_H
