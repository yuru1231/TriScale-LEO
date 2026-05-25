/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-channel.cc
 *
 * Channel model implementation for Multi-Beam LEO simulation.
 * See sat-multi-beam-channel.h for full API documentation.
 *
 * Beam gain: exact UPA Dirichlet-kernel formula — matches Python channel.py.
 *   Replaces the previous Gaussian approximation.
 */

#include "sat-multi-beam-channel.h"

#include "ns3/boolean.h"
#include "ns3/channel-condition-model.h"
#include "ns3/double.h"
#include "ns3/geo-coordinate.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/satellite-constant-position-mobility-model.h"
#include "ns3/satellite-free-space-loss.h"
#include "ns3/satellite-mobility-model.h"
#include "ns3/satellite-utils.h"
#include "ns3/three-gpp-propagation-loss-model.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>

namespace ns3
{

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace
{

/** Convert linear power ratio to dB. */
inline double
ToDb(double linear)
{
    return SatUtils::LinearToDb(linear);
}

/** Convert dB to linear power ratio. */
inline double
FromDb(double db)
{
    return SatUtils::DbToLinear(db);
}

Ptr<GeocentricConstantPositionMobilityModel>
CreateNtnMobility(const Vec3& pos, const SimConfig& cfg)
{
    auto mobility = CreateObject<GeocentricConstantPositionMobilityModel>();
    mobility->SetCoordinateTranslationReferencePoint(
        Vector(cfg.latitudeCenterDeg, cfg.longitudeCenterDeg, 0.0));
    mobility->SetPosition(Vector(pos.x, pos.y, pos.z));
    return mobility;
}

Ptr<SatMobilityModel>
CreateSatelliteMobility(const GeoCoordinate& position)
{
    auto mobility = CreateObject<SatConstantPositionMobilityModel>();
    mobility->SetGeoPosition(position);
    return mobility;
}

Ptr<SatFreeSpaceLoss>
GetSatelliteFreeSpaceLossModel()
{
    static Ptr<SatFreeSpaceLoss> loss = CreateObject<SatFreeSpaceLoss>();
    return loss;
}

Ptr<ThreeGppNTNDenseUrbanPropagationLossModel>
GetNtnLosPropagationLossModel(double frequencyHz)
{
    static Ptr<ThreeGppNTNDenseUrbanPropagationLossModel> model;
    static double configuredFrequencyHz = -1.0;

    if (!model)
    {
        model = CreateObject<ThreeGppNTNDenseUrbanPropagationLossModel>();
        model->SetAttribute("ShadowingEnabled", BooleanValue(false));
        model->SetChannelConditionModel(CreateObject<AlwaysLosChannelConditionModel>());
    }

    if (configuredFrequencyHz != frequencyHz)
    {
        model->SetAttribute("Frequency", DoubleValue(frequencyHz));
        configuredFrequencyHz = frequencyHz;
    }

    return model;
}

/**
 * LocalCart2Pol3D — Cartesian → spherical (physics convention).
 * Mirrors Python's utils.cart2pol3D():
 *   phi   = sign(y) * arccos(x / sqrt(x²+y²))   in [-π, π]
 *   theta = arccos(z / r)                         in [0, π]
 */
inline void
LocalCart2Pol3D(double x, double y, double z,
                double& r, double& phi, double& theta)
{
    r = std::sqrt(x * x + y * y + z * z);
    const double xy = std::sqrt(x * x + y * y);
    if (xy < 1e-12)
    {
        phi = 0.0;
    }
    else
    {
        double sign = (y >= 0.0) ? 1.0 : -1.0;
        phi = sign * std::acos(std::max(-1.0, std::min(1.0, x / xy)));
    }
    theta = (r > 0.0) ? std::acos(std::max(-1.0, std::min(1.0, z / r))) : 0.0;
}

// ---------------------------------------------------------------------------
// ArrayTransform — precomputed per-frame rotation parameters.
//
// Phase 2.5 extension: adds a z-axis pre-rotation (cosZ, sinZ) that maps
// the satellite's actual horizontal direction to the +x axis before applying
// the existing y-axis rotation.  This fixes the Phase 2.0/2.2 approximation
// where BuildArrayTransform assumed satPos.y = 0 (arc model only).
//
// Original model: y-axis rotation only — mirrors Python get_angles_to_satellite()
//   which was designed for arc mode where the satellite moves in the x-z plane.
// Phase 2.5 model: z-pre-rotation → y-axis rotation — correct for any azimuth.
//
// Backward compatibility: when satPos.y = 0 (arc mode), satHoriz = |satPos.x|,
//   cosZ = ±1, sinZ = 0 → z-rotation reduces to identity or sign flip, identical
//   to original behaviour.
// ---------------------------------------------------------------------------

struct ArrayTransform
{
    // Shifted satellite position after z-pre-rotation (satellite in x-z plane)
    double satX{0.0};
    double satY{0.0};
    double satZ{0.0};
    // y-axis rotation coefficients (existing)
    double cosR{1.0};
    double sinR{0.0};
    // Phase 2.5: z-axis pre-rotation coefficients
    // R_z @ (E, N, U) = (cosZ·E + sinZ·N,  −sinZ·E + cosZ·U,  U)
    // Maps satellite's horizontal direction (E_s, N_s) → (+satHoriz, 0)
    double cosZ{1.0};
    double sinZ{0.0};
};

/**
 * BuildArrayTransform — compute rotation parameters from satPos.
 *
 * Phase 2.5 steps (extension of Python get_angles_to_satellite):
 *   0. (NEW) z-pre-rotation: cosZ = E_s / satHoriz, sinZ = N_s / satHoriz
 *      After z-rotation: satellite is at (satHoriz, 0, U_s) — in the x-z plane.
 *   1. Shift satellite to array corner using post-rotation x = satHoriz.
 *   2. tan_vec = [1, −satX / (satZ + r_earth)]
 *   3. rotation_angle = arccos(tan_vec[0] / |tan_vec|) × sign(satX) + π
 *   4. T = [[cosR, 0, −sinR], [0, 1, 0], [sinR, 0, cosR]]  (y-axis rotation)
 */
inline ArrayTransform
BuildArrayTransform(const Vec3& satPos, const SimConfig& cfg)
{
    ArrayTransform at;
    const double spacing = cfg.GetAntennaSpacing(); // λ/2

    // Step 0 (Phase 2.5) — z-axis pre-rotation.
    // satHoriz is the satellite's horizontal (East-North plane) distance.
    // R_z maps (E_s, N_s) → (satHoriz, 0) so satellite enters the y-axis
    // rotation in the x-z plane regardless of its true azimuth.
    // Threshold 1.0 m: satellite always > 500 km away, so satHoriz >> 1 m.
    const double satHoriz = std::sqrt(satPos.x * satPos.x + satPos.y * satPos.y);
    if (satHoriz > 1.0)
    {
        at.cosZ = satPos.x / satHoriz;
        at.sinZ = satPos.y / satHoriz;
    }
    else
    {
        // Satellite directly overhead — z-rotation is identity
        at.cosZ = 1.0;
        at.sinZ = 0.0;
    }

    // Step 1 — shift satellite to array corner.
    // After z-rotation the satellite's x = satHoriz, y = 0.
    at.satX = satHoriz - static_cast<double>(cfg.nAntennaX * cfg.nBeamsX) / 2.0 * spacing;
    at.satY = 0.0      - static_cast<double>(cfg.nAntennaY * cfg.nBeamsY) / 2.0 * spacing;
    at.satZ = satPos.z;

    // Steps 2-3 — rotation angle (unchanged: uses at.satX from z-rotated frame)
    const double tanVecZ   = -at.satX / (at.satZ + cfg.rEarthM);
    const double tanVecNorm = std::sqrt(1.0 + tanVecZ * tanVecZ);
    const double cosAngle  = std::max(-1.0, std::min(1.0, 1.0 / tanVecNorm));
    const double signX     = (at.satX >= 0.0) ? 1.0 : -1.0;
    const double rotAngle  = std::acos(cosAngle) * signX + M_PI;

    // Step 4 — rotation matrix coefficients (y-axis, unchanged)
    at.cosR = std::cos(rotAngle);
    at.sinR = std::sin(rotAngle);

    return at;
}

/**
 * GetSpatialFreqs — compute UPA spatial frequencies (Φ_x, Φ_y) for one position.
 *
 * Phase 2.5: applies z-pre-rotation to pos before the existing transform so
 * that all positions are in the same rotated frame as the satellite.
 *
 *   Step 0 (NEW): z-rotation  pos_r = R_z @ pos
 *   Step 1:       translate   rel   = pos_r − sat_shifted
 *   Step 2:       y-rotation  t     = T_y @ rel
 *   Step 3:       spherical   Φ_x = cos(φ)·sin(θ),  Φ_y = sin(φ)·sin(θ)
 */
inline void
GetSpatialFreqs(const Vec3& pos, const ArrayTransform& at,
                double& PhiX, double& PhiY)
{
    // Step 0 (Phase 2.5) — z-axis pre-rotation.
    // R_z: [[cosZ, sinZ, 0], [−sinZ, cosZ, 0], [0, 0, 1]]
    const double posXr = at.cosZ * pos.x + at.sinZ * pos.y;
    const double posYr = -at.sinZ * pos.x + at.cosZ * pos.y;
    const double posZr = pos.z;

    // Step 1 — translate relative to shifted satellite (in z-rotated frame)
    const double relX = posXr - at.satX;
    const double relY = posYr - at.satY;
    const double relZ = posZr - at.satZ;

    // Step 2 — apply y-axis rotation: T = [[cosR, 0, −sinR], [0, 1, 0], [sinR, 0, cosR]]
    const double tX = at.cosR * relX - at.sinR * relZ;
    const double tY = relY;
    const double tZ = at.sinR * relX + at.cosR * relZ;

    // Step 3 — spherical coordinates (physics convention)
    double r, phi, theta;
    LocalCart2Pol3D(tX, tY, tZ, r, phi, theta);

    PhiX = std::cos(phi) * std::sin(theta);
    PhiY = std::sin(phi) * std::sin(theta);
}

/**
 * DirichletKernel2 — squared magnitude of the length-N geometric series.
 *
 *   |Σ_{n=0}^{N-1} exp(j·π·ΔΦ·n)|²  =  sin²(N·π·ΔΦ/2) / sin²(π·ΔΦ/2)
 *
 * At ΔΦ = 0 (or any even integer): returns N² (L'Hôpital limit).
 */
inline double
DirichletKernel2(int N, double dPhi)
{
    const double halfArg = M_PI * dPhi / 2.0;
    const double denom   = std::sin(halfArg);
    if (std::abs(denom) < 1e-10)
    {
        // ΔΦ = 0 or grating lobe — return peak value N²
        return static_cast<double>(N) * N;
    }
    const double numer = std::sin(static_cast<double>(N) * halfArg);
    return (numer / denom) * (numer / denom);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// ComputeFSPL_dB
// ---------------------------------------------------------------------------

double
ComputeFSPL_dB(double distanceM, double freqHz)
{
    auto txMob = CreateSatelliteMobility(GeoCoordinate(0.0, 0.0, 0.0));
    auto rxMob = CreateSatelliteMobility(GeoCoordinate(0.0, 0.0, distanceM));

    return GetSatelliteFreeSpaceLossModel()->GetFsldB(txMob, rxMob, freqHz);
}

// ---------------------------------------------------------------------------
// ComputeAtmosphericLoss_dB
// ---------------------------------------------------------------------------

double
ComputeAtmosphericLoss_dB(double elevationDeg)
{
    const double defaultKaBandHz = 30.0e9;
    const double txPowerDbm = 0.0;
    const double distM = 600.0e3;
    const double elevRad = std::max(1.0, elevationDeg) * M_PI / 180.0;
    const Vec3 userPos{0.0, 0.0, 0.0};
    const Vec3 satPos{distM * std::cos(elevRad), 0.0, distM * std::sin(elevRad)};
    SimConfig cfg;
    cfg.centerFreqHz = defaultKaBandHz;

    auto userMob = CreateNtnMobility(userPos, cfg);
    auto satMob = CreateNtnMobility(satPos, cfg);
    const double ntnLoss = -GetNtnLosPropagationLossModel(defaultKaBandHz)
                                ->CalcRxPower(txPowerDbm, userMob, satMob);
    const double fspl = ComputeFSPL_dB(userMob->GetDistanceFrom(satMob), defaultKaBandHz);
    return std::max(0.0, ntnLoss - fspl);
}

// ---------------------------------------------------------------------------
// ComputePathLoss_dB
// ---------------------------------------------------------------------------

double
ComputePathLoss_dB(const Vec3& userPos, const Vec3& satPos, const SimConfig& cfg)
{
    auto userMob = CreateNtnMobility(userPos, cfg);
    auto satMob = CreateNtnMobility(satPos, cfg);
    return -GetNtnLosPropagationLossModel(cfg.centerFreqHz)->CalcRxPower(0.0, userMob, satMob);
}

// ---------------------------------------------------------------------------
// ComputeUPABeamGainPower — exact UPA model matching Python channel.py
// ---------------------------------------------------------------------------

double
ComputeUPABeamGainPower(const Vec3& satPos,
                         const Vec3& userPos,
                         const Vec3& beamCentre,
                         const SimConfig& cfg)
{
    // Precompute rotation parameters for this satellite position
    const ArrayTransform at = BuildArrayTransform(satPos, cfg);

    // Spatial frequencies for user and beam centre
    double PhiX_u, PhiY_u;
    double PhiX_b, PhiY_b;
    GetSpatialFreqs(userPos,    at, PhiX_u, PhiY_u);
    GetSpatialFreqs(beamCentre, at, PhiX_b, PhiY_b);

    // ΔΦ = Φ_user − Φ_beam
    // Python precoder negates beam angles → ΔΦ = Φ_x_user − Φ_x_beam_raw.
    // Both GetSpatialFreqs() calls use the same (positive) convention here,
    // so ΔΦ correctly represents the steering offset.
    const double dPhiX = PhiX_u - PhiX_b;
    const double dPhiY = PhiY_u - PhiY_b;

    // Array factor squared (Dirichlet kernel) per dimension
    const double AF_x2 = DirichletKernel2(cfg.nAntennaX, dPhiX);
    const double AF_y2 = DirichletKernel2(cfg.nAntennaY, dPhiY);

    // Normalised beam gain power: |beam_gain[u,j]|² = AF_x² * AF_y² / (Nx²·Ny²·Nbeams)
    // At beam centre (ΔΦ=0): = Nx²*Ny² / (Nx²·Ny²·Nbeams) = 1/Nbeams  ← matches Python.
    const double Nx2   = static_cast<double>(cfg.nAntennaX) * cfg.nAntennaX;
    const double Ny2   = static_cast<double>(cfg.nAntennaY) * cfg.nAntennaY;
    const double norm  = Nx2 * Ny2 * static_cast<double>(cfg.nBeams);

    return AF_x2 * AF_y2 / norm;
}

// ---------------------------------------------------------------------------
// SampleRicianAmplitude
// ---------------------------------------------------------------------------

double
SampleRicianAmplitude(double K, std::mt19937& rng)
{
    // Mirrors Python's get_Rician_fading_coefficient():
    //   mu = sqrt(K / (2*(K+1))),  sigma = sqrt(1 / (2*(K+1)))
    //   g  = N(mu, sigma) + j*N(mu, sigma)
    // Returns |g| (amplitude, not power).  E[|g|²] = 1.
    //
    // Phase terms (Doppler shift, propagation delay) are omitted because
    // they cancel when computing received power |·|².
    const double mu    = std::sqrt(K / (2.0 * (K + 1.0)));
    const double sigma = std::sqrt(1.0 / (2.0 * (K + 1.0)));
    std::normal_distribution<double> norm(mu, sigma);
    const double gReal = norm(rng);
    const double gImag = norm(rng);
    return std::sqrt(gReal * gReal + gImag * gImag);
}

// ---------------------------------------------------------------------------
// ComputeFrameResults — mirrors simulation.py inner loop
// ---------------------------------------------------------------------------

std::vector<UserLinkResult>
ComputeFrameResults(const Vec3&                    satPos,
                    const std::vector<Vec3>&        userPos,
                    const std::array<Vec3, 19>&     beamCenters,
                    const SimConfig&                cfg,
                    std::mt19937&                   rng,
                    bool                            withFading)
{
    const int    nUser    = static_cast<int>(userPos.size());
    const int    nBeams   = cfg.nBeams;        // 19
    const double noisePow = cfg.GetNoisePower();
    const double txPow    = cfg.transmitPowerW;
    const double gainLin  = FromDb(cfg.antennaGainDb);  // total array gain (linear)

    // Index of the paper's "centre beam gain" metric (beam 9, 0-indexed)
    const int centreBeamIdx = 9;

    // -----------------------------------------------------------------------
    // Precompute array transform and beam spatial frequencies once per frame.
    // Python computes get_angles_to_satellite() once per satellite position.
    // -----------------------------------------------------------------------
    const ArrayTransform at = BuildArrayTransform(satPos, cfg);

    // Spatial frequencies for each beam centre (Φ_x, Φ_y per beam)
    std::vector<double> beamPhiX(nBeams), beamPhiY(nBeams);
    for (int j = 0; j < nBeams; ++j)
    {
        GetSpatialFreqs(beamCenters[j], at, beamPhiX[j], beamPhiY[j]);
    }

    const double Nx2  = static_cast<double>(cfg.nAntennaX) * cfg.nAntennaX;
    const double Ny2  = static_cast<double>(cfg.nAntennaY) * cfg.nAntennaY;
    const double norm = Nx2 * Ny2 * static_cast<double>(nBeams);

    // -----------------------------------------------------------------------
    // Step 1 — path loss and UPA beam gain power for every (user, beam) pair.
    //
    // Mirrors Python:
    //   macro_loss² = txPow * gainLin / lossLin
    //   macroPow[u][j] = macro_loss² * |beam_gain[u,j]|²
    // -----------------------------------------------------------------------
    std::vector<double>              pathLossDb(nUser);
    std::vector<std::vector<double>> macroPow(nUser, std::vector<double>(nBeams));

    for (int u = 0; u < nUser; ++u)
    {
        pathLossDb[u] = ComputePathLoss_dB(userPos[u], satPos, cfg);

        const double lossLin     = FromDb(pathLossDb[u]);
        const double macroScalar = txPow * gainLin / lossLin; // |macro_loss|²

        // Spatial frequencies for this user
        double PhiX_u, PhiY_u;
        GetSpatialFreqs(userPos[u], at, PhiX_u, PhiY_u);

        for (int j = 0; j < nBeams; ++j)
        {
            const double dPhiX = PhiX_u - beamPhiX[j];
            const double dPhiY = PhiY_u - beamPhiY[j];

            const double AF_x2       = DirichletKernel2(cfg.nAntennaX, dPhiX);
            const double AF_y2       = DirichletKernel2(cfg.nAntennaY, dPhiY);
            const double beamGainPow = AF_x2 * AF_y2 / norm; // |beam_gain[u,j]|²

            macroPow[u][j] = macroScalar * beamGainPow;
        }
    }

    // -----------------------------------------------------------------------
    // Step 2 — beam association: assign each user to the highest macro power.
    //          Mirrors: beam_index = argmax(|macro_channel|²)
    // -----------------------------------------------------------------------
    std::vector<int> beamIdx(nUser);
    for (int u = 0; u < nUser; ++u)
    {
        beamIdx[u] = static_cast<int>(
            std::max_element(macroPow[u].begin(), macroPow[u].end()) -
            macroPow[u].begin());
    }

    // -----------------------------------------------------------------------
    // Step 3 — Rician fading amplitude per user (|h|²).
    //          Mirrors get_Rician_fading_coefficient(); phase terms omitted
    //          (they cancel in power |·|²).
    // -----------------------------------------------------------------------
    std::vector<double> ricianAmp2(nUser, 1.0); // default: no fading (= 1)
    if (withFading)
    {
        for (int u = 0; u < nUser; ++u)
        {
            const double amp = SampleRicianAmplitude(cfg.ricianK, rng);
            ricianAmp2[u]    = amp * amp;
        }
    }

    // -----------------------------------------------------------------------
    // Step 4 — SINR, SNR, and beam gain metrics.
    //
    // Mirrors Python's calculate_simulation_result():
    //   desired_power    = rec_power[u, beam_idx[u]]
    //   interference     = sum(rec_power[u, :]) - desired_power
    //   sinr = desired / (interference + noise)
    //   snr  = desired / noise
    //   center_beam_gain_dB = to_dB(|beam_gain[u, 9]|²) + antenna_gain_dB
    // -----------------------------------------------------------------------
    std::vector<UserLinkResult> results(nUser);

    for (int u = 0; u < nUser; ++u)
    {
        double totalPow = 0.0;
        for (int j = 0; j < nBeams; ++j)
        {
            totalPow += macroPow[u][j] * ricianAmp2[u];
        }

        const double desiredPow = macroPow[u][beamIdx[u]] * ricianAmp2[u];
        const double intPow     = totalPow - desiredPow;

        // SINR and SNR
        const double sinrLin = desiredPow / (intPow + noisePow);
        const double snrLin  = desiredPow / noisePow;

        // Serving beam gain metric: to_dB(|beam_gain[u, beamIdx]|²) + G_max
        // (Mirrors Python for the serving beam; Python only reports beam 9.)
        const double macroScalar  = txPow * gainLin / FromDb(pathLossDb[u]);
        const double bgPowServing = (macroScalar > 0.0)
                                    ? macroPow[u][beamIdx[u]] / macroScalar
                                    : 0.0;
        const double servingBgDb  = ToDb(bgPowServing) + cfg.antennaGainDb;

        // Centre beam gain (beam 9) — paper figure metric, same formula as Python:
        //   center_beam_gain_dB = to_dB(|beam_gain[u, 9]|²) + antenna_gain_dB
        double PhiX_u, PhiY_u;
        GetSpatialFreqs(userPos[u], at, PhiX_u, PhiY_u);
        const double dPx9  = PhiX_u - beamPhiX[centreBeamIdx];
        const double dPy9  = PhiY_u - beamPhiY[centreBeamIdx];
        const double bgPow9 = DirichletKernel2(cfg.nAntennaX, dPx9) *
                              DirichletKernel2(cfg.nAntennaY, dPy9) / norm;
        const double centBgDb = ToDb(bgPow9) + cfg.antennaGainDb;

        results[u] = UserLinkResult{
            u,
            beamIdx[u],
            pathLossDb[u],
            servingBgDb,
            ToDb(sinrLin),
            ToDb(snrLin),
            centBgDb
        };
    }

    return results;
}

} // namespace ns3
