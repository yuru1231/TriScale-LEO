/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-channel.cc
 *
 * Channel model implementation for Multi-Beam LEO simulation.
 * Copied from phase2/code/ for use by the constellation scanner (orbit-sgp4).
 * See sat-multi-beam-channel.h for full API documentation.
 *
 * Beam gain: exact UPA Dirichlet-kernel formula — matches Python channel.py.
 *   Replaces the previous Gaussian approximation.
 */

#include "sat-multi-beam-channel.h"

#include "ns3/boolean.h"
#include "ns3/channel-condition-model.h"
#include "ns3/double.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/satellite-utils.h"
#include "ns3/three-gpp-propagation-loss-model.h"

#include <algorithm>
#include <cmath>

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

inline double
DistanceM(const Vec3& a, const Vec3& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
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
// z-axis pre-rotation (cosZ, sinZ) maps the satellite's actual horizontal
// direction to the +x axis before applying the y-axis rotation.
// This fixes the arc-model approximation where satPos.y = 0 was assumed.
// ---------------------------------------------------------------------------

struct ArrayTransform
{
    double satX{0.0};
    double satY{0.0};
    double satZ{0.0};
    double cosR{1.0};
    double sinR{0.0};
    double cosZ{1.0};
    double sinZ{0.0};
};

/**
 * BuildArrayTransform — compute rotation parameters from satPos.
 *
 * Steps:
 *   0. z-pre-rotation: cosZ = E_s / satHoriz, sinZ = N_s / satHoriz
 *   1. Shift satellite to array corner.
 *   2. tan_vec = [1, −satX / (satZ + r_earth)]
 *   3. rotation_angle = arccos(tan_vec[0] / |tan_vec|) × sign(satX) + π
 *   4. T = [[cosR, 0, −sinR], [0, 1, 0], [sinR, 0, cosR]]
 */
inline ArrayTransform
BuildArrayTransform(const Vec3& satPos, const SimConfig& cfg)
{
    ArrayTransform at;
    const double spacing = cfg.GetAntennaSpacing();

    const double satHoriz = std::sqrt(satPos.x * satPos.x + satPos.y * satPos.y);
    if (satHoriz > 1.0)
    {
        at.cosZ = satPos.x / satHoriz;
        at.sinZ = satPos.y / satHoriz;
    }
    else
    {
        at.cosZ = 1.0;
        at.sinZ = 0.0;
    }

    at.satX = satHoriz - static_cast<double>(cfg.nAntennaX * cfg.nBeamsX) / 2.0 * spacing;
    at.satY = 0.0      - static_cast<double>(cfg.nAntennaY * cfg.nBeamsY) / 2.0 * spacing;
    at.satZ = satPos.z;

    const double tanVecZ   = -at.satX / (at.satZ + cfg.rEarthM);
    const double tanVecNorm = std::sqrt(1.0 + tanVecZ * tanVecZ);
    const double cosAngle  = std::max(-1.0, std::min(1.0, 1.0 / tanVecNorm));
    const double signX     = (at.satX >= 0.0) ? 1.0 : -1.0;
    const double rotAngle  = std::acos(cosAngle) * signX + M_PI;

    at.cosR = std::cos(rotAngle);
    at.sinR = std::sin(rotAngle);

    return at;
}

/**
 * GetSpatialFreqs — compute UPA spatial frequencies (Φ_x, Φ_y) for one position.
 *
 * Applies z-pre-rotation to pos before the y-axis rotation.
 */
inline void
GetSpatialFreqs(const Vec3& pos, const ArrayTransform& at,
                double& PhiX, double& PhiY)
{
    const double posXr = at.cosZ * pos.x + at.sinZ * pos.y;
    const double posYr = -at.sinZ * pos.x + at.cosZ * pos.y;
    const double posZr = pos.z;

    const double relX = posXr - at.satX;
    const double relY = posYr - at.satY;
    const double relZ = posZr - at.satZ;

    const double tX = at.cosR * relX - at.sinR * relZ;
    const double tY = relY;
    const double tZ = at.sinR * relX + at.cosR * relZ;

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
 * At ΔΦ = 0: returns N² (L'Hôpital limit).
 */
inline double
DirichletKernel2(int N, double dPhi)
{
    const double halfArg = M_PI * dPhi / 2.0;
    const double denom   = std::sin(halfArg);
    if (std::abs(denom) < 1e-10)
    {
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
    NS_ABORT_MSG_IF(distanceM <= 0.0, "Distance must be positive");
    return 20.0 * std::log10(4.0 * M_PI * distanceM * freqHz / 299792458.0);
}

// ---------------------------------------------------------------------------
// ComputeAtmosphericLoss_dB
// ---------------------------------------------------------------------------

double
ComputeAtmosphericLoss_dB(double elevationDeg, double freqHz)
{
    const double txPowerDbm = 0.0;
    const double distM      = 600.0e3;
    const double elevRad    = std::max(1.0, elevationDeg) * M_PI / 180.0;

    const Vec3 userPos{0.0, 0.0, 0.0};
    const Vec3 satPos{distM * std::cos(elevRad), 0.0, distM * std::sin(elevRad)};

    SimConfig cfg;
    cfg.centerFreqHz = freqHz;

    auto userMob = CreateNtnMobility(userPos, cfg);
    auto satMob  = CreateNtnMobility(satPos,  cfg);

    const double ntnLoss = -GetNtnLosPropagationLossModel(freqHz)
                                ->CalcRxPower(txPowerDbm, userMob, satMob);
    const double fspl    = ComputeFSPL_dB(userMob->GetDistanceFrom(satMob), freqHz);
    return std::max(0.0, ntnLoss - fspl);
}

// ---------------------------------------------------------------------------
// ComputePathLoss_dB
// ---------------------------------------------------------------------------

double
ComputePathLoss_dB(const Vec3& userPos, const Vec3& satPos, const SimConfig& cfg)
{
    // Elevation is computed per-cell from the vector (satPos - userPos).
    // userPos is the beam-centre ENU, so each of the 25 cells gets its own
    // elevation angle — outer cells at low satellite elevation differ by up to
    // ~0.83° from the ROI-centre elevation, which maps to ~3.7 dB at the
    // 3 dB hard-cell threshold.  Per-cell computation is mandatory for
    // correct hard-cell identification (D10).
    const Vec3 delta{satPos.x - userPos.x, satPos.y - userPos.y, satPos.z - userPos.z};
    const double distanceM    = DistanceM(userPos, satPos);
    const double horizM       = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    const double elevationDeg = std::atan2(delta.z, horizM) * 180.0 / M_PI;

    return ComputeFSPL_dB(distanceM, cfg.centerFreqHz) +
           ComputeAtmosphericLoss_dB(elevationDeg, cfg.centerFreqHz);
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
    const ArrayTransform at = BuildArrayTransform(satPos, cfg);

    double PhiX_u, PhiY_u;
    double PhiX_b, PhiY_b;
    GetSpatialFreqs(userPos,    at, PhiX_u, PhiY_u);
    GetSpatialFreqs(beamCentre, at, PhiX_b, PhiY_b);

    const double dPhiX = PhiX_u - PhiX_b;
    const double dPhiY = PhiY_u - PhiY_b;

    const double AF_x2 = DirichletKernel2(cfg.nAntennaX, dPhiX);
    const double AF_y2 = DirichletKernel2(cfg.nAntennaY, dPhiY);

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
                    const std::array<Vec3, 25>&     beamCenters,
                    const SimConfig&                cfg,
                    std::mt19937&                   rng,
                    bool                            withFading)
{
    const int    nUser    = static_cast<int>(userPos.size());
    const int    nBeams   = cfg.nBeams;        // 25
    const double noisePow = cfg.GetNoisePower();
    const double txPow    = cfg.transmitPowerW;
    const double gainLin  = FromDb(cfg.antennaGainDb);

    // Centre beam: row=0, col=0 → index (0+2)*5+(0+2) = 12
    const int centreBeamIdx = 12;

    // Precompute array transform and beam spatial frequencies once per frame
    const ArrayTransform at = BuildArrayTransform(satPos, cfg);

    std::vector<double> beamPhiX(nBeams), beamPhiY(nBeams);
    for (int j = 0; j < nBeams; ++j)
    {
        GetSpatialFreqs(beamCenters[j], at, beamPhiX[j], beamPhiY[j]);
    }

    const double Nx2  = static_cast<double>(cfg.nAntennaX) * cfg.nAntennaX;
    const double Ny2  = static_cast<double>(cfg.nAntennaY) * cfg.nAntennaY;
    const double norm = Nx2 * Ny2 * static_cast<double>(nBeams);

    // Step 1 — path loss and UPA beam gain power for every (user, beam) pair
    std::vector<double>              pathLossDb(nUser);
    std::vector<std::vector<double>> macroPow(nUser, std::vector<double>(nBeams));

    for (int u = 0; u < nUser; ++u)
    {
        pathLossDb[u] = ComputePathLoss_dB(userPos[u], satPos, cfg);

        const double lossLin     = FromDb(pathLossDb[u]);
        const double macroScalar = txPow * gainLin / lossLin;

        double PhiX_u, PhiY_u;
        GetSpatialFreqs(userPos[u], at, PhiX_u, PhiY_u);

        for (int j = 0; j < nBeams; ++j)
        {
            const double dPhiX = PhiX_u - beamPhiX[j];
            const double dPhiY = PhiY_u - beamPhiY[j];

            const double AF_x2       = DirichletKernel2(cfg.nAntennaX, dPhiX);
            const double AF_y2       = DirichletKernel2(cfg.nAntennaY, dPhiY);
            const double beamGainPow = AF_x2 * AF_y2 / norm;

            macroPow[u][j] = macroScalar * beamGainPow;
        }
    }

    // Step 2 — beam association: assign each user to the highest macro power
    std::vector<int> beamIdx(nUser);
    for (int u = 0; u < nUser; ++u)
    {
        beamIdx[u] = static_cast<int>(
            std::max_element(macroPow[u].begin(), macroPow[u].end()) -
            macroPow[u].begin());
    }

    // Step 3 — Rician fading amplitude per user
    std::vector<double> ricianAmp2(nUser, 1.0);
    if (withFading)
    {
        for (int u = 0; u < nUser; ++u)
        {
            const double amp = SampleRicianAmplitude(cfg.ricianK, rng);
            ricianAmp2[u]    = amp * amp;
        }
    }

    // Step 4 — SINR, SNR, and beam gain metrics
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

        const double sinrLin = desiredPow / (intPow + noisePow);
        const double snrLin  = desiredPow / noisePow;

        const double macroScalar  = txPow * gainLin / FromDb(pathLossDb[u]);
        const double bgPowServing = (macroScalar > 0.0)
                                    ? macroPow[u][beamIdx[u]] / macroScalar
                                    : 0.0;
        const double servingBgDb  = ToDb(bgPowServing) + cfg.antennaGainDb;

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
