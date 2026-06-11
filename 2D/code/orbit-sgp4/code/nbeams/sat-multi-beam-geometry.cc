/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-geometry.cc
 *
 * Implementation of geometry functions for Multi-Beam LEO simulation.
 * See sat-multi-beam-geometry.h for full API documentation.
 *
 * [orbit-sgp4 version]
 * GetSatelliteArcPositions, GetElevationAngleDeg (2-D arc version), and
 * GetSatellitePositionAtTime have been removed.  Historical versions that
 * retain those functions are in 2D/phase2/code/ and 2D/code/phase1-ns3/.
 */

#include "sat-multi-beam-geometry.h"

#include "sgp4unit.h"   // gstime() — GAST for ECI → ECEF rotation

#include <cassert>
#include <cmath>
#include <random>

namespace ns3
{

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace
{

/** pol2cart3D — spherical (r, phi, theta) → Cartesian (x, y, z). */
inline void
Pol2Cart3D(double r, double phi, double theta,
           double& x, double& y, double& z)
{
    x = r * std::sin(theta) * std::cos(phi);
    y = r * std::sin(theta) * std::sin(phi);
    z = r * std::cos(theta);
}

/** cart2pol3D — Cartesian → (r, phi, theta) physics convention. */
inline void
Cart2Pol3D(double x, double y, double z,
           double& r, double& phi, double& theta)
{
    r = std::sqrt(x * x + y * y + z * z);
    if (x == 0.0 && y == 0.0)
    {
        phi = 0.0;
    }
    else
    {
        double sign = (y >= 0.0) ? 1.0 : -1.0;
        phi = sign * std::acos(x / std::sqrt(x * x + y * y));
    }
    theta = (r > 0.0) ? std::acos(z / r) : 0.0;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// GetRandomUserPositions — mirrors networkGeometry.get_user_position()
// ---------------------------------------------------------------------------

std::vector<Vec3>
GetRandomUserPositions(const SimConfig& cfg, uint32_t seed)
{
    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.GetEffectiveFootprintM();
    const int    nUser = cfg.nUser;

    // Uniform sphere-cap sampling
    // theta = arccos(uniform(cos(r_foot/r_earth), 1))
    // phi   = uniform(0, 2*pi)
    const double cosMax = std::cos(rFoot / rE); // cos(theta_lim)

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uniCos(cosMax, 1.0);
    std::uniform_real_distribution<double> uniPhi(0.0, 2.0 * M_PI);

    std::vector<Vec3> users;
    users.reserve(nUser);

    for (int i = 0; i < nUser; ++i)
    {
        const double theta = std::acos(uniCos(rng));
        const double phi   = uniPhi(rng);

        double x, y, z;
        Pol2Cart3D(rE, phi, theta, x, y, z);
        // re-centre z around footprint (subtract r_earth)
        users.push_back({x, y, z - rE});
    }
    return users;
}

// ---------------------------------------------------------------------------
// GetGridUserPositions — mirrors networkGeometry.get_grid_positions()
// ---------------------------------------------------------------------------

std::vector<Vec3>
GetGridUserPositions(double spacingM, const SimConfig& cfg)
{
    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.GetEffectiveFootprintM();

    const int nTier = static_cast<int>(std::ceil(rFoot / spacingM + 1.0));

    // Pre-allocate: sum_{k=0}^{nTier} k*6 + nTier = nTier*(nTier+1)*3 + nTier
    const int nTotal = nTier * (nTier + 1) * 3 + nTier;
    std::vector<Vec3> users;
    users.reserve(nTotal);

    // Centre point (theta=0)
    users.push_back({0.0, 0.0, 0.0});

    // Concentric rings
    for (int tier = 1; tier <= nTier; ++tier)
    {
        const int    nPts  = tier * 6;
        const double theta = (rFoot / nTier / rE) * static_cast<double>(tier);
        for (int k = 0; k < nPts; ++k)
        {
            const double phi = 2.0 * M_PI * k / nPts;
            double x, y, z;
            Pol2Cart3D(rE, phi, theta, x, y, z);
            users.push_back({x, y, z - rE});
        }
    }
    return users;
}

// ---------------------------------------------------------------------------
// GetHexBeamCenters — mirrors networkGeometry.hex_grid_centers_two_rings()
// ---------------------------------------------------------------------------

std::array<Vec3, 19>
GetHexBeamCenters(const SimConfig& cfg)
{
    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.GetEffectiveFootprintM();

    // Cell angular radius unit (same as Python: 2*r_footprint/10/r_earth)
    const double cellRad = 2.0 * rFoot / 10.0 / rE;

    // theta_side = 1/cos(pi/6) = 2/sqrt(3)
    const double thetaSide = 1.0 / std::cos(M_PI / 6.0);

    // Azimuth phi_hex = pi/2 + pi/6 * factor[i]
    // 19 cells in row order: top(3), second(4), centre(5), fourth(4), bottom(3)
    static const int phiFactor[19] = {
        4, 3, 2,           // top row
        5, 4, 2, 1,        // second row
        6, 6, 0, 0, 0,     // centre row
        7, 8, 10, 11,      // fourth row
        8, 9, 10           // bottom row
    };

    // Elevation theta_hex = cellRad * factor[i]
    const double ts = thetaSide;
    const double thetaFactor[19] = {
        4.0,    3.0*ts, 4.0,
        3.0*ts, 2.0,    2.0,    3.0*ts,
        4.0,    2.0,    0.0,    2.0,    4.0,
        3.0*ts, 2.0,    2.0,    3.0*ts,
        4.0,    3.0*ts, 4.0
    };

    std::array<Vec3, 19> centers;
    for (int i = 0; i < 19; ++i)
    {
        const double phi   = M_PI / 2.0 + M_PI / 6.0 * phiFactor[i];
        const double theta = cellRad * thetaFactor[i];

        double x, y, z;
        Pol2Cart3D(rE, phi, theta, x, y, z);
        // re-centre z around footprint
        centers[i] = {x, y, z - rE};
    }
    return centers;
}

// ---------------------------------------------------------------------------
// GetNadirFromTime — nadir (sub-satellite point) at simulation time T
// ---------------------------------------------------------------------------

void
GetNadirFromTime(double timeS, const SimConfig& cfg, double& latDeg, double& lonDeg)
{
    // Step 1 — compute arc angle eps at time T.
    //
    // The arc runs from eps_zero to pi - eps_zero.
    // Frame index = timeS / tFrameS; angle step per frame = GetFrameAngleStepRad().
    const double eps0 = cfg.GetMinElevRad();
    const double dEps = cfg.GetFrameAngleStepRad();
    const double eps  = eps0 + (timeS / cfg.tFrameS) * dEps;

    // Step 2 — nadir = Earth-surface point directly below satellite.
    //
    // In the old local frame (origin = pass centre at 90° elevation):
    //   satellite_global = { R * cos(eps), 0, R * sin(eps) }
    //   nadir_global     = { rEarth * cos(eps), 0, rEarth * sin(eps) }
    //   nadir_local.z    = rEarth * sin(eps) - rEarth  (re-centred)
    const double rE = cfg.rEarthM;
    Vec3 nadirLocal;
    nadirLocal.x = rE * std::cos(eps);
    nadirLocal.y = 0.0;
    nadirLocal.z = rE * std::sin(eps) - rE;   // 0 at eps=pi/2 (pass centre)

    // Step 3 — convert local Cartesian to geographic lat/lon.
    GetLatLon(nadirLocal, cfg, latDeg, lonDeg);
}

// ---------------------------------------------------------------------------
// GetLatLon — mirrors utils.get_positions_in_lat_long_coordinates()
// ---------------------------------------------------------------------------

void
GetLatLon(const Vec3& pos, const SimConfig& cfg, double& latDeg, double& lonDeg)
{
    const double rE     = cfg.rEarthM;
    const double latC   = cfg.latitudeCenterDeg  * M_PI / 180.0;
    const double lonC   = cfg.longitudeCenterDeg * M_PI / 180.0;

    // Globe-centre Cartesian of the footprint centre
    double xC, yC, zC;
    // Physics convention: theta=colatitude, phi=longitude
    // latitude_center is latitude not colatitude → theta_colat = pi/2 - lat
    const double colatC = M_PI / 2.0 - latC;
    Pol2Cart3D(rE, lonC, colatC, xC, yC, zC);

    // Global Cartesian of user = footprint_centre + local_offset
    const double xG = xC + pos.x;
    const double yG = yC + pos.y;
    const double zG = zC + pos.z;

    // Convert back to spherical (physics convention)
    double r, phi, theta;
    Cart2Pol3D(xG, yG, zG, r, phi, theta);

    // phi = longitude, theta = colatitude
    latDeg = (M_PI / 2.0 - theta) * 180.0 / M_PI;
    lonDeg = phi * 180.0 / M_PI;
}

// ---------------------------------------------------------------------------
// GetElevationAngleDeg_3D — elevation angle for arbitrary ENU Vec3
// ---------------------------------------------------------------------------

double
GetElevationAngleDeg_3D(const Vec3& pos)
{
    // Horizontal distance in the ENU plane (East-North)
    const double horiz = std::sqrt(pos.x * pos.x + pos.y * pos.y);
    if (horiz < 1e-6)
    {
        return 90.0; // directly overhead
    }
    return std::atan2(pos.z, horiz) * 180.0 / M_PI;
}

// ---------------------------------------------------------------------------
// EcefOffsetToEnu — rotate ECEF-offset Vec3 to local ENU at observer
// ---------------------------------------------------------------------------

Vec3
EcefOffsetToEnu(const Vec3& ecefOffset, double obsLatDeg, double obsLonDeg)
{
    const double d2r    = M_PI / 180.0;
    const double latRad = obsLatDeg * d2r;
    const double lonRad = obsLonDeg * d2r;

    const double sinLat = std::sin(latRad);
    const double cosLat = std::cos(latRad);
    const double sinLon = std::sin(lonRad);
    const double cosLon = std::cos(lonRad);

    // Standard ECEF-offset → ENU rotation
    const double E = -sinLon * ecefOffset.x + cosLon * ecefOffset.y;
    const double N = -sinLat * cosLon * ecefOffset.x
                     - sinLat * sinLon * ecefOffset.y
                     + cosLat * ecefOffset.z;
    const double U =  cosLat * cosLon * ecefOffset.x
                     + cosLat * sinLon * ecefOffset.y
                     + sinLat * ecefOffset.z;

    return Vec3{E, N, U};
}

// ---------------------------------------------------------------------------
// EciToEcef — rotate ECI/TEME position to ECEF via GAST
// ---------------------------------------------------------------------------

Vec3
EciToEcef(const Vec3& eciKm, double jdUT1)
{
    // gstime() returns Greenwich Apparent Sidereal Time (GAST) in radians.
    // Rotating ECI by -GAST around the z-axis gives ECEF.
    const double gast   = gstime(jdUT1);
    const double cosG   = std::cos(gast);
    const double sinG   = std::sin(gast);

    return Vec3{
         eciKm.x * cosG + eciKm.y * sinG,
        -eciKm.x * sinG + eciKm.y * cosG,
         eciKm.z
    };
}

// ---------------------------------------------------------------------------
// EcefToGeodetic — spherical Earth ECEF → lat/lon/alt
// ---------------------------------------------------------------------------

void
EcefToGeodetic(const Vec3& ecefM, double rEarthM,
               double& latDeg, double& lonDeg, double& altM)
{
    const double r = std::sqrt(ecefM.x * ecefM.x +
                               ecefM.y * ecefM.y +
                               ecefM.z * ecefM.z);

    // Avoid division by zero for r ≈ 0 (should never occur for LEO sats)
    if (r < 1.0)
    {
        latDeg = 0.0;
        lonDeg = 0.0;
        altM   = -rEarthM;
        return;
    }

    latDeg = std::asin(ecefM.z / r) * 180.0 / M_PI;
    lonDeg = std::atan2(ecefM.y, ecefM.x) * 180.0 / M_PI;
    altM   = r - rEarthM;
}

// ---------------------------------------------------------------------------
// GetBeamCentersFromSatPos — elliptical off-nadir beam footprint correction
// Ported from 2D/phase1/sgp4/code/sat-multi-beam-geometry.cc
// ---------------------------------------------------------------------------

std::array<Vec3, 19>
GetBeamCentersFromSatPos(const Vec3& satEnu, const SimConfig& cfg)
{
    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.GetEffectiveFootprintM();

    // Step 1: Nadir direction — from satellite toward Earth centre.
    //   Earth centre in ROI ENU frame is at (0, 0, -rE).
    const double dX  = -satEnu.x;
    const double dY  = -satEnu.y;
    const double dZ  = -rE - satEnu.z;
    const double len = std::sqrt(dX * dX + dY * dY + dZ * dZ);
    const Vec3 nadirUnit{dX / len, dY / len, dZ / len};

    // Step 2: Satellite beam-frame basis in ENU (Gram-Schmidt).
    //   xSat ← East direction projected perpendicular to nadirUnit.
    //   ySat ← nadirUnit × xSat  (right-hand, roughly North-like).
    const double dotE = nadirUnit.x;
    Vec3 xSat{1.0 - dotE * nadirUnit.x,
                   - dotE * nadirUnit.y,
                   - dotE * nadirUnit.z};
    const double xLen = std::sqrt(xSat.x * xSat.x +
                                   xSat.y * xSat.y +
                                   xSat.z * xSat.z);
    xSat = {xSat.x / xLen, xSat.y / xLen, xSat.z / xLen};

    const Vec3 ySat{
        nadirUnit.y * xSat.z - nadirUnit.z * xSat.y,
        nadirUnit.z * xSat.x - nadirUnit.x * xSat.z,
        nadirUnit.x * xSat.y - nadirUnit.y * xSat.x
    };

    // Steps 3 & 4: Hex angular layout → beam directions → ray-ground intersection.
    const double cellRad   = 2.0 * rFoot / 10.0 / rE;
    const double thetaSide = 1.0 / std::cos(M_PI / 6.0);   // 2/√3

    static const int phiFactor[19] = {
        4, 3, 2,
        5, 4, 2, 1,
        6, 6, 0, 0, 0,
        7, 8, 10, 11,
        8, 9, 10
    };
    const double ts = thetaSide;
    const double thetaFactor[19] = {
        4.0,    3.0*ts, 4.0,
        3.0*ts, 2.0,    2.0,    3.0*ts,
        4.0,    2.0,    0.0,    2.0,    4.0,
        3.0*ts, 2.0,    2.0,    3.0*ts,
        4.0,    3.0*ts, 4.0
    };

    std::array<Vec3, 19> centers;
    for (int i = 0; i < 19; ++i)
    {
        const double phi   = M_PI / 2.0 + M_PI / 6.0 * static_cast<double>(phiFactor[i]);
        const double theta = cellRad * thetaFactor[i];

        const double bx = std::sin(theta) * std::cos(phi);
        const double by = std::sin(theta) * std::sin(phi);
        const double bz = std::cos(theta);

        const Vec3 d{
            bx * xSat.x + by * ySat.x + bz * nadirUnit.x,
            bx * xSat.y + by * ySat.y + bz * nadirUnit.y,
            bx * xSat.z + by * ySat.z + bz * nadirUnit.z
        };

        const double t = -satEnu.z / d.z;
        centers[i] = Vec3{satEnu.x + t * d.x,
                           satEnu.y + t * d.y,
                           0.0};
    }
    return centers;
}

// ---------------------------------------------------------------------------
// GetEllipticBeamCenters — nBeamsX×nBeamsY elliptic inscribed-rectangle grid
// ---------------------------------------------------------------------------

std::vector<Vec3>
GetEllipticBeamCenters(const Vec3& satEnu,
                       const Vec3& prevSatEnu,
                       const SimConfig& cfg)
{
    // Elevation angle at current satellite position (no floor — callers must assert ≥ minElevDeg)
    const double elevDeg = GetElevationAngleDeg_3D(satEnu);
    const double elevRad = elevDeg * M_PI / 180.0;
    const double sinE    = std::sin(elevRad);

    // Nadir footprint radius (derived from UPA geometry via GetEffectiveFootprintM)
    const double r = cfg.GetEffectiveFootprintM();

    // Elliptic footprint semi-axes — both elevation-dependent (D2):
    //   b(ε) = r / sin(ε)       cross-track: slant-range scaling
    //   a(ε) = r / sin²(ε)      along-track: slant-range × tilt factor
    const double b = r / sinE;
    const double a = r / (sinE * sinE);

    // Max inscribed rectangle half-widths
    const double wAlong = a / std::sqrt(2.0);
    const double hCross = b / std::sqrt(2.0);

    // Cell pitch — uniform nBeamsX×nBeamsY subdivision of the inscribed rectangle.
    // Set to 0 for single-beam axes (along-track or cross-track = 1) so that all
    // offsets collapse to zero regardless of direction.
    const double cellA = (cfg.nBeamsX > 1) ? 2.0 * wAlong / cfg.nBeamsX : 0.0;
    const double cellC = (cfg.nBeamsY > 1) ? 2.0 * hCross / cfg.nBeamsY : 0.0;

    // Along-track unit vector from satellite motion in ENU x-y plane.
    // Only required when nBeamsX > 1 (multi-column grid needs direction).
    // For single-column grids (nBeamsX == 1) all col offsets are zero — direction irrelevant.
    double dE = 1.0;
    double dN = 0.0;
    if (cfg.nBeamsX > 1)
    {
        double ddE = satEnu.x - prevSatEnu.x;
        double ddN = satEnu.y - prevSatEnu.y;
        const double movLen = std::sqrt(ddE * ddE + ddN * ddN);
        assert(movLen >= 1.0);  // fires if Run() did not pre-compute prevSatEnu
        dE = ddE / movLen;
        dN = ddN / movLen;
    }

    // Cross-track unit vector (90° CCW from along-track in ENU x-y)
    const double cE = -dN;
    const double cN =  dE;

    // nBeamsX×nBeamsY grid in row-major order (row=cross-track, col=along-track)
    // row ∈ {-halfY .. +halfY}, col ∈ {-halfX .. +halfX}
    // Centre beam: index (halfY)*nBeamsX + halfX
    const int halfX = cfg.nBeamsX / 2;
    const int halfY = cfg.nBeamsY / 2;

    std::vector<Vec3> centers;
    centers.reserve(cfg.nBeams);
    for (int row = -halfY; row <= halfY; ++row)
    {
        for (int col = -halfX; col <= halfX; ++col)
        {
            const double ao = static_cast<double>(col) * cellA;
            const double co = static_cast<double>(row) * cellC;
            centers.push_back(Vec3{ao * dE + co * cE,
                                   ao * dN + co * cN,
                                   0.0});
        }
    }
    return centers;
}

} // namespace ns3
