/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-geometry.cc
 *
 * Implementation of geometry free functions for Multi-Beam LEO simulation.
 * See sat-multi-beam-geometry.h for full API documentation.
 *
 * [orbit-sgp4 version]
 * GetSatelliteArcPositions, GetElevationAngleDeg (2-D arc), and
 * GetSatellitePositionAtTime have been removed.  Historical versions
 * that retain those functions are in 2D/phase2/code/ and 2D/code/phase1-ns3/.
 *
 * Logging component: SatMultiBeamGeometry
 *   Enable with: LogComponentEnable("SatMultiBeamGeometry", LOG_LEVEL_DEBUG)
 */

#include "sat-multi-beam-geometry.h"

#include "sgp4unit.h"   // gstime() — GAST for ECI → ECEF rotation

#include "ns3/log.h"

#include <cassert>
#include <cmath>
#include <random>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatMultiBeamGeometry");

// ---------------------------------------------------------------------------
// Internal helpers (anonymous namespace — file-scope only)
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
    NS_LOG_FUNCTION(cfg.nUser << cfg.rFootprintM << seed);

    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.rFootprintM;
    const int    nUser = cfg.nUser;

    // Uniform sphere-cap sampling:
    //   theta = arccos(uniform(cos(r_foot/r_earth), 1))
    //   phi   = uniform(0, 2*pi)
    const double cosMax = std::cos(rFoot / rE);

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
        // re-centre z around footprint (subtract r_earth so nadir → z=0)
        users.push_back({x, y, z - rE});
    }

    NS_LOG_DEBUG("Generated " << users.size() << " random user positions");
    return users;
}

// ---------------------------------------------------------------------------
// GetGridUserPositions — mirrors networkGeometry.get_grid_positions()
// ---------------------------------------------------------------------------

std::vector<Vec3>
GetGridUserPositions(double spacingM, const SimConfig& cfg)
{
    NS_LOG_FUNCTION(spacingM << cfg.rFootprintM);

    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.rFootprintM;

    const int nTier = static_cast<int>(std::ceil(rFoot / spacingM + 1.0));

    std::vector<Vec3> users;
    users.reserve(nTier * (nTier + 1) * 3 + nTier);

    // Centre point (nadir — theta=0)
    users.push_back({0.0, 0.0, 0.0});

    // Concentric rings k=1..nTier, each with k*6 uniformly spaced points
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

    NS_LOG_DEBUG("Generated " << users.size()
                 << " grid positions  nTier=" << nTier
                 << "  spacingM=" << spacingM);
    return users;
}

// ---------------------------------------------------------------------------
// GetHexBeamCenters — mirrors networkGeometry.hex_grid_centers_two_rings()
// ---------------------------------------------------------------------------

std::array<Vec3, 19>
GetHexBeamCenters(const SimConfig& cfg)
{
    NS_LOG_FUNCTION(cfg.rFootprintM);

    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.rFootprintM;

    // Cell angular radius unit (matches Python: 2*r_footprint/10/r_earth)
    const double cellRad  = 2.0 * rFoot / 10.0 / rE;
    const double thetaSide = 1.0 / std::cos(M_PI / 6.0);   // = 2/sqrt(3)

    // Azimuth factor: phi = pi/2 + pi/6 * phiFactor[i]
    // 19 cells: top(3), second(4), centre(5), fourth(4), bottom(3)
    static const int phiFactor[19] = {
        4, 3, 2,           // top row
        5, 4, 2, 1,        // second row
        6, 6, 0, 0, 0,     // centre row
        7, 8, 10, 11,      // fourth row
        8, 9, 10           // bottom row
    };

    // Elevation factor: theta = cellRad * thetaFactor[i]
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
        centers[i] = {x, y, z - rE};
    }

    NS_LOG_DEBUG("Computed 19 hex beam centres (rFoot=" << rFoot << " m)");
    return centers;
}

// ---------------------------------------------------------------------------
// GetNadirFromTime — sub-satellite point at simulation time T
// ---------------------------------------------------------------------------

void
GetNadirFromTime(double timeS, const SimConfig& cfg, double& latDeg, double& lonDeg)
{
    NS_LOG_FUNCTION(timeS);

    // Arc angle eps at time T:
    //   The arc runs from eps_zero to pi - eps_zero.
    //   Frame index = timeS / tFrameS; angle step per frame = GetFrameAngleStepRad().
    const double eps0 = cfg.GetMinElevRad();
    const double dEps = cfg.GetFrameAngleStepRad();
    const double eps  = eps0 + (timeS / cfg.tFrameS) * dEps;

    const double rE = cfg.rEarthM;
    Vec3 nadirLocal;
    nadirLocal.x = rE * std::cos(eps);
    nadirLocal.y = 0.0;
    nadirLocal.z = rE * std::sin(eps) - rE;   // 0 at eps=pi/2 (pass centre)

    GetLatLon(nadirLocal, cfg, latDeg, lonDeg);

    NS_LOG_DEBUG("t=" << timeS << "s  eps=" << eps * 180.0 / M_PI
                 << " deg  nadir lat=" << latDeg << " lon=" << lonDeg);
}

// ---------------------------------------------------------------------------
// GetLatLon — mirrors utils.get_positions_in_lat_long_coordinates()
// ---------------------------------------------------------------------------

void
GetLatLon(const Vec3& pos, const SimConfig& cfg, double& latDeg, double& lonDeg)
{
    const double rE   = cfg.rEarthM;
    const double latC = cfg.latitudeCenterDeg  * M_PI / 180.0;
    const double lonC = cfg.longitudeCenterDeg * M_PI / 180.0;

    // Globe-centre Cartesian of the footprint centre
    // (physics convention: theta=colatitude)
    double xC, yC, zC;
    const double colatC = M_PI / 2.0 - latC;
    Pol2Cart3D(rE, lonC, colatC, xC, yC, zC);

    // Global Cartesian of point = footprint_centre + local_offset
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
// GetElevationAngleDeg_3D
// ---------------------------------------------------------------------------

double
GetElevationAngleDeg_3D(const Vec3& pos)
{
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
    // gstime() returns Greenwich Apparent Sidereal Time (radians).
    // Rotating ECI by -GAST around the z-axis gives ECEF.
    const double gast = gstime(jdUT1);
    const double cosG = std::cos(gast);
    const double sinG = std::sin(gast);

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

    // r ≈ 0 should never occur for LEO satellites
    if (r < 1.0)
    {
        NS_LOG_WARN("EcefToGeodetic: r=" << r << " m (near zero — unexpected)");
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
// ---------------------------------------------------------------------------

std::array<Vec3, 19>
GetBeamCentersFromSatPos(const Vec3& satEnu, const SimConfig& cfg)
{
    NS_LOG_FUNCTION(satEnu.x << satEnu.y << satEnu.z);

    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.rFootprintM;

    // -----------------------------------------------------------------------
    // Step 1: Nadir direction — from satellite toward Earth centre.
    //   Earth centre in ROI ENU frame is at (0, 0, -rE).
    // -----------------------------------------------------------------------
    const double dX  = -satEnu.x;
    const double dY  = -satEnu.y;
    const double dZ  = -rE - satEnu.z;
    const double len = std::sqrt(dX * dX + dY * dY + dZ * dZ);

    const Vec3 nadirUnit{dX / len, dY / len, dZ / len};

    // -----------------------------------------------------------------------
    // Step 2: Satellite beam-frame basis in ENU (Gram-Schmidt).
    //   xSat ← East direction projected perpendicular to nadirUnit.
    //   ySat ← nadirUnit × xSat  (right-hand, roughly North-like).
    // -----------------------------------------------------------------------
    const double dotE = nadirUnit.x;  // East·nadir (East unit = (1,0,0))
    Vec3 xSat{1.0 - dotE * nadirUnit.x,
                   - dotE * nadirUnit.y,
                   - dotE * nadirUnit.z};
    const double xLen = std::sqrt(xSat.x * xSat.x +
                                   xSat.y * xSat.y +
                                   xSat.z * xSat.z);
    xSat = {xSat.x / xLen, xSat.y / xLen, xSat.z / xLen};

    // ySat = nadirUnit × xSat
    const Vec3 ySat{
        nadirUnit.y * xSat.z - nadirUnit.z * xSat.y,
        nadirUnit.z * xSat.x - nadirUnit.x * xSat.z,
        nadirUnit.x * xSat.y - nadirUnit.y * xSat.x
    };

    // -----------------------------------------------------------------------
    // Step 3 & 4: For each of 19 beams, compute beam direction in ENU
    //   then intersect with the ground plane (z = 0).
    //
    //   The angular layout is identical to GetHexBeamCenters():
    //     phi_i   = π/2 + π/6 * phiFactor[i]
    //     theta_i = cellRad * thetaFactor[i]
    //   where theta is the polar angle from nadir (0 = directly below).
    // -----------------------------------------------------------------------
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

        // Beam direction in satellite frame (theta=0 ≡ nadir/Earth-centre).
        const double bx = std::sin(theta) * std::cos(phi);
        const double by = std::sin(theta) * std::sin(phi);
        const double bz = std::cos(theta);   // along nadirUnit

        // Rotate to ENU: d = bx*xSat + by*ySat + bz*nadirUnit
        const Vec3 d{
            bx * xSat.x + by * ySat.x + bz * nadirUnit.x,
            bx * xSat.y + by * ySat.y + bz * nadirUnit.y,
            bx * xSat.z + by * ySat.z + bz * nadirUnit.z
        };

        // Ray–ground intersection: satEnu + t*d, with z-component = 0.
        // d.z < 0 (pointing downward) so t > 0.
        const double t = -satEnu.z / d.z;

        centers[i] = Vec3{satEnu.x + t * d.x,
                           satEnu.y + t * d.y,
                           0.0};
    }

    NS_LOG_DEBUG("GetBeamCentersFromSatPos: centre beam at ("
                 << centers[9].x << ", " << centers[9].y << ") m");
    return centers;
}

} // namespace ns3
