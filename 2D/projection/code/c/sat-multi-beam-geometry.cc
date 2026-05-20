/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-geometry.cc
 *
 * Implementation of geometry functions for Multi-Beam LEO simulation.
 * See sat-multi-beam-geometry.h for full API documentation.
 */

#include "sat-multi-beam-geometry.h"

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
// GetSatelliteArcPositions — mirrors networkGeometry.get_satellite_pos()
// ---------------------------------------------------------------------------

std::vector<Vec3>
GetSatelliteArcPositions(const SimConfig& cfg)
{
    const double R   = cfg.GetOrbitalRadius();      // orbital radius (m)
    const double eps0 = cfg.GetMinElevRad();        // first/last arc angle (rad)
    const double dEps = cfg.GetFrameAngleStepRad(); // angle increment per frame

    // eps runs from eps0 to pi - eps0 exclusive
    const double epsEnd = M_PI - eps0;
    std::vector<Vec3> positions;
    positions.reserve(cfg.GetTotalFrames() + 2);

    for (double eps = eps0; eps < epsEnd; eps += dEps)
    {
        Vec3 p;
        // pol2cart: x = R*cos(eps), z = R*sin(eps)
        p.x = R * std::cos(eps);
        p.y = 0.0;
        // re-centre z around footprint centre (subtract r_earth)
        p.z = R * std::sin(eps) - cfg.rEarthM;
        positions.push_back(p);
    }
    return positions;
}

// ---------------------------------------------------------------------------
// GetElevationAngleDeg — mirrors utils.get_elevation_angle_from_center()
// ---------------------------------------------------------------------------

double
GetElevationAngleDeg(const Vec3& satPos)
{
    // Elevation angle from footprint centre (origin).
    // Python: arctan(z_sat / |x_sat|), with special case x_sat == 0 → 90°.
    if (satPos.x == 0.0)
    {
        return 90.0;
    }
    double elevRad = std::atan(satPos.z / std::abs(satPos.x));
    // If x < 0 the satellite is on the far side → pi - elev (Python logic)
    if (satPos.x < 0.0)
    {
        elevRad = M_PI - elevRad;
    }
    return elevRad * 180.0 / M_PI;
}

// ---------------------------------------------------------------------------
// GetRandomUserPositions — mirrors networkGeometry.get_user_position()
// ---------------------------------------------------------------------------

std::vector<Vec3>
GetRandomUserPositions(const SimConfig& cfg, uint32_t seed)
{
    const double rE    = cfg.rEarthM;
    const double rFoot = cfg.rFootprintM;
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
    const double rFoot = cfg.rFootprintM;

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
    const double rFoot = cfg.rFootprintM;

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
    // factors expressed as multiples of 1 or thetaSide
    // Python: [4, 3*ts, 4, 3*ts, 2, 2, 3*ts, 4, 2, 0, 2, 4, 3*ts, 2, 2, 3*ts, 4, 3*ts, 4]
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
    // Python uses pol2cart3D(r_earth, latitude_center, longitude_center)
    // Physics convention: theta=colatitude, phi=longitude
    // BUT Python's latitude_center is latitude not colatitude →
    // theta_colat = pi/2 - lat
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

} // namespace ns3
