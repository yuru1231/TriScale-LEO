/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-geometry.h
 *
 * Satellite and user position geometry for Multi-Beam LEO simulation.
 * Mirrors networkGeometry.py from the Python framework.
 *
 * Coordinate system (same as Python):
 *   Origin  : ground point directly below satellite at 90° elevation (nadir).
 *   x-axis  : along-track direction (east for ascending orbit).
 *   y-axis  : cross-track direction.
 *   z-axis  : radially outward from Earth centre.
 *   Units   : metres.
 *
 * At 90° elevation the satellite is at [0, 0, h_satellite].
 * Users are distributed on the spherical Earth surface near this centre point.
 */

#ifndef SAT_MULTI_BEAM_GEOMETRY_H
#define SAT_MULTI_BEAM_GEOMETRY_H

#include "sat-multi-beam-config.h"

#include <array>
#include <cmath>
#include <random>
#include <vector>

namespace ns3
{

/** Simple 3-element Cartesian vector (x, y, z) in metres. */
struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

// ---------------------------------------------------------------------------
// Satellite arc trajectory — mirrors networkGeometry.get_satellite_pos()
// ---------------------------------------------------------------------------

/**
 * GetSatelliteArcPositions
 *
 * Returns the satellite position (Vec3) for every frame of the pass.
 * The satellite moves in the x-z plane (y = 0) along a circular arc at
 * orbital radius R = r_earth + h_satellite.
 *
 * Arc parameter eps runs from eps_zero to pi - eps_zero in steps of
 * delta_eps = v_sat * t_frame / R.
 *
 *   x_sat[k] = R * cos(eps[k])
 *   z_sat[k] = R * sin(eps[k]) - r_earth   (re-centred to footprint)
 *   y_sat[k] = 0
 *
 * Frame index for specific elevation angles (default params):
 *   frame 38537 → 90°  (nadir)
 *   frame 33090 → ~55°
 *   frame 23932 → ~25°
 *
 * @param cfg  Simulation configuration.
 * @return     Vector of Vec3 positions, one per frame.
 */
std::vector<Vec3> GetSatelliteArcPositions(const SimConfig& cfg);

/**
 * GetElevationAngleDeg
 *
 * Elevation angle (degrees) of the satellite position as seen from the
 * footprint centre (origin).  Mirrors utils.get_elevation_angle_from_center().
 *
 *   elev = atan(z_sat / |x_sat|)   (y_sat always 0)
 *
 * Returns 90° when x_sat == 0.
 */
double GetElevationAngleDeg(const Vec3& satPos);

// ---------------------------------------------------------------------------
// User position generation
// ---------------------------------------------------------------------------

/**
 * GetRandomUserPositions
 *
 * Randomly distribute n_user positions uniformly on the spherical surface
 * within the footprint circle (radius r_footprint from nadir).
 * Mirrors networkGeometry.get_user_position().
 *
 * Uniform sphere cap sampling:
 *   theta = arccos(uniform(cos(r_footprint/r_earth), 1))
 *   phi   = uniform(0, 2*pi)
 * Then converted to 3-D Cartesian re-centred at the footprint origin.
 *
 * @param cfg   Simulation configuration (r_footprint, r_earth, n_user).
 * @param seed  RNG seed (default 42, for reproducibility).
 * @return      Vector of Vec3 user positions.
 */
std::vector<Vec3> GetRandomUserPositions(const SimConfig& cfg, uint32_t seed = 42);

/**
 * GetGridUserPositions
 *
 * Concentric ring grid — mirrors networkGeometry.get_grid_positions().
 * Rings at radii r_footprint/n_tier * k (k=1..n_tier), each with k*6 points.
 * Centre point included.
 *
 * @param spacingM  Radial step between rings (metres).
 * @param cfg       Simulation configuration.
 * @return          Vector of Vec3 user positions.
 */
std::vector<Vec3> GetGridUserPositions(double spacingM, const SimConfig& cfg);

// ---------------------------------------------------------------------------
// Beam centres — mirrors networkGeometry.hex_grid_centers_two_rings()
// ---------------------------------------------------------------------------

/**
 * GetHexBeamCenters
 *
 * Returns the 19 beam centre positions (Vec3) for the two-ring flat-top
 * hexagonal layout used in the Python framework.
 *
 * Positions are on the Earth sphere surface (re-centred to footprint origin).
 * The angular radius of each cell: 2*r_footprint/10 / r_earth.
 *
 * Layout (row order from top to bottom):
 *   Top row (3):       ring-2 corners
 *   Second row (4):    alternating ring-2 / ring-1
 *   Centre row (5):    ring-2, ring-1, centre, ring-1, ring-2
 *   Fourth row (4):    mirror of second row
 *   Bottom row (3):    mirror of top row
 *
 * @param cfg  Simulation configuration (r_footprint, r_earth).
 * @return     Array of 19 Vec3 beam centre positions.
 */
std::array<Vec3, 19> GetHexBeamCenters(const SimConfig& cfg);

// ---------------------------------------------------------------------------
// Coordinate helper — lat/lon of a Vec3 position
// ---------------------------------------------------------------------------

/**
 * GetLatLon
 *
 * Convert a Vec3 position (local Cartesian centred at nadir) to geographic
 * latitude and longitude (degrees).  Mirrors utils.get_positions_in_lat_long_coordinates().
 *
 * @param pos    Local Cartesian position (m).
 * @param cfg    Simulation configuration (latitude_center, longitude_center, r_earth).
 * @param latDeg Output latitude (degrees).
 * @param lonDeg Output longitude (degrees).
 */
void GetLatLon(const Vec3& pos,
               const SimConfig& cfg,
               double& latDeg,
               double& lonDeg);

} // namespace ns3

#endif // SAT_MULTI_BEAM_GEOMETRY_H
