/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-multi-beam-geometry.h
 *
 * Satellite and user position geometry for Multi-Beam LEO simulation.
 * Mirrors networkGeometry.py from the Python framework.
 *
 * [orbit-sgp4 version]
 * Arc-mode functions (GetSatelliteArcPositions, GetElevationAngleDeg,
 * GetSatellitePositionAtTime) have been removed.  All satellite motion is
 * driven by SGP4 orbit propagation.  Historical versions that retain arc
 * functions are in 2D/phase2/code/ and 2D/code/phase1-ns3/.
 *
 * Coordinate system (same as Python):
 *   Origin  : ground point directly below satellite at 90° elevation (nadir).
 *   x-axis  : along-track direction (east for ascending orbit).
 *   y-axis  : cross-track direction.
 *   z-axis  : radially outward from Earth centre.
 *   Units   : metres.
 *
 * All functions are free functions (not ns3::Object); they are pure
 * geometry utilities with no internal state.  NS_LOG macros are used
 * where helpful for tracing.
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
// Nadir sub-satellite point
// ---------------------------------------------------------------------------

/**
 * GetNadirFromTime
 *
 * Compute the geographic position of the satellite sub-satellite point
 * (nadir) at simulation time T using the simplified arc orbit model.
 * Replaces frame-index lookup: the caller supplies time T (seconds) and
 * receives lat/lon of the nadir directly.
 *
 * Equivalent SNS3 call: create_basic_ground_station_for_satellite_shadow()
 * returns the same moving ground point at gid=-1.
 *
 * @param timeS    Simulation time (seconds from pass start).
 * @param cfg      Simulation configuration (orbit parameters, pass centre).
 * @param latDeg   Output — nadir latitude  (degrees N).
 * @param lonDeg   Output — nadir longitude (degrees E).
 */
void GetNadirFromTime(double           timeS,
                      const SimConfig& cfg,
                      double&          latDeg,
                      double&          lonDeg);

// ---------------------------------------------------------------------------
// User position generation
// ---------------------------------------------------------------------------

/**
 * GetRandomUserPositions
 *
 * Uniformly distribute n_user positions on the spherical surface within
 * the footprint circle (radius r_footprint from nadir).
 * Mirrors networkGeometry.get_user_position().
 *
 * Uniform sphere cap sampling:
 *   theta = arccos(uniform(cos(r_footprint/r_earth), 1))
 *   phi   = uniform(0, 2*pi)
 * Result converted to 3-D Cartesian re-centred at footprint origin.
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
 * @param cfg  Simulation configuration (r_footprint, r_earth).
 * @return     Array of 19 Vec3 beam centre positions.
 */
std::array<Vec3, 19> GetHexBeamCenters(const SimConfig& cfg);

// ---------------------------------------------------------------------------
// Coordinate helper — local Vec3 to geographic lat/lon
// ---------------------------------------------------------------------------

/**
 * GetLatLon
 *
 * Convert a Vec3 position (local Cartesian centred at nadir) to geographic
 * latitude and longitude (degrees).
 * Mirrors utils.get_positions_in_lat_long_coordinates().
 *
 * @param pos    Local Cartesian position (m).
 * @param cfg    Simulation configuration (latitude_center, longitude_center, r_earth).
 * @param latDeg Output latitude  (degrees N).
 * @param lonDeg Output longitude (degrees E).
 */
void GetLatLon(const Vec3&      pos,
               const SimConfig& cfg,
               double&          latDeg,
               double&          lonDeg);

// ---------------------------------------------------------------------------
// ENU frame helpers
// ---------------------------------------------------------------------------

/**
 * GetElevationAngleDeg_3D
 *
 * Elevation angle (degrees) from the local-frame origin to a 3-D position
 * expressed in ENU frame (x=East, y=North, z=Up).
 *
 *   horiz = sqrt(x² + y²)
 *   elev  = atan2(z, horiz)   (returns 90° when horiz ≈ 0)
 *
 * @param pos  Position in local ENU frame (m).
 * @return     Elevation angle (degrees, -90 to +90).
 */
double GetElevationAngleDeg_3D(const Vec3& pos);

/**
 * EcefOffsetToEnu
 *
 * Rotate a Vec3 in ECEF-offset frame (output of GetHexBeamCenters or
 * GetGridUserPositions) to the local ENU frame at the observer location.
 *
 * Rotation (standard ECEF → ENU):
 *   E = -sin(lon)*x + cos(lon)*y
 *   N = -sin(lat)*cos(lon)*x - sin(lat)*sin(lon)*y + cos(lat)*z
 *   U =  cos(lat)*cos(lon)*x + cos(lat)*sin(lon)*y + sin(lat)*z
 *
 * @param ecefOffset  Vec3 in ECEF-offset frame (m).
 * @param obsLatDeg   Observer geodetic latitude  (degrees N).
 * @param obsLonDeg   Observer geodetic longitude (degrees E).
 * @return            Vec3 in local ENU frame (m).
 */
Vec3 EcefOffsetToEnu(const Vec3& ecefOffset, double obsLatDeg, double obsLonDeg);

// ---------------------------------------------------------------------------
// SGP4 coordinate bridge (constellation mode)
// ---------------------------------------------------------------------------

/**
 * EciToEcef
 *
 * Rotate a position vector from ECI (TEME) to ECEF by applying Earth's
 * rotation (GAST).  Uses gstime() from sgp4unit to compute Greenwich
 * Apparent Sidereal Time from the Julian Date.
 *
 * ECEF and ECI share the z-axis; only an x-y rotation by -GAST is needed:
 *   ecef.x =  eci.x * cos(gast) + eci.y * sin(gast)
 *   ecef.y = -eci.x * sin(gast) + eci.y * cos(gast)
 *   ecef.z =  eci.z
 *
 * @param eciKm   Position in ECI/TEME frame (km), as returned by sgp4().
 * @param jdUT1   Julian Date (UT1) at the propagation instant.
 *                Compute as: satrec.jdsatepoch + tsince_min / 1440.0
 * @return        Position in ECEF frame (km).
 */
Vec3 EciToEcef(const Vec3& eciKm, double jdUT1);

/**
 * EcefToGeodetic
 *
 * Convert an ECEF position (metres) to geodetic coordinates using a
 * spherical Earth model (radius = rEarthM).
 *
 * lat = asin(z / r)
 * lon = atan2(y, x)
 * alt = r - rEarthM
 *
 * @param ecefM    Position in ECEF frame (metres).
 * @param rEarthM  Mean Earth radius (metres).
 * @param latDeg   Output geodetic latitude  (degrees N).
 * @param lonDeg   Output geodetic longitude (degrees E).
 * @param altM     Output altitude above sphere surface (metres).
 */
void EcefToGeodetic(const Vec3& ecefM,
                    double      rEarthM,
                    double&     latDeg,
                    double&     lonDeg,
                    double&     altM);

// ---------------------------------------------------------------------------
// Elliptical beam footprint (off-nadir correction)
// ---------------------------------------------------------------------------

/**
 * GetBeamCentersFromSatPos
 *
 * Compute 19 beam-centre ground positions in the ROI ENU frame,
 * accounting for the satellite's actual off-nadir position.
 *
 * GetHexBeamCenters() assumes the satellite is directly overhead (nadir).
 * This function instead:
 *   1. Derives the true nadir direction from the satellite to the Earth centre.
 *   2. Builds a local beam-frame basis (Gram-Schmidt) aligned to that direction.
 *   3. Projects each beam's steering angle onto the ground (z=0 ray intersection).
 *
 * At nadir the result converges to GetHexBeamCenters() + EcefOffsetToEnu().
 * At low elevation the pattern elongates into an ellipse (long axis ∝ 1/sin(ε)).
 *
 * Must be called per-timestep as satEnu changes every second.
 *
 * @param satEnu  Satellite position in ROI ENU frame (metres), from SGP4.
 * @param cfg     Simulation config (rEarthM, rFootprintM).
 * @return        Array of 19 Vec3 beam-centre positions on the ground (z=0, ENU).
 */
std::array<Vec3, 19> GetBeamCentersFromSatPos(const Vec3& satEnu,
                                               const SimConfig& cfg);

} // namespace ns3

#endif // SAT_MULTI_BEAM_GEOMETRY_H
