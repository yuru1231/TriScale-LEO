/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-orbit-reader.h
 *
 * Phase 2.0 — SGP4 satellite orbit CSV reader and coordinate converter.
 *
 * Reads the output of run_sgp4.py (satellite_orbit.csv) and provides:
 *   - OrbitPoint: one timestep of satellite position data.
 *   - LoadOrbit(): parse the CSV, skip metadata comment lines.
 *   - OrbitPointToEnuVec3(): convert geodetic satellite position to the
 *     local ENU Cartesian frame centred at the fixed observation point.
 *
 * Coordinate convention (ENU, same as RoiCell positions):
 *   x → East,  y → North,  z → Up (altitude above observer surface ≈ alt_m)
 *
 * The returned Vec3 satisfies:
 *   satPos.z + rEarth ≈ orbital radius   (flat-Earth approximation)
 * which is the value required by BuildArrayTransform() for beam gain.
 *
 * A spherical Earth (radius = cfg.rEarthM) is used consistently with the
 * rest of the simulation framework.
 */

#ifndef SAT_ORBIT_READER_H
#define SAT_ORBIT_READER_H

#include "sat-multi-beam-geometry.h"

#include <string>
#include <vector>

namespace ns3
{

/**
 * OrbitPoint — one row from satellite_orbit.csv (output of run_sgp4.py).
 *
 * sat_lat_deg / sat_lon_deg are WGS72 GEODETIC coordinates (Bowring-converted
 * by run_sgp4.py from PyEphem's geocentric sublat/sublong).
 *
 * elevation_deg / azimuth_deg are observer-relative look angles from PyEphem
 * and are used directly for the satellite visibility threshold check.
 */
struct OrbitPoint
{
    double timeS{0.0};         // seconds since simulation start
    double latDeg{0.0};        // satellite geodetic latitude  (degrees N)
    double lonDeg{0.0};        // satellite geodetic longitude (degrees E)
    double altM{0.0};          // satellite altitude above ellipsoid (m)
    double elevationDeg{0.0};  // elevation angle from observer (degrees, PyEphem)
    double azimuthDeg{0.0};    // azimuth from observer (degrees, N=0, PyEphem)
};

/**
 * LoadOrbit
 *
 * Parse satellite_orbit.csv produced by run_sgp4.py.
 *
 * The file may contain comment lines (starting with '#') before the CSV
 * header.  These are skipped automatically.  Column order is resolved from
 * the header line, so extra columns are harmless.
 *
 * Required columns: time_s, sat_lat_deg, sat_lon_deg, sat_alt_m
 * Optional columns: elevation_deg, azimuth_deg (set to 0 if absent)
 *
 * @param csvPath  Path to satellite_orbit.csv.
 * @return         Vector of OrbitPoint, one per data row, in file order.
 */
std::vector<OrbitPoint> LoadOrbit(const std::string& csvPath);

/**
 * OrbitPointToEnuVec3
 *
 * Convert a satellite geodetic position to a local ENU Vec3 relative to the
 * fixed observation point (ROI centre).
 *
 * Algorithm:
 *   1. Satellite ECEF (spherical Earth, r_sat = rEarthM + altM):
 *        sx = r_sat · cos(lat_sat) · cos(lon_sat)
 *        sy = r_sat · cos(lat_sat) · sin(lon_sat)
 *        sz = r_sat · sin(lat_sat)
 *   2. Observer ECEF:
 *        ox = rEarthM · cos(lat_obs) · cos(lon_obs)
 *        oy = rEarthM · cos(lat_obs) · sin(lon_obs)
 *        oz = rEarthM · sin(lat_obs)
 *   3. ECEF delta: Δ = sat_ecef − obs_ecef
 *   4. ENU rotation at observer:
 *        E = −sin(lon_obs)·Δx + cos(lon_obs)·Δy
 *        N = −sin(lat_obs)·cos(lon_obs)·Δx
 *            − sin(lat_obs)·sin(lon_obs)·Δy
 *            + cos(lat_obs)·Δz
 *        U =  cos(lat_obs)·cos(lon_obs)·Δx
 *            + cos(lat_obs)·sin(lon_obs)·Δy
 *            + sin(lat_obs)·Δz
 *   5. Return Vec3{E, N, U}
 *
 * Physical interpretation of satPos.z = U:
 *   U ≈ alt_m for all elevation angles (flat-Earth approximation).
 *   Therefore satPos.z + rEarthM ≈ rEarthM + alt_m ≈ orbital radius,
 *   which is what BuildArrayTransform() requires for the y-axis rotation.
 *
 * @param pt          OrbitPoint with geodetic position.
 * @param obsLatDeg   Observer geodetic latitude  (degrees N).
 * @param obsLonDeg   Observer geodetic longitude (degrees E).
 * @param rEarthM     Mean Earth radius (m), from SimConfig::rEarthM.
 * @return            Satellite ENU position Vec3 (m).
 */
Vec3 OrbitPointToEnuVec3(const OrbitPoint& pt,
                          double obsLatDeg,
                          double obsLonDeg,
                          double rEarthM);

} // namespace ns3

#endif // SAT_ORBIT_READER_H
