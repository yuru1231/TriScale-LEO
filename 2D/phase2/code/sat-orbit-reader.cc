/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-orbit-reader.cc
 *
 * Phase 2.0 — SGP4 orbit CSV reader and ECEF → ENU coordinate converter.
 * See sat-orbit-reader.h for full API documentation.
 */

#include "sat-orbit-reader.h"

#include "ns3/abort.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3
{

// ---------------------------------------------------------------------------
// LoadOrbit
// ---------------------------------------------------------------------------

std::vector<OrbitPoint>
LoadOrbit(const std::string& csvPath)
{
    std::ifstream in(csvPath);
    NS_ABORT_MSG_IF(!in.is_open(), "Cannot open orbit CSV: " << csvPath);

    std::vector<OrbitPoint> orbit;
    std::string line;
    bool headerFound = false;

    // Column indices resolved from the CSV header line
    int idxTime{-1}, idxLat{-1}, idxLon{-1}, idxAlt{-1};
    int idxElev{-1}, idxAz{-1};

    while (std::getline(in, line))
    {
        // Trim trailing carriage return (Windows line endings)
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }

        // Skip blank lines and run_sgp4.py metadata comment lines
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        if (!headerFound)
        {
            // Parse header to determine column order
            std::istringstream hss(line);
            std::string colName;
            int idx = 0;
            while (std::getline(hss, colName, ','))
            {
                // Trim leading/trailing spaces
                colName.erase(0, colName.find_first_not_of(" \t"));
                colName.erase(colName.find_last_not_of(" \t") + 1);

                if      (colName == "time_s")       idxTime = idx;
                else if (colName == "sat_lat_deg")  idxLat  = idx;
                else if (colName == "sat_lon_deg")  idxLon  = idx;
                else if (colName == "sat_alt_m")    idxAlt  = idx;
                else if (colName == "elevation_deg") idxElev = idx;
                else if (colName == "azimuth_deg")  idxAz   = idx;
                ++idx;
            }

            NS_ABORT_MSG_IF(
                idxTime < 0 || idxLat < 0 || idxLon < 0 || idxAlt < 0,
                "satellite_orbit.csv missing required columns."
                "  Expected: time_s, sat_lat_deg, sat_lon_deg, sat_alt_m"
                "  Found header: " << line);

            headerFound = true;
            continue;
        }

        // Parse data row
        std::istringstream rss(line);
        std::string tok;
        std::vector<std::string> fields;
        while (std::getline(rss, tok, ','))
        {
            fields.push_back(tok);
        }

        const int requiredCols = std::max({idxTime, idxLat, idxLon, idxAlt}) + 1;
        if (static_cast<int>(fields.size()) < requiredCols)
        {
            continue; // skip malformed rows
        }

        OrbitPoint pt;
        pt.timeS = std::stod(fields[idxTime]);
        pt.latDeg = std::stod(fields[idxLat]);
        pt.lonDeg = std::stod(fields[idxLon]);
        pt.altM   = std::stod(fields[idxAlt]);
        pt.elevationDeg = (idxElev >= 0 && idxElev < static_cast<int>(fields.size()))
                          ? std::stod(fields[idxElev]) : 0.0;
        pt.azimuthDeg   = (idxAz   >= 0 && idxAz   < static_cast<int>(fields.size()))
                          ? std::stod(fields[idxAz])   : 0.0;

        orbit.push_back(pt);
    }

    NS_ABORT_MSG_IF(orbit.empty(), "No orbit data rows found in: " << csvPath);
    return orbit;
}

// ---------------------------------------------------------------------------
// OrbitPointToEnuVec3
// ---------------------------------------------------------------------------

Vec3
OrbitPointToEnuVec3(const OrbitPoint& pt,
                     double obsLatDeg,
                     double obsLonDeg,
                     double rEarthM)
{
    const double d2r = M_PI / 180.0;

    const double latSat = pt.latDeg * d2r;
    const double lonSat = pt.lonDeg * d2r;
    const double latObs = obsLatDeg * d2r;
    const double lonObs = obsLonDeg * d2r;

    // Step 1: Satellite ECEF (spherical Earth, r_sat = rEarth + alt_m)
    const double rSat = rEarthM + pt.altM;
    const double sx = rSat * std::cos(latSat) * std::cos(lonSat);
    const double sy = rSat * std::cos(latSat) * std::sin(lonSat);
    const double sz = rSat * std::sin(latSat);

    // Step 2: Observer ECEF (on the surface, altitude = 0)
    const double ox = rEarthM * std::cos(latObs) * std::cos(lonObs);
    const double oy = rEarthM * std::cos(latObs) * std::sin(lonObs);
    const double oz = rEarthM * std::sin(latObs);

    // Step 3: ECEF offset vector (satellite relative to observer)
    const double dx = sx - ox;
    const double dy = sy - oy;
    const double dz = sz - oz;

    // Step 4: Rotate ECEF delta to local ENU at observer
    // E = -sin(lon_obs)*dx + cos(lon_obs)*dy
    // N = -sin(lat_obs)*cos(lon_obs)*dx - sin(lat_obs)*sin(lon_obs)*dy + cos(lat_obs)*dz
    // U =  cos(lat_obs)*cos(lon_obs)*dx + cos(lat_obs)*sin(lon_obs)*dy + sin(lat_obs)*dz
    const double sinLat = std::sin(latObs);
    const double cosLat = std::cos(latObs);
    const double sinLon = std::sin(lonObs);
    const double cosLon = std::cos(lonObs);

    const double E = -sinLon * dx + cosLon * dy;
    const double N = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz;
    const double U =  cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz;

    // Vec3{E, N, U}: East, North, Up
    // U ≈ alt_m (flat-Earth), so satPos.z + rEarthM ≈ orbital radius
    return Vec3{E, N, U};
}

} // namespace ns3
