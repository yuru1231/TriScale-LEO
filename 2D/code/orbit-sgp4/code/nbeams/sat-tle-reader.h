/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-tle-reader.h
 *
 * Reads the SNS3 constellation folder (tles.txt + beams/fwdConf.txt) and
 * initialises Vallado SGP4 records for all 66 satellites.
 *
 * Design:
 *   - SatTleEntry holds one satellite: name, raw TLE strings, initialised
 *     elsetrec, and the list of beam IDs assigned to it (from fwdConf.txt).
 *   - SatTleReader owns the vector of entries and exposes it read-only.
 *   - SGP4 initialisation (twoline2rv) runs at construction time so that
 *     callers pay the parse cost once and then call sgp4() freely.
 *
 * SGP4 time convention (matches Hypatia satellite.cc):
 *   twoline2rv called with typerun='c', typeinput='e', opsmode='i', wgs72.
 *   tsince (minutes since epoch) = simulation_time_s / 60.0.
 *   JD for GAST  = satrec.jdsatepoch + tsince / 1440.0.
 *
 * Required files in scratch (copy from hypatia-master/.../model/):
 *   sgp4unit.h / sgp4unit.cpp
 *   sgp4ext.h  / sgp4ext.cpp
 *   sgp4io.h   / sgp4io.cpp
 */

#ifndef SAT_TLE_READER_H
#define SAT_TLE_READER_H

#include "sgp4unit.h"   // elsetrec, sgp4(), gstime()
#include "sgp4io.h"     // twoline2rv()

#include <string>
#include <vector>

namespace ns3
{

/**
 * SatTleEntry — data for one satellite from the constellation folder.
 *
 * beamIds lists every forward-link beam ID that fwdConf.txt assigns to this
 * satellite (column 2, 1-indexed).  Used to know how many beams serve each
 * satellite for future beam-pattern lookup; not used by the scanner directly.
 */
struct SatTleEntry
{
    std::string name;          // e.g. "iridium-75 45"
    int         satIndex{0};   // 0-based index within tles.txt (= row in tles)
    char        tle1[130]{};   // TLE line 1 (mutable copy required by twoline2rv)
    char        tle2[130]{};   // TLE line 2
    elsetrec    satrec{};      // initialised SGP4 propagator record
    std::vector<int> beamIds;  // forward-link beam IDs assigned to this satellite
};

/**
 * SatTleReader — loads constellation TLE and beam-mapping data at construction.
 *
 * Usage:
 *   SatTleReader reader("path/to/tles.txt", "path/to/beams/fwdConf.txt");
 *   for (const auto& sat : reader.GetSatellites()) {
 *       double r[3], v[3];
 *       sgp4(wgs72, sat.satrec, tsince_min, r, v);
 *   }
 */
class SatTleReader
{
public:
    /**
     * @param tlesPath    Path to positions/tles.txt in the constellation folder.
     * @param fwdConfPath Path to beams/fwdConf.txt.
     */
    SatTleReader(const std::string& tlesPath, const std::string& fwdConfPath);

    const std::vector<SatTleEntry>& GetSatellites() const { return m_sats; }
    int GetNumSats() const { return static_cast<int>(m_sats.size()); }

    /** Julian date of TLE epoch (from satrec.jdsatepoch of satellite 0).
     *  Used to compute tsince and GAST JD for all sats (same epoch file). */
    double GetEpochJd() const { return m_epochJd; }

private:
    std::vector<SatTleEntry> m_sats;
    double                   m_epochJd{0.0};

    void ParseTles(const std::string& path);
    void ParseFwdConf(const std::string& path);
};

} // namespace ns3

#endif // SAT_TLE_READER_H
