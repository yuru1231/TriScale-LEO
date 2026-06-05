/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-tle-reader.h
 *
 * ns3::Object that reads the SNS3 constellation folder
 * (positions/tles.txt + beams/fwdConf.txt) and initialises Vallado
 * SGP4 records for all 66 satellites.
 *
 * Design:
 *   - SatTleEntry holds one satellite: name, raw TLE strings, initialised
 *     elsetrec, and the list of beam IDs assigned to it (from fwdConf.txt).
 *   - SatTleReader is an ns3::Object; call Load() after CreateObject<>().
 *   - SGP4 initialisation (twoline2rv) runs inside Load() so callers
 *     pay the parse cost once and then call sgp4() freely.
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

#include "ns3/object.h"

#include <string>
#include <vector>

namespace ns3
{

/**
 * SatTleEntry — data for one satellite from the constellation folder.
 *
 * Plain struct (value type); not an ns3::Object.
 * beamIds lists every forward-link beam ID that fwdConf.txt assigns to this
 * satellite (column 2, 1-indexed).
 */
struct SatTleEntry
{
    std::string       name;          // e.g. "iridium-75 45"
    int               satIndex{0};   // 0-based index within tles.txt
    char              tle1[130]{};   // TLE line 1 (mutable copy for twoline2rv)
    char              tle2[130]{};   // TLE line 2
    elsetrec          satrec{};      // initialised SGP4 propagator record
    std::vector<int>  beamIds;       // forward-link beam IDs on this satellite
};

/**
 * SatTleReader — ns3::Object that owns the constellation TLE dataset.
 *
 * Usage:
 *   Ptr<SatTleReader> reader = CreateObject<SatTleReader>();
 *   reader->Load("path/tles.txt", "path/beams/fwdConf.txt");
 *   for (const auto& sat : reader->GetSatellites()) {
 *       double r[3], v[3];
 *       sgp4(wgs72, sat.satrec, tsince_min, r, v);
 *   }
 */
class SatTleReader : public Object
{
public:
    /**
     * GetTypeId — ns3 TypeId registration.
     * Group: Satellite.
     */
    static TypeId GetTypeId();

    SatTleReader()           = default;
    ~SatTleReader() override = default;

    /**
     * Load — parse tles.txt and fwdConf.txt, initialise SGP4 records.
     *
     * Must be called once after CreateObject<SatTleReader>() before any
     * getter is used.  Throws std::runtime_error on file-not-found.
     *
     * @param tlesPath     Path to positions/tles.txt in the constellation folder.
     * @param fwdConfPath  Path to beams/fwdConf.txt.
     */
    void Load(const std::string& tlesPath, const std::string& fwdConfPath);

    /** Return the full satellite list (read-only). */
    const std::vector<SatTleEntry>& GetSatellites() const { return m_sats; }

    /** Number of satellites loaded. */
    int GetNumSats() const { return static_cast<int>(m_sats.size()); }

    /**
     * GetEpochJd — Julian date of TLE epoch (from satrec.jdsatepoch of
     * satellite 0).  Used to compute tsince and GAST JD for all satellites
     * (same epoch file).
     */
    double GetEpochJd() const { return m_epochJd; }

private:
    std::vector<SatTleEntry> m_sats;
    double                   m_epochJd{0.0};

    // Internal parsers
    void ParseTles(const std::string& path);
    void ParseFwdConf(const std::string& path);
};

} // namespace ns3

#endif // SAT_TLE_READER_H
