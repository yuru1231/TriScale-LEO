/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-tle-reader.cc
 *
 * ns3::Object implementation of SatTleReader.
 * See sat-tle-reader.h for API documentation and design notes.
 *
 * Logging component: SatTleReader
 *   Enable with: LogComponentEnable("SatTleReader", LOG_LEVEL_INFO)
 */

#include "sat-tle-reader.h"

#include "sgp4ext.h"   // jday(), etc.

#include "ns3/log.h"
#include "ns3/abort.h"

#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatTleReader");
NS_OBJECT_ENSURE_REGISTERED(SatTleReader);

// ---------------------------------------------------------------------------
// TypeId registration
// ---------------------------------------------------------------------------

TypeId
SatTleReader::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatTleReader")
            .SetParent<Object>()
            .SetGroupName("Satellite")
            .AddConstructor<SatTleReader>();
    return tid;
}

// ---------------------------------------------------------------------------
// Load — public entry point (replaces constructor args)
// ---------------------------------------------------------------------------

void
SatTleReader::Load(const std::string& tlesPath,
                   const std::string& fwdConfPath)
{
    NS_LOG_FUNCTION(this << tlesPath << fwdConfPath);

    ParseTles(tlesPath);
    ParseFwdConf(fwdConfPath);

    if (!m_sats.empty())
    {
        m_epochJd = m_sats[0].satrec.jdsatepoch;
    }

    NS_LOG_INFO("Loaded " << m_sats.size() << " satellites"
                << "  epochJD=" << m_epochJd);
}

// ---------------------------------------------------------------------------
// ParseTles — read positions/tles.txt
//
// Format:
//   Line 0 : "nPlanes nSatsPerPlane"    (header, logged but not parsed)
//   Repeated triplets:
//     <satellite name>
//     1 <TLE line 1>
//     2 <TLE line 2>
// ---------------------------------------------------------------------------

void
SatTleReader::ParseTles(const std::string& path)
{
    NS_LOG_FUNCTION(this << path);

    std::ifstream f(path);
    NS_ABORT_MSG_IF(!f.is_open(), "SatTleReader: cannot open TLE file: " << path);

    // First line is the "nPlanes nSatsPerPlane" header — skip for parsing
    std::string header;
    std::getline(f, header);
    NS_LOG_DEBUG("TLE file header: " << header);

    int satIndex = 0;
    std::string line;

    while (std::getline(f, line))
    {
        if (line.empty()) { continue; }

        SatTleEntry entry;
        entry.name     = line;
        entry.satIndex = satIndex++;

        std::string tleStr1, tleStr2;
        if (!std::getline(f, tleStr1) || !std::getline(f, tleStr2))
        {
            NS_ABORT_MSG("SatTleReader: truncated TLE file at satellite: " << entry.name);
        }

        // twoline2rv modifies its char[] arguments in-place — use fixed buffers
        std::strncpy(entry.tle1, tleStr1.c_str(), 129);
        std::strncpy(entry.tle2, tleStr2.c_str(), 129);
        entry.tle1[129] = '\0';
        entry.tle2[129] = '\0';

        // Initialise SGP4 record — mirrors Hypatia satellite.cc usage
        double startmfe, stopmfe, deltamin;
        twoline2rv(entry.tle1, entry.tle2,
                   'c',    // typerun  : catalog mode
                   'e',    // typeinput: epoch-relative time
                   'i',    // opsmode  : improved
                   wgs72,
                   startmfe, stopmfe, deltamin,
                   entry.satrec);

        // Verify SGP4 initialised correctly (tsince=0 must return valid r/v)
        double r[3], v[3];
        bool ok = sgp4(wgs72, entry.satrec, 0.0, r, v);
        if (!ok || entry.satrec.error != 0)
        {
            NS_LOG_WARN("SGP4 init failed for " << entry.name
                        << "  error=" << entry.satrec.error);
        }
        else
        {
            NS_LOG_DEBUG("Loaded sat[" << entry.satIndex << "] " << entry.name);
        }

        m_sats.push_back(std::move(entry));
    }
}

// ---------------------------------------------------------------------------
// ParseFwdConf — read beams/fwdConf.txt
//
// Format (space-separated, no header):
//   beam_id  sat_id  freq_color  beam_index_on_sat
//
// sat_id is 1-indexed in fwdConf; satIndex in m_sats is 0-indexed.
// ---------------------------------------------------------------------------

void
SatTleReader::ParseFwdConf(const std::string& path)
{
    NS_LOG_FUNCTION(this << path);

    std::ifstream f(path);
    NS_ABORT_MSG_IF(!f.is_open(),
                    "SatTleReader: cannot open fwdConf file: " << path);

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#') { continue; }

        std::istringstream ss(line);
        int beamId, satId, freqColor, beamIdxOnSat;
        if (!(ss >> beamId >> satId >> freqColor >> beamIdxOnSat)) { continue; }

        // fwdConf sat_id is 1-indexed → convert to 0-based
        const int satIdx = satId - 1;
        if (satIdx < 0 || satIdx >= static_cast<int>(m_sats.size()))
        {
            NS_LOG_WARN("fwdConf beam " << beamId
                        << " references out-of-range satId=" << satId);
            continue;
        }
        m_sats[satIdx].beamIds.push_back(beamId);
    }

    NS_LOG_INFO("fwdConf parsed: beam assignments loaded for "
                << m_sats.size() << " satellites");
}

} // namespace ns3
