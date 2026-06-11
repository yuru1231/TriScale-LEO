/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-tle-reader.cc
 *
 * Implementation of SatTleReader.
 * See sat-tle-reader.h for API documentation and design notes.
 */

#include "sat-tle-reader.h"

#include "sgp4ext.h"   // jday(), etc.

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace ns3
{

// ---------------------------------------------------------------------------
// SatTleReader constructor
// ---------------------------------------------------------------------------

SatTleReader::SatTleReader(const std::string& tlesPath,
                           const std::string& fwdConfPath)
{
    ParseTles(tlesPath);
    ParseFwdConf(fwdConfPath);

    if (!m_sats.empty())
    {
        m_epochJd = m_sats[0].satrec.jdsatepoch;
    }

    std::cout << "[SatTleReader] Loaded " << m_sats.size() << " satellites"
              << "  epoch JD=" << m_epochJd << "\n";
}

// ---------------------------------------------------------------------------
// ParseTles — read positions/tles.txt
//
// Format:
//   Line 0 : "nPlanes nSatsPerPlane"
//   Then repeated triplets:
//     <satellite name>
//     1 <tle line 1>
//     2 <tle line 2>
// ---------------------------------------------------------------------------

void
SatTleReader::ParseTles(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        throw std::runtime_error("[SatTleReader] Cannot open: " + path);
    }

    // First line: "nPlanes nSatsPerPlane" — used for logging only
    std::string header;
    std::getline(f, header);

    int satIndex = 0;
    std::string line;

    while (std::getline(f, line))
    {
        // Skip empty lines
        if (line.empty()) { continue; }

        // This line is the satellite name
        SatTleEntry entry;
        entry.name     = line;
        entry.satIndex = satIndex++;

        // Read TLE line 1 (starts with '1 ')
        std::string tleStr1, tleStr2;
        if (!std::getline(f, tleStr1) || !std::getline(f, tleStr2))
        {
            throw std::runtime_error(
                "[SatTleReader] Truncated TLE for sat: " + entry.name);
        }

        // twoline2rv modifies its char[] arguments in-place — copy to fixed buffer
        std::strncpy(entry.tle1, tleStr1.c_str(), 129);
        std::strncpy(entry.tle2, tleStr2.c_str(), 129);
        entry.tle1[129] = '\0';
        entry.tle2[129] = '\0';

        // Initialise SGP4 record — mirrors Hypatia satellite.cc usage exactly
        double startmfe, stopmfe, deltamin;
        twoline2rv(entry.tle1, entry.tle2,
                   'c',    // typerun  : catalog mode
                   'e',    // typeinput: epoch-relative time
                   'i',    // opsmode  : improved
                   wgs72,
                   startmfe, stopmfe, deltamin,
                   entry.satrec);

        // Verify SGP4 initialised correctly (tsince=0 should return valid r/v)
        double r[3], v[3];
        bool ok = sgp4(wgs72, entry.satrec, 0.0, r, v);
        if (!ok || entry.satrec.error != 0)
        {
            std::cerr << "[SatTleReader] SGP4 init failed for " << entry.name
                      << " error=" << entry.satrec.error << "\n";
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
// We map sat_id → satIndex via sat_id - 1.
// ---------------------------------------------------------------------------

void
SatTleReader::ParseFwdConf(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
    {
        throw std::runtime_error("[SatTleReader] Cannot open: " + path);
    }

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#') { continue; }

        std::istringstream ss(line);
        int beamId, satId, freqColor, beamIdxOnSat;
        if (!(ss >> beamId >> satId >> freqColor >> beamIdxOnSat))
        {
            continue;
        }

        // fwdConf sat_id is 1-indexed → convert to 0-based satIndex
        const int satIdx = satId - 1;
        if (satIdx < 0 || satIdx >= static_cast<int>(m_sats.size()))
        {
            std::cerr << "[SatTleReader] fwdConf beam " << beamId
                      << " references out-of-range satId=" << satId << "\n";
            continue;
        }
        m_sats[satIdx].beamIds.push_back(beamId);
    }
}

} // namespace ns3
