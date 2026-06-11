/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-antenna-pattern-reader.h
 *
 * Interface for per-beam antenna gain lookup from SNS3 pattern files.
 *
 * Current state: STUB — always reports UPA fallback (HasRealData() == false).
 *
 * The SNS3 antenna pattern file format is:
 *   col1  col2  gain_dB
 * where col1/col2 are off-axis angles (degrees) at 0.25° resolution,
 * and gain_dB is the antenna gain in dB.  All values in the copied
 * constellation folder were NaN (files are Unix symlinks pointing to
 * additional-input/antennapatterns/ which was not copied).
 *
 * When the real files are available on VMware, populate m_tables in
 * the constructor and set m_hasData = true.  The rest of the codebase
 * (SatConstellationScanner) calls GetGainDb() without knowing which
 * backend is active.
 *
 * The UPA fallback delegates to the existing ComputeBeamGainDb() in
 * sat-multi-beam-channel — keeping all gain physics in one place.
 */

#ifndef SAT_ANTENNA_PATTERN_READER_H
#define SAT_ANTENNA_PATTERN_READER_H

#include <string>

namespace ns3
{

/**
 * SatAntennaPatternReader — placeholder for per-beam antenna gain lookup.
 *
 * Real SNS3 pattern files (SatAntennaGain72BeamsShifted) use absolute
 * geographic lat/lon as lookup keys and cover only the GEO satellite's
 * service area (Europe).  They cannot be used for Iridium LEO + arbitrary
 * ROI.  Beam gain is therefore always computed by the UPA model inside
 * ComputeFrameResults(); this class always returns 0.0 dB offset.
 */
class SatAntennaPatternReader
{
public:
    SatAntennaPatternReader() = default;

    /**
     * GetGainDb — returns 0.0 dB.
     * UPA gain is applied independently by ComputeFrameResults().
     */
    double GetGainDb(int beamId, double azimuthDeg, double elevationDeg) const;
};

} // namespace ns3

#endif // SAT_ANTENNA_PATTERN_READER_H
