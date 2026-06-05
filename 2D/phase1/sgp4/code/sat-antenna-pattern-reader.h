/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-antenna-pattern-reader.h
 *
 * ns3::Object wrapper for per-beam antenna gain lookup.
 *
 * Design note:
 *   SNS3 GEO pattern files (SatAntennaGain72BeamsShifted) use absolute
 *   geographic lat/lon and cover only the GEO service area (Europe).
 *   They are incompatible with Iridium LEO + arbitrary ROI.
 *   All beam gain is therefore computed by the UPA model inside
 *   ComputeFrameResults(); this class always returns 0.0 dB offset.
 *
 * When real per-beam pattern files become available on VMware, override
 * GetGainDb() without changing any call sites in SatConstellationScanner.
 */

#ifndef SAT_ANTENNA_PATTERN_READER_H
#define SAT_ANTENNA_PATTERN_READER_H

#include "ns3/object.h"

namespace ns3
{

/**
 * SatAntennaPatternReader — ns3::Object placeholder for antenna gain lookup.
 *
 * Registered in the TypeId system so it can be instantiated via
 * ObjectFactory and configured through the Attribute framework.
 */
class SatAntennaPatternReader : public Object
{
public:
    /**
     * GetTypeId — ns3 TypeId registration.
     * Group: Satellite.
     */
    static TypeId GetTypeId();

    SatAntennaPatternReader()           = default;
    ~SatAntennaPatternReader() override = default;

    /**
     * GetGainDb — returns 0.0 dB (UPA gain applied separately).
     *
     * @param beamId      Beam index (0-based).
     * @param azimuthDeg  Off-axis azimuth angle (degrees).
     * @param elevationDeg Elevation angle (degrees).
     * @return            Antenna pattern offset gain (dB); always 0.0 in stub.
     */
    double GetGainDb(int beamId, double azimuthDeg, double elevationDeg) const;
};

} // namespace ns3

#endif // SAT_ANTENNA_PATTERN_READER_H
