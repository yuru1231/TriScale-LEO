/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-antenna-pattern-reader.cc
 *
 * Returns 0.0 dB unconditionally.  Beam gain is handled by the UPA model
 * inside ComputeFrameResults().  SNS3 GEO pattern files (lat/lon lookup)
 * are incompatible with Iridium LEO + arbitrary ROI.
 */

#include "sat-antenna-pattern-reader.h"

namespace ns3
{

double
SatAntennaPatternReader::GetGainDb(int /*beamId*/,
                                    double /*azimuthDeg*/,
                                    double /*elevationDeg*/) const
{
    return 0.0;
}

} // namespace ns3
