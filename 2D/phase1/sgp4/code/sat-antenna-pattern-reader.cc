/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * sat-antenna-pattern-reader.cc
 *
 * ns3::Object implementation of SatAntennaPatternReader.
 * Returns 0.0 dB unconditionally — beam gain is handled by the UPA model
 * inside ComputeFrameResults().
 *
 * SNS3 GEO pattern files (lat/lon lookup) are incompatible with
 * Iridium LEO + arbitrary ROI; see header for details.
 */

#include "sat-antenna-pattern-reader.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatAntennaPatternReader");
NS_OBJECT_ENSURE_REGISTERED(SatAntennaPatternReader);

// ---------------------------------------------------------------------------
// TypeId registration
// ---------------------------------------------------------------------------

TypeId
SatAntennaPatternReader::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatAntennaPatternReader")
            .SetParent<Object>()
            .SetGroupName("Satellite")
            .AddConstructor<SatAntennaPatternReader>();
    return tid;
}

// ---------------------------------------------------------------------------
// GetGainDb
// ---------------------------------------------------------------------------

double
SatAntennaPatternReader::GetGainDb(int    beamId,
                                    double azimuthDeg,
                                    double elevationDeg) const
{
    NS_LOG_DEBUG("GetGainDb beamId=" << beamId
                 << " az=" << azimuthDeg << " el=" << elevationDeg
                 << " -> 0.0 dB (UPA fallback active)");
    return 0.0;
}

} // namespace ns3
