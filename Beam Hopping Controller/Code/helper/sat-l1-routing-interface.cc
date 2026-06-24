/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-l1-routing-interface.cc
 *
 * SatL1RoutingInterface implementation.
 * See sat-l1-routing-interface.h for full documentation.
 */

#include "sat-l1-routing-interface.h"

#include "ns3/boolean.h"
#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatL1RoutingInterface");
NS_OBJECT_ENSURE_REGISTERED(SatL1RoutingInterface);

TypeId
SatL1RoutingInterface::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatL1RoutingInterface")
            .SetParent<Object>()
            .AddConstructor<SatL1RoutingInterface>()
            .AddAttribute("Enabled",
                          "Enable live L1 routing data (false = stub mode)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&SatL1RoutingInterface::m_enabled),
                          MakeBooleanChecker());
    return tid;
}

SatL1RoutingInterface::SatL1RoutingInterface()
    : m_enabled(false)
{
}

std::set<uint32_t>
SatL1RoutingInterface::GetActivePathBeams(uint32_t srcGwId, uint32_t dstGwId) const
{
    if (!m_enabled)
    {
        NS_LOG_DEBUG("SatL1RoutingInterface::GetActivePathBeams [stub] src="
                     << srcGwId << " dst=" << dstGwId << " → empty");
        return {};
    }
    // Phase E: walk ISL routing table, collect beamIds on active forwarding paths
    NS_LOG_WARN("SatL1RoutingInterface: Enabled=true but ISL graph not yet wired");
    return {};
}

double
SatL1RoutingInterface::GetLinkLoad(uint32_t satId, uint32_t beamId) const
{
    if (!m_enabled)
        return 0.0;
    // Phase E: query ISL load monitor
    NS_LOG_WARN("SatL1RoutingInterface: GetLinkLoad called but ISL load monitor not wired"
                << " sat=" << satId << " beam=" << beamId);
    return 0.0;
}

void
SatL1RoutingInterface::RegisterBeamStateCallback(BeamStateCallback cb)
{
    m_beamStateCb = cb;
    NS_LOG_INFO("SatL1RoutingInterface: BeamStateCallback registered");
}

void
SatL1RoutingInterface::NotifyBeamState(uint32_t satId, uint32_t beamId, bool isActive)
{
    if (!m_beamStateCb)
        return;
    m_beamStateCb(satId, beamId, isActive);
    NS_LOG_DEBUG("SatL1RoutingInterface::NotifyBeamState sat=" << satId
                 << " beam=" << beamId << " active=" << isActive);
}

} // namespace ns3
