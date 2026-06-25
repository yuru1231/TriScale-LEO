/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-l1-routing-interface.h
 *
 * SatL1RoutingInterface — stub bridge between Layer 2 BH controller and
 * Layer 1 ISL routing (Layer2.md §5.7).
 *
 * Current status: STUB (Phase B). Live integration deferred to Phase E after
 * ISL routing (Layer 1) is complete and hook paths are confirmed.
 *
 * Layer 1 → Layer 2:
 *   GetActivePathBeams() tells the BH scheduler which beams are currently
 *   forwarding ISL-routed traffic so they are not parked mid-stream.
 *
 * Layer 2 → Layer 1:
 *   NotifyBeamState() fires BeamStateCallback when a beam goes on/off,
 *   allowing the L1 ISL cost model to update routing tables immediately.
 *
 * Invariant when Enabled == false (current stub behaviour):
 *   GetActivePathBeams() returns empty set  (no beam protection)
 *   GetLinkLoad()        returns 0.0        (no load data)
 *   NotifyBeamState()    is a no-op if no callback is registered
 */

#ifndef SAT_L1_ROUTING_INTERFACE_H
#define SAT_L1_ROUTING_INTERFACE_H

#include "ns3/object.h"

#include <cstdint>
#include <functional>
#include <set>

namespace ns3
{

// Callback fired when a beam transitions ON or OFF.
// Signature: (satId, beamId, isActive)
// Registered by Layer 1 ISL cost updater via RegisterBeamStateCallback().
using BeamStateCallback = std::function<void(uint32_t satId, uint32_t beamId, bool isActive)>;

class SatL1RoutingInterface : public Object
{
  public:
    static TypeId GetTypeId();
    SatL1RoutingInterface();

    // ── Layer 1 → Layer 2 queries ─────────────────────────────────────────

    /// Return beam IDs currently on the active ISL path between two gateways.
    /// Stub returns empty set. Phase E: queries the ISL routing table.
    std::set<uint32_t> GetActivePathBeams(uint32_t srcGwId, uint32_t dstGwId) const;

    /// Return current traffic load on a feeder/ISL link in Mbps.
    /// Stub returns 0.0. Phase E: queries ISL load monitor.
    double GetLinkLoad(uint32_t satId, uint32_t beamId) const;

    // ── Layer 2 → Layer 1 notifications ──────────────────────────────────

    /// Register callback invoked when a beam changes state.
    /// Called by SatBhHelper in Phase E to connect L1 cost updater.
    void RegisterBeamStateCallback(BeamStateCallback cb);

    /// Fire the registered callback (called by SatResourceManager after
    /// ApplyAssociation changes beam ON/OFF state).
    void NotifyBeamState(uint32_t satId, uint32_t beamId, bool isActive);

    bool IsEnabled() const { return m_enabled; }

  private:
    bool             m_enabled;      ///< false = stub; true = live L1 link
    BeamStateCallback m_beamStateCb; ///< Registered L1 state-change sink
};

} // namespace ns3

#endif // SAT_L1_ROUTING_INTERFACE_H
