#ifndef BEAM_HOPPING_MANAGER_H
#define BEAM_HOPPING_MANAGER_H

#include "ft-filter.h" // for ComputeElevationDeg utility

#include "ns3/nstime.h"
#include "ns3/node.h"
#include "ns3/object.h"
#include "ns3/satellite-sgp4-mobility-model.h"
#include "ns3/satellite-topology.h"
#include "ns3/simulator.h"
#include "ns3/singleton.h"
#include "ns3/vector.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ns3
{

// ── Cell definition ───────────────────────────────────────────────────────

struct CellDef
{
    uint32_t    id;
    double      latDeg;
    double      lonDeg;
    std::string name;
};

// ── Scheduled beam-switch event ───────────────────────────────────────────

struct BhEvent
{
    Time     t;
    uint32_t satId;
    uint32_t cellId;
};

// ── Traffic demand interface (reserved) ──────────────────────────────────
//
// Implement this interface to inject real traffic demand into BH scheduling.
// Default: UniformDemandProvider (equal weight for all cells).
//
class TrafficDemandProvider
{
  public:
    virtual ~TrafficDemandProvider()                                     = default;
    virtual double GetDemandMbps(uint32_t cellId, Time t) const = 0;
};

class UniformDemandProvider : public TrafficDemandProvider
{
  public:
    explicit UniformDemandProvider(double mbps = 1.0) : m_mbps(mbps) {}
    double GetDemandMbps(uint32_t /*cellId*/, Time /*t*/) const override { return m_mbps; }
  private:
    double m_mbps;
};

// ── SNS3 beam-switch injection callback ──────────────────────────────────
//
// Register this callback to bridge BH schedule events into the SNS3 API.
// Signature: (satId, cellId, simTime)
// Default stub: logs to stdout.
//
// TODO SNS3_BH_INJECT: replace stub with actual SNS3 beam switch API call.
//   Candidate API (to be confirmed):
//     m_orbDevs[satId]->SetActiveBeam(cellId);
//   or per SatBeamScheduler enablement.
//
using BhSwitchCallback =
    std::function<void(uint32_t satId, uint32_t cellId, Time t)>;

// ── BeamHoppingManager ────────────────────────────────────────────────────
//
// Layer 2: offline BH schedule computation + runtime NS3 event injection.
//
// Usage:
//   1. AddCell(id, lat, lon)  for each cell in target region
//   2. SetDemandProvider(provider)   [optional, default = uniform]
//   3. SetBhSwitchCallback(cb)       [optional, default = stdout log]
//   4. ComputeBhSchedule(numSlots, slotIntervalSec)  ← offline
//   5. ScheduleBhUpdates()           ← before Simulator::Run()
//
// Layer 3 hook:
//   GetCurrentCell(satId) → current serving cellId for QoS context
//
class BeamHoppingManager : public Object
{
  public:
    static TypeId GetTypeId();
    BeamHoppingManager();

    // Configuration
    void AddCell(uint32_t id, double latDeg, double lonDeg, const std::string& name = "");
    void SetElevationThreshold(double deg);
    void SetDemandProvider(std::shared_ptr<TrafficDemandProvider> provider);
    void SetBhSwitchCallback(BhSwitchCallback cb);

    // Offline: compute full BH schedule across all time slots
    void ComputeBhSchedule(uint32_t numSlots, double slotIntervalSec);

    // Runtime: register all BhEvents with NS3 Simulator
    void ScheduleBhUpdates();

    // Layer 3 hook: returns current serving cellId for a satellite
    uint32_t GetCurrentCell(uint32_t satId) const;

    // Query
    const std::vector<BhEvent>& GetSchedule() const { return m_schedule; }

    void PrintSchedule() const;

  private:
    // Compute cells visible from satellite at given ECEF position
    std::vector<uint32_t> ComputeVisibleCells(const Vector& satEcef) const;

    // Called by NS3 scheduler at each BH event time
    void ApplyBhEvent(uint32_t satId, uint32_t cellId);

    // Load NS3 orbiter node pointers (same pattern as IslRoutingManager)
    void LoadOrbiterNodes();

    // Config
    std::vector<CellDef>  m_cells;
    double                m_elevThreshDeg{5.0};
    double                m_superframeSec{0.25}; // DVB-S2 superframe ~250 ms
    uint32_t              m_numSatellites{66};

    // Orbiter nodes (for SGP4 position queries)
    std::vector<Ptr<Node>> m_orbNodes;

    // Schedule (sorted by Time)
    std::vector<BhEvent>           m_schedule;
    std::map<uint32_t, uint32_t>   m_currentCell; // satId → cellId

    // Pluggable providers / callbacks
    std::shared_ptr<TrafficDemandProvider> m_demandProvider;
    BhSwitchCallback                       m_switchCallback;
};

} // namespace ns3
#endif // BEAM_HOPPING_MANAGER_H
