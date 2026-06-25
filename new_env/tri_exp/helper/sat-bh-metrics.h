/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef SAT_BH_METRICS_H
#define SAT_BH_METRICS_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <string>

namespace ns3
{

struct BeamKey
{
    uint32_t satId;
    uint32_t beamId;

    bool operator<(const BeamKey& other) const
    {
        if (satId != other.satId)
        {
            return satId < other.satId;
        }
        return beamId < other.beamId;
    }
};

struct BhBeamKpi
{
    uint64_t rxBytes = 0;
    double totalDelayMs = 0.0;
    double maxDelayMs = 0.0;
    uint32_t pktCount = 0;
    uint32_t droppedPkts = 0;
    uint32_t totalOfferedPkts = 0;
    double dwellMs = 0.0;
    double slotAllocMs = 0.0;
    double slotUsedMs = 0.0;

    double ThroughputMbps(double windowSec) const;
    double AvgDelayMs() const;
    double DropRatePct() const;
    double SlotUtilPct() const;
    void Reset();
};

class SatBhMetrics : public Object
{
  public:
    static TypeId GetTypeId();

    SatBhMetrics();
    ~SatBhMetrics() override;

    void SetOutputFile(const std::string& path);
    std::string GetOutputFile() const;

    void SetReportInterval(Time interval);
    Time GetReportInterval() const;

    void SetWarmUpTime(Time warmUp);
    Time GetWarmUpTime() const;

    // wrappers for ns-3 Double Attribute system
    void SetReportIntervalMs(double ms);
    double GetReportIntervalMs() const;

    void SetWarmUpTimeSec(double sec);
    double GetWarmUpTimeSec() const;

    void OnPacketReceived(uint32_t satId, uint32_t beamId,
                          Ptr<const Packet> pkt, double delayMs);
    void OnPacketDropped(uint32_t satId, uint32_t beamId,
                         Ptr<const Packet> pkt);
    void OnSlotActivated(uint32_t satId, uint32_t beamId, Time duration);
    void OnSlotDeactivated(uint32_t satId, uint32_t beamId, Time usedDuration);

    void FlushMetrics();
    void FinalFlush();
    void ScheduleNextFlush();

  private:
    BhBeamKpi& GetOrCreate(uint32_t satId, uint32_t beamId);
    void EnsureFileOpen();
    void FlushAndReschedule();
    double ComputeJainFairnessIndex() const;

  private:
    std::map<BeamKey, BhBeamKpi> m_kpiTable;

    bool m_headerWritten;
    Time m_reportInterval;
    Time m_warmUpTime;
    Time m_windowStart;
    // Rate-limit gate: FlushMetrics only writes when Simulator::Now() >= m_nextFlushTime.
    // Updated to Now() + T_p after each write, so at most one write per T_p regardless
    // of how many callers (timer, OBC slot callback, RunDynamicBstpCycle) trigger it.
    Time m_nextFlushTime;
    // Last in-simulation slot event; used to detect destructor calls after
    // Simulator::Destroy() has reset Simulator::Now() to zero.
    Time m_lastEventTime;

    std::string m_outputPath;
    std::ofstream m_csvFile;
};

} // namespace ns3

#endif // SAT_BH_METRICS_H
