/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-metrics.cc
 *
 * Implementation of SatBhMetrics (passive KPI collector).
 * See sat-bh-metrics.h for full documentation.
 */

#include "sat-bh-metrics.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatBhMetrics");
NS_OBJECT_ENSURE_REGISTERED(SatBhMetrics);

// ── BhBeamKpi ─────────────────────────────────────────────────────────────

double
BhBeamKpi::ThroughputMbps(double windowSec) const
{
    if (windowSec <= 0.0)
    {
        return 0.0;
    }

    // bytes × 8 bits / (window_s × 1e6 bits/Mbit)
    return static_cast<double>(rxBytes) * 8.0 / (windowSec * 1.0e6);
}

double
BhBeamKpi::AvgDelayMs() const
{
    if (pktCount == 0)
    {
        return 0.0;
    }
    return totalDelayMs / static_cast<double>(pktCount);
}

double
BhBeamKpi::DropRatePct() const
{
    if (totalOfferedPkts == 0)
    {
        return 0.0;
    }

    return 100.0 * static_cast<double>(droppedPkts) /
           static_cast<double>(totalOfferedPkts);
}

double
BhBeamKpi::SlotUtilPct() const
{
    if (slotAllocMs <= 0.0)
    {
        return 0.0;
    }

    // Clamp to [0, 100] to guard against floating-point rounding errors
    double raw = 100.0 * slotUsedMs / slotAllocMs;
    return std::min(100.0, std::max(0.0, raw));
}

void
BhBeamKpi::Reset()
{
    rxBytes = 0;
    totalDelayMs = 0.0;
    maxDelayMs = 0.0;
    pktCount = 0;
    droppedPkts = 0;
    totalOfferedPkts = 0;
    dwellMs = 0.0;
    slotAllocMs = 0.0;
    slotUsedMs = 0.0;
}

// ── SatBhMetrics — TypeId ─────────────────────────────────────────────────

TypeId
SatBhMetrics::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatBhMetrics")
            .SetParent<Object>()
            .AddConstructor<SatBhMetrics>()
            .AddAttribute("ReportIntervalMs",
                          "KPI flush interval in milliseconds (default = T_p = 503 ms)",
                          DoubleValue(503.0),
                          MakeDoubleAccessor(&SatBhMetrics::SetReportIntervalMs,
                                             &SatBhMetrics::GetReportIntervalMs),
                          MakeDoubleChecker<double>(1.0, 60000.0))
            .AddAttribute("WarmUpTimeSec",
                          "Warm-up duration [s] before KPI recording begins "
                          "(spec Section 9: discard first 10 s)",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&SatBhMetrics::SetWarmUpTimeSec,
                                             &SatBhMetrics::GetWarmUpTimeSec),
                          MakeDoubleChecker<double>(0.0, 3600.0))
            .AddAttribute("OutputFile",
                          "CSV file path for BH KPI output",
                          StringValue("bh-metrics.csv"),
                          MakeStringAccessor(&SatBhMetrics::SetOutputFile,
                                             &SatBhMetrics::GetOutputFile),
                          MakeStringChecker());
    return tid;
}

// ── SatBhMetrics — constructor / destructor ───────────────────────────────

SatBhMetrics::SatBhMetrics()
    : m_headerWritten(false),
      m_reportInterval(MilliSeconds(503.0)), // T_p
      m_warmUpTime(Seconds(10.0)),           // spec Section 9
      m_windowStart(Seconds(0.0)),
      m_outputPath("bh-metrics.csv")
{
}

SatBhMetrics::~SatBhMetrics()
{
    FinalFlush();
    if (m_csvFile.is_open())
    {
        m_csvFile.close();
    }
}

// ── Configuration ─────────────────────────────────────────────────────────

void
SatBhMetrics::SetOutputFile(const std::string& path)
{
    m_outputPath = path;
}

std::string
SatBhMetrics::GetOutputFile() const
{
    return m_outputPath;
}

void
SatBhMetrics::SetReportInterval(Time interval)
{
    NS_ASSERT_MSG(interval.IsPositive(),
                  "SatBhMetrics::SetReportInterval: interval must be > 0");
    m_reportInterval = interval;
}

Time
SatBhMetrics::GetReportInterval() const
{
    return m_reportInterval;
}

void
SatBhMetrics::SetWarmUpTime(Time warmUp)
{
    m_warmUpTime = warmUp;
}

Time
SatBhMetrics::GetWarmUpTime() const
{
    return m_warmUpTime;
}

void
SatBhMetrics::SetReportIntervalMs(double ms)
{
    SetReportInterval(MilliSeconds(ms));
}

double
SatBhMetrics::GetReportIntervalMs() const
{
    return GetReportInterval().GetMilliSeconds();
}

void
SatBhMetrics::SetWarmUpTimeSec(double sec)
{
    SetWarmUpTime(Seconds(sec));
}

double
SatBhMetrics::GetWarmUpTimeSec() const
{
    return GetWarmUpTime().GetSeconds();
}

// ── Trace sinks ───────────────────────────────────────────────────────────

void
SatBhMetrics::OnPacketReceived(uint32_t satId, uint32_t beamId,
                               Ptr<const Packet> pkt, double delayMs)
{
    if (Simulator::Now() < m_warmUpTime)
    {
        return;
    }

    BhBeamKpi& kpi = GetOrCreate(satId, beamId);

    if (pkt)
    {
        kpi.rxBytes += pkt->GetSize();
    }

    kpi.totalDelayMs += delayMs;
    kpi.maxDelayMs = std::max(kpi.maxDelayMs, delayMs);
    kpi.pktCount += 1;
    kpi.totalOfferedPkts += 1;

    NS_LOG_DEBUG("SatBhMetrics::OnPacketReceived"
                 << " sat=" << satId
                 << " beam=" << beamId
                 << " size=" << (pkt ? pkt->GetSize() : 0)
                 << " delay=" << delayMs << " ms");
}

void
SatBhMetrics::OnPacketDropped(uint32_t satId, uint32_t beamId,
                              Ptr<const Packet> /*pkt*/)
{
    if (Simulator::Now() < m_warmUpTime)
    {
        return;
    }

    BhBeamKpi& kpi = GetOrCreate(satId, beamId);
    kpi.droppedPkts += 1;
    kpi.totalOfferedPkts += 1;

    NS_LOG_DEBUG("SatBhMetrics::OnPacketDropped"
                 << " sat=" << satId
                 << " beam=" << beamId);
}

void
SatBhMetrics::OnSlotActivated(uint32_t satId, uint32_t beamId, Time duration)
{
    if (Simulator::Now() < m_warmUpTime)
    {
        return;
    }

    BhBeamKpi& kpi = GetOrCreate(satId, beamId);
    kpi.slotAllocMs += duration.GetMilliSeconds();
    kpi.dwellMs += duration.GetMilliSeconds();

    NS_LOG_DEBUG("SatBhMetrics::OnSlotActivated"
                 << " sat=" << satId
                 << " beam=" << beamId
                 << " dur=" << duration.GetMilliSeconds() << " ms");
}

void
SatBhMetrics::OnSlotDeactivated(uint32_t satId, uint32_t beamId, Time usedDuration)
{
    if (Simulator::Now() < m_warmUpTime)
    {
        return;
    }

    BhBeamKpi& kpi = GetOrCreate(satId, beamId);
    kpi.slotUsedMs += usedDuration.GetMilliSeconds();

    NS_LOG_DEBUG("SatBhMetrics::OnSlotDeactivated"
                 << " sat=" << satId
                 << " beam=" << beamId
                 << " used=" << usedDuration.GetMilliSeconds() << " ms");
}

// ── Output ────────────────────────────────────────────────────────────────

void
SatBhMetrics::FlushMetrics()
{
    EnsureFileOpen();

    const double nowSec = Simulator::Now().GetSeconds();
    const double windowSec = (Simulator::Now() - m_windowStart).GetSeconds();
    const double jfi = ComputeJainFairnessIndex();

    for (auto& entry : m_kpiTable)
    {
        const BeamKey& key = entry.first;
        BhBeamKpi& kpi = entry.second;

        m_csvFile << std::fixed << std::setprecision(6)
                  << nowSec << ","
                  << key.satId << ","
                  << key.beamId << ","
                  << kpi.ThroughputMbps(windowSec) << ","
                  << kpi.AvgDelayMs() << ","
                  << kpi.maxDelayMs << ","
                  << kpi.dwellMs << ","
                  << kpi.SlotUtilPct() << ","
                  << kpi.DropRatePct() << ","
                  << jfi << "\n";

        NS_LOG_INFO("SatBhMetrics::FlushMetrics"
                    << " sat=" << key.satId
                    << " beam=" << key.beamId
                    << " throughput=" << kpi.ThroughputMbps(windowSec)
                    << " avgDelayMs=" << kpi.AvgDelayMs()
                    << " maxDelayMs=" << kpi.maxDelayMs
                    << " dwellMs=" << kpi.dwellMs
                    << " slotUtilPct=" << kpi.SlotUtilPct()
                    << " dropRatePct=" << kpi.DropRatePct()
                    << " JFI=" << std::fixed << std::setprecision(4) << jfi);

        kpi.Reset();
    }

    m_csvFile.flush();
    m_windowStart = Simulator::Now();
}

double
SatBhMetrics::ComputeJainFairnessIndex() const
{
    double sumX = 0.0;
    double sumXsq = 0.0;
    uint32_t n = 0;

    const double windowSec = (Simulator::Now() - m_windowStart).GetSeconds();

    for (const auto& entry : m_kpiTable)
    {
        const BhBeamKpi& kpi = entry.second;
        double xi = kpi.ThroughputMbps(windowSec);
        sumX += xi;
        sumXsq += xi * xi;
        ++n;
    }

    if (n <= 1 || sumXsq == 0.0)
    {
        return 1.0;
    }

    return (sumX * sumX) / (static_cast<double>(n) * sumXsq);
}

void
SatBhMetrics::FinalFlush()
{
    if (!m_kpiTable.empty())
    {
        FlushMetrics();
    }
}

void
SatBhMetrics::ScheduleNextFlush()
{
    Simulator::Schedule(m_reportInterval,
                        &SatBhMetrics::FlushAndReschedule,
                        this);
}

// ── Private helpers ───────────────────────────────────────────────────────

BhBeamKpi&
SatBhMetrics::GetOrCreate(uint32_t satId, uint32_t beamId)
{
    return m_kpiTable[BeamKey{satId, beamId}];
}

void
SatBhMetrics::EnsureFileOpen()
{
    if (!m_csvFile.is_open())
    {
        m_csvFile.open(m_outputPath, std::ios::out | std::ios::app);
        NS_ASSERT_MSG(m_csvFile.is_open(),
                      "SatBhMetrics: cannot open output file: " << m_outputPath);
    }

    if (!m_headerWritten)
    {
        m_csvFile << "time_s,sat_id,beam_id,"
                     "throughput_mbps,avg_delay_ms,max_delay_ms,"
                     "dwell_time_ms,slot_util_pct,drop_rate_pct,"
                     "jain_fairness_index\n";
        m_headerWritten = true;
    }
}

void
SatBhMetrics::FlushAndReschedule()
{
    FlushMetrics();

    Simulator::Schedule(m_reportInterval,
                        &SatBhMetrics::FlushAndReschedule,
                        this);
}

} // namespace ns3