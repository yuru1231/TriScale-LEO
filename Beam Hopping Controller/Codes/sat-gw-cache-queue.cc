/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-gw-cache-queue.cc
 *
 * SatGwCacheQueue — STUB implementation (Phase 3, not yet active).
 *
 * Enqueue() and DequeueAll() are no-ops in this stub.
 * Interfaces are final; only this .cc changes when Phase 3 is implemented.
 *
 * Implementation checklist (Layer2.md Step 5):
 *   [ ] Enqueue: compute packet size, check m_maxQueueBytesPerBeam,
 *                tail-drop + invoke m_dropCb if full,
 *                else push CacheEntry{pkt, Now()} into queue
 *   [ ] DequeueAll: for each CacheEntry: compute delay, invoke m_dequeueCb,
 *                   clear queue and m_queueBytes
 *   [ ] GetQueueBytes / GetTotalQueueBytes / GetQueueDepth live accounting
 *   [ ] Connect to SatBhObc::BeamActivateCallback via SatBhHelper
 *   [ ] Connect Enqueue as sink to GwMac::Tx trace (or PhyTxEnd fallback)
 */

#include "sat-gw-cache-queue.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatGwCacheQueue");
NS_OBJECT_ENSURE_REGISTERED(SatGwCacheQueue);

TypeId
SatGwCacheQueue::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatGwCacheQueue")
            .SetParent<Object>()
            .AddConstructor<SatGwCacheQueue>()
            .AddAttribute("SatId",
                          "i: satellite index (formal model parameter i)",
                          UintegerValue(0),
                          MakeUintegerAccessor(&SatGwCacheQueue::m_i),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("MaxQueueSizeMB",
                          "Per-beam maximum queue size in MB "
                          "(spec: Q_max = R_peak × T_p ≈ 31.4 MB → default 40 MB)",
                          DoubleValue(40.0),
                          MakeDoubleAccessor(&SatGwCacheQueue::SetMaxQueueSizeMB,
                                             &SatGwCacheQueue::GetMaxQueueSizeMB),
                          MakeDoubleChecker<double>(1.0, 1000.0));
    return tid;
}

SatGwCacheQueue::SatGwCacheQueue()
    : m_i(0),
      m_maxQueueBytesPerBeam(static_cast<uint64_t>(40e6)) // 40 MB default
{
    NS_LOG_INFO("SatGwCacheQueue: constructed (Phase 3 stub — not yet implemented)");
}

// ── Configuration ─────────────────────────────────────────────────────────

void
SatGwCacheQueue::SetSatId(uint32_t satId)
{
    m_i = satId;
}

// ── Packet operations ─────────────────────────────────────────────────────

void
SatGwCacheQueue::Enqueue(uint32_t beamId, Ptr<Packet> pkt)
{
    NS_ASSERT_MSG(pkt, "SatGwCacheQueue::Enqueue: null packet");

    const uint64_t pktSize = static_cast<uint64_t>(pkt->GetSize());

    // Tail-drop when per-beam queue is full
    if (m_queueBytes[beamId] + pktSize > m_maxQueueBytesPerBeam)
    {
        NS_LOG_WARN("SatGwCacheQueue::Enqueue beam=" << beamId
                    << " TAIL_DROP size=" << pktSize
                    << " queueBytes=" << m_queueBytes[beamId]
                    << " limit=" << m_maxQueueBytesPerBeam);
        if (m_dropCb)
            m_dropCb(m_i, beamId, pkt);
        return;
    }

    m_queues[beamId].push_back(CacheEntry{pkt, Simulator::Now()});
    m_queueBytes[beamId] += pktSize;

    NS_LOG_DEBUG("SatGwCacheQueue::Enqueue beam=" << beamId
                 << " size=" << pktSize
                 << " depth=" << m_queues[beamId].size()
                 << " queueBytes=" << m_queueBytes[beamId]);
}

void
SatGwCacheQueue::DequeueAll(uint32_t beamId)
{
    auto it = m_queues.find(beamId);
    if (it == m_queues.end() || it->second.empty())
    {
        NS_LOG_DEBUG("SatGwCacheQueue::DequeueAll beam=" << beamId << " — queue empty, nothing to release");
        return;
    }

    auto& queue = it->second;
    NS_LOG_INFO("SatGwCacheQueue::DequeueAll beam=" << beamId
                << " releasing " << queue.size() << " packets");

    for (auto& entry : queue)
    {
        const double delayMs = (Simulator::Now() - entry.enqueueTime).GetMilliSeconds();
        NS_LOG_DEBUG("  pkt size=" << entry.pkt->GetSize()
                     << " delayMs=" << delayMs);
        if (m_dequeueCb)
            m_dequeueCb(m_i, beamId, entry.pkt, delayMs);
    }

    // Clear queue and reset byte counter
    queue.clear();
    m_queueBytes[beamId] = 0;
}

// ── Callback registration ─────────────────────────────────────────────────

void
SatGwCacheQueue::SetDequeueCallback(CacheDequeueCallback cb)
{
    m_dequeueCb = cb;
}

void
SatGwCacheQueue::SetDropCallback(CacheDropCallback cb)
{
    m_dropCb = cb;
}

// ── Query ─────────────────────────────────────────────────────────────────

uint64_t
SatGwCacheQueue::GetQueueBytes(uint32_t beamId) const
{
    auto it = m_queueBytes.find(beamId);
    return (it != m_queueBytes.end()) ? it->second : 0;
}

uint64_t
SatGwCacheQueue::GetTotalQueueBytes() const
{
    uint64_t total = 0;
    for (const auto& [bid, bytes] : m_queueBytes)
        total += bytes;
    return total;
}

uint32_t
SatGwCacheQueue::GetQueueDepth(uint32_t beamId) const
{
    auto it = m_queues.find(beamId);
    if (it == m_queues.end()) return 0;
    return static_cast<uint32_t>(it->second.size());
}

} // namespace ns3
