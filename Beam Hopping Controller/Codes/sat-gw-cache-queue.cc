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
    // TODO Phase 3:
    //   1. Check m_queueBytes[beamId] + pkt->GetSize() ≤ m_maxQueueBytesPerBeam
    //   2. If overflow: m_dropCb(m_i, beamId, pkt); return
    //   3. Else: m_queues[beamId].push_back({pkt, Simulator::Now()})
    //            m_queueBytes[beamId] += pkt->GetSize()
    NS_LOG_DEBUG("SatGwCacheQueue::Enqueue beam=" << beamId
                 << " size=" << (pkt ? pkt->GetSize() : 0)
                 << " [STUB — packet not buffered]");
}

void
SatGwCacheQueue::DequeueAll(uint32_t beamId)
{
    // TODO Phase 3:
    //   for each CacheEntry in m_queues[beamId]:
    //     double delayMs = (Now() - entry.enqueueTime).GetMilliSeconds()
    //     m_dequeueCb(m_i, beamId, entry.pkt, delayMs)
    //   m_queues[beamId].clear()
    //   m_queueBytes[beamId] = 0
    NS_LOG_DEBUG("SatGwCacheQueue::DequeueAll beam=" << beamId
                 << " [STUB — no packets to release]");
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
