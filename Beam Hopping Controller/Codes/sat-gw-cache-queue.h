/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-gw-cache-queue.h
 *
 * SatGwCacheQueue — Gateway-side packet cache for inactive beam periods.
 *
 * Implements spec Section 7.3 (SatGwCacheQueue) and Layer2.md Section 7.3.
 * Implementation phase: Phase 3 (Layer2.md Section 10).
 *
 * Responsibilities (GW side):
 *   - Buffer packets destined for a beam while that beam is inactive
 *     (i.e. not in current slot's active beam set)
 *   - Dequeue all buffered packets when the beam's slot starts
 *   - Apply TAIL_DROP when per-beam queue exceeds MaxQueueSizePerBeam
 *   - Record per-packet queuing delay for SatBhMetrics
 *   - Emit drop events to SatBhMetrics via CacheDropCallback
 *
 * Queue capacity (spec Section 7.3):
 *   Q_max = R_peak × T_p = 500 Mbps × 503 ms ≈ 31.4 MB → set 40 MB
 *   DropPolicy = TAIL_DROP (drop newest packet when queue full)
 *
 * Connection model (non-invasive, Layer2.md Principle A/B):
 *   - Enqueue() is called from SatBhHelper trace sink connected to GwMac::Tx
 *     (or PhyTxEnd fallback) — not by modifying MAC internals
 *   - DequeueAll() is called from SatBhObc's BeamActivateCallback
 *   - The dequeued packets are re-injected into the MAC via a
 *     re-injection callback (to be confirmed during Hook feasibility check)
 *
 * Delay measurement:
 *   Each enqueued packet is tagged with its enqueue timestamp.
 *   On dequeue, delay = Simulator::Now() − enqueueTime.
 *   This delay is reported to SatBhMetrics via CacheDequeueCallback.
 */

#ifndef SAT_GW_CACHE_QUEUE_H
#define SAT_GW_CACHE_QUEUE_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"

#include <cstdint>
#include <deque>
#include <functional>
#include <map>

namespace ns3
{

// ── Per-packet cache entry ────────────────────────────────────────────────

struct CacheEntry
{
    Ptr<Packet> pkt;          ///< Buffered packet
    Time        enqueueTime;  ///< Simulator::Now() when enqueued (for delay measurement)
};

// ── Callbacks emitted by SatGwCacheQueue ─────────────────────────────────
//
// Connected by SatBhHelper to SatBhMetrics.

/// Called for each packet successfully dequeued at slot start
/// Signature: (satId, beamId, pkt, delayMs)
using CacheDequeueCallback =
    std::function<void(uint32_t satId, uint32_t beamId, Ptr<const Packet> pkt, double delayMs)>;

/// Called when a packet is tail-dropped (queue full)
/// Signature: (satId, beamId, pkt)
using CacheDropCallback =
    std::function<void(uint32_t satId, uint32_t beamId, Ptr<const Packet> pkt)>;

// ── SatGwCacheQueue ───────────────────────────────────────────────────────

class SatGwCacheQueue : public Object
{
  public:
    static TypeId GetTypeId();
    SatGwCacheQueue();

    // ── Configuration ─────────────────────────────────────────────────────

    /// Set the satellite ID this cache queue belongs to
    void SetSatId(uint32_t satId);
    uint32_t GetSatId() const { return m_satId; }

    // ── Packet operations ─────────────────────────────────────────────────

    /// Buffer a packet for the given beam.
    /// If the per-beam queue exceeds MaxQueueSizePerBeam, the packet
    /// is tail-dropped and CacheDropCallback is invoked.
    /// @param beamId  Destination beam ID
    /// @param pkt     Packet to buffer
    void Enqueue(uint32_t beamId, Ptr<Packet> pkt);

    /// Release all buffered packets for the given beam (called at slot start).
    /// For each dequeued packet: measures delay, invokes CacheDequeueCallback.
    /// After this call the beam's queue is empty.
    /// @param beamId  Beam whose slot just started
    void DequeueAll(uint32_t beamId);

    // ── Callback registration ─────────────────────────────────────────────

    /// Register the callback invoked for each successfully dequeued packet
    void SetDequeueCallback(CacheDequeueCallback cb);

    /// Register the callback invoked for each tail-dropped packet
    void SetDropCallback(CacheDropCallback cb);

    // ── Query ─────────────────────────────────────────────────────────────

    /// Current byte count buffered for the given beam
    uint64_t GetQueueBytes(uint32_t beamId) const;

    /// Total byte count across all beams
    uint64_t GetTotalQueueBytes() const;

    /// Number of packets currently buffered for the given beam
    uint32_t GetQueueDepth(uint32_t beamId) const;

  private:
    // ── Attribute accessor (double MB ↔ uint64_t bytes, required by MakeDoubleAccessor) ──
    void   SetMaxQueueSizeMB(double mb) { m_maxQueueBytesPerBeam = static_cast<uint64_t>(mb * 1.0e6); }
    double GetMaxQueueSizeMB() const    { return static_cast<double>(m_maxQueueBytesPerBeam) / 1.0e6; }

    uint32_t m_satId;               ///< Satellite this queue is attached to
    uint64_t m_maxQueueBytesPerBeam;///< Per-beam queue capacity (attr: MaxQueueSizeMB × 1e6)

    /// Per-beam FIFO queue: beamId → deque<CacheEntry>
    std::map<uint32_t, std::deque<CacheEntry>> m_queues;

    /// Per-beam current byte count (for fast capacity check)
    std::map<uint32_t, uint64_t> m_queueBytes;

    CacheDequeueCallback m_dequeueCb; ///< Emitted on successful dequeue
    CacheDropCallback    m_dropCb;    ///< Emitted on tail-drop
};

} // namespace ns3

#endif // SAT_GW_CACHE_QUEUE_H
