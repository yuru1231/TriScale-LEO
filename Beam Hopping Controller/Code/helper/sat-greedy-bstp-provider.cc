/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-greedy-bstp-provider.cc
 *
 * SatGreedyBstpProvider implementation.
 *
 * See sat-greedy-bstp-provider.h for design rationale and parameter docs.
 */

#include "sat-greedy-bstp-provider.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <set>
#include <unordered_set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatGreedyBstpProvider");
NS_OBJECT_ENSURE_REGISTERED(SatGreedyBstpProvider);

// ── TypeId ─────────────────────────────────────────────────────────────────

TypeId
SatGreedyBstpProvider::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatGreedyBstpProvider")
            .SetParent<SatDynamicBstpProvider>()
            .AddConstructor<SatGreedyBstpProvider>()
            .AddAttribute("MaxActiveBeams",
                          "Maximum number of simultaneously active beams per BHTP window (K).",
                          UintegerValue(2),
                          MakeUintegerAccessor(&SatGreedyBstpProvider::m_maxActiveBeams),
                          MakeUintegerChecker<uint32_t>(1, 64))
            .AddAttribute("BacklogWeight",
                          "Scoring coefficient for beam demand [kbps]. "
                          "Higher → prioritise high-throughput beams.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&SatGreedyBstpProvider::m_backlogWeight),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("FairnessWeight",
                          "Scoring coefficient for time-since-last-served [s]. "
                          "Higher → prioritise long-idle beams.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&SatGreedyBstpProvider::m_fairnessWeight),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ValidityInSuperframes",
                          "Number of superframes the returned Conf remains valid (≥ 1).",
                          UintegerValue(1),
                          MakeUintegerAccessor(&SatGreedyBstpProvider::m_validityInSuperframes),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("StarvationThreshold",
                          "Number of consecutive skipped cycles before a beam is "
                          "unconditionally forced into the active set.",
                          UintegerValue(5),
                          MakeUintegerAccessor(&SatGreedyBstpProvider::m_starvationThreshold),
                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

// ── Constructor ────────────────────────────────────────────────────────────

SatGreedyBstpProvider::SatGreedyBstpProvider()
    : m_rrPointer(0),
      m_cycleCount(0),
      m_maxActiveBeams(2),
      m_backlogWeight(1.0),
      m_fairnessWeight(0.5),
      m_validityInSuperframes(1),
      m_starvationThreshold(5)
{
}

// ── TypeId for abstract base ───────────────────────────────────────────────

TypeId
SatDynamicBstpProvider::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatDynamicBstpProvider")
            .SetParent<Object>();
    return tid;
}

// ── Beam registry ──────────────────────────────────────────────────────────

void
SatGreedyBstpProvider::AddEnabledBeamInfo(uint32_t beamId,
                                           uint32_t userFreqId,
                                           uint32_t feederFreqId,
                                           uint32_t gwId)
{
    if (m_beams.count(beamId))
    {
        NS_LOG_WARN("SatGreedyBstpProvider::AddEnabledBeamInfo: beamId="
                    << beamId << " already registered — ignored");
        return;
    }

    BeamState s;
    s.info.beamId       = beamId;
    s.info.userFreqId   = userFreqId;
    s.info.feederFreqId = feederFreqId;
    s.info.gwId         = gwId;
    s.lastServedTime    = Seconds(0.0); // not yet served
    m_beams[beamId]     = s;

    NS_LOG_DEBUG("SatGreedyBstpProvider: registered beam=" << beamId
                 << " gw=" << gwId << " feederFreq=" << feederFreqId);
}

// ── Demand update ──────────────────────────────────────────────────────────

void
SatGreedyBstpProvider::UpdateBeamDemand(uint32_t beamId, double demandKbps)
{
    auto it = m_beams.find(beamId);
    if (it == m_beams.end())
    {
        NS_LOG_DEBUG("SatGreedyBstpProvider::UpdateBeamDemand: unknown beamId=" << beamId);
        return;
    }
    it->second.demandKbps = demandKbps;
}

void
SatGreedyBstpProvider::UpdateBeamBacklog(uint32_t beamId, uint64_t queueBytes)
{
    auto it = m_beams.find(beamId);
    if (it == m_beams.end())
        return;
    it->second.backlogBytes = queueBytes;
}

// ── Score helper ───────────────────────────────────────────────────────────

double
SatGreedyBstpProvider::ComputeScore(const BeamState& s, Time now) const
{
    // Age in seconds since last service (0 → never skipped)
    double ageSec = (now - s.lastServedTime).GetSeconds();
    if (ageSec < 0.0) ageSec = 0.0;

    return m_backlogWeight  * s.demandKbps
         + m_fairnessWeight * ageSec;
}

// ── Core scheduling ────────────────────────────────────────────────────────

SatDynamicBstpProvider::Conf
SatGreedyBstpProvider::GetNextConf(Time now)
{
    m_cycleCount++;

    if (m_beams.empty())
    {
        NS_LOG_WARN("SatGreedyBstpProvider::GetNextConf: no beams registered — returning empty Conf");
        Conf c;
        c.validityInSuperframes = m_validityInSuperframes;
        return c;
    }

    // ── Check if any beam has non-zero demand ─────────────────────────────
    bool anyDemand = false;
    for (const auto& kv : m_beams)
        if (kv.second.demandKbps > 0.0) { anyDemand = true; break; }

    if (!anyDemand)
    {
        // Fallback: round-robin until real demand arrives (warm-up / pre-Phase-F)
        NS_LOG_DEBUG("SatGreedyBstpProvider: cycle=" << m_cycleCount
                     << " all demands=0, falling back to round-robin");
        return FallbackRoundRobin();
    }

    // ── Phase 1: collect starvation-override beams ────────────────────────
    //
    // Beams skipped StarvationThreshold times in a row are forced in regardless
    // of score, to prevent permanent starvation of low-demand beams.
    //
    std::vector<uint32_t> forced, candidates;
    for (auto& kv : m_beams)
    {
        if (kv.second.starvationCount >= m_starvationThreshold)
            forced.push_back(kv.first);
        else
            candidates.push_back(kv.first);
    }

    // Trim forced list to at most K beams.  Multiple beams can become starved
    // in the same cycle; choosing the first K beam IDs would permanently bias
    // service toward low-numbered beams.  Prefer the beams that have waited the
    // longest, then the oldest service time, with beam ID only as a stable tie-break.
    if (forced.size() > m_maxActiveBeams)
    {
        std::sort(forced.begin(),
                  forced.end(),
                  [&](uint32_t a, uint32_t b) {
                      const BeamState& ba = m_beams.at(a);
                      const BeamState& bb = m_beams.at(b);
                      if (ba.starvationCount != bb.starvationCount)
                          return ba.starvationCount > bb.starvationCount;
                      if (ba.lastServedTime != bb.lastServedTime)
                          return ba.lastServedTime < bb.lastServedTime;
                      return a < b;
                  });
        forced.resize(m_maxActiveBeams);
    }

    // ── Phase 2: score remaining candidates ───────────────────────────────

    // Sort candidates by score descending
    std::sort(candidates.begin(), candidates.end(),
              [&](uint32_t a, uint32_t b)
              {
                  return ComputeScore(m_beams.at(a), now)
                       > ComputeScore(m_beams.at(b), now);
              });

    // ── Phase 3: fill active set (forced first, then top scorers) ─────────

    std::vector<uint32_t> selected = forced;

    // Track (gwId, feederFreqId) pairs already committed (constraint 5)
    // to avoid activating two beams on the same GW with the same feeder freq.
    using GwFreqKey = std::pair<uint32_t, uint32_t>;
    std::set<GwFreqKey> usedGwFreq;
    for (uint32_t bid : selected)
    {
        const auto& info = m_beams.at(bid).info;
        usedGwFreq.insert({info.gwId, info.feederFreqId});
    }

    for (uint32_t bid : candidates)
    {
        if (selected.size() >= m_maxActiveBeams) break;

        const auto& info = m_beams.at(bid).info;
        GwFreqKey key{info.gwId, info.feederFreqId};

        if (usedGwFreq.count(key))
        {
            // This beam would conflict on feeder frequency — skip
            NS_LOG_DEBUG("SatGreedyBstpProvider: beam=" << bid
                         << " skipped (feederFreq conflict gw=" << info.gwId
                         << " feederFreq=" << info.feederFreqId << ")");
            continue;
        }

        selected.push_back(bid);
        usedGwFreq.insert(key);
    }

    // ── Phase 4: update per-beam service state ────────────────────────────

    std::unordered_set<uint32_t> selectedSet(selected.begin(), selected.end());
    for (auto& kv : m_beams)
    {
        if (selectedSet.count(kv.first))
        {
            kv.second.lastServedTime    = now;
            kv.second.starvationCount   = 0;
        }
        else
        {
            kv.second.starvationCount++;
        }
    }

    Conf conf = MakeConf(selected);

    NS_LOG_DEBUG("SatGreedyBstpProvider: cycle=" << m_cycleCount
                 << " t=" << now.GetSeconds() << "s"
                 << " selected=" << selected.size() << " beams"
                 << " forced=" << forced.size());

    return conf;
}

// ── Fallback: round-robin ──────────────────────────────────────────────────

SatDynamicBstpProvider::Conf
SatGreedyBstpProvider::FallbackRoundRobin()
{
    // Collect all beam IDs in sorted order for deterministic behaviour
    std::vector<uint32_t> allBeams;
    allBeams.reserve(m_beams.size());
    for (const auto& kv : m_beams)
        allBeams.push_back(kv.first);
    // m_beams is std::map → already sorted by beamId

    const uint32_t N = static_cast<uint32_t>(allBeams.size());
    const uint32_t K = std::min(m_maxActiveBeams, N);

    std::vector<uint32_t> selected;
    selected.reserve(K);
    for (uint32_t i = 0; i < K; i++)
        selected.push_back(allBeams[(m_rrPointer + i) % N]);

    // Advance pointer by K so next call picks the next group
    m_rrPointer = (m_rrPointer + K) % N;

    // Update service state (starvation counter)
    std::unordered_set<uint32_t> selectedSet(selected.begin(), selected.end());
    Time now = Simulator::Now();
    for (auto& kv : m_beams)
    {
        if (selectedSet.count(kv.first))
        {
            kv.second.lastServedTime  = now;
            kv.second.starvationCount = 0;
        }
        else
        {
            kv.second.starvationCount++;
        }
    }

    return MakeConf(selected);
}

// ── MakeConf helper ────────────────────────────────────────────────────────

SatDynamicBstpProvider::Conf
SatGreedyBstpProvider::MakeConf(const std::vector<uint32_t>& selected) const
{
    Conf c;
    c.validityInSuperframes = m_validityInSuperframes;
    c.activeBeams           = selected;
    return c;
}

// ── ValidateConf ───────────────────────────────────────────────────────────

bool
SatGreedyBstpProvider::ValidateConf(const Conf& conf) const
{
    // Rule 1: validity ≥ 1
    if (conf.validityInSuperframes < 1)
    {
        NS_LOG_WARN("SatGreedyBstpProvider::ValidateConf: validityInSuperframes < 1");
        return false;
    }

    // Rule 2: active beam count ≤ K
    if (conf.activeBeams.size() > m_maxActiveBeams)
    {
        NS_LOG_WARN("SatGreedyBstpProvider::ValidateConf: |activeBeams|="
                    << conf.activeBeams.size() << " > MaxActiveBeams=" << m_maxActiveBeams);
        return false;
    }

    // Rule 3: no duplicates
    std::unordered_set<uint32_t> seen;
    for (uint32_t bid : conf.activeBeams)
    {
        if (!seen.insert(bid).second)
        {
            NS_LOG_WARN("SatGreedyBstpProvider::ValidateConf: duplicate beamId=" << bid);
            return false;
        }
    }

    // Rule 4: all beams in enabled set
    for (uint32_t bid : conf.activeBeams)
    {
        if (!m_beams.count(bid))
        {
            NS_LOG_WARN("SatGreedyBstpProvider::ValidateConf: beamId=" << bid
                        << " not in enabled-beam registry");
            return false;
        }
    }

    // Rule 5: no two active beams share (gwId, feederFreqId)
    std::set<std::pair<uint32_t, uint32_t>> usedGwFreq;
    for (uint32_t bid : conf.activeBeams)
    {
        const auto& info = m_beams.at(bid).info;
        auto key = std::make_pair(info.gwId, info.feederFreqId);
        if (!usedGwFreq.insert(key).second)
        {
            NS_LOG_WARN("SatGreedyBstpProvider::ValidateConf: feederFreq conflict"
                        " gw=" << info.gwId << " feederFreq=" << info.feederFreqId
                        << " (beamId=" << bid << ")");
            return false;
        }
    }

    return true;
}

} // namespace ns3
