/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * sat-bh-time-plan.cc
 *
 * Implementation of SatBhTimePlan and BhSlotEntry.
 * See sat-bh-time-plan.h for full documentation.
 */

#include "sat-bh-time-plan.h"

#include "ns3/log.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SatBhTimePlan");
NS_OBJECT_ENSURE_REGISTERED(SatBhTimePlan);

// ── Free helpers ──────────────────────────────────────────────────────────

std::string
BeamRadiusTypeToString(BeamRadiusType r)
{
    switch (r)
    {
        case BeamRadiusType::XSMALL: return "XSMALL";
        case BeamRadiusType::SMALL:  return "SMALL";
        case BeamRadiusType::MIDDLE: return "MIDDLE";
        case BeamRadiusType::LARGE:  return "LARGE";
        case BeamRadiusType::XLARGE: return "XLARGE";
        default:                     return "UNKNOWN";
    }
}

// ── BhSlotEntry ───────────────────────────────────────────────────────────

BhSlotEntry::BhSlotEntry()
    : startTime(Seconds(0.0)),
      duration(MilliSeconds(10.0)),   // T_s project fixed value (10 ms/slot)
      modcod(0)
{
}

BhSlotEntry::BhSlotEntry(uint32_t beamId, Time start, Time dur,
                          BeamRadiusType pattern, ModcodIndex mc)
    : startTime(start),
      duration(dur),
      modcod(mc)
{
    beamIds.push_back(beamId);
    // Single-beam slot: cluster ID defaults to the beam's own ID (no interference grouping)
    clusterIds.push_back(beamId);
    // Record the per-beam pattern for this single-beam slot
    beamPatterns[beamId] = pattern;
}

bool
BhSlotEntry::ContainsBeam(uint32_t beamId) const
{
    return std::find(beamIds.begin(), beamIds.end(), beamId) != beamIds.end();
}

void
BhSlotEntry::SetBeamPattern(uint32_t beamId, BeamRadiusType pattern)
{
    beamPatterns[beamId] = pattern;
}

BeamRadiusType
BhSlotEntry::GetBeamPattern(uint32_t beamId, BeamRadiusType defaultVal) const
{
    auto it = beamPatterns.find(beamId);
    return (it != beamPatterns.end()) ? it->second : defaultVal;
}

void
BhSlotEntry::Print(std::ostream& os) const
{
    os << "  Slot ["
       << std::fixed << std::setprecision(2)
       << startTime.GetMilliSeconds() << " ms .. "
       << GetEndTime().GetMilliSeconds() << " ms]"
       << "  beams={";
    for (size_t i = 0; i < beamIds.size(); i++)
    {
        if (i) os << ",";
        os << beamIds[i];
    }
    // Per-beam pattern summary: radius=SMALL (or "beam1:SMALL,beam4:LARGE" if mixed)
    os << "}  radius=";
    if (beamPatterns.empty())
    {
        os << "MIDDLE";   // fallback when no patterns recorded
    }
    else if (beamPatterns.size() == 1)
    {
        os << BeamRadiusTypeToString(beamPatterns.begin()->second);
    }
    else
    {
        // Mixed patterns — show first; full per-beam detail in ToCsv
        bool first = true;
        for (const auto& kv : beamPatterns)
        {
            if (!first) os << ",";
            os << kv.first << ":" << BeamRadiusTypeToString(kv.second);
            first = false;
        }
    }
    os << "  modcod=" << modcod
       << "  clusters={";
    for (size_t i = 0; i < clusterIds.size(); i++)
    {
        if (i) os << ",";
        os << clusterIds[i];
    }
    os << "}\n";
}

// ── SatBhTimePlan — TypeId ────────────────────────────────────────────────

TypeId
SatBhTimePlan::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::SatBhTimePlan")
            .SetParent<Object>()
            .AddConstructor<SatBhTimePlan>();
    return tid;
}

// ── SatBhTimePlan — constructor ───────────────────────────────────────────

SatBhTimePlan::SatBhTimePlan()
    : m_planId(0),
      m_n(0),
      m_periodStart(Seconds(0.0)),
      m_periodEnd(MilliSeconds(80.0))   // T_p project fixed: 8 slots × 10 ms
{
}

// ── Setters / builders ────────────────────────────────────────────────────

void
SatBhTimePlan::SetPlanId(uint32_t id)
{
    NS_ASSERT_MSG(id > 0, "SatBhTimePlan::SetPlanId: planId must be > 0");
    m_planId = id;
}

void
SatBhTimePlan::SetFrameN(uint32_t n)
{
    m_n = n;  // n = frame number in the formal BH model
}

void
SatBhTimePlan::SetPeriodBounds(Time start, Time end)
{
    NS_ASSERT_MSG(end > start,
                  "SatBhTimePlan::SetPeriodBounds: periodEnd must be after periodStart");
    m_periodStart = start;
    m_periodEnd   = end;
}

void
SatBhTimePlan::AddSlot(const BhSlotEntry& slot)
{
    // Enforce chronological ordering so GetSlotAt() binary-search is valid
    if (!m_slots.empty())
    {
        NS_ASSERT_MSG(slot.startTime >= m_slots.back().GetEndTime(),
                      "SatBhTimePlan::AddSlot: slot startTime="
                          << slot.startTime.GetMilliSeconds()
                          << "ms overlaps or precedes previous slot end="
                          << m_slots.back().GetEndTime().GetMilliSeconds() << "ms");
    }
    m_slots.push_back(slot);
    NS_LOG_DEBUG("SatBhTimePlan plan=" << m_planId
                 << " added slot " << (m_slots.size() - 1)
                 << " start=" << slot.startTime.GetMilliSeconds() << "ms"
                 << " beams=" << slot.beamIds.size());
}

// ── Getters ───────────────────────────────────────────────────────────────

const BhSlotEntry*
SatBhTimePlan::GetSlotAt(Time t) const
{
    // t is an absolute simulation time.
    // Convert to period-relative offset for comparison.
    const Time offset = t - m_periodStart;

    // Slots are stored in ascending order; linear scan is fine for M ≈ 19.
    // For large M, this can be replaced with std::lower_bound on startTime.
    for (const auto& slot : m_slots)
    {
        if (offset >= slot.startTime && offset < slot.GetEndTime())
            return &slot;
    }
    return nullptr;
}

int32_t
SatBhTimePlan::GetSlotIndexAt(Time t) const
{
    const Time offset = t - m_periodStart;
    for (int32_t i = 0; i < static_cast<int32_t>(m_slots.size()); i++)
    {
        if (offset >= m_slots[i].startTime && offset < m_slots[i].GetEndTime())
            return i;
    }
    return -1;
}

std::vector<uint32_t>
SatBhTimePlan::GetAllBeamIds() const
{
    std::vector<uint32_t> ids;
    for (const auto& slot : m_slots)
    {
        for (uint32_t bid : slot.beamIds)
        {
            // Append only if not already present (preserve order of first appearance)
            if (std::find(ids.begin(), ids.end(), bid) == ids.end())
                ids.push_back(bid);
        }
    }
    return ids;
}

double
SatBhTimePlan::GetBeamDwellMs(uint32_t beamId) const
{
    double totalMs = 0.0;
    for (const auto& slot : m_slots)
    {
        if (slot.ContainsBeam(beamId))
            totalMs += slot.duration.GetMilliSeconds();
    }
    return totalMs;
}

// ── Validation ────────────────────────────────────────────────────────────

bool
SatBhTimePlan::Validate(uint32_t maxActiveBeams) const
{
    bool ok = true;

    // ── Check 1: planId assigned ──────────────────────────────────────────
    if (m_planId == 0)
    {
        NS_LOG_WARN("SatBhTimePlan::Validate: planId is 0 (not yet assigned by scheduler)");
        ok = false;
    }

    // ── Check 2: period bounds consistent ────────────────────────────────
    if (m_periodEnd <= m_periodStart)
    {
        NS_LOG_WARN("SatBhTimePlan::Validate: periodEnd <= periodStart");
        ok = false;
    }

    // ── Check 3: slot-level invariants ────────────────────────────────────
    for (size_t i = 0; i < m_slots.size(); i++)
    {
        const BhSlotEntry& s = m_slots[i];
        const Time period = m_periodEnd - m_periodStart;

        // 3a: at least one beam per slot
        if (s.beamIds.empty())
        {
            NS_LOG_WARN("SatBhTimePlan::Validate: slot " << i << " has no beams");
            ok = false;
        }

        // 3b: beam count does not exceed K
        if (s.beamIds.size() > maxActiveBeams)
        {
            NS_LOG_WARN("SatBhTimePlan::Validate: slot " << i
                        << " activates " << s.beamIds.size()
                        << " beams, exceeds K=" << maxActiveBeams);
            ok = false;
        }

        // 3c: slot lies within declared period (using period-relative offsets)
        if (s.startTime < Seconds(0.0) || s.GetEndTime() > period)
        {
            NS_LOG_WARN("SatBhTimePlan::Validate: slot " << i
                        << " offset [" << s.startTime.GetMilliSeconds() << " ms, "
                        << s.GetEndTime().GetMilliSeconds() << " ms)"
                        << " lies outside period duration "
                        << period.GetMilliSeconds() << " ms");
            ok = false;
        }

        // 3d: no overlap with previous slot
        if (i > 0 && s.startTime < m_slots[i - 1].GetEndTime())
        {
            NS_LOG_WARN("SatBhTimePlan::Validate: slot " << i
                        << " overlaps slot " << (i - 1));
            ok = false;
        }

        // 3e: clusterIds size must match beamIds size (if provided)
        if (!s.clusterIds.empty() && s.clusterIds.size() != s.beamIds.size())
        {
            NS_LOG_WARN("SatBhTimePlan::Validate: slot " << i
                        << " clusterIds.size()=" << s.clusterIds.size()
                        << " != beamIds.size()=" << s.beamIds.size());
            ok = false;
        }
    }

    if (ok)
    {
        NS_LOG_INFO("SatBhTimePlan::Validate: plan " << m_planId
                    << " passed (" << m_slots.size() << " slots, period="
                    << GetPeriodDuration().GetMilliSeconds() << " ms)");
    }

    return ok;
}

// ── Serialization / debug ─────────────────────────────────────────────────

void
SatBhTimePlan::PrettyPrint(std::ostream& os) const
{
    os << "\n=== SatBhTimePlan ===\n"
       << "  planId      : " << m_planId << "\n"
       << "  periodStart : " << std::fixed << std::setprecision(3)
                             << m_periodStart.GetSeconds() << " s\n"
       << "  periodEnd   : " << m_periodEnd.GetSeconds()   << " s\n"
       << "  duration    : " << GetPeriodDuration().GetMilliSeconds() << " ms"
                             << "  (T_p = 80 ms project fixed)\n"
       << "  numSlots    : " << m_slots.size() << "\n"
       << "  slots:\n";

    for (const auto& s : m_slots)
        s.Print(os);

    // Per-beam dwell summary
    os << "  beam dwell summary:\n";
    for (uint32_t bid : GetAllBeamIds())
    {
        os << "    beam " << bid << " : "
           << std::fixed << std::setprecision(2)
           << GetBeamDwellMs(bid) << " ms\n";
    }

    os << "====================\n\n";
}

std::string
SatBhTimePlan::ToCsv() const
{
    std::ostringstream oss;
    // Header matches the slot table format (not the KPI table)
    oss << "slotIdx,startMs,durationMs,beamIds,beamPatterns,modcod,clusterIds\n";

    for (size_t i = 0; i < m_slots.size(); i++)
    {
        const BhSlotEntry& s = m_slots[i];

        oss << i << ","
            << std::fixed << std::setprecision(3)
            << s.startTime.GetMilliSeconds() << ","
            << s.duration.GetMilliSeconds()  << ",";

        // beamIds as semicolon-separated list inside one CSV column
        for (size_t j = 0; j < s.beamIds.size(); j++)
        {
            if (j) oss << ";";
            oss << s.beamIds[j];
        }
        // per-beam patterns: "beamId:PATTERN" pairs, semicolon-separated
        oss << ",";
        bool firstPat = true;
        for (uint32_t bid : s.beamIds)
        {
            if (!firstPat) oss << ";";
            oss << bid << ":" << BeamRadiusTypeToString(s.GetBeamPattern(bid));
            firstPat = false;
        }
        oss << "," << s.modcod << ",";

        // clusterIds — empty if not assigned
        for (size_t j = 0; j < s.clusterIds.size(); j++)
        {
            if (j) oss << ";";
            oss << s.clusterIds[j];
        }
        oss << "\n";
    }

    return oss.str();
}

} // namespace ns3
