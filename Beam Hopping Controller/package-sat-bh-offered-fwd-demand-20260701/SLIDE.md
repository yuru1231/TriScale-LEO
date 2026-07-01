---
marp: true
theme: default
paginate: true
backgroundColor: "#ffffff"
---

# FWD Offered Demand — Beam Hopping Validation

**sat-bh-example · offered-fwd 套件**
2026-07-01

---

## Objective

Validate the **pure FWD Offered Demand scheduling** pipeline end-to-end:

```
FWD Offered Load (20 Mbps/UT)
  → enableFwdOfferedDemand=1
    → DynamicBstp writes OFFERED_FWD demand (no RBDC)
      → SatBhObc ToggleState
        → GW SatNetDevice real beam switch
          → throughput verification
```

---

## What Changed from apply-tr

| | apply-tr | **offered-fwd (this package)** |
|---|---|---|
| DEMAND source | RBDC (BacklogRequestsTrace) | **FWD Offered Load (CBR 20 Mbps/beam)** |
| DEMAND type | `RBDC` | `OFFERED_FWD` |
| RBDC rows | > 0 | **0** |
| demand_kbps | dynamic | **fixed 20,000 kbps/beam** |

> Key goal: clean separation — OFFERED_FWD path carries **zero RBDC contamination**

---

## Scenario Parameters

| Parameter | Value |
|---|---|
| Constellation | Starlink-1584 (1584 sats) |
| ROI Center | Tokyo (35.676°N, 139.650°E) |
| Beam format | 25-beam 5×5 UPA elliptical grid |
| Helper satellites | 382, 404 (2 sats) |
| K (active beams/slot) | 2 |
| BHTP period | 8 slots × 10 ms = **80 ms** |
| Sim duration | 30 s (warmUp = 0.1 s) |
| Features enabled | OBC + DynamicBstp + enableFwdOfferedDemand |
| ISL / RBDC | **Disabled** (no Phase F) |

---

## Run Command

```bash
./ns3 run "sat-bh-example \
  --scenario=starlink25 \
  --enableObc=1 \
  --enableDynamicBstp=1 \
  --enableFwdOfferedDemand=1 \
  --fwdOfferedDemandKbps=20000 \
  --maxActiveBeams=2 \
  --simTime=30 \
  --warmUp=0.1 \
  --helperSatList=382,404 \
  --metricsFile=/tmp/offered2-metrics-test.csv \
  --timePlanFile=/tmp/offered2-timeplan-test.csv \
  --trafficTraceFile=/tmp/offered2-traffic-test.tr"
```

> Do **not** add `--enablePhaseF` — avoids RBDC path interference

---

## Key Validation Results

```text
PLAN              6,000
DEMAND           18,750
OFFERED_FWD      18,750   ← all OFFERED_FWD, zero RBDC
OFFERED_FWD_min  20,000 kbps
OFFERED_FWD_max  20,000 kbps
RBDC                  0   ← clean separation confirmed
EVENT            23,992
mapped_toggled   23,992   ← all EVENTs: mapped=1, toggled=1
sats             382, 404
```

- Every DEMAND row is `OFFERED_FWD` type, `demand_kbps = 20000`
- All 23,992 OBC EVENTs successfully mapped and toggled
- Final DEACTIVATE at **t = 29.998 s** (plan_id = 375)

---

## Output Files

| File | Rows | Description |
|---|---|---|
| `offered2-metrics-test_starlink25_*.csv` | 1,493 | KPI per T_p=80ms, per satellite, per beam |
| `offered2-timeplan-test_starlink25_*.csv` | 9 (incl. header) | BHTP slot table |
| `offered2-traffic-test_starlink25_*.tr` | 48,743 | BH traffic trace (PLAN + DEMAND + EVENT) |

---

## Trace Fields

```
record_type, time_s, sat_id, beam_id, event, plan_id, slot_idx,
demand_kbps, duration_ms, active_beams, mapped, toggled
```

| record_type | Description |
|---|---|
| `PLAN` | BHTP slot plan per scheduling cycle |
| `DEMAND` | Beam demand snapshot — all `OFFERED_FWD`, demand_kbps = 20000 |
| `EVENT` | OBC ACTIVATE/DEACTIVATE execution record |

- `mapped=1` — OBC found the corresponding GW SatNetDevice
- `toggled=1` — `ToggleState()` was actually called

---

## BHTP Slot Structure

| slotIdx | Start (ms) | Duration (ms) | Active Beams | modcod |
|---|---|---|---|---|
| 0 | 0 | 10 | 1, 2 | 5 |
| 1 | 10 | 10 | 3, 4 | 5 |
| 2 | 20 | 10 | 5, 6 | 5 |
| 3 | 30 | 10 | 7, 8 | 5 |
| 4 | 40 | 10 | 9, 10 | 5 |
| 5 | 50 | 10 | 11, 12 | 5 |
| 6 | 60 | 10 | 13, 14 | 5 |
| 7 | 70 | 10 | 15, 16 | 5 |

Period T_p = 80 ms · 8 slots · K = 2 beams/slot
Initial plan covers beams 1–16; DynamicBstp adjusts dynamically each cycle.

---

## Traffic Setup

| Direction | Type | Interval | Packet Size | Rate |
|---|---|---|---|---|
| FWD (GW → UT) | CBR UDP | 600 μs | 1500 B | **20 Mbps/UT** |
| RTN baseline (UT → GW) | CBR UDP | 100 ms | 512 B | ~41 kbps/UT |

- FWD 20 Mbps offered load injected directly into DynamicBstp via `enableFwdOfferedDemand=1`
- Phase F disabled → RTN remains baseline only, no extra RBDC traffic

---

## Execution Phases

| Phase | Flag | Function |
|---|---|---|
| Phase 1 | (default on) | SatBhTimePlan + SatBhMetrics (static BHTP, synthetic drive) |
| Phase 2 / OBC | `--enableObc=1` | SatBhObc real ToggleState switching |
| Phase G | `--enableDynamicBstp=1` | SatDynamicBstpProvider greedy Top-K scheduling |
| **Offered FWD** | `--enableFwdOfferedDemand=1` | **FWD offered load as demand — no Phase F / RBDC** |

---

## Design Principles

1. **Interface isolation** — `sat-bh-example.cc` calls only `SatBhHelper`; never accesses Scheduler / OBC / DynamicBstp directly

2. **Non-invasive wiring** — all SNS3 callbacks connected via `Config::ConnectWithoutContext()` / trace callbacks; **SNS3 source untouched**

3. **BSTP conflict avoidance** — OBC enabled → auto-set `EnableFwdLinkBeamHopping=false` to prevent SNS3 built-in SatBstpController from racing with SatBhObc

4. **Clean demand path** — `enableFwdOfferedDemand=1` guarantees DEMAND rows come only from FWD offered load; verify by checking `demand_kbps == 20000` with zero RBDC rows

---

## Expected Behavior (Step-by-Step)

1. Load `starlink25` scene — Starlink-1584 constellation, `t_offset=4168s` (Tokyo ROI peak elevation snapshot)
2. Install 2 × SatBhHelper (sat 382, sat 404), each covering 25 beams
3. FWD CBR 20 Mbps/UT + RTN baseline ~41 kbps/UT start after warmUp=0.1s *(no Phase F RBDC traffic)*
4. SatDynamicBstpProvider selects K=2 optimal beams every T_p=80ms using OFFERED_FWD=20000 kbps
5. SatBhObc calls `ToggleState(true/false)` at slot boundaries — `mapped=1, toggled=1` verified
6. Simulation ends at t=30s; final DEACTIVATE EVENT at **t=29.998s** (plan_id=375)

---

## Summary

| Verification Point | Result |
|---|---|
| DEMAND type | 100% OFFERED_FWD |
| RBDC contamination | 0 rows |
| demand_kbps uniformity | Fixed 20,000 kbps/beam |
| OBC event coverage | 23,992 / 23,992 (mapped & toggled) |
| SNS3 source modified | **No** |

> Pipeline validated: FWD Offered Load → OFFERED_FWD demand → OBC beam switch → GW SatNetDevice toggled
