# TriScale-LEO — System Model

> **Status:** Living document — updated as each layer finalises its orbit interface.
>
> This document is the single source of truth for all orbit, channel, and constellation parameters used across Layer 1 (ISL Routing), Layer 2 (Beam Hopping), and Layer 3 (QoS Scheduler).  Each layer section describes exactly what it reads from the orbit model and what it produces.

---

## Table of Contents

- [1. Constellation](#1-constellation)
- [2. Orbit Propagation — SGP4](#2-orbit-propagation--sgp4)
- [3. Channel Model](#3-channel-model)
- [4. ROI Grid](#4-roi-grid)
- [5. Satellite Serving Sequence](#5-satellite-serving-sequence)
- [6. Layer Interfaces](#6-layer-interfaces)
  - [6.1 Layer 1 — ISL Routing](#61-layer-1--isl-routing)
  - [6.2 Layer 2 — Beam Hopping](#62-layer-2--beam-hopping)
  - [6.3 Layer 3 — QoS Scheduler](#63-layer-3--qos-scheduler)
- [7. End-to-End Data Flow](#7-end-to-end-data-flow)
- [8. Parameters Reference](#8-parameters-reference)

---

## 1. Constellation

| Parameter | Value | Source |
|-----------|-------|--------|
| Constellation | Iridium-NEXT (Iridium-66) | TLE file `2D/data/tle/iridium.txt` |
| Number of satellites | 66 | 6 planes × 11 satellites per plane |
| Orbital altitude | ~780 km | TLE-derived |
| Inclination | ~86.4° | TLE-derived |
| Orbital period | ~100 min | TLE-derived |
| Satellite index convention | `iridium-75 00` … `iridium-75 65` | PyEphem TLE name field |

**Orbit plane labelling** (for Phase 2 handover analysis):

```
Plane 0: sat  0 – 10     Plane 3: sat 33 – 43
Plane 1: sat 11 – 21     Plane 4: sat 44 – 54
Plane 2: sat 22 – 32     Plane 5: sat 55 – 65
```

> Same-plane consecutive satellites (e.g., `iridium-75 44` → `iridium-75 45`) produce the shortest overlap window and are the primary handover scenario for Layer 2.

---

## 2. Orbit Propagation — SGP4

All satellite positions are computed by **`2D/code/orbit-sgp4/run_sgp4.py`** using the PyEphem SGP4 propagator (WGS72 reference ellipsoid, same as Hypatia).

### 2.1 Coordinate Output

Each orbit CSV row carries:

| Column | Unit | Description |
|--------|------|-------------|
| `time_s` | s | Seconds from window start (0-indexed) |
| `sat_lat_deg` | °N | Geodetic latitude (WGS72, Bowring-converted) |
| `sat_lon_deg` | °E | Sub-point longitude |
| `sat_alt_m` | m | Height above WGS72 ellipsoid |
| `elevation_deg` | ° | Elevation from observer (PyEphem, corrected) |
| `azimuth_deg` | ° | Azimuth from observer (N = 0°) |

### 2.2 Two Propagation Modes

| Mode | CLI flag | Use case |
|------|----------|----------|
| `single` | `--mode single` | L1 snapshot; L2 grid mode (one satellite) |
| `sequence` | `--mode sequence` | L2 dual mode (sat[i] + sat[i+1] handover) |

**Sequence mode** scans all 66 satellites over a search window, finds all passes with peak elevation ≥ `--min-peak-elev-deg`, sorts them chronologically, and outputs two CSVs on a **shared 0-based time axis** — required by the dual-mode C++ simulation.

### 2.3 ENU Frame Conversion

The C++ modules convert the geodetic orbit CSV to a local **ENU frame** (East-North-Up) centred at the observer/ROI:

```
OrbitPointToEnuVec3(orbitPt, obsLatDeg, obsLonDeg)
  → ECEF offset = geodetic_to_ecef(sat) - geodetic_to_ecef(observer)
  → EcefOffsetToEnu(ecefOffset, obsLat, obsLon)
  → Vec3 {East_m, North_m, Up_m}
```

All beam-gain and SNR calculations operate in this ENU frame.

---

## 3. Channel Model

### 3.1 RF Parameters

| Parameter | Value |
|-----------|-------|
| Carrier frequency | 30 GHz (Ka-band downlink) |
| Bandwidth | 25 MHz |
| Satellite TX power | 10 W per beam |
| Noise temperature | 290 K |
| Noise power | −131.6 dBW (= kTB) |

### 3.2 Antenna — 32×32 UPA

Each beam uses a **32 × 32 Uniform Planar Array (UPA)** with half-wavelength spacing:

```
AF(θ, φ) = Σ_{m,n} exp(j · d_λ · [m·sin(θ)cos(φ) + n·sin(θ)sin(φ)])
BeamGain_lin = |AF_x(θ,φ)|² × |AF_y(θ,φ)|²   (separable Dirichlet kernel)
```

The array pointing direction (steering vector) is updated at each timestep based on the satellite ENU position.  A **z-axis pre-rotation** (`BuildArrayTransform`) corrects for azimuth-induced gain error at low elevation angles (Phase 2.5 fix).

### 3.3 19-Beam Hexagonal Layout

19 beams are arranged in two concentric hexagonal rings centred on nadir.  At each timestep, each user/cell is assigned to the beam with the highest UPA gain (`argmax` over 19 beams).

```
beam_id[u] = argmax_{b ∈ 0..18} BeamGain(satPos, beamCenter[b], userPos[u])
```

### 3.4 Path Loss

```
PathLoss_dB = FSPL_dB + AtmosphericAbsorption_dB
FSPL_dB     = 20·log10(4π·d·f / c)
AtmAbsorb   = 0.1 dB/km × slant_range_km   (3GPP NTN approximation)
```

### 3.5 SNR / SINR

```
SNR_dB  = TxPower_dBW + BeamGain_dB − PathLoss_dB − NoisePower_dBW
SINR_dB = desired / (Σ inter-beam interference + noise)
```

Sentinel value **−999 dB** marks "satellite not visible" (elevation < `min_elevation_deg = 5°`).

---

## 4. ROI Grid

A fixed ground **Region of Interest (ROI)** is defined as a d × d rectangular grid inscribed in the satellite footprint circle.

| Parameter | Value | Description |
|-----------|-------|-------------|
| ROI centre | 35.676°N, 139.650°E | Tokyo (default, overridable via `--lat --lon`) |
| Footprint radius | 100 km | Single-satellite service circle |
| Grid dimension | d = 5 (default) | 5 × 5 = 25 cells |
| Grid side length | L = r_footprint × √2 ≈ 141.4 km | Inscribed square |
| Cell spacing | L / (d−1) ≈ 28.3 km | Uniform |
| In-footprint filter | `cx² + cy² ≤ r²` | Cells outside circle excluded |

Cell positions are computed by `GetRoiCellPositions()` in `sat-roi-grid.cc` and passed directly to `ComputeFrameResults()` as user positions.

---

## 5. Satellite Serving Sequence

At any ROI location, satellites pass overhead sequentially.  The **serving sequence** is derived by `run_sgp4.py --mode sequence`:

```
All 66 TLEs → scan passes → sort by peak_time → qualifying list
qualifying[0]  = sat[i]    (current serving satellite)
qualifying[1]  = sat[i+1]  (next serving satellite)
qualifying[n]  = sat[n]    (generalised, via --pair-index n)
```

**Overlap window** (Phase 2.3): the period during which both sat[i] and sat[i+1] are simultaneously above the 5° elevation threshold.  Five handover thresholds are recorded:

| Threshold | Definition | Default result (Tokyo, same-plane pair) |
|-----------|------------|----------------------------------------|
| 10% | sat[i+1] covers ≥ 10% of ROI cells | t ≈ 318.7 s |
| 25% | ≥ 25% | t ≈ 322.3 s |
| 50% | ≥ 50% | t ≈ 327.7 s |
| 75% | ≥ 75% | t ≈ 334.5 s |
| 90% | ≥ 90% | t ≈ 336.4 s |

The **Greedy assignment** baseline (Phase 2.4) assigns each cell at each timestep to the satellite with higher SNR: `selected_sat[cell] = argmax(SNR_i, SNR_i1)`.

---

## 6. Layer Interfaces

### 6.1 Layer 1 — ISL Routing

**What Layer 1 reads from the orbit model:**

```
GetPositionsAt(τ_k)
  → SGP4 queries all 66 satellite positions at slot time τ_k
  → returns Vec3 array [66] in ECEF coordinates
```

Layer 1 calls the SNS3 built-in SGP4 mobility model (`satellite-sgp4-mobility-model`) — it does **not** use the orbit CSVs from `run_sgp4.py`.  The two SGP4 implementations share the same TLE epoch and converge to the same positions.

**What Layer 1 produces:**

| Output | Format | Consumer |
|--------|--------|----------|
| ISL topology graph | Adjacency list, per time slot | Dijkstra routing |
| Routing tables | `m_tables[k]` per slot | Arbiter updates |
| `routing_plan.csv` | `slot, src, dst, nexthop, bandwidth_budget_Mbps` | Layer 2 bandwidth constraint |

**Key parameters from constellation:**

| Parameter | Value | Role |
|-----------|-------|------|
| ISL distance threshold | 5000 km | Edge pruning in topology graph |
| Slot interval | configurable (default 1 s) | Routing update cadence |
| Number of satellites | 66 | Graph node count |

---

### 6.2 Layer 2 — Beam Hopping

**What Layer 2 reads from the orbit model:**

Layer 2 consumes the **pre-computed per-cell SNR time series** output from the 2D orbit-sgp4 simulation:

```
orbit_sat_i.csv   ─┐
                   ├─→  dual mode C++ simulation  ─→  cell_result.csv
orbit_sat_i1.csv  ─┘     (RunDualMode)                cell_summary.csv
                                                       overlap.json
```

| Input to BH Scheduler | Source | Description |
|-----------------------|--------|-------------|
| `snr_i_dB[cell][t]` | `cell_result.csv` | Per-cell SNR from sat[i] at each timestep |
| `snr_i1_dB[cell][t]` | `cell_result.csv` | Per-cell SNR from sat[i+1] at each timestep |
| `overlap.json` thresholds | `overlap.json` | Timestamps when sat[i+1] reaches 10/25/50/75/90% coverage |
| `greedy_mean_snr_dB[cell]` | `cell_summary.csv` | Time-averaged Greedy baseline SNR per cell |

**What Layer 2 produces:**

| Output | Description |
|--------|-------------|
| Beam hopping schedule | Per-frame: which cells are served, which beams are active, power allocation |
| Handover trigger | Decision point to switch primary satellite from sat[i] → sat[i+1] |
| Throughput per beam per slot | Based on SNR → MCS mapping |

**Orbit-to-BH connection (time axis alignment):**

```
time_s in orbit CSV (100 ms steps)
  ↕  mapped to  ↕
T_slot = 26.5 ms  (Layer 2 time unit)
T_frame = 503 ms  (19 slots)
```

The BH scheduler operates at T_slot resolution; the orbit CSV provides SNR at 100 ms resolution.  The scheduler interpolates or resamples as needed.

---

### 6.3 Layer 3 — QoS Scheduler

**What Layer 3 reads from the orbit model:**

Layer 3 receives per-beam available capacity, derived from the orbit-to-channel pipeline:

```
SNR_dB[beam][t]  →  MCS selection  →  capacity_Mbps[beam][t]
```

| Input to QoS Scheduler | Derived from | Description |
|------------------------|--------------|-------------|
| `capacity_Mbps[beam][t]` | SNR + Shannon bound or MCS table | Max throughput per beam per slot |
| `beam_active[beam][t]` | BH schedule from L2 | Whether this beam is allocated in this slot |
| `bandwidth_budget_Mbps` | `routing_plan.csv` from L1 | ISL bandwidth constraint for this GW-SAT link |

**Capacity estimation from SNR:**

```
spectral_efficiency = min(log2(1 + SNR_lin), capacity_cap_bps_per_hz)
capacity_Mbps       = spectral_efficiency × bandwidth_Hz / 1e6
```

**What Layer 3 produces:**

| Output | Description |
|--------|-------------|
| Packet schedule | Which packets are served in each slot, prioritised by QoS class |
| Queue depth per flow | Residual queue after scheduling |
| QoS violation flag | Whether delay/throughput SLA is met per flow |

---

## 7. End-to-End Data Flow

```
TLE file (iridium.txt)
        │
        ▼
run_sgp4.py --mode sequence
        │  → orbit_sat_i.csv   (sat[i]   position, elevation, azimuth over time)
        │  → orbit_sat_i1.csv  (sat[i+1] position, elevation, azimuth over time)
        │
        ├──────────────────────────────────────────────────────────────────┐
        │  [Layer 1 path]                                                  │
        │  SNS3 SGP4 mobility model                                        │
        │    → satellite positions at each routing slot                   │
        │    → ISL topology graph  →  Dijkstra  →  routing_plan.csv       │
        │                                                                  │
        ▼  [Layer 2 / 3 path]                                             │
dual mode C++ simulation (RunDualMode)                                    │
        │  → per-cell SNR[t] for sat[i] and sat[i+1]                     │
        │  → overlap.json (handover thresholds)                           │
        │  → cell_summary.csv (time-averaged SNR per cell)                │
        │                                                                  │
        ▼                                                                  │
Layer 2 — Beam Hopping Scheduler                                          │
        │  inputs:  SNR[cell][t], overlap.json, routing_plan.csv ◄────────┘
        │  outputs: BH schedule, handover trigger, capacity[beam][t]
        │
        ▼
Layer 3 — QoS Packet Scheduler
        │  inputs:  capacity[beam][t], beam_active[beam][t], queue depths
        │  outputs: packet schedule, QoS violation flags
```

---

## 8. Parameters Reference

### Constellation & Orbit

| Symbol | Value | Description |
|--------|-------|-------------|
| N_sat | 66 | Total satellites |
| N_plane | 6 | Orbital planes |
| N_per_plane | 11 | Satellites per plane |
| h | ~780 km | Orbital altitude |
| v_sat | ~7.46 km/s | Orbital velocity |
| T_orbit | ~100 min | Orbital period |
| Pass duration | ~10 min | Visible time above 5° from Tokyo |

### Channel

| Symbol | Value | Description |
|--------|-------|-------------|
| f_c | 30 GHz | Carrier frequency (Ka-band) |
| B | 25 MHz | Bandwidth |
| N_ant | 32 × 32 | UPA elements per beam |
| d_ant | 0.5λ | Antenna element spacing |
| N_beam | 19 | Hexagonal beam layout |
| r_footprint | 100 km | Single-satellite footprint radius |
| K_Rician | 10 | Rician K-factor |
| min_elev | 5° | Minimum elevation for SNR computation |

### ROI Grid

| Symbol | Value | Description |
|--------|-------|-------------|
| lat_c, lon_c | 35.676°N, 139.650°E | ROI centre (Tokyo, default) |
| d | 5 | Grid dimension (d × d = 25 cells) |
| L | r_footprint × √2 ≈ 141.4 km | Grid side length |
| Δ_cell | L / (d−1) ≈ 28.3 km | Cell spacing |

### Time

| Symbol | Value | Description |
|--------|-------|-------------|
| Δt_orbit | 100 ms | Orbit CSV timestep |
| T_slot | 26.5 ms | Layer 2 time slot |
| T_frame | 503 ms | Layer 2 frame (19 slots) |
| T_slot_L1 | 1 s (configurable) | Layer 1 routing slot |

---

> **Related files:**
> - Orbit CSV generator: [`2D/code/orbit-sgp4/run_sgp4.py`](2D/code/orbit-sgp4/run_sgp4.py) (production) / [`2D/phase2/code/run_sgp4.py`](2D/phase2/code/run_sgp4.py) (historical)
> - 2D simulation: [`2D/code/orbit-sgp4/`](2D/code/orbit-sgp4/) (arc removed) / [`2D/phase2/code/`](2D/phase2/code/) (historical with arc)
> - Layer 1: [`Topology & ISL Routing/Layer1.md`](Topology%20%26%20ISL%20Routing/Layer1.md)
> - Layer 2: [`Beam Hopping Controller/Layer2.md`](Beam%20Hopping%20Controller/Layer2.md)
> - Layer 3: [`QoS-Aware Packet Scheduler/Layer3.md`](QoS-Aware%20Packet%20Scheduler/Layer3.md)
