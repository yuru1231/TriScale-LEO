
* **Guide = 概念架構**
* **V6 = 目前實作(v6為新版本)**
* **Gap = 後續研究**

---

# LEO ISL Routing Architecture and V6 Implementation Status

## 1. Introduction

The purpose of this report is twofold:

1. Define ISL routing system** for LEO constellations (referred to as the *ISL Routing Guide*).
2. To analyze how the **current V6 implementation** maps to this architecture and identify which components are already implemented and which remain future work.
---

# 2. ISL Routing Guide

## 2.1 Satellite Network Model

A LEO satellite network can be modeled as a **time-varying graph**:

```id="j3jxt6"
G(t) = (V, E(t))
```

Where:

* **V** represents the set of satellites
* **E(t)** represents the set of inter-satellite links available at time `t`

Because satellites follow fixed orbital trajectories, their positions can be predicted using orbital propagation models. As time progresses, the connectivity between satellites changes, resulting in a sequence of topology snapshots:

```
G(t0)
G(t1)
G(t2)
...
```

Each snapshot represents the network topology at a specific time.

---

# 2.2 Satellite Position Generation

Satellite positions are generated using orbital propagation models such as **SGP4**.

For each simulation time step:

```id="xpdhr1"
satellite_positions(t)
```

These positions are used to determine geometric relationships between satellites, including:

* inter-satellite distance
* line-of-sight availability
* relative elevation

---

# 2.3 ISL Connectivity Rules

An inter-satellite link (ISL) is considered active when geometric constraints are satisfied.

A typical baseline rule is:

```id="l3h5p0"
distance(sat_i , sat_j) ≤ islMaxDistanceKm
```

Additional constraints may include:

* line-of-sight restrictions
* antenna steering limits
* elevation thresholds

If the conditions are satisfied, an edge is added to the network graph.

---

# 2.4 Edge Weight Model

Each ISL is assigned a routing cost:

```id="zh02ju"
w(e)
```

Common baseline cost definitions include:

### Hop Count

```id="ry11ys"
w(e) = 1
```

### Propagation Delay

```id="e6oew3"
w(e) = distance / c
```

Future routing models may incorporate additional metrics such as:

* congestion
* queue backlog
* switching cost
* reliability

---

# 2.5 Routing Problem

Given a source and destination:

```
source = s
destination = d
```

The routing problem is to find a path that minimizes total cost:

```id="zicbhp"
minimize Σ w(e)
```

Subject to the constraint that the path exists in the current topology.

---

# 2.6 Baseline Routing Strategy

The simplest routing strategy for LEO networks is **snapshot routing**.

For each time step:

```
1. construct G(t)
2. compute shortest path
```

This approach uses standard shortest-path algorithms such as Dijkstra.

Although simple, snapshot routing may lead to frequent path changes because each time step is computed independently.

---

# 2.7 Predictive Routing

Because satellite motion is predictable, future network topologies can be estimated in advance.

Instead of recomputing routes dynamically, routing plans may be **precomputed**:

```
routing_table(t0)
routing_table(t1)
routing_table(t2)
```

During operation, the system simply switches to the appropriate routing table based on the current time.

Advantages include:

* reduced control overhead
* near-zero routing convergence time
* deterministic routing behavior

---

# 2.8 Conceptual System Architecture

The ISL routing framework can be decomposed into the following modules:

```
Topology Sampler
      ↓
Link Builder
      ↓
Graph Builder
      ↓
Path Solver
      ↓
Plan Generator
      ↓
Analyzer
```

Each module has a specific responsibility:

### Topology Sampler

Generates satellite positions for each simulation time.

### Link Builder

Determines which ISLs are active.

### Graph Builder

Constructs the network graph.

### Path Solver

Computes routing paths between nodes.

### Plan Generator

Builds routing schedules across time.

### Analyzer

Evaluates routing performance and topology metrics.

---

# 3. V6 Implementation Overview

The current V6 system represents an executable implementation of the baseline routing architecture.

## 3.1 Execution Model

V6 adopts a **schedule-driven simulation model**:

* a single `Simulator::Run()` execution
* periodic routing evaluation using `ScheduledSample()`

At each sampling event, routing-related computations are performed.

---

# 3.2 V6 Routing Pipeline

The V6 execution pipeline can be summarized as:

```
Simulator::Run()
        ↓
ScheduledSample()
        ↓
GraphCache update
        ↓
Routing computation
        ↓
Append routing result
```

This pipeline corresponds to snapshot routing where routing decisions are evaluated at discrete time steps.

---

# 3.3 Output and Monitoring

V6 introduces several execution-management mechanisms:

### Incremental Output

Routing results are written incrementally to avoid data loss if a simulation stops unexpectedly.

### Progress Tracking

```
progress.json
```

Records simulation progress.

### Execution Status

```
summary.json
```

Tracks the simulation state:

```
RUNNING
COMPLETED
FAILED
```

### Path Verification

The `verify_path` function ensures routing results correspond to valid network paths.

---

# 4. Mapping Between Guide Architecture and V6

The following table compares the conceptual architecture with the current V6 implementation.

| Architecture Module              | Guide 定義                                   | V6 Implementation                                    | Status      | 說明                               |
| -------------------------------- | ------------------------------------------ | ---------------------------------------------------- | ----------- | -------------------------------- |
| **Topology Sampler**             | 在每個時間 `t` 取得衛星位置 `satellite_positions(t)`  | V6 透過 **ScheduledSample()** 在每個 step 更新衛星幾何資訊        | Implemented | 每個 sampling step 都會更新衛星相關狀態      |
| **Time Discretization**          | 時間離散化 `t = tStart + k·dt`                  | V6 使用 `tStart / tEnd / dt` 控制 sampling               | Implemented | routing 與 connectivity 在固定時間步長更新 |
| **Satellite Position Model**     | 使用 **SGP4 orbital propagation** 計算位置       | V6 使用 scenario constellation mobility                | Implemented | constellation scenario 提供軌道與位置   |
| **Link Builder**                 | 根據條件建立 ISL (`distance ≤ islMaxDistanceKm`) | V6 在每個 step 判斷 ISL connectivity                      | Implemented | 生成 snapshot connectivity         |
| **ISL Constraint Model**         | distance / LOS / elevation 等限制             | V6 目前主要使用 **distance constraint**                    | Partial     | 其他限制尚未完全加入                       |
| **Graph Builder**                | 建立 snapshot graph `G(t)`                   | V6 使用 **GraphCache** 更新 graph                        | Implemented | 每個 step refresh topology         |
| **Edge Weight Assignment**       | 定義 `w(e)` (hop / delay / cost)             | V6 目前主要使用 baseline cost                              | Partial     | 尚未完整支援多 cost model               |
| **Routing Solver**               | shortest path search (Dijkstra)            | V6 進行 routing path search 並可 verify                  | Implemented | 已能輸出 routing path                |
| **Snapshot Routing**             | 每個 `t` 重新計算 path                           | V6 在每個 sampling step 進行 routing                      | Implemented | baseline snapshot routing        |
| **Predictive Routing Framework** | 預先計算 routing schedule                      | V6 step-wise append routing result                   | Partial     | routing plan 尚未完整壓縮              |
| **Routing Plan Generator**       | 將 snapshot routing 組合成 routing plan        | V6 每 step append routing output                      | Implemented | incremental write                |
| **Plan Compression**             | 合併相同路徑區間 `[t_start, t_end]`                | V6 尚未完全壓縮 routing plan                               | Future      | 後續優化                             |
| **Route Stability Control**      | route holding / switching penalty          | V6 目前無 explicit stability policy                     | Future      | snapshot routing 仍可能頻繁變化         |
| **Time-Dependent Cost**          | `w(e,t)` 時間相關 cost                         | V6 未正式實作 temporal cost                               | Future      | future routing model             |
| **Analyzer – Connectivity**      | reachability / partition                   | V6 可輸出 connectivity 相關資訊                             | Partial     | metrics 未完全自動化                   |
| **Analyzer – Path Metrics**      | hop count / delay                          | V6 path output 可計算                                   | Partial     | 尚未完整整合 analyzer                  |
| **Analyzer – Route Stability**   | route change / lifetime                    | V6 尚未正式計算                                            | Future      | 研究分析部分                           |
| **Analyzer – Topology Metrics**  | degree / ISL count / diameter              | V6 有 connectivity data                               | Partial     | 需額外 analyzer                     |
| **Execution Framework**          | routing evaluation pipeline                | V6 採 **single Simulator::Run() + ScheduledSample()** | Implemented | schedule-based execution         |
| **Graph Update Strategy**        | 每 snapshot 更新 graph                        | V6 GraphCache per-step refresh                       | Implemented | snapshot graph                   |
| **Progress Tracking**            | execution monitoring                       | V6 `progress.json`                                   | Implemented | execution observability          |
| **Execution Status**             | simulation state                           | V6 `summary.json`                                    | Implemented | RUNNING / COMPLETED              |
| **Incremental Output**           | 避免 crash loss                              | V6 append writing                                    | Implemented | v6 新增                            |
| **Path Verification**            | path correctness check                     | V6 `verify_path`                                     | Implemented | v6 新增                            |


---

# 5. Current Limitations

Although the V6 system implements the baseline routing pipeline, several capabilities described in the conceptual architecture remain incomplete.

## 5.1 Route Stability Optimization

Snapshot routing recomputes routes independently at each time step.

This may lead to unnecessary route switching even when the previous route remains viable.

Potential solutions include:

* route holding strategies
* switching penalties
* hysteresis mechanisms

---

## 5.2 Routing Plan Compression

Predictive routing typically represents routing plans as time intervals:

```
[t_start , t_end] → path
```

The current V6 implementation records snapshot results but does not yet compress them into interval-based routing schedules.

---

## 5.3 Time-Dependent Routing Models

Future routing models may consider edge costs that vary over time:

```id="43dmt6"
w(e,t)
```

This would enable routing algorithms to incorporate time-dependent delay or congestion effects.

---

# 6. Future Work

Future development will focus on extending the baseline routing framework with the following capabilities:

1. **Route stability mechanisms** to reduce unnecessary path switching.
2. **Predictive routing schedule generation** to compress routing plans across time.
3. **Comprehensive analyzer metrics** for evaluating routing performance.
4. **Time-dependent routing models** that incorporate dynamic edge costs.

These extensions will allow the routing framework to evolve from baseline snapshot routing toward more advanced predictive routing strategies.

---
