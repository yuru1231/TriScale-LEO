# E2E 整合系統文件

> [!CAUTION]
> 本文件為私有文件，請勿公開。發表論文後方可開放。

---

## 目錄

- [E2E 整合系統文件](#e2e-整合系統文件)
  - [目錄](#目錄)
  - [簡介](#簡介)
    - [1. 研究背景](#1-研究背景)
    - [2. 研究重要性](#2-研究重要性)
    - [3. 研究貢獻](#3-研究貢獻)
    - [4. 基準設定（固定不可更改）](#4-基準設定固定不可更改)
  - [執行進度](#執行進度)
  - [系統模型](#系統模型)
  - [系統架構](#系統架構)
  - [使用案例圖](#使用案例圖)
  - [訊息序列圖 MSC](#訊息序列圖-msc)
    - [UC1：識別 ROI 服務衛星](#uc1識別-roi-服務衛星)
  - [流程圖](#流程圖)
    - [UC1：識別 ROI 服務衛星](#uc1識別-roi-服務衛星-1)
    - [UC2：建立 QoS Demand Table](#uc2建立-qos-demand-table)
    - [UC3：產生動態 BHTP](#uc3產生動態-bhtp)
  - [類別圖](#類別圖)
  - [關鍵時間參數](#關鍵時間參數)
  - [參考文獻](#參考文獻)

---

## 簡介

### 1. 研究背景

TriScale-LEO E2E（End-to-End）整合層的目標是將三個獨立開發的功能層串接成完整的管線：

- **Layer 1（ISL 路由）**：透過預計算 Dijkstra 最短路徑，建立衛星連接表（connect table），確保封包可在 Starlink-1584 星座中透過 ISL 正確轉送至目標衛星
- **2D 軌道分析（orbit-sgp4）**：基於 SGP4 軌道傳播，計算東京上空保證服務的 ROI 範圍，並將其分割為 5×5 共 25 個格子（grid）
- **Layer 2（動態 Beam Hopping）**：依據 25 格子的 QoS 流量需求，套用Dynamic BHTP以得到最佳服務

### 2. 研究重要性

| 問題 | 說明 |
|------|------|
| 服務連續性 | 東京上方的 Starlink-1584 衛星持續移動，需動態確認哪顆衛星服務 ROI |
| 資源利用率 | 25 個 grid 的流量分佈不均，靜態 beam 分配導致資源浪費 |
| 跨層依賴 | Beam Hopping 排程需知道當前服務衛星的 ISL 連接狀態 |

### 3. 研究貢獻

1. **ROI-aware 服務衛星識別**：結合 ISL connect table 與仰角過濾（≥37°），每個 time slot 動態確認東京 ROI 的服務衛星
2. **25-grid QoS Demand Table**：以 `OnDemandReceived()` 為統一接入口，每格獨立注入流量需求
3. **BHTP 動態生成**：輸出 8 slots × 80 ms 的 BHTP，由 OBC 執行切換

### 4. 基準設定（固定不可更改）

| 參數 | 值 |
|------|-----|
| 星座 | Starlink-1584（1584 顆衛星，72 軌道面 × 22 顆） |
| 軌道高度 | 550 km |
| 軌道傾斜角 | 53° |
| 觀測地點 | 東京（35.68°N, 139.69°E） |
| 最低仰角閾值 | min_elev_deg = 37 |
| ROI 格子數 | 5×5 = 25 格 |
| 每 frame 時槽數 | M = 8（每槽 T_s = 10 ms，frame = 80 ms） |

---

## 執行進度

> **狀態說明：**
> - ○ — 曾實作，待確認 E2E 使用的 code 為乾淨版本（非實驗殘留）
> - ⏳ — 計畫中，尚未實作於 E2E

| 階段 | 狀態 | 來源位置 |
|------|------|---------|
| ISL 預計算（connect table） | ○ | `Topology & ISL Routing/Codes/isl-graph.h/cc` |
| ROI 保證覆蓋範圍計算（東京，37°） | ○ | `2D/orbit-sgp4/code/25beams/code/sat-roi-grid.h` |
| 5×5 ROI 格子建構 | ○ | `GenerateRoiGrid(d=5, rFootprintM)` in `sat-roi-grid.h` |
| 每格覆蓋矩陣計算 | ○ | `2D/orbit-sgp4/data/processed/beam_coverage_projection/` |
| 每 time slot 服務衛星識別 | ○ | `IslRoutingManager::GetGwUtRoute()` + `ComputeElevationDeg()` |
| QoS Demand 接入口（每格，單一介面） | ⏳ | `SatBhScheduler::OnDemandReceived(beamId, bytes)` 接口預留 |
| 動態 BHTP 生成 | ○ | `SatBhScheduler::RunSchedulingCycle()` in `Beam Hopping Controller/Code/helper/` |
| E2E Bridge（ISL connect table → serving sat → BHTP） | ⏳ | 尚未整合 |

---

## 系統模型

E2E 管線的輸入到輸出資料流：

```mermaid
flowchart LR
    subgraph INPUT["輸入資料"]
        TLE["TLE 軌道根數\n(Starlink-1584)"]
        ROI_C["ROI 座標\n(Tokyo 35.68°N, 139.69°E)"]
        ISL_D["ISL 定義\n(isls.txt)"]
    end

    subgraph L1["Layer 1 — ISL Routing"]
        PRECOMP["PrecomputeAllTables()\n每 slot 建立 Dijkstra 路由表"]
        CT["Connect Table\nm_tables[slot][src]"]
        GW_UT["GetGwUtRoute()\nGW → serving sat 路徑"]
        PRECOMP --> CT --> GW_UT
    end

    subgraph ORBIT["2D orbit-sgp4"]
        SGP4["SGP4 傳播\nsat-constellation-scanner"]
        ELEV["仰角過濾\nelevation ≥ 37°"]
        GRID["5×5 ROI Grid\nGenerateRoiGrid(d=5)"]
        BEAMS["25 Beam 中心\nGetEllipticBeamCenters()"]
        SGP4 --> ELEV --> BEAMS
        GRID --> BEAMS
    end

    subgraph L2["Layer 2 — Beam Hopping"]
        DEMAND["Demand Entry Point\nOnDemandReceived(beamId, bytes)"]
        EM["EM 演算法\nRunEM() → λ_n → d_n"]
        BUILD["BuildPlan()\nBhSlotEntry[0..7]"]
        OBC["SatBhObc\nSlot Switching (T_s=10ms)"]
        DEMAND --> EM --> BUILD --> OBC
    end

    TLE --> SGP4
    ISL_D --> PRECOMP
    ROI_C --> ELEV
    GW_UT -->|"serving sat per slot"| BEAMS
    BEAMS -->|"beam-cell mapping"| DEMAND
```

---

## 系統架構

```mermaid
graph TB
    subgraph L1_BOX["Layer 1 — ISL Routing\n(Topology and ISL Routing/Codes/)"]
        ISL_MGR["IslRoutingManager\nisl-graph.h / isl-graph.cc"]
        DIJKSTRA["ComputeBaseRoutes()\nDijkstra 最短路徑（每 slot）"]
        RT["Routing Table\nm_tables[slot][src]"]
        GW_UT_R["GetGwUtRoute(gwId, utId, slot)\n→ GwToUtRoute"]
        ISL_MGR --> DIJKSTRA --> RT
        ISL_MGR --> GW_UT_R
    end

    subgraph ORBIT_BOX["2D orbit-sgp4\n(2D/orbit-sgp4/code/25beams/code/)"]
        SCANNER["SatConstellationScanner\nsat-constellation-scanner.h"]
        ROI_G["RoiGrid / RoiCell\nsat-roi-grid.h"]
        GEOM["GetEllipticBeamCenters()\nsat-multi-beam-geometry.h"]
        COV["Coverage Matrix\nexp_beam_coverage_projection.py"]
        SCANNER --> ROI_G
        SCANNER --> GEOM
        ROI_G --> COV
    end

    subgraph L2_BOX["Layer 2 — Beam Hopping\n(Beam Hopping Controller/Code/helper/)"]
        SCHED["SatBhScheduler\nsat-bh-scheduler.h"]
        PLAN["SatBhTimePlan\nsat-bh-time-plan.h"]
        OBC_B["SatBhObc\nsat-bh-obc.h"]
        SCHED -->|"BuildPlan()"| PLAN
        PLAN -->|"ReceiveNewPlan()"| OBC_B
    end

    subgraph E2E_BOX["E2E Bridge（待實作）"]
        BRIDGE["E2E Integrator\n服務衛星 → Beam 分配 → BHTP"]
    end

    GW_UT_R -->|"servingSatId per slot"| BRIDGE
    GEOM -->|"Vec3[25] beam centers"| BRIDGE
    BRIDGE -->|"OnDemandReceived(beamId, bytes)"| SCHED

    classDef l1 fill:#bbdefb,stroke:#1565c0,color:#000
    classDef orbit fill:#c8e6c9,stroke:#2e7d32,color:#000
    classDef l2 fill:#fff9c4,stroke:#f57f17,color:#000
    classDef e2e fill:#ffcdd2,stroke:#c62828,color:#000,stroke-dasharray: 5 5

    class ISL_MGR,DIJKSTRA,RT,GW_UT_R l1
    class SCANNER,ROI_G,GEOM,COV orbit
    class SCHED,PLAN,OBC_B l2
    class BRIDGE e2e
```

---

## 使用案例圖

```mermaid
graph LR
    CTRL["E2E Controller"]

    subgraph "E2E 整合系統"
        UC1["UC1: 識別每 time slot\nROI 服務衛星"]
        UC2["UC2: 建立 25-grid\nQoS Demand Table"]
        UC3["UC3: 產生動態\nBeam Hopping Time Plan"]
    end

    CTRL -->|"時槽推進"| UC1
    UC1 -->|"servingSatId + beam centers"| UC2
    UC2 -->|"demand[0..24]"| UC3

    classDef actor fill:#e3f2fd,stroke:#1976d2,color:#000
    classDef uc fill:#fff9c4,stroke:#f57f17,color:#000
    class CTRL actor
    class UC1,UC2,UC3 uc
```

---

## 訊息序列圖 MSC

### UC1：識別 ROI 服務衛星

此 MSC 說明每個 time slot 中，E2E Controller 如何透過 ISL 連接表與軌道掃描器，確認東京 ROI 的服務衛星並取得 25 個 beam 中心座標。

```mermaid
sequenceDiagram
    participant E2E as E2E Controller
    participant SCAN as SatConstellationScanner
    participant ISL as IslRoutingManager
    participant GRID as RoiGrid

    Note over E2E,GRID: 時槽推進 slot = k

    E2E->>SCAN: GetSatellitePositions(slot=k)
    SCAN-->>E2E: satPos[] (ECEF, all 1584 sats)

    E2E->>ISL: ComputeElevationDeg(lat=35.68, lon=139.69, satEcef[i])
    ISL-->>E2E: elevDeg[] (per satellite)

    Note over E2E: Filter: elevDeg[i] ≥ 37°
    E2E->>E2E: candidateSats[] = visible satellites

    E2E->>ISL: GetGwUtRoute(gwId=Tokyo_GW, utId=ROI, slot=k)
    ISL-->>E2E: GwToUtRoute { servingSatId, satPath[], islCost }

    E2E->>GRID: GetRoiCellPositions()
    GRID-->>E2E: Vec3[] (25 cell centers in ENU)

    E2E->>SCAN: GetEllipticBeamCenters(servingSatEnu, cfg)
    SCAN-->>E2E: Vec3[25] beam centers

    Note over E2E,GRID: 輸出：servingSatId + beam[0..24] → cell[0..24] 映射
```

---

## 流程圖

### UC1：識別 ROI 服務衛星

```mermaid
flowchart TD
    Start(["Start: Time Slot k"])
    GET_POS["Get satellite positions\n(SGP4, Starlink-1584 at slot k)"]
    CALC_ELEV["Compute elevation angle\nto Tokyo (35.68°N, 139.69°E)\nfor each satellite"]
    FILTER{"elevation ≥ 37°?"}
    NO_CAND{"candidateSats\nempty?"}
    QUERY_ISL["Query ISL connect table\nGetGwUtRoute(Tokyo_GW, ROI, slot=k)"]
    SELECT["Select serving satellite\n(minimum islCost path)"]
    GET_BEAMS["Get 25 beam centers\nGetEllipticBeamCenters(servingSatEnu, cfg)"]
    MAP["Map beam[i] → cell[j]\nvia ROI grid cell positions"]
    End(["End: servingSatId\n+ beam-cell mapping output"])
    WAIT(["No visible satellite\n→ skip slot"])

    Start --> GET_POS --> CALC_ELEV --> FILTER
    FILTER -->|"Yes (add to candidates)"| QUERY_ISL
    FILTER -->|"No"| NO_CAND
    NO_CAND -->|"Yes"| WAIT
    NO_CAND -->|"No (continue filtering)"| FILTER
    QUERY_ISL --> SELECT --> GET_BEAMS --> MAP --> End

    style Start fill:#e3f2fd,color:#000
    style End fill:#c8e6c9,color:#000
    style WAIT fill:#ffcdd2,color:#000
    style FILTER fill:#fff9c4,color:#000
    style NO_CAND fill:#fff9c4,color:#000
```

### UC2：建立 QoS Demand Table

```mermaid
flowchart TD
    Start(["Start: Serving sat + beam-cell mapping ready"])
    INIT["Initialize demand vector\nD[0..24] = 0"]
    FOR_CELL["For each grid cell i = 0..24\n(iterate over ROI cells)"]
    GET_D["Collect traffic demand\nfor cell i (bytes per period)"]
    MAP_B["Map cell i → beamId\nvia beam-cell mapping"]
    INJECT["OnDemandReceived(beamId, bytes)\n→ SatBhScheduler demand history"]
    MORE{"i < 24?"}
    DONE["Build demand vector\nD[beamId] = bytes per beam"]
    End(["End: Demand table D[] ready\nfor EM scheduling"])

    Start --> INIT --> FOR_CELL --> GET_D --> MAP_B --> INJECT --> MORE
    MORE -->|"Yes, i++"| FOR_CELL
    MORE -->|"No"| DONE --> End

    style Start fill:#e3f2fd,color:#000
    style End fill:#c8e6c9,color:#000
    style MORE fill:#fff9c4,color:#000
```

### UC3：產生動態 BHTP

```mermaid
flowchart TD
    Start(["Start: Demand table D[0..24] ready\n(every T_p = 80 ms)"])
    EM_E["EM E-step\nQ(λ | λ_old) = Σ [x_n,t × log(λ_n) − λ_n × T_s]"]
    EM_M["EM M-step\nλ_n_new = (1 / W×M) × Σ_t x_n,t"]
    CONV{"Converge?\n||λ_new − λ_old|| < ε\nor max iterations?"}
    SLOT["Compute slot allocation\nd_n = max(1, round(λ_n / Σλ × M × K_b))"]
    VIRT["Compute virtual traffic\nA_n = L_n × α × (1 + 1/T_p)"]
    CLUSTER["Group beams into clusters\nω_{i,j} ≥ κ → merge"]
    BUILD["BuildPlan()\n→ BhSlotEntry[0..7]\n(K beams per slot, MODCOD, patterns)"]
    VALID{"Validate plan\n(K-limited, no overlap,\nall d_n satisfied?)"}
    SEND["Send to OBC\n(+ T_prop = 10 ms propagation delay)"]
    EXEC["OBC: Execute slot switching\nevery T_s = 10 ms"]
    End(["End: BHTP active\nframe period = 80 ms"])
    ADJUST["Adjust d_n\n(global slot correction)"]

    Start --> EM_E --> EM_M --> CONV
    CONV -->|"No"| EM_E
    CONV -->|"Yes"| SLOT
    SLOT --> VIRT --> CLUSTER --> BUILD --> VALID
    VALID -->|"Valid"| SEND
    VALID -->|"Invalid"| ADJUST --> SLOT
    SEND --> EXEC --> End

    style Start fill:#e3f2fd,color:#000
    style End fill:#c8e6c9,color:#000
    style CONV fill:#fff9c4,color:#000
    style VALID fill:#fff9c4,color:#000
    style ADJUST fill:#ffcdd2,color:#000
```

---

## 類別圖

E2E 管線涉及三個層次的關鍵 class，以下展示其屬性、方法與跨層關係。

```mermaid
classDiagram
    class IslRoutingManager {
        <<Layer 1 — isl-graph.h>>
        -RoutingTable m_tables
        -ISLGraph m_graph
        -GwToUtRoute m_gwUtRoutes
        +PrecomputeAllTables()
        +GetGwUtRoute(gwId, utId, slot) GwToUtRoute
        +ComputeElevationDeg(lat, lon, satEcef) double
        +ApplyRoutingTable(slot)
    }

    class GwToUtRoute {
        +uint32_t gwId
        +uint32_t utId
        +uint32_t entrySatId
        +uint32_t servingSatId
        +vector satPath
        +double islCost
        +bool valid
    }

    class RoiGrid {
        <<2D — sat-roi-grid.h>>
        -int d
        -double L_m
        -double W_m
        -RoiCell cells[]
        +GenerateRoiGrid(d, rFootprintM) RoiGrid
        +GetRoiCellPositions() Vec3[]
        +GetRoiCellIndexMap() size_t[]
    }

    class RoiCell {
        +int row
        +int col
        +double cx_m
        +double cy_m
        +bool inFootprint
    }

    class SatConstellationScanner {
        <<2D — sat-constellation-scanner.h>>
        +ScanAtTime(timeS, cfg)
        +GetEllipticBeamCenters(satEnu, cfg) Vec3_25
    }

    class SatBhScheduler {
        <<Layer 2 — sat-bh-scheduler.h>>
        -float m_demandHistory[beamId]
        -float m_lambda[beamId]
        -int m_slotAlloc[beamId]
        -float m_virtualTraffic[beamId]
        +OnDemandReceived(beamId, bytes)
        +RunEM()
        +ComputeSlotAllocation()
        +ComputeVirtualTraffic()
        +BuildPlan() SatBhTimePlan
        +RunSchedulingCycle()
    }

    class SatBhTimePlan {
        <<Layer 2 — sat-bh-time-plan.h>>
        -uint32_t planId
        -uint32_t frameN
        -BhSlotEntry m_slots_8
        +Validate() bool
        +ToCsv() string
    }

    class BhSlotEntry {
        +vector beamIds
        +Time startTime
        +Time duration
        +ModcodIndex modcod
        +vector clusterIds
    }

    class SatBhObc {
        <<Layer 2 — sat-bh-obc.h>>
        +ReceiveNewPlan(plan, T_prop)
        +ExecuteSlotSwitch(slotEntry)
    }

    IslRoutingManager --> GwToUtRoute : outputs
    RoiGrid "1" --> "*" RoiCell : contains
    SatBhTimePlan "1" --> "8" BhSlotEntry : contains
    SatBhScheduler ..> SatBhTimePlan : generates
    SatBhObc ..> SatBhTimePlan : executes
    SatConstellationScanner ..> RoiGrid : provides positions for
    SatBhScheduler ..> GwToUtRoute : uses serving sat from
```

---

## 關鍵時間參數

| 符號 | 定義 | 值 | 備註 |
|------|------|-----|------|
| T_s | Slot 持續時間 | 10 ms | 每個 beam 切換週期（2026-06 更新） |
| T_p | BHTP 週期（M × T_s） | 80 ms | 每個 frame 的總長度 |
| T_sw | Beam 切換死區時間 | 2 ms | 可用窗口 = T_s − T_sw = 8 ms |
| T_prop | GW → 衛星傳播延遲 | 10 ms | 不可更改（550 km 軌道高度） |
| M | 每 frame 的 slot 數 | 8 | BHTP 共 8 個 BhSlotEntry |
| K | 同時啟動最大 beam 數 | 3 | 預設值，可設 2–4 |
| λ_n | 第 n 個 beam 的 Poisson 到達率 | EM 估計 | 每 frame 更新 |
| d_n | 第 n 個 beam 的 slot 分配數 | ∈ [1, M] | 由 λ_n 計算 |

---

## 參考文獻

[1] SpaceX, "Starlink Satellite Constellation," FCC Filing SAT-MOD-20190830-00087, 2019.

[2] 3GPP, "Solutions for NR to support Non-Terrestrial Networks (NTN)," Technical Specification TS 38.821, 2024.

[3] D. Bhattacherjee and A. Singla, "Network topology design at 27,000 km/hour," in *Proc. ACM CoNEXT*, 2019.

[4] F. Díaz-Rodríguez et al., "Dynamic beam hopping for high-throughput satellite systems," *IEEE Trans. on Broadcasting*, vol. 68, no. 2, 2022.
