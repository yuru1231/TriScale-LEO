# SNS3 預先排程路由架構 — Guide

> 只記當前有效的設計與接口。歷史決策與修改原因見 `decisions/`。

---

## 1. 系統概述

### 1.1 核心主張

原生 SNS3 在每個更新週期執行完整的 O(N²) 拓樸重建加全量 Dijkstra。本架構將這部分工作移至模擬開始前的離線預計算階段，執行期只做輕量的即時修正。

### 1.2 與原生 SNS3 的對比

| 項目 | 原生 SNS3 | 本架構 |
|------|----------|--------|
| 拓樸重建 | 每 60s 重建，O(N²) | 離線一次完成，執行期不重建 |
| Dijkstra | 每次全量重算 | 只對受影響節點重跑 |
| load 感知 | 不支援 | 執行期讀 device queue 微調 |
| 時鐘依賴 | `GetPosition()` 依賴 `Simulator::Now()` | SGP4 直接代入 τ_k，與時鐘無關 |

### 1.3 星座參數

| 參數 | 數值 |
|------|------|
| Scenario | `constellation-iridium-66-sats-fixed` |
| 衛星數量 | 66 顆（6 軌道面 × 11 顆） |
| 軌道高度 | 780 km，傾角 86.4° |
| ISL 數量 | 132 條（每顆衛星 4 條） |
| 模擬時長 | 600 秒 |
| 路由更新粒度 τ | 60 秒，共 10 個時間點 |

---

## 2. 三層架構

### 2.1 系統級三層（由大到小的時間維度）

| Layer | 職責 | 時間維度 | 執行時機 |
|------|------|---------|---------|
| **Layer 1：ISL Routing** | FT pair 篩選 + 衛星間最短路由 | 分鐘級（60s slot） | 離線預計算；執行期每 60s 套用 |
| **Layer 2：Beam Hopping** | BHTP-based 多 beam 動態調度（K=3 同時活動），EM 需求估算 + 虛擬流量排程 | DVB-S2X super-frame 級（T_s=26.5ms，T_p=503ms） | NCC 端排程；OBC 端每 slot 切換 |
| **Layer 3：QoS Scheduling** | 在給定 beam 服務時間內對 UE 做 priority + WFQ 排程 | 封包級 | SNS3 `SatBeamScheduler` 原生處理 |

### 2.2 Layer 1 內部子層（ISL Routing 模組原有設計）

| 子層 | 職責 | 執行時機 |
|------|------|---------|
| 離線預計算 | 批次計算各時間點的拓樸與基礎路由表 | `Simulator::Run()` 之前 |
| 執行期微調 | 讀取 device queue，判斷是否需要微調 Dijkstra | 每 60s，`Simulator::Schedule` 觸發 |
| 封包轉發 | 封包到達節點後查 Arbiter 轉發 | 隨時，SNS3 Arbiter 機制處理 |

### 2.3 Layer 2 內部子系統（Beam Hopping 正式架構）

正式 BH 架構為 7 模組協作，以 BHTP（Beam Hopping Time Plan）為核心資料模型：

| 模組 | 位置 | 核心職責 |
|------|------|---------|
| `SatBhTimePlan` | 資料模型 | 儲存完整 BHTP 週期與 slot 配置（beamIds, radius, modcod, clusterIds） |
| `SatBhScheduler` | NCC 側 | EM 需求估算、虛擬流量計算、beam scheduling、cluster 分組、輸出 BHTP |
| `SatBhObc` | 衛星側 | 接收 BHTP、執行 beam switching 狀態機（IDLE/ACTIVE/SWITCHING/WAIT_PLAN） |
| `SatGwCacheQueue` | Gateway 側 | beam inactive 時暫存封包（最大 40 MB/beam），slot 開始時排空 |
| `SatBhPrecoder` | Gateway 側 | cluster 條件成立時做 MMSE 預編碼（W = H^H(HH^H + σ²I)^-1） |
| `SatBhMetrics` | 被動監測 | 被動收集 throughput / delay / JFI / drop rate 等 KPI |
| `SatBhHelper` | 安裝入口 | 統一安裝所有模組並接好 trace/hook |

### 2.4 層間接口

```
Layer 1 (IslRoutingManager)
  → 輸出：m_tables[slotIndex][satId] → vector<RouteEntry>
  → FtVisibilityFilter 讀取 GetRouteCost(entry, exit, slot) 做篩選

Layer 1 extension (FtVisibilityFilter)
  → 輸出：GetBestTransit(ftI, ftJ, slot) → FtTransitRoute{entrySat, exitSat, cost}
  → 告知 Layer 2 哪些衛星是 contracted path 上的 transit nodes

Layer 2 (SatBhScheduler / SatBhObc)
  → 輸出：SatBhTimePlan → BhSlotEntry{beamIds, startTime, duration, beamRadius, modcod, clusterIds}
  → SatBhMetrics 被動收集各 slot 的 KPI

Layer 3 (SNS3 native)
  → 讀取 SatBeamScheduler 設定，在對應 beam 的服務時間內依 QoS 優先序排程
```

---

## 3. SNS3 接口（當前有效）

### ISL 相關類別

| 類別 | 用途 |
|------|------|
| `PointToPointIslNetDevice` | ISL device，內含 `DropTailQueue<Packet>` |
| `SatOrbiterNetDevice` | 衛星主 device，管理所有 ISL device 與 Arbiter |
| `SatIslArbiterUnicast` | 實作類，內部為 `map<destSatId, islInterfaceIndex>` |

### 各模組接口

| 模組 | SNS3 接口 |
|------|-----------|
| `BuildISLGraph(τ_k)` | `sat->GetObject<SatSGP4MobilityModel>()->GetGeoPositionAt(τ_k)` |
| `UpdateLoadCosts()` | `islDev->GetQueue()->GetNPackets()` |
| `ApplyRoutingTable()` | `orbDev->SetArbiter(newArbiter)` |
| `ApplyTable(τ_k)` | `Simulator::Schedule(Seconds(60), callback)` |

### `InitOrbiterDevices()`（在 `Simulator::Run()` 前呼叫）

```cpp
// 預先建立並安裝 66 個 arbiter，快取供排程使用
for (uint32_t i = 0; i < 66; i++) {
    Ptr<Node> sat = ...;
    Ptr<SatOrbiterNetDevice> orbDev =
        DynamicCast<SatOrbiterNetDevice>(sat->GetDevice(0));
    Ptr<SatIslArbiterUnicast> arbiter =
        CreateObject<SatIslArbiterUnicast>(sat);   // 在有效上下文建立
    orbDev->SetArbiter(arbiter);
    m_arbiters[i] = arbiter;                       // 快取
}
```

### `ApplyRoutingTable()`（排程執行時呼叫）

```cpp
// 使用預先建立的 arbiter，原地清空後重填
Ptr<SatIslArbiterUnicast> arbiter = m_arbiters[satId];
arbiter->ClearNextHopEntries();

for (auto& entry : routingTable[satId])
{
    arbiter->AddNextHopEntry(entry.destSatId, entry.islInterfaceIndex);
}
```

注意：排程內不可呼叫 `CreateObject<SatIslArbiterUnicast>()`，會觸發 fatal。詳見 `decisions/DEC-003-arbiter-lifecycle.md`。

### `BuildISLGraph()` 效能注意事項

位置計算必須在迴圈外預先快取，不可在 ISL 邊迴圈內逐次呼叫：

```cpp
// 正確：一次性快取所有衛星位置
std::vector<Vector> pos(m_numSatellites);
for (uint32_t i = 0; i < m_numSatellites; i++)
    pos[i] = m_orbNodes[i]->GetObject<SatSGP4MobilityModel>()
                           ->GetGeoPositionAt(tau_k).ToVector();
// 之後 ISL 邊迴圈直接用 pos[a], pos[b]
```

`PrecomputeAllTables` 中以滾動快取傳遞 `graphNext`，避免重複建圖：
- 錯誤做法：每輪獨立呼叫，10 個 slot 實際建圖 19 次
- 正確做法：`graphNext` 以 `std::move` 成為下輪 `graphCurr`，只建 10 次

### `UpdateLoadCosts()` 實作

```cpp
std::vector<Ptr<PointToPointIslNetDevice>> islDevs =
    orbDev->GetIslsNetDevices();

for (uint32_t i = 0; i < islDevs.size(); i++)
{
    uint32_t queuePackets = islDevs[i]->GetQueue()->GetNPackets();
    // 套用 EMA，計算 load_cost
}
```

### `ifIndex` 對應方式

不依賴 `isls.txt` 順序，改用 peer nodeId 查 vector index：

```cpp
for (uint32_t j = 0; j < islDevs.size(); j++) {
    Ptr<Node> peer = islDevs[j]->GetDestinationNode();
    if (peer) peerNodeIdToIfIdx[i][peer->GetId()] = j;
}
```

### Layer 2 BH 核心時間參數

| 符號 | 定義 | 數值 |
|------|------|------|
| `T_sf` | DVB-S2X Super-frame | 26.5 ms |
| `T_s` | BH Time Slot | 26.5 ms |
| `T_p` | BHTP period | 503 ms |
| `T_sw` | beam switching time | 2 ms |
| `T_prop` | 指令下發延遲 | 10 ms（建議） |
| `K` | 同時活動 beam 數 | 3（預設） |

### BH 非侵入式擴充原則

所有新增模組不得修改 SNS3 原生程式碼，互動只能透過：
- `TraceSource / TraceCallback`
- `Object::AggregateObject()`
- `Config::ConnectWithoutContext()`
- `ObjectFactory + TypeId`

---

## 4. ISL 連接條件

| 條件 | 數值 |
|------|------|
| 距離上限 | 5000 km |
| 仰角門檻 | 不適用（僅用於 UT/GW 鏈路） |
| Beam 狀態 | 第一階段恆為 true |

---

## 5. Cost Function

### Layer 1：ISL Routing

```
total_cost = α × propagation_cost + β × load_cost

propagation_cost = distance_ab / c        （離線算好，固定）
load_cost        = queue_packets × packet_size / bandwidth  （執行期讀取）
```

| 參數 | 數值 |
|------|------|
| c | 3×10⁸ m/s |
| bandwidth | 10 Mbps |
| packet_size | 1500 bytes |
| α, β | 各為 1（等權重） |

第一階段 `load_cost` 初始為 0，cost 等同純傳播延遲。

### Layer 2：BH 虛擬流量

```
A_{p,j} = L_{p,j} × α × (1 + 1 / T_{p,j})   // 單封包虛擬流量
A_j = Σ_p A_{p,j}                              // 小區總虛擬流量
```

| 參數 | 說明 | 建議值 |
|------|------|------|
| `α` | delay sensitivity factor | 2（初始） |
| `T_{p,j}` | 封包剩餘 TTL（slot 計） | — |
| `κ` | 干擾 cluster 合併門檻 | 0.08 |

---

## 6. 執行順序

### Layer 1（ISL Routing）

```
1. 建立節點 + 安裝 InternetStack
2. 建立 ISL 鏈路 + 分配 IP 位址
3. BuildNodeToIpMap()
4. PrecomputeAllTables()
5. InitOrbiterDevices()          ← 預先建立並安裝 66 個 arbiter，快取指標
6. ScheduleRoutingUpdates()      ← Simulator::Schedule × 10（t=0,60,...,540）
7. 安裝應用層流量（從 t=1s 開始）
8. Simulator::Stop(Seconds(600))
9. Simulator::Run()
10. FlowMonitor 輸出 + Simulator::Destroy()
```

應用層從 t=1s 開始，確保 `ApplyTable(0)` 先於封包發送執行。

### Layer 2（Beam Hopping）

```
✅ Phase 1: SatBhTimePlan + SatBhMetrics（靜態 BHTP + ApplySyntheticSlot 驅動）
✅ Phase 2: SatBhScheduler（EM only）+ SatBhObc（狀態機）+ ApplySyntheticDemand 注入
⏳ Phase 3: SatGwCacheQueue + SatBhPrecoder
⏳ Phase 4: SatBhScheduler（完整）+ SatBhHelper（統一安裝）+ 真實 SNS3 trace 接入
```

---

## 7. 資料結構

### Layer 1

```cpp
struct ISLState {
    uint32_t nodeA, nodeB;
    bool     eligible;
    bool     stableToNext;           // tiebreaker 用
    double   propagation_cost;
    double   load_cost;
    double   smoothed_load;
    double   total_cost;
    std::set<uint32_t> sourcesUsingThisISL;
};

struct RouteEntry {
    uint32_t destNodeId;
    uint32_t nextHopNodeId;
    uint32_t islInterfaceIdx;        // GetIslsNetDevices() vector index
    double   cost;
};

struct RoutingTableSnapshot {
    Time     timestamp;
    std::map<uint32_t, std::vector<RouteEntry>> routingTables;
    std::vector<ISLState> islStates;
    std::map<uint32_t, std::vector<std::pair<uint32_t,double>>> adjacency;
};

std::map<Time, RoutingTableSnapshot> m_precomputedTables;
```

### Layer 2

```cpp
struct BhSlotEntry {
    std::vector<uint32_t> beamIds;   // 本 slot 同時活動的 beam（最多 K 個）
    Time startTime;
    Time duration;
    BeamSize beamRadius;             // SMALL/MIDDLE/LARGE
    uint32_t modcod;
    std::vector<uint32_t> clusterIds;
};

struct SatBhTimePlan {
    uint32_t planId;
    Time periodStart;
    Time periodEnd;
    std::vector<BhSlotEntry> slots;
};
```

---

## 8. 執行期震盪抑制（Layer 1）

| 機制 | 設定 |
|------|------|
| EMA | `smoothed = 0.7×prev + 0.3×instant` |
| Hysteresis | 新路徑 cost 必須低於現有路徑 δ（建議 0.5×avg_prop） |
| Cooldown | 同一 ISL 切換後 60s 內不再切換 |

---

## 9. 程式檔案狀態

### Layer 1：ISL Routing

| 檔案 | 修改內容 | 狀態 |
|------|---------|------|
| `satellite-sgp4-mobility-model.h/.cc` | 新增 `GetGeoPositionAt(Time t)` | ✅ 完成 |
| `satellite-isl-arbiter-unicast.h` | 新增 `ClearNextHopEntries()` | ✅ 完成 |
| `v5_isl-graph.h/.cc` | 完整 ISL routing，load-aware，tiebreaker；`GetNumTimeSlots()`、`GetNumSatellites()`、`GetTimeSlotInterval()`、`GetRouteCost()` | ✅ 完成 |

### Layer 1 Extension：FT Visibility Filter

| 檔案 | 內容 | 狀態 |
|------|------|------|
| `ft-filter.h` | `FtDef`、`FtTransitRoute`、`FtVisibilityFilter` 介面 | ✅ 完成 |
| `ft-filter.cc` | `PrecomputeVisibility()`、`GetAccessSats()`、`GetBestTransit()`、`ComputeElevationDeg()` | ✅ 完成 |

### Layer 2：Beam Hopping（正式架構，7 模組）

| 檔案 | 內容 | 狀態 |
|------|------|------|
| `sat-bh-time-plan.h/.cc` | `SatBhTimePlan`、`BhSlotEntry` 資料模型 | ✅ Phase 1 |
| `sat-bh-metrics.h/.cc` | `SatBhMetrics` 被動 KPI 收集，CSV 輸出 | ✅ Phase 1 |
| `sat-bh-scheduler.h/.cc` | `SatBhScheduler`：EM 估算、虛擬流量、beam scheduling、cluster 分組；`OnDemandReceived` 1-indexed 修正（2026-03-31） | ✅ Phase 2 |
| `sat-bh-obc.h/.cc` | `SatBhObc`：BHTP 接收、beam switching 狀態機（IDLE→ACTIVE→SWITCHING→WAIT_PLAN） | ✅ Phase 2 |
| `sat-gw-cache-queue.h/.cc` | `SatGwCacheQueue`：beam inactive 暫存、slot start dequeue | ⏳ Phase 3 |
| `sat-bh-precoder.h/.cc` | `SatBhPrecoder`：MMSE 預編碼（cluster ≥ 2 beam 時啟動） | ⏳ Phase 3 |
| `sat-bh-helper.h/.cc` | `SatBhHelper`：統一安裝入口，trace/hook 連線，feature flag；Phase 2 新增 `ApplySyntheticDemand`、修正 OBC/Scheduler 初始化順序 | ✅ Phase 2 active（Phase 3/4 stub 保留） |

> 舊版 `beam-hopping-manager.h/.cc` 保留作 Phase 0 prototype validator，不作為正式架構。

### Layer 3：QoS（純配置）

| 位置 | 內容 | 狀態 |
|------|------|------|
| `v5_test-iridium.cc` `ConfigureQoS()` | Class A (CRA)、Class B (RBDC)、Class C (VBDC) SNS3 attribute config | ✅ 架構完成，attribute 路徑待驗證 |

### 整合測試

| 檔案 | 內容 | 狀態 |
|------|------|------|
| `v5_test-iridium.cc` | Layer 1 + FT filter 完整整合，FT 台灣/日本/美國，4 個台灣 cell | ✅ 完成（BH 為 stub） |

---

## 10. 待確認事項（blocking）

| 項目 | 說明 | 位置 |
|------|------|------|
| **BH Hook 點可行性** | `DaRequestReceived`、`SlotAllocated`、`GwMac::Tx/Rx`、`ChannelEstimation`、`HandoverCompleted` 是否存在、路徑與 callback 簽章是否匹配；若不存在需記錄 fallback；目前以 `ApplySyntheticDemand` 替代 | Phase 4（`ConnectTraces()` stub 解封前） |
| **SNS3 BH 注入 API** | Phase 3 `SatGwCacheQueue` 需呼叫正確的 SNS3 method 攔截封包。候選：`SatOrbiterFeederMac`、`GwMac::Tx` trace、或 `SatBeamHelper` 動態 enable/disable | `sat-gw-cache-queue.cc`（Phase 3） |
| **台灣 beam ID** | `v5_test-iridium.cc` 目前用 `SetBeamSet({1})`，需從 scenario 資料確認涵蓋台灣（25°N 121°E）的 beam ID | `v5_test-iridium.cc` 標記 `TODO BH` |
| **Layer 3 QoS attribute 路徑** | `SatLowerLayerServiceConf::DaService*` 的 attribute key 需對照安裝的 SNS3 版本確認 | `v5_test-iridium.cc` `ConfigureQoS()` |

---

## 11. 擴充點（預留）

- **增量更新**：`ApplyRoutingTable()` 改為 diff-based 寫入
- **真實流量輸入**：實作 `SatBhScheduler::OnDemandReceived`，從 UE queue 或外部預測資料注入 cell demand
- **多 FT 負載平衡**：`GetBestTransit()` 可擴充為考慮 FT 當前負載的多路由選擇
- **MMSE 全量開啟**：`SatBhHelper::EnableMMSEPrecoding(true)`，對所有 cluster size ≥ 2 的 slot 啟動

---

## 12. 已知問題

### Beam Scheduler 開銷（DEC-004）

**現象**：simTime=630s 的模擬 wall time 約 2760s（≈4.38× 放大）

**原因**：SNS3 DVB MAC beam scheduler 持續產生排程事件（66 顆衛星 × forward+return × superframe 250ms），即使沒有使用者流量也會產生大量事件。

**現況**：`PrecomputeAllTables` 僅 2ms，瓶頸完全在 `Simulator::Run`。

**待評估的優化方向**：
- 降低 superframe 週期（250ms → 更長）
- 關閉未使用 beam 的 scheduler（只開 beam 1 等）

---

## 13. 設計決策參考

| 決策 | 文件 |
|------|------|
| Arbiter 機制取代 IP 層路由 | `Decisions/01_Arbiter mechanism replaces IP layer routing.md` |
| ISL 距離門檻設為 5000 km | `Decisions/02_ISL Distance Threshold.md` |
| Arbiter 預先建立（非排程內建立） | `Decisions/03_Arbiter lifecycle management.md` |
| Beam Scheduler 開銷根本原因 | `Decisions/04_Beam Scheduler.md` |

---

## 14. 驗證基準

### Layer 1 v5 Output（simTime=630s, slotInterval=60s）

```
LoadISLDefs: loaded 132 ISLs
InitOrbiterDevices: done
PrecomputeAllTables: start
  slot=0  t=0s:   SAT0_routes=65  ✓
  ...
  slot=11 t=660s: SAT0_routes=65  ✓
PrecomputeAllTables: complete
ScheduleRoutingUpdates: 12 events scheduled
ApplyRoutingTable: slot=0  t=0s   done
...
ApplyRoutingTable: slot=11 t=600s done
```

### Layer 2 BH KPI 目標（正式規格）

| KPI | 目標 |
|-----|------|
| throughput 提升 | +1.43% ~ +3.44% |
| 平均延遲降低 | -35.5% ~ -62.25% |
| JFI | ≥ 0.90 |
| dwell utilization | ≥ 85% |
| drop rate | < 0.5% |

### Layer 2 Phase 2 已驗證輸出（v2.1_bh-metrics.csv，2026-03-31）

| KPI | Phase 2 實測 | 說明 |
|-----|------------|------|
| hotspot dwell 分配 | beam1=240ms（+43% vs Phase 1 靜態） | EM 正確分配更多 slot 給 hotspot |
| 動態調度追蹤 | dwell 跟隨 T_cycle=60s 正弦變化 | beam3 高峰 288ms；各 beam 相位差 60° 可辨識 |
| 資源守恆 | 每 period 總 slots=38（M×K=19×2） | 無 slot 遺漏或重複 |
| slot_util_pct | 100% | OBC 正確執行 usable duration（T_s−T_sw=24ms） |
| throughput / delay / drop_rate | 0（預期） | Phase 3 封包路徑尚未實作，為預期行為 |

> Phase 3（SatGwCacheQueue + SatBhPrecoder）實作後，throughput / delay / drop_rate 才會有真實數值。

---

## 15. 版本對應

### Layer 1

| 版本 | 主要內容 |
|------|----------|
| v1 | non-OOP 原型，全域函式，驗證基本路由流程 |
| v2 | OOP 重構為 `IslRoutingManager`，NS3 Attribute 機制 |
| v3 | 效能優化 Fix 1–4，`UpdateLoadCosts` / `HasSignificantChange` / `RecomputeAffectedRoutes` 實作 |
| v4 | 計時拆解，確認效能瓶頸在 `Simulator::Run`，`BuildISLGraphWithLoad` 加入 |
| v5 | 新增 `ft-filter.h/.cc`（FtVisibilityFilter），新增 Layer 2/3 accessor（GetRouteCost 等），整合三層測試 |

### Layer 2

| Phase | 主要內容 | 狀態 |
|-------|----------|------|
| Phase 0 | `beam-hopping-manager.h/.cc`：簡化 prototype（dwell + switch），保留作 validator | ✅ 完成 |
| Phase 1 | `SatBhTimePlan` + `SatBhMetrics`：資料模型 + 被動 KPI；`ApplySyntheticSlot` 靜態驅動 | ✅ 完成 |
| Phase 2 | `SatBhScheduler`（EM only）+ `SatBhObc`（狀態機）+ `ApplySyntheticDemand`；`OnDemandReceived` 1-indexed bug fix | ✅ 完成（v2.1 驗證通過） |
| Phase 3 | `SatGwCacheQueue` + `SatBhPrecoder`（封包緩衝 + MMSE 預編碼） | ⏳ 待實作 |
| Phase 4 | `SatBhScheduler`（完整）+ `SatBhHelper`（統一安裝）+ SNS3 trace 接入（`ConnectTraces()`） | ⏳ 待實作 |
