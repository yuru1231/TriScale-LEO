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
| 衛星數量 | 66 顆 |
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
| **Layer 2：Beam Hopping** | 決定特定衛星在可視時間內的 beam 服務順序 | Superframe 級（250ms） | 離線預計算排程；執行期每 superframe 觸發 |
| **Layer 3：QoS Scheduling** | 在給定 beam 服務時間內對 UE 做 priority + WFQ 排程 | 封包級 | SNS3 `SatBeamScheduler` 原生處理 |

### 2.2 Layer 1 內部子層（ISL Routing 模組原有設計）

| 子層 | 職責 | 執行時機 |
|------|------|---------|
| 離線預計算 | 批次計算各時間點的拓樸與基礎路由表 | `Simulator::Run()` 之前 |
| 執行期微調 | 讀取 device queue，判斷是否需要微調 Dijkstra | 每 60s，`Simulator::Schedule` 觸發 |
| 封包轉發 | 封包到達節點後查 Arbiter 轉發 | 隨時，SNS3 Arbiter 機制處理 |

### 2.3 層間接口

```
Layer 1 (IslRoutingManager)
  → 輸出：m_tables[slotIndex][satId] → vector<RouteEntry>
  → FtVisibilityFilter 讀取 GetRouteCost(entry, exit, slot) 做篩選

Layer 1 extension (FtVisibilityFilter)
  → 輸出：GetBestTransit(ftI, ftJ, slot) → FtTransitRoute{entrySat, exitSat, cost}
  → 告知 Layer 2 哪些衛星是 contracted path 上的 transit nodes

Layer 2 (BeamHoppingManager)
  → 輸出：BhEvent{t, satId, cellId} 序列
  → GetCurrentCell(satId) 供 Layer 3 context 使用

Layer 3 (SNS3 native)
  → 讀取 GetCurrentCell()，在對應 beam 的服務時間內依 QoS 優先序排程
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

---

## 4. ISL 連接條件

| 條件 | 數值 |
|------|------|
| 距離上限 | 5000 km |
| 仰角門檻 | 不適用（僅用於 UT/GW 鏈路） |
| Beam 狀態 | 第一階段恆為 true |

---

## 5. Cost Function

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

---

## 6. 執行順序

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

---

## 7. 資料結構

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

---

## 8. 執行期震盪抑制

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
<<<<<<< ours
| `contrib/satellite/helper/isl-graph.h/.cc` | `LoadISLDefs`、`BuildISLGraph`、`ComputeBaseRoutes`、`ApplyTiebreaker`；v3 效能優化（位置快取、滾動圖、O(1) 第一跳、unordered_set） | ✅ 完成 |
| `satellite-isl-arbiter-unicast.h` | 新增 `ClearNextHopEntries()` | ✅ 完成 |
| `contrib/satellite/helper/isl-graph.cc` | `UpdateLoadCosts`、`HasSignificantChange`、`RecomputeAffectedRoutes`、`RebuildIslSources`、`GetLinkQueueDelay`、`BuildISLGraphWithLoad`；`ApplyRoutingTable` 改為 `ClearNextHopEntries`（DEC-003） | ✅ 完成 |
| `contrib/satellite/helper/isl-graph.h` | `m_loadCosts`、`m_prevLoadCosts`、`m_islSources`、`m_arbiters`、`m_edgeOfPair`、`m_lastRecomputeTime`、`SlotStats` | ✅ 完成 |
| `scratch/my-simulation.cc` | 整合全部模組，加入實際流量 | ⏳ 待完成 |
=======
| `isl-graph.h/.cc`（v4） | 完整 ISL routing，load-aware，tiebreaker | ✅ 完成 |
| `isl-graph.h`（v4 新增） | `GetNumTimeSlots()`、`GetNumSatellites()`、`GetTimeSlotInterval()`、`GetRouteCost()` | ✅ 完成 |
| `satellite-isl-arbiter-unicast.h` | `ClearNextHopEntries()` | ✅ 完成 |

### Layer 1 extension：FT Visibility Filter

| 檔案 | 內容 | 狀態 |
|------|------|------|
| `ft-filter.h` | `FtDef`、`FtTransitRoute`、`FtVisibilityFilter` 介面 | ✅ 完成 |
| `ft-filter.cc` | `PrecomputeVisibility()`、`GetAccessSats()`、`GetBestTransit()`、`ComputeElevationDeg()` | ✅ 完成 |

### Layer 2：Beam Hopping Manager

| 檔案 | 內容 | 狀態 |
|------|------|------|
| `beam-hopping-manager.h` | `CellDef`、`BhEvent`、`TrafficDemandProvider`、`BhSwitchCallback`、`BeamHoppingManager` 介面 | ✅ 完成 |
| `beam-hopping-manager.cc` | `ComputeBhSchedule()`（離線）、`ScheduleBhUpdates()`（執行期）、`ApplyBhEvent()`（SNS3 注入點 TODO） | ✅ 架構完成，SNS3 注入點待確認 |

### Layer 3：QoS（純配置）

| 位置 | 內容 | 狀態 |
|------|------|------|
| `v5_test-iridium.cc` `ConfigureQoS()` | Class A (CRA)、Class B (RBDC)、Class C (VBDC) SNS3 attribute config | ✅ 架構完成，attribute 路徑待驗證 |

### 整合測試

| 檔案 | 內容 | 狀態 |
|------|------|------|
| `v5_test-iridium.cc` | 三層完整整合，FT 台灣/日本/美國，4 個台灣 cell | ✅ 完成（BH 注入點 stub） |
>>>>>>> theirs

---

## 10. 待確認事項（blocking）

| 項目 | 說明 | 位置 |
|------|------|------|
| **SNS3 BH 注入 API** | `ApplyBhEvent()` 需要呼叫正確的 SNS3 method 切換 beam。候選：`SatOrbiterFeederMac`、`SatBeamScheduler`、或 `SatBeamHelper` 動態 enable/disable | `beam-hopping-manager.cc` 標記 `TODO SNS3_BH_INJECT` |
| **台灣 beam ID** | `v5_test-iridium.cc` 目前用 `SetBeamSet({1})`，需從 scenario 資料確認涵蓋台灣（25°N 121°E）的 beam ID | `v5_test-iridium.cc` 標記 `TODO BH` |
| **Layer 3 QoS attribute 路徑** | `SatLowerLayerServiceConf::DaService*` 的 attribute key 需對照安裝的 SNS3 版本確認 | `v5_test-iridium.cc` `ConfigureQoS()` |
| **TrafficDemandProvider 實作** | 目前用 `UniformDemandProvider`（均勻分配）。真實流量輸入介面已預留，待 LEO topo 完成後接入 | `beam-hopping-manager.h` `TrafficDemandProvider` 介面 |

## 11. 擴充點（預留）

- **增量更新**：`ApplyRoutingTable()` 改為 diff-based 寫入
- **真實流量輸入**：實作 `TrafficDemandProvider`，從 UE queue 或外部預測資料注入 cell demand
- **多 FT 負載平衡**：`GetBestTransit()` 可擴充為考慮 FT 當前負載的多路由選擇
