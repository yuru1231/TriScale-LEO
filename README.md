# SNS3 預先排程路由架構 — Guide

> 只記當前有效的設計與接口。歷史決策與修改原因見 `Decisions/`。

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

| Layer | 職責 | 執行時機 |
|------|------|---------|
| Layer 1：離線預計算 | 批次計算 10 個時間點的拓樸與基礎路由表 | `Simulator::Run()` 之前，只執行一次 |
| Layer 2：執行期微調 | 讀取 device queue，判斷是否需要微調 Dijkstra | 每 60s，由 `Simulator::Schedule` 觸發 |
| Layer 3：封包轉發 | 封包到達節點後查 Arbiter 轉發 | 隨時，SNS3 Arbiter 機制處理 |

Layer 1 與 Layer 2 的接口：`map<Time, RoutingTableSnapshot>`，Layer 1 填好後 Layer 2 只讀不寫。

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

## 9. 需要修改的程式位置

| 檔案 | 修改內容 | 狀態 |
|------|---------|------|
| `satellite-sgp4-mobility-model.h/.cc` | 新增 `GetGeoPositionAt(Time t)` | ✅ 完成 |
| `contrib/satellite/helper/isl-graph.h/.cc` | `LoadISLDefs`、`BuildISLGraph` | ✅ 完成 |
| `satellite-isl-arbiter-unicast.h` | 新增 `ClearNextHopEntries()` | ✅ 完成 |
| `satellite/helper/satellite-routing-helper.h/.cc` | `PrecomputeAllTables`、`InitOrbiterDevices`、`ScheduleRoutingUpdates`、`ApplyRoutingTable` | ✅ 完成 |
| `scratch/my-simulation.cc` | 整合全部模組，加入實際流量 | ⏳ 待完成 |

---

## 10. 擴充點（預留）

- **BH 整合**：修改點只有 `BuildISLGraph()` 中讀取 `beamStatus` 的邏輯
- **增量更新**：修改點只有 `ApplyRoutingTable()` 改為 diff-based 寫入
