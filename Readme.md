# SNS3 預先排程路由架構 — Guide

> 只記當前有效的設計與接口。歷史決策與修改原因見 `Decisions/`。

---

## 1. 系統概述

### 1.1 核心主張

原生 SNS3 在每個更新週期執行完整的 O(N²) 拓樸重建加全量 Dijkstra。將這部分工作移至模擬開始前的離線預計算階段，執行期僅在 load cost 變化超過`ChangeThreshold`時，針對受影響節點進行局部 Dijkstra 重算，其餘時間直接套用離線路由表

### 1.2 與原生 SNS3 的對比

| 項目 | 原生 SNS3 | 本架構 |
|------|----------|--------|
| 拓樸重建 | 每 60s 重建，O(N²) | 離線一次完成，執行期不重建 |
| Dijkstra | 每次全量重算 | 當某條 ISL 的 load cost 變化比例超過`ChangeThreshold`，重算|
| load 感知 | 不支援 | 執行期讀`PointToPointIslNetDevice::GetQueue()->GetNPackets()`微調 |
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

### 1.4 縮寫對照表

| 縮寫 | 全名 |
|------|------|
| NCC | Network Control Center |
| OBC | On-Board Computer|
| FT | Fixed Terminal|
| EMA | Exponential Moving Average|
| EM | Estimated Metric（需求估算指標，BH Scheduler 用於虛擬流量計算） |
| WFQ | Weighted Fair Queuing|
| CRA | Constant Rate Assignment |
| RBDC | Rate-Based Dynamic Capacity|
| VBDC | Volume-Based Dynamic Capacity|
| MMSE | Minimum Mean Square Error|
| JFI | Jain's Fairness Index|

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

### Layer 1：ISL 接口

#### ISL 相關類別

| 類別 | 用途 |
|------|------|
| `PointToPointIslNetDevice` | ISL device，內含 `DropTailQueue<Packet>` |
| `SatOrbiterNetDevice` | 衛星主 device，管理所有 ISL device 與 Arbiter |
| `SatIslArbiterUnicast` | 實作類，內部為 `map<destSatId, islInterfaceIndex>` |

#### 各模組接口

| 模組 | SNS3 接口 |
|------|-----------|
| `BuildISLGraph(τ_k)` | `sat->GetObject<SatSGP4MobilityModel>()->GetGeoPositionAt(τ_k)` |
| `UpdateLoadCosts()` | `islDev->GetQueue()->GetNPackets()` |
| `ApplyRoutingTable()` | `orbDev->SetArbiter(newArbiter)` |
| `ApplyTable(τ_k)` | `Simulator::Schedule(Seconds(60), callback)` |

#### `InitOrbiterDevices()`（在 `Simulator::Run()` 前呼叫）

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

#### `ApplyRoutingTable()`（排程執行時呼叫）

```cpp
// 使用預先建立的 arbiter，原地清空後重填
Ptr<SatIslArbiterUnicast> arbiter = m_arbiters[satId];
arbiter->ClearNextHopEntries();

for (auto& entry : routingTable[satId])
{
    arbiter->AddNextHopEntry(entry.destSatId, entry.islInterfaceIndex);
}
```

注意：排程內不可呼叫 `CreateObject<SatIslArbiterUnicast>()`，在排程 callback 中缺少有效 Node context，會導致 Arbiter 初始化失敗並觸發 NS_FATAL。詳見 `decisions/DEC-003-arbiter-lifecycle.md`。

#### `BuildISLGraph()` 效能注意事項

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

#### `UpdateLoadCosts()` 實作

```cpp
std::vector<Ptr<PointToPointIslNetDevice>> islDevs =
    orbDev->GetIslsNetDevices();

for (uint32_t i = 0; i < islDevs.size(); i++)
{
    uint32_t queuePackets = islDevs[i]->GetQueue()->GetNPackets();
    // 套用 EMA，計算 load_cost
}
```

#### `ifIndex` 對應方式

不依賴 `isls.txt` 順序，改用 peer nodeId 查 vector index：

```cpp
for (uint32_t j = 0; j < islDevs.size(); j++) {
    Ptr<Node> peer = islDevs[j]->GetDestinationNode();
    if (peer) peerNodeIdToIfIdx[i][peer->GetId()] = j;
}
```

### Layer 2：BH 接口

#### BH 核心時間參數

| 符號 | 定義 | 數值 |
|------|------|------|
| `T_sf` | DVB-S2X Super-frame | 26.5 ms |
| `T_s` | BH Time Slot | 26.5 ms |
| `T_p` | BHTP period | 503 ms |
| `T_sw` | beam switching time | 2 ms |
| `T_prop` | 指令下發延遲 | 10 ms（建議） |
| `K` | 同時活動 beam 數 | 3（預設） |

#### BH 非侵入式擴充原則

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
| Beam 狀態 | 假設所有 beam 皆為可用 |

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

### Layer 1 Config 參數（NS3 Attribute）

所有參數均透過 NS3 Attribute 機制設定，可在 `v5_test-iridium.cc` 中以 `mgr->SetAttribute(...)` 覆蓋預設值。

| Attribute 名稱 | 成員變數 | 型別 | 預設值 | 說明 |
|----------------|---------|------|--------|------|
| `NumSatellites` | `m_numSatellites` | `uint32` | `66` | 星座衛星總數 |
| `IslMaxDistanceKm` | `m_islMaxDistanceKm` | `double` | `5000.0` | ISL 連接距離上限（km）；超過此值的 ISL 在建圖時被排除（`eligible = false`） |
| `NumTimeSlots` | `m_numTimeSlots` | `uint32` | `10` | 離線預計算的時間槽總數 |
| `TimeSlotInterval` | `m_timeSlotInterval` | `double` | `60.0` | 時間槽間隔（秒）；同時決定 `ScheduleRoutingUpdates` 的觸發週期 |
| `IslsFilePath` | `m_islsFilePath` | `string` | `""` | `isls.txt` 完整路徑；`Initialize()` 時呼叫 `LoadISLDefs()` 讀入 |
| `EmaAlpha` | `m_emaAlpha` | `double` | `0.3` | EMA（指數移動平均）新樣本權重（0 < α ≤ 1）；值越大 load cost 對瞬時 queue 越敏感 |
| `ChangeThreshold` | `m_changeThreshold` | `double` | `0.1` | 觸發 `RecomputeAffectedRoutes` 的 load cost 相對變化門檻（10%）；低於此值不重算 |
| `CooldownSeconds` | `m_cooldownSeconds` | `double` | `30.0` | 兩次 partial recompute 之間的最短間隔（秒）；防止 load 震盪導致頻繁切換 |
| `IslLinkRateBps` | `m_islLinkRateBps` | `double` | `10.0e6` | ISL 鏈路速率（bps）；用於將 queue packet count 換算為 load delay（`bits / rate`） |

#### 硬編碼常數（非 Attribute，修改需直接改 code）

| 常數 | 數值 | 所在函式 | 說明 |
|------|------|---------|------|
| 光速 `C` | `3×10⁸ m/s` | `BuildISLGraph()`、`BuildISLGraphWithLoad()` | 傳播延遲計算基準：`propCost = dist / C` |
| 假設封包大小 | `1500 bytes` | `GetLinkQueueDelay()` | load cost 換算：`bits = nPackets × 1500 × 8` |
| cost 權重 α, β | 各為 `1.0`（隱含） | `BuildISLGraphWithLoad()` | `total_cost = propCost + loadCost`，兩者等權，無獨立 weight 欄位 |

#### 參數調整的連動效應

| 調整項目 | 直接效果 | 注意事項 |
|---------|---------|---------|
| `EmaAlpha` ↑ | load cost 對瞬時壅塞更敏感，切換更快 | 與 `CooldownSeconds` 搭配；單獨調高易造成震盪 |
| `ChangeThreshold` ↓ | 更容易觸發 `RecomputeAffectedRoutes` | 計算開銷增加，建議搭配 `CooldownSeconds` 限制頻率 |
| `CooldownSeconds` ↑ | 降低 recompute 頻率，穩定性提升 | 副作用：ISL 失效後最長延遲 `CooldownSeconds` 才能恢復（見 Readme 第 12 節） |
| `NumTimeSlots` ↑ | 預計算解析度提升 | 記憶體與初始化時間線性增加；`TimeSlotInterval` × `NumTimeSlots` = 總覆蓋秒數 |
| `IslMaxDistanceKm` ↑ | 連通性增強，可能建立長延遲 ISL | 超過 ~5500 km 後 Iridium 星座中可能出現不穩定跨面連結 |

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

#### 資料模組總覽

| 資料結構 | 職責 | 關鍵欄位 |
|---------|------|---------|
| `ISLEdge` | 一條有向 ISL 邊（鄰接表的節點） | `nodeB`, `propagation_cost`, `islIfIndexOnA`, `islIfIndexOnB` |
| `ISLDef` | 靜態 ISL 定義，從 `isls.txt` 讀入 | `nodeA`, `nodeB` |
| `RouteEntry` | 一條路由表項目（目的地 → 下一跳 + 出口介面） | `destSatId`, `nextHopSatId`, `islIfIndexOnA`, `cost` |
| `ISLGraph` | 66 節點的鄰接表（`vector<vector<ISLEdge>>`） | — |
| `RoutingTable` | 66 顆衛星各自的路由表（`vector<vector<RouteEntry>>`） | — |
| `SlotStats` | 每槽執行統計 | `slotIndex`, `simTimeSec`, `applyWallMs`, `recomputeWallMs`, `recomputedSources`, `significantChange` |
| `GwDef` | 地面閘道器定義 | `gwId`, `latDeg`, `lonDeg`, `name` |
| `GwToGwRoute` | GW→GW 最佳路由結果（一對 GW 在一個時槽） | `srcGwId`, `dstGwId`, `entrySatId`, `exitSatId`, `satPath`, `islCost`, `valid` |
| `UtDef` | 使用者終端定義 | `utId`, `latDeg`, `lonDeg`, `name` |
| `GwToUtRoute` | GW→UT 最佳路由結果（一對 GW–UT 在一個時槽） | `gwId`, `utId`, `entrySatId`, `servingSatId`, `satPath`, `islCost`, `valid` |

#### 完整結構定義

```cpp
struct ISLEdge {
    uint32_t nodeB;
    double   propagation_cost;
    uint32_t islIfIndexOnA;          // A 端出口 ifIndex
    uint32_t islIfIndexOnB;          // B 端出口 ifIndex（tiebreaker 用）
};

struct RouteEntry {
    uint32_t destSatId;
    uint32_t nextHopSatId;
    uint32_t islIfIndexOnA;          // GetIslsNetDevices() vector index
    double   cost;
};

struct GwDef {
    uint32_t    gwId;
    double      latDeg, lonDeg;
    std::string name;
};

struct GwToGwRoute {
    uint32_t              srcGwId, dstGwId;
    uint32_t              entrySatId, exitSatId;
    std::vector<uint32_t> satPath;
    double                islCost;
    bool                  valid;
};

struct UtDef {
    uint32_t    utId;
    double      latDeg, lonDeg;
    std::string name;
};

struct GwToUtRoute {
    uint32_t              gwId, utId;
    uint32_t              entrySatId, servingSatId;
    std::vector<uint32_t> satPath;
    double                islCost;
    bool                  valid;
};

struct SlotStats {
    uint32_t slotIndex;
    double   simTimeSec;
    long     applyWallMs, recomputeWallMs;
    uint32_t recomputedSources;
    bool     significantChange;
};
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
| EMA | `smoothed = (1 - α) × previous + α × current`  α=0.3|
| Hysteresis | 新路徑 cost 低於現有路徑 δ（建議 0.5×avg_prop）允許切換 |
| Cooldown | 同一 ISL 切換後 60s 內不再切換 |

---

## 9. 程式檔案狀態

### Layer 1：ISL Routing

| 檔案 | 修改內容 | 狀態 |
|------|---------|------|
| `satellite-sgp4-mobility-model.h/.cc` | 新增 `GetGeoPositionAt(Time t)` | ✅ 完成 |
| `satellite-isl-arbiter-unicast.h` | 新增 `ClearNextHopEntries()` | ✅ 完成 |
| `isl-graph.h/.cc` | 完整 ISL routing，load-aware，tiebreaker；GW/UT 路由（v6/v7）；`GetNumTimeSlots()`、`GetNumSatellites()`、`GetTimeSlotInterval()`、`GetRouteCost()`；`PrecomputeGwRoutes`、`PrecomputeGwUtRoutes` | ✅ 完成（v7） |

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


### Layer 3：QoS（純配置）

| 位置 | 內容 | 狀態 |
|------|------|------|
| `v5_test-iridium.cc` `ConfigureQoS()` | Class A (CRA)、Class B (RBDC)、Class C (VBDC) SNS3 attribute config | ✅ 架構完成，attribute 路徑待驗證 |

### 整合測試

| 檔案 | 內容 | 狀態 |
|------|------|------|
| `test-iridium.cc` | Layer 1 三模式（sat2sat / gw2gw / gw2ut）整合測試；GW preset：TW-Taipei(0)、JP-Tokyo(1)、US-SanFrancisco(2) | ✅ 完成（v7 已驗證） |

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

| 決策 | 原因 | 文件 |
|------|------|------|
| Arbiter 機制取代 IP 層路由 | SNS3 ISL 封包轉發繞過 IP 層，`Ipv4StaticRouting` 無效；轉發由 `SatIslArbiterUnicast` 查 `map<destSatId, islInterfaceIndex>` 決定出口 | `Decisions/01_Arbiter mechanism replaces IP layer routing.md` |
| ISL 距離門檻設為 5000 km | Iridium 跨軌道面 ISL 瞬時距離可達 ~4800 km；2500 km（官方平均值）會導致拓樸圖斷裂，部分衛星無法連通 | `Decisions/02_ISL Distance Threshold.md` |
| Arbiter 預先建立（非排程內建立） | 排程內呼叫 `CreateObject<SatIslArbiterUnicast>()` 因 default constructor 缺少衛星節點指標，物件處於無效狀態，觸發 fatal crash | `Decisions/03_Arbiter lifecycle management.md` |
| Beam Scheduler 開銷根本原因 | SNS3 DVB MAC scheduler 無流量時仍持續排程（66 顆 × forward/return × superframe 250ms），event count 超過 `UINT32_MAX`，`Simulator::Run` 佔 99.9% wall time | `Decisions/04_Beam Scheduler.md` |

---

## 14. 驗證基準

### Layer 1 v7 Output（simTime=630s, slotInterval=60s, numSlots=11）

配置：`[CFG] mode=<mode> simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600`

通用基準（三個模式相同）：

```
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | PrecomputeAllTables: complete | wall=4–5ms
[CHKPT] 0s | ScheduleRoutingUpdates: 11 events scheduled
```

#### mode=sat2sat（SAT0→SAT33）

```
./ns3 run "scratch/test-iridium --mode=sat2sat --satSrc=0 --satDst=33"
```

| slot 0–4 | `0->1->2->57->46->35->34->33`（7 hops）cost 0.078→0.069 |
| slot 5–10 | `0->1->56->45->34->33`（5 hops）cost 0.066→0.048  ← PATH CHANGED @ slot=5 |

Wall time: 2722.79s

#### mode=gw2gw（TW-Taipei↔JP-Tokyo）

```
./ns3 run "scratch/test-iridium --mode=gw2gw --gwSrc=0 --gwDst=1"
```

| slot 0–4 | entry/exit=SAT15 |
| slot 5–6 | entry/exit=SAT44  ← ROUTE CHANGED @ slot=5 |
| slot 7–10 | entry/exit=SAT14  ← ROUTE CHANGED @ slot=7 |

雙向對稱，isl_cost=0.0。Wall time: 2403.38s

#### mode=gw2ut（TW-Taipei → UT-Taipei）

```
./ns3 run "scratch/test-iridium --mode=gw2ut --gwId=0 --utId=0 --utLatDeg=25.0330 --utLonDeg=121.5654 --utName=UT-Taipei"
```

| slot 0–5 | entry/serving=SAT15（UT slot=5 仍有 2 顆可見衛星） |
| slot 6 | entry/serving=SAT44  ← ROUTE CHANGED @ slot=6（比 gw2gw 晚一槽） |
| slot 7–10 | entry/serving=SAT14  ← ROUTE CHANGED @ slot=7 |

Wall time: 2765.45s

#### mode=gw2gw（TW-Taipei↔US-SanFrancisco，長距離跨太平洋）

```
./ns3 run "scratch/test-iridium --mode=gw2gw --gwSrc=0 --gwDst=2"
```

| slot 0–1 | entry=SAT15 / exit=SAT37 / path=`15->14->25->36->37` / isl_cost≈0.044–0.046s |
| slot 2 | entry=SAT15 / exit=SAT1 / path=`15->14->13->2->1` ← ROUTE CHANGED |
| slot 3–5 | entry=SAT15 / exit=SAT36 / path=`15->14->25->36` ← ROUTE CHANGED @ slot=3 |
| slot 6 | entry=SAT44 / exit=SAT36 / path=`44->45->46->35->36` ← ROUTE CHANGED |
| slot 7–8 | entry=SAT14 / exit=SAT36 / path=`14->13->24->35->36` ← ROUTE CHANGED @ slot=7 |
| slot 9–10 | entry=SAT44 / exit=SAT0 / path=`44->45->56->1->0` ← ROUTE CHANGED @ slot=9 |

雙向對稱，isl_cost 範圍 0.0377～0.0476s，ISL 跳數 4～5 跳，共 5 次路徑切換（vs TW→JP 的 2 次）。

#### mode=gw2ut（TW-Taipei → UT-SanFrancisco，長距離跨太平洋）

```
./ns3 run "scratch/test-iridium --mode=gw2ut --gwId=0 --utId=1 --utLatDeg=37.8 --utLonDeg=-122.4 --utName=UT-SanFrancisco"
```

| slot 0–1 | entry=SAT15 / serving=SAT37 / path=`15->14->25->36->37` / isl_cost≈0.044–0.046s |
| slot 2 | entry=SAT15 / serving=SAT1 / path=`15->14->13->2->1` ← ROUTE CHANGED |
| slot 3–5 | entry=SAT15 / serving=SAT36 / path=`15->14->25->36` ← ROUTE CHANGED @ slot=3 |
| slot 6 | entry=SAT44 / serving=SAT36 / path=`44->45->46->35->36` ← ROUTE CHANGED |
| slot 7–8 | entry=SAT14 / serving=SAT36 / path=`14->13->24->35->36` ← ROUTE CHANGED @ slot=7 |
| slot 9–10 | entry=SAT44 / serving=SAT0 / path=`44->45->56->1->0` ← ROUTE CHANGED @ slot=9 |

與 gw2gw TPE→SF 路徑和切換時機完全一致（UT@SF 座標 = GW2@SF 座標，serving = exit 全程）。Wall time: 2921.66s

### Layer 2 BH KPI 目標

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
| v5 | 新增 `ft-filter.h/.cc`（FtVisibilityFilter），新增 Layer 2/3 accessor（`GetRouteCost` 等），整合三層測試 |
| v6 | 新增 GW-to-GW 路由（`GwDef`、`GwToGwRoute`、`PrecomputeGwRoutes`、`PrintGwRouteReport`）；GW 可見性以仰角 >5° 篩選；Report v6 格式（entry / ISL_path / exit / isl_cost） |
| v7 | 新增 GW-to-UT 路由（`UtDef`、`GwToUtRoute`、`PrecomputeGwUtRoutes`、`PrintGwUtRouteReport`）；複用 GW 可見性；Report v7 格式（增加 serving 欄）；test-iridium.cc 支援三模式 CLI 切換 |

### Layer 2

| Phase | 主要內容 | 狀態 |
|-------|----------|------|
| Phase 0 | `beam-hopping-manager.h/.cc`：簡化 prototype（dwell + switch），保留作 validator | ✅ 完成 |
| Phase 1 | `SatBhTimePlan` + `SatBhMetrics`：資料模型 + 被動 KPI；`ApplySyntheticSlot` 靜態驅動 | ✅ 完成 |
| Phase 2 | `SatBhScheduler`（EM only）+ `SatBhObc`（狀態機）+ `ApplySyntheticDemand`；`OnDemandReceived` 1-indexed bug fix | ✅ 完成（v2.1 驗證通過） |
| Phase 3 | `SatGwCacheQueue` + `SatBhPrecoder`（封包緩衝 + MMSE 預編碼） | ⏳ 待實作 |
| Phase 4 | `SatBhScheduler`（完整）+ `SatBhHelper`（統一安裝）+ SNS3 trace 接入（`ConnectTraces()`） | ⏳ 待實作 |
