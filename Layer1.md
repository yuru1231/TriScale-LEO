# Layer 1 — Topology & ISL Routing

## 目標

在 SNS3（ns-3 衛星模組）的 Iridium-66 星座場景中，實作基於 ISL（Inter-Satellite Link）的路由系統。

核心目標：
- 在模擬開始前離線預計算所有時槽的最短路徑路由表
- 在模擬期間定時切換路由表，適應星座拓樸變化
- 偵測 ISL 鏈路負載變化，必要時觸發局部路由重算（動態路由）
- 提供 FT（Feeder Terminal）可見性過濾，限制只有合約 FT pair 可以使用路由

---

## 架構概覽

```
isls.txt
   │
   ▼
LoadISLDefs()          ← 讀入 132 條 ISL 定義，建立 satellite→ifIndex 對應
   │
   ▼
InitOrbiterDevices()   ← 快取 66 顆衛星的 Node / SatOrbiterNetDevice / Arbiter
   │
   ▼
PrecomputeAllTables()  ← 離線計算所有時槽的路由表
   │  ├─ GetPositionsAt(τ_k)      → SGP4 查詢 66 顆衛星位置
   │  ├─ BuildISLGraph(pos)       → 過濾距離 > 5000km，建立帶 propagation_cost 的鄰接表
   │  ├─ ComputeBaseRoutes(graph) → 66× Dijkstra，每顆衛星計算 65 條路由
   │  └─ ApplyTiebreaker(routes, graphNext) → 等 cost 路徑優先選下個時槽仍 eligible 的 ISL
   │
   ▼
ScheduleRoutingUpdates() ← 排入 N 個 NS3 事件，每 slotInterval 秒觸發一次
   │
   ▼  (Simulator::Run 期間)
ApplyRoutingTable(slotIndex)
   │  ├─ UpdateLoadCosts()          → EMA 讀取 ISL queue delay，更新 load cost
   │  ├─ HasSignificantChange()     → 超過 changeThreshold 且冷卻期已過 → 觸發重算
   │  ├─ RecomputeAffectedRoutes()  → 只重算受影響 source 的路由（局部 Dijkstra）
   │  └─ ClearNextHopEntries() + AddNextHopEntry() → 更新 Arbiter
   │
   ▼
PrintStats()  ← 輸出每槽的 apply / recompute 耗時
```

---

## 核心資料結構

| 結構 | 說明 |
|------|------|
| `ISLEdge` | `{nodeB, propagation_cost, islIfIndexOnA, islIfIndexOnB}` — 一條有向 ISL 邊 |
| `ISLDef` | `{nodeA, nodeB}` — 靜態 ISL 定義（從 isls.txt 讀入） |
| `RouteEntry` | `{destSatId, nextHopSatId, islIfIndexOnA, cost}` — 一條路由表項目 |
| `ISLGraph` | `vector<vector<ISLEdge>>` — 66 個節點的鄰接表 |
| `RoutingTable` | `vector<vector<RouteEntry>>` — 66 顆衛星各自的路由表 |
| `SlotStats` | `{slotIndex, simTimeSec, applyWallMs, recomputeWallMs, recomputedSources, significantChange}` — 每槽執行統計 |

---

## 類別：IslRoutingManager

**位置**：`contrib/satellite/helper/isl-graph.h/.cc`

### Attributes（NS3 Config 可設定）

| Attribute | 型別 | 預設值 | 說明 |
|-----------|------|--------|------|
| `NumSatellites` | uint32 | 66 | 星座衛星總數 |
| `NumTimeSlots` | uint32 | 10 | 預計算時槽數 |
| `TimeSlotInterval` | double | 60.0 | 每槽間隔秒數 |
| `IslMaxDistanceKm` | double | 5000.0 | ISL 啟用距離門檻（km） |
| `IslsFilePath` | string | "" | isls.txt 完整路徑 |
| `EmaAlpha` | double | 0.3 | EMA 新樣本權重 |
| `ChangeThreshold` | double | 0.1 | 觸發重算的 load cost 變化比例 |
| `CooldownSeconds` | double | 30.0 | 兩次重算的最短間隔（秒） |
| `IslLinkRateBps` | double | 10e6 | ISL 鏈路速率（用於計算 queue delay） |

### 正確初始化順序

```cpp
Ptr<IslRoutingManager> mgr = CreateObject<IslRoutingManager>();
mgr->SetAttribute("NumSatellites",    UintegerValue(66));
mgr->SetAttribute("NumTimeSlots",     UintegerValue(numSlots));
mgr->SetAttribute("TimeSlotInterval", DoubleValue(slotInterval));
mgr->SetAttribute("IslMaxDistanceKm", DoubleValue(5000.0));
mgr->SetAttribute("IslsFilePath",     StringValue(islsFilePath));
mgr->SetAttribute("EmaAlpha",         DoubleValue(0.3));
mgr->SetAttribute("ChangeThreshold",  DoubleValue(0.1));
mgr->SetAttribute("CooldownSeconds",  DoubleValue(slotInterval / 2.0));
mgr->SetAttribute("IslLinkRateBps",   DoubleValue(10.0e6));

// 必須在 CreateSatScenario() 之後才能呼叫
mgr->Initialize(islsFilePath);
mgr->PrecomputeAllTables();
mgr->ScheduleRoutingUpdates();
// 接著執行 simHelper->RunSimulation()
```

### 公開方法

| 方法 | 說明 |
|------|------|
| `Initialize(islsFilePath)` | 讀 ISL 定義、快取衛星裝置、初始化 load cost 陣列 |
| `PrecomputeAllTables()` | 離線計算所有時槽路由表，存入 `m_tables` |
| `ScheduleRoutingUpdates()` | 排入 N 個 NS3 排程事件 |
| `ApplyRoutingTable(slotIndex)` | 更新 Arbiter（含負載感知重算邏輯） |
| `PrintStats()` | 輸出各槽執行統計 |

---

## 動態路由（負載感知）

每次 `ApplyRoutingTable()` 被觸發時（slot > 0）：

1. **`UpdateLoadCosts()`**：讀取每條 ISL 的 `DropTailQueue::GetNPackets()`，換算成 queue delay（bytes / linkRate），以 EMA 平滑更新 `m_loadCosts[a*N+b]`

2. **`HasSignificantChange()`**：比對 `m_loadCosts` 與 `m_prevLoadCosts`，任一 ISL 方向的變化超過 `m_changeThreshold`（且冷卻期已過）就回傳 true

3. **`RecomputeAffectedRoutes()`**：
   - 找出 load cost 變化超標的 ISL 邊
   - 透過 `m_islSources[edgeIdx]`（哪些 source 的路由經過此邊）找出受影響的 source 集合
   - 對這些 source 重跑 Dijkstra（使用 `BuildISLGraphWithLoad`，cost = propagation + load）
   - 更新 `m_tables[slotIndex]` 中對應的路由

4. **`RebuildIslSources()`**：根據最新路由表，重建 `m_islSources`（每條 ISL 邊被哪些 source 使用）

---

## 效能優化（v3 Fix 1–4）

| Fix | 問題 | 解法 | 效果 |
|-----|------|------|------|
| Fix 1 | 每槽重建相同圖 | `std::move(graphNext)` 作為下一槽的 `graphCurr` | BuildISLGraph 呼叫次數從 19 降為 10 |
| Fix 2 | 同一 τ_k 多次呼叫 SGP4 | 迴圈前預快取 66 顆衛星位置 | SGP4 呼叫從 2640 降為 660（4×） |
| Fix 3 | Dijkstra 後逆向追蹤第一跳 O(N²) | 鬆弛時同步傳播 `firstHopNode/If` | 路徑回溯從 O(N²) → O(1) per dest |
| Fix 4 | tiebreaker 用 `std::set` 查詢 O(log N) | 改 `unordered_set` + 自訂 hash | 查詢 O(1) 均攤 |

---

## SNS3 原始碼修改

| 檔案 | 位置 | 修改內容 |
|------|------|----------|
| `satellite-sgp4-mobility-model.h/.cc` | `contrib/satellite/model/` | 新增 `GetGeoPositionAt(Time t)` — 允許在任意 τ_k 查詢 SGP4 位置，不受 `Simulator::Now()` 限制 |
| `satellite-isl-arbiter-unicast.h` | `contrib/satellite/model/` | 新增 `ClearNextHopEntries()` — 原地清空路由表，無需重建 Arbiter 物件 |

---

## CMakeLists.txt 修改

**位置**：`contrib/satellite/CMakeLists.txt`

在 `source_files` 新增：
```cmake
helper/isl-graph.cc
helper/ft-filter.cc
```

在 `header_files` 新增：
```cmake
helper/isl-graph.h
helper/ft-filter.h
```

---

## Layer 1 Extension：FtVisibilityFilter

**位置**：`contrib/satellite/helper/ft-filter.h/.cc`

**功能**：基於 FT（Feeder Terminal）合約關係過濾可用路由，只允許有合約的 FT pair 之間建立路徑。

### 核心資料結構

| 結構 | 說明 |
|------|------|
| `FtDef` | `{ftId, latDeg, lonDeg, name}` — 一個地面站 |
| `FtTransitRoute` | `{srcFtId, dstFtId, slotIndex, srcSatId, dstSatId, transitCost}` — 一條可用的 FT 間路徑 |

### 公開方法

| 方法 | 說明 |
|------|------|
| `AddFt(id, lat, lon, name)` | 登記一個地面站 |
| `AddContractedPair(ftA, ftB)` | 登記合約 FT pair（雙向） |
| `SetRoutingManager(mgr)` | 綁定 IslRoutingManager |
| `SetElevationThreshold(deg)` | 設定最低仰角門檻（預設 5°） |
| `PrecomputeVisibility()` | 離線計算所有時槽下各 FT 可見哪些衛星，篩出合法路徑 |
| `PrintVisibilityReport()` | 輸出可見性報告 |
| `static ComputeElevationDeg(lat, lon, satPos)` | 靜態工具：計算 FT 對衛星的仰角（Layer 2 也會用到） |

---

## 已知問題

### Beam Scheduler 開銷（DEC-004）

**現象**：simTime=630s 的模擬 wall time 約 2760s（≈4.38× 放大）

**原因**：SNS3 DVB MAC beam scheduler 持續產生排程事件（66 顆衛星 × forward+return × superframe 250ms），即使沒有使用者流量也會產生大量事件。

**現況**：`PrecomputeAllTables` 僅 2ms，瓶頸完全在 `Simulator::Run`。

**待評估的優化方向**：
- 降低 superframe 週期（250ms → 更長）
- 關閉未使用 beam 的 scheduler（只開 beam 1 等）

---

## 設計決策參考

| 決策 | 文件 |
|------|------|
| Arbiter 機制取代 IP 層路由 | `Decisions/01_Arbiter mechanism replaces IP layer routing.md` |
| ISL 距離門檻設為 5000 km | `Decisions/02_ISL Distance Threshold.md` |
| Arbiter 預先建立（非排程內建立） | `Decisions/03_Arbiter lifecycle management.md` |
| Beam Scheduler 開銷根本原因 | `Decisions/04_Beam Scheduler.md` |

---

## 驗證基準（v3 Output）

執行 `simTime=630s, slotInterval=60s` 後預期輸出：

```
LoadISLDefs: loaded 132 ISLs
InitOrbiterDevices: done
PrecomputeAllTables: start
  slot=0  t=0s:   SAT0_routes=65  ✓
  slot=1  t=60s:  SAT0_routes=65  ✓
  ...
  slot=11 t=660s: SAT0_routes=65  ✓
PrecomputeAllTables: complete
ScheduleRoutingUpdates: 12 events scheduled
ApplyRoutingTable: slot=0  t=0s   done
ApplyRoutingTable: slot=1  t=60s  done
...
ApplyRoutingTable: slot=11 t=600s done
```

---

## 版本對應

| 版本 | 主要內容 |
|------|----------|
| v1 | non-OOP 原型，全域函式，驗證基本路由流程 |
| v2 | OOP 重構為 `IslRoutingManager`，NS3 Attribute 機制 |
| v3 | 效能優化 Fix 1–4，`UpdateLoadCosts` / `HasSignificantChange` / `RecomputeAffectedRoutes` 實作 |
| v4 | 測試腳本加入計時拆解，確認效能瓶頸在 `Simulator::Run`，`BuildISLGraphWithLoad` 加入 |
| v5 | 新增 `ft-filter.h/.cc`（FtVisibilityFilter），新增 Layer 2/3 accessor（GetRouteCost 等）|