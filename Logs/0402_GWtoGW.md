# 工作日誌 2026-04-02

## 目標
執行 ISL Routing v6 三種路由模式（gw2gw、gw2ut、sat2sat）的模擬驗證，確認 Iridium 66 星座下動態切換行為正確。

---

## 完成事項

### 1. gw2gw 路由驗證（TW-Taipei ↔ JP-Tokyo）

**現象**：TW-Taipei（GW0）↔ JP-Tokyo（GW1）雙向路由在 11 個時槽中出現兩次 ROUTE CHANGED，分別於 t=300s 與 t=420s 發生衛星切換。

**原因**：Dijkstra 以 ISL propagation cost 為權重動態計算最短路徑。t=300s 星座移動後，服務衛星由 sat=15 切換至 cost 更低的 sat=44；t=420s 再切換至 sat=14。isl_cost 始終為 0.0s，代表雙城距離近到同一顆衛星即可中繼，無需跨 ISL。

**修正**：無需修正，驗證既有實作。

**驗證**：
```
slot 0–4  (0–240s)  : entry=15 exit=15 isl_cost=0.000000
slot 5–6  (300–360s): entry=44 exit=44 isl_cost=0.000000  <-- ROUTE CHANGED (t=300s)
slot 7–10 (420–600s): entry=14 exit=14 isl_cost=0.000000  <-- ROUTE CHANGED (t=420s)
```
雙向（TW-Taipei→JP-Tokyo 與 JP-Tokyo→TW-Taipei）路由完全對稱，切換時間點一致 ✅

---

### 2. gw2ut 路由驗證（TW-Taipei → UT-Taipei）

**現象**：GW0（TW-Taipei）→ UT-Taipei（lat=25.033, lon=121.565）路由共發生兩次 ROUTE CHANGED，第一次在 t=360s（slot 6），比 gw2gw 晚一個時槽。

**原因**：gw2ut 模式中，路徑出口判定依據 UT 可見衛星集合而非目標 GW 的可見衛星。由於 UT 位置與 GW0 台北位置相近，t=300s 時 UT 仍由 sat=15 服務（GwUtRouting 顯示 slot=5 為 UT0=2sats），Dijkstra 選擇不切換；至 t=360s（slot=6，UT0=1sat）才切換至 sat=44。

**修正**：無需修正，驗證既有實作。

**驗證**：
```
slot 0–5  (0–300s)  : entry=15 serving=15 isl_cost=0.000000
slot 6    (360s)    : entry=44 serving=44 isl_cost=0.000000  <-- ROUTE CHANGED
slot 7–10 (420–600s): entry=14 serving=14 isl_cost=0.000000  <-- ROUTE CHANGED
```
slot 7 apply=4ms、recompute=4ms 為全測試峰值，仍在可接受範圍 ✅

---

### 3. sat2sat 路由驗證（SAT0 → SAT33）

**現象**：SAT0 → SAT33 路徑於 t=300s（slot 5）由 7 hop 縮短為 5 hop，且 route_cost 自 0.078 持續下降至 0.048，中途無再次切換。

**原因**：t=300s 前，最短路徑為 0→1→2→57→46→35→34→33（cost 0.078→0.069，衛星漸近但仍 7 hop 最優）。t=300s 後，5 hop 路徑 0→1→56→45→34→33 的 propagation cost（0.066）低於 7 hop 路徑，Dijkstra 切換至新路徑。此後衛星持續相互靠近，cost 單調遞減至 0.048。

**修正**：無需修正，驗證既有實作。

**驗證**：
```
slot 0–4 (0–240s)  : 0->1->2->57->46->35->34->33  cost 0.078→0.069 (7 hop)
slot 5   (300s)    : 0->1->56->45->34->33          cost 0.066        <-- PATH CHANGED
slot 6–10(360–600s): 0->1->56->45->34->33          cost 0.062→0.048 (5 hop, 持續下降)
```
路徑縮短且 cost 單調遞減，反映星座軌道靠近行為正確 ✅

---

### 4. 局部 Dijkstra 重算行為確認（三模式共通）

**現象**：三組測試的 `IslRoutingManager Stats` 顯示 recomputedSrc 從 slot 1 的 3 增長到 slot 10 的 50，且每個 slot（除 slot 0 外）均觸發 `HasSignificantChange=YES`。

**原因**：`RecomputeAffectedRoutes()` 採局部 BFS 擴散機制，只重算鄰近 ISL 狀態改變的 source 衛星，而非全量 66 顆重算。隨著模擬時間推進，累積受影響的 source 數量逐漸增加，但始終未觸發全量重算（66/66）。

**修正**：無需修正，驗證既有實作。

**驗證**：gw2gw 與 sat2sat 的 recompSrc 序列完全一致（3→13→20→25→26→36→41→44→46→50），反映兩者使用相同星座與 ISL 定義、slot 0 初始化方式相同 ✅

| slot | gw2gw recompSrc | sat2sat recompSrc | 一致 |
|------|-----------------|-------------------|------|
| 1    | 3               | 3                 | ✅   |
| 5    | 26              | 26                | ✅   |
| 10   | 50              | 50                | ✅   |

---

### 5. Wall Time 符合 DEC-004 已知行為

**現象**：三組測試 wall time 分別為 gw2gw=2403.38s、gw2ut=2765.45s、sat2sat=2722.79s，約為 simTime=630s 的 3.8–4.4 倍。

**原因**：SNS3 DVB MAC beam scheduler 固有開銷（DEC-004 已記錄）。`PrecomputeAllTables` wall=4–5ms，`ApplyRoutingTable` 各槽 0–4ms，ISL 路由邏輯本身不是瓶頸，wall time 差異完全來自 DVB MAC 層。

**修正**：無需修正，確認為已知行為。

**驗證**：
```
gw2gw  wall=2403.38s (≈3.8×), PrecomputeAllTables=5ms
gw2ut  wall=2765.45s (≈4.4×), PrecomputeAllTables=4ms
sat2sat wall=2722.79s (≈4.3×), PrecomputeAllTables=5ms
```
三組均在 DEC-004 已知範圍內 ✅

→ 若有重要設計決策，見 `Decisions/DEC-004-Beam Scheduler.md`

---

## 驗證結果總表

| 驗證項目 | 觀察輸出 | 結論 |
|----------|----------|------|
| gw2gw 路由切換（TW-Taipei↔JP-Tokyo） | t=300s sat=15→44，t=420s sat=44→14，雙向對稱 | 動態切換正確 ✅ |
| gw2ut 路由切換（TW-Taipei→UT-Taipei） | t=360s sat=15→44（較 gw2gw 晚一槽），t=420s sat=44→14 | UT 可見衛星切換時機合理 ✅ |
| sat2sat 路徑縮短（SAT0→SAT33） | t=300s 由 7 hop 縮為 5 hop，cost 0.078→0.048 單調遞減 | 路徑最優化正確 ✅ |
| 局部 Dijkstra 重算 | recompSrc slot1=3、slot10=50，未觸發全量 66/66 | 效能符合預期 ✅ |
| gw2gw 與 sat2sat recompSrc 一致性 | 兩者序列完全相同（3→13→...→50） | 星座初始化一致 ✅ |
| gw2ut apply/recompute 峰值 | slot7 apply=4ms，recompute=4ms | 效能在可接受範圍 ✅ |
| Wall time（三組） | 2403–2766s（3.8–4.4×），DVB MAC 開銷 | 符合 DEC-004 已知行為 ✅ |

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Outputs\gw2gw.md` | 新增 gw2gw 模式完整執行輸出（v6 report，simTime=630s） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Outputs\gw2ut.md` | 新增 gw2ut 模式完整執行輸出（v7 report，simTime=630s） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Outputs\sat2sat.md` | 新增 sat2sat 模式完整執行輸出，含完整路徑序列） |

> 今日無程式碼修改，僅執行驗證並記錄輸出結果。

---

## 備註

- gw2ut 輸出標示 Report version=v7，gw2gw 與 sat2sat 標示 v6；版號差異反映 report 格式小幅調整（新增 `serving` 欄位），功能邏輯相同。
- gw2gw 輸出中 ISL_path 欄位僅顯示單顆中繼衛星（entry=exit），代表 TW-Taipei 與 JP-Tokyo 在各槽均由同一顆衛星直接服務，無跨 ISL 傳遞段。
- sat2sat 測試中 route_cost 為 propagation delay（秒），反映實際星間傳播延遲，與 gw2gw/gw2ut 的 isl_cost 定義一致。

---

## 程式碼
 [`isl-graph.h`]()\
 [`isl-graph.cc`]()\
 [`test-iridium.cc`]()

### isl-graph.h — 資料結構與介面

| 結構 | 說明 |
|------|------|
| `ISLEdge` / `ISLDef` | ISL 圖的邊與靜態定義 |
| `RouteEntry` | 單條路由表項目（dest、nextHop、ifIndex、cost） |
| `SlotStats` | 每槽的 apply/recompute wall time 統計 |
| `GwDef` / `GwToGwRoute` | v6：GW 定義與路由結果（entrySat → ISL path → exitSat） |
| `UtDef` / `GwToUtRoute` | v7：UT 定義與路由結果（entrySat → ISL path → servingSat） |

`IslRoutingManager` 繼承 `ns3::Object`，透過 ns-3 Attribute 機制管理所有參數（NumSatellites、IslMaxDistanceKm、NumTimeSlots 等 8 個 Attribute）。

### isl-graph.cc — 核心流程

**初始化與預計算：**
```
Initialize()
  ├─ LoadISLDefs()          讀 isls.txt → 132 ISL，建立 m_perSatISLOrder / m_edgeOfPair
  └─ InitOrbiterDevices()   從 SatTopology 取 66 顆衛星，預建 66 個 SatIslArbiterUnicast

PrecomputeAllTables()
  └─ 逐 slot：ComputeBaseRoutes(graph) + ApplyTiebreaker(graphNext)
     → m_tables[k] 存 11 個 slot 的完整 routing table
```

**Runtime 更新（每槽觸發）：**
```
ApplyRoutingTable(slotIndex)
  ├─ UpdateLoadCosts()         EMA 更新各 ISL 的 queue delay
  ├─ HasSignificantChange()    判斷是否超過 changeThreshold（cooldown 期間直接略過）
  │   YES → RecomputeAffectedRoutes()
  │          ├─ 找 load cost 超閾值的 ISL edge
  │          ├─ 透過 m_islSources[edgeIdx] 擴散受影響 source
  │          └─ 只重算受影響 source，非全量 66 顆
  └─ ClearNextHopEntries() + AddNextHopEntry() 更新 66 個 arbiter
```

**GW / UT 可見性與路由（離線預計算）：**
```
PrecomputeGwRoutes() / PrecomputeGwUtRoutes()
  ├─ ComputeElevationDeg()  仰角計算（球形地球 R=6371km，threshold=5°）
  └─ 暴力枚舉 (entry ∈ vis(src)) × (exit/serving ∈ vis(dst))
     → 選最小 GetRouteCost(entry, exit, k)
     → entry == exit 時 cost = 0（同顆衛星直接覆蓋）
```

### test-iridium.cc — 測試腳本結構

三個 mode 對應不同的呼叫路徑：

| mode | 呼叫鏈 |
|------|--------|
| `sat2sat` | `PrintRouteReport({{satSrc, satDst}})` |
| `gw2gw` | `AddGateway` × 2 → `AddGwPair` → `PrecomputeGwRoutes` → `PrintGwRouteReport` |
| `gw2ut` | `AddGateway` + `AddUserTerminal` → `AddGwUtPair` → `PrecomputeGwUtRoutes` → `PrintGwUtRouteReport` |

GW preset 表（hardcode 於 `kPresets`）：
- 0 = TW-Taipei（25.0, 121.5）
- 1 = JP-Tokyo（35.7, 139.7）
- 2 = US-SanFrancisco（37.8, -122.4）

### 

1. **`TracePath` 重建邏輯**：每一跳查「目的地 == dst 的 route entry 的 nextHop」，若有環路（Dijkstra 理論上不產生）則跑滿 `m_numSatellites` 次後停止，邏輯安全。

2. **`HasSignificantChange` 在無流量時的行為**：load cost 全為 0 時，`refAb/refBa < 1e-12` 條件不成立，函式仍回傳 `true`（不觸發 cooldown 分支）；三組測試 slot 1–10 全為 YES 正是此行為。

3. **`ISL_path` 欄位顯示單一數字**：entry == exit 時 `satPath = {entry}`，`pathToStr` 只輸出一個節點，代表 TW-Taipei 與 JP-Tokyo 由同一顆衛星直接服務，無跨 ISL 段，與輸出觀察一致。

---



