# 2026-04-14

## 今日目標

完成 test-iridium-e2e.cc 的三段 E2E 架構重構，並對六種 pathType（sat2sat / gw2sat / sat2gw / gw2ut_e2e / gw2gw_e2e / sat2ut）分別執行模擬驗證，確認各段段架構、觀測器（OBS）、路由路徑與流量交付均正確運作。

---

## 完成項目

- [x] test-iridium-e2e.cc：新增 E2E 三段架構（E2EConfig / E2EExecutionPlan / BuildE2EPlan）
- [x] test-iridium-e2e.cc：新增 E2E Link Observability 系統（SegLinkStats / ConnectLinkObserverTraces / TakeObsSnapshot / PrintObsFinalSummary）
- [x] test-iridium-e2e.cc：新增 ValidateE2EConfig / ApplyLegacySegmentDefaults / PrintE2ERunBanner
- [x] test-iridium-e2e.cc：新增六種 pathType 的 ConfigureRoutingCase / 各段流量安裝函式
- [x] isl-graph.cc：移除 PrecomputeAllTables 內的 ApplyTiebreaker 呼叫（dead code 清理）
- [x] isl-graph.cc：GetLinkQueueDelay 改用 GetNBytes() 取代 nPackets × 1500 估算
- [x] beam-hopping-manager.h：完成 BeamHoppingManager class 結構（Layer 2 API 定義）
- [x] 六種 pathType 模擬驗證完畢，結果存入 Outputs/E2E-PathType/

---

## 修改檔案

| 檔案 | 修改內容 |
|------|---------|
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\test-iridium-e2e.cc` | 從 baseline 重構為三段 E2E 架構；新增 E2ESegment enum、E2EConfig / E2EExecutionPlan struct、BuildE2EPlan / ValidateE2EConfig / ApplyLegacySegmentDefaults；新增 E2E Link Observability 系統（SegLinkStats、三類 callback、TakeObsSnapshot、PrintObsFinalSummary）；新增六段流量安裝函式；main() 新增 --enableFeederlink/Isl/Servicelink CLI 參數；SimulationHelper 名稱改為 "test-iridium-3segment-e2e" |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\isl-graph.cc` | PrecomputeAllTables 移除 ApplyTiebreaker 呼叫（no-op，加 dead code 說明注釋）；GetLinkQueueDelay 改為 `q->GetNBytes()` 取代 `nPackets * 1500` 估算，修正低負載下 queue delay 假性非零問題 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Beam Hopping Controller\Codes\beam-hopping-manager.h` | 新增 CellDef / BhEvent struct；新增 TrafficDemandProvider 介面與 UniformDemandProvider 實作；新增 BhSwitchCallback typedef；完成 BeamHoppingManager class 定義（AddCell / SetDemandProvider / SetBhSwitchCallback / ComputeBhSchedule / ScheduleBhUpdates / GetCurrentCell API）；Layer 3 hook 介面預留 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Layer1.md` | 更新 mermaid 流程圖節點，反映 test-iridium-e2e 的六種 pathType 與三段架構；補充核心資料結構表（SegLinkStats / ObsConfig / E2EConfig / E2EExecutionPlan） |

---

## 關鍵決策

**1. 以 pathType 取代 mode 作為主要分流維度**

原 test-iridium_baseline.cc 使用 `--mode`（sat2sat / gw2gw / gw2ut）作為路由模式標籤，無法直接表達「鏈路段」概念。e2e 版改以 `pathType`（sat2sat / gw2sat / sat2gw / sat2ut / gw2ut_e2e / gw2gw_e2e）為主，讓路由設定（ConfigureRoutingCase）與流量安裝（InstallE2ETraffic）都以 pathType 為依據，segment 開關（feederlink/isl/servicelink）疊加其上，兩個維度互不干擾。

**2. ApplyLegacySegmentDefaults 保持向下相容**

不傳 `--enableFeederlink/Isl/Servicelink` 時，`ApplyLegacySegmentDefaults()` 依 `legacyProfile` 自動推斷段開關，行為與 baseline 一致，不破壞既有測試腳本。

**3. GetLinkQueueDelay 改用 GetNBytes()**

原 `nPackets * 1500` 在 SNS3 控制平面小封包場景下嚴重高估，導致 `trafficProfile=none` 時 EMA load cost 出現假性非零值並觸發不必要的路由重算。改為直接讀取佇列持有 bytes 數後再換算，修正此問題。

**4. ISL obs callback 以 multicast 並行掛載**

`ConnectLinkObserverTraces()` 對同一 `PacketDropRateTrace` 掛第二個 `IslObsCb`，與既有 `IslPacketDropCallback` 並行運作（NS3 TracedCallback 為 multicast），不修改既有統計邏輯，僅新增 OBS 路徑。

---

## 輸出結果

以下為今日各 pathType 驗證結果：

### gw2gw_e2e（GW0 TW-Taipei → GW2 US-SanFrancisco，120s）

| 項目 | 結果 |
|------|------|
| ISL trace 連接 | 264 介面 / 132 unique links [OK] |
| GW2GW 資料平面交付 | 603,648 bytes (~1179 pkts) [PASS] |
| ISL drop rate | 0.000%（6,340,555 pkts）[PASS] |
| 動態路由重算 | slot1: 3/66 src, slot2: 17/66 src |
| Slot 2 路徑切換 | exit sat 由 SAT37 → SAT1（exit sat 重選）|

ISL 路由路徑：

| slot | entry | ISL path | exit | isl_cost(s) |
|------|-------|----------|------|-------------|
| 0 | SAT15 | 15→14→25→36→37 | SAT37 | 0.043969 |
| 1 | SAT15 | 15→14→25→36→37 | SAT37 | 0.046214 |
| 2 | SAT15 | 15→14→13→2→1 | SAT1 | 0.047600 |

### sat2sat（SAT0 → SAT33，120s）

| 項目 | 結果 |
|------|------|
| ISL trace 連接 | 264 介面 / 132 unique links [OK] |
| 路由路徑 | 0→1→2→57→46→35→34→33（7 hops，全 3 slots 穩定）|
| route_cost 趨勢 | 0.078176 → 0.074919 → 0.072055（持續下降）|
| 動態路由重算 | slot1: 13/66, slot2: 21/66 |

### gw2sat（GW0 TW-Taipei feederlink 上行，120s）

| 項目 | 結果 |
|------|------|
| ISL trace 連接 | 264 介面 / 132 unique links [OK] |
| 主覆蓋衛星 | SAT15（32,087 pkts, delay=4.62ms）|
| feeder drop rate | 0.00% [PASS] |

### gw2ut_e2e（GW0 TW-Taipei → UT0 Taipei，120s）

| 項目 | 結果 |
|------|------|
| ISL trace 連接 | 264 介面 / 132 unique links [OK] |
| 路由結果 | entry=SAT15, serving=SAT15（無 ISL 跳，GW 與 UT 距離 <10km）|
| feeder drop rate | 0.00% |
| 動態路由重算 | slot1: 3/66, slot2: 16/66 |

（sat2gw / sat2ut 輸出結果亦已記錄於 Outputs/E2E-PathType/ 對應 md 檔）

---

## 遺留問題 / 下一步

- gw2sat / sat2gw / sat2ut 三種 pathType 在 feeder/service link OBS 出現 false alarm（sat61 service throughput=0）
- gw2ut_e2e 的 entry sat == serving sat 情況（GW/UT 近距離），需在 `PrintGwUtRouteReport` 中明確標注no ISL hop而非留空 isl_cost 欄位
- Layer1.md 流程圖需補充 E2E Link Observability 路徑（ConnectLinkObserverTraces → TakeObsSnapshot → PrintObsFinalSummary）
