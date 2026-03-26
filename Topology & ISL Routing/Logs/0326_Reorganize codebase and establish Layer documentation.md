# 工作日誌 2026-03-26

## 目標
整理專案程式碼版本歸檔、補建三層架構說明文件（Layer1/2/3.md），並確認 Layer 2 BeamHoppingManager 的獨立驗證計畫。

---

## 完成事項

### 1. 程式碼版本歸檔與路徑整理

**現象**：v3、v4、v5 的 `isl-graph` 系列、`ft-filter`、`beam-hopping-manager` 散落於舊路徑或尚未放入正確 Layer 資料夾，專案結構不清晰。

**原因**：開發迭代過程中直接在根目錄或臨時路徑儲存，未同步整理到對應層目錄。

**修正**：將以下檔案移入或新增至正確路徑：
- `v3_isl-graph.cc / .h / v3_test-iridium.cc` — 效能優化版本（Fix 1–4）
- `v4_isl-graph.cc / .h / v4_test-iridium.cc` — 加入計時拆解、`BuildISLGraphWithLoad` 的過渡版本
- `v5_isl-graph.cc / .h / v5_test-iridium.cc` — OOP 重構版本（含 Accessor、診斷 API、`m_blockedEdges`）
- `ft-filter.cc / ft-filter.h` — FtVisibilityFilter 模組，歸入 Layer 1
- `beam-hopping-manager.cc / beam-hopping-manager.h` — 同時放入 Layer 1（供 FtFilter 共用靜態工具）與 Layer 2（獨立實作）

**驗證**：各版本檔案均已對應至 `Topology & ISL Routing/Codes/` 與 `Beam Hopping Controller/Codes/`，版本演進可追溯 ✅

---

### 2. 補建 Layer 1 架構說明文件（Layer1.md）

**現象**：Layer 1 實作已完整（v1–v5 共五個版本），但無單一文件可讓新成員快速了解整體架構與使用方式。

**原因**：開發期間優先寫程式，架構文件延後補建。

**修正**：建立 `Layer1.md`，內容涵蓋：
- 架構流程圖（`LoadISLDefs` → `InitOrbiterDevices` → `PrecomputeAllTables` → `ScheduleRoutingUpdates` → `ApplyRoutingTable`）
- 所有 NS3 Attribute 參數說明與預設值
- 動態路由邏輯（`UpdateLoadCosts` / `HasSignificantChange` / `RecomputeAffectedRoutes`）
- 效能優化 Fix 1–4 說明
- SNS3 原始碼修改清單（`satellite-sgp4-mobility-model`、`satellite-isl-arbiter-unicast`）
- CMakeLists.txt 修改項目
- FtVisibilityFilter 子模組說明
- 版本對應表（v1–v5）
- 已知問題（DEC-004 Beam Scheduler 開銷）

**驗證**：`Layer1.md` 包含完整初始化範例程式碼與驗證基準輸出，可直接對照執行 ✅

---

### 3. 補建 Layer 2 架構說明文件（Layer2.md）

**現象**：`BeamHoppingManager` 模組昨日建立完成，但尚無正式架構說明。

**原因**：模組剛完成，架構文件尚未撰寫。

**修正**：建立 `Layer2.md`，目前為初稿（檔案已建立，內容待填充）。預計涵蓋 `BeamHoppingManager` 的 Attribute 參數、`ComputeBhSchedule` 演算法說明、與 Layer 1 的整合界面，以及 `TODO SNS3_BH_INJECT` 待確認項目。

**驗證**：檔案已建立，後續填充內容後可作為 Layer 2 獨立驗證的參考基準 ✅

---

### 4. 補建 Layer 3 架構說明文件（Layer3.md）

**現象**：Layer 3（QoS-Aware Packet Scheduler）尚未開始實作，需建立文件佔位並規劃介面。

**原因**：三層文件結構需一致，即使尚未實作也應建立文件框架。

**修正**：建立 `Layer3.md` 檔案（初稿，內容待填充），預計記錄與 Layer 2 的 `GetCurrentCell()` 介面依賴及 QoS 封包排程設計方向。

**驗證**：檔案已建立，作為 Layer 3 開發的起點文件 ✅

---

### 5. E2E v1 整合腳本與輸出歸檔

**現象**：E2E 腳本 `v1_e2e-iridium.cc` 及其執行輸出昨日產生，需歸入 `E2E/` 目錄並建立輸出記錄。

**原因**：E2E 腳本跨越 Layer 1（ISL + FtFilter）與 Layer 2（BeamHoppingManager），歸類至獨立 `E2E/` 路徑。

**修正**：
- `E2E/Codes/v1_e2e-iridium.cc`：整合 `IslRoutingManager`、`FtVisibilityFilter`、`BeamHoppingManager` 三個模組，加入 UDP 流量觸發動態路由
- `E2E/Outputs/v1_output.md`：完整執行輸出，包含 FtFilter 可見性報告、BH 事件紀錄、每槽 `HasSignificantChange` 狀態與 `recomputedSrc` 數量

關鍵輸出摘要：
```
PrecomputeAllTables: complete | wall=5ms
ScheduleRoutingUpdates: 12 events scheduled
BeamHoppingManager: 64 events across 12 slots
slot 1–10: HasSignificantChange=YES（recomputedSrc: 3 → 51）
Total wall time: 2775.79s
```

**驗證**：三模組協同運行無衝突，E2E 輸出與各模組單獨驗證結果一致 ✅

→ 若有重要設計決策，見 `Topology & ISL Routing/Decisions/04_Beam Scheduler.md`

---

## 驗證結果總表

| 項目 | 狀態 | 說明 |
|------|------|------|
| v3/v4/v5 程式碼歸檔 | ✅ | 所有版本已放入 `Topology & ISL Routing/Codes/` |
| ft-filter 歸入 Layer 1 | ✅ | `ft-filter.cc / .h` 路徑正確 |
| beam-hopping-manager 雙路徑 | ✅ | Layer 1 共用工具 + Layer 2 主實作各一份 |
| Layer1.md 架構文件 | ✅ | 完整，包含初始化範例與版本對應表 |
| Layer2.md 架構文件 | ⏳ | 初稿建立，內容待填充 |
| Layer3.md 架構文件 | ⏳ | 初稿建立，內容待填充 |
| E2E v1 輸出歸檔 | ✅ | 完整輸出存於 `E2E/Outputs/v1_output.md` |

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v3_isl-graph.cc` | 歸檔：v3 效能優化版本（Fix 1–4） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v3_isl-graph.h` | 歸檔：v3 標頭檔 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v3_test-iridium.cc` | 歸檔：v3 測試腳本 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v4_isl-graph.cc` | 歸檔：v4 計時拆解版本（含 `BuildISLGraphWithLoad`） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v4_isl-graph.h` | 歸檔：v4 標頭檔 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v4_test-iridium.cc` | 歸檔：v4 測試腳本 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v5_isl-graph.cc` | 歸檔：v5 OOP 完整實作（含 Arbiter 預建、`ClearNextHopEntries`、`RecomputeAffectedRoutes`） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v5_isl-graph.h` | 歸檔：v5 標頭檔（含 `TracePath`、`BlockISL`、`GetRouteCost` 等 API） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v5_test-iridium.cc` | 歸檔：v5 測試腳本（OOP API 驗證） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\ft-filter.cc` | 新增：FtVisibilityFilter 實作，含 `PrecomputeVisibility`、`GetBestTransit` |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\ft-filter.h` | 新增：FtVisibilityFilter 標頭檔，定義 `FtDef`、`FtTransitRoute` 結構 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\beam-hopping-manager.cc` | 新增：Layer 1 側 BHM（提供 `ComputeElevationDeg` 靜態工具給 FtFilter 共用） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\beam-hopping-manager.h` | 新增：Layer 1 側 BHM 標頭檔 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Beam Hopping Controller\Codes\beam-hopping-manager.cc` | 新增：Layer 2 主實作（`ComputeBhSchedule`、`ScheduleBhUpdates`、`ApplyBhEvent`） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Beam Hopping Controller\Codes\beam-hopping-manager.h` | 新增：Layer 2 標頭檔（定義 `CellDef`、`BhEvent`、`TrafficDemandProvider`、`BeamHoppingManager`） |
| `C:\Users\wenj\Desktop\TriScale-LEO\E2E\Codes\v1_e2e-iridium.cc` | 新增：E2E 整合腳本（Layer 1 + Layer 2 + UDP 流量） |
| `C:\Users\wenj\Desktop\TriScale-LEO\E2E\Outputs\v1_output.md` | 新增：E2E 完整執行輸出記錄 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Layer1.md` | 新增：Layer 1 完整架構說明文件 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Layer2.md` | 新增：Layer 2 架構說明文件（初稿） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Layer3.md` | 新增：Layer 3 架構說明文件（初稿，待填充） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Decisions\04_Beam Scheduler.md` | 新增：DEC-004 決策記錄（Beam Scheduler 開銷根本原因確認） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Outputs\v3_output.md` | 新增：v3 基準輸出記錄 |

---

## 明日計畫

- 填充 `Layer2.md`：補充 `BeamHoppingManager` 完整說明，包含 Attribute 參數表、`ComputeBhSchedule` 演算法細節、`TrafficDemandProvider` 插拔機制、與 Layer 1 的整合界面
- 確認 `TODO SNS3_BH_INJECT`：調查 SNS3 實際 beam switch API（`SetActiveBeam` 或 `SatBeamScheduler::EnableBeam`），決定 `ApplyBhEvent` 是否需修改 SNS3 原始碼
- 規劃 Layer 2 獨立驗證腳本：建立 `BH_test.cc`，不依賴 E2E 腳本，單獨驗證 `BeamHoppingManager` 排程正確性
