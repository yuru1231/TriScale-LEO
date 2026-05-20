# SNS3 預先排程路由架構

## 1. 系統概述

### 1.1 核心主張

原生 SNS3 在每個更新週期執行完整的 O(N²) 拓樸重建加全量 Dijkstra。將這部分工作移至模擬開始前的離線預計算階段，執行期僅在 load cost 變化超過 `ChangeThreshold` 時，針對受影響節點進行局部 Dijkstra 重算，其餘時間直接套用離線路由表。

### 1.2 與原生 SNS3 的對比

| 項目 | 原生 SNS3 | 本架構 |
|------|----------|--------|
| 拓樸重建 | 每 60s 重建，O(N²) | 離線一次完成，執行期不重建 |
| Dijkstra | 每次全量重算 | 當某條 ISL 的 load cost 變化比例超過 `ChangeThreshold`，重算 |
| load 感知 | 不支援 | 執行期讀 `PointToPointIslNetDevice::GetQueue()->GetNPackets()` 微調 |
| 時鐘依賴 | `GetPosition()` 依賴 `Simulator::Now()` | SGP4 直接代入 τ_k，與時鐘無關 |

### 1.3 星座參數

| 參數 | 數值 |
|------|------|
| Scenario | `constellation-iridium-66-sats-fixed` |
| 衛星數量 | 66 顆（6 軌道面 × 11 顆） |
| 軌道高度 | 780 km，傾角 86.4° |
| ISL 數量 | 132 條（每顆衛星 4 條） |
| 模擬時長 | 120–300 秒（依 pathType） |
| 路由更新粒度 τ | 60 秒 |

### 1.4 縮寫對照表

| 縮寫 | 全名 |
|------|------|
| NCC | Network Control Center |
| OBC | On-Board Computer |
| FT | Fixed Terminal |
| EMA | Exponential Moving Average |
| EM | Estimated Metric（BH Scheduler 用於虛擬流量計算） |
| WFQ | Weighted Fair Queuing |
| CRA | Constant Rate Assignment |
| RBDC | Rate-Based Dynamic Capacity |
| VBDC | Volume-Based Dynamic Capacity |
| MMSE | Minimum Mean Square Error |
| JFI | Jain's Fairness Index |

---

## 2. 三層架構

### 2.1 系統級三層

| Layer | 職責 | 時間維度 | 執行時機 |
|------|------|---------|---------|
| **Layer 1：ISL Routing** | FT pair 篩選 + 衛星間最短路由 | 分鐘級（60s slot） | 離線預計算；執行期每 60s 套用 |
| **Layer 2：Beam Hopping** | BHTP-based 多 beam 動態調度（K=3 同時活動），EM 需求估算 + 虛擬流量排程。圖示見 [`sat-bh-example-flow.drawio`](Beam%20Hopping%20Controller/sat-bh-example-flow.drawio)（Page 2 示意圖、Page 3 時序圖） | DVB-S2X super-frame 級（T_s=26.5ms，T_p=503ms） | NCC 端排程；OBC 端每 slot 切換 |
| **Layer 3：QoS Scheduling** | 在給定 beam 服務時間內對 UE 做 priority + WFQ 排程 | 封包級 | SNS3 `SatBeamScheduler` 原生處理 |

### 2.2 層間接口

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

#### 2.3 Layer 進度狀態

| Layer | 狀態 | 關閉日期 | 文件 |
|-------|------|---------|------|
| Layer 1：ISL Routing | ✅ 完成 | 2026-04-23 | [`TechRef.md §2`](TechRef.md) |
| Layer 2：Beam Hopping | ⏳ Phase C/D/E 完成，Phase F 整合測試中 | — | [`Layer2.md`](Layer2.md)、[`Beam Hopping Controller/`](Beam%20Hopping%20Controller/) |
| Layer 3：QoS Scheduling | ⏳ 架構完成，attribute 路徑待驗證 | — | — |

---

## 3. 文件地圖

### 3.1 核心文件（兩份主文件）

| 文件 | 定位 | 目標讀者 | 使用時機 |
|------|------|---------|---------|
| [`TechRef.md`](TechRef.md) | **超詳細技術參考文件** | 共同開發者、接手工程師 | 查詢任何技術細節：API、資料結構、程式流程、驗證結果、開發指南 |
| [`Report/chapter3_sns3.md`](Report/chapter3_sns3.md) | **論文交付稿**（7 章骨架） | 指導教授、學術審查者 | 論文初稿提交、章節撰寫、研究貢獻說明 |

### 3.2 補充文件

| 文件 | 角色 | 使用時機 |
|------|------|---------|
| **本文件（Readme.md）** | 專案導航入口 | 初次瞭解專案架構，找到對應文件 |
| [`Topology & ISL Routing/Layer1.md`](Topology%20%26%20ISL%20Routing/Layer1.md) | Layer 1 技術參考（TechRef §2 的分層版） | 單獨查閱 Layer 1 細節 |
| [`Topology & ISL Routing/test-iridium-e2e_code_audit.md`](Topology%20%26%20ISL%20Routing/test-iridium-e2e_code_audit.md) | `test-iridium-e2e.cc` 程式碼閱讀輔助 | 查閱架構圖、流程圖、function mapping（TechRef §3–5 的來源） |
| [`Layer2.md`](Layer2.md) | Layer 2 Beam Hopping 技術規格 | 雙尺度框架、BHTP 時間模型、模組 API、Phase 實作順序 |
| [`Beam Hopping Controller/sat-bh-example-flow.drawio`](Beam%20Hopping%20Controller/sat-bh-example-flow.drawio) | Layer 2 三張圖：程式流程圖 / Beam Hop 示意圖 / Time-scale 排程圖 | 視覺化理解 BH 系統架構與時序 |
| [`Decisions/`](Decisions/) | 設計決策記錄 | 查閱具體設計選擇的背景與取捨 |
| [`Logs/`](Logs/) | 工作日誌 | 查閱各階段修改紀錄與驗證過程 |

---

## 4. 設計決策索引

| 決策 | 原因摘要 | 文件 |
|------|---------|------|
| Arbiter 機制取代 IP 層路由 | SNS3 ISL 封包轉發繞過 IP 層，`Ipv4StaticRouting` 無效 | `Decisions/01_Arbiter mechanism replaces IP layer routing.md` |
| ISL 距離門檻設為 5000 km | 2500 km 導致拓樸圖斷裂；Iridium 跨面瞬時距離可達 ~4800 km | `Decisions/02_ISL Distance Threshold.md` |
| Arbiter 預先建立（非排程內建立） | 排程內呼叫 `CreateObject<SatIslArbiterUnicast>()` 缺少衛星節點指標，觸發 fatal crash | `Decisions/03_Arbiter lifecycle management.md` |
| Beam Scheduler 開銷根本原因 | SNS3 DVB MAC scheduler 無流量時仍持續排程，`Simulator::Run` 佔 99.9% wall time | `Decisions/04_Beam Scheduler.md` |
