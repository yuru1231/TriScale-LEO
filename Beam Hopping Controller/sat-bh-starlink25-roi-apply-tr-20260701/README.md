# sat-bh-example Starlink25 ROI Apply-TR 封包

日期：2026-07-01

---

## 目的

本封包包含 Starlink25 ROI Beam Hopping 驗證執行的程式碼與輸出結果（不含 ISL）。

此情境將東京 ROI（感興趣區域）的 25 波束網格套用至指定的 Starlink 衛星上，對這些衛星安裝 BH Helper，並將真實需求（DAMA RBDC）、時間計畫（PLAN）與 OBC ToggleState 事件寫入 BH 流量 trace 檔案。

**驗證目標：** 確認每一筆 OBC ACTIVATE/DEACTIVATE 事件均能正確對應至真實 SNS3 SatNetDevice，並成功呼叫 `ToggleState(true/false)`。

---

## 情境參數

| 參數 | 數值 |
|------|------|
| 星座 | constellation-starlink-1584-sats |
| ROI 中心 | 東京 (35.676°N, 139.650°E) |
| 波束網格 | 25 波束（5×5 UPA 橢圓格） |
| 輔助衛星數量 | 6 顆 |
| 最大同時啟動波束 K | 2 |
| 模擬時長 | 120 秒（暖機 1 秒） |
| 啟用功能 | OBC + Dynamic BSTP + Phase F（真實 RBDC 需求） |
| ISL | 未啟用 |

---

## 選定的 ROI 輔助衛星

```
382, 404, 448, 883, 927, 948
```

這 6 顆衛星由 2D orbit-sgp4 掃描結果選定，為模擬時刻 t=4168s 時東京上空仰角最高的 Starlink 衛星群。

---

## 完整執行指令

```bash
mkdir -p results/sat-bh-starlink25-roi-apply-tr-full

./ns3 run "sat-bh-example \
  --scenario=starlink25 \
  --enableObc=1 \
  --enableDynamicBstp=1 \
  --enablePhaseF=1 \
  --maxActiveBeams=2 \
  --simTime=120 \
  --warmUp=1 \
  --helperSatList=382,404,448,883,927,948 \
  --metricsFile=results/sat-bh-starlink25-roi-apply-tr-full/bh-metrics.csv \
  --timePlanFile=results/sat-bh-starlink25-roi-apply-tr-full/bh-timeplan.csv \
  --trafficTraceFile=results/sat-bh-starlink25-roi-apply-tr-full/sat-bh-traffic.tr"
```

---

## 關鍵驗證結果

完整執行的 trace 統計如下：

```text
headers   1
PLAN      72000
DEMAND    719400
EVENT     287976
sats      = 927 404 948 382 448 883
mapped_toggled_events = 287976
```

**驗證通過條件：** `EVENT == mapped_toggled_events`

所有 287,976 筆 OBC ACTIVATE/DEACTIVATE 事件均成功對應至真實 SNS3 SatNetDevice，且 `ToggleState(true/false)` 已正確呼叫（trace 中每筆 EVENT 行的 `mapped=1`、`toggled=1`）。

---

## 結果檔案說明

### 主要結果（`173241` 為完整執行）

| 檔案 | 行數 | 說明 |
|------|------|------|
| `bh-metrics_starlink25_20260701_173241.csv` | 17,845 行 | 每 80ms（T_p）一筆 KPI 指標，含 6 顆衛星 × 25 波束 |
| `bh-timeplan_starlink25_20260701_173241.csv` | 9 行（含標頭）| BHTP 時槽表，8 個時槽/週期，每槽 10ms，K=2 |
| `sat-bh-traffic_starlink25_20260701_173241.tr` | 1,079,377 行 | 完整 BH 流量 trace，含 PLAN / DEMAND / EVENT 三種記錄 |

> 舊版 `173009` 為中途中斷的短版執行，保留供參考對照。

---

## Trace 欄位說明

```
record_type, time_s, sat_id, beam_id, event, plan_id, slot_idx,
demand_kbps, duration_ms, active_beams, mapped, toggled
```

| 記錄類型 | 說明 |
|----------|------|
| `DEMAND` | 從 SNS3 DAMA trace 取得的真實 Phase F RBDC 需求 |
| `PLAN`   | Dynamic BSTP 時槽計畫（由 SatDynamicBstpProvider 產生） |
| `EVENT`  | OBC 波束 ACTIVATE/DEACTIVATE 事件，以及 ToggleState 結果 |

**EVENT 欄位說明：**
- `mapped=1`：事件已成功對應至真實 SatNetDevice
- `toggled=1`：`ToggleState()` 已成功呼叫

---

## BHTP 時槽結構

由 `bh-timeplan_starlink25_20260701_173241.csv` 可見：

```
slotIdx  startMs  durationMs  beamIds  modcod  clusterIds
0        0        10          1;2      5       1;2
1        10       10          3;4      5       3;4
...
7        70       10          15;16    5       15;16
```

- 每個 BHTP 週期 80ms，共 8 個時槽
- 每槽 10ms，最多 2 個波束同時啟動（K=2）
- Modcod = 5（對應星座預設波形）
- 25 個波束依序分配，輪轉排程

---

## 流量設定

| 方向 | 類型 | 封包間隔 | 封包大小 | 速率（估算） |
|------|------|----------|----------|--------------|
| 前向（FWD） | CBR UDP | 600 μs | 1500 bytes | 20 Mbps/UT |
| 回向（RTN）基礎 | CBR UDP | 100 ms | 512 bytes | ~41 kbps/UT |
| 回向（RTN）Phase F | CBR UDP | 80 ms | 1000 bytes | 100 kbps/UT |

- 每顆輔助衛星各建立 25 個 UT，共 150 個 UT（6 顆衛星 × 25 波束）
- Phase F 啟用後，RBDC 模式開啟（DaService3 CRA=off，RBDC=on）
- RTN Phase F 流量觸發 `BacklogRequestsTrace`，使 DynamicBstp 讀到真實需求

---

## 程式碼檔案

| 檔案 | 說明 |
|------|------|
| `code/scratch/bh_dynamic/Codes/sat-bh-example.cc` | 主模擬程式（Phase 1 → Phase G 統一入口） |
| `code/contrib/satellite/helper/sat-bh-helper.cc` | SatBhHelper 安裝器（Phase 1–G 所有模組連接） |
| `code/contrib/satellite/helper/sat-bh-helper.h` | SatBhHelper 標頭（BhExperimentConfig 定義） |

---

## 執行階段說明

`sat-bh-example.cc` 支援 Phase 1 → Phase G 的漸進式啟用：

| 階段 | 啟用旗標 | 功能 |
|------|----------|------|
| Phase 1（預設） | — | SatBhTimePlan + SatBhMetrics（靜態 BHTP + 合成驅動） |
| Phase 2 | `--enableScheduler=1 --enableObc=1` | EM 排程演算法 + OBC 槽切換狀態機 |
| Phase 3 | `+ --enableCacheQueue=1 --enablePrecoder=1` | CacheQueue 封包緩衝 + MMSE 預編碼 |
| Phase C | `--enableResourceManager=1 --enableUserAssociation=1` | 自主排程迴圈 |
| Phase E | `+ --enablePhaseE=1` | 連接真實 SNS3 API（MoveUtBetweenBeams） |
| Phase F | `+ --enablePhaseF=1` | 真實 DAMA 需求（BacklogRequestsTrace → RBDC） |
| **Phase G（本次）** | `--enableObc=1 --enableDynamicBstp=1 --enablePhaseF=1` | 動態貪婪 BSTP + 真實 RBDC 需求 |

---

## 設計原則

- 本程式僅呼叫 `SatBhHelper`，不直接操作 Scheduler / OBC / CacheQueue / Precoder
- 所有 SNS3 連接透過 `Config::ConnectWithoutContext()` / trace callback 完成，**不修改 SNS3 原始碼**
- `EnableFwdLinkBeamHopping=false` 防止 SNS3 內建 SatBstpController 與 SatBhObc 衝突

---

## 預期行為

1. SNS3 建立 1584 顆 Starlink 衛星，但只對 6 顆 ROI 衛星安裝 BH Helper
2. 每顆 ROI 衛星各建立 25 個 UT，置於東京 ROI 波束中心
3. Phase F 啟用後，UT 發送 RBDC 容量請求，`BacklogRequestsTrace` 觸發並更新需求快取
4. SatDynamicBstpProvider 依需求分數（貪婪 top-K）選出每週期啟動的 2 個波束
5. SatBhObc 於每槽邊界呼叫對應 GW SatNetDevice 的 `ToggleState()`
6. 所有 ACTIVATE/DEACTIVATE 事件均記錄至 `.tr` 檔，`mapped` 與 `toggled` 欄位應全為 1
