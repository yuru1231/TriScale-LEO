# SNS3 Beam Hopping 正式架構與實作綱要

## 1. 文件目的

- 作為 SNS3 / ns-3 Beam Hopping 實作的主規格
- 作為後續拆分 `.h/.cc` 模組與 example 的依據
- 作為與既有簡化版 controller 區隔的正式版本定義
- 作為後續研究紀錄、每日工作日誌與實驗設計的統一參考

---
## 2.Rules
### 2.1時間控制必須與標準對齊
BH 時序以 DVB-S2X Type 1 Super-frame 為基準：

- `T_sf = 26.5 ms`
- `T_s = 26.5 ms`
- `T_p = 503 ms`
- `T_sw = 2 ms`
- `T_prop = 5~20 ms`，預設 10 ms 

### 2.2正式架構是多模組協作，不是單一 controller
正式 BH 不是單一 class 就能完成，而是：

- NCC 端做需求估算與排程
- OBC 端接收 BHTP 並執行 slot 切換
- GW 端做 queue cache 與 dequeue
- Metrics 被動收集 KPI
- Precoder 視 cluster 狀況執行 MMSE
- Helper 作為統一安裝入口

---

## 3. 系統總覽

## 3.1 高階資料流

```text
Traffic demand / DaRequest
    ↓
SatBhScheduler (NCC)
    ↓
Generate SatBhTimePlan (BHTP)
    ↓
BHTP propagation delay
    ↓
SatBhObc (satellite side)
    ↓
Beam switching / slot activation
    ↓
SatGwCacheQueue dequeue on slot start
    ↓
Optional MMSE precoding for clustered beams
    ↓
Packets transmitted
    ↓
SatBhMetrics passively collect KPIs
```

---

## 3.2 高階角色分工

| 模組 | 位置 | 核心職責 |
|---|---|---|
| SatBhTimePlan | 資料模型 | 儲存完整 BHTP 週期與 slot 配置 |
| SatBhScheduler | NCC 側 | EM 估算、虛擬流量計算、調度、cluster 分組 |
| SatBhObc | 衛星側 | 接收 BHTP、執行 beam switching 狀態機 |
| SatGwCacheQueue | Gateway 側 | beam inactive 時暫存封包，slot 開始時排空 |
| SatBhPrecoder | Gateway 側 | Cluster 條件成立時做 MMSE 預編碼 |
| SatBhMetrics | 被動監測 | 收集 throughput / delay / JFI / drop rate 等 KPI |
| SatBhHelper | 安裝入口 | 統一安裝所有模組並接好 trace/hook |


---

## 4. 物理與系統假設

## 4.1 星座與頻段

最終規格採用：

- Iridium Original: `6 × 11 = 66` 顆衛星
- 軌道高度 `550 km`
- 軌道傾角 `86.4°`
- 每顆衛星有前後左右 4 條 ISL（介面已預留）
- 頻段 Ka / Ku，下行中心頻率 `19.5 / 20 GHz`
- 全頻率重用因子 = `1`（Full Frequency Reuse）

---

## 4.2 波束與半徑模型

正式規格不是固定單一 beam size，而是支援三種動態波束半徑：

| BeamSize | 半徑 | 增益 | 場景 |
|---|---:|---:|---|
| SMALL | 20 km | 43.89 dBi | 熱點、高 SINR、抗干擾 |
| MIDDLE | 40 km | 37.89 dBi | 預設一般場景 |
| LARGE | 80 km | 31.89 dBi | 非熱點稀疏覆蓋 |

並且維持 SNS3 原生天線方向圖，只透過修改 3dB 波束寬度輸入完成，不修改原生天線模組。

---

## 4.3 RF Chain 與同時活動 beam 數

正式規格中：

- `K = 2 ~ 4`
- 建議初始值 `K = 3`
- K 是單一 slot 同時活動 beam 數上限
- K 的大小同時影響：
  - 頻譜效率
  - 硬體成本
  - MMSE 預編碼複雜度 `O(K^3)` 

**部分並行 multi-beam BH**。

---

## 5. 時間模型與 BHTP 結構

## 5.1 基本時間參數

| 符號 | 定義 | 建議值 |
|---|---|---|
| `T_sf` | DVB-S2X Super-frame | 26.5 ms |
| `T_s` | BH Time Slot | 26.5 ms |
| `T_p` | BHTP period | 503 ms |
| `T_sw` | beam switching time | 2 ms |
| `T_prop` | 指令下發延遲 | 10 ms（建議） |


---

## 5.2 BHTP / Hopping Window 定義

一個 BHTP 週期由 `M` 個 time slots 構成，每個 slot 會指定：

- 本 slot 同時活動的 beam 集合
- 各 beam 的 beam radius
- 各 beam 的 MODCOD
- 各 beam 的 cluster ID
- slot 的開始時間與持續時間 

M 的決定原則：

1. 最小值：`M ≥ N / K`
2. EM 輸出：依 `d_n` 決定實際所需 slot 數
3. 上限：`T_p = M × T_s ≤ 1000 ms` 為建議上界 

---

## 5.3 SatBhTimePlan 資料模型

`SatBhTimePlan` 是整個正式架構的資料核心。

### 欄位
- `planId`
- `periodStart`
- `periodEnd`
- `slots : std::vector<BhSlotEntry>`

### `BhSlotEntry`
- `beamIds`
- `startTime`
- `duration`
- `beamRadius`
- `modcod`
- `clusterIds` 


- scheduler 決定某 beam 在一個 BHTP 內出現幾個 slots
- OBC 按 slot 執行
- metrics 最後統計某 beam 在一個週期內的總服務時間 `dwell_time_n`

---

## 6. 核心演算法

## 6.1 EM 需求估算

EM 在 NCC 側執行，用來估算下一個 BHTP 中各 beam 所需 slot 數 `d_n`。

### 輸入
- 觀測窗口 `W = 5` 個 BHTP 週期（建議）
- 每個 beam / cell 在各 slot 的觀測流量 `x_n,t`

### 輸出
- 各 beam 的需求 slot 數 `d_n`
- 需滿足 `Σ d_n = M × K`

### 觸發條件
- 定期觸發：每個週期結束評估是否需要重跑
- 提前觸發：連續兩個 slot 需求變化超過 `20%` 時立即重算 

### 公式
- E-step:
  `Q(λ | λ_old) = Σ_n Σ_t [ x_n,t × log(λ_n) - λ_n × T_s ]`
- M-step:
  `λ_n_new = (1/W×M) × Σ_t x_n,t`
- 終止條件：
  - `||λ_new - λ_old||₂ < ε`
  - 或迭代次數達上限
- 預設：
  - `ε = 0.001`
  - 最大迭代次數 = `50`
### d_n 轉換
`d_n = max(1, round( λ_n / Σ_n λ_n × M × K ))`  
若總數不等於 `M × K`，需做全域校正。

---

## 6.2 虛擬流量

正式排程優先序不是只看 demand bytes，而是看 **虛擬流量**。

### 單封包
`A_{p,j} = L_{p,j} × α × (1 + 1 / T_{p,j})`

其中：
- `L_{p,j}` = packet 長度
- `α` = delay sensitivity factor
- `T_{p,j}` = 剩餘 TTL（以 slot 計）

### 小區總虛擬流量
`A_j = Σ_p A_{p,j}`

### α 建議值
- `α = 2`：初始值，平衡 delay / throughput
- `α = 3~4`：延遲改善明顯，吞吐量小幅下降
- `α = 5`：延遲更低，但吞吐量開始顯著下降
- `α > 5`：不建議一般場景使用 

---

## 6.3 波束調度策略

正式版本的 scheduler 包含兩段：

### 第一段：非熱點小區優先給大波束
- `d_n` 低於第 25 百分位的小區視為非熱點
- 以地理鄰近做 greedy clustering
- 若單一大波束可覆蓋整個 cluster，則合併服務
- 建議分配連續 slots，降低切換次數 
### 第二段：熱點做動態半徑調整
- 依虛擬流量 `A_n` 排序
- 熱點密度高 -> 優先小波束
- 中等密度 -> 中波束
- 若 RF chain 不足或干擾限制不滿足 -> 回退到大波束
- 每個 slot 最多填入 `K` 個 beams，需滿足干擾條件後才可提交至 BHTP

---

## 6.4 干擾管理與 Cluster

系統採用雙重干擾規避門檻：

### 第一層：空間隔離
- 中波束對應 `θ ≈ 3~5°`
- 可用地面等效距離判定
- 若 `D > 1.5 × (r_i + r_j)`，可視為近似噪聲受限，免做第二層判定
### 第二層：影響因子
`ω_{i,j} = P_interference(i→j) / P_signal(j)`  
簡化後可寫為：
`ω_{i,j} = G_i(θ_{i→j}) / G_j(0°)`

若 `ω_{i,j} ≥ κ`，則兩 beam 必須併入同一 cluster 做 MMSE。  
預設 `κ = 0.08`。
### Cluster 分組
- 初始化：每個 beam 先是單一 cluster
- 比較：計算不同 cluster 間 beam 對的 `ω_ij`
- 合併：若 `ω_ij ≥ κ` 則合併 cluster
- 直到 cluster 不再變化為止 

---

## 6.5 MMSE 預編碼

### 啟動條件
只有當 cluster 內至少有 2 個活動 beam 時才啟動 MMSE。  
若 cluster 僅 1 個 beam，則不做 MMSE。

### 計算位置
- GW 端
- 不在衛星載荷上算

### 公式
`W = H^H × (H × H^H + σ²_n × I)^(-1)`

### 注意事項
- CSI 以每個 slot 開始的信道快照近似
- 建議以 Cholesky 分解做矩陣反運算
- 若 `|C_k| > 3`，記錄 condition number 作為數值穩定性指標

---

## 7. 模組規格與實作責任

## 7.1 SatBhScheduler

### 職責
- 收需求
- 跑 EM
- 算虛擬流量
- 做 beam scheduling
- 做 cluster 分組
- 輸出 `SatBhTimePlan`

### 主要 attributes
- `NumBeams = 19`
- `MaxActiveBeams = 3`
- `BhtpPeriod = 503ms`
- `SlotDuration = 26.5ms`
- `EmMaxIterations = 50`
- `EmConvergenceEps = 0.001`
- `DemandChangeThreshold = 0.20`
- `AlphaDelaySensitivity = 2.0`
- `InterferenceKappa = 0.08`
- `NonHotspotPercentile = 0.25`
- `PropagationDelay = 10ms` 

### 主要 methods
- `OnDemandReceived(uint32_t beamId, uint32_t bytes)`
- `RunSchedulingCycle(Time now)`
- `ScheduleNextCycle()` 

---

## 7.2 SatBhObc

### 職責
- 接收新 BHTP
- 依 slot 切 beam
- 管理狀態機
- 發送 `BeamSwitch` callback

### 狀態
- `IDLE`
- `ACTIVE`
- `SWITCHING`
- `WAIT_PLAN`

### 狀態轉換摘要
- `IDLE -> ACTIVE`：收到新 BHTP
- `ACTIVE -> SWITCHING`：slot 結束
- `SWITCHING -> ACTIVE`：`T_sw` 結束
- `ACTIVE -> WAIT_PLAN`：BHTP 最後一個 slot 結束
- `WAIT_PLAN -> ACTIVE`：收到新 BHTP 

---

## 7.3 SatGwCacheQueue

### 職責
- beam 不服務某小區時暫存封包
- slot 開始時排空對應 beam queue
- 供 metrics 計算 delay / drop rate

### 關鍵規格
- `MaxQueueSizePerBeam = 40 MB`
- `DropPolicy = TAIL_DROP`
- `RecordDropRate = true`

### Queue 容量依據
`Q_max = R_peak × T_p`  
估算：
- `R_peak ≈ 500 Mbps`
- `T_p = 503 ms`
- 理論值約 `31.4 MB`
- 建議設 `≥ 40 MB` 以保留安全餘量 

---

## 7.4 SatBhMetrics

### 職責
被動收集 KPI，不主動查詢原生內部狀態。

### 指標
- throughput
- avg_delay
- max_delay
- jain_fairness_index
- dwell_time
- slot_utilization
- drop_rate
- cluster_size_dist 

### CSV 表頭
`time_s, sat_id, beam_id, throughput_mbps, avg_delay_ms, max_delay_ms, dwell_time_ms, slot_util_pct, drop_rate_pct, jain_fairness_index`
---

## 7.5 SatBhHelper

### 職責
- 統一安裝所有新增模組
- 完成 AggregateObject 附掛
- 完成 trace 連線
- 提供 feature flag 與實驗控制介面

### 主要 API
- `Install(Ptr<SatHelper> satHelper)`
- `EnableMMSEPrecoding(bool enable)`
- `SetAlpha(double alpha)`
- `GetMetrics() const` 
---

## 8. Hook 點與替代方案

正式規格要求先確認這些 trace 是否存在：

| Hook | 用途 | 必要性 | 若不存在 |
|---|---|---|---|
| `DaRequestReceived` | 收需求，觸發 EM | 必要 | 解析 `SatGwMac::Rx` 封包頭 |
| `SlotAllocated` | slot 邊界同步 | 必要 | 以 `Simulator::Schedule` 推算 |
| `GwMac::Tx` | throughput | 必要 | `PhyTxEnd` |
| `GwMac::Rx` | enqueue 觸發 | 必要 | `PhyRxEnd` |
| `ChannelEstimation` | CSI / MMSE | MMSE 開啟時必要 | `SatPhyRxCarrier::SNR` 近似 |
| `HandoverCompleted` | handover / queue migration | LEO 多衛星時必要 | 監聽節點位置更新 |


---

## 9. KPI 與驗證目標

正式規格定義的 KPI 目標：

- throughput 提升：`+1.43% ~ +3.44%`
- 平均延遲降低：`-35.5% ~ -62.25%`
- JFI：`≥ 0.90`
- dwell utilization：`≥ 85%`
- drop rate：`< 0.5%` 

### 量測條件
- 去除前 10 秒 warm-up
- 模擬時長建議 `≥ 300 秒`
- 要含足夠流量變化週期

---

## 10. 實作 Phase 與依賴

### Phase 1   資料模型 + 被動觀測
    - SatBhTimePlan   → BHTP 資料結構，靜態建立
    - SatBhMetrics    → 被動收 KPI，不影響任何控制流

### Phase 2   加入控制面：動態排程 + OBC 執行
    - SatBhScheduler  → NCC 端，EM 演算法 → 動態產生 BHTP
    - SatBhObc        → 衛星端，接收 BHTP → 狀態機執行 slot 切換

### Phase 3   加入資料面：緩衝封包 + 干擾抑制
    - SatGwCacheQueue → GW 端，beam 非活躍時暫存封包
    - SatBhPrecoder   → GW 端，cluster ≥ 2 beam 時做 MMSE 預編碼
---

## 11. 執行順序

### Step 0 - Trace feasibility check
先確認：
- `DaRequestReceived`
- `SlotAllocated`
- `GwMac Tx/Rx`
- `ChannelEstimation`
- `HandoverCompleted`

是否存在、路徑是否正確、callback 簽章是否匹配。  
若不存在，先記錄 fallback，不要直接開始寫模組。

### Step 1 - 純資料模型
完成 `SatBhTimePlan` / `BhSlotEntry`
- 序列化 / pretty print
- slot 邊界計算
- planId 驗證
- unit test

### Step 2 - 被動 metrics
完成 `SatBhMetrics`
- throughput
- delay
- JFI
- drop rate
- csv writer

### Step 3 - 最小排程器
完成 `SatBhScheduler` 的 EM only 版本
- 先不做完整 radius / interference / MMSE
- 先驗證 `OnDemandReceived -> d_n -> TimePlan`

### Step 4 - 最小 OBC
完成 `SatBhObc`
- 單衛星
- 固定 BHTP
- 驗證 slot -> switching -> next slot 流程

### Step 5 - Cache Queue
完成 `SatGwCacheQueue`
- beam inactive 暫存
- slot start dequeue
- delay / drop 輸出

### Step 6 - 完整 scheduler
補齊：
- virtual demand
- hotspot / non-hotspot split
- radius selection
- cluster grouping
- K-limited slot packing

### Step 7 - Precoder
完成 `SatBhPrecoder`
- 固定 H 驗證
- cluster size >= 2 時啟動

### Step 8 - Helper
完成 `SatBhHelper`
- 統一安裝
- flag 控制
- alpha / mmse 開關
- metrics 對外介面

### Step 9 - 全情境實驗
跑：
- baseline 1
- baseline 2
- fixed demand single sat
- Iridium Walker full-scale

---

## 12. 基礎情境與全規模情境

## 12.1 基礎驗證情境
- 1 顆衛星
- 7 個 beams
- K = 2
- Poisson demand
- 3 熱點、4 非熱點
- 模擬 300 秒
- α = 2
- κ = 0.08
- MMSE 關閉 

## 12.2 全規模 Iridium Walker
- 66 顆衛星
- 每衛星最多 19 beams
- K = 3
- 熱點密度 r = 0.3 / 0.45 / 0.6
- 模擬 600 秒
- α = 2, 3, 4, 5
- κ = 0.05, 0.08, 0.12
- MMSE 開關對照
- 原生 handover 啟用 

## 12.3 Baselines
- Baseline 1: 靜態 beam，無 BH，無 Cache Queue
- Baseline 2: 固定 pattern BH，不做 EM，各 beam 均分 slots 

---
## 13.Reference
[A Beam Hopping Scheme Based on Adaptive Beam Radius for LEO Satellites](https://pmc.ncbi.nlm.nih.gov/articles/PMC11511224/)\
[The Next Generation of Beam Hopping Satellite Systems: Dynamic Beam Illumination with](https://ieeexplore.ieee.org/document/9923617)
