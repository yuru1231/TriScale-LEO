# Layer 2：Beam Hopping + User Scheduling + Power Allocation

## 1. 文件目的

- Beam Hopping 時序管理
- User Association / Scheduling
- 功率分配Power Allocation
- 雙尺度（Frame/slot）資源管理框架
- 與 Layer 1 ISL Routing 的預留介面

---

## 2. 系統圖

### 2.1 Time Axis（時間結構）



**關鍵時間參數：**

| 符號 | 定義 | 值 |
|------|------|----|
| T_slot | 單一時隙持續時間 | 26.5 ms |
| T_frame | 幀持續時間 = 19 × T_slot | 503 ms |
| T_sw | Beam switching time | 2 ms |
| T_prop | GW → SAT 指令傳播延遲 | 10 ms |
| M | 每幀時隙總數 | 19 |
| K | 同一時隙最多同時活躍 beam 數 | 3 |

---

### 2.2 Coverage Position（波束覆蓋）

**波束 Pattern 選擇指標 νsn（Beam Gain Ratio）：**

νsn 衡量一個候選波束 pattern 對本 beam UT 的天線增益，相對於對鄰近 beam 所產生的干擾增益之比值。

```
νsn = Σ G(θ_k)  for k ∈ own UTs
      ─────────────────────────────
      Σ G(θ_j)  for j ∈ neighbor beam centers
```

- **分子**：此 pattern 在本 beam 所有 UT 方向上的增益總和（越高代表服務品質越好）
- **分母**：此 pattern 在鄰近 beam 中心方向上的增益總和（越低代表干擾越小）
```c++
SatAntennaGainPattern::GetAntennaGain_lin(lat, lon)
```
**選擇原則：** 從 5 種候選波束寬度（1.0–3.0°）中，選 νsn 最大的 pattern。

| 情境 | 選到的 pattern |
|------|--------------|
| UT 集中、高需求區 | 窄波束（高增益、小干擾半徑） |
| UT 分散、邊緣覆蓋 | 寬波束（擴大覆蓋範圍） |

> 此分配在模擬開始前執行一次（setup-time），模擬期間不動態切換。

---

### 2.3 System Architecture（系統架構）

![structure](./Beam%20Hopping%20Controller/sat-bh-example-strc.png)
---

## 3. 雙尺度框架

### 3.1 Frame Scale

**週期**：503 ms 
**決策者**：`SatResourceManager`（NCC / GW 端）

| 決策 | 模組 | 輸入 | 輸出 |
|------|------|------|------|
| 波束模式 | SatBeamPatternSelector | 流量分佈、干擾矩陣 | patternId per beam |
| 使用者關聯 | SatUserAssociator | UT 位置、C/N0、需求 | utId → beamId 映射 |

決策頻率低，每Frame僅計算一次，輸出 `BeamConfig`（波束配置）供slot使用。

---

### 3.2 Timeslot Scale

**週期**：每 26.5 ms 執行一次  
**決策者**：`SatTimeslotController`（GW 端）

| 決策 | 模組 | 輸入 | 輸出 |
|------|------|------|------|
| 功率分配 | SatPowerAllocator | 信道矩陣、需求 | powerDbw per beam |

> **排程決策已合併至 Frame Scale**：UT 排程（WFQ / Priority / RR）由 `SatUserAssociator` 的 pre-planned schedule 負責，在每個 frame 決定哪個 UT 分配到哪個 beam，slot scale 不再需要獨立的 scheduler 模組。

---

### 3.3 資料流總覽

```
UT 回報 (channel state, demand, position)
    ↓
[Frame @ 503ms 起始]
SatResourceManager
    ├─ SatBeamPatternSelector → patternId[0..4] per beam
    └─ SatUserAssociator     → utId→beamId assignment（含 WFQ/Priority/RR 排程邏輯）
           ↓ BeamConfig（含 UT 分配與優先序）
[Timeslot @ 每個 26.5ms 起始]
SatTimeslotController
    └─ SatPowerAllocator     → powerDbw per beam
           ↓ SlotConfig
[執行層（既有模組）]
    ├─ SatBhObc              → beam switching 狀態機
    ├─ SatGwCacheQueue       → 封包緩衝 / dequeue
    ├─ SatBhPrecoder         → MMSE（cluster ≥ 2 beam）
    └─ SatBhMetrics          → 被動 KPI 收集
```

---

## 4. 系統參數

### 4.1 硬體與通道參數

| 參數 | 值 |來源|
|------|----|------|
| 衛星軌道高度 | 550 km |LAYER1|
| 載波頻率 | 20 GHz | 
| 系統頻寬 | 400 MHz | 
| 總功率預算 | 43 dBm | 
| 接收天線增益 | 35 dBi | 
| 雜訊功率 | -126.47 dBW | 
| 最低仰角 |  | 

### 4.2 時間結構參數

| 參數 | 符號 | 值 |
|------|------|----|
| Timeslot | T_slot | 26.5 ms |
| Frame of slot | M | 19 |
| Frame | T_frame | 503 ms |
| Beam switching time | T_sw | 2 ms |
| 指令傳播延遲 | T_prop | 10 ms |
| 最多同時活躍 beam 數 | K |  |

### 4.3 波束模式參數（5 種候選）

| Pattern Index | 3dB Beamwidth | 地面覆蓋半徑 | 增益（dBi） | 適用場景 |
|:---:|:---:|:---:|:---:|------|
| 0 | 1.0° | ~10 km | 43.89 | 極高需求 / hot-spot 核心 |
| 1 | 1.5° | ~15 km | 41.39 | 高需求 |
| 2 | 2.0° | ~20 km | 37.89 | 中等需求（預設） |
| 3 | 2.5° | ~25 km | 35.01 | 低需求 |
| 4 | 3.0° | ~30 km | 31.89 | 邊緣 / cold-spot |

### 4.4 QoS 與排程參數

| 參數 | 值 |
|------|----|
| 最低速率需求 R_min | 100 kbps |
| 最大容許延遲 T_max | 20 × T_slot = 530 ms |
| 功率最佳化最大迭代次數 | 30 |
| 波束選擇最大迭代次數 | 10 |

---

## 5. 模組規格

### 5.1 SatBeamPatternSelector

根據各 beam 的流量分佈，從 5 種候選 pattern 中選出最佳圖形。

**選擇演算法**：
1. 計算每個 candidate pattern 的 νsn 比率
2. `νsn = Σ G(θ_k) for k in own_UTs / Σ G(θ_j) for j in neighbor_beams`
3. 選擇 νsn 最大的 pattern（最大自身增益、最小鄰近干擾）
4. 最多迭代 10 次

**主要 Attributes（TypeId）：**
- `CandidateBeamwidths` : DoubleVector, 預設 {1.0, 1.5, 2.0, 2.5, 3.0}
- `MaxSelectionIterations` : uint32_t, 預設 10

**主要 Methods：**
- `SelectPattern(beamId, trafficDemand, channelInfo) → uint32_t patternIndex`
- `ComputeNuRatio(beamId, patternIndex, utPositions) → double`
- `GetPatternGain(patternIndex, angleRad) → double gainDb`

**執行時機：Setup-time 靜態分配（一次性）**

`SatBeamPatternSelector` 在模擬開始前執行一次，為每個 beam 選好 patternId（0~4），
於 `AttachChannels()` 時傳入對應的 `SatAntennaGainPattern` 物件。模擬期間不切換。

**無法做 runtime 動態切換的原因（trace 確認）：**
- `SatPhyTx::SetAntennaGainPattern()` 有 `NS_ASSERT(m_antennaGainPattern == nullptr)`，只允許初始化一次
- `SatAntennaGainPattern::GetAntennaGain_lin()` 不是 virtual，Proxy Pattern 無法攔截

**動態控制改由其他模組負責：**
- `SatUserAssociator`：每 503 ms 決定哪個 UT 關聯到哪個 beam
- `SatPowerAllocator`：每 26.5 ms 決定每個 beam 的 TX 功率
- `SatBhObc`：每 26.5 ms 決定哪個 beam 在哪個 slot 活躍

**研究貢獻定位調整：**
heterogeneous beam pattern 初始分配（窄/寬 beam 依需求分佈靜態設定）對 BH 系統效能的影響，而非 runtime beamwidth adaptation。

**SNS3 hook：**
```cpp
// setup 時（SatBhHelper::Install() 內）
Ptr<SatAntennaGainPattern> pattern = m_patternContainer->GetAntennaGainPattern(beamId, patternId);
orbiterHelper->AttachChannels(..., pattern, ...);
```

---

### 5.2 SatUserAssociator

**職責**：Frame-scale 決定哪些 UT 關聯到哪個 beam，同時負責 UT 間的排程邏輯（WFQ / Priority / RR）。`SatUserScheduler` 已合併至此模組。

**設計原則（pre-planned schedule）：**
- 在 frame N 計算 frame N+1 的 UT-beam 分配表（基於需求預測）
- 排程策略決定哪個 UT 優先進入哪個 beam，等效於 WFQ / Priority / RR
- 在 frame N 結尾呼叫 `MoveUtBetweenBeams()`，TIM-U 延遲在 frame N+1 生效
- 延遲是設計的一部分，不是 race condition

**排程模式：**

| 模式 | 邏輯 | 效果 |
|------|------|------|
| Round-Robin | UT 輪流被分配到 beam | 公平，不考慮需求 |
| Priority | 高優先 UT 優先佔用 beam slot | 保護關鍵服務 |
| WFQ | 依 weight × 需求量決定 frame 分配份額 | 加權公平，主要模式 |

**主要 Attributes（TypeId）：**
- `SchedulingMode` : enum {WFQ=0, PRIORITY=1, ROUND_ROBIN=2}, 預設 WFQ
- `MaxReassignmentPerFrame` : uint32_t, 預設 5（限制每幀最多切換幾個 UT）
- `MinRateKbps` : double, 預設 100（最低速率保護門檻）
- `MaxDelayMs` : double, 預設 530（超過此延遲強制優先服務）

**主要 Methods：**
- `Associate(utList, beamList, channelMap, demandMap) → AssignmentMap`
- `ApplyAssociation(map)` — 觸發 UT beam reassignment（見下方 SNS3 hook 說明）
- `ComputeWfqWeight(utId, bufferBytes, demandKbps) → double`
- `EnforceDeadlineProtection(map)` — 超過 T_max 的 UT 強制升為最高優先

**SNS3 hook 說明（trace 確認）：**

採用**預先排程（pre-planned schedule）**設計，與 BHTP 概念一致：

```
Frame N：SatUserAssociator 計算 UT 需求預測 → 產生 frame N+1 UT-beam 分配表
         → 立即呼叫 MoveUtBetweenBeams() 觸發 TIM-U
Frame N+1：TIM-U 延遲到期，UT 完成 beam 切換 → 分配表生效
```

呼叫方式：
```cpp
// 在 frame N 結尾（RunFrameOptimization() 內）
ncc->MoveUtBetweenBeams(utId, srcSatId, srcBeamId, dstSatId, dstBeamId);
```

TIM-U 的 1-frame 延遲是**設計的一部分**，不是 race condition。
`SatUserAssociator` 產生的是 frame N+1 的預測分配，故在 frame N 觸發剛好對齊。

不使用 `TransferUtToBeam()` 直接呼叫（只搬 scheduler 狀態，狀態不一致）。

---

### 5.3 SatResourceManager

**職責**：Frame-scale主控器，每 503 ms 執行一次，協調 UserAssociator + PowerAllocator。

**主要 Attributes（TypeId）：**
- `FrameDuration` : Time, 預設 503ms
- `EnablePatternSelection` : bool, 預設 true
- `EnableUserAssociation` : bool, 預設 true

**觸發機制：Self-scheduling loop**
`SatResourceManager` 在 `DoInitialize()` 排第一次 `RunFrameOptimization()`；
每次執行結束時用 `Simulator::Schedule(T_frame, ...)` 排下一次，避免 timer drift。

**主要 Methods：**
- `RunFrameOptimization()` — 每 503 ms 自排程執行
- `GetBeamConfig(beamId) → BeamConfig`
- `SetDemandUpdateCallback(DemandUpdateCallback cb)`
- `SetL1Interface(Ptr<SatL1RoutingInterface> iface)`

---



### 5.4 SatPowerAllocator

**職責**：Slot-scale在總功率預算 43 dBm 下最佳化每個 active beam 的功率分配。

**最佳化目標**：最大化 sum-rate（或 min-SINR）subject to Σ p_k ≤ P_total

**最佳化變數**：`p_k = m_eirpWoGainW` per beam（linear W，即 TxMaxPowerDbw 扣掉 output/pointing/OBO/antenna losses 後的值）

SNS3 資料流：
```
TxMaxPowerDbw [dBW]
  → (扣 losses) → m_eirpWoGainW [W] = p_k（最佳化變數）
  → × G_tx（antenna gain）
  → / FSL（free-space loss）
  → × G_rx → m_rxPower_W [W]
  → / (noise + interference) → SINR
```

**SINR 公式（linear domain）：**
```
SINR_k = (p_k × G_tx_k × G_rx_k / FSL_k) / (σ² + Σ_{j≠k} p_j × G_tx_j × G_rx_k / FSL_jk)
```

**迭代演算法（最多 30 次）：**
1. 初始化：等功率分配 `p_k = P_total / K`（linear W）
2. 計算各 beam SINR（公式見上）
3. 根據 SINR 梯度更新 `p_k`（gradient ascent / water-filling）
4. 投影回可行集：`Σ p_k ≤ P_total, p_k ≥ 0`
5. 收斂條件：`||Δp||₂ < ε = 0.001`

**寫回 SNS3（ApplyPower）：**
```cpp
double txMaxDbw = SatUtils::WToDbW(p_k) + outputLossDb + pointingLossDb + oboLossDb + antLossDb;
phy->SetTxMaxPowerDbw(txMaxDbw);
phy->Initialize();  // 必須重算 m_eirpWoGainW
```

**主要 Attributes（TypeId）：**
- `TotalPowerBudgetDbm` : double, 預設 43
- `MaxIterations` : uint32_t, 預設 30
- `NoisePowerDbw` : double, 預設 -126.47
- `ConvergenceEpsilon` : double, 預設 0.001

**主要 Methods：**
- `Allocate(activeBeams, channelMap, demandMap) → PowerMap`
- `ApplyPower(PowerMap)` — 呼叫 `SatOrbiterUserPhy::SetTxMaxPowerDbw()`

**SNS3 hook（trace 確認）**：

```cpp
Ptr<SatOrbiterUserPhy> phy = orbiterNetDevice->GetUserPhy(beamId);
phy->SetTxMaxPowerDbw(powerDbw);  // 單位：dBW（不是 W）
phy->Initialize();                 // 必須重呼叫，否則 m_eirpWoGainW 不更新
```

**單位契約（已驗證）：**
- `SetTxMaxPowerDbw()` 接受 **dBW**（TX max power，含 output/pointing/OBO/antenna losses 前的值）
- 內部計算 `eirpWoGainDbw = TxMaxPowerDbw − losses`，再轉成 linear W
- `SatPowerAllocator` 的輸出必須是 TX max power 的 dBW，不是 EIRP

若 `SatPowerAllocator` 最佳化輸出是 EIRP dBW，需補回 losses：
```cpp
txMaxPowerDbw = eirpDbw - antGainDb + outputLossDb + pointingLossDb + oboLossDb + antLossDb;
```

---

### 5.5 SatBhTimePlan（擴充）

在原有欄位基礎上新增（已實作，Phase B 完成）：

**BhSlotEntry 擴充欄位：**

| 欄位 | 型別 | 填入者 | 說明 |
|------|------|--------|------|
| `beamPatterns` | `std::map<uint32_t, BeamRadiusType>` | SatBhScheduler / SatBhHelper | Per-beam antenna pattern；Key = beamId（1-indexed），取代原 slot-wide `beamRadius` |
| `scheduledUtIds` | `std::vector<uint32_t>` | SatResourceManager | 此 slot 排程的 UT ID 列表（SatUserAssociator 輸出） |
| `allocatedPowerDbw` | `std::map<uint32_t, double>` | SatPowerAllocator | Per-beam TX max power [dBW]（IWFA 輸出） |

**BhSlotEntry 新增 helpers：**
```cpp
void SetBeamPattern(uint32_t beamId, BeamRadiusType pattern);
BeamRadiusType GetBeamPattern(uint32_t beamId,
                              BeamRadiusType defaultVal = MIDDLE) const;
```

**SatBhTimePlan 新增欄位：**
- `frameId : uint32_t` — 幀計數器（由 SatResourceManager 設定，單調遞增）

> **設計注意**：`framePatternIndex` 未加入 SatBhTimePlan。同一 slot 內多個 beam 各自可用不同 pattern，故以 per-beam `beamPatterns` map 表達，不需 frame-wide 單一值。

**Print() 輸出格式（per-beam patterns）：**
```
Slot [0 ms .. 26 ms]  beams={1,4}  radius=1:SMALL,4:LARGE  modcod=5  clusters={1,4}
```

---

### 5.6 SatL1RoutingInterface（Layer 1 預留介面，stub）

**職責**：Layer 2 與 Layer 1 ISL Routing 的橋接介面，目前為 stub 實作。

**主要 Attributes（TypeId）：**
- `Enabled` : bool, 預設 false（stub 模式）

**主要 Methods：**
- `GetActivePathBeams(srcGwId, dstGwId) → std::set<uint32_t>` — 回傳路由路徑所用的 beamId 集合
- `GetLinkLoad(satId, beamId) → double loadMbps` — 回傳目前 ISL / feeder 負載
- `RegisterBeamStateCallback(BeamStateCallback cb)` — 當 beam 上線 / 下線時通知 Layer 1

**Layer 1 → Layer 2**：保護路由使用中的 beam 不被 BH scheduler 關閉  
**Layer 2 → Layer 1**：beam 狀態改變時觸發 routing cost 重算

---

### 5.7 SatBhKpiLogger（新增，Phase E）

**職責**：Frame-level KPI 統一觀測點，補足 `SatBhMetrics`（packet-level beam KPI）無法覆蓋的 5 個核心指標。由 `SatResourceManager` 持有，每 503 ms flush 一行 CSV。

**與 SatBhMetrics 的分工：**

| 模組 | 粒度 | 輸出來源 |
|------|------|----------|
| SatBhMetrics | Packet-level per-beam | OBC BeamActivate/Deactivate 回呼 |
| SatBhKpiLogger | Frame-level per-beam | ResourceManager + SNS3 trace |

**5 KPI 覆蓋：**

| 指標 | 欄位 | 來源 |
|------|------|------|
| Capacity-Demand Gap | `allocated_kbps`, `sns3_demand_kbps`, `gap_kbps` | `UsableCapacityTrace` + `UnmetCapacityTrace` |
| E2E Latency | `avg_delay_ms`, `max_delay_ms` | `SatNetDevice::RxDelay` |
| Packet Delivery Rate | `pdr_pct` | Tx/Rx 計數器 |
| Power Consumption | `tx_power_dbw` | `SatPowerAllocator::PowerMap` |
| User Association Count | `assigned_ut_count` | `AssignmentMap` |

**CSV Schema：**
```
frame_id, sim_time_s, sat_id, beam_id,
allocated_kbps, sns3_demand_kbps, bh_demand_kbps, unmet_kbps, gap_kbps,
tx_power_dbw, assigned_ut_count,
avg_delay_ms, max_delay_ms, pdr_pct, sample_count
```

**主要 API：**
```cpp
void SetOutputFile(const std::string& path);
void OnUsableCapacity(uint32_t satId, uint32_t beamId, uint32_t kbps);
void OnUnmetCapacity(uint32_t satId, uint32_t beamId, uint32_t kbps);
void OnPacketDelay(Time delay);
void OnPacketTx();
void OnFrameOptimized(uint32_t frameId,
                      const std::map<uint32_t, BeamConfig>& beamConfigs,
                      const std::vector<UtInfo>& utList,
                      const AssignmentMap& assignment);
```

**SNS3 接線（SatBhHelper::ConnectTraces()）：**
```cpp
// UsableCapacityTrace / UnmetCapacityTrace
Ptr<SatBeamScheduler> sched = ncc->GetBeamScheduler(satId, beamId);
sched->TraceConnectWithoutContext("UsableCapacityTrace",
    MakeBoundCallback(&SatBhKpiLogger::OnUsableCapacity, logger, satId, beamId));
// RxDelay（全局 delay / PDR）
Config::ConnectWithoutContext(
    "/NodeList/*/DeviceList/*/$ns3::SatNetDevice/RxDelay",
    MakeCallback(&SatBhKpiLogger::OnPacketDelay, logger));
```

**flush 觸發**：`RunFrameOptimization()` 結束時呼叫 `OnFrameOptimized()`，保證 frame_id 與所有欄位對齊，無 timer 漂移。

---

### 5.8 既有模組（保留，不改動）

| 模組 | 職責 | 狀態 |
|------|------|------|
| SatBhObc | 接收 BHTP、執行 beam switching 狀態機（IDLE/ACTIVE/SWITCHING/WAIT_PLAN） | ✅ 保留 |
| SatGwCacheQueue | beam inactive 時暫存封包，slot 開始時排空 | ✅ 保留 |
| SatBhPrecoder | cluster ≥ 2 beam 時執行 MMSE 預編碼 | ✅ 保留 |
| SatBhMetrics | 被動收集 packet-level KPI（throughput/delay/JFI/drop_rate/dwell_time） | ✅ 保留 |
| SatBhTimePlan | BHTP 資料模型（BhSlotEntry per-beam 擴充） | ✅ 已擴充（Phase B） |
| SatL1RoutingInterface | Layer 1 ISL Routing 預留介面 | ✅ stub 實作 |

---

## 6. SNS3 Hook 對照表

> 所有 API 均在 Phase E 實作時確認。✅ = Phase E 中已接線；🔒 = 不使用。

| 功能 | SNS3 API | 所在檔案 | 單位 / 注意事項 | Phase E |
|------|----------|----------|----------------|:-------:|
| 改變 beam 功率 | `SatPhy::SetTxMaxPowerDbw(double)` + `Initialize()` | satellite-phy.h:259 | **dBW**；改完必須呼叫 `Initialize()` 重算 `m_eirpWoGainW` | ✅ |
| 取得 orbiter beam PHY | `SatOrbiterNetDevice::GetUserPhy(beamId)` | satellite-orbiter-net-device.h:129 | 回傳 `Ptr<SatPhy>` | ✅ |
| 取得 orbiter node | `SatTopology::GetOrbiterNode(satId)` | satellite-topology.h:403 | — | ✅ |
| UT beam 完整重分配 | `SatNcc::MoveUtBetweenBeams(Address utAddr, srcSat, srcBeam, dstSat, dstBeam)` | satellite-ncc.h:249 | 傳入 **MAC Address**（非 uint32_t utId）；含 TIM-U + handover delay | ✅ |
| UT MAC Address 查詢 | `SatNetDevice::GetAddress()` | satellite-net-device.h | 用於建立 utId → MAC Address 映射表 | ✅ |
| UT 所在 beam 查詢 | `SatTopology::GetUtBeamId(Ptr<Node>)` | satellite-topology.h:635 | PollUtStates 每幀呼叫 | ✅ |
| UT 所屬衛星查詢 | `SatTopology::GetUtSatId(Ptr<Node>)` | satellite-topology.h:626 | 用於過濾 multi-sat 場景 | ✅ |
| 全部 UT 節點 | `SatTopology::GetUtNodes()` | satellite-topology.h:292 | 回傳 NodeContainer | ✅ |
| 取得 NCC | `SimHelper->GetSatHelper()->GetBeamHelper()->GetNcc()` | simulation-helper.h:486 | — | ✅ |
| Capacity KPI trace | `SatBeamScheduler::UsableCapacityTrace` + `UnmetCapacityTrace` | satellite-beam-scheduler.cc:269 | 用於 SatBhKpiLogger 接線 | ✅ |
| Packet delay trace | `SatNetDevice::RxDelay` | satellite-net-device.cc:104 | 用於 delay / PDR 統計 | ✅ |
| UT scheduler-only 搬移 | `SatBeamScheduler::TransferUtToBeam(utId, dstScheduler)` | satellite-beam-scheduler.cc:973 | 🔒 只搬 NCC 資料，UT MAC/PHY/routing 不更新；不使用 | 🔒 |

---

## 7. 實作 Phase

### Phase A — 文件重寫（✅ 完成）
重寫 Layer2.md 涵蓋雙尺度框架、所有模組規格。

---

### Phase B — 資料模型擴充（✅ 完成）
- `SatBhTimePlan` / `BhSlotEntry` 擴充：
  - 加入 `beamPatterns : std::map<uint32_t, BeamRadiusType>`（per-beam，取代 slot-wide `beamRadius`）
  - 加入 `scheduledUtIds`、`allocatedPowerDbw`、`frameId`
  - 新增 `SetBeamPattern()` / `GetBeamPattern()` helpers
- 新增 `SatL1RoutingInterface`（stub）

---

### Phase C — Frame-scale模組（✅ 完成）
- `SatUserAssociator`：WFQ / Priority / RR + `MoveUtBetweenBeams` 接線（傳入 MAC Address）
- `SatResourceManager`：Self-scheduling loop（503 ms）、整合 UserAssociator + PowerAllocator
- `SatBeamPatternSelector`：**已確認不可在執行期動態切換**（SNS3 `SetTxAntennaGainPattern()` 只允許初始化一次）→ 改為 setup-time 靜態分配，為**選配功能（feature flag）**，視論文需求延後實作

---

### Phase D — Slot-scale模組（✅ 完成）
- `SatPowerAllocator`：IWFA water-filling 迭代功率最佳化
  - 最佳化變數：`m_eirpWoGainW` per beam（linear W）
  - 寫回：`phy->SetTxMaxPowerDbw(dBW)` + `phy->Initialize()`
- ~~`SatUserScheduler`~~：已合併至 Phase C 的 `SatUserAssociator`

---

### Phase E — 真實 SNS3 API 接線（✅ 完成，2026-05-14）
- `SatBhHelper::ConnectTracesPhaseE()`：
  1. `BuildUtAddressMap()`：建立 container index → MAC Address 映射（`SatNetDevice::GetAddress()`）
  2. `CacheOrbiterDevice()`：快取 `SatOrbiterNetDevice`（供 ApplyPowerCallback 使用）
  3. `MoveUtCallback` → `SatNcc::MoveUtBetweenBeams(MAC Address, ...)`（真實 UT 重分配）
  4. `ApplyPowerCallback` → `SatOrbiterUserPhy::SetTxMaxPowerDbw()` + `Initialize()`
  5. `PollUtStates()`：每 T_frame 查詢 `SatTopology::GetUtBeamId()`，更新 RM 中的 UT 狀態
- 啟動條件：`enablePhaseE=1` AND `enableResourceManager=1` AND `SetSimulationHelper()` 已呼叫
- 驗證結果：見 §8

---

### Phase F — 真實需求輸入（待實作）
- `PollUtStates()`：目前使用合成需求 1000 kbps，Phase F 接 `SatGwMac` DAMA 請求佇列 trace
- `SatResourceManager` channelGains：目前為 1.0，Phase F 接 `SatPhy::CnoInfo` trace

---

## 8. Phase E 驗證結果（2026-05-14）

### 8.1 測試指令

```bash
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 \
           --satId=1 --simTime=60" 2>&1 | tee bh_phasee_v2.log
```

**注意**：`--satId=1`（2-satellite 場景所有 UT 均在 SAT 1；預設 satId=0 會過濾掉全部 UT）。

---

### 8.2 Phase E Wiring 確認

```
SatBhHelper::BuildUtAddressMap: scanning 5 UT nodes
SatBhHelper::BuildUtAddressMap: mapped 5 of 5 UTs
SatBhHelper::CacheOrbiterDevice: found SatOrbiterNetDevice on sat 1 (device index=0)
SatBhHelper::ConnectTracesPhaseE: MoveUtCallback wired to SatNcc
SatBhHelper::ConnectTracesPhaseE: ApplyPowerCallback wired to SatOrbiterUserPhy
SatBhHelper::ConnectTracesPhaseE: scheduling PollUtStates() every T_frame=503ms
```

---

### 8.3 ResourceManager 自排程迴圈

```
SatResourceManager::RunFrameOptimization frameId=1  t=0s      utCount=0   ← PollUtStates 尚未執行
SatResourceManager::RunFrameOptimization frameId=2  t=0.503s  utCount=5   ← 第一次 Poll 後 UT 注入
SatResourceManager::RunFrameOptimization frameId=3  t=1.006s  utCount=5
...
SatResourceManager::RunFrameOptimization frameId=120 t=59.857s utCount=5  ← 全程穩定
```

**frameId=1 的 utCount=0 說明**：`DoInitialize()` 在 t=0 立即觸發 RM，但 `PollUtStates()` 也排在 t=0，NS-3 事件佇列執行順序造成第一幀 UT 尚未注入。frameId=2 起穩定（設計行為，非 race condition）。

---

### 8.4 Per-beam Pattern 輸出確認

```
Slot [0 ms .. 26 ms]  beams={1,4}  radius=1:SMALL,4:LARGE  modcod=5  clusters={1,4}
```

---

### 8.5 輸出檔案

| 檔案 | 內容 |
|------|------|
| `bh-metrics.csv` | Packet-level beam KPI（SatBhMetrics，每 503 ms） |
| `bh-timeplan.csv` | BHTP slot table（per-beam pattern 格式） |
| `bh-attributes.xml` | ns-3 ConfigStore attribute snapshot |

---

## 9. 模擬情境

| 情境 | BH | Pattern Sel | User Scheduling | Power Alloc | 目的 |
|------|----|:-----------:|:---------------:|:-----------:|------|
| Baseline | ❌ | ❌ | ❌ | Equal | 參考基準 |
| BH-only | ✅ | ❌ | ❌ | Equal | 驗證現有 BH 核心 |
| BH + QoS | ✅ | ✅ | ✅ | Equal | 驗證排程 QoS 效益 |
| Full System | ✅ | ✅ | ✅ | ✅ | 完整系統效能評估 |

每個情境：模擬 300 秒，去除前 10 秒 warm-up，輸出 CSV 至 `Outputs/`。

---

## 10. KPI 驗證目標

| 指標 | 目標值 | 說明 |
|------|--------|------|
| Capacity-demand gap | Full < Baseline | 主要研究指標 |
| 平均延遲 | BH+QoS vs Baseline ↓ ≥ 30% | 排程效益 |
| Jain Fairness Index | ≥ 0.90 | 使用者公平性 |
| Drop rate | < 0.5% | 緩衝設計驗證 |
| Min-rate 滿足率 | ≥ 95% | QoS 保證 |

---

## 11. 關鍵檔案清單

### 新增檔案（已完成）

| 檔案 | 模組 | Phase | 狀態 |
|------|------|-------|------|
| `sat-user-associator.h/.cc` | SatUserAssociator（WFQ/Priority/RR + MoveUtBetweenBeams） | C | ✅ |
| `sat-bh-resource-manager.h/.cc` | SatResourceManager（503 ms self-scheduling loop） | C | ✅ |
| `sat-power-allocator.h/.cc` | SatPowerAllocator（IWFA water-filling） | D | ✅ |
| `sat-l1-routing-interface.h/.cc` | SatL1RoutingInterface（stub） | B | ✅ |
| `sat-bh-kpi-logger.h/.cc` | SatBhKpiLogger（frame-level 5 KPI CSV） | E | 設計完成，待實作 |
| `sat-beam-pattern-selector.h/.cc` | SatBeamPatternSelector（setup-time 靜態選配） | C | 選配，延後 |

### 已修改檔案

| 檔案 | 變更內容 | 狀態 |
|------|---------|------|
| `sat-bh-time-plan.h/.cc` | BhSlotEntry 擴充：per-beam `beamPatterns`、`scheduledUtIds`、`allocatedPowerDbw`；SatBhTimePlan 加 `frameId` | ✅ |
| `sat-bh-helper.h/.cc` | Phase C/D/E 模組安裝、feature flags、`ConnectTracesPhaseE()`、`PollUtStates()` | ✅ |
| `sat-bh-scheduler.cc` | `BuildPlan()` 改用 per-beam `SetBeamPattern()` | ✅ |
| `sat-bh-example.cc` | Phase C/D/E CLI args、`SetSimulationHelper()` | ✅ |

---

## 12. 參考文獻

1. [A Beam Hopping Scheme Based on Adaptive Beam Radius for LEO Satellites](https://pmc.ncbi.nlm.nih.gov/articles/PMC11511224/)
2. [The Next Generation of Beam Hopping Satellite Systems: Dynamic Beam Illumination with Multi-timescale Resource Management](https://ieeexplore.ieee.org/document/9923617)
