# 2026-05-14 工作日誌

## 今日目標

1. 確認實作路線
2. 實作 Phase E：接上真實 SNS3 API
3. 執行BhSlotEntry 重構
4. Phase C + D + E 整合執行

---

## 1、Layer 2 implement Grill decision

### 同一 slot 內多個 beam 能否各自使用不同 antenna pattern？

**BeamRadiusType 擴充為 5 值：**

| Enum | patternIndex | Beamwidth | 半徑 |
|------|:---:|:---:|:---:|
| XSMALL | 0 | 1.0° | ~10 km |
| SMALL | 1 | 1.5° | ~15 km |
| MIDDLE | 2 | 2.0° | ~20 km |
| LARGE | 3 | 2.5° | ~25 km |
| XLARGE | 4 | 3.0° | ~30 km |

**BhSlotEntry 重構：**

- 移除 slot-wide `beamRadius`（單一值無法表達同一 slot 內多 beam 不同 pattern）
- 改為：
  - `beamPatterns : std::map<uint32_t, BeamRadiusType>` — per-beam pattern
  - `scheduledUtIds : std::vector<uint32_t>` — slot 排程 UT
  - `allocatedPowerDbw : std::map<uint32_t, double>` — per-beam 功率分配
- `SatBhTimePlan` 新增 `frameId : uint32_t`

**影響檔案**：`sat-bh-time-plan.h/.cc`、`sat-bh-scheduler.cc`、`sat-bh-helper.cc`（`BuildStaticBhtp()`）

---

### ResourceManager 如何觸發自排程？UT 搬移應呼叫哪個 SNS3 API？

** ResourceManager 觸發：Self-scheduling loop**

```
DoInitialize() → Simulator::Schedule(t=0, RunFrameOptimization)
RunFrameOptimization 結尾 → Simulator::Schedule(T_frame, RunFrameOptimization)
```

不依賴外部 timer，避免 drift 累積。

** UT 搬移 API 選擇：**

- `TransferUtToBeam()` ❌ — 只搬 NCC scheduler 狀態，UT MAC/PHY/routing 不更新
- `SatNcc::MoveUtBetweenBeams()` ✅ — 完整搬移，Pre-planned schedule 設計

```
Frame N end：Associate() + ApplyAssociation() → 呼叫 MoveUtBetweenBeams()
Frame N+1：TIM-U 延遲到期，UT 完成切換 → 生效
```

---

### SatPhy::SetTxMaxPowerDbw 的輸入單位是 dBm 還是 dBW？設定後需要額外步驟嗎？

- API：`SatPhy::SetTxMaxPowerDbw(double powerDb)` — 接受 **dBW**
- 設定後必須呼叫 `phy->Initialize()` 才會重算 `m_eirpWoGainW`
- `SatPhy::Initialize()` 是 SatPhy 自己的 setup method（非 ns-3 Object 生命週期），可在執行期安全多次呼叫

SNS3 資料流：
```
TxMaxPowerDbw → Initialize() → m_eirpWoGainW [W] → SendPduWithParams → channel
```

---

### 能否繼承 SatFrameAllocator 來 override UT 排程行為？

- `SatFrameAllocator::SortUts()` 為 private non-virtual，無法從外部 override
- **決策**：廢除獨立 `SatUserScheduler`，WFQ / Priority / RR 排程邏輯全部整合進 `SatUserAssociator`

---

### IWFA 的最佳化決策變數是什麼？結果如何寫回 SNS3 PHY？

- 最佳化變數 = `p_k = m_eirpWoGainW` per beam（linear W）
- SINR 公式：`SINR_k = (p_k × G_tx × G_rx / FSL) / (σ² + Σ_{j≠k} p_j × G_tx × G_rx / FSL_jk)`
- 寫回 SNS3：`phy->SetTxMaxPowerDbw(txMaxDbw); phy->Initialize();`

---

### Runtime 動態切換 beam antenna pattern 在不修改 SNS3 的前提下是否可行？

**SNS3 trace 結果：** Runtime 動態 pattern 切換在不修改 SNS3 原生碼的前提下不可行：

| 路徑 | 原因 |
|------|------|
| `SetTxAntennaGainPattern()` | `NS_ASSERT(m_antennaGainPattern == nullptr)` — 只允許初始化一次 |
| `GetAntennaGain_lin()` | non-virtual，Proxy Pattern 無法攔截 |
| `SatHelper` attribute 注入 | 無可用 attribute 接口 |

**降級決策：**
- `SatBeamPatternSelector` 改為 setup-time 靜態分配（選配，延後）
- 動態控制由 `SatUserAssociator`（UT 分配）+ `SatPowerAllocator`（功率）承擔

---

### Phase C / D / E 的最終實作路線是什麼？

```
Phase C（已完成）
├── SatUserAssociator：WFQ/Priority/RR + MoveUtBetweenBeams
└── SatResourceManager：503 ms self-scheduling loop

Phase D（已完成）
└── SatPowerAllocator：IWFA water-filling，最佳化變數 m_eirpWoGainW

Phase E（今日完成）
├── ConnectTracesPhaseE()：接上真實 SNS3 API
└── PollUtStates()：每 T_frame 查詢 SatTopology，更新 UT 狀態
```

---

## 2、Phase E 實作

### 2.1 SNS3 API 驗證

在 `sns3-satellite-master/` 內確認以下 API：

| API | 位置 | 備註 |
|-----|------|------|
| `SatNcc::MoveUtBetweenBeams(Address, srcSat, srcBeam, dstSat, dstBeam)` | `satellite-ncc.h:249` | 傳入 MAC Address，非 uint32_t |
| `SatOrbiterNetDevice::GetUserPhy(uint32_t beamId)` | `satellite-orbiter-net-device.h:129` | 回傳 `Ptr<SatPhy>` |
| `SatPhy::SetTxMaxPowerDbw(double)` | `satellite-phy.h:259` | 設定後需呼叫 Initialize() |
| `SatPhy::Initialize()` | `satellite-phy.cc:208` | 重算 m_eirpWoGainW，可安全多次呼叫 |
| `SatTopology::GetUtNodes()` | `satellite-topology.h:292` | 回傳 NodeContainer |
| `SatTopology::GetUtSatId(Ptr<Node>)` | `satellite-topology.h:626` | 過濾 UT 所屬衛星 |
| `SatTopology::GetUtBeamId(Ptr<Node>)` | `satellite-topology.h:635` | 查詢 UT 現在所在 beam |
| `SatTopology::GetOrbiterNode(uint32_t)` | `satellite-topology.h:403` | 取衛星 Node |
| `SimulationHelper::GetSatelliteHelper()->GetBeamHelper()->GetNcc()` | `simulation-helper.h:486` | 取得 SatNcc 指標 |

---

### 2.2 設計決策

**utId 慣例**：採用 `SatTopology::GetUtNodes()` 的 0-based container index，在 `BuildUtAddressMap()` 與 `PollUtStates()` 之間保持一致。

**Power 寫回機制**：
- `SatPhy::Initialize()` 是 SatPhy 自己的 setup method（`satellite-phy.cc:208`），不是 ns-3 `Object::Initialize()`
- 多次呼叫安全：重設 callback 為 replace 語意，satId/beamId 設定為 idempotent
- 序列：`phy->SetTxMaxPowerDbw(dbw)` → `phy->Initialize()`

**Phase E 啟動條件**：
- `enablePhaseE=1` AND `enableResourceManager=1` AND `SetSimulationHelper()` 已呼叫
- 任一缺失 → NS_LOG_WARN 跳過，不影響 Phase C/D 計算

---

### 2.3 修改檔案

#### `sat-bh-helper.h`

**新增至 `BhExperimentConfig`：**
```cpp
bool enablePhaseE{false};  // Phase E: wire callbacks to real SNS3 APIs
```

**新增公開方法：**
```cpp
void SetSimulationHelper(Ptr<SimulationHelper> simHelper);
```

**新增私有方法：**
```cpp
void BuildUtAddressMap();      // container idx → MAC Address
void CacheOrbiterDevice();     // 快取 SatOrbiterNetDevice
void ConnectTracesPhaseE();    // 主要 wiring 方法
void PollUtStates();           // 每 T_frame 查詢 SatTopology
```

**新增成員欄位：**
```cpp
Ptr<SimulationHelper>       m_simHelper;      // Phase E: NCC + topology 存取
std::map<uint32_t, Address> m_utAddressMap;   // Phase E: UT index → MAC addr
Ptr<SatOrbiterNetDevice>    m_orbiterDev;     // Phase E: 快取衛星 net device
```

---

#### `sat-bh-helper.cc`

**新增 includes（Phase E）：**
```cpp
#include "ns3/satellite-net-device.h"
#include "ns3/satellite-ncc.h"
#include "ns3/satellite-orbiter-net-device.h"
#include "ns3/satellite-phy.h"
#include "ns3/satellite-topology.h"
#include "ns3/simulation-helper.h"
#include "ns3/singleton.h"
```

**`BuildUtAddressMap()`**：
- 遍歷 `SatTopology::GetUtNodes()`，對每個 UT 節點找第一個 `SatNetDevice`
- 儲存 `m_utAddressMap[containerIdx] = satDev->GetAddress()`

**`CacheOrbiterDevice()`**：
- 取 `GetOrbiterNode(m_cfg.satId)` 後遍歷 devices，找 `SatOrbiterNetDevice`
- 儲存為 `m_orbiterDev`（ApplyPowerCallback 使用）

**`ConnectTracesPhaseE()`** 執行順序：
1. `BuildUtAddressMap()` + `CacheOrbiterDevice()`
2. 取 `SatNcc`，設定 `MoveUtCallback` → `ncc->MoveUtBetweenBeams(m_utAddressMap[utId], ...)`
3. 設定 `ApplyPowerCallback` → `phy->SetTxMaxPowerDbw(dbw); phy->Initialize();`
4. `Simulator::Schedule(t=0, PollUtStates)`

**`PollUtStates()`**：
- 遍歷 `GetUtNodes()`，過濾 `GetUtSatId() == m_cfg.satId`
- 以 `GetUtBeamId()` 建立 `UtInfo`，呼叫 `m_resourceManager->UpdateUtState()`
- Demand = 1000 kbps（合成值，Phase F 接真實 DAMA trace 取代）
- 每 `T_frame` 重排程

**`Install()` 加入 Phase E 啟動段：**
```cpp
if (m_cfg.enablePhaseE) {
    if (!m_resourceManager)      → NS_LOG_WARN + skip
    else if (!m_simHelper)       → NS_LOG_WARN + skip
    else                         → ConnectTracesPhaseE()
}
```

---

#### `sat-bh-example.cc`

```cpp
// Phase E CLI arg
cmd.AddValue("enablePhaseE",
    "Phase E: wire callbacks to real SNS3 APIs", cfg.enablePhaseE);

// Pass simHelper before Install
if (cfg.enablePhaseE)
    bhHelper->SetSimulationHelper(simHelper);

// Startup log 新增一行
<< "\n  PhaseE(RealSNS3Wiring=" << cfg.enablePhaseE << ")"
```

---

## 3、SatBhKpiLogger 設計

### 3.1 背景

`SatBhMetrics` 只記錄 packet-level beam KPI，無法覆蓋五個核心成功指標。新建 `SatBhKpiLogger` 紀錄指標`Capacity-Demand Gap`,`E2E Latency `,`Packet Delivery Rate`,`Power Consumption`,`User Association Count`。

---

### 3.2 成功指標（5 KPI）

| 指標 | 說明 | 來源 |
|------|------|------|
| Capacity-Demand Gap | ∑\|R_k − D_k\|，最小化分配與需求差距 | `UnmetCapacityTrace` + `UsableCapacityTrace` |
| E2E Latency | 平均 / 最大端到端延遲 | `SatNetDevice::RxDelay` |
| Packet Delivery Rate | rxPackets / txPackets per frame | Tx/Rx 計數器 |
| Power Consumption | per-beam txMaxPowerDbw | `SatPowerAllocator::PowerMap` |
| User Association Count | assigned_ut_count per beam | `AssignmentMap` |

---

### 3.3 架構決策

| 決策點 | 結論 | 理由 |
|--------|------|------|
| 新建 vs 擴充 `SatBhMetrics` | **新建 `SatBhKpiLogger`** | SatBhMetrics 職責保持 packet-level；frame-level KPI 性質不同 |
| 持有者 | **`SatResourceManager`** holds `Ptr<SatBhKpiLogger>` | RunFrameOptimization 是唯一同時擁有全部資料的時間點 |
| SNS3 接線者 | **`SatBhHelper::ConnectTraces()`** | Helper 是唯一對外接觸 SNS3 的物件 |
| flush 觸發 | **`RunFrameOptimization()` 結束時** | 保證 frame_id 與所有資料對齊，無 timer 漂移 |
| feature flag | **`kpiOutputFile{"scheduler-kpi.csv"}`** | 空字串關閉，與 metricsOutputFile 慣例一致，不加 bool flag |

---

### 3.4 CSV Schema

```
frame_id, sim_time_s, sat_id, beam_id,
allocated_kbps, sns3_demand_kbps, bh_demand_kbps, unmet_kbps, gap_kbps,
tx_power_dbw, assigned_ut_count,
avg_delay_ms, max_delay_ms, pdr_pct, sample_count
```

| 欄位 | 說明 | 來源 |
|------|------|------|
| `allocated_kbps` | R_k = usableCapacity | `UsableCapacityTrace` |
| `sns3_demand_kbps` | SNS3 實際 RCST 請求總和 | `usable + unmet` |
| `bh_demand_kbps` | BH 系統估計需求（UtInfo 聚合） | `AssignmentMap × UtInfo::demandKbps` |
| `unmet_kbps` | 未滿足需求 | `UnmetCapacityTrace` |
| `gap_kbps` | sns3_demand − allocated | 計算值 |
| `tx_power_dbw` | per-beam TX 功率 | `SatPowerAllocator::PowerMap` |
| `assigned_ut_count` | 該 beam 分配的 UT 數 | `AssignmentMap` |
| `avg_delay_ms` | frame 內全局平均延遲 | `RxDelay` rolling avg（Phase C/D 為 global） |
| `max_delay_ms` | frame 內全局最大延遲 | `RxDelay` rolling max |
| `pdr_pct` | Packet Delivery Rate [%] | rx / tx 計數 |
| `sample_count` | frame 內封包樣本數（品質指標） | `RxDelay` 觸發次數 |


**delay/PDR **：Phase C/D 為 global per frame（每個 beam row 填同值），Phase E 改 per-beam。

---

### 3.5 SNS3 路徑

**UnmetCapacityTrace / UsableCapacityTrace：**
```cpp
Ptr<SatNcc> ncc = simHelper->GetSatelliteHelper()->GetBeamHelper()->GetNcc();
for (uint32_t beamId : beamSet) {
    Ptr<SatBeamScheduler> sched = ncc->GetBeamScheduler(satId, beamId);
    sched->TraceConnectWithoutContext("UnmetCapacityTrace",
        MakeBoundCallback(&SatBhKpiLogger::OnUnmetCapacity, logger, satId, beamId));
    sched->TraceConnectWithoutContext("UsableCapacityTrace",
        MakeBoundCallback(&SatBhKpiLogger::OnUsableCapacity, logger, satId, beamId));
}
```

---

### 3.6 I/O

| 設計 | 頻率 | I/O 負擔 |
|------|------|----------|
| packet callback（bucket 累積） | 每封包 | 零 I/O（in-memory only） |
| frame flush（寫 CSV row） | 每 503ms × beams | 極低（30s / 8 beams ≈ 477 行） |


---

### 3.7 實作順序

```
1. sat-bh-kpi-logger.h / .cc        ← 新建，GetTypeId()、DoInitialize()、DoDispose()
2. CMakeLists.txt                    ← 加入 sat-bh-kpi-logger.cc
3. sat-bh-resource-manager.h / .cc  ← 加 Ptr<SatBhKpiLogger>、SetKpiLogger()、
                                        RunFrameOptimization() 末尾呼叫 OnFrameOptimized()
4. sat-bh-helper.h / .cc            ← BhExperimentConfig 加 kpiOutputFile、
                                        SetupPhaseC() 建立 KpiLogger、
                                        ConnectTraces() 接 UnmetCapacityTrace / RxDelay
5. sat-bh-example.cc                ← ParseConfig() 加 --kpiFile CommandLine argument
```

---

### 3.8 code and function

#### 新建 `SatBhKpiLogger`（不擴充 `SatBhMetrics`）

| 項目 | 說明 |
|------|------|
| **新建檔案** | `Beam Hopping Controller/Codes/sat-bh-kpi-logger.h` / `.cc` |
| **對應現有** | `sat-bh-metrics.h` — 現有 packet-level beam KPI，職責不變 |
| **功能** | KpiLogger 接收 frame-level 資料（allocation、demand、power、user count）與 packet-level 滾動統計（delay、PDR），每幀輸出一行 `scheduler-kpi.csv`，提供覆蓋 5 KPI 的統一觀測點 |

**新建 class 核心 API：**
```cpp
// 由 SatBhHelper 在 DoInitialize() 後開檔；DoDispose() 關檔
void SetOutputFile(const std::string& path);

// 由 SatBhHelper::ConnectTraces() 接 SatBeamScheduler trace
void OnUsableCapacity(uint32_t satId, uint32_t beamId, uint32_t kbps);
void OnUnmetCapacity(uint32_t satId, uint32_t beamId, uint32_t kbps);

// 由 SatBhHelper::ConnectTraces() 接 SatNetDevice::RxDelay
void OnPacketDelay(Time delay);
void OnPacketTx();   // Tx 計數，PDR 分母

// 由 SatResourceManager::RunFrameOptimization() 末尾呼叫
// 聚合 bucket → 計算各欄 → 寫一行 CSV → 重置 bucket
void OnFrameOptimized(uint32_t frameId,
                      const std::map<uint32_t, BeamConfig>& beamConfigs,
                      const std::vector<UtInfo>& utList,
                      const AssignmentMap& assignment);
```

---

#### `SatResourceManager` 持有 KpiLogger，`RunFrameOptimization()` 觸發 flush

| 項目 | 說明 |
|------|------|
| **修改檔案** | `sat-bh-resource-manager.h` / `sat-bh-resource-manager.cc` |
| **新增成員** | `Ptr<SatBhKpiLogger> m_kpiLogger` |
| **新增方法** | `void SetKpiLogger(Ptr<SatBhKpiLogger> logger)` |
| **修改方法** | `RunFrameOptimization()` — 在步驟 7（排程下一幀）之前插入 KpiLogger 呼叫 |

**`RunFrameOptimization()` 修改位置（`sat-bh-resource-manager.cc`）：**
```cpp
// 步驟 6：fire FrameConfigCallback（現有）
if (m_frameConfigCb) m_frameConfigCb(m_frameId, m_beamConfigs);

// 步驟 6.5：flush KPI（新增）
if (m_kpiLogger) {
    m_kpiLogger->OnFrameOptimized(
        m_frameId, m_beamConfigs,
        GetUtList(),          // std::vector<UtInfo> from m_utStateMap
        m_associator ? m_associator->GetLastAssignment() : AssignmentMap{});
}

// 步驟 7：排程下一幀（現有）
Simulator::Schedule(m_frameDuration, &SatResourceManager::RunFrameOptimization, this);
```

**功能**：frame_id、BeamConfig（allocation、power）、UtInfo（bh_demand）、AssignmentMap（user count）在同一時間點傳給 KpiLogger，保證欄位對齊，無 timer 漂移。

---

#### `SatBhHelper` 負責建立 KpiLogger 並接 SNS3 trace

| 項目 | 說明 |
|------|------|
| **修改檔案** | `sat-bh-helper.h` / `sat-bh-helper.cc` |
| **新增 config 欄位** | `BhExperimentConfig::kpiOutputFile{"scheduler-kpi.csv"}` |
| **新增成員** | `Ptr<SatBhKpiLogger> m_kpiLogger` |
| **修改方法** | `SetupPhaseC()` 建立並 Initialize KpiLogger；`ConnectTraces()` 接 SNS3 trace |

**`SetupPhaseC()` 新增段（`sat-bh-helper.cc`）：**
```cpp
// 建立 KpiLogger（與 ResourceManager 同批建立）
if (!m_cfg.kpiOutputFile.empty()) {
    m_kpiLogger = CreateObject<SatBhKpiLogger>();
    m_kpiLogger->SetOutputFile(m_cfg.kpiOutputFile);
    m_kpiLogger->Initialize();
    if (m_resourceManager)
        m_resourceManager->SetKpiLogger(m_kpiLogger);
}
```

**`ConnectTraces()` 新增（`sat-bh-helper.cc`）：**
```cpp
if (m_kpiLogger && m_simHelper) {
    // 接 UnmetCapacityTrace / UsableCapacityTrace
    // 來源：satellite-beam-scheduler.cc:269（SatBeamScheduler per-beam trace）
    // 取得路徑：SimulationHelper → SatHelper → SatBeamHelper → SatNcc → GetBeamScheduler()
    Ptr<SatNcc> ncc = m_simHelper->GetSatelliteHelper()
                                  ->GetBeamHelper()->GetNcc();
    for (uint32_t beamId : m_beamSet) {
        Ptr<SatBeamScheduler> sched = ncc->GetBeamScheduler(m_cfg.satId, beamId);
        if (!sched) continue;
        sched->TraceConnectWithoutContext("UsableCapacityTrace",
            MakeBoundCallback(&SatBhKpiLogger::OnUsableCapacity,
                              m_kpiLogger, m_cfg.satId, beamId));
        sched->TraceConnectWithoutContext("UnmetCapacityTrace",
            MakeBoundCallback(&SatBhKpiLogger::OnUnmetCapacity,
                              m_kpiLogger, m_cfg.satId, beamId));
    }

    // 接 RxDelay（global delay / PDR，Phase E 升級 per-beam）
    // 來源：satellite-net-device.cc:104（SatNetDevice per-packet trace）
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::SatNetDevice/RxDelay",
        MakeCallback(&SatBhKpiLogger::OnPacketDelay, m_kpiLogger));
}
```

**功能**：KpiLogger 透過 `UnmetCapacityTrace` / `UsableCapacityTrace` 取得 `sns3_demand_kbps` 和 `allocated_kbps`（SNS3 真實值）；透過 `RxDelay` 累積全局 delay / PDR。所有接線集中在 Helper。

---

#### demand 兩欄分開（`sns3_demand_kbps` + `bh_demand_kbps`）

| 欄位 | 計算方式 | 對應 code |
|------|----------|-----------|
| `sns3_demand_kbps` | `usable + unmet`（SNS3 RCST 請求總和） | `OnUsableCapacity()` + `OnUnmetCapacity()` 累積，`OnFrameOptimized()` 加總 |
| `bh_demand_kbps` | `Σ utInfo.demandKbps` for UTs assigned to beam | `OnFrameOptimized()` 內，遍歷 `utList` + `assignment` 聚合 |
| `gap_kbps` | `sns3_demand − allocated` | `OnFrameOptimized()` 計算後直接寫入 |

**功能**：兩欄並列量化BH 系統對需求的估計誤差（`bh_demand` vs `sns3_demand`），這個差值本身是 UtInfo synthetic model 的準確度指標。

---

## 4、BhSlotEntry 重構實作

### 4.1 修改動機

`BhSlotEntry` 原有 `beamRadius : BeamRadiusType`（slot-wide 單一值），無法表達同一 slot 內多個 beam 各自使用不同 pattern。改為 per-beam map

### 4.2 修改檔案一覽

| 檔案 | 修改內容 |
|------|---------|
| `sat-bh-time-plan.h` | 移除 `beamRadius`；加 `beamPatterns : std::map<uint32_t, BeamRadiusType>`；新增 `SetBeamPattern()` / `GetBeamPattern()` 宣告 |
| `sat-bh-time-plan.cc` | 更新兩個 constructor（移除 `beamRadius` init）；實作 `SetBeamPattern` / `GetBeamPattern`；更新 `Print()` 輸出 per-beam pattern；更新 `ToCsv()` 改為 `bid:PATTERN` 格式 |
| `sat-bh-scheduler.cc` | `BuildPlan()` Step 5 已改用 `slot.SetBeamPattern(beamId1, beamRadius[bIdx])` per-beam 寫入（舊版用 `slot.beamRadius = ...` slot-wide 寫入） |
| `sat-bh-helper.cc` | `BuildStaticBhtp()` 改為逐 beam 呼叫 `slot.SetBeamPattern(bid, BeamRadiusType::SMALL/LARGE)`，hotspot beam → SMALL，non-hotspot beam → LARGE |

### 4.3 主要 API 變更

**`BhSlotEntry` 新增方法：**

```cpp
// 設定指定 beam 的 antenna pattern（beamId 1-indexed）
void SetBeamPattern(uint32_t beamId, BeamRadiusType pattern);

// 取得指定 beam 的 pattern（找不到時回傳 defaultVal）
BeamRadiusType GetBeamPattern(uint32_t beamId,
                              BeamRadiusType defaultVal = BeamRadiusType::MIDDLE) const;
```

**`BuildStaticBhtp()` 修改前後對比：**

```cpp
// 舊版（slot-wide，後寫覆蓋先寫）
slot.beamRadius = slotHasHotspot ? BeamRadiusType::SMALL : BeamRadiusType::LARGE;

// 新版（per-beam，每個 beam 獨立設定）
slot.SetBeamPattern(bid, BeamRadiusType::SMALL);   // hotspot beam
slot.SetBeamPattern(bid, BeamRadiusType::LARGE);   // non-hotspot beam
```

**`Print()` 輸出格式變化：**

```
# 舊格式（slot-wide）
Slot [0 ms .. 26 ms]  beams={1,4}  radius=SMALL  ...

# 新格式（per-beam）
Slot [0 ms .. 26 ms]  beams={1,4}  radius=1:SMALL,4:LARGE  ...
```

**`ToCsv()` beamPatterns 欄位格式：**

```
# 舊格式
...,SMALL,...

# 新格式
...,1:SMALL;4:LARGE,...
```

---

## 5、Phase E 驗證結果（bh_phasee_v2.log）

### 5.1 測試指令

```bash
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 \
           --satId=1 --simTime=60" 2>&1 | tee bh_phasee_v2.log
```

**參數：**
- `--satId=1`：2-satellite 場景所有 UT 均在 SAT 1，必須傳入值；預設 `satId=0` 會導致 `PollUtStates()` 過濾掉全部 UT


### 5.2 Phase E Wiring 確認

```
SatBhHelper::BuildUtAddressMap: scanning 5 UT nodes
SatBhHelper::BuildUtAddressMap: mapped 5 of 5 UTs           ← 5/5 UT address map 建立成功
SatBhHelper::CacheOrbiterDevice: found SatOrbiterNetDevice on sat 1 (device index=0)
SatBhHelper::ConnectTracesPhaseE: MoveUtCallback wired to SatNcc      ← UT move hook 接上
SatBhHelper::ConnectTracesPhaseE: ApplyPowerCallback wired to SatOrbiterUserPhy  ← 功率寫回 hook 接上
SatBhHelper::ConnectTracesPhaseE: scheduling PollUtStates() every T_frame=503ms  ← 輪詢on
```

### 5.3 ResourceManager 自排程迴圈

```
SatResourceManager::RunFrameOptimization frameId=1  t=0s      utCount=0   ← PollUtStates 未執行
SatResourceManager::RunFrameOptimization frameId=2  t=0.503s  utCount=5   ← 第一次 Poll 後 UT 注入
SatResourceManager::RunFrameOptimization frameId=3  t=1.006s  utCount=5
...
SatResourceManager::RunFrameOptimization frameId=120 t=59.857s utCount=5  ← 全程穩定
```

- frameId=1 的 utCount=0 ：`DoInitialize()` 在 t=0 立即觸發 RM，但 `PollUtStates()` 也排在 t=0 執行，執行順序造成第一幀 UT 尚未注入
- frameId=2 起 utCount=5，之後全程穩定，共 120 個 frame（60s / 503ms ≈ 119.3，結果為 120）

### 5.4 Per-beam Pattern 輸出確認

```
Slot [0 ms .. 26 ms]  beams={1,4}  radius=1:SMALL,4:LARGE  modcod=5  clusters={1,4}
```

### 5.5 模擬結果

```
[BH Example] Simulation complete.
  KPI metrics  : bh-metrics.csv
  BHTP table   : bh-timeplan.csv
  SNS3 stats   : data/
  Attributes   : bh-attributes.xml
```



---

## 六、未完成事項

| 項目 | 說明 |
|------|------|
| Phase F — 真實 Demand | `PollUtStates()` 目前使用合成需求（1000 kbps），Phase F 接 `SatGwMac` DAMA 請求佇列 trace |
| Phase F — 真實 CSI | `SatResourceManager` channelGains 目前為 1.0，Phase F 接 `SatPhy::CnoInfo` trace |
| 4 種實驗情境 | `sat-bh-example.cc` 加入 Baseline / BH-only / BH+Scheduling / Full System 四種情境腳本 |
