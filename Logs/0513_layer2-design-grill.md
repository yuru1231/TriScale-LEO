# 2026-05-13 Layer 2 Design 

## 概述

 Layer 2（Beam Hopping + User Scheduling + Power Allocation）設計審查，，確定實作路線

---


## 決策詳細紀錄

### Q1 — Frame 結構

**問題**：Layer2.md 原寫 T_frame = 848 ms（M=32），但現有程式碼 `bhtpPeriodMs = 503 ms`（M=19）不一致。

**決策**：統一為 T_frame = **503 ms，M = 19**，與現有 BHTP 週期對齊。

**修改**：
- `Layer2.md` §2.1、§4.2、§3.1、§5.3：所有 848 ms / 32 → 503 ms / 19

---

### Q2/Q3 — BeamRadiusType & BhSlotEntry

**問題 1**：`BeamRadiusType` 原為 3 種（SMALL/MIDDLE/LARGE），擴充成 5 種 pattern。

| Enum | patternIndex | Beamwidth | 半徑 | 增益 |
|------|:---:|:---:|:---:|:---:|
| XSMALL | 0 | 1.0° | ~10 km | 43.89 dBi |
| SMALL | 1 | 1.5° | ~15 km | 41.39 dBi |
| MIDDLE | 2 | 2.0° | ~20 km | 37.89 dBi |
| LARGE | 3 | 2.5° | ~25 km | 35.01 dBi |
| XLARGE | 4 | 3.0° | ~30 km | 31.89 dBi |

**問題 2**：`BhSlotEntry.beamRadius` 是 slot-wide 單一值，無法表達 per-beam 不同 pattern。

**決策**：移除 `beamRadius`，改為：
- `beamPatterns : std::map<uint32_t, BeamRadiusType>` — per-beam pattern（主要）
- `scheduledUtIds : std::vector<uint32_t>` — slot 排程 UT（SatUserAssociator 填入）
- `allocatedPowerDbw : std::map<uint32_t, double>` — per-beam 功率分配（SatPowerAllocator 填入）

新增 helpers：`GetBeamPattern(beamId)`、`SetBeamPattern(beamId, pattern)`

`SatBhTimePlan` 新增 `frameId : uint32_t`（由 SatResourceManager 設定，不加 framePatternIndex）。

---

### Q4/Q5 — Trigger & Hook

**Q4 — SatResourceManager 觸發**

**決策**：Self-scheduling loop
- `DoInitialize()` 排第一次 `RunFrameOptimization()`
- 每次執行結束用 `Simulator::Schedule(T_frame, ...)` 排下一次
- 不依賴外部 timer，避免 timer drift

**Q5 — SatUserAssociator UT 搬移**

**Trace 結果**：不可直接呼叫 `TransferUtToBeam()`（只搬 NCC scheduler 狀態，UT MAC/PHY/routing 不更新）。

**決策**：Pre-planned schedule 設計
```
Frame N：計算 Frame N+1 的 UT-beam 分配 → 立即呼叫 MoveUtBetweenBeams()
Frame N+1：TIM-U 延遲到期，UT 完成切換
```
TIM-U 的 1-frame 延遲是設計的一部分，已預先規劃進去。

---

### Q6/Q7/Q8 — Power Unit & Scheduler Removal

**Q6 — SetTxMaxPowerDbw 單位**

**Trace 結果**：
- `SetTxMaxPowerDbw()` 接受 **dBW**
- 設定後必須呼叫 `phy->Initialize()` 才會重算 `m_eirpWoGainW`
- SNS3 資料流：`TxMaxPowerDbw → m_eirpWoGainW [W] → × G_tx → / FSL → m_rxPower_W`

**Q7/Q8 — SatUserScheduler 廢除**

**Trace 結果**：`SatFrameAllocator::SortUts()` 是 private non-virtual，無法在新 .h/.cc 中 override。

**決策**：移除獨立的 `SatUserScheduler` 模組，WFQ / Priority / RR 排程邏輯合併進 `SatUserAssociator` 的 pre-planned schedule。`sat-user-scheduler.h/.cc` 從新增檔案清單中移除。

---

### Q9 — Optimization Variable

**Trace 結果**：SNS3 channel 實際使用的功率參數是 `m_eirpWoGainW`（TX power 扣掉 output/pointing/OBO/antenna losses 後的 linear W 值）。

**決策**：最佳化變數 = `p_k = m_eirpWoGainW` per beam（linear W）

SINR 公式：
```
SINR_k = (p_k × G_tx_k × G_rx_k / FSL_k) / (σ² + Σ_{j≠k} p_j × G_tx_j × G_rx_k / FSL_jk)
```

寫回 SNS3：
```cpp
double txMaxDbw = SatUtils::WToDbW(p_k) + outputLossDb + pointingLossDb + oboLossDb + antLossDb;
phy->SetTxMaxPowerDbw(txMaxDbw);
phy->Initialize();
```

---

### Q11~Q15 — Pattern Selector Trace

**Trace 結果（SNS3 antenna pattern 系統）**：
1. Pattern 從 `antennapatterns/` 目錄讀入，beamId 對應到 `.txt` 檔名
2. `SatBeamHelper::Install()` 在 `AttachChannels()` 時一次性掛到 PHY
3. `SetTxAntennaGainPattern()` 有 `NS_ASSERT(m_antennaGainPattern == nullptr)`，只允許初始化時呼叫一次
4. `GetAntennaGain_lin()` 不是 virtual，Proxy Pattern 無法攔截
5. `SatHelper` 無 attribute 可注入自訂 pattern container

**結論**：Runtime 動態 pattern 切換在不修改 SNS3 原生碼的前提下不可行。

**降級決策**：
- `SatBeamPatternSelector` 改為 setup-time 靜態分配（一次性）
- 動態控制由 `SatUserAssociator`（UT 分配）+ `SatPowerAllocator`（功率）承擔
- Pattern selection 標為選配功能（feature flag），Phase D 完成後視論文需求再評估

---

### Q16/Q17 — Pattern Injection

**注入點問題**：setup-time pattern 替換需在 `CreateSatScenario()` 之前介入，唯一路徑為 pattern 目錄注入（Path A），但涉及磁碟 I/O 且複雜。

**決策**（Q17）：`SatBeamPatternSelector` 標為選配（Phase C 延後），先完成：
1. Phase D：`SatPowerAllocator`
2. 之後視論文貢獻需求決定是否實作 pattern selection

---

## 程式碼修改摘要 {#code-changes}

| 檔案 | 修改內容 |
|------|---------|
| `sat-bh-time-plan.h` | `BeamRadiusType` 擴充為 5 值；`BhSlotEntry` 移除 `beamRadius` 加 `beamPatterns`/`scheduledUtIds`/`allocatedPowerDbw`；`SatBhTimePlan` 加 `frameId`；新增 `SetBeamPattern`/`GetBeamPattern`/`SetFrameId` 宣告 |
| `sat-bh-time-plan.cc` | 更新 constructor；實作 `GetBeamPattern`/`SetBeamPattern`/`SetFrameId`；更新 `Print()` 改為 per-beam pattern 輸出；更新 `ToCsv()` header 和格式 |
| `sat-bh-scheduler.cc` | `rMiddle` 40→20 km；modcod switch 補 XSMALL/XLARGE；beam radius 賦值改用 `SetBeamPattern()` per beam；移除 slot-level `beamRadius` 賦值 |
| `sat-bh-helper.cc` | `BuildStaticBhtp()` 改用 `SetBeamPattern()` 取代直接賦值 `slot.beamRadius`（詳見下方說明） |
| `Layer2.md` | 全面更新：Frame 參數、BeamRadiusType 說明、BhSlotEntry 設計、SatUserAssociator（含排程邏輯）、SatUserScheduler（廢除）、SatPowerAllocator（最佳化變數）、SatBeamPatternSelector（降為選配）、SNS3 Hook 對照表、Phase C/D、檔案清單 |

### `sat-bh-helper.cc` 修改細節

**函式**：`BuildStaticBhtp()`

**舊寫法（slot-wide 單一值）：**
```cpp
// hotspot beam
slot.beamRadius = BeamRadiusType::SMALL;

// non-hotspot beam — 需要條件判斷，避免覆蓋同一 slot 中已設的 SMALL
if (slot.beamRadius != BeamRadiusType::SMALL)
    slot.beamRadius = BeamRadiusType::LARGE;
```

**問題**：`BhSlotEntry.beamRadius` 是 slot-wide 單一欄位，同一 slot 內有多個 beam 時，後寫的值會覆蓋先寫的，導致只能有一種 pattern。

**新寫法（per-beam map）：**
```cpp
// hotspot beam
slot.SetBeamPattern(bid, BeamRadiusType::SMALL);   // High demand → narrow beam

// non-hotspot beam
slot.SetBeamPattern(bid, BeamRadiusType::LARGE);   // Low demand → wide beam
```

**對應決策**：Q2/Q3（`BhSlotEntry` 重構）。
`beamRadius` 欄位已從 `BhSlotEntry` 移除，改為 `beamPatterns: std::map<uint32_t, BeamRadiusType>`。
每個 beam 獨立設定 pattern，不再需要 if 判斷，slot 內多 beam 可各自持有不同 pattern。

---

## 確定實作路線

```
Phase B（下一步）
├── 擴充 SatBhTimePlan / BhSlotEntry（✅ 已完成）
└── 新增 SatL1RoutingInterface（stub）

Phase C（必做）
├── SatUserAssociator：WFQ pre-planned schedule + MoveUtBetweenBeams
├── SatResourceManager：self-scheduling loop（503 ms）
└── SatBeamPatternSelector：選配，延後

Phase D（必做）
└── SatPowerAllocator：water-filling，最佳化變數 m_eirpWoGainW，寫回 SetTxMaxPowerDbw

Phase E（整合）
├── SatBhHelper：加入 Phase C/D 模組安裝 + feature flags
└── sat-bh-example.cc：4 種模擬情境
    - Baseline：無 BH，靜態 UT 分配，等功率
    - BH-only：有 BH，靜態 UT 分配，等功率
    - BH + QoS：有 BH，WFQ SatUserAssociator，等功率
    - Full System：有 BH，WFQ SatUserAssociator，SatPowerAllocator
```

---

## SNS3 Hook 確認清單（Trace 完成）

| 功能 | API | 狀態 |
|------|-----|------|
| UT beam 重分配 | `SatNcc::MoveUtBetweenBeams()` | ✅ 確認 |
| 功率寫入 | `SatPhy::SetTxMaxPowerDbw()` + `Initialize()` | ✅ 確認（單位：dBW）|
| Pattern 動態切換 | — | ❌ 不可行（non-virtual + assert）|
| 排程插入（SortUts） | — | ❌ 不可行（private non-virtual）|
| Self-scheduling | `Simulator::Schedule()` | ✅ 無需 SNS3 hook |

---




---

### Phase C Config {#phase-c-config}

**修改檔案**：`sat-bh-helper.h`、`sat-bh-example.cc`

新增至 `BhExperimentConfig`：

```cpp
bool     enableResourceManager{false};
bool     enableUserAssociation{true};
bool     enablePatternSelection{false};
uint8_t  schedulingMode{0};           // 0=WFQ, 1=Priority, 2=RoundRobin
uint32_t maxReassignmentPerFrame{5};
double   nominalKbpsPerSlot{50000.0};
double   maxDelayMs{530.0};
```

`sat-bh-example.cc` 新增 CLI arguments，並修正 `uint8_t` 使用 `uint32_t` proxy：

```cpp
uint32_t schedulingModeArg = cfg.schedulingMode;
cmd.AddValue("schedulingMode", "...", schedulingModeArg);
cmd.Parse(argc, argv);
cfg.schedulingMode = static_cast<uint8_t>(schedulingModeArg);
```

**原因**：ns-3 `CommandLine::AddValue` 不支援 `uint8_t` 模板參數，直接傳入會導致 `cmd.Parse()` 靜默呼叫 `exit(0)`，模擬無任何輸出即退出。

---

### SetupPhaseC() 實作 {#setup-phase-c}

**修改檔案**：`sat-bh-helper.cc`、`sns3-satellite-master/CMakeLists.txt`

`Install()` 末端加入：
```cpp
if (m_cfg.enableResourceManager)
    SetupPhaseC();
```

`SetupPhaseC()` 建立順序：
1. `SatL1RoutingInterface`（stub）→ `Initialize()`
2. `SatUserAssociator` → 設定 SchedulingMode / MaxReassignment / MaxDelayMs → `Initialize()`
3. `SatResourceManager` → 設定 FrameDuration / EnableUserAssociation / NominalKbpsPerSlot → `Initialize()`（觸發自排程迴圈）

`CMakeLists.txt` 新增至 `source_files` 與 `header_files`：
```cmake
helper/sat-bh-resource-manager.cc
helper/sat-bh-user-associator.cc
helper/sat-l1-routing-interface.cc
```

---

### Build 問題排查 {#build-debug}

**問題**：`./ns3 build scratch/sat-bh-example` 無輸出，binary 不存在。

**根因**：
1. ns-3 實際 cmake build 目錄為 `cmake-cache/`，不是 `build/`
2. `cmake-cache/` 使用 `Makefile`，不是 `ninja`（`build.ninja` 不存在）
3. `sat-bh-example.cc` 在 cmake configure 之後才加入 scratch，`CONFIGURE_DEPENDS` glob 未重新掃描

**解法**：
```bash
touch ~/workspace/ns-3.43/scratch/CMakeLists.txt   # 強制 cmake 重新執行 GLOB
cd ~/workspace/ns-3.43/cmake-cache
cmake .                                              # 重新 configure，偵測到新 .cc
make scratch_sat-bh-example                         # 建置 target
```

Binary 建立於 `build/scratch/ns3.43-sat-bh-example-default`（2026-05-13 23:52）。

---

### ns-3 Bool Parsing 限制 {#cli-fix}

**問題**：`--enableResourceManager=true` 導致 binary 靜默退出（exit code 0，無任何輸出）。

**根因**：此版本 ns-3 `CommandLine` 對 `bool` 型別只接受 `0` / `1`，**不接受** `true` / `false` 字串。

**確認**：
```bash
# 無效（靜默 exit 0）
./ns3 run "sat-bh-example --enableResourceManager=true"

# 有效（正常執行）
./ns3 run "sat-bh-example --enableResourceManager=1"
```

**修正**：`sat-bh-example.cc` 頂部 USAGE 說明全部改為 `=1`，並加入說明：
```
NOTE: ns-3 CommandLine bool values use 0/1, NOT true/false.
```

---

### 未完成項目

Phase C 自排程迴圈目前能啟動（topology 正常印出），但 `SatResourceManager::RunFrameOptimization()` 的 NS_LOG 輸出尚未驗證，原因不明。明日需確認：

1. `NS_LOG="SatResourceManager=info|prefix_time"` 是否有 `frameId=1,2,...` 輸出
2. 若無，檢查 `DoInitialize()` → `Simulator::Schedule()` 的觸發路徑
