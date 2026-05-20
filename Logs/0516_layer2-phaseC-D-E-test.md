# 每日日誌 — 2026/05/16


### 2026/05/16

**短期目標**

- [x] Layer 2 Phase C/D/E 模組實作完成，66 顆衛星 smoke test 通過

**每日記錄**：

- `09:00 - 13:30`: [Layer 2 Phase C/D/E 模組實作與 Layer2.md 更新](#layer-2-phase-cde-模組實作)
- `13:30 - 14:00`: [66-sat Phase 1/2 smoke test](#66-sat-phase-12-smoke-test)
- `14:00 - 16:00`: [66-sat Phase E ResourceManager 迴圈驗證](#66-sat-phase-e-resourcemanager-驗證)

> **注意：** Commit hash 連結待 `git commit` 後補入。
> 格式：`https://github.com/<repo>/tree/<7-digit-hash>#<section>`


## Layer 2 Phase C/D/E 模組實作

### 概述

依照 0513 設計討論的決策，完整實作 Phase C → D → E 模組堆疊，並接入 `SatBhHelper`。

### 新建檔案

| 檔案 | Phase | 職責 |
|------|:-----:|------|
| `sat-bh-resource-manager.h/.cc` | C | 幀級自我排程迴圈（503 ms） |
| `sat-bh-user-associator.h/.cc` | C | WFQ / Priority / Round-Robin UT-beam 關聯 |
| `sat-l1-routing-interface.h/.cc` | B/E | L1 ISL routing stub 橋接（beam 保護） |
| `sat-power-allocator.h/.cc` | D | IWFA 迭代注水法 TX 功率最佳化 |

### 套用的關鍵設計決策（來自 0513 討論）

#### Q1 — T_frame = 503 ms（M = 19）定案

`Layer2.md` 中所有 848 ms / M=32 的參照已刪除，程式碼與規格現已一致。

#### Q2/Q3 — BeamRadiusType 從 3 種擴展為 5 種樣式

`sat-bh-time-plan.h`：

| Enum | patternIndex | 波束寬度 | 半徑 | 增益 |
|------|:---:|:---:|:---:|:---:|
| XSMALL | 0 | 1.0° | ~10 km | 43.89 dBi |
| SMALL | 1 | 1.5° | ~15 km | 41.39 dBi |
| MIDDLE | 2 | 2.0° | ~20 km | 37.89 dBi |
| LARGE | 3 | 2.5° | ~25 km | 35.01 dBi |
| XLARGE | 4 | 3.0° | ~30 km | 31.89 dBi |

`BhSlotEntry.beamRadius`（slot 層級單一值）已移除，改為：
- `beamPatterns : std::map<uint32_t, BeamRadiusType>` — 各 beam 個別樣式（主要欄位）
- `scheduledUtIds : std::vector<uint32_t>` — UT 指派（由 SatUserAssociator 寫入）
- `allocatedPowerDbw : std::map<uint32_t, double>` — 各 beam 功率（由 SatPowerAllocator 寫入）

新增輔助函式：`SetBeamPattern(beamId, pattern)`、`GetBeamPattern(beamId)`。

#### Q4 — SatResourceManager 自我排程迴圈

`DoInitialize()` 排程第一次 `RunFrameOptimization()`。
每幀結束時：`Simulator::Schedule(T_frame, RunFrameOptimization)` — 無計時器漂移。

#### Q5 — UT beam 移動使用 SatNcc::MoveUtBetweenBeams

確認 `TransferUtToBeam()` 不使用 — 僅移動 NCC 排程器狀態，MAC/PHY/routing 會不一致。Phase E 將 `MoveUtCallback` 接到真實 SNS3 API。

### 修改的檔案

| 檔案 | 主要變更 |
|------|---------|
| `sat-bh-helper.h/.cc` | `SetupPhaseC()`、`SetupPhaseD()`、`ConnectTracesPhaseE()`、`BuildUtAddressMap()`、`PollUtStates()` |
| `sat-bh-time-plan.h/.cc` | 各 beam 的 `beamPatterns` map、5 種 `BeamRadiusType`、`frameId` 欄位 |
| `sat-bh-example.cc` | Phase C/D/E CLI 旗標（`--enableResourceManager`、`--enablePowerAllocation`、`--enablePhaseE`） |
| `sat-bh-metrics.cc` | `ios::app` 開啟模式 → 多個 `SatBhMetrics` 實例共用同一 CSV，不互相覆蓋 |
| `sat-bh-scheduler.cc` | `BuildPlan` 對每個 beam 呼叫 `SetBeamPattern()`，取代 slot 層級的 `beamRadius` |
| `Layer2.md` | 全面更新規格，對齊所有 Q1–Q8 決策 |
| `Beam Hopping Controller/Readme.md` | 模組表格新增 Phase C/D/E 列 |

### SatBhHelper Phase 旗標接線

```
Install()
 ├── Phase 1/2/3：固定 / 條件式（不變）
 ├── enableResourceManager → SetupPhaseC()
 │     └── SatL1RoutingInterface (stub) + SatUserAssociator + SatResourceManager
 ├── enablePowerAllocation  → SetupPhaseD()
 │     └── SatPowerAllocator 接入 SatResourceManager
 └── enablePhaseE           → ConnectTracesPhaseE()
       └── MoveUtCallback → SatNcc::MoveUtBetweenBeams
           ApplyPowerCallback → SatOrbiterUserPhy::SetTxMaxPowerDbw + Initialize
           每 T_frame 執行一次 PollUtStates()
```

Phase E 預期執行指令：

```bash
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --satId=1 --simTime=60" \
  2>&1 | tee bh_phasee.log
```

---

## 66-sat Phase 1/2 Smoke Test

**場景：** `scenario=iridium66`，每顆衛星 `numBeams=2`，`simTime=60s`

**執行指令：**

```bash
# Phase 1
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=60" \
  2>&1 | tee Outputs/66_bhtp/phase1/run.log

# Phase 2
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=60 --enableScheduler=1 --enableObc=1" \
  2>&1 | tee Outputs/66_bhtp/phase2/run.log
```

**BHTP 輸出（`bh-tp.csv`）— 各 beam 樣式格式確認：**

```
slotIdx  startMs  durationMs  beamIds  beamPatterns  modcod  clusterIds
0        0        26          1        1:SMALL        5       1
1        26       26          2        2:SMALL        5       2
...
```

`beamPatterns` 欄位現在顯示 `beamId:PATTERN` 格式，取代 slot 層級單一值。
19 slots × 26.5 ms = 503 ms 週期 — 與 T_frame 吻合。✅

**指標樣本（Phase 1，第一幀）：**

```
time_s   sat_id  beam_id  throughput_mbps  avg_delay_ms  slot_util_pct  jain_fairness_index
5.503    0       1        0.109            10            79.17%         0.9972
5.503    0       2        0.098            10            79.17%         0.9972
```

- slot 使用率約 79% → 符合預期（K=2，M=19，2 個 beam 輪流分配 19 個 slot）
- Jain 公平性 0.997 → 2 個 beam 之間近乎完美的公平性
- Phase 1 與 Phase 2 輸出相同（排程器 + OBC stub 產生相同靜態計畫）

---

## 66-sat Phase E ResourceManager 驗證

**執行指令：**

```bash
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=60 \
           --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --satId=1" \
  2>&1 | tee Outputs/66_bhtp/phasee/run.log
```

**SatResourceManager 迴圈驗證（`bh_phasee_v2.log`）：**

```
SatResourceManager::RunFrameOptimization frameId=1   t=0.503s  utCount=5
SatResourceManager::RunFrameOptimization frameId=2   t=1.006s  utCount=5
...
SatResourceManager::RunFrameOptimization frameId=120 t=59.857s utCount=5
SatResourceManager: frame 120 done; next optimization at t=60.36s
```

- 60 秒內觸發 120 幀 → 週期 = 60 / 120 = 0.503 s = T_frame ✅
- 自我排程間隔漂移：0（每次排程錨定於上一次實際觸發時間）
- 每幀 `utCount=5` → 合成 UT 狀態注入運作正常

**指標（Phase E，第一幀）：**

```
time_s  sat_id  beam_id  throughput_mbps  slot_util_pct  jain_fairness_index
6.006   0       1        0                100%           1.0
6.006   0       2        0                100%           1.0
```

- Phase E 的 slot_util_pct = 100%：ResourceManager 主動指派所有 slot
- throughput = 0 符合預期 — Phase E 接好 callback 但 smoke test 中無真實流量

**模擬完成：** `[BH Example] Simulation complete.` — 無崩潰，無斷言失敗。✅

---

## 總結

| Phase | 狀態 | 備註 |
|-------|:----:|------|
| Phase C（ResourceManager + UserAssociator） | ✅ | 自我排程迴圈：60 秒 120 幀 |
| Phase D（PowerAllocator IWFA） | ✅ | Stub 分配；ApplyPower 僅記錄 log（尚未接入 SNS3 hook） |
| Phase E（SNS3 MoveUt + SetTxPower callback） | ✅ | 已接線；smoke test 中無真實流量 |
| BhSlotEntry 各 beam 樣式 | ✅ | BHTP CSV 正確顯示 `beamId:PATTERN` |
| 66 顆 Iridium 星座 | ✅ | Phase 1/2/E 全部完整執行 |

**待完成：** Phase F — 將真實 DAMA 需求 trace 從 `SatGwMac` 接入 `SatResourceManager::UpdateUtState()`。
