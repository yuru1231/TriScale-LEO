# 2026-05-15 Daily Log
## 今日目標

1. 將模擬場景從 2-sat 切換至 Iridium-like 66-sat
2. 實作 `sat-bh-example.cc` 多衛星 BH helper 迴圈
3. 加入 `--scenario` 參數，支援 `leo2sat` / `iridium66` 兩種場景切換
4. 修正 BeamRadiusType（3 → 5 pattern）與 BhSlotEntry 結構

---

## 工作紀錄

### 1. Multi-sat BH + Handover Architecture Design

**目標**：66 顆衛星全部跑 BH，支援衛星間 handover。

**架構**：
- 每顆衛星各自一個 `SatBhHelper`（66 個）
- Handover 透過 `PollUtStates()` 的 `utSatId` 過濾自動處理
  - 舊星 helper 下個 poll cycle 自然過濾掉已換星的 UT
  - 新星 helper 自動接管
- 不需要額外 handover callback

---

### 2. SNS3 Iridium-66 Scenario Analysis

**場景**：`constellation-iridium-66-sats`

```
tles.txt:     66 衛星，11/plane × 6 planes
isls.txt:     132 ISL（in-plane + cross-plane）
gw_positions: 5 個 GW
ut_positions: 5 個 UT
beams:        72 beams（fwdConf.txt / rtnConf.txt）
```

**fwdConf.txt beam distribution（格式：beamId gwId freqColor localBeamIdx）**：

| freqColor | beam 數 |
|---|---|
| 1 | 14 |
| 2 | 15 |
| 3 | 14 |
| 4 | 14 |
| 5 | 15 |
| **Total** | **72** |

72 beams / 66 sats = 6 顆衛星有 2 beams，60 顆有 1 beam。

**決定**：
- 有 2 beams 的衛星開 2 個；有 1 beam 的開 1 個
- 統一設 `numBeams=2`，零需求 beam 由 scheduler 自然分配最少 slot 時間

---

### 3. Code Modifications

**修改檔案**：`sat-bh-example.cc`、`sat-bh-helper.cc`、`sat-bh-helper.h`

#### sat-bh-example.cc — dual scenario + 66-helper loop

| 位置 | 改動 |
|---|---|
| `ParseConfig()` signature | 新增 `std::string& scenario`, `uint32_t& numSats` out-params |
| Scenario block | `iridium66`: all 72 beams, `numSats=66`, load `constellation-iridium-66-sats` |
| Output naming | `{base}_{scenario}_{YYYYMMDD_HHMMSS}.csv` — 每次 run 自動 timestamp，不覆蓋 |
| BH Install | 單 helper → `std::vector<Ptr<SatBhHelper>> bhHelpers(numSats)` 迴圈 |
| FinalFlush | `for (auto& h : bhHelpers)` |

```cpp
constexpr uint32_t kNumSats     = 66;
constexpr uint32_t kBeamsPerSat = 2;

std::vector<Ptr<SatBhHelper>> bhHelpers(numSats);
for (uint32_t satId = 0; satId < numSats; satId++)
{
    bhHelpers[satId] = CreateObject<SatBhHelper>();
    bhHelpers[satId]->Configure(cfg);
    bhHelpers[satId]->SetNumBeams(cfg.numBeams);
    bhHelpers[satId]->SetSatId(satId);
    bhHelpers[satId]->Install();
}
```

#### sat-bh-helper.h — Phase C/D/E flags added to BhExperimentConfig

新增 feature flags：

| Flag | Phase | 功能 |
|---|---|---|
| `enableResourceManager` | C | SatResourceManager self-scheduling loop |
| `enableUserAssociation` | C | SatUserAssociator 每 frame 執行 |
| `enablePatternSelection` | C/E | beam pattern selection |
| `enablePowerAllocation` | D | SatPowerAllocator IWFA |
| `enablePhaseE` | E | wire callbacks to real SNS3 APIs |

新增 getter：`GetResourceManager()`, `GetUserAssociator()`, `GetL1Interface()`

新增方法：`SetResourceManagerEnabled()`, `SetSimulationHelper()`

---

### 4. BeamRadiusType and BhSlotEntry Refactor

**修改檔案**：`sat-bh-time-plan.h`, `sat-bh-time-plan.cc`, `sat-bh-scheduler.cc`

#### BeamRadiusType：3 → 5 patterns

| 值 | beamwidth | 半徑 | 用途 |
|---|---|---|---|
| `XSMALL = 0` | 1.0° | ~10 km | hotspot core |
| `SMALL = 1` | 1.5° | ~15 km | high demand |
| `MIDDLE = 2` | 2.0° | ~20 km | default |
| `LARGE = 3` | 2.5° | ~25 km | low demand |
| `XLARGE = 4` | 3.0° | ~30 km | edge / cold-spot |

#### BhSlotEntry：per-beam pattern 取代 slot-wide beamRadius

| 新增欄位 | 型別 | 用途 |
|---|---|---|
| `beamPatterns` | `std::map<uint32_t, BeamRadiusType>` | per-beam pattern（取代舊 `beamRadius`） |
| `scheduledUtIds` | `std::vector<uint32_t>` | Phase C UserAssociator 填入 |
| `allocatedPowerDbw` | `std::map<uint32_t, double>` | Phase D PowerAllocator 填入 |

#### sat-bh-scheduler.cc — interference radius fix

- `rMiddle` 修正：`40.0 km` → `20.0 km`（符合 MIDDLE 2.0° beamwidth）
- MODCOD 新增 `XSMALL=20`, `XLARGE=2` case
- `BuildPlan()` 改為呼叫 `slot.SetBeamPattern(beamId1, beamRadius[bIdx])`

---

### 5. `--scenario` Parameter and CommandLine Fix

**目標**：`sat-bh-example.cc` 加入 `--scenario` 切換兩種場景。

**問題**：`BhExperimentConfig` struct 沒有 `scenario` / `numSats` 成員。
- **解**：`scenario` 和 `numSats` 保持在 `main()` 的 local 變數，透過 out-param 傳入 `ParseConfig()`

```cpp
static BhExperimentConfig
ParseConfig(int argc, char* argv[], std::string& scenario, uint32_t& numSats)
{
    BhExperimentConfig cfg;
    CommandLine cmd;
    cmd.AddValue("scenario", "leo2sat | iridium66", scenario);
    cmd.AddValue("numSats",  "satellites to install BH on", numSats);
    // ... 其他所有 AddValue ...
    cmd.Parse(argc, argv);
    return cfg;
}

// main() 呼叫：
std::string scenario{"leo2sat"};
uint32_t    numSats{1};
BhExperimentConfig cfg = ParseConfig(argc, argv, scenario, numSats);
```

**場景**：

| 場景 | 指令 | 目的 |
|---|---|---|
| `leo2sat` | `./ns3 run "sat-bh-example"` | 1 衛星 × 7 beams，驗證 EM 排程邏輯、hotspot 分配、slot ratio |
| `iridium66` | `./ns3 run "sat-bh-example --scenario=iridium66"` | 66 衛星 × 2 beams，驗證 handover + multi-sat BH 完整系統 |

**驗證順序**：先跑 `leo2sat` 確認 BHTP CSV 正確，再跑 `iridium66` 做全系統評估。

---

### 6. Simulation Output Validation (leo2sat 7-beam)

**輸出**：`Beam Hopping Controller/Outputs/2satplan/`

#### bh-timeplan.csv 解析

19 slots × 26ms；每 slot 同時啟動 1 hotspot + 1 non-hotspot（K=2）：

| beam | pattern | 出現次數 | 有效 dwell = slots × 24ms |
|---|---|---|---|
| 1 (hotspot) | SMALL | 7 | 168 ms |
| 2, 3 (hotspot) | SMALL | 6 | 144 ms |
| 4, 5, 6 (non-hotspot) | LARGE | 5 | 120 ms |
| 7 (non-hotspot) | LARGE | 4 | 96 ms |

#### bh-metrics.csv （7-beam 段）

| 指標 | Hotspot (beam 1-3) | Non-hotspot (beam 4-7) |
|---|---|---|
| dwell_time_ms | 168 ms | 96–120 ms |
| avg_delay_ms | **10 ms** | **20 ms** |
| throughput_mbps | 0.83–0.95 | 0.13–0.20 |
| slot_util_pct | **79.17%** | **37.5%** |
| drop_rate_pct | ~2.78% (beam1) | 0% |

**驗證**：dwell 與 timeplan slot 數 × 24ms 完全吻合；hotspot/non-hotspot 差異化正確 ✅

---

### 7. bh-metrics.cc Output Bug Fix

**修改**：`sat-bh-metrics.cc:389`

**問題**：`EnsureFileOpen()` 使用 `ios::app`，每次模擬啟動都 append，CSV 出現重複 header。

**修正**：保留 `ios::app`（讓同場景多顆衛星共用同一個檔案），改用 `tellp()==0` 判斷空檔再寫 header。

```cpp
// Before
m_csvFile.open(m_outputPath, std::ios::out | std::ios::app);
if (!m_headerWritten) { /* write header */ }
m_headerWritten = true;

// After
m_csvFile.open(m_outputPath, std::ios::out | std::ios::app);
if (m_csvFile.tellp() == std::streampos(0))
{
    m_csvFile << "time_s,sat_id,beam_id,...\n";  // header only on empty file
}
m_headerWritten = true;
```

---

### 8. Output Naming Redesign

**問題**：
1. 66 顆衛星各自寫 `bh-metrics-sat{id}.csv` → 66 個分散檔案，難分析
2. 多次跑模擬會互相覆蓋

**解法**：`sat-bh-example.cc` 加入 auto-timestamp；所有衛星共用一個 CSV，以 `sat_id` column 區分

```
bh-metrics_iridium66_20260515_143022.csv  (66 rows per frame)
bh-timeplan_iridium66_20260515_143022.csv
```

---

### 9. iridium66 Validation (66sat-fix)

**輸出**：`Beam Hopping Controller/Outputs/66sat-fix/`

#### bh-tp.csv

全 19 slots 都是 `beam1(SMALL) + beam2(LARGE)` 同時啟動，無 hopping。

**原因**：`numBeams=2 = K=maxActiveBeams=2`，兩個 beam 全程並聯，排程器不需切換。

#### bh-metrics.csv

| 指標 | beam1 (hotspot) | beam2 (non-hotspot) |
|---|---|---|
| dwell_ms | 456 ms | 456 ms |
| avg_delay_ms | **10 ms** | **20 ms** |
| slot_util_pct | **79.17%** | **37.5%** |
| avg_throughput | 2.24 Mbps | 0.61 Mbps |
| sat_id 範圍 | 0–65（全 66 顆）✅ | |
| 重複 header | 0 ✅ | |

**驗證**：
- 66 顆衛星全部記錄在同一個 CSV，header 無重複 ✅
- hotspot/non-hotspot delay 分化正確（10ms vs 20ms）✅
- 66 顆數值完全相同 → synthetic demand 均一，預期行為 ✅

**待改善**：
- `numBeams=2 = K=2`，不發生真正的 hopping → 需跑 `--maxActiveBeams=1` 驗證

---

## 未完成 / 待確認

- [x] leo2sat 7-beam BHTP CSV 驗證：hotspot beam slot ratio 明顯高於 non-hotspot ✅
- [x] iridium66 66-sat 全系統驗證（bh-metrics 無重複 header、66 sat 齊全）✅
- [ ] iridium66 加 `--maxActiveBeams=1` 驗證真實 beam hopping 切換
- [ ] 確認 satId=0 在 t=0 服務哪些 beamId（需查 antennapatterns 或執行期 log）
- [ ] 對地投影點覆蓋半徑實作（`R = h / tan(θ)` 接入 UserAssociator）
- [ ] Phase F：接入 SatGwMac DAMA 真實 demand（延後）
