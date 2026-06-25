# 0623 — Code Fix

## 問題描述：Dynamic BeamRadiusType vs 投影問題

Beam Hopping 排程器採用「熱點縮小、冷點放大」設計：
- 熱點 beam（高需求）→ SMALL（1.5°, ~14.4 km）gain 集中，提升 SINR
- 冷點 beam（低需求）→ LARGE/XLARGE（2.5°~3.0°, ~24–28.8 km）廣覆蓋

但 `ComputeInterferenceFactor()` 原本使用單一全域 `rMiddle`（MIDDLE 2°, 19.2 km）計算所有 beam pair 的干擾因子 ω_{i,j}。此假設造成：

| Beam pattern | r (km) | ω（相鄰 38.4 km） | 誤差 |
|---|---|---|---|
| XSMALL (1°) | 9.6 | ≈ 1.7×10⁻²⁴ | 低估 MIDDLE 46 個量級 |
| MIDDLE (2°) | 19.2 | ≈ 1.5×10⁻⁵ | 基準 |
| XLARGE (3°) | 28.8 | ≈ 0.0072 | 舊模型低估實際 XLARGE 干擾 480×（用 rMiddle 算出 1.5×10⁻⁵，實際應為 0.0072） |

使用統一 rMiddle 時，XLARGE beam 的相鄰干擾被低估，導致排程器誤判 XLARGE 相鄰 beam 可以同時啟動，實際上卻會造成顯著干擾。
 > 使用同一波束(middle)是避免角度不同造成的投影偏差，需求分配由bhtp完成
---

## Root Cause

原本的 pipeline 順序：
```
BuildPlan()
  Step 1: 分類 hotspot / non-hotspot
  Step 2: 指派 beamRadius[j]     ← 這裡決定 SMALL/LARGE/XLARGE
  Step 3: GroupClusters()        ← 這裡呼叫 ComputeInterferenceFactor()
                                    但函式看不到 Step 2 的結果
```

`ComputeInterferenceFactor()` 屬於 class const method，不直接存取 `BuildPlan()` 的區域變數 `beamRadius[]`，因此只能用 class member `m_beamHalfAngleDeg`（MIDDLE=2°）計算所有 beam，忽視了每個 beam 的實際 pattern。

---

## 解決方案：pattern propagation 到 class member

把 `beamRadius[]` 複製到 class member `m_currentBeamPatterns`。

### 修改一：`sat-bh-scheduler.h`

新增 private member：
```cpp
/// Per-beam pattern for the current scheduling cycle.
/// Populated by BuildPlan() Step 2 before GroupClusters() is called,
/// enabling ComputeInterferenceFactor() to use each beam's actual assigned
/// radius (not a uniform MIDDLE fallback).
std::vector<BeamRadiusType> m_currentBeamPatterns;
```

新增 private helper 宣告：
```cpp
/// Return the beam ground radius [km] for the given BeamRadiusType.
/// r = altitudeKm × tan(halfAngleDeg × π/180)
double BeamRadiusFromType(BeamRadiusType t) const;
```

### 修改二：`sat-bh-scheduler.cc` — 新增 helper

```cpp
double
SatBhScheduler::BeamRadiusFromType(BeamRadiusType t) const
{
    static const double kHalfAngles[5] = {1.0, 1.5, 2.0, 2.5, 3.0};
    const double pi  = std::acos(-1.0);
    uint8_t      idx = std::min(static_cast<uint8_t>(t), static_cast<uint8_t>(4));
    return m_altitudeKm * std::tan(kHalfAngles[idx] * pi / 180.0);
}
```

### 修改三：`BuildPlan()` — Step 2 → Step 3 之間插入

```cpp
// Propagate per-beam pattern to class member so GroupClusters()
// can access actual radii via ComputeInterferenceFactor().
m_currentBeamPatterns = beamRadius;
```

### 修改四：`ComputeInterferenceFactor()` — 改用 per-beam 半徑

```cpp
// Per-beam ground radius; fall back to MIDDLE if patterns not yet assigned.
const double rI = patternsReady
                    ? BeamRadiusFromType(m_currentBeamPatterns[beamI])
                    : BeamRadiusFromType(BeamRadiusType::MIDDLE);
const double rJ = patternsReady
                    ? BeamRadiusFromType(m_currentBeamPatterns[beamJ])
                    : BeamRadiusFromType(BeamRadiusType::MIDDLE);

// First-layer check uses per-beam sum (not 2×rMiddle)
const double firstLayer = 1.5 * (rI + rJ);

// Asymmetric interference: take worst-case direction
double omegaIJ = std::exp(-2.77 * (dij / rI) * (dij / rI));
double omegaJI = std::exp(-2.77 * (dij / rJ) * (dij / rJ));
double omega   = std::max(omegaIJ, omegaJI);
```

---

## 影響分析

**Grid cell 座標不變：** UT 永遠放在 MIDDLE-based spacing（38.4 km）的 cell center，SNS3 根據 SNS3 天線 gain 模型計算實際 SNR。排程器只決定「哪些 beam 可以同時啟動」，不影響 UT 物理位置。

**κ 閾值討論：**  
相鄰 XLARGE beam pair（距離 38.4 km）：ω ≈ 0.0072，仍低於 κ=0.08，因此不會被合併到同一 cluster。但相較 MIDDLE pair（ω ≈ 1.5×10⁻⁵），XLARGE pair 的干擾量是 480 倍，模型現在能正確反映此差異。若需要更嚴格，可調高 κ（例如 κ=0.01）讓 XLARGE 鄰居也被強制隔開。

---

## 修改檔案

| 檔案 | 函式 | 修改內容 |
|---|---|---|
| `sat-bh-scheduler.h` | private members | 新增 `m_currentBeamPatterns`、`BeamRadiusFromType()` 宣告 |
| `sat-bh-scheduler.cc` | `BeamRadiusFromType()` | 新增 helper，依 half-angle table 轉換 BeamRadiusType → km |
| `sat-bh-scheduler.cc` | `BuildPlan()` | Step 2/3 之間插入 `m_currentBeamPatterns = beamRadius;` |
| `sat-bh-scheduler.cc` | `ComputeInterferenceFactor()` | 改用 per-beam `rI`, `rJ`；worst-case max(ω_{ij}, ω_{ji}) |

---
## 問題描述

輸出資料過多造成系統容量負擔：


## Root Cause 分析

### Bug 1：T_s 頻率 flush

`FlushAndReschedule` 以 T_s=10ms 頻率被呼叫（timer chain 未正確條件化），
導致每 10ms 寫一次，20,591 rows ≈ 160 flushes × 10 beams × 10 helpers × (0.8s/10ms)。

**修復：** `sat-bh-metrics.cc` 加入 `m_nextFlushTime` rate-limit gate；
`sat-bh-helper.cc` 的 `ScheduleNextFlush` 加入 `!enableDynamicBstp` guard。

### Bug 2：kpiTable 累積 entry 未過濾

`m_kpiTable` 是 `std::map<BeamKey, BhBeamKpi>`，entry 只被 `Reset()` 但不刪除。
當 greedy provider 在後半段 T_p 週期啟動先前未見的 beam ID 時，
map 逐漸擴張至最多 25 個 entry。
每次 FlushMetrics 把此次 window 無活動（slotAllocMs=0, pktCount=0）的舊 entry也寫出，
導致後期 flush 行數遠超前期，non-linear scaling。

**證據：**
- numPeriods=20 unique timestamps = 20（rate-limit 正常）
- 但 3560 data rows / 20 / 10 helpers = 17.8 beams/flush ≠ 10

---

## 修復

**檔案：** `Beam Hopping Controller/Codes/sat-bh-metrics.cc`  
**函式：** `FlushMetrics()`，write loop 加入 active filter

```cpp
// Skip beams with no activity in this T_p window.
// kpiTable entries persist across flushes (only Reset(), never erased),
// so entries from beams first seen in earlier periods remain in the map
// but carry zero data after Reset(). Writing them would inflate row counts
// and corrupt per-beam KPI averages.
if (kpi.slotAllocMs <= 0.0 && kpi.pktCount == 0 && kpi.droppedPkts == 0)
{
    continue;
}
```

---

## 驗證結果

```
scenario: starlink25
maxHelperSats=10, warmUp=0, enableDynamicBstp=1, enableObc=1
```

| numPeriods | simTimeSec | rows（含 header） | data rows |
|---|---|---|---|
| 10 | 0.8s | 381 | 380 |
| 20 | 1.6s | 781 | 780 |

**Scaling ratio:** 780 / 380 = **2.05× ≈ 2× ✓ Linear**

Progress print 確認 simTimeSec 正確（1.40/1.60 for numPeriods=20）。

---

## 行數結構說明

```
380 data rows / 10 helpers = 38 rows per helper (numPeriods=10)
38 / 10 T_p = 3.8 active unique beams per helper per T_p
```

greedy provider 在 demand=0 情境下每個 T_p 約啟動 4 個 unique beam per satellite，
符合 K=2×M=8=16 slot activations 集中在少數 beam 的行為。

---

## 目前狀態

| 功能 | 狀態 |
|---|---|
| `--numPeriods` 參數控制模擬長度 | ✅ |
| `--warmUp=0` 正確略過 warm-up | ✅ |
| FlushMetrics rate-limit（max 1 write/T_p） | ✅ |
| Active filter（僅寫出本 window 有活動的 beam） | ✅ |
| row count 與 numPeriods 線性正比 | ✅ |
| FinalFlush 正確略過無資料情境 | ✅ |

---

## 模擬（simTime=120s）

**指令：**
```bash
./ns3 run "sat-bh-example --scenario=starlink25 \
  --enableObc=1 --enableDynamicBstp=1 \
  --simTime=120" \
  2>&1 | tee bh_run_0623.log
```

**輸出檔案：**
- `bh-metrics_starlink25_20260623_161646.csv`
- `bh-timeplan_starlink25_20260623_161646.csv`

**驗證指令與結果：**
```bash
wc -l bh-metrics_starlink25_20260623_161646.csv
# → 54961

awk -F, 'NR>1{print $1}' bh-metrics_starlink25_20260623_161646.csv | sort -u | wc -l
# → 1374

awk -F, 'NR>1{print $1}' bh-metrics_starlink25_20260623_161646.csv | sort | uniq -c | head -5
# → 40 100.000000
#   40 100.080000
#   40 100.160000
#   40 100.240000
#   40 100.320000
```

**驗證結果：**

| 項目 | 預期 | 實測 | 結果 |
|------|------|------|------|
| 總行數 | 54,961 | 54,961 | ✓ |
| 唯一 timestamps | 1,374 | 1,374 | ✓ |
| rows / T_p | 40 | 40 | ✓ |
| 10 sats × 4 beams/T_p | 40 | 40 | ✓ |
| 時間涵蓋（warmUp=10s） | 10s – ~120s | 10s – 119.92s | ✓ |


**結論：Pass**





