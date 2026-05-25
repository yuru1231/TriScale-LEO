# Phase 2 SOP — 從 Clone 到產圖

## 目錄

- [資料夾結構](#資料夾結構)
- [執行環境需求](#執行環境需求)
- [Step 1 — 複製程式碼到 ns3 scratch](#step-1--複製程式碼到-ns3-scratch)
- [Step 2 — 更新 CMakeLists.txt](#step-2--更新-cmakeliststxt)
- [Step 3 — 編譯](#step-3--編譯)
- [Step 4 — 產生軌道 CSV（可跳過）](#step-4--產生軌道-csv可跳過)
- [Step 5 — 執行 Grid mode 模擬](#step-5--執行-grid-mode-模擬)
- [Step 6 — 執行 Dual mode 模擬](#step-6--執行-dual-mode-模擬)
- [Step 7 — 產生所有圖表](#step-7--產生所有圖表)
- [驗收條件總表](#驗收條件總表)
- [常見問題](#常見問題)

---

## 資料夾結構

```
2D/phase2/
├── code/                            ← 所有 C++ 原始碼與 Python 腳本
│   ├── sat-multi-beam-simulation.cc ← 主程式（Phase 1 + Phase 2 dispatch）
│   ├── sat-multi-beam-channel.h/.cc
│   ├── sat-multi-beam-geometry.h/.cc
│   ├── sat-multi-beam-config.h
│   ├── sat-orbit-reader.h/.cc
│   ├── sat-roi-grid.h/.cc
│   ├── sat-phase2-grid.h/.cc        ← Phase 2.0 grid mode 模組
│   ├── sat-phase2-dual.h/.cc        ← Phase 2.2/2.3 dual mode 模組
│   ├── run_sgp4.py                  ← 軌道 CSV 產生器
│   └── exp_phase2_plots.py          ← 統一繪圖腳本（Figure A–D）
│
├── data/                            ← 軌道 CSV（已預先產生）
│   ├── orbit_sat_i.csv              ← sat[i]:   iridium-75 45，峰值仰角 66.4°
│   └── orbit_sat_i1.csv             ← sat[i+1]: iridium-75 44，峰值仰角 85.2°
│
├── result/
│   ├── grid/                        ← Grid mode 輸出（Step 5 產生）
│   │   ├── cell_result.csv
│   │   └── cell_summary.csv
│   └── dual/                        ← Dual mode 輸出（Step 6 產生）
│       ├── cell_result.csv
│       ├── cell_summary.csv
│       └── overlap.json
│
└── figures/                         ← 圖表輸出（Step 7 產生）
    ├── fig_A_snr_heatmap.png/.svg   ← sat[i] / sat[i+1] / Greedy SNR 熱圖
    ├── fig_B_snr_cdf.png/.svg       ← 三種策略 per-cell SNR CDF
    ├── fig_C_overlap_timeline.png/.svg ← sat[i+1] ROI coverage buildup 曲線
    └── fig_D_grid_snr_heatmap.png/.svg ← 單星 SNR 熱圖 + min/max range
```

---

## 執行環境需求

| 環境 | 用途 |
|------|------|
| VMware — Ubuntu 22.04 + ns-3.43 | Step 1–6（C++ 編譯與模擬） |
| Windows 或 VMware — Python 3.9+ | Step 7（繪圖，可在任一環境執行） |

**Python 依賴（Step 4 和 Step 7 需要）：**

```bash
pip install ephem numpy pandas matplotlib
python -c "import ephem, numpy, pandas, matplotlib; print('OK')"
```

---

## Step 1 — 複製程式碼到 ns3 scratch

在 VMware 的 ns3 根目錄下執行：

```bash
# 複製所有 C++ 檔案（共 12 個）
cp /path/to/phase2/code/sat-multi-beam-simulation.cc  scratch/
cp /path/to/phase2/code/sat-multi-beam-channel.h      scratch/
cp /path/to/phase2/code/sat-multi-beam-channel.cc     scratch/
cp /path/to/phase2/code/sat-multi-beam-geometry.h     scratch/
cp /path/to/phase2/code/sat-multi-beam-geometry.cc    scratch/
cp /path/to/phase2/code/sat-multi-beam-config.h       scratch/
cp /path/to/phase2/code/sat-orbit-reader.h            scratch/
cp /path/to/phase2/code/sat-orbit-reader.cc           scratch/
cp /path/to/phase2/code/sat-roi-grid.h                scratch/
cp /path/to/phase2/code/sat-roi-grid.cc               scratch/
cp /path/to/phase2/code/sat-phase2-grid.h             scratch/
cp /path/to/phase2/code/sat-phase2-grid.cc            scratch/
cp /path/to/phase2/code/sat-phase2-dual.h             scratch/
cp /path/to/phase2/code/sat-phase2-dual.cc            scratch/

# 複製 Python 腳本
cp /path/to/phase2/code/run_sgp4.py          scratch/
cp /path/to/phase2/code/exp_phase2_plots.py  scratch/

# 複製軌道 CSV（如果 data/ 已存在）
cp /path/to/phase2/data/orbit_sat_i.csv   scratch/
cp /path/to/phase2/data/orbit_sat_i1.csv  scratch/
```

---

## Step 2 — 更新 CMakeLists.txt

找到 `scratch/CMakeLists.txt`（或 ns3 根目錄下管理 scratch 的 CMakeLists）中的 `build_exec` 區塊，加入三個新的 `.cc` 檔案：

```cmake
build_exec(
  NAME sat-multi-beam-simulation
  SOURCE_FILES
    scratch/sat-multi-beam-simulation.cc
    scratch/sat-multi-beam-geometry.cc
    scratch/sat-multi-beam-channel.cc
    scratch/sat-orbit-reader.cc
    scratch/sat-roi-grid.cc
    scratch/sat-phase2-grid.cc      # ← 新增
    scratch/sat-phase2-dual.cc      # ← 新增
  LIBRARIES_TO_LINK
    ${libcore}
)
```

> **注意：** `sat-phase2-grid.cc` 和 `sat-phase2-dual.cc` 是本次模組化新增的檔案，舊版 CMakeLists.txt 不含這兩行，必須手動加入。

---

## Step 3 — 編譯

```bash
./ns3 build sat-multi-beam-simulation 2>&1 | tee scratch/build.log
```

**預期輸出：**
```
Build completed successfully
```

若出現 `undefined reference` 錯誤，確認 Step 2 的兩行是否已加入 CMakeLists.txt。

**確認 `--mode` 參數存在（快速健康檢查）：**

```bash
./ns3 run "sat-multi-beam-simulation --help" 2>&1 | grep -E "mode|d=|orbit"
```

---

## Step 4 — 產生軌道 CSV（可跳過）

**若 `data/orbit_sat_i.csv` 和 `data/orbit_sat_i1.csv` 已複製到 `scratch/`，直接跳至 Step 5。**

如需重新產生：

```bash
python scratch/run_sgp4.py \
    --mode sequence \
    --tle-file scratch/iridium.txt \
    --observer-lat 35.676 \
    --observer-lon 139.650 \
    --start-utc "2000/01/01 00:00:00" \
    --search-window-s 7200 \
    --scan-step-s 10 \
    --min-peak-elev-deg 50 \
    --pair-index 0 \
    --margin-s 60 \
    --step-ms 100 \
    --output-sat-i  scratch/orbit_sat_i.csv \
    --output-sat-i1 scratch/orbit_sat_i1.csv \
    2>&1 | tee scratch/run_sgp4.log
```

**驗證：**

```bash
grep -v "^#" scratch/orbit_sat_i.csv  | wc -l   # 預期 10301
grep -v "^#" scratch/orbit_sat_i1.csv | wc -l   # 預期 10301
head -5 scratch/orbit_sat_i.csv                  # 確認衛星名為 iridium-75 45
head -5 scratch/orbit_sat_i1.csv                 # 確認衛星名為 iridium-75 44
```

---

## Step 5 — 執行 Grid mode 模擬

Grid mode 模擬單顆衛星（sat[i]）對 5×5 ROI 格點的覆蓋。

```bash
./ns3 run "sat-multi-beam-simulation \
  --mode=grid \
  --d=5 \
  --orbit-csv=scratch/orbit_sat_i.csv \
  --update-ms=100 \
  --min-elevation-deg=5 \
  --lat=35.676 \
  --lon=139.650 \
  --out-dir=scratch/result/grid" \
  2>&1 | tee scratch/result/grid_sim.log
```

**預期 console 輸出：**

```
[grid mode] 5×5 grid = 25 cells  in-footprint = 25
  L = W = 141.421 km  cell = 28.284 km × 28.284 km
  wrote scratch/result/grid/cell_results.csv
  wrote scratch/result/grid/cell_summary.csv
Done.  Output in: scratch/result/grid
```

**確認輸出：**

```bash
ls scratch/result/grid/
# cell_result.csv  cell_summary.csv

wc -l scratch/result/grid/cell_result.csv    # 約 93,076 行
wc -l scratch/result/grid/cell_summary.csv   # 26 行（含 header）
```

---

## Step 6 — 執行 Dual mode 模擬

Dual mode 同時模擬兩顆衛星，內建 Phase 2.3 重疊閾值偵測。

```bash
./ns3 run "sat-multi-beam-simulation \
  --mode=dual \
  --d=5 \
  --orbit-csv-i=scratch/orbit_sat_i.csv \
  --orbit-csv-i1=scratch/orbit_sat_i1.csv \
  --update-ms=100 \
  --min-elevation-deg=5 \
  --snr-thresh-db=0 \
  --lat=35.676 \
  --lon=139.650 \
  --out-dir=scratch/result/dual" \
  2>&1 | tee scratch/result/dual_sim.log
```

**預期 console 輸出：**

```
[dual mode] 5×5 grid = 25 cells  in-footprint = 25
  sat[i]   coverage_s = 372.x s
  sat[i+1] coverage_s = 651.x s
  Greedy mean SNR across 25 cells = 3.1x dB
  wrote scratch/result/dual/cell_results.csv
  wrote scratch/result/dual/cell_summary.csv
  wrote scratch/result/dual/overlap.json
Done.  Output in: scratch/result/dual
```

**確認輸出：**

```bash
ls scratch/result/dual/
# cell_result.csv  cell_summary.csv  overlap.json

cat scratch/result/dual/overlap.json
```

**預期 overlap.json：**
```json
{
  "n_in_footprint": 25,
  "snr_threshold_dB": 0.000,
  "thresholds_pct": [10, 25, 50, 75, 90],
  "sat_i1_coverage_times_s": {
    "10pct": 318.7,
    "25pct": 322.3,
    "50pct": 327.7,
    "75pct": 334.5,
    "90pct": 336.4
  }
}
```

五個時刻均須落在 [190, 440 s] 的雙星重疊窗口內。

---

## Step 7 — 產生所有圖表

**可在 VMware 或 Windows 上執行（路徑對應修改）。**

```bash
python scratch/exp_phase2_plots.py \
    --grid-summary scratch/result/grid/cell_summary.csv \
    --dual-summary scratch/result/dual/cell_summary.csv \
    --dual-result  scratch/result/dual/cell_result.csv \
    --overlap      scratch/result/dual/overlap.json \
    --out-dir      scratch/figures \
    2>&1 | tee scratch/figures_gen.log
```

**預期 console 輸出：**

```
Phase 2 — Unified Figure Generator
figures  : A, B, C, D
  [A] fig_A_snr_heatmap.png + .svg
  [B] fig_B_snr_cdf.png + .svg
  [C] fig_C_overlap_timeline.png + .svg
  [D] fig_D_grid_snr_heatmap.png + .svg
Done.
```

**確認 8 個輸出檔案存在：**

```bash
ls scratch/figures/
# fig_A_snr_heatmap.png        fig_A_snr_heatmap.svg
# fig_B_snr_cdf.png            fig_B_snr_cdf.svg
# fig_C_overlap_timeline.png   fig_C_overlap_timeline.svg
# fig_D_grid_snr_heatmap.png   fig_D_grid_snr_heatmap.svg
```

**只產生部分圖表：**

```bash
# 只畫 A 和 B
python scratch/exp_phase2_plots.py \
    --dual-summary scratch/result/dual/cell_summary.csv \
    --overlap      scratch/result/dual/overlap.json \
    --out-dir      scratch/figures \
    --figures A B
```

| 圖表 | 檔案 | 內容 |
|------|------|------|
| Figure A | `fig_A_snr_heatmap` | sat[i] / sat[i+1] / Greedy 三欄 5×5 SNR 熱圖 |
| Figure B | `fig_B_snr_cdf` | 三種策略 per-cell mean SNR CDF |
| Figure C | `fig_C_overlap_timeline` | sat[i+1] ROI coverage buildup 曲線（Phase 2.3） |
| Figure D | `fig_D_grid_snr_heatmap` | Grid mode 單星 SNR 熱圖 + min/max range bar |

---

## 驗收條件總表

執行以下腳本自動驗證所有條件：

```bash
python3 - <<'EOF'
import csv, json

BASE = "scratch/result"

# --- Dual summary ---
dual = list(csv.DictReader(open(f"{BASE}/dual/cell_summary.csv")))
fp   = [r for r in dual if r["in_footprint"] == "1"]
n_fp = len(fp)
mean_i  = sum(float(r["mean_snr_i_dB"])      for r in fp) / n_fp
mean_i1 = sum(float(r["mean_snr_i1_dB"])     for r in fp) / n_fp
mean_gr = sum(float(r["greedy_mean_snr_dB"]) for r in fp) / n_fp
center  = next(float(r["greedy_mean_snr_dB"])
               for r in fp if r["row"]=="2" and r["col"]=="2")

# --- Overlap ---
ov    = json.load(open(f"{BASE}/dual/overlap.json"))
times = ov["sat_i1_coverage_times_s"]

# --- Figures ---
import os
FIG = "scratch/figures"
figs = [os.path.isfile(f"{FIG}/{f}") for f in [
    "fig_A_snr_heatmap.png", "fig_B_snr_cdf.png",
    "fig_C_overlap_timeline.png", "fig_D_grid_snr_heatmap.png"]]

results = [
    ("EC-1", "有效格點 == 25",                     n_fp == 25),
    ("EC-2", "Greedy > sat[i]",                   mean_gr > mean_i),
    ("EC-3", "Greedy > sat[i+1]",                 mean_gr > mean_i1),
    ("EC-4", "全部 5 個重疊時刻在 [190,440s]",
             all(190 <= float(v) <= 440 for v in times.values())),
    ("EC-5", "Figure A PNG 存在",                 figs[0]),
    ("EC-6", "Figure B PNG 存在",                 figs[1]),
    ("EC-7", "Figure C PNG 存在",                 figs[2]),
    ("EC-8", "Figure D PNG 存在",                 figs[3]),
    ("EC-9", "中心格 Greedy SNR 在 [3.5,5.0] dB", 3.5 <= center <= 5.0),
    ("EC-10","全格 Greedy 均值 SNR 在 [3.0,3.3] dB", 3.0 <= mean_gr <= 3.3),
]

print(f"\n{'條件':<6}  {'說明':<36}  {'結果'}")
print("-" * 58)
for code, desc, passed in results:
    print(f"{code:<6}  {desc:<36}  {'PASS' if passed else 'FAIL'}")

all_pass = all(p for _, _, p in results)
print(f"\n整體：{'全部 PASS ✓' if all_pass else '有條件 FAIL，請檢查上方輸出'}")
print(f"\n  sat[i] 均值 SNR  = {mean_i:.3f} dB")
print(f"  sat[i+1] 均值 SNR = {mean_i1:.3f} dB")
print(f"  Greedy 均值 SNR   = {mean_gr:.3f} dB")
print(f"  中心格 Greedy SNR = {center:.3f} dB")
EOF
```

| 條件 | 說明 | 通過標準 |
|------|------|---------|
| EC-1 | 有效格點數 | == 25 |
| EC-2 | Greedy SNR 均值 > sat[i] 均值 | Greedy > sat[i] |
| EC-3 | Greedy SNR 均值 > sat[i+1] 均值 | Greedy > sat[i+1] |
| EC-4 | 5 個重疊閾值均在重疊窗口內 | 全部 ∈ [190, 440 s] |
| EC-5 | Figure A 存在 | `fig_A_snr_heatmap.png` |
| EC-6 | Figure B 存在 | `fig_B_snr_cdf.png` |
| EC-7 | Figure C 存在 | `fig_C_overlap_timeline.png` |
| EC-8 | Figure D 存在 | `fig_D_grid_snr_heatmap.png` |
| EC-9 | 中心格 Greedy SNR 合理 | ∈ [3.5, 5.0] dB |
| EC-10 | 全格 Greedy 均值 SNR 合理 | ∈ [3.0, 3.3] dB |

---

## 常見問題

### CMakeLists 更新後仍找不到 RunGridMode / RunDualMode

確認 `sat-phase2-grid.cc` 和 `sat-phase2-dual.cc` 都有加入 `SOURCE_FILES`，且已重新執行 Step 3 編譯。

---

### orbit CSV 行數不是 10301

`--step-ms 100` 對應 1030 s 窗口應為 10301 行。若行數不符，確認 `--margin-s` 設定覆蓋了完整的雙星過境。可用 `--search-window-s 14400` 擴大搜尋範圍。

---

### Figure C 曲線全為 0%

`dual/cell_result.csv` 的 `snr_i1_dB` 欄位全為 `-999`，表示 sat[i+1] 在模擬窗口內完全不可見。確認兩個 orbit CSV 的時間軸重疊（雙星需共用相同的 t=0 起點）。

---

### 繪圖時 `No module named 'matplotlib'`

```bash
pip install matplotlib numpy pandas
```

若在 Windows 上執行 `exp_phase2_plots.py`，路徑改為 Windows 格式：

```bash
python 2D\phase2\code\exp_phase2_plots.py \
    --grid-summary 2D\phase2\result\grid\cell_summary.csv \
    --dual-summary 2D\phase2\result\dual\cell_summary.csv \
    --dual-result  2D\phase2\result\dual\cell_result.csv \
    --overlap      2D\phase2\result\dual\overlap.json \
    --out-dir      2D\phase2\figures
```
