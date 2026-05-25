# Phase 2 程式碼驗證 SOP

**對象**：學姊（驗證 Phase 2 雙衛星 SNR 模擬程式碼）  
**範圍**：Phase 2.1 → 2.2 → 2.3 → 2.4 → 2.5 完整 pipeline  
**日期**：2026-05-25

---

## 目錄

- [1. 功能概述](#1-功能概述)
- [2. 執行環境需求](#2-執行環境需求)
- [3. 檔案位置](#3-檔案位置)
- [4. 驗證步驟](#4-驗證步驟)
  - [Step 1 — 環境確認](#step-1--環境確認)
  - [Step 2 — 確認 Phase 2.1 軌道 CSV](#step-2--確認-phase-21-軌道-csv)
  - [Step 3 — Phase 2.2 雙衛星 ns3 模擬](#step-3--phase-22-雙衛星-ns3-模擬)
  - [Step 4 — Phase 2.3 重疊閾值偵測驗證](#step-4--phase-23-重疊閾值偵測驗證)
  - [Step 5 — Phase 2.4 圖形輸出驗證](#step-5--phase-24-圖形輸出驗證)
  - [Step 6 — Phase 2.5 z 軸旋轉修正驗證](#step-6--phase-25-z-軸旋轉修正驗證)
- [5. 通過條件總表](#5-通過條件總表)
- [6. 常見問題排除](#6-常見問題排除)

---

## 1. 功能概述

Phase 2 模擬兩顆 Iridium 衛星依序服務同一固定地面 ROI（東京）的換手場景，  
計算每個觀測格點的 SNR，並以 Greedy 策略（每格取較高 SNR 的衛星）產生基準結果。

```
Stage 1: Phase 2.1 — run_sgp4.py --mode sequence
           → orbit_sat_i.csv    (sat[i]:   iridium-75 45, 仰角峰值 66.4° at t=50s)
           → orbit_sat_i1.csv   (sat[i+1]: iridium-75 44, 仰角峰值 85.2° at t=580s)
           → sequence_summary.json

Stage 2: Phase 2.2 + 2.3 — ns3 mode=dual
           → dual_cell_results.csv   (每時間步 × 每格，兩衛星 SNR)
           → dual_cell_summary.csv   (每格均值 SNR、Greedy SNR、覆蓋秒數)
           → dual_overlap.json       (sat[i+1] 覆蓋 10/25/50/75/90% ROI 的首次時刻)

Stage 3: Phase 2.4 — exp_phase2_4_snr_heatmap_cdf.py
           → fig_A_snr_heatmap.png / .svg  (三種策略的 5×5 SNR 熱圖)
           → fig_B_snr_cdf.png / .svg      (每格均值 SNR 的 CDF)

Stage 4: Phase 2.5 — z 軸旋轉修正（已嵌入 C++ BuildArrayTransform）
           → 同 Stage 2 輸出，低仰角格點 beam gain 已修正
```

**關鍵模擬參數：**

| 參數 | 數值 |
|---|---|
| 觀測點（ROI 中心） | 東京：lat=35.676°N，lon=139.650°E |
| ROI 格點 | d=5 → 5×5 = 25 格，r_footprint=100 km |
| 格點間距 | ~28.3 km |
| 模擬時間窗口 | [0, 1030 s]，步長 100 ms → 每顆衛星 10,301 行 |
| 雙衛星同時可見區間 | t ∈ [190, 440 s]（250 s 重疊窗口） |

---

## 2. 執行環境需求

所有步驟均在 **VMware（Ubuntu 22.04 + ns-3.43）** 上執行。  
標記 `[Python]` 的步驟也可在 Windows 主機上執行。

### 2.1 Python 依賴

```bash
# 在 VMware 上安裝
pip install ephem numpy pandas matplotlib

# 確認安裝成功
python -c "import ephem, numpy, pandas, matplotlib; print('OK')"
```

### 2.2 ns3 編譯確認

模擬執行檔必須已完成編譯，執行以下指令確認：

```bash
# 在 ns3 根目錄下
./ns3 build sat-multi-beam-simulation
# 預期輸出：Build completed successfully
```

若編譯失敗，請複製缺少的檔案（見 [Section 3](#3-檔案位置)），並在 `CMakeLists.txt` 的 `build_exec` 區塊加入：

```cmake
build_exec(
    EXECNAME sat-multi-beam-simulation
    SOURCE_FILES
        sat-multi-beam-simulation.cc
        sat-multi-beam-geometry.cc
        sat-multi-beam-channel.cc
        sat-orbit-reader.cc
        sat-roi-grid.cc
    LIBRARIES_TO_LINK
        ${libcore}
        ${libsatellite}
)
```

---

## 3. 檔案位置

### 來源檔案（複製至 VMware scratch 目錄）

| 檔案（Windows 路徑） | 複製目的地（VMware） |
|---|---|
| `2D/code/phase1-ns3/sat-multi-beam-simulation.cc` | `scratch/` |
| `2D/code/phase1-ns3/sat-multi-beam-geometry.h/.cc` | `scratch/` |
| `2D/code/phase1-ns3/sat-multi-beam-channel.h/.cc` | `scratch/` |
| `2D/code/phase1-ns3/sat-multi-beam-config.h` | `scratch/` |
| `2D/code/phase1-ns3/sat-orbit-reader.h/.cc` | `scratch/` |
| `2D/code/phase1-ns3/sat-roi-grid.h/.cc` | `scratch/` |
| `2D/code/scripts/run_sgp4.py` | 任意可存取路徑 |
| `2D/code/scripts/exp_phase2_4_snr_heatmap_cdf.py` | 任意可存取路徑 |
| `2D/data/tle/iridium.txt` | 任意可存取路徑 |

### 已驗證結果（Windows，供比對參考）

| 檔案 | 說明 |
|---|---|
| `2D/results/dual_d5_v2/dual_cell_summary.csv` | Phase 2.5 最終每格統計 |
| `2D/results/dual_d5_v2/dual_overlap.json` | Phase 2.3 重疊時刻 |
| `2D/results/dual_d5_v2/figures/fig_A_snr_heatmap.png` | Phase 2.4 熱圖 |
| `2D/results/dual_d5_v2/figures/fig_B_snr_cdf.png` | Phase 2.4 CDF |

---

## 4. 驗證步驟

### Step 1 — 環境確認

確認 Python 與 ns3 均已就緒：

```bash
# 確認 Python 依賴
python -c "import ephem; print('ephem OK')"

# 確認 ns3（應顯示 --mode、--d、--orbit-csv 等參數說明）
./ns3 run "sat-multi-beam-simulation --help"
```

**預期結果：** 無 ImportError 或 segfault；ns3 印出 help 訊息。

---

### Step 2 — 確認 Phase 2.1 軌道 CSV

Phase 2.1 負責產生衛星對的軌道 CSV。  
若 VMware 上已存在 `scratch/orbit_sat_i.csv` 與 `scratch/orbit_sat_i1.csv`，直接跳至 [Step 3](#step-3--phase-22-雙衛星-ns3-模擬)。

**如需重新產生：**

```bash
python run_sgp4.py \
    --mode sequence \
    --tle-file iridium.txt \
    --observer-lat 35.676 \
    --observer-lon 139.650 \
    --start-utc "2000/01/01 00:00:00" \
    --search-window-s 7200 \
    --scan-step-s 10 \
    --min-peak-elev-deg 50 \
    --pair-index 0 \
    --margin-s 60 \
    --step-ms 100 \
    --output-sat-i scratch/orbit_sat_i.csv \
    --output-sat-i1 scratch/orbit_sat_i1.csv \
    2>&1 | tee scratch/run_sgp4_sequence.log
```

> **注意：** `--pair-index 0` 選擇第一個合格過境作為 sat[i]。  
> 腳本會印出所有合格過境的表格，請確認：  
> - sat[i] = iridium-75 45，仰角峰值 ≈ 66.4°  
> - sat[i+1] = iridium-75 44，仰角峰值 ≈ 85.2°  
>
> 若這兩顆衛星不在 index 0/1，請用 `--pair-index` 與 `--pair-i1-index` 手動指定。

**確認軌道 CSV 正確性：**

```bash
# 確認行數（預期各約 10301 行，不含 # 開頭的 metadata 行）
grep -v "^#" scratch/orbit_sat_i.csv | wc -l
grep -v "^#" scratch/orbit_sat_i1.csv | wc -l

# 確認最後一筆的時間
grep -v "^#" scratch/orbit_sat_i.csv | tail -1
```

**預期結果：**
- 行數：`10301`（1030 s ÷ 0.1 s + 1）
- 最後一行 `time_s` ≈ `1030.0`
- `sat_alt_m` 欄位：值在 770,000–800,000 m 之間（Iridium NEXT 軌道高度）

---

### Step 3 — Phase 2.2 雙衛星 ns3 模擬

執行雙衛星模擬（同時包含 Phase 2.3 重疊偵測）：

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
  --out-dir=scratch/dual_d5_verify" \
  2>&1 | tee scratch/dual_d5_verify.log
```

**預期 console 輸出（關鍵行）：**

```
  d=5  in-footprint cells=25
  sat[i]   coverage_s = 372.x s
  sat[i+1] coverage_s = 651.x s
  Greedy mean SNR across 25 cells = 3.1x dB
Done. Output in: scratch/dual_d5_verify
```

**確認輸出檔案存在：**

```bash
ls scratch/dual_d5_verify/
# 預期：dual_cell_results.csv  dual_cell_summary.csv  dual_overlap.json
```

**快速正確性檢查（EC-1/2/3）：**

```bash
python3 - <<'EOF'
import csv, sys

summary = list(csv.DictReader(open('scratch/dual_d5_verify/dual_cell_summary.csv')))
fp = [r for r in summary if r['in_footprint'] == '1']

mean_i  = sum(float(r['mean_snr_i_dB'])         for r in fp) / len(fp)
mean_i1 = sum(float(r['mean_snr_i1_dB'])        for r in fp) / len(fp)
mean_gr = sum(float(r['greedy_mean_snr_dB'])     for r in fp) / len(fp)

print(f"有效格點數      : {len(fp)} / {len(summary)}")
print(f"sat[i]   均值 SNR : {mean_i:.3f} dB   （參考 ~2.58 dB）")
print(f"sat[i+1] 均值 SNR : {mean_i1:.3f} dB   （參考 ~2.42 dB）")
print(f"Greedy   均值 SNR : {mean_gr:.3f} dB   （參考 ~3.13 dB）")
print()
print(f"EC-1（25 個有效格點）        : {'PASS' if len(fp)==25 else 'FAIL'}")
print(f"EC-2（Greedy SNR > sat[i]）  : {'PASS' if mean_gr > mean_i else 'FAIL'}")
print(f"EC-3（Greedy SNR > sat[i+1]）: {'PASS' if mean_gr > mean_i1 else 'FAIL'}")
EOF
```

---

### Step 4 — Phase 2.3 重疊閾值偵測驗證

`dual_overlap.json` 在 Step 3 結束時自動寫出。  
驗證 5 個重疊閾值均落在預期窗口 [190, 440 s] 內：

```bash
python3 - <<'EOF'
import json

data  = json.load(open('scratch/dual_d5_verify/dual_overlap.json'))
times = data['sat_i1_coverage_times_s']

print("Phase 2.3 重疊閾值驗證")
print(f"  有效格點數       : {data['n_in_footprint']}  （預期 25）")
print(f"  SNR 判定門檻     : {data['snr_threshold_dB']} dB  （預期 0.0）")
print()

# 參考值 ± 容差（Phase 2.5 v2）
reference = {
    '10pct': (310, 330, 318.7),
    '25pct': (315, 335, 322.3),
    '50pct': (320, 340, 327.7),
    '75pct': (328, 345, 334.5),
    '90pct': (330, 348, 336.4),
}

all_pass = True
for key, (lo, hi, ref) in reference.items():
    t = times.get(key, -1)
    in_window = (190 <= t <= 440)
    near_ref  = (lo <= t <= hi)
    status    = 'PASS' if in_window else 'FAIL'
    print(f"  {key}: t={t:.1f}s  落在 [190,440s]={in_window}  接近參考值 {ref}s [{lo},{hi}s]={near_ref}  -> {status}")
    if not in_window:
        all_pass = False

print()
print(f"整體結果: {'PASS' if all_pass else 'FAIL'}")
EOF
```

**Phase 2.3 參考值（Phase 2.5 v2 最終結果）：**

| 閾值 | 說明 | 參考時刻 (s) | 可接受範圍 |
|---|---|---|---|
| 10% | 3 格被 sat[i+1] 覆蓋 | 318.7 | 310–330 |
| 25% | 7 格被 sat[i+1] 覆蓋 | 322.3 | 315–335 |
| 50% | 13 格被 sat[i+1] 覆蓋 | 327.7 | 320–340 |
| 75% | 19 格被 sat[i+1] 覆蓋 | 334.5 | 328–345 |
| 90% | 23 格被 sat[i+1] 覆蓋 | 336.4 | 330–348 |

5 個時刻均須在重疊窗口 [190, 440 s] 內。

---

### Step 5 — Phase 2.4 圖形輸出驗證

從 summary 與 overlap 檔案產生 Figure A（SNR 熱圖）與 Figure B（CDF）：

```bash
python exp_phase2_4_snr_heatmap_cdf.py \
    --summary scratch/dual_d5_verify/dual_cell_summary.csv \
    --overlap scratch/dual_d5_verify/dual_overlap.json \
    --out-dir scratch/dual_d5_verify/figures \
    --dpi 300 \
    2>&1 | tee scratch/phase2_4_figures.log
```

**預期 console 輸出：**

```
Policy mean SNR across 25 cells:
  sat[i]   : 2.5xx dB
  sat[i+1] : 2.4xx dB
  Greedy   : 3.1xx dB

  Figure A saved → scratch/dual_d5_verify/figures/fig_A_snr_heatmap.png + .svg
  Figure B saved → scratch/dual_d5_verify/figures/fig_B_snr_cdf.png + .svg
```

**確認輸出檔案存在（EC-5/6）：**

```bash
ls scratch/dual_d5_verify/figures/
# 預期：fig_A_snr_heatmap.png  fig_A_snr_heatmap.svg
#        fig_B_snr_cdf.png      fig_B_snr_cdf.svg
```

**Figure A（熱圖）目視驗收條件：**

| 條件 | 說明 |
|---|---|
| 三個子圖 | sat[i]、sat[i+1]、Greedy，共用同一色軸 |
| Greedy 整體較亮 | Greedy 格點整體應比兩顆單衛星的格點更偏綠（高 SNR） |
| 數值標注可見 | 每格中央印有 SNR 數字 |
| 無空白格 | 25 個有效格點均應有顏色，不得出現白色空格 |

**Figure B（CDF）目視驗收條件：**

| 條件 | 說明 |
|---|---|
| 三條 CDF 曲線 | 每種策略一條，顏色不同 |
| Greedy 曲線在最右側 | Greedy CDF 應明顯右移，代表 SNR 分布較高 |
| 垂直虛線 | 每條曲線對應一條均值虛線 |
| Y 軸範圍 0.0–1.0 | CDF 從 0 開始，最終到達 1.0 |

---

### Step 6 — Phase 2.5 z 軸旋轉修正驗證

Phase 2.5 修正嵌入於 `sat-multi-beam-channel.cc` 的 `BuildArrayTransform`，  
驗證重點：中心格 Δ = 0（對稱性），角落格有小幅負修正。

```bash
python3 - <<'EOF'
import csv

new = {(int(r['row']), int(r['col'])): float(r['greedy_mean_snr_dB'])
       for r in csv.DictReader(open('scratch/dual_d5_verify/dual_cell_summary.csv'))
       if r['in_footprint'] == '1'}

center = new.get((2, 2), None)
ref_center = 3.625   # Phase 2.5 v2 參考值

print(f"中心格 (row=2, col=2) greedy_mean_snr_dB = {center:.3f} dB")
print(f"  Phase 2.5 v2 參考值                     = {ref_center} dB")
print(f"  差異                                     = {abs(center - ref_center):.4f} dB")
print(f"  EC-7（|差異| < 0.05 dB）                : {'PASS' if abs(center - ref_center) < 0.05 else 'FAIL'}")
print()

all_vals = list(new.values())
mean_all = sum(all_vals) / len(all_vals)
print(f"所有格點 Greedy 均值 SNR = {mean_all:.3f} dB  （參考 ~3.13 dB）")
print(f"EC-8（全格均值在 [3.00, 3.30] dB）        : {'PASS' if 3.00 <= mean_all <= 3.30 else 'FAIL'}")
EOF
```

---

## 5. 通過條件總表

| 編號 | Phase | 驗證項目 | 參考數值 | 通過條件 |
|---|---|---|---|---|
| EC-1 | 2.2 | 有效格點數 | 25 / 25 格 | == 25 |
| EC-2 | 2.2 | Greedy SNR 均值 > sat[i] 均值 | Greedy ≈ 3.13 dB，sat[i] ≈ 2.58 dB | Greedy > sat[i] |
| EC-3 | 2.2 | Greedy SNR 均值 > sat[i+1] 均值 | Greedy ≈ 3.13 dB，sat[i+1] ≈ 2.42 dB | Greedy > sat[i+1] |
| EC-4 | 2.3 | 5 個重疊閾值均在 [190, 440 s] 內 | 10pct=318.7s，…，90pct=336.4s | 全部 5 個落在窗口內 |
| EC-5 | 2.4 | Figure A 檔案已產生（PNG + SVG） | 2 個檔案 | 檔案存在 |
| EC-6 | 2.4 | Figure B 檔案已產生（PNG + SVG） | 2 個檔案 | 檔案存在 |
| EC-7 | 2.5 | 中心格 (2,2) Greedy SNR ≈ 參考值 | ≈ 3.625 dB | \|差異\| < 0.05 dB |
| EC-8 | 2.5 | 全格 Greedy SNR 均值在合理範圍 | ≈ 3.13 dB | 在 [3.00, 3.30] dB 內 |

**8 個條件全部通過，Phase 2 程式碼驗證完成。**

---

## 6. 常見問題排除

### 問題一：run_sgp4.py 顯示「No qualifying passes found」

**原因：** `--min-peak-elev-deg` 門檻過高，或 `--search-window-s` 搜尋窗口太短。

**解法：**
```bash
python run_sgp4.py \
    --mode sequence \
    --tle-file iridium.txt \
    --search-window-s 14400 \     # 延長至 4 小時
    --min-peak-elev-deg 40 \      # 降低門檻
    ...
```

---

### 問題二：ns3 segfault 或「找不到 orbit CSV」

**原因：** CSV 路徑是相對路徑，相對位置與 `./ns3 run` 執行目錄不符。

**解法：** 使用絕對路徑：
```bash
./ns3 run "sat-multi-beam-simulation \
  --mode=dual \
  --orbit-csv-i=$(pwd)/scratch/orbit_sat_i.csv \
  --orbit-csv-i1=$(pwd)/scratch/orbit_sat_i1.csv \
  ..."
```

---

### 問題三：Python 找不到 ephem 或 pandas

**解法：**
```bash
pip install ephem numpy pandas matplotlib
# 若使用 virtualenv：
source .venv/bin/activate && pip install ephem numpy pandas matplotlib
```

---

### 問題四：重疊時刻不在 [190, 440 s] 內

**原因：** 選到錯誤的衛星對，或 TLE epoch 不同。

**診斷：**
```bash
# 確認 CSV header 中衛星名稱
head -20 scratch/orbit_sat_i.csv      # 查看 "# satellite" 欄位
head -20 scratch/orbit_sat_i1.csv

# 印出 overlap 結果
python3 -c "import json; print(json.dumps(json.load(open('scratch/dual_d5_verify/dual_overlap.json')), indent=2))"
```

**預期衛星名稱：**
- `orbit_sat_i.csv`：`# satellite : iridium-75 45`
- `orbit_sat_i1.csv`：`# satellite : iridium-75 44`

若衛星不符，重新執行 Phase 2.1，調整 `--pair-index` / `--pair-i1-index` 參數。

---

### 問題五：Figure A 出現白色空格（NaN 格點）

**原因：** `in_footprint` 欄位全為 0，或 `mean_snr_i_dB` 欄位缺失。

**診斷：**
```bash
# 確認 summary CSV 的欄位與有效行
head -3 scratch/dual_d5_verify/dual_cell_summary.csv
grep ",1," scratch/dual_d5_verify/dual_cell_summary.csv | head -3
```

**預期：** `in_footprint` 欄位存在，且有 25 行數值為 `1`。

---

*本 SOP 涵蓋 Phase 2.1–2.5。Phase 1.x 的驗證流程請參閱 [README.md](README.md) 的「驗證結果」章節。*
