### 2026/06/25

### 實驗目的

分析不同衛星仰角（θ）、方位角（φ）與 beam 半角（T）組合下，
beam 橢圓投影到固定地理 ROI（5×5 格，Tokyo 為中心）的覆蓋品質。

**核心指標：**

```
Cov[n, m] = 交集面積(beam_n 橢圓, cell_m 格子) / cell_m 面積

n = beam 索引（beam n 瞄準 cell n 中心）
m = 被照射的 cell 索引

Cov[n, n] → 自我覆蓋 Sc（beam 打中目標的比例）
Cov[n, m] for n≠m → 溢波 Sp（打到其他 cell 的比例）
```

**橢圓投影幾何：**

```
天底（θ = 90°）：beam 投影為圓，半徑 r = h · tan(T)

低仰角（θ < 90°）：footprint 沿軌方向拉伸為橢圓
  a_along  = r / sin²(θ)   沿軌半軸（較大）
  b_cross  = r / sin(θ)    跨軌半軸
```

**Beam 類型：**

| 標籤 | 半角 T | r_beam @ 550 km |
|---|---|---|
| NARROW | 1.0° | 9.6 km |
| SLIGHT | 1.5° | 14.4 km |
| MIDDLE | 2.0° | 19.2 km |
| BROAD  | 2.5° | 24.0 km |
| WIDE   | 3.0° | 28.9 km |

---

### Phase 1 — ROI 尺寸研究

**目的：** 比較天底（θ=90°）與低仰角（θ=37°）下各 beam 類型的 footprint 範圍，決定 ROI 邊長。

**執行指令：**

```bash
python 2D/code/orbit-sgp4/analysis/exp_beam_coverage_projection.py ^
    --phase roi-study ^
    --altitude-km 550 ^
    --output-dir 2D/code/orbit-sgp4/data/processed/beam_coverage_projection
```

**輸出：**

```
data/processed/beam_coverage_projection/
└── phase1_roi/
    ├── nadir_vs_elev37_footprint.png   天底 vs 37° 投影比較圖
    ├── all_beamtypes_nadir.png         5 種 beam 天底疊圖
    └── roi_size_candidates.csv         各 beam 類型的 r_beam、ROI 尺寸建議
```

**roi_size_candidates.csv 欄位：**

| 欄位 | 說明 |
|---|---|
| `beam_type` | 半角（度） |
| `r_beam_km` | 天底 beam 半徑（km） |
| `nadir_cell_size_km` | 天底 cell 邊長 = 2 × r_beam |
| `nadir_roi_km` | ROI 邊長 = 5 × cell_size |
| `a_along_37deg` | θ=37° 時的沿軌半軸 |
| `b_cross_37deg` | θ=37° 時的跨軌半軸 |

**預期結果：** MIDDLE beam（2°）在 h=550 km 下：`r_beam ≈ 19.2 km`，ROI ≈ 192 km。

---

### Phase 2 — Beam vs Cell 大小匹配

**目的：** 給定固定 ROI，計算各 beam 類型的 `r_beam / cell_size` 比值，評估匹配程度。

**執行指令：**

```bash
python 2D/code/orbit-sgp4/analysis/exp_beam_coverage_projection.py ^
    --phase beam-cell-match ^
    --altitude-km 550 --roi-km 192 ^
    --output-dir 2D/code/orbit-sgp4/data/processed/beam_coverage_projection
```

**輸出：**

```
data/processed/beam_coverage_projection/
└── phase2_beam_cell_match/
    ├── nadir_overlay_5beamtypes.png   5 種 beam 疊在固定 ROI 格的天底圖
    └── beam_vs_cell_size.csv
```

**beam_vs_cell_size.csv 欄位：**

| 欄位 | 說明 |
|---|---|
| `r_beam_km` | beam 半徑（km） |
| `cell_km` | cell 邊長 = roi_km / 5 |
| `r_over_cell` | r_beam / cell_km（比值） |
| `cov_nadir` | 天底下 beam 覆蓋 cell 的面積比（π r² / A_cell，上限 1.0） |
| `assessment` | `too small` / `good match` / `spills over` |

**評估標準：**

| r_beam / cell | 評估 |
|---|---|
| < 0.35 | too small — beam 打不滿整個 cell |
| 0.35 ~ 0.65 | good match — beam 與 cell 大小相近 |
| > 0.65 | spills over — beam 超出 cell 邊界 |

---

### Phase 3 — 完整 25×25 Cov 矩陣掃描

**目的：** 對所有 (θ, φ, T) 組合計算 25×25 Cov 矩陣，產生 Sc/Sp 對仰角曲線與推薦 beam 類型。

**相依套件：** 需安裝 Shapely（`pip install shapely`）

**執行指令：**

```bash
python 2D/code/orbit-sgp4/analysis/exp_beam_coverage_projection.py ^
    --phase coverage ^
    --altitude-km 550 --roi-km 192 ^
    --elev 37 45 50 55 60 65 70 75 80 85 90 ^
    --azim 0 45 90 135 ^
    --beam-types 1.0 1.5 2.0 2.5 3.0 ^
    --sp-threshold 0.20 ^
    --sc-threshold 0.80 ^
    --output-dir 2D/code/orbit-sgp4/data/processed/beam_coverage_projection
```

**掃描範圍：** 11 仰角 × 4 方位角 × 5 beam 類型 = 220 種組合

**輸出：**

```
data/processed/beam_coverage_projection/
├── coverage_matrices/
│   └── cov_theta37_phi000_T2.0.csv    每個組合的 25×25 Cov 矩陣
├── figures/
│   ├── heatmap_theta37_phi000_T2.0.png   Cov[n,m] heatmap
│   ├── self_coverage_vs_elevation.png    Sc 對仰角曲線（5 種 beam）
│   ├── spillover_vs_elevation.png        Sp 對仰角曲線（5 種 beam）
│   └── recommendation_table.png          各（θ, T）組合的 Sc 推薦熱圖
└── summary_metrics.csv
```

**summary_metrics.csv 欄位：**

| 欄位 | 說明 |
|---|---|
| `theta` | 衛星仰角（°） |
| `phi` | 方位角（°） |
| `beam_type` | beam 半角（度） |
| `mean_sc` | 25 個 beam 的平均自我覆蓋 |
| `mean_sp` | 25 個 beam 的平均最大溢波 |
| `mean_iso` | 平均隔離度（sc / (sc + sp)） |
| `pct_cells_sc_ok` | Sc ≥ 門檻的 beam 比例（%） |
| `pct_cells_sp_ok` | Sp ≤ 門檻的 beam 比例（%） |

**推薦門檻：** `Sc ≥ 0.80` 且 `Sp ≤ 0.20`

**預期輸出（終端機）：**

```
=== Recommended Beam Type per Elevation (highest mean Sc) ===
 Elevation   Best T      Label    mean Sc
-----------------------------------------
       37°     3.0°       WIDE      0.xxx
       45°     2.5°      BROAD      0.xxx
       ...
       90°     1.5°     SLIGHT      0.xxx
```

---

### 參數對照表

| 參數 | 預設值 | 說明 |
|---|---|---|
| `--phase` | （必填） | `roi-study` / `beam-cell-match` / `coverage` |
| `--altitude-km` | `550.0` | 衛星高度（km），Starlink 預設 |
| `--roi-km` | `192.0` | ROI 邊長（Phase 2 & 3） |
| `--elev` | 37 45 50 55 60 65 70 75 80 85 90 | 要掃描的仰角（Phase 3） |
| `--azim` | `0 45 90 135` | 衛星方位角（Phase 3） |
| `--beam-types` | `1.0 1.5 2.0 2.5 3.0` | Beam 半角（度） |
| `--sc-threshold` | `0.80` | 自我覆蓋接受門檻 |
| `--sp-threshold` | `0.20` | 最大溢波接受門檻 |
| `--output-dir` | `data/processed/beam_coverage_projection` | 輸出根目錄 |


[ui](https://github.com/yuru1231/TriScale-LEO/blob/6bbe2c90008ddfb2922ed00d7d23f98306cf258f/2D/code/orbit-sgp4/analysis/beam_coverage_ui.py)