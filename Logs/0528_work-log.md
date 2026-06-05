# 2026/05/28

### 1. Phase 2 Code Modularization

**動機：** `sat-multi-beam-simulation.cc` ，Phase 1 與 Phase 2 程式碼混雜。

**做法：**
- 新增 `sat-phase2-grid.h/.cc` — Grid mode （`CellStats`、`GridSimState`、`GridUpdateStep`、`RunGridMode`）
- 新增 `sat-phase2-dual.h/.cc` — Dual mode （`DualCellStats`、`DualSimState`、`DualUpdateStep`、`RunDualMode`），Phase 2.3 overlap detection 內建於 `DualUpdateStep`
- Phase 2 在 `userPos` 建立前即 return

**架構重點：**
```
sat-multi-beam-config.h          ← 所有模組共用
        ↓
sat-multi-beam-geometry + channel + orbit-reader + roi-grid
        ↓
sat-phase2-grid.cc / sat-phase2-dual.cc   ← 各自封裝 anonymous namespace
        ↓
sat-multi-beam-simulation.cc              ← 只做 CLI + dispatch
```

**CMakeLists.txt 需手動加入：**
```cmake
scratch/sat-phase2-grid.cc
scratch/sat-phase2-dual.cc
```

---

### 2. Unified Plotting Script

**動機：** 原本三個獨立繪圖腳本（`exp_phase2_4_snr_heatmap_cdf.py`、`exp_phase2_3_overlap_timeline.py`、`exp_phase2_0_grid_heatmap.py`）合併為一支。

**結果：** `exp_phase2_plots.py` 一鍵產生四張圖：

| Figure | 輸入 | 說明 |
|--------|------|------|
| A | `dual/cell_summary.csv` | sat[i] / sat[i+1] / Greedy 三欄 SNR 熱圖 |
| B | `dual/cell_summary.csv` | 三種策略 per-cell SNR CDF |
| C | `dual/cell_result.csv` + `overlap.json` | sat[i+1] ROI coverage buildup 曲線 |
| D | `grid/cell_summary.csv` | 單星 SNR 熱圖 + min/max range bar |

```bash
python exp_phase2_plots.py \
    --grid-summary result/grid/cell_summary.csv \
    --dual-summary result/dual/cell_summary.csv \
    --dual-result  result/dual/cell_result.csv \
    --overlap      result/dual/overlap.json \
    --out-dir      figures
```

---

### 3. Phase 2 SOP

涵蓋完整七步驟流程：
1. 複製 code/ 到 ns3 scratch
2. 更新 CMakeLists.txt（加入兩個新 .cc）
3. 編譯
4. 產生軌道 CSV（可跳過，data/ 已預存）
5. Grid mode 模擬 → `result/grid/`
6. Dual mode 模擬 → `result/dual/`
7. 產生 Figure A–D

