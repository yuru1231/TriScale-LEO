# 每日日誌 — 2026/05/18

## 正式記號重構

### 目的

將程式內部變數對齊 `2D/readme.md` 所定義的正式 2D BH 模型記號：

| 舊變數 | 新變數 | 正式意義 |
|---|---|---|
| `m_numBeams` | `m_J` | J：每顆衛星的 beam 總數 |
| `m_maxActiveBeams` | `m_Kb` | Kb：每個 slot 最多同時啟動的 beam 數 |
| `ComputeM()` | `ComputeF()` | F：每幀的時槽數（BHTP 週期） |
| slot 數量 `M` | `F` | F = round(T_p / T_s) |

**NOTE：** `Kb`（同時啟動 beam 數）≠ `K`（2D 幾何模型中的衛星沿軌間距）。
改名為 `Kb` 是為了避免後續加入排程數學式時與 `K` 混淆。

### 修改檔案

**`Beam Hopping Controller/Codes/sat-bh-scheduler.h`：**
- `m_numBeams` → `m_J`（attribute 說明更新：`J: total beams per satellite`）
- `m_maxActiveBeams` → `m_Kb`（說明補充 `Kb ≠ K`）
- 建構子初始化列表同步修改

**`Beam Hopping Controller/Codes/sat-bh-scheduler.cc`：**
- 所有 `m_numBeams`、`m_maxActiveBeams` 引用替換為 `m_J`、`m_Kb`
- `m_slotAlloc`、`m_demandHistory`、`m_prevDemand`、`m_lambda`、`m_virtualTraffic`、`m_consecutiveChanges` 的 resize 呼叫全部從 `m_numBeams` 改為 `m_J`
- `beamId` 範圍檢查上界更新為 `m_J`
- 排程對應第 i 顆衛星、第 j 個 beam、第 f 個 slot，為格子 a_{L,W} 內的第 u 個使用者提供服務

**`Beam Hopping Controller/Codes/sat-bh-time-plan.h`：**
- `ComputeM()` → `ComputeF()`，說明更新：計算 F = round(T_p / T_s) — 每幀時槽數

**`Beam Hopping Controller/Codes/sat-bh-time-plan.cc`：**
- 所有 `ComputeM()` 呼叫更新為 `ComputeF()`

---

## 2D Footprint 模組

### 目的

將衛星服務範圍轉換成平面
驗證每個服務格子 a_{L,W} 可唯一對應到地理座標，
且 C_i[n]（每幀可服務格子集合）可從衛星軌道預先計算。

### 新增檔案

**`Beam Hopping Controller/Codes/sat-bh-2d-footprint.cc`：**

獨立 SNS3 場景，不使用任何 BH 排程器元件。

關鍵資料結構：
```cpp
struct FootprintConfig {
    double   latCenter_deg  = 35.67619190;  // 東京
    double   lonCenter_deg  = 139.65031060;
    double   rFootprint_m   = 300000.0;     // R = h/tan(θ_min)
    uint32_t Nx             = 4;            // 沿軌欄數
    uint32_t Ny             = 3;            // 跨軌列數
    uint32_t satId          = 0;            // 0–65（Iridium-66）
    double   simTimeSec     = 60.0;
};

struct BeamCell {
    uint32_t beamId;       // 1-indexed = row × Nx + col + 1
    uint32_t row, col;
    double cx_m, cy_m;     // 直角座標（東向/北向，公尺）
    double lat_deg, lon_deg;
};
```

執行流程：
```
ParseConfig → ComputeGrid → PrintMatrix → ExportGeometryCsv
  → （simTime > 0 時）SNS3 場景 → AddCbrTraffic → RunSimulation
  → ExportResultsCsv
```

幾何推導：
- Footprint 半徑 R → 最大內接正方形邊長 S = R × √2
- 格子邊長 d = S / Nx（沿軌）= S / Ny（跨軌，正方形時相等）
- 格子中心：cx = -S/2 + (L - 0.5) × d，cy = -S/2 + (W - 0.5) × d
- 經緯度轉換與 Python `utils.get_positions_in_lat_long_coordinates()` 公式一致

輸出檔案：
| 檔案 | 內容 | 寫入時機 |
|------|------|---------|
| `footprint-geometry.csv` | beam_id, row, col, lat_deg, lon_deg, cx_m, cy_m | SNS3 初始化之前 |
| `footprint-results.csv` | 模擬後 UT 位置對照 | 模擬結束後 |
| `sat-stats-per-ut-fwd-composite-sinr-scatter-*.dat` | 每顆 UT 的 SINR | 模擬結束後 |

執行指令：
```bash
# 僅驗證幾何（不執行模擬）
./ns3 run "sat-bh-2d-footprint --Nx=4 --Ny=3 --rFootprint=300000 --simTime=0"

# 完整模擬（60 秒）
./ns3 run "sat-bh-2d-footprint --Nx=4 --Ny=3 --rFootprint=300000 --simTime=60" \
  2>&1 | tee footprint_run.log
```

**`2D/readme.md`：**

正式 2D 模型完整說明文件：
- 系統參數表（J、Kb、F、h、θ_min、R、Ld、d、A、K、x_i[n]、C_i[n]）
- 衛星通過 5 階段模型（進入 → 接近 → 中心正上方 → 遠離 → 離開）
- 幾何推導步驟 1–5
- BHTP 驗證框架與未來演算法接入點

---
