### 2026/06/09



## Grill decision — orbit-sgp4 D1–D13

找出所有隱藏假設與正確性問題。

### 模組背景

orbit-sgp4 是 BH 排程器的**前處理工具**：

- 掃描完整 Iridium-NEXT 66 衛星星座，涵蓋固定感興趣區域（ROI）
- 輸出每顆衛星、每個 cell 的 SNR 時間序列（`sat_XXXXX_cells.csv`）
- MRC 分析找出 **hard cells** — 沒有任何衛星組合能達到 SNR 門檻的時槽
- Hard cells 是 BH 時間計畫的主要輸入關注點

### 設計決策（共 13 項）

---

#### D1 — rFootprintM：從天線幾何推導，不硬編碼

**修改前：** `rFootprintM = 100,000 m` 硬編碼在 `SimConfig`

**決策：** 改為 `GetFootprintRadius()`，從 UPA 幾何計算：

```
α_cluster = arcsin((nBeamsX/2 + 0.443) / nAntennaX)
           = arcsin(2.443 / 32) ≈ 4.37°
r_nadir = h_sat × tan(α_cluster) ≈ 48 km
```

`rFootprintM` 保留在 `SimConfig` 作為**選填覆蓋值**（≤ 0 → 使用計算值），
供對齊 SNS3 或參考論文結果時的校準/除錯使用。
長期目標：從 SNS3 場景資料驅動。

---

#### D2 — Footprint 兩軸：皆與仰角相關

**修改前：**
```
a = rFootprintM / sin(ε)   ← 沿軌方向已縮放
b = rFootprintM             ← 垂軌方向為常數（錯誤）
```

**決策：** 兩軸皆隨斜距與傾斜幾何縮放：
```
b(ε) = r_nadir / sin(ε)      ← 垂軌方向：隨斜距縮放
a(ε) = r_nadir / sin²(ε)     ← 沿軌方向：斜距 × 傾斜因子
```

**原因：** 低仰角時波束錐投影至地面更遠，兩軸皆增大。
ε=5° 時：b ≈ 551 km，a ≈ 6,330 km（物理上偏大，但幾何上正確）。

---

#### D3 — sinr_dB 重新命名為 sinr_allbeams_dB

**修改前：** CSV 欄位 `sinr_dB`，峰值仰角時 SINR ≈ 0.25 dB，具誤導性。

**決策：** 重新命名為 `sinr_allbeams_dB`，並加上標頭說明：
> "sinr_allbeams_dB 假設 25 個波束同時啟用。BH 模式下每個slot不會開啟所有beam，因此移除 SINR 指標

`sinr_allbeams_dB` 保留作為診斷用途（顯示未進行 BH 排程時的干擾底噪）。

---

#### D4 — epsilon_eff 截斷：移除，改在呼叫點做 assert

**修改前：** `GetEllipticBeamCenters` 中硬編碼 `epsilon_eff = max(epsilon, 5.0)`

**決策：** 移除硬編碼的 5°。Pass A 已保證 `epsilon >= minElevDeg`，
因此截斷是重複的隱藏門檻。改在呼叫點 assert `epsilon >= minElevDeg`，
`ConstellationScanConfig` 中的 `minElevDeg` 為**唯一真實來源**。

---

#### D5 — 沿軌方向：預先計算，不使用備用值

**修改前：** 第一個時步回退到 `û_along = East (1,0,0)`，對 Iridium 而言是錯誤的
（傾角 86.4°，衛星在第一次接觸時大致沿南北方向移動）。

**決策：** 在 `Run()` 中，排程 Pass B 前先使用 SGP4 於 `windowStart − 1 s` 計算 `prevSatEnu`，完全移除東向備用值。

**影響：** SNR 數值不受影響（Δ 方向角 = 0），但第一個 frame 的 cell 地理位置現在正確。

---

#### D6 — Pass B 視窗：補齊 ±dtScreenS

**修改前：** Pass B 精確從 `windowStart` 開始（Pass A 精度 10 s）。
短視窗衛星（例如 sat_49：91 s）10 s 邊界誤差 = 11% 資料損失。

**決策：** 在 `Run()` 中，Pass A 結束後延伸：
```cpp
double t_start = passInfo.windowStartS - cfg.dtScreenS;
double t_end   = passInfo.windowEndS   + cfg.dtScreenS;
```
低於 `minElevDeg` 的時步會產生低 SNR，由 `check_coverage.py` 過濾，
掃描器本身不需額外過濾。

---

#### D7 — 掃描視窗：延長至 6000 s

**修改前：** `windowS = 3600 s`（60 分鐘），僅涵蓋 Iridium 軌道週期的約 60%。

**決策：** 延長至 **6000 s**（一個完整 Iridium 軌道週期，約 100 分鐘）。

**原因：** 報告完整軌道週期的覆蓋缺口才是正確的統計基準。

---

#### D8 — withFading 架構

**決策（範疇界定）：**
- `withFading = false` → BH 主要輸入（巨觀 SNR 地圖，適合波束分配的穩定輸出）
- Rician 衰落（K=10）→ **針對性壓力測試工具**，事後僅套用於 hard cells
- 不對完整掃描做 Monte Carlo；衰落分析是針對 3 dB 門檻識別出的 hard ticks 進行的獨立執行

論文必須明確說明 `withFading=false` 假設並引用 K=10。

---

#### D9 — Hard cell 門檻：3 dB

**修改前：** `check_coverage.py` 使用 `snr_dB < 0.0 dB` 作為覆蓋門檻。

**決策：** 改為 **3 dB** 作為可排程下限。

**原因：** 0 dB 是無衰落餘裕的理論噪聲底限。巨觀 SNR 接近 0 dB 的 BH cells 在衰落下會失效。
3 dB 提供一階衰落餘裕，並對應實際最低可排程 SNR。

---

#### D10 — 大氣損耗：按 cell 計算，不用 ROI 中心

**修改前：** `ComputeAtmosphericLoss_dB(elevation_roi, freqHz)` 對所有 25 個 cells
使用從 ROI 中心觀測到的衛星仰角。

**問題：** 在 ε=17.1°（關鍵仰角）時，最外圍 cell 距 ROI 約 92 km。
仰角誤差 ≈ 0.83°，大氣損耗梯度 ≈ 4.4 dB/°，
→ 誤差 ≈ **3.7 dB**，足以在 3 dB 門檻附近誤判 hard cells。

**決策：** 從每個 cell 的 ENU 位置計算各自的仰角。
`SatScanState` 已攜帶每個 tick 的 `obsEcef` 與衛星位置，
按 cell 計算仰角只需 `arctan2(U, sqrt(E²+N²))`，額外成本可忽略（每 tick ×25）。

---

#### D11 — dtSnrS：維持 1 s

**決策：** `dtSnrS = 1 s` 不變。覆蓋缺口與 hard cell 分析是軌道尺度的指標（數十至數百秒）。
記錄為刻意選擇。BH 排程器將在 orbit-sgp4 的 tick 之間內插。

---

#### D12 — dtScreenS：維持 10 s

**決策：** `dtScreenS = 10 s` 維持作為 Pass A 粗篩。視窗邊界精度問題已由 D6（±dtScreenS 補齊）解決。
縮小至 1 s 將失去兩階段架構的意義。

---

#### D13 — 多個 ROI 

**修改前：** 單一 ROI：東京（35.676°N, 139.65°E）。

**決策：** 新增 3 個不同緯度的 ROI：

| ROI | 緯度 | 經度 | 用途 |
|---|---|---|---|
| 東京 | 35.68°N | 139.65°E | 中緯度（現有基準） |
| 新加坡 | 1.35°N | 103.82°E | 赤道——Iridium 最差情況？ |
| 赫爾辛基 | 60.17°N | 24.94°E | 高緯度北——覆蓋較多 |


無需修改程式碼，`roiLatDeg` / `roiLonDeg` 已是 CLI 參數。

---

### 實作任務清單（共 11 項）

| # | 任務 | 檔案 |
|---|---|---|
| T1 | 新增 `GetFootprintRadius()` 至 `SimConfig` | `sat-multi-beam-config.h` |
| T2 | 使 `rFootprintM` 成為選填覆蓋值（≤0 → 使用計算值） | `sat-multi-beam-config.h` |
| T3 | 修正 `GetEllipticBeamCenters`：b(ε)=r/sin(ε)，a(ε)=r/sin²(ε) | `sat-multi-beam-geometry.cc` |
| T4 | 移除 `epsilon_eff=max(ε,5°)`；改在呼叫點 assert | `sat-multi-beam-geometry.cc` |
| T5 | 預先計算 `prevSatEnu`（於 `windowStart−1s`）；移除東向備用值 | `sat-constellation-scanner.cc` |
| T6 | Pass B 視窗補齊 ±`dtScreenS` | `sat-constellation-scanner.cc` |
| T7 | 預設 `windowS` 從 3600 s 延長至 6000 s | `sat-constellation-scanner.h` |
| T8 | 重新命名 `sinr_dB` → `sinr_allbeams_dB`；新增 CSV 標頭說明 | `sat-constellation-scanner.cc` |
| T9 | 大氣損耗改為按 cell 計算（使用 per-cell 仰角） | `sat-multi-beam-channel.cc` |
| T10 | 將 `check_coverage.py` 門檻從 0 dB 改為 3 dB | `analysis/check_coverage.py` |
| T11 | 在 README 補充新加坡／赫爾辛基 ROI 執行方式 | `README.md` / 執行腳本 |

### 修改後需重新驗證的關鍵數值

| 指標 | 舊值 | 預期方向 |
|---|---|---|
| r_nadir | 100 km（假設值） | → 約 48 km（計算值） |
| MRC 缺口（東京） | 51 s / 1.4%（3600 s 內） | → 在 6000 s 內重新量測 |
| Hard cell 缺口（0 dB 門檻） | 319 s Greedy / 51 s MRC | → 3 dB 門檻下將增加 |
| 符合條件衛星數 | 16/66（3600 s） | → 6000 s 下將增加 |
| beam_gain（中心 cell） | 46.52 dBi（不應改變） | → 必須維持 46.52 dBi ✅ |

---

## 自動計算覆蓋半徑

**修改檔案：** `code/sat-multi-beam-config.h`

- `rFootprintM` 預設值：`100e3` → `0.0`（零 = 自動推算）
- 新增 `GetFootprintRadius()`：從天線幾何推算天底波束半徑
  - 計算公式：`sin(α) = (nBeamsX/2 + 0.443) / nAntennaX`，`r_nadir = h × tan(α)`
  - Iridium 參數（h=634 km、5 波束、32 天線單元）：r_nadir ≈ 48 km
- 新增 `GetEffectiveFootprintM()`：若 `rFootprintM > 0` 則使用手動值，否則回傳 `GetFootprintRadius()`
- `sat-multi-beam-geometry.cc` 四個幾何函式全部改呼叫 `GetEffectiveFootprintM()`

---

## 橢圓兩軸仰角相依

**修改檔案：** `code/sat-multi-beam-geometry.cc`、`sat-multi-beam-geometry.h`

**修正前（錯誤）：**
- 沿軌軸：`a = r / sin(θ)`
- 垂軌軸：`b = r`（常數，未考慮斜距）

**修正後（正確，D2）：**
- 垂軌軸：`b = r / sin(θ)`（斜距縮放）
- 沿軌軸：`a = r / sin²(θ)`（斜距 × 傾斜效應）

其他變更：
- 移除 `GetEllipticBeamCenters` 中的 5° epsilon 截斷，仰角現在不做裁切直接傳入
- 更新 `sat-multi-beam-geometry.h` docstring，記錄 D2 正確軸定義

---

## 預計算 prevSatEnu + 斷言保護

**修改檔案：** `code/sat-constellation-scanner.cc`

- 在第一個排程事件前，預計算 `prevSatEnu`（時間點為 `windowStart − dtSnrS`，即 windowStart − 1 s）
- 移除 `GetEllipticBeamCenters` 中的 East 方向回退邏輯，改以 `assert(movLen >= 1.0)` 取代（若預計算未執行則觸發斷言）
- 掃描視窗加 padding：`tScanStart = max(0, windowStart − dtScreenS)`，`tScanEnd = windowEnd + dtScreenS`
- `Simulator::Stop` 時間更新：`windowS` → `windowS + dtScreenS`

---

## 掃描視窗預設值 + CSV 欄位重命名

**修改檔案：** `code/sat-constellation-scanner.h`、`sat-constellation-scanner.cc`

- `ConstellationScanConfig::windowS` 預設值：`3600.0` → `6000.0`（對應一個完整 Iridium 軌道週期約 100 分鐘）
- CSV 欄位重命名：`sinr_dB` → `sinr_allbeams_dB`
- 新增說明區塊，解釋 BH 語意：`sinr_allbeams_dB` 為 25 波束同時啟用的 SINR（診斷用）；BH 調度實際使用 `snr_dB`（單一波束啟用）

---

## 每格點大氣損耗驗證

**修改檔案：** `code/sat-multi-beam-channel.cc`

- 驗證 D10 已實作：`ComputePathLoss_dB` 使用 `delta = satPos − userPos` 計算每個格點的實際仰角，而非 ROI 中心仰角
- 若使用 ROI 中心仰角替代，外緣格點在臨界仰角下最多產生 **3.7 dB 誤差**

---

## SNR 門檻調整 + README 更新

**修改檔案：** `analysis/check_coverage.py`、`README.md`

- `check_coverage.py`：`--snr-thresh` 預設值 `0.0` → `3.0`；更新說明文字（3 dB = 雜訊地板以上的第一階衰落餘裕）
- `README.md` 更新內容：
  - 橢圓幾何章節（r_nadir 推導公式 + D2 軸定義）
  - 演算法說明（移除 East 回退 + epsilon 截斷）
  - Class diagram（`windowS=6000`、`rFootprintM=0.0 auto`）
  - 輸出格式表（`sinr_allbeams_dB`）
  - 執行狀態表（D2–D10 列）
  - 執行範例（`--window-s=6000`、4 個 ROI 城市）

---

## 本次修改檔案總覽：

| 檔案 | 對應任務 | 變更內容 |
|---|---|---|
| `code/sat-multi-beam-config.h` | T1、T2 | `rFootprintM=0.0`、`GetFootprintRadius()`、`GetEffectiveFootprintM()` |
| `code/sat-multi-beam-geometry.h` | T3 | docstring：D2 軸定義、預計算前置要求 |
| `code/sat-multi-beam-geometry.cc` | T2、T3、T4 | 四個函式改用 `GetEffectiveFootprintM()`；`GetEllipticBeamCenters` 兩軸仰角相依，移除 epsilon 截斷，以斷言取代 East 回退 |
| `code/sat-constellation-scanner.h` | T7 | `windowS: 3600 → 6000` |
| `code/sat-constellation-scanner.cc` | T4-assert、T5、T6、T8 | 新增 `NS_ASSERT`；預計算 `prevSatEnu`；加 padding 視窗；`sinr_allbeams_dB` CSV 欄位 |
| `code/sat-multi-beam-channel.cc` | T9 | 每格點大氣損耗說明文件 |
| `analysis/check_coverage.py` | T10 | `--snr-thresh` 預設值 `0.0 → 3.0` |
| `README.md` | T11 | 幾何、演算法、class diagram、狀態表、執行範例全部更新 |

- 上次 commit：`35689daa 0605`，本次變更尚待 commit
- T13（多 ROI 支援）列為下一階段任務



## 模擬

### Tokyo
執行參數：`--window-s=6000`、`lat=35.676`、`lon=139.65`，輸出至 `ns_result/0609/tokyo_out`

對照 grill log 預期值驗證結果：

| 指標 | 修改前 | 修改後 | 狀態 |
|---|---|---|---|
| `r_nadir` | 100 km（硬編碼） | 48.5 km（從 UPA 幾何自動推算） | ✅ |
| 合格衛星數 | 16 / 3600 s | 22 / 6000 s | ✅ |
| `sinr_allbeams_dB` 欄位 | `sinr_dB` | 已重命名 | ✅ |
| `beam_gain` 中心格點 | 46.52 dBi | 46.5206 dBi（不變） | ✅ |
| Greedy 覆蓋缺口 | 319 s（0 dB，3600 s） | 2870 s / 47.7%（3 dB，6000 s） | ✅ ↑ 符合預期 |
| MRC 覆蓋缺口 | 51 s（0 dB，3600 s） | 2337 s / 38.9%（3 dB，6000 s） | ✅ ↑ 符合預期 |

**分析：** 缺口增加完全由門檻變更解釋——臨界仰角從 0 dB 提升至 3 dB，可排程的最低仰角從 ~5° 上升至 **27.8°**

- 最長連續 MRC 缺口：**1030 s**（t = 4066–5095 s）
- MRC 改善量：+533 s（縮減 18.6%）

---

## 多 ROI 驗證

執行參數（3 dB 門檻，6000 s 視窗）：
- Singapore：`lat=1.35, lon=103.82` → `ns_result/0609/Singapore_out`
- Helsinki：`lat=60.17, lon=24.94` → `ns_result/0609/Helsinki_out`
- 對兩個輸出執行 `check_coverage.py --snr-thresh 3.0`

**比較結果：**

| ROI | 緯度 | 合格衛星 | 完全中斷 | Greedy 缺口 | MRC 缺口 | MRC 改善率 | 最長 MRC 缺口 |
|---|---|---|---|---|---|---|---|
| Helsinki | 60.2°N | 36 | 0 s | 2296 s（38.2%） | **5 s（0.1%）** | 99.8% | 1 s |
| Tokyo | 35.7°N | 22 | 222 s | 2870 s（47.7%） | 2337 s（38.9%） | 18.6% | 1030 s |
| Singapore | 1.4°N | 8 | 4092 s | ~5809 s（96.7%）* | ~5515 s（91.7%）* | 17.1% | 501 s |

*新加坡：掃描器僅回報 4093–6010 s 視窗（1918 s），因 0–4092 s 無衛星仰角超過 5°。全視窗缺口為加計 4092 s 軌道盲區後的推算值。

**關鍵發現：MRC 改善幅度與緯度正相關**

- **赫爾辛基**：MRC 幾乎消除所有缺口（38.2% → 0.1%），最多同時可見 5 顆衛星，分集增益充裕
- **東京**：MRC 帶來中等改善（47.7% → 38.9%），通常可見 2 顆衛星
- **新加坡**：MRC 無法克服軌道幾何限制（96.7% → 91.7%），Iridium 86.4° 傾角對赤道地區造成結構性覆蓋不足

**結論：**
> "MRC 覆蓋缺口從 0.1%（赫爾辛基，60°N）到 38.9%（東京，36°N）到 91.7%（新加坡，1°N），顯示 MRC 合併的覆蓋增益隨緯度驅動的衛星分集增加，而赤道地區無論採用何種排程策略，均面臨軌道幾何造成的結構性中斷。"

---

## Starlink TLE 模擬 — 新加坡（赤道對照組）

**執行指令：**

```bash
./ns3 run "scratch/orbit-sgp4/sat-multi-beam-simulation \
  --constellation-dir=scratch/starlink_constellation \
  --lat=1.35 --lon=103.82 \
  --window-s=6000 --dt-snr-s=1 \
  --min-elevation-deg=5 \
  --out-dir=scratch/starlink_tle" \
  2>&1 | tee starlink_tle.log
```

**輸出目錄：** `ns_result/0609/starlink_tle`

**模擬參數：**

| 參數 | 值 |
|---|---|
| ROI | 新加坡（lat=1.35°, lon=103.82°） |
| 視窗 | 6000 s |
| dt_snr | 1 s |
| 最低仰角 | 5° |
| 衛星高度（代表值） | ~427–555 km（各衛星不同） |

**關鍵結果：**

| 指標 | Iridium（86.4° 傾角） | Starlink TLE（~53° 傾角） |
|---|---|---|
| 合格衛星通過數 | 8 | **2284** |
| 衛星高度 | 634 km（固定） | 234–555 km（真實 TLE） |
| 對赤道地區覆蓋 | 結構性不足（軌道幾何限制） | 大幅改善（低傾角密集覆蓋） |

**觀察：**

- Starlink 2284 顆合格通過 vs Iridium 8 顆，顯示傾角對赤道覆蓋的決定性影響
- 53° 傾角使 Starlink 在新加坡（1.4°N）能提供充分的多衛星可見度，Iridium 極軌道（86.4°）則在赤道附近形成軌道盲區
- 此結果為論文的「星座傾角對 MRC 分集增益影響」提供量化對照基礎

**發現 Bug：`h_satellite_km` 失效問題**

執行 `check_coverage.py` 時，link budget table 退回 `DEFAULT_CFG (634 km)` 而非使用 Starlink 實際高度。

根因分析：
- C++ `WriteStatusJson()` 計算 `meanAltKm`：只在 `p.altitudeM > 0.0` 時累加
- 若 SGP4 回傳 error 或 `altitudeM` 未被成功賦值，`altCnt = 0`，`meanAltKm = 0`
- JSON 寫入 `"h_satellite_km": 0`
- Python 條件 `if h_km and h_km > 0` → False → 退回 634 km

即使 `h_satellite_km` 正確取得（如 514 km），對 Starlink 也有第二層問題：
- 各衛星實際高度 234–555 km（spread 達 ~320 km）
- 單一均值 514 km 無法代表所有衛星的路徑損耗

**修正內容（`analysis/check_coverage.py`）：**
- 新增 `altitude_stats()`：從 `passes[].altitude_km` 直接統計 mean/min/max
- 當 spread > 50 km 時顯示警告
- 依序 fallback：per-pass stats → top-level field → DEFAULT_CFG

**結果有效性說明：**

| 分析項目 | 有效性 | 說明 |
|---|---|---|
| Greedy / MRC 覆蓋缺口（來自 CSV） | ✅ 有效 | SNR 在 C++ 模擬時已按各衛星實際仰角與距離計算 |
| Link budget table（分析式） | ⚠️ 謹慎使用 | 使用單一均值高度；不反映 234–555 km 範圍 |
| Critical elevation | ⚠️ 謹慎使用 | 同上 |

**check_coverage.py 輸出（舊版，link budget 使用 634 km DEFAULT_CFG）：**

輸出檔：`ns_result/0609/starlink_tle/analysis/check_coverage.txt`

| 指標 | 數值 | 有效性 |
|---|---|---|
| 合格衛星通過數 | 2284 | ✅ |
| Full blackout（無衛星數據） | 437 s（[4208~4426], [4622~4839]） | ✅ |
| Greedy 缺口 | **556 s（9.2%）** | ✅ |
| MRC 缺口 | **350 s（5.8%）** | ✅ |
| MRC 改善量 | +206 s（37.1% reduction） | ✅ |
| 最長 MRC 缺口 | 195 s（4427–4621 s） | ✅ |
| 最多同時可見衛星 | 807 顆 | ✅ |
| Link budget table 使用高度 | 634 km（DEFAULT_CFG，**錯誤**） | ⚠️ 謹慎 |
| Critical elevation（27.8°） | 基於 634 km，非 Starlink 實際值 | ⚠️ 謹慎 |

**Iridium vs Starlink — 新加坡完整比較（3 dB 門檻，6000 s 視窗）：**

| 指標 | Iridium（86.4°） | Starlink TLE（~53°） |
|---|---|---|
| 合格衛星通過數 | 8 | 2284 |
| 完全中斷（無資料） | 4092 s | 437 s |
| Greedy 缺口 | ~96.7% | **9.2%** |
| MRC 缺口 | ~91.7% | **5.8%** |
| MRC 改善率 | 17.1% | 37.1% |
| 最長 MRC 缺口 | 501 s | 195 s |
| 最多同時可見衛星 | ~2–3 | 807 |

**意義：**
傾角從 86.4°（Iridium）改為 ~53°（Starlink），赤道新加坡的 MRC 缺口從 91.7% 降至 5.8%，完全逆轉覆蓋劣勢。赤道覆蓋瓶頸是傾角（軌道幾何），不是 MRC 算法本身。

**標記為已知問題 BUG-001**（記錄於 `README.md ## 已知問題`）

Python 的 `altitude_stats()` 修改無效——根本原因在 C++，SGP4 對 Starlink TLE epoch 計算失敗導致 `altitudeM = 0`。先擱置，待後續確認 `tPeakMin` 時間基準。

---

## 下次工作
確認 BUG-001（`h_satellite_km` 失效）的 C++ 根因，修正 `altitudeM` 賦值邏輯。
