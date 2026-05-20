# 2D 對地投影場景

## 概述

將單顆 LEO 衛星的對地覆蓋範圍（footprint）投影到 2D 地面平面，並將其劃分為帶有編號的矩形矩陣，每個格子對應一個 beam，UT 放置於格子中心座標。

**目的：** 建立 System Architecture，驗證對地投影範圍可以被明確編號（beam ID ↔ 地理座標）。

---

## 參數

| 參數 | 數值 | 來源 |
|---|---|---|
| 中心座標 | 35.676°N, 139.650°E（東京） | `params.py` |
| 衛星高度 h | 600 km（LEO） | `params.py` |
| 中心頻率 | 30 GHz（Ka-band） | `params.py` |
| 頻寬 | 25 MHz | `params.py` |
| 地球半徑 | R_earth = 6371 km | `params.py` |
| 座標轉換 | 球形地球投影 | `utils.get_positions_in_lat_long_coordinates()` |

---

## 系統參數

### 幾何參數

| 符號 | 定義 | 說明 |
|------|------|------|
| h | Satellite altitude | 衛星高度（LEO: 600 km） |
| θ_min | Minimum servable elevation angle | 最小可服務仰角（地面 UT 視角） |
| R | Coverage radius = h / tan(θ_min) = Ld / 2 | 最大水平覆蓋距離 |
| Ld | Service area horizontal span = 2R = 2h / tan(θ_min) | 服務區域沿軌（along-track）跨度 |
| W | Service area cross-track span = d × Ny | 服務區域跨軌（cross-track）跨度 |
| d | Grid cell side length | 格子邊長（= coverage area of one beam） |
| A | Region of Interest (ROI) = Ld × W | 服務區域，即衛星最多可覆蓋的格子集合 |
| K | Along-track distance between two satellites | 兩顆衛星沿軌方向的地面間距 |

### 排程參數

| 符號 | 定義 | 說明 |
|------|------|------|
| F | Number of slots per frame | 每幀的時槽數 |
| n | Frame number | 幀編號 |
| i | Satellite index (1 ≤ i ≤ I) | 衛星索引 |
| j | Beam index (1 ≤ j ≤ J) | Beam 索引，每顆衛星有 j 個 beam |
| f | Slot index (1 ≤ f ≤ F) | 時槽索引 |
| u | User index | 使用者索引 |
| x_i[n] | Relative position of satellite i in frame n | 衛星 i 在第 n 幀的相對位置（位於 ROI 水平等分線上） |
| C_i[n] | Set of grids that satellite i can serve in frame n | 衛星 i 在第 n 幀可服務的格子集合，C_i[n] ∈ A |
| a_{L,W} | Service grid at position (L, W) within the ROI | ROI 中位置 (L, W) 的服務格子 |
| B_{i,j} | The j-th beam of satellite i | 衛星 i 的第 j 個 beam |

### 核心定義

> **Beam Hopping** 的目標是找出 a_{L,W} 與衛星 i 第 j 個 beam B_{i,j} 之間的對應關係，
> 格子來自 C_i[n]（1 ≤ i ≤ I，1 ≤ j ≤ J）。

> **Scheduling** 是使用 **第 i 顆衛星**、**第 j 個 beam**、**第 f 個時槽**，
> 為 **第 a_{L,W} 格**內的 **第 u 個使用者** 提供服務的過程。

---

## 場景設計
![serving-area](fig\serving-area.jpg)

### 幾何推導

#### 第一步：footprint 圓形 → 最大內接矩形

θ_min（最小可服務仰角）決定衛星在地面的覆蓋半徑 R：

```
R = h / tan(θ_min)
```

衛星正下方時，地面 footprint 為**圓形**，半徑 R。
服務區域 A 取該圓形內的**最大內接矩形**。

對半徑為 R 的圓，最大內接矩形為**正方形**，四個角恰好落在圓周上：

```
側邊長 S = R × sqrt(2)
Ld = S    （沿軌跨度）
W  = S    （跨軌跨度，= Ld）
A  = Ld × W = 2R²
```

> 若 footprint 為**橢圓**（衛星非正下方），則最大內接矩形的 Ld ≠ W，
> 需依橢圓半長軸 a、半短軸 b 另行推導（此處以圓形為預設）。

服務區域確定後，格子邊長 d 由格子數 L_max 決定：

```
d     =  grid length
W_max = W / d = S / d = L_max 
```

> θ_min 同時定義 C_i[n] 的邊界：從地面格子往衛星仰角 ≥ θ_min，
> 該格子才進入可服務集合 C_i[n]。

#### 第二步：服務區域內最大矩形格子矩陣

將 A 均分為 L_max × W_max 個格子，格子編號為 a_{L,W}：

```
L : 沿軌方向欄索引，L = 1, 2, ..., L_max    （L_max = Ld / d）
W : 跨軌方向列索引，W = 1, 2, ..., W_max    （W_max = W_total / d）
d : 格子邊長（一個 beam 的覆蓋面積邊長）

a_{L,W} : 位於服務區域內 (L, W) 位置的格子
```

格子矩陣示意（L_max=4，W_max=3）：

```
W=3 │ a_{1,3}  a_{2,3}  a_{3,3}  a_{4,3}   ← 最北列
W=2 │ a_{1,2}  a_{2,2}  a_{3,2}  a_{4,2}
W=1 │ a_{1,1}  a_{2,1}  a_{3,1}  a_{4,1}   ← 最南列
     欄:   L=1      L=2      L=3      L=4
```

#### 第三步：候選服務格子集合

```
C_i[n] = { a_{L,W} | 衛星 i 在第 n 幀對 a_{L,W} 的仰角 ≥ θ_min }
C_i[n] ⊆ A
```

衛星移動時 x_i[n] 改變，使得 C_i[n] 隨幀數 n 滑動（沿軌方向位移）。

#### 第四步：各格中心的直角座標（x = 東/沿軌，y = 北/跨軌，單位：公尺）

```
cx = -Ld/2 + (L - 0.5) × d      （L = 1..L_max）
cy = -W_total/2 + (W - 0.5) × d （W = 1..W_max）
```

#### 第五步：直角座標 → 經緯度（與 Python `utils.get_positions_in_lat_long_coordinates` 公式一致）

```
lat_c_rad = lat_c_deg × π / 180
Δlat = cy / R_earth                          （弧度）
Δlon = cx / (R_earth × cos(lat_c_rad))       （弧度）
lat_deg = lat_c_deg + Δlat × (180/π)
lon_deg = lon_c_deg + Δlon × (180/π)
```

其中 `R_earth = 6371000` m。

---



## BHTP 驗證框架

### 當前目標：幾何驗證（非演算法實作）

`sat-bh-2d-footprint.cc` **只做幾何與 link budget 驗證**，不含任何 BHTP 演算法。

驗證目的：
- 確認 a_{L,W} 格子編號可以明確對應至地理座標（beam ID ↔ lat/lon）
- 確認 C_i[n] 的計算邊界（哪些格子在衛星可服務仰角範圍內）
- 建立 BHTP 演算法未來所需的幾何基礎

---

### 衛星通過過程（Satellite Pass）

BHTP 的研究單位為**一次衛星通過（satellite pass）**，而非單幀（frame）。

一次通過包含以下五個階段：

```
階段 1：進入服務區域（Entering）
  衛星從 A 的邊緣進入，C_i[n] 由零個格子開始增加
  → 只有靠近衛星進入方向的幾個格子進入 C_i[n]

階段 2：接近中心（Approaching Center）
  衛星移近服務區域中心，C_i[n] 快速擴大
  → 越來越多格子的仰角 ≥ θ_min

階段 3：位於中心正上方（At Nadir of A）
  衛星正下方，所有 a_{L,W} ∈ A 的格子仰角最大
  → C_i[n] = A（全部格子均可服務，最大覆蓋）

階段 4：遠離中心（Moving Away）
  C_i[n] 開始縮減，格子依序離開可服務集合

階段 5：離開服務區域（Leaving）
  衛星離開 A 的邊緣，C_i[n] 歸零
```

C_i[n] 隨幀數 n 的演變（示意，L_max=4, W_max=3）：

```
幀 n₁（進入）:   C_i[n₁] = { a_{1,1}, a_{1,2} }    ← 僅西側格子
幀 n₂（接近）:   C_i[n₂] = { a_{1,*}, a_{2,*}, a_{3,*} }
幀 n₃（中心）:   C_i[n₃] = A = 全部 12 格
幀 n₄（遠離）:   C_i[n₄] = { a_{2,*}, a_{3,*}, a_{4,*} }
幀 n₅（離開）:   C_i[n₅] = { a_{4,1}, a_{4,2} }    ← 僅東側格子
```

> 衛星軌道已知，因此 C_i[n] 對每顆衛星、每幀均可**預先計算**。

---

### 幾何驗證可證明的事項

| 驗證項目 | 對應 BHTP 意義 |
|---------|--------------|
| beam_id ↔ lat/lon 一一對應 | 演算法可用格子索引 a_{L,W} 唯一操作特定地理格 |
| C_i[n] 隨 n 連續變化 | 演算法可依幀推算可服務格子，無需即時感測 |
| 所有格子 ≤ Nx×Ny ≤ 72 | SNS3 Beam ID 有效，每格可對應一個 SNS3 beam |
| 格子中心 SINR 可量測 | 提供 link budget 依據，作為排程效益的評估基準 |

---

### 未來 BHTP 演算法接入點

幾何驗證通過後，BHTP 演算法將在以下層次加入：

```
input:  C_i[n]（每幀可服務格子集合）
        每格的流量需求 D_{L,W}[n]
        每顆衛星的 beam 數量 J
        每幀的時槽數 F

output: B_{i,j}[f][n]（第 i 顆衛星，第 j 個 beam，
                        在第 n 幀第 f 個時槽服務哪個格子）
```

**當前版本不實作此演算法**，僅保留 a_{L,W} 格子矩陣作為其輸入結構的基礎。

---

## SNS3 場景檔案

**實作路徑：** `Beam Hopping Controller/Codes/sat-bh-2d-footprint.cc`

### 設定結構

```cpp
struct FootprintConfig {
    double   latCenter_deg  = 35.67619190;   // 東京（中心緯度）
    double   lonCenter_deg  = 139.65031060;  // 東京（中心經度）
    double   rFootprint_m   = 300000.0;      // R = h/tan(θ_min)，覆蓋半徑（公尺）
    uint32_t Nx             = 4;             // 欄數（Ld 方向）
    uint32_t Ny             = 3;             // 列數（W 方向）
    uint32_t satId          = 0;             // 衛星索引（0–65）
    double   simTimeSec     = 60.0;
    double   warmUpSec      = 5.0;
    std::string geoCsvFile    = "footprint-geometry.csv";
    std::string resultCsvFile = "footprint-results.csv";
};
```

> `rFootprint_m` = R = h / tan(θ_min)
> 服務區域 Ld = 2 × rFootprint_m，A = Ld × Ld（正方形），d = Ld / Nx

### 格子資料結構

```cpp
struct BeamCell {
    uint32_t beamId;    // 1-indexed：row × Nx + col + 1（C_i[n] 元素）
    uint32_t row;       // 0-indexed（row 0 = 最南列）
    uint32_t col;       // 0-indexed（col 0 = 最西欄）
    double   cx_m;      // 直角座標 x，東向（公尺）
    double   cy_m;      // 直角座標 y，北向（公尺）
    double   lat_deg;
    double   lon_deg;
};
```

### 執行流程

```
main()
 │
 ├─ ParseConfig(argc, argv)
 │   解析 rFootprint(=R), Nx, Ny, satId, simTime 等參數
 │
 ├─ ComputeGrid(cfg) → vector<BeamCell>
 │   Ld = 2R, W = 2R（預設正方形）
 │   計算所有格子的 beam_id（C_i[n]）、座標、經緯度
 │
 ├─ PrintMatrix(cells)
 │   在終端機印出矩陣（SNS3 初始化之前）
 │   beam_id  row  col  lat_deg  lon_deg
 │   ← 此時可驗證 C_i[n] 編號是否正確
 │
 ├─ ExportGeometryCsv(cells, "footprint-geometry.csv")
 │   ← 幾何驗證完畢，可在此終止程式
 │
 ├─ [SNS3 全域設定]
 │   Ka-band BW=25MHz、regeneration mode、
 │   HandoversEnabled=false、OutputOverwrite=true
 │
 ├─ simHelper->LoadScenario("constellation-iridium-66-sats")
 │
 ├─ 建立 BeamUserInfoMap_t：
 │   for each cell in C_i[n]:
 │     beamInfos[{satId, cell.beamId}] = SatBeamUserInfo(1, 1)
 │
 ├─ simHelper->SetBeamUserInfo(beamInfos)
 ├─ simHelper->SetBeamSet(beamSet)       ← 必須呼叫，否則不會建立 UT
 ├─ simHelper->CreateSatScenario(SatHelper::NONE)
 │
 ├─ AddCbrTraffic(FWD_LINK, UDP, 間隔 20ms, 1500B)   ← 最小流量觸發 link
 ├─ AddPerUtFwdCompositeSinr（stats）                 ← 記錄每顆 UT 的 SINR
 │
 ├─ simHelper->RunSimulation()
 │
 └─ ExportResultsCsv(cells, "footprint-results.csv")
```

### Ka-band 設定

SNS3 預設已是 Ka-band（FWD 19.7 GHz / RTN 29.5 GHz），只需覆寫頻寬：

```cpp
Config::SetDefault("ns3::SatConf::FwdUserLinkBandwidth",     DoubleValue(25.0e6));
Config::SetDefault("ns3::SatConf::RtnUserLinkBandwidth",     DoubleValue(25.0e6));
Config::SetDefault("ns3::SatConf::FwdUserLinkBaseFrequency", DoubleValue(30.0e9));
```

---

## 輸出檔案

| 檔案 | 內容 | 寫入時機 |
|------|------|---------|
| `footprint-geometry.csv` | `beam_id, row, col, lat_deg, lon_deg, cx_m, cy_m` | SNS3 初始化之前 |
| `footprint-results.csv` | `beam_id, row, col, sat_id, lat_deg, lon_deg`（模擬後位置驗證） | 模擬結束後 |
| `data/sat-stats-per-ut-fwd-composite-sinr-scatter-*.dat` | 每顆 UT 的 SINR | 模擬結束後 |

---

## 執行指令

```bash
# 步驟一：只驗證幾何（檢查 C_i[n] 矩陣編號）
./ns3 run "sat-bh-2d-footprint --Nx=4 --Ny=3 --rFootprint=300000 --simTime=0"

# 步驟二：完整模擬（60秒，12 beams，Ka-band）
./ns3 run "sat-bh-2d-footprint --Nx=4 --Ny=3 --rFootprint=300000 --simTime=60" \
  2>&1 | tee footprint_run.log

# 步驟三：較大格子（5×4 = 20 beams）
./ns3 run "sat-bh-2d-footprint --Nx=5 --Ny=4 --rFootprint=300000 --simTime=60"
```

> **參數換算：** `--rFootprint=R`（公尺）= h / tan(θ_min)
> 例：h=600km，θ_min=63.4° → R ≈ 300000 m

---

## 驗證清單

- [ ] `footprint-geometry.csv` 在 SNS3 啟動前寫出
- [ ] 終端機矩陣顯示 beam_id=1（C_i[n] 第一格）在左下角
- [ ] 所有格子距中心點距離 ≤ R = rFootprint_m
- [ ] `Nx × Ny ≤ 72` 斷言通過
- [ ] 模擬後 `sat-stats` scatter 檔案有 Nx×Ny 筆資料（每顆 UT 一筆）
- [ ] `footprint-results.csv` 正確對應 beam_id ↔ 地理座標

---

## 實作注意事項

1. **`rFootprint_m` 對應 R = h / tan(θ_min)。** 服務區域邊長 Ld = 2R，格子寬度 d = Ld / Nx。

2. **`SatBeamUserInfo(1,1)` 不會明確設定 UT 位置。** SNS3 會將 UT 放在該 beam 涵蓋範圍內，實際位置可能與格子中心稍有偏差。

3. **`SetBeamSet` 必須呼叫。** 缺少此呼叫會導致 `SetBeamUserInfo` 無效，UT 不會被建立。

4. **Beam ID 有效範圍為 1–72。** 超出範圍會在執行期報錯（Nx × Ny ≤ 72）。

5. **本檔案不包含任何 BH 排程器元件。** 僅驗證 C_i[n] 幾何座標與基本 link budget。
