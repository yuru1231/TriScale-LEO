# 2026/06/05

## 19-cell Hex 改為 25-cell 橢圓修正 ROI Grid

## 設計決策

### 問題背景（
| 問題 | 根本原因 |
|------|---------|
| `GetBeamCentersFromSatPos` 編譯失敗 | `orbit-sgp4/code/sat-multi-beam-geometry.h` 未宣告該函式（phase1/sgp4 已有實作，orbit-sgp4 尚未 port）|
| Beam gain 僅 ~17 dBi（預期 47.7 dBi） | 25-cell 方形格點與 19-beam 不對齊，cell 落在 sidelobe |
| 仰角修正不正確 | `beamCentersEnu` 以天頂假設靜態預計算，低仰角無橢圓拉伸 |

### 設計

```
仰角 θ（ROI 中心對衛星）→ 橢圓半軸：
  a (along-track) = r_footprint / sin(θ)
  b (cross-track) = r_footprint

最大內切矩形：
  W_along = a / √2,  H_cross = b / √2

Along-track 方向：satEnu − prevSatEnu（有限差分）
5×5 grid center beam：index 12（row=0, col=0）
cellPos = beamCenters → ΔΦ = 0 → peak gain
```

### nBeams vs nBeamsY 說明

| 參數 | 用途 | 改動 |
|---|---|---|
| `nBeams = 25` | `ComputeFrameResults` 迴圈上限、SINR 干擾加總 | 19 → **25** |
| `nBeamsX = 5` | UPA x 方向 beam port 數，影響天線陣列幾何尺寸（beam gain 計算） | 不變 |
| `nBeamsY` | UPA y 方向 beam port 數，影響天線陣列幾何尺寸 | 4 → **5**（待確認衛星天線配置）|

---

## 修改清單

| 檔案 | 修改內容 |
|---|---|
| `sat-multi-beam-config.h` | `nBeams 19→25`，`nBeamsY 4→5` |
| `sat-multi-beam-geometry.h` | 新增 `GetBeamCentersFromSatPos`（19-beam，修復編譯錯誤）及 `GetEllipticBeamCenters`（25-beam）宣告 |
| `sat-multi-beam-geometry.cc` | Port `GetBeamCentersFromSatPos` from phase1/sgp4；實作 `GetEllipticBeamCenters` |
| `sat-multi-beam-channel.h` | `array<Vec3,19>→25`，center beam 備註 9→12；更新 docstring（FSPL 改為直接公式）；`ComputeAtmosphericLoss_dB` 加 `freqHz` 預設參數（預設 30 GHz）|
| `sat-multi-beam-channel.cc` | `array<Vec3,19>→25`，`centreBeamIdx = 9→12`（已於 2026-05 提交）；z-axis 預旋轉修正 `BuildArrayTransform`/`GetSpatialFreqs`（已於 2026-05 提交）；今日（0605）：移除 SNS3-specific includes；`ComputeFSPL_dB` 改為直接公式；`ComputePathLoss_dB` 改為 FSPL + atmospheric 分離計算，並傳入 `cfg.centerFreqHz` |
| `sat-roi-grid.h` | 新增 `EllipticFootprint` struct + `ComputeEllipticFootprint()` 宣告 |
| `sat-roi-grid.cc` | 實作 `ComputeEllipticFootprint()` |
| `sat-constellation-scanner.h` | `SatScanState`：移除 `beamCentersEnu`，新增 `prevSatEnu`/`hasPrevSat`；`SetTleReader` 改用 `const SatTleReader&`（修正 `ns3::Ptr<>` 誤用）；新增 explicit constructor |
| `sat-constellation-scanner.cc` | `ScanSnrCallback` 改用 `GetEllipticBeamCenters` 25-cell；`Run()` 移除靜態 19-beam 預計算；加入 `NS_ABORT_MSG_IF` null guard |

---

## 編譯錯誤修正


### 1. `ns3::Ptr<SatTleReader>` 誤用

**原因**：`ns3::Ptr<T>` 要求 `T` 繼承 `ns3::SimpleRefCount<T>` 或 `ns3::Object`，`SatTleReader` 不符合。

**修正**：
```cpp
// scanner.h — 改用裸指標
const SatTleReader* m_tleReader{nullptr};

// 介面改為 const reference
explicit SatConstellationScanner(const SatTleReader& tleReader);
void SetTleReader(const SatTleReader& tleReader);

// scanner.cc — constructor 實作
SatConstellationScanner::SatConstellationScanner(const SatTleReader& tleReader)
    : m_tleReader(&tleReader) {}

// Run() 加入 null guard
NS_ABORT_MSG_IF(m_tleReader == nullptr, "SatTleReader must be set before Run()");
```
✅ fixed

---

## 驗證結果

### 模擬輸出（elev = 85.1°，sat_44）

```
t=582.0  elev=85.17°  cell= 0  beam_gain=46.52 dBi  snr=8.92 dB
t=582.0  elev=85.17°  cell= 1  beam_gain=46.52 dBi  snr=8.95 dB
t=582.0  elev=85.17°  cell=12  beam_gain=46.52 dBi  snr=8.96 dB  ← center
```

| 指標 | 0604 sidelobe bug | Phase 2.1（今日）| 狀態 |
|---|---|---|---|
| beam_gain | ~17.5 dBi | **46.5 dBi** |  +29 dB |
| snr | ~−35 dB | **+8.9 dB** |  可服務 |
| Low-SNR gap | 304 s (8.4%) | **319 s (8.9%)** |  橢圓修正使路徑損耗更真實 |
| MRC gap | 3573 s (99.2%) | **51 s (1.4%)** |  −3522 s |
| Critical elevation | 7.3° (600 km) | **11.1°** |  Iridium 780 km 正確值 |

**剩餘 51 s gap**：結構性問題（sat_19、sat_49 最高仰角 12~13°，MRC 仍低於 0 dB），為 Iridium 軌道設計限制。

---

## 驗證指令

```bash
# 1. 編譯
./ns3 build sat-multi-beam-simulation 2>&1 | grep -E "error:|warning:"

# 2. 執行星座掃描
./ns3 run "sat-multi-beam-simulation \
  --constellation-dir=contrib/satellite/data/scenarios/constellation-iridium-next-66-sats \
  --lat=35.676 --lon=139.65 \
  --window-s=3600 --dt-snr-s=1 \
  --min-elevation-deg=5 \
  --out-dir=scratch/grid25_out" 2>&1 | tee grid25.log

# 預期：
# [Constellation] Phase 2.1 — 5x5 elliptic grid  n_cells=25
# cell_idx 0~24（共 25 格）
# cell 12（中心格）beam_gain_dB ≈ 47.7 dBi（天頂衛星）
```
