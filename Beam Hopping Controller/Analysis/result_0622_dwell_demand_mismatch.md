# Phase G 結果分析 — Dwell Time vs Traffic Demand Mismatch

**日期**：2026-06-22  
**場景**：starlink25, K=2, simTime=60s, enableObc=1, enableDynamicBstp=1  
**資料**：`Outputs/bh_dynamic_1584/bh-metrics-starlink25_20260622.csv`, sat_id=498

---

## 1. 問題：假設的熱點 ≠ 真正的流量熱點

### Traffic hotspot（sat-bh-example.cc, line 495）

```cpp
const std::set<uint32_t> kHotspotBeams = {1, 4, 13, 19, 22};
// FWD hotspot → 600 kbps/UT  (5× demand)
// FWD non-hotspot → 120 kbps/UT
```

這些 beam 對應 5×5 UPA 格網的高流量位置：

| Cell idx | Beam ID | 流量 |
|---|---|---|
| 0 | **1** | 600 kbps |
| 3 | **4** | 600 kbps |
| 12 | **13** | 600 kbps |
| 18 | **19** | 600 kbps |
| 21 | **22** | 600 kbps |
| 其餘 20 beam | 6–25 (excluding 13,19,22) | 120 kbps |

### BHTP hotspot（BuildStaticBhtp 的 numHotspotBeams=5）

```
BHTP treats beams 1..5 as "hotspot" → gets SMALL pattern + more slots
```

### 對比

| Beam | Traffic demand | BHTP 分類 | 是否對齊 |
|---|---|---|---|
| 1 | 600 kbps | SMALL（hotspot） | ✓ |
| 2 | 120 kbps | SMALL（hotspot） | ✗ 多服務低需求 |
| 3 | 120 kbps | SMALL（hotspot） | ✗ 多服務低需求 |
| 4 | 600 kbps | SMALL（hotspot） | ✓ |
| 5 | 120 kbps | SMALL（hotspot） | ✗ 多服務低需求 |
| **13** | **600 kbps** | **LARGE（non-hotspot）** | **✗ 高需求低服務** |
| **19** | **600 kbps** | **LARGE（non-hotspot）** | **✗ 高需求低服務** |
| **22** | **600 kbps** | **LARGE（non-hotspot）** | **✗ 高需求低服務** |
| 6–12,14–18,20–25 | 120 kbps | LARGE（non-hotspot） | ✓ |

**對齊率：22/25 beam，3 個嚴重錯誤（beam 13, 19, 22 高需求卻是 LARGE）**

---

## 2. Dwell Time 實測（Phase G, 無 Phase F demand，fairness 模式）

從 bh-metrics active satellite 資料（sat_id=498 filter）：

| Beam | avg_dwell_ms | BHTP 分類 | Traffic demand | 狀態 |
|---|---|---|---|---|
| 1 | 171ms | SMALL | 600 kbps | 高dwell + 高需求 ✓ |
| 2 | 147ms | SMALL | 120 kbps | 高dwell + 低需求 ✗ |
| 3 | 147ms | SMALL | 120 kbps | 高dwell + 低需求 ✗ |
| 4 | ~24ms\* | SMALL | 600 kbps | 低dwell + 高需求 ✗ |
| 5 | ~24ms\* | SMALL | 120 kbps | （此點待確認）|
| 6–12 | 24ms | LARGE | 120 kbps | 對齊 ✓ |
| **13** | **24ms** | **LARGE** | **600 kbps** | **嚴重低服務 ✗** |
| 14–18 | 24ms | LARGE | 120 kbps | 對齊 ✓ |
| **19** | **24ms** | **LARGE** | **600 kbps** | **嚴重低服務 ✗** |
| 20–21 | 24ms | LARGE | 120 kbps | 對齊 ✓ |
| **22** | **24ms** | **LARGE** | **600 kbps** | **嚴重低服務 ✗** |
| 23–25 | 24ms | LARGE | 120 kbps | 已修 ✓ |

\* beam 4 的 dwell 需重新確認（原始 awk 用 $6 非 $7 可能誤讀）

---

## 3. 根本原因

`numHotspotBeams = 5` 讓 BHTP 把 **beams 1–5（前 5 個 ID）** 當熱點，
但實際熱點是 **{1, 4, 13, 19, 22}（UPA 格網的 5 個角落/邊緣位置）**。

這兩個集合交集只有 beam 1 和 beam 4，其餘 3 個真實熱點（13, 19, 22）
被錯誤地分配到 LARGE 類別，每個 T_p 只得到 24ms dwell。

Phase G greedy provider 在沒有 Phase F 的情況下退化為 **fairness round-robin**，
因此不會根據實際 demand 自動調整，mismatch 會持續整個模擬。

---

## 4. 修正方向

### Option A：修正 numHotspotBeams 設定（不等效）

`numHotspotBeams` 是「前 H 個 beam」的簡單分組，無法表達 {1,4,13,19,22} 這種非連續集合。
將 H 改為 5 並不能解決問題，H=22 才能讓 beam 22 進入 hotspot 範圍，但這樣 beam 6–21 也全變 hotspot，效果更差。

→ 這個修正方向**無效**。

### Option B：讓 Phase G demand-aware（接 Phase F）— 推薦

有了真實 RBDC demand，`SatGreedyBstpProvider` 會根據 demand score 選 top-K beam：

```
score(b) = bhDemandBacklogWeight × demand_kbps(b)
         + bhFairnessWeight × time_since_served(b)
```

Beam 13（600kbps）的 demand score 會壓過 beam 2（120kbps），
provider 自動在每個 T_p 選 beam 13 而非 beam 2。

**這正是 Phase G 設計的核心目的：取代靜態分組，用 demand 動態決定服務順序。**

執行指令：
```bash
./ns3 run "Codes/sat-bh-example --scenario=starlink25 \
  --enableObc=1 --enableDynamicBstp=1 \
  --enableResourceManager=1 --enablePhaseE=1 --enablePhaseF=1 \
  --maxActiveBeams=2 --simTime=120" \
  2>&1 | tee bh_starlink25_phaseEF_0622.log
```

### Option C：修改 BuildStaticBhtp 接受自訂 hotspot beam ID 列表

擴充 `BhExperimentConfig` 加入 `hotspotBeamIds: vector<uint32_t>`，
讓靜態計畫也能正確反映非連續熱點集合。這能修正靜態 fallback plan，
但 Phase G 仍需 Phase F 才能動態感知 demand。

---

## 5. 圖表

執行分析腳本（在 VMware 上）：

```bash
python3 "Beam Hopping Controller/Analysis/exp_bh_dwell_vs_demand_0622.py" \
  --metrics "Beam Hopping Controller/Outputs/bh_dynamic_1584/bh-metrics-starlink25_20260622.csv" \
  --sat_id 498 \
  --output "Beam Hopping Controller/Analysis/fig_bh_dwell_demand_0622.png"
```

**輸出：**
- `fig_bh_dwell_demand_0622.png` — 雙面板：上圖 dwell time，下圖 traffic demand，顏色標示 mismatch
- `fig_bh_dwell_demand_0622_scatter.png` — scatter：X=demand, Y=dwell，理想應為正相關
- `fig_bh_dwell_demand_0622_stats.json` — per-beam 數據

**色碼說明：**
- 紅色：beam 同時是 traffic hotspot + BHTP hotspot（beam 1, 4）
- 橘色：traffic hotspot 但 BHTP non-hotspot（beam 13, 19, 22）← 問題所在
- 藍色：BHTP hotspot 但 traffic non-hotspot（beam 2, 3, 5）← 浪費服務
- 淺藍：雙 non-hotspot（正常）

---

## 6. 預期 Phase F 後的改善

| Beam | 現在 dwell（無 Phase F） | 預期 dwell（有 Phase F） |
|---|---|---|
| 13, 19, 22 | 24ms（非熱點） | >>24ms（demand 高，greedy 優先選） |
| 2, 3, 5 | 147ms（錯誤優先） | <147ms（demand 低，greedy 跳過） |
| 1, 4 | 171/24ms | 較高（demand + BHTP 雙重優先） |

Jain Fairness Index 預期從 0.217 提升（需求越高的 beam 越能得到服務）。
