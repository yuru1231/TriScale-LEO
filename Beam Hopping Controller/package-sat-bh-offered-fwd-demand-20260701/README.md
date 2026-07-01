# sat-bh-example：FWD Offered Demand 模型驗證


## 目的

FWD Offered Demand 排程流程：

```text
FWD Offered Load (20 Mbps/UT)
    → enableFwdOfferedDemand=1
        → DynamicBstp DEMAND 寫入 OFFERED_FWD
            → SatBhObc ToggleState
                → GW SatNetDevice 真實 beam 切換
                    → 封包吞吐量驗證
```

## 情境參數

| 參數 | 值 |
|------|----|
| 星座 | Starlink-1584（constellation-starlink-1584-sats）|
| ROI 中心 | 東京（35.676°N, 139.650°E）|
| 波束格式 | 25-beam 5×5  |
| 輔助衛星 | 382, 404（2 顆）|
| K（每槽同時啟動波束數）| 2 |
| BHTP 週期 | 8 槽 × 10ms = 80ms |
| 模擬時長 | 30s（simTime=30）|
| 暖機時間 | 0.1s（warmUp=0.1）|
| 啟用功能 | OBC + DynamicBstp + enableFwdOfferedDemand |

---

## 執行指令

```bash
./ns3 run "sat-bh-example \
  --scenario=starlink25 \
  --enableObc=1 \
  --enableDynamicBstp=1 \
  --enableFwdOfferedDemand=1 \
  --fwdOfferedDemandKbps=20000 \
  --maxActiveBeams=2 \
  --simTime=30 \
  --warmUp=0.1 \
  --helperSatList=382,404 \
  --metricsFile=/tmp/offered2-metrics-test.csv \
  --timePlanFile=/tmp/offered2-timeplan-test.csv \
  --trafficTraceFile=/tmp/offered2-traffic-test.tr"
```

---

## 結果檔案

| 檔案 | 列數 | 說明 |
|------|------|------|
| `offered2-metrics-test_starlink25_20260701_203429.csv` | 1,493 | KPI 每 T_p=80ms 每衛星每波束 |
| `offered2-timeplan-test_starlink25_20260701_203429.csv` | 9（含表頭）| BHTP 時槽表 |
| `offered2-traffic-test_starlink25_20260701_203429.tr` | 48,743 | BH 流量追蹤（PLAN+DEMAND+EVENT）|

---

## Trace 欄位說明

```text
record_type, time_s, sat_id, beam_id, event, plan_id, slot_idx,
demand_kbps, duration_ms, active_beams, mapped, toggled
```

| record_type | 說明 |
|-------------|------|
| PLAN | 每個排程週期的 BHTP 時槽計畫，含各槽啟動波束清單 |
| DEMAND | 波束需求快照；本套件全為 `OFFERED_FWD` 類型，demand_kbps=20000 |
| EVENT | OBC 執行 ACTIVATE/DEACTIVATE 的實際切換記錄 |

- `mapped=1`：OBC 成功找到對應 GW SatNetDevice
- `toggled=1`：ToggleState() 已實際呼叫
- 所有 23,992 筆 EVENT 均為 `mapped=1, toggled=1`

---

## BHTP 時槽結構

| slotIdx | 起始 (ms) | 持續 (ms) | 啟動波束 | modcod |
|---------|-----------|-----------|----------|--------|
| 0 | 0 | 10 | 1, 2 | 5 |
| 1 | 10 | 10 | 3, 4 | 5 |
| 2 | 20 | 10 | 5, 6 | 5 |
| 3 | 30 | 10 | 7, 8 | 5 |
| 4 | 40 | 10 | 9, 10 | 5 |
| 5 | 50 | 10 | 11, 12 | 5 |
| 6 | 60 | 10 | 13, 14 | 5 |
| 7 | 70 | 10 | 15, 16 | 5 |

- 週期 T_p = 80ms，共 8 槽，每槽 K=2 波束
- 初始計畫覆蓋波束 1–16；DynamicBstp 在後續週期動態調整

---

## 流量設定

| 方向 | 類型 | 間隔 | 封包大小 | 速率 |
|------|------|------|----------|------|
| FWD（GW → UT）| CBR UDP | 600 μs | 1500 B | 20 Mbps/UT |
| RTN 基礎線（UT → GW）| CBR UDP | 100 ms | 512 B | ~41 kbps/UT |

> FWD 20 Mbps 的 offered load 透過 `enableFwdOfferedDemand=1` 直接注入
> DynamicBstp 排程器，形成 `OFFERED_FWD` demand 記錄。


---

## 程式碼檔案

| 檔案 | 說明 |
|------|------|
| `code/scratch/bh_dynamic/Codes/sat-bh-example.cc` | 主模擬入口；StarLink25 場景、FWD PDR 追蹤、Phase F/G 旗標 |
| `code/contrib/satellite/helper/sat-bh-helper.h` | BhExperimentConfig 結構；所有模擬旗標定義 |
| `code/contrib/satellite/helper/sat-bh-helper.cc` | SatBhHelper 實作；DynamicBstp / OBC 安裝與接線 |

---

## 執行階段說明

| 階段 | 旗標 | 功能 |
|------|------|------|
| Phase 1 | 預設啟用 | SatBhTimePlan + SatBhMetrics（靜態 BHTP，合成驅動）|
| Phase 2/OBC | `--enableObc=1` | SatBhObc 真實 ToggleState 切換 |
| Phase G | `--enableDynamicBstp=1` | SatDynamicBstpProvider 貪婪 Top-K 排程 |
| **Offered FWD** | `--enableFwdOfferedDemand=1` | **直接以 FWD offered load 作為需求|

---

## TR 檔結果說明

`.tr` 檔（`offered2-traffic-test_starlink25_20260701_203429.tr`）共 48,743 筆，涵蓋 PLAN、DEMAND、EVENT 三類記錄，記錄整個 30s 模擬的排程決策與執行狀況。

### 記錄數量分佈

| record_type | 筆數 | 說明 |
|-------------|------|------|
| PLAN | ~3,008 | 每 T_p=80ms 產生一組時槽計畫（2 衛星 × 376 週期 × 4 行/計畫） |
| DEMAND | ~23,744 | 每週期每波束一筆；demand_kbps=20000（OFFERED_FWD 固定值） |
| EVENT | 23,992 | 每槽邊界的 ACTIVATE / DEACTIVATE 切換紀錄 |

> 總週期數約 376（30s ÷ 80ms），2 顆衛星各自獨立排程。

---

### PLAN 記錄解讀

```text
PLAN, 0.080, 382, -, -, 1, 0, -, 10, 2, -, -
```

- `sat_id=382`，`plan_id=1`，`slot_idx=0`，`duration_ms=10`，`active_beams=2`
- 代表第 1 個 BHTP 週期的第 0 槽，382 號衛星啟動 K=2 條波束，持續 10ms
- DynamicBstp 每週期依 OFFERED_FWD 需求貪婪選出排名前 K 的波束填入各槽

---

### DEMAND 記錄解讀

```text
DEMAND, 0.080, 382, 3, OFFERED_FWD, 1, -, 20000, -, -, -, -
```

- `beam_id=3`，`event=OFFERED_FWD`，`demand_kbps=20000`
- 所有 DEMAND 均為 `OFFERED_FWD` 類型，值固定 20000 kbps（等於 FWD CBR 設定）
- 驗證：`enableFwdOfferedDemand=1` 成功將 FWD offered load 注入排程器

---

### EVENT 記錄解讀

```text
EVENT, 0.080, 382, 1, ACTIVATE,   1, 0, -, 10, 2, 1, 1
EVENT, 0.170, 382, 1, DEACTIVATE, 1, 0, -, 10, 2, 1, 1
```

- `event=ACTIVATE`：OBC 於槽開始時呼叫 `ToggleState(true)`
- `event=DEACTIVATE`：OBC 於槽結束時呼叫 `ToggleState(false)`
- `mapped=1`：OBC 成功找到對應 GW SatNetDevice
- `toggled=1`：ToggleState() 實際執行（非空呼叫）
- 全部 23,992 筆 EVENT 均滿足 `mapped=1, toggled=1` → **OBC 切換 100% 成功**

---

### 關鍵驗證結論

| 驗證項目 | 結果 |
|----------|------|
| DEMAND 類型 | 全為 `OFFERED_FWD`，無 `SYNTHETIC` 混入 |
| DEMAND 值 | 固定 20000 kbps，與 `--fwdOfferedDemandKbps=20000` 一致 |
| EVENT 覆蓋率 | mapped=1 且 toggled=1 佔比 **100%**（23,992/23,992） |
| 最後切換時間 | `t=29.998s`（plan_id=375），符合 simTime=30s |
| BHTP 週期數 | 約 376 週期 × 80ms ≈ 30.08s，涵蓋完整模擬時長 |

> **結論**：FWD Offered Demand 模式下，排程器正確以 offered load 驅動波束選擇，
> OBC 每槽邊界切換全數成功，TR 檔內容與預期流程完全吻合。

---

## 逐步說明

1. starlink25 場景載入 Starlink-1584 星座，t_offset=4168s（東京 ROI 峰值仰角快照）
2. 安裝 2 個 SatBhHelper（sat 382, sat 404），各覆蓋 25 波束
3. FWD CBR 20 Mbps/UT、RTN 基礎線 ~41 kbps/UT 開始於 warmUp=0.1s 後
4. SatDynamicBstpProvider 每 T_p=80ms 以 OFFERED_FWD=20000 kbps 選出 K=2 最優波束
5. SatBhObc 在槽邊界呼叫 ToggleState(true/false) — `mapped=1, toggled=1` 驗證通過
6. 模擬至 t=30s，最後 DEACTIVATE EVENT 於 t=29.998s（plan_id=375）
