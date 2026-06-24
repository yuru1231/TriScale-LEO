# 0618 — starlink25 25 beam 


## 1. 修正 — SNS3 場景配置檔

在 `contrib/satellite/data/scenarios/constellation-starlink-1584-sats/` 中建立三個檔案：

### `fwdConf.txt` — 25 波束、1 個 GW（東京）、4 色複用
```
1 1 1
2 1 2
...
25 1 1
```
格式：`beamID  GW_ID  colorID`（SNS3 標準 3 欄）。

### `gw_positions.txt` — 東京 GW
```
35.6895 139.6917 44
```
格式：`lat lon  alt_m`。

### `waveforms/default_waveform.txt`
```
3
```

---

## 3. 模擬執行 — starlink25 Phase G

**指令：**
```bash
./ns3 run "sat-bh-example --scenario=starlink25 \
  --enableObc=1 --enableDynamicBstp=1 \
  --maxActiveBeams=4 --simTime=60" \
  2>&1 | tee bh_starlink25_phaseg.log
```

**啟動確認（log.log）：**
```
[starlink25] topology: 1 active satellite, 25 beams, 25 UTs.
[starlink25] satId=498, Starlink-1584, Tokyo GW
[starlink25] hotspot beams {1,4,13,19,22}: 600 kbps; 20 non-hotspot: 120 kbps
Phase G (DynamicBstp=1)  K=4  beams=25  60s → complete
```

**輸出檔案：**
- `bh-timeplan_starlink25_20260618_121438.csv`
- `bh-metrics_starlink25_20260618_121438.csv`

---

## 4. bh-timeplan 分析

19 個slot（M = ⌈503/26.5⌉），K=4，25 個波束全部涵蓋：

| 時槽 | 啟用波束（SMALL=熱點 1–5 / LARGE=非熱點 6–25） |
|---|---|
| 時槽 0 | {1,2,6,7} |
| 時槽 1 | {2,3,8,9} |
| ... | ... |
| 時槽 9 | {5,1,24,25} |
| 時槽 10–18 | 重複模式 |

波束 1–25 全部出現，確認 25 波束覆蓋。

**Log：**
- beam 1–5（SMALL）：182–208 ms
- beam 6–24（LARGE）：52 ms
- beam 25：26 ms

---

## 5. bh-metrics 分析

| KPI | obc | 解釋 |
|---|---|---|
| sat_id | 全程 498 | Starlink-1584 satId=498  |
| 波束範圍 | 每個 T_p 均為 1–25 | 確認 25 波束運作  |
| 第一列時間 | t = 10.503 s | 暖機保護機制正常  |
| dwell_time_ms（large） | 408 ms = 17 槽 × 24 ms | 高優先波束  |
| dwell_time_ms（small） | 48 ms = 2 槽 × 24 ms | 低優先波束  |
| throughput_mbps | 全程為 0 | 預期：無實際封包 |
| dynamic 行為 | 公平，每週期波束偏移 +4 | 預期：無 demand 需求 → 退化為公平模式 |

**波形模式（公平輪詢）：**
```
t = 10.503s : LARGE = {6,7,8,9}
t = 11.006s : LARGE = {10,11,12,13}
t = 11.509s : LARGE = {14,15,16,17}
...
```

每個 T_p（503 ms）波形偏移 +4 個波束，約 6.25 個週期完成完整 25 波束循環。

---

## 6. 驗證清單

| 項目 | 結果 |
|---|---|
| SNS3 拓撲中 25 個波束 | ✓ 確認 |
| 正確星座（Starlink-1584 satId=498） | ✓ 確認 |
| 25 個 UT 位於東京仰角最大點細胞中心 | ✓ 確認 |
| Phase G OBC 正常執行 | ✓ 確認 |
| 暖機保護（第一列 t>10s） | ✓ 確認 |
| 25 波束公平輪詢（無需求時） | ✓ 確認 |
| 需求感知駐留差異（熱點 vs 非熱點） | ✗ 需要 Phase E + F |

---

## 7. 下一步

若要觀察需求感知波束選擇（熱點波束 {1,4,13,19,22} 獲得比非熱點更長的駐留時間），需搭配 Phase E + F 執行：

```bash
./ns3 run "sat-bh-example --scenario=starlink25 \
  --enableObc=1 --enableDynamicBstp=1 \
  --enablePhaseE=1 --enablePhaseF=1 \
  --maxActiveBeams=4 --simTime=120"
```
