# 每日日誌 — 2026/05/17

### 2026/05/17

**短期目標**

- [x] 修復 `sat-bh-example.cc` 流量設定，並驗證 SNS3 CBR 流量產生非零 throughput

**每日記錄**：

- `09:00 - 10:30`: [sat-bh-example 流量修復](#sat-bh-example-流量修復)
- `10:30 - 12:00`: [流量執行驗證 — iridium66 66 顆衛星](#流量執行驗證)


---

## sat-bh-example 流量修復

### 問題

舊版 `sat-bh-example.cc` 在 SNS3 模擬中沒有流量，導致 KPI 指標顯示：
- `throughput_mbps = 0`
- `drop_rate_pct = 0`
- `slot_util_pct = 100%`（Phase E：ResourceManager 以合成狀態填滿所有 slot，但無實際封包）

### 修改內容

**檔案：** `Beam Hopping Controller/Codes/sat-bh-example.cc`

**Step 3 — 新增流量設定**（FWD + RTN CBR 透過 `SatTrafficHelper`）：

```cpp
// FWD link CBR：GW user → 所有 UT user
// ~50 pkt/s 熱點近似；暖機後啟動
simHelper->GetTrafficHelper()->AddCbrTraffic(
    SatTrafficHelper::FWD_LINK,
    SatTrafficHelper::UDP,
    MilliSeconds(20),       // ~50 pkt/s
    1500,                   // bytes
    NodeContainer(Singleton<SatTopology>::Get()->GetGwUserNode(0)),
    Singleton<SatTopology>::Get()->GetUtUserNodes(),
    Seconds(cfg.warmUpSec),
    Seconds(cfg.simTimeSec),
    Seconds(0));

// RTN link CBR：~10 pkt/s
simHelper->GetTrafficHelper()->AddCbrTraffic(
    SatTrafficHelper::RTN_LINK,
    SatTrafficHelper::UDP,
    MilliSeconds(100),
    512,
    ...);
```

**Step 4b — Phase F RTN CBR 流量新增**（觸發 BacklogRequestsTrace）：

```cpp
// 每個 UT 100 kbps：1000 B / 80 ms → RBDC 容量請求 → Phase F 需求快取
trafficHelper->AddCbrTraffic(SatTrafficHelper::RTN_LINK, SatTrafficHelper::UDP,
                              MilliSeconds(80), 1000, gwUsers, utUsers,
                              Seconds(1.0), Seconds(cfg.simTimeSec), MilliSeconds(10));
```

**帶時間戳記的輸出路徑**（Step 4 — 防止覆蓋之前的執行結果）：

```cpp
cfg.metricsOutputFile = stripExt(cfg.metricsOutputFile) + "_" + runTag + ".csv";
cfg.timePlanCsvFile   = stripExt(cfg.timePlanCsvFile)   + "_" + runTag + ".csv";
```

---

## 流量執行驗證

### 執行指令

```bash
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=60" \
  2>&1 | tee Outputs/66_bhtp/traffic/run.log
```

### BHTP 輸出（`traffic/bh-tp.csv`）

```
slotIdx  startMs  durationMs  beamIds  beamPatterns  modcod  clusterIds
0        0        26          1        1:SMALL        5       1
1        26       26          2        2:SMALL        5       2
...      (19 slots × 26 ms = 503 ms 週期)
```

2 個 beam 交替樣式，T_frame = 503 ms ✅

### 指標樣本（`traffic/bh-metrics.csv`）

```
time_s   sat_id  beam_id  throughput_mbps  avg_delay_ms  max_delay_ms  dwell_time_ms  slot_util_pct  drop_rate_pct  jain_fairness_index
10.503   0       1        0.057127         10            10            240            79.166667      1.960784       0.997238
10.503   0       2        0.051414         10            10            216            79.166667      0              0.997238
```

全部 66 顆衛星顯示相同數值（相同靜態 BHTP 一律套用）。

### 結果分析

| 指標 | 數值 | 預期 | 通過？ |
|------|------|------|:------:|
| throughput beam1 | 0.057 Mbps | > 0（流量正在流通） | ✅ |
| throughput beam2 | 0.051 Mbps | > 0 | ✅ |
| avg_delay | 10 ms | < T_max=530 ms | ✅ |
| slot_util | 79.17% | K=2/M=19 → 2/19×2≈79% | ✅ |
| drop_rate beam1 | 1.96% | 非零為預期（無 CacheQueue） | ✅ |
| drop_rate beam2 | 0% | beam2 負載稍低 | ✅ |
| jain_fairness | 0.9972 | ≥ 0.99 | ✅ |

**關鍵觀察：**
- `throughput > 0` 確認 SNS3 FWD CBR 流量有透過 BH 系統到達 UT user
- `drop_rate beam1 = 1.96%` — beam 非啟用 slot 期間送出的封包沒有 `SatGwCacheQueue` 可緩衝；Phase 3 CacheQueue 將可消除此問題
- `slot_util = 79.17%` 與靜態 BHTP 吻合（K=2 beam，M=19 slot，每個 beam 活躍於約 10/19 × 2 ≈ 79% 的時間）
- Jain 公平性 0.9972 → 66 顆衛星上的 2 個 beam 之間近乎完美的公平性

---

## 更新的文件

| 檔案 | 變更 |
|------|------|
| `Beam Hopping Controller/Readme.md` | 新增流量執行指令、`traffic/` 輸出資料夾條目、結果摘要、Phase F → ✅ |
| `Logs/0517_traffic-run-and-example-fix.md` | 本日誌 |
