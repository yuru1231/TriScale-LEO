# Layer 1 結案 (stub)

**2026-04-16**
**狀態：** 暫時結案
**Build tag：** `2026-04-16-rf-v4`

---

## 1. 模組清單

| 模組 | 檔案 | 說明 |
|------|------|------|
| ISL 圖建構 | `isl-graph.cc / .h` | 從 `isls.txt` 建立衛星鄰接圖，計算 ISL 距離 |
| Dijkstra 路徑計算 | `isl-graph.cc` | 以 ISL 傳播延遲 + EMA load cost 為權重的最短路徑 |
| 時間槽路由管理 | `isl-routing-manager.cc / .h` | 預計算各時間槽路由表、增量更新、Cooldown 機制 |
| GW 可見衛星選擇 | `isl-routing-manager.cc` | 仰角門檻篩選，支援多衛星 tie-breaking |
| GW-to-GW 路由 | `isl-routing-manager.cc` | 跨 GW 對的雙向 ISL 路由預計算與 Report |
| GW-to-UT 路由 | `isl-routing-manager.cc` | entry→serving satellite 多 hop ISL 路由 |
| ISL Load Cost (EMA) | `isl-routing-manager.cc` | 佇列延遲 EMA 平滑、load-aware Dijkstra cost |
| E2E 模擬入口 | `test-iridium-e2e.cc` | 6 種 pathType、分段流量安裝、觀測框架 |
| ISL 封包落包監控 | `test-iridium-e2e.cc` | `PacketDropRateTrace` 接上 132 條 ISL |
| E2E Link Observability | `test-iridium-e2e.cc` | feeder/service/ISL 三段式統計、CSV log |
| GW-side feeder obs | `test-iridium-e2e.cc` | SAT→GW 下行（return feeder），用於 `sat2gw` |

---

## 2. 支援的 pathType

| pathType | feeder | ISL | service | 流量類型 | 驗證狀態 |
|----------|--------|-----|---------|---------|---------|
| `sat2sat` | — | ✓ | — | ISL background load | ✅ 驗證 |
| `gw2sat` | ✓ (up) | — | — | GW↔UT CBR | ✅ 驗證 |
| `sat2gw` | ✓ (dn) | — | — | GW↔UT CBR | ✅ 驗證 |
| `sat2ut` | — | — | ✓ | GW↔UT CBR | ✅ 驗證（E2E-Refactor） |
| `gw2ut_e2e` | ✓ | ✓ | ✓ | GW↔UT CBR | ✅ 驗證 |
| `gw2gw_e2e` | ✓* | ✓ | — | GW user → GW user UDP | ✅ 驗證 |

> *`gw2gw_e2e` 的 feeder 觀測在 REGENERATION_NETWORK 模式下不可行（見第 5 節說明），改以應用層封包計數驗證 E2E 。

---

## 3. 已驗證行為

### 3.1 ISL 拓撲建構
- 從 `isls.txt` 正確載入 132 條 ISL，生成 264 個有向介面

**log 摘錄（所有測試均出現）：**
```
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[OBS] traces connected:  feeder=66  service=66  isl=264
```

---

### 3.2 Dijkstra 路徑正確性
- TW-Taipei → US-SanFrancisco：slot 0 路徑 `15->14->25->36->37`，isl_cost = 0.0440s，符合預期
- slot 2（120s）衛星幾何變化 `15->14->13->2->1`
- slot 3（180s）衛星幾何恢復，縮短為 `15->14->25->36`（4 hop，exit sat 提前）

**log 摘錄（`gw2gw_e2e_TW_US_300s_clean.log`）：**
```
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)
0     0         15      15->14->25->36->37                                37      0.043969
1     60        15      15->14->25->36->37                                37      0.046214
2     120       15      15->14->13->2->1                                  1       0.047600   <-- ROUTE CHANGED
3     180       15      15->14->25->36                                    36      0.037717   <-- ROUTE CHANGED
```

---

### 3.3 時間槽路由動態切換
- 6 個時間槽（0/60/120/180/240/300s），slot 2 與 slot 3 觸發路由變更
- 每次切換均正確觸發 `ApplyRoutingTable: HasSignificantChange=YES`，並將新路由表寫入各衛星節點的 Ipv4StaticRouting 路由表

**log 摘錄（`gw2gw_e2e_TW_US_300s_clean.log`）：**
```
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
```

---

### 3.4 增量路由重計算
- 只重算幾何有變化的 src nodes，非全表重算（66 個 sat 僅部分重算）
- 重算比例隨 slot 累積增長（幾何逐漸偏移）

**log 摘錄（`gw2gw_e2e_TW_US_300s_clean.log`）：**
```
slot  simTime   apply(ms)  recompute(ms)  recompSrc  changed
0     0.00      0          0              0          NO
1     60.00     0          0              3          YES
2     120.00    1          0              17         YES
3     180.00    0          0              26         YES
4     240.00    1          1              30         YES
5     300.00    1          1              31         YES
```

---

### 3.5 ISL 封包落包率
- 14 條 E2E ISL 連結，1,585 萬+ 封包，0 drop

**log 摘錄（`gw2gw_e2e_TW_US_300s_clean.log`）：**
```
TOTAL: 15859375 pkts, 0 dropped | drop_rate=0.000% | success_rate=100.000%
[PASS] overall ISL drop rate < 1.000%
```

---

### 3.6 GW-to-GW E2E 交付
- GW0(TW-Taipei) → GW2(US-SanFrancisco)，simTime=300s
- GW2GW_DIRECT 應用層 byte count 驗證封包送達

**log 摘錄（`gw2gw_e2e_TW_US_300s_clean.log`）：**
```
[GW2GW_DIRECT] === Delivery summary ===
  src: GW0 (90.2.0.2)
  dst: GW2 (90.2.0.4)
  received: 1525248 bytes (~2979 pkts)
  [PASS] received>0, gateway-to-gateway path is active
```

---

### 3.7 sat2gw feeder 下行觀測
- GW-side `SatNetDevice::Rx` trace 成功捕捉 SAT→GW 下行流量
- `feeder:gw0 = 5148 pkts`（120s 模擬）
- **來源：** `E2E-ReturnFeeder/sat2gw_120s.md`

---

### 3.8 ISL Load Cost EMA（有背景負載時）
- 有背景負載時，ISL 14-15 load cost 從 0 升至 0.8385ms，ISL 36-37 升至 0.4333ms
- EMA 平滑計算正確，值納入 Dijkstra 權重

**log 摘錄（`gw2gw_e2e_TW_US_300s_bgload.log`）：**
```
edgeIdx satA  satB  loadAB(ms)  loadBA(ms)
28      14    15    0.8385      0.2690
72      36    37    0.0000      0.4333
```

**對照（`gw2gw_e2e_TW_US_300s_clean.log`）：**
```
edgeIdx satA  satB  loadAB(ms)  loadBA(ms)
28      14    15    0.0000      0.6816      ← 無背景負載，反向有值（feeder 流量）
```

---

### 3.9 GW 可見衛星動態變化
- GW0[TW-Taipei]：各 slot 可見 1–2 顆，正確反映衛星軌道移動
- GW2[US-SanFrancisco]：各 slot 可見 1–2 顆

**log 摘錄（`gw2gw_e2e_TW_US_300s_clean.log`）：**
```
[GwRouting] slot=0 t=0s   GW0[TW-Taipei]=2sats  GW2[US-SanFrancisco]=1sats
[GwRouting] slot=1 t=60s  GW0[TW-Taipei]=1sats  GW2[US-SanFrancisco]=2sats
[GwRouting] slot=2 t=120s GW0[TW-Taipei]=1sats  GW2[US-SanFrancisco]=1sats
[GwRouting] slot=3 t=180s GW0[TW-Taipei]=1sats  GW2[US-SanFrancisco]=2sats
```

---

### 3.10 sat2sat 8-hop 路徑 + ISL 多段流量
- SAT0→SAT33，完整 8 hop，7 條 ISL 全部有流量
- ISL 14-15 有高 load cost（10.18ms），為 GW 側大量背景負載所致

**log 摘錄（`sat2sat_s0s33_120s.log`）：**
```
time(s)  src  dst  full_path                            route_cost
0        0    33   0->1->2->57->46->35->34->33          0.078176
```

```
link      rx_pkts  drop_pkts  drop_rate(%)
isl:0-1   23974    0          0.00
isl:1-2   29745    0          0.00
isl:2-57  23977    0          0.00
isl:57-46 23977    0          0.00
isl:46-35 23975    0          0.00
isl:35-34 23975    0          0.00
isl:34-33 23973    0          0.00
```

```
TOTAL: 6926030 pkts, 16689 dropped | drop_rate=0.241% | success_rate=99.759%
[PASS] overall ISL drop rate < 1.000%
```

---

### 3.11 gw2ut_e2e 三段完整 E2E
- GW0(TW-Taipei) → UT-SanFrancisco(37.8°N, 122.4°W)
- ISL path `15->14->25->36->37`，4 slot，2 次路由切換
- feeder / ISL / service 三段觀測全部有數據，0 drop

**log 摘錄（`gw2ut_e2e_TW_SF_180s.log`）：**
```
[OBS] scope: feeder=1 service=3 isl=7

link             rx_pkts  rx_bytes   drop_pkts  drop_rate(%)  avg_delay(ms)
feeder:sat15     48261    74170544   0          0.00          5.15
service:sat37    2193     650090     0          0.00          3891.63
service:sat1     1261     389694     0          0.00          13341.96
isl:15-14        35972    35972      0          0.00          0.00
isl:14-25        35973    35973      0          0.00          0.00
isl:25-36        35972    35972      0          0.00          0.00
isl:36-37        35973    35973      0          0.00          0.00
...（7 條 ISL 全有流量）
```

```
TOTAL: 9579193 pkts, 0 dropped | drop_rate=0.000% | success_rate=100.000%
[PASS] overall ISL drop rate < 1.000%
```

---

## 4. 架構限制（已記錄）

### 4.1 gw2gw_e2e feeder 觀測不可行

**根因：**
Iridium-66 fixed 場景在 REGENERATION_NETWORK 模式下，只有 1 個物理 GW node（掛 66 個 `SatNetDevice`）。`SatNetDevice::Rx` 和 `SatGwMac::Tx` 不對 GW2GW_DIRECT unicast IP 流量觸發，這是 SNS3 regeneration 模式的架構限制。

**處理方式：**
- `ConfigureObsScope` 中 `gw2gw_e2e` 明確設定 `activeFeeder = false`
- feeder 段改由 GW2GW_DIRECT 應用層 byte count 驗證 E2E 交付
- code comment 記錄完整根因，避免後續重複踩坑

**程式碼位置：** `test-iridium-e2e.cc`，`ConfigureObsScope / gw2gw_e2e` block

---

### 4.2 sat2gw 觀測視窗假警報

- `sat2gw_120s.md` 中 70s 出現 `[OBS][EVENT] POSSIBLE LINK FAILURE`
- 實際原因：slot 轉換造成服務衛星切換，觀測視窗計算吞吐量為 0（前一視窗流量歸屬上一衛星）
- 非真實落包或路由失敗，屬於觀測時窗邊界效應
- **修復（rf-v4）：** 將觸發條件從 `rxPkts > 0 && tput < 0` 改為 `prevTput > 0 && tput < 0`（邊緣偵測），消除衛星切換瞬間的假警報

---

## 5. 補充證據結果（2026-04-16 補跑）

### sat2sat 8-hop 

- 路徑：`0->1->2->57->46->35->34->33`（8 hop）
- 7 條 ISL 連結全部有流量，整體 drop rate = 0.241% < 1%（[PASS]）
- ISL 14-15 在高背景負載下出現 10.18ms load cost，EMA 回饋機制有效運作
- **來源：** `E2E-ReturnFeeder/sat2sat_s0s33_120s.log`

---

### gw2ut_e2e 三段 E2E 

- GW0(TW-Taipei) → UT-SanFrancisco(37.8, -122.4)，ISL path = `15->14->25->36->37`（5 hop）
- `feeder:sat15` = 48261 pkts、`service:sat37` = 2193 pkts、`service:sat1` = 1261 pkts
- 7 條 ISL 連結均有流量，4 slot 有 2 次路由切換，0 drop
- 三段觀測（feeder / ISL / service）全部有數據
- **來源：** `E2E-ReturnFeeder/gw2ut_e2e_TW_SF_180s.log`

---

### Load-aware routing 
⚠️ 機制已驗，路由切換因負載規模不足而未觸發（預期行為）

比較有無背景負載的 TW→SF gw2gw_e2e：

| 指標 | clean.log（無負載）| bgload.log（有背景負載）|
|------|-------------------|----------------------|
| TW→SF 路由路徑 | `15->14->25->36->37` | 完全相同 |
| ISL 14-15 load (AB) | 0.0000 ms | 0.8385 ms |
| ISL 36-37 load (BA) | 0.0000 ms | 0.4333 ms |
| ISL 14-15 drops | 0 | 272 pkts (0.13%) |

**結論：** EMA load cost 計算正確（有背景負載時值明顯升高）；但最大 load cost ~0.84ms 遠小於傳播延遲 ~44ms（佔比 ~2%），幾何代價主導 Dijkstra，路由不變。這是正確行為，非 bug。

**判定：** load cost 機制存在且正確計算 ；路由切換未觸發屬預期行為，不列為缺陷。若未來需驗證 load-driven 路由切換，需設計接近 ISL 飽和的壓測場景。

- **來源：** `E2E-ReturnFeeder/gw2gw_e2e_TW_US_300s_bgload.log`

---

## 6. 程式碼變更

### 6.1 `test-iridium-e2e.cc` — 主要重構（整體架構調整）

#### (a) 新增 `ObsScope` struct 與 scope gating

**目的：** 過去 feeder / service / ISL observer 對所有 64 顆衛星都會嘗試觀測，導致 `gw2gw_e2e` 的 final summary 出現大量無意義的 `feeder:satN = 0` 行，干擾驗證。

**修改內容：**
```cpp
struct ObsScope
{
    bool activeFeeder{false};
    bool activeService{false};
    bool activeIsl{false};
    std::set<std::string> feederKeys;   // 僅觀測此集合內的 key
    std::set<std::string> serviceKeys;
    std::set<std::string> islKeys;
};
static ObsScope g_obsScope;
```

`ConfigureObsScope()` 依 pathType 填入各段的 active flag 與 key set：
- `gw2ut_e2e`：feeder = `{satN}`（entry sat），service = `{serving sats}`，isl = 沿路各 ISL
- `gw2gw_e2e`：**feeder = disabled**（SNS3 架構限制），isl = 沿路各 ISL
- `sat2gw`：feeder = `{gwN}`（GW Rx），isl = off

新增 `IsObsKeyInScope()` 做 gating，`CheckAndAlertObs()` 在最前端呼叫，非 scope 內的 key 直接 return：
```cpp
static bool IsObsKeyInScope(const std::string& linkType, const std::string& key)
```

---

#### (b) 新增 key 工廠函式

**目的：** 統一 key 格式，消除散落各處的 string 拼接，避免 feeder key 在不同 obs mode 下不一致的 bug。

```cpp
MakeSatKey(uint32_t satId)   → "sat<N>"   // orbiter feeder / service obs
MakeGwKey(uint32_t gwId)     → "gw<N>"    // GW-side Rx feeder obs (SAT→GW)
MakeGwTxKey(uint32_t gwId)   → "gwtx<N>"  // GW-side Tx obs (GW→SAT uplink)
MakeIslKey(uint32_t src, uint32_t dst) → "<src>-<dst>"
```

---

#### (c) 新增 `GatewayRxFeederCb` 與 `GatewayTxFeederCb`

**目的：** 支援 SAT→GW 下行（return feeder）觀測，用於 `sat2gw` pathType。`ConnectLinkObserverTraces(bool useGwReturnFeederObs)` 依 flag 決定掛 orbiter Rx callback 還是 GW SatNetDevice Rx callback。

```cpp
static void GatewayRxFeederCb(std::string key, Ptr<const Packet> pkt, ...)
// return feeder RX: gateway receives packets from satellite (SAT -> GW)

static void GatewayTxFeederCb(std::string key, Ptr<const Packet> pkt)
// entry feeder TX: gateway sends packets toward satellite (GW -> SAT uplink)
```

---

#### (d) 假警報修復（`CheckAndAlertObs` 邊緣偵測改進）

**問題：** 舊邏輯 `if (rxPkts > 0 && tput < 0)` 在衛星切換後第一個視窗內就觸發假警報（前一顆衛星已收到封包，但新視窗流量歸到新衛星 key，tput 瞬間為 0）。

**修復：** 改為追蹤 `prevTput`，只有 `prevTput > 0 && tput < 0` 才觸發（真正的從有到無才算異常）：
```cpp
// Before:
if (stats.rxPkts > 0 && tput < 1e-9)  // 只要收過封包且現在為 0，就告警

// After:
if (prevTput > 1e-9 && tput < 1e-9)   // 必須前一視窗有流量，本視窗才歸零，才算異常
```

---

#### (e) `gw2gw_e2e` feeder obs 明確停用

**位置：** `ConfigureObsScope()` → `gw2gw_e2e` branch

```cpp
// SNS3 REGENERATION_NETWORK mode has a single physical GW node with 66 SatNetDevices.
// SatNetDevice::Rx and SatGwMac::Tx do NOT fire for GW2GW_DIRECT unicast IP traffic.
// Feeder segment observability is therefore DISABLED for gw2gw_e2e.
// E2E delivery is verified via GW2GW_DIRECT byte count (application level).
g_obsScope.activeFeeder = false;
```

---

### 6.2 `isl-graph.cc` — 路由 Report 輸出微調

**位置：** `IslRoutingManager::PrintGwUtRouteReport()`

**修改：** 當路由有效但無 ISL transit hop（直連衛星）時，isl_cost 欄位從 `"N/A"` 改為 `"no ISL hop"`，語意更清楚：

```cpp
// Before:
costSs << "N/A";

// After:
costSs << "no ISL hop";
```

---

## 7. Layer 2 前提確認

| 項目 | 狀態 |
|------|------|
| ISL 路由功能正確 | ✅ |
| 時間槽動態更新 | ✅ |
| E2E 封包交付（GW2GW）| ✅ |
| ISL 落包監控 | ✅ |
| Load cost EMA 計算 | ✅（load-driven 路由切換因負載規模不足未觸發，屬預期行為）|
| feeder obs（sat2gw 方向）| ✅ |
| feeder obs（gw2gw 方向）| ⚠️ 架構限制，已記錄，以應用層 byte count 替代 |
| Layer 2 所需介面 | `IslRoutingManager`、時間槽機制、衛星拓撲 API 均就位 |

---

## 8. Outputs 

| 測試場景 | Log 位置 |
|---------|---------|
| Baseline 矩陣（4 模式 × 2 時長）| `Outputs/Baseline/validation_baseline.md` |
| 6 pathType 全覆蓋 | `Outputs/E2E-ReturnFeeder/*.md` |
| TW→SF 300s（最終驗證）| `Outputs/E2E-ReturnFeeder/gw2gw_e2e_TW_US_300s_clean.log` |
| TW→JP with background load | `Outputs/E2E-ReturnFeeder/gw2gw_e2e_load.md` |
| sat2sat 8-hop（SAT0→SAT33）| `Outputs/E2E-ReturnFeeder/sat2sat_s0s33_120s.log` |
| gw2ut_e2e 三段（TW→SF）| `Outputs/E2E-ReturnFeeder/gw2ut_e2e_TW_SF_180s.log` |
| gw2gw_e2e bgload 對比 | `Outputs/E2E-ReturnFeeder/gw2gw_e2e_TW_US_300s_bgload.log` |
