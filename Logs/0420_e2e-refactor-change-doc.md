# 改動文件 — test-iridium-e2e.cc 重構
**日期：** 2026-04-20
**異動檔案：**
- `Topology & ISL Routing/Codes/test-iridium-e2e.cc`
- `Topology & ISL Routing/Codes/isl-graph.cc`

---

## 總覽

本次重構將舊有的「segment flag 手動模式」（`--enableFeederlink / --enableIsl / --enableServicelink`）替換為 **pathType 驅動的 ObsScope 系統**。每個 `pathType` 自動決定需要觀測哪些 link 段、如何預算 routing，以及各層通過條件。新增 `PrintE2EFinalVerdict` 輸出分層 PASS/FAIL 結果，使模擬驗證結果機器可讀。

---

## 1. CLI 參數異動

| 舊參數 | 新參數 | 原因 |
|--------|--------|------|
| `--mode` | `--pathType` | 與 `E2EConfig::pathType` 命名一致 |
| `--trafficProfile` | （已移除） | legacy profile 系統全部移除 |
| `--enableFeederlink` | （已移除） | 由 pathType 隱含的 ObsScope 取代 |
| `--enableIsl` | （已移除） | 由 pathType 隱含的 ObsScope 取代 |
| `--enableServicelink` | （已移除） | 由 pathType 隱含的 ObsScope 取代 |
| （新增） | `--obsDebug` | 開啟 GW device 列表與 rx_hits 詳細輸出 |
| （新增） | `--satStats` | 開啟 SNS3 原生 SatStatsHelperContainer scatter 檔輸出 |

---

## 2. ObsScope 機制（新增）

### 結構體：`ObsScope`
```cpp
struct ObsScope {
    bool activeFeeder{false};
    bool activeService{false};
    bool activeIsl{false};
    std::set<std::string> feederKeys;
    std::set<std::string> serviceKeys;
    std::set<std::string> islKeys;
};
static ObsScope g_obsScope;
```

**設計目的：** 根據 `pathType` 明確定義哪些 link key 在觀測範圍內，供警示觸發與 verdict 計算使用。避免跨 pathType 的誤報（例如 `gw2gw_e2e` 在 REGENERATION_NETWORK 模式下 feeder_dn 架構上不可見，不應觸發 feeder PHY 警示）。

### 相關函式：
- `ConfigureObsScope(routingMgr, cfg, numSlots)` — 在 `ConfigureRoutingCase()` 之後呼叫，設定 slot 0 的 scope。
- `UpdateObsScopeForSlot(routingMgr, cfg, k)` — 排程於每個 slot 邊界（k=1..numSlots-1），在 `ScheduleRoutingUpdates()` 之後入隊，確保 scope 與最新 routing table 同步。
- `IsObsKeyInScope(linkType, key)` — 在 `CheckAndAlertObs()` 開頭呼叫，若不在 scope 內則直接回傳，不觸發警示。

### main() 執行順序（新增部分）：
```
ConfigureRoutingCase()       // routing tables 建立完成
ConfigureObsScope()          // slot-0 scope 設定
InstallE2ETraffic()          // 在 routing 完成後安裝流量
ScheduleRoutingUpdates()     // slot 1..N 排程
for k=1..numSlots-1:         // scope 更新與 ApplyRoutingTable 同一時間點
    Schedule(k * slotInterval, UpdateObsScopeForSlot)
```
NS-3 FIFO scheduler 保證 `UpdateObsScopeForSlot` 在同一時間點的 `ApplyRoutingTable` 之後執行（後入隊即後執行）。

---

## 3. Key 輔助函式（新增）

| 函式 | 回傳值 | 用途 |
|------|--------|------|
| `MakeSatKey(satId)` | `"sat<id>"` | feeder/service link 統計 map 的 key |
| `MakeGwKey(gwId)` | `"gw<id>"` | SAT→GW Rx 觀測 key |
| `MakeGwTxKey(gwId)` | `"gwtx<id>"` | GW→SAT Tx 觀測 key（與 MakeGwKey 不同） |
| `MakeIslKey(src, dst)` | `"<src>-<dst>"` | ISL 統計 map 的 key |
| `SumScopedRxPkts(linkType)` | `uint64_t` | 加總指定 linkType 所有 scoped key 的 rxPkts |

---

## 4. GW 端 Feeder 觀測 Callback（新增）

### `GatewayRxFeederCb(key, pkt, addr)`
- **鏈路方向：** feeder_dn（SAT→GW 下行，回程方向）
- **掛載點：** GW 節點的 `SatNetDevice::Rx`
- **使用 pathType：** `sat2gw`

### `GatewayTxFeederCb(key, pkt)`
- **鏈路方向：** feeder_up（GW→SAT 上行，TX 側）
- **掛載點：** GW 節點 MAC 層 send
- **注意：** TX 側無 drop model，因此 `rxPkts == txPkts`

### `GatewayDeviceRxDebugCb(key, pkt, addr)`
- **用途：** Debug 專用的 per-device 命中計數器
- **啟用條件：** `--obsDebug=1`
- **輸出時機：** `PrintObsFinalSummary()` 中當 `g_obsDebug` 為 true 時印出

---

## 5. Gw2GwAppDeliveryStats（新增）

```cpp
struct Gw2GwAppDeliveryStats {
    bool     installed{false};
    bool     reported{false};
    uint32_t srcGwId{0}, dstGwId{0};
    Ipv4Address srcAddr, dstAddr;
    uint64_t rxBytes{0};
    uint64_t estPkts{0};
};
static Gw2GwAppDeliveryStats g_gw2gwDelivery;
```

**設計目的：** 針對 `gw2gw_e2e` 追蹤 application 層的封包實際送達情況。`PrintE2EFinalVerdict` 的 PACKET_LAYER verdict 讀取 `g_gw2gwDelivery.rxBytes > 0` 來確認 end-to-end 封包確實送達，不依賴 feeder PHY trace。

---

## 6. ConnectLinkObserverTraces() 介面異動

**異動前：**
```cpp
static void ConnectLinkObserverTraces();
```

**異動後：**
```cpp
static void ConnectLinkObserverTraces(bool useOrbiterFeeder, bool useGwFeeder);
```

**呼叫端邏輯（main() 中）：**
```cpp
bool useOrbiterFeeder = (e2eCfg.pathType != "sat2gw" && e2eCfg.pathType != "gw2gw_e2e");
bool useGwFeeder      = (e2eCfg.pathType == "sat2gw");
ConnectLinkObserverTraces(useOrbiterFeeder, useGwFeeder);
```

**設計原因：** `gw2gw_e2e` 在 REGENERATION_NETWORK 模式下，封包在衛星端被重新產生，原始 GW 端 header 不可見，`SatOrbiterNetDevice::RxFeeder` 對此段流量無法觀測。若強行掛載，rxPkts 恆為 0，導致 verdict 產生誤報 FAIL。因此 `gw2gw_e2e` 改用 ROUTING_LAYER + ISL_LAYER + PACKET_LAYER 三層 verdict 取代 feeder PHY 驗證。

---

## 7. ConfigureRoutingCase() 補齊 PathType Routing 初始化

三個 pathType 之前在 `ConfigureRoutingCase()` 中直接 return，沒有呼叫必要的 routing 設定函式，導致 `ConfigureObsScope` 拿不到 route 資料：

| PathType | 新增呼叫 |
|----------|----------|
| `gw2sat` | `SetGwElevationThreshold`, `AddGatewayOrAbort`, `PrecomputeGwRoutes` |
| `sat2gw` | `SetGwElevationThreshold`, `AddGatewayOrAbort`, `PrecomputeGwRoutes` |
| `sat2ut` | `SetGwElevationThreshold`, `AddGatewayOrAbort`, `AddUserTerminal`, `AddGwUtPair`, `PrecomputeGwUtRoutes` |

**影響：** 若未補齊，ObsScope 的 scope keys 為空，verdict 輸出全部為 not_applicable，無法正常驗證。

---

## 8. E2EPlan → PathTypePlan

| 異動前 | 異動後 |
|--------|--------|
| `E2EExecutionPlan` | `PathTypePlan` |
| `BuildE2EPlan(cfg)` | `BuildPathTypePlan(cfg)` |
| `ApplyLegacySegmentDefaults(cfg)` | （已移除） |
| `e2eCfg.legacyProfile` | （已移除） |
| `e2eCfg.explicitSegments` | （已移除） |

legacy「explicit segment 覆蓋」路徑全部移除。`pathType` 成為唯一決定哪些 segment 啟用的依據。

---

## 9. PrintE2EFinalVerdict()（新增）

**呼叫位置：** `main()` 中 `PrintObsFinalSummary()` 之後。

**輸出格式：**
```
=== E2E PathType Verdict ===
pathType=gw2gw_e2e
[ROUTING_LAYER] PASS | validSlots=63/63 gwSrc=0 gwDst=2
[ISL_LAYER]     PASS | scopedLinks=4 scopedRxPkts=1820
[PACKET_LAYER]  PASS | appInstalled=1 reported=1 rxBytes=92400 estPkts=77
[FEEDER_PHY_LAYER] not_applicable | gw2gw_e2e verdict intentionally does not use feeder PHY counters
```

**各 pathType 分層 verdict 對應：**

| pathType | FEEDER | SERVICE | ISL | PACKET |
|----------|--------|---------|-----|--------|
| `gw2gw_e2e` | not_applicable | not_applicable | 觀測 | 觀測 |
| `gw2ut_e2e` | 觀測 | 觀測 | 觀測 | — |
| `sat2sat` | 觀測 | — | 觀測 | — |
| `gw2sat` | 觀測 | — | — | — |
| `sat2gw` | 觀測 | — | — | — |
| `sat2ut` | — | 觀測 | — | — |

### `PrintLayerVerdict(layer, applicable, pass, detail)`
輸出單行 verdict。`applicable=false` 時無論 pass 值均輸出 `not_applicable`。

---

## 10. 警示 State Machine 修正

**問題：** `g_prevObsDropRate` 原本以 `key` 為索引，若 feeder key 與 service key 字串相同則發生衝突，造成警示狀態機誤判。

**修正：** 改以 `linkType + ":" + key` 為索引（例如 `"feeder:sat3"` vs `"service:sat3"`）。

同時，`CheckAndAlertObs()` 在函式開頭呼叫 `IsObsKeyInScope()`，若 key 不在 scope 內則立即 return，不進入 drop rate 或 throughput 比對邏輯。

---

## 11. SNS3 統計標籤預設啟用

```cpp
Config::SetDefault("ns3::SatPhy::EnableStatisticsTags", BooleanValue(true));
Config::SetDefault("ns3::SatNetDevice::EnableStatisticsTags", BooleanValue(true));
```

**原因：** `SatStatsHelperContainer` 需要這兩個 flag 為 true 才能接收 trace 資料。先前預設為 false，導致 `--satStats=1` 輸出的 scatter 檔全部為空。

---

## 12. 原生統計輸出（--satStats）

`--satStats=1` 時，透過 `SatStatsHelperContainer` 註冊以下 scatter 檔輸出：

| 方向 | 觀測單位 | 指標 |
|------|----------|------|
| FWD feeder（GW→SAT） | per-sat、per-gw | Dev 吞吐量 |
| RTN feeder（SAT→GW） | per-sat、per-gw | Dev 吞吐量 |
| FWD user（SAT→UT） | per-sat、per-ut | Dev 吞吐量 |
| RTN user（UT→SAT） | per-sat、per-ut | Dev 吞吐量 |

輸出檔案前綴：`<pathType>-stats-*`（例如 `gw2gw_e2e-stats-fwd-feeder-dev-throughput-per-gw.dat`）。

**使用時機：** 當 `gw2gw_e2e` 的 PACKET_LAYER 顯示 FAIL 時，搭配 `--satStats=1` 確認 feeder_dn 在 SNS3 內部是否有流量，協助診斷 REGENERATION_NETWORK 架構下的封包追蹤盲區。

---

## 13. 新增全域狀態變數

| 變數名稱 | 型別 | 用途 |
|----------|------|------|
| `g_gwDeviceRxHits` | `map<string,uint64_t>` | per-device debug 命中計數 |
| `g_gwDeviceTypes` | `map<string,string>` | device 型別標籤（debug 輸出用） |
| `g_obsDebug` | `bool` | 由 `--obsDebug=1` 開啟 |
| `g_totalOrbiterFeederRxCalls` | `uint64_t` | 不論 scope 的 OrbiterRxFeeder 總呼叫次數，用於診斷 trace 是否真的有觸發 |
| `g_prevObsThroughputKbps` | `map<string,double>` | throughput 警示 state machine 前一次數值 |
| `g_gw2gwDelivery` | `Gw2GwAppDeliveryStats` | `gw2gw_e2e` application 層送達追蹤 |
| `g_obsScope` | `ObsScope` | 目前作用中的觀測範圍定義（每個 slot 更新） |

---

## 14. 輸入驗證修正

| 異動前 | 異動後 |
|--------|--------|
| `simTime >= 0.0`（允許 0） | `simTime > 0.0` |
| 無 start/stop 驗證 | 新增 `NS_ABORT_MSG_IF(trafficStart >= resolvedStop, ...)` |

---

## 15. isl-graph.cc：PrintGwUtRouteReport()

**異動：** 對有效 GW 但無 ISL 中繼路徑的 route，將 `"N/A"` 改為 `"no ISL hop"`。

**位置：** `isl-graph.cc` 約第 1847 行

**原因：** `"N/A"` 語義不清（可能是 route 無效，也可能是無 ISL）。`"no ISL hop"` 明確表示此 route 是單跳直連 feeder 路徑，非 ISL 中繼。

---

## main() 執行順序（重構後）

```
cmd.Parse()
g_obsCfg / g_obsDebug / satStats 套用
NS_ABORT_MSG_IF 輸入驗證
NormalizePathType()
BuildPathTypePlan()                ← 取代 BuildE2EPlan
SNS3 環境設定（SatHelper、scenario）
[選用] SatStatsHelperContainer 註冊（--satStats）
ConnectIslDropTrace()
g_obsLog.open()
ConnectLinkObserverTraces(useOrbiterFeeder, useGwFeeder)
Schedule TakeObsSnapshot
[選用] RBDC trace
routingMgr 建立與初始化
routingMgr->PrecomputeAllTables()
ConfigureRoutingCase()             ← routing tables + GW/UT 註冊
ConfigureObsScope()                ← slot-0 scope 設定
PrintE2ERunBanner()
InstallE2ETraffic()                ← routing 完成後才安裝流量
routingMgr->ScheduleRoutingUpdates()
for k=1..numSlots-1:
    Schedule(k*slotInterval, UpdateObsScopeForSlot)
simHelper->RunSimulation()
PrintObsFinalSummary()
PrintE2EFinalVerdict()             ← 新增
routingMgr->PrintStats()
routingMgr->PrintLoadStats()
PrintIslDropStats()
g_obsLog.close()
Simulator::Destroy()
```

---

## 測試指令

### gw2gw_e2e 基本測試
```bash
./ns3 run "test-iridium-e2e \
  --pathType=gw2gw_e2e \
  --gwSrc=0 --gwDst=2 \
  --simTime=120 --slotInterval=10 \
  --obsIntervalSec=10" \
  2>&1 | tee gw2gw_e2e_120s.log
```

### gw2gw_e2e 搭配原生統計交叉驗證
```bash
./ns3 run "test-iridium-e2e \
  --pathType=gw2gw_e2e \
  --gwSrc=0 --gwDst=2 \
  --simTime=120 --slotInterval=10 \
  --satStats=1" \
  2>&1 | tee gw2gw_e2e_satStats.log
```

### sat2gw 搭配 GW feeder 觀測
```bash
./ns3 run "test-iridium-e2e \
  --pathType=sat2gw \
  --gwId=0 \
  --simTime=120 --slotInterval=10" \
  2>&1 | tee sat2gw_120s.log
```

---

## 預期輸出（gw2gw_e2e）

```
=== E2E PathType Verdict ===
pathType=gw2gw_e2e
[ROUTING_LAYER] PASS | validSlots=N/N gwSrc=0 gwDst=2
[ISL_LAYER]     PASS | scopedLinks=K scopedRxPkts=M
[PACKET_LAYER]  PASS | appInstalled=1 reported=1 rxBytes=R estPkts=P
[FEEDER_PHY_LAYER] not_applicable | gw2gw_e2e verdict intentionally does not use feeder PHY counters
```

若 PACKET_LAYER 顯示 FAIL（`rxBytes=0`）：表示 routing 層有效路由存在，但 application 層封包未實際送達，需檢查 data-plane 設定。
