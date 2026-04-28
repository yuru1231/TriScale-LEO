# Layer 1 — Topology & ISL Routing

> **結案狀態**：Layer 1 closed — 2026-04-23
> **最終 Build**：`2026-04-17-fullobs-v1`
> **最終驗證**：`Topology & ISL Routing/Outputs/audit/final/`（6 種 pathType，全部 PASS）
> **Layer 2 所需介面**：`IslRoutingManager`、時間槽機制、衛星拓撲 API 均就位

---

## 1. 目標與系統組成

### 1.1 目標

在 SNS3 的 Iridium-66 LEO 星座場景中，實作基於 ISL 的時間槽路由系統：

- **離線預計算**：利用 LEO 軌道可預測性，模擬前依各時槽計算 ISL 拓樸與最短路徑路由表
- **Runtime 套用**：每隔 `slotInterval` 秒將對應時槽的路由表寫入 Arbiter，反映衛星幾何變化
- **Load-aware 重算**：以 ISL queue bytes 換算 queue delay proxy，並用 EMA 平滑；負載變化超過門檻時對受影響來源節點執行局部 Dijkstra

### 1.2 系統組成

| 元件 | 檔案 | 職責 |
|------|------|------|
| `IslRoutingManager` | `contrib/satellite/helper/isl-graph.h/.cc` | 路由計算、Arbiter 寫入、load-aware 動態路由 |
| E2E 測試框架 | `scratch/test-iridium-e2e.cc` | 6 種 pathType 的流量安裝、鏈路觀測、Verdict 輸出 |
| FT Visibility Filter | `contrib/satellite/helper/ft-filter.h/.cc` | 依仰角過濾衛星可見性（Layer 2 呼叫用） |

---

## 2. IslRoutingManager

### 2.1 初始化與預計算流程

```
isls.txt
   │
   ▼
LoadISLDefs()
   │  讀入 132 條 ISL 定義，建立 satellite→ifIndex 反查表
   ▼
InitOrbiterDevices()
   │  快取 66 顆衛星的 Node / SatOrbiterNetDevice / Arbiter
   ▼
PrecomputeAllTables()
   │  for each slot k:
   │    GetPositionsAt(τ_k)     → SGP4 查詢 66 顆衛星位置
   │    BuildISLGraph(pos)       → 距離 > 5000km 過濾，建鄰接表
   │    ComputeRoutesForSrc(g)   → 66 × Dijkstra，純傳播 cost
   │    m_tables[k] = result
   ▼
ScheduleRoutingUpdates()
   │  排入 N 個 NS3 事件
   ▼
[Simulator::Run]
   │
   ▼
ApplyRoutingTable(slotIndex)   ← 每 slotInterval 秒觸發
   │  slot 0: 直接套用預計算表
   │  slot k>0:
   │    UpdateLoadCosts()         → 佇列 byte→delay 換算 + EMA
   │    HasSignificantChange()    → 超過 ChangeThreshold + CooldownSeconds
   │    RecomputeAffectedRoutes() → 局部 Dijkstra（只算受影響 source）
   │    ClearNextHopEntries() + AddNextHopEntry() → 寫入 Arbiter
   ▼
PrintStats() / PrintLoadStats() / PrintIslDropStats()
```

### 2.2 ISL 連接條件

| 條件 | 數值 |
|------|------|
| 距離上限 | 5000 km（超過此值的邊在建圖時排除） |
| 仰角門檻 | 不適用（僅用於 UT/GW 鏈路） |
| Beam 狀態 | 假設所有 beam 皆為可用 |

### 2.3 Cost Function

```
total_cost = propagation_cost + load_cost

propagation_cost = distance_ab / c             （離線算好，固定）
load_cost        = queue_bytes * 8 / bandwidth  （執行期讀取，EMA 平滑）
```

| 常數 | 數值 | 說明 |
|------|------|------|
| `c` | 3×10⁸ m/s | 光速，傳播延遲基準 |
| bandwidth | 10 Mbps | ISL 鏈路速率（`IslLinkRateBps` Attribute） |
| 假設封包大小 | 1500 bytes | `GetLinkQueueDelay()` 中 load cost 換算基準 |
| α, β weight | 各為 1.0（隱含） | `total_cost = propCost + loadCost`，等權無獨立欄位 |

第一階段（precompute）`load_cost` 初始為 0，cost 等同純傳播延遲。

### 2.4 Attributes（NS3 Config）

| Attribute | 型別 | 預設值 | 說明 |
|-----------|------|--------|------|
| `NumSatellites` | uint32 | 66 | 星座衛星總數 |
| `NumTimeSlots` | uint32 | 10 | 預計算時槽數 |
| `TimeSlotInterval` | double | 60.0 | 每槽間隔（秒） |
| `IslMaxDistanceKm` | double | 5000.0 | ISL 啟用距離門檻（km） |
| `IslsFilePath` | string | "" | isls.txt 完整路徑 |
| `EmaAlpha` | double | 0.3 | EMA 新樣本權重 |
| `ChangeThreshold` | double | 0.1 | 觸發重算的 load cost 相對變化門檻 |
| `CooldownSeconds` | double | 30.0 | 兩次重算最短間隔（秒） |
| `IslLinkRateBps` | double | 10e6 | ISL 鏈路速率（佇列 delay 換算用） |

#### 參數調整的連動效應

| 調整項目 | 直接效果 | 注意事項 |
|---------|---------|---------|
| `EmaAlpha` ↑ | load cost 對瞬時壅塞更敏感，切換更快 | 與 `CooldownSeconds` 搭配；單獨調高易造成震盪 |
| `ChangeThreshold` ↓ | 更容易觸發 `RecomputeAffectedRoutes` | 計算開銷增加，建議搭配 `CooldownSeconds` 限制頻率 |
| `CooldownSeconds` ↑ | 降低 recompute 頻率，穩定性提升 | ISL 失效後最長延遲 `CooldownSeconds` 才能恢復 |
| `NumTimeSlots` ↑ | 預計算解析度提升 | 記憶體與初始化時間線性增加 |
| `IslMaxDistanceKm` ↑ | 連通性增強，可能建立長延遲 ISL | 超過 ~5500 km 後 Iridium 星座中可能出現不穩定跨面連結 |

### 2.5 公開方法

#### Lifecycle

| 方法 | 說明 |
|------|------|
| `Initialize(islsFilePath)` | 讀 ISL 定義、快取衛星裝置、初始化 load cost 陣列 |
| `PrecomputeAllTables()` | 離線計算所有時槽路由表，存入 `m_tables` |
| `ScheduleRoutingUpdates()` | 排入 N 個 NS3 排程事件 |
| `ApplyRoutingTable(slotIndex)` | 更新 Arbiter（含 load-aware 重算邏輯） |

#### 路由查詢 / 診斷

| 方法 | 說明 |
|------|------|
| `GetGwRoute(srcGwId, dstGwId, slot)` | 取得 GW→GW 路由（含入口 / 出口衛星與路徑資訊） |
| `GetGwUtRoute(gwId, utId, slot)` | 取得 GW→UT 路由（含 entry / serving sat） |
| `TracePath(src, dst, slot)` | 重建 src→dst 完整跳數序列 |
| `GetGwVisibleSats(gwId, slot)` | 取得 GW 在指定時槽可見的衛星集合 |
| `BlockISL(a, b)` / `UnblockISL(a, b)` | 暫時標記 ISL 不可用（繞路測試用） |

#### 統計輸出

| 方法 | 說明 |
|------|------|
| `PrintStats()` | 各槽 apply / recompute 執行時間 |
| `PrintLoadStats()` | 各 ISL 最終 EMA load cost（ms） |
| `PrintRouteReport(pairs)` | sat2sat 每槽完整路徑 |
| `PrintGwRouteReport()` | gw2gw 每槽 entry / ISL_path / exit / isl_cost |
| `PrintGwUtRouteReport()` | gw2ut 每槽 entry / ISL_path / serving / isl_cost |

### 2.6 初始化範例

```cpp
Ptr<IslRoutingManager> mgr = CreateObject<IslRoutingManager>();
mgr->SetAttribute("NumSatellites",    UintegerValue(66));
mgr->SetAttribute("NumTimeSlots",     UintegerValue(numSlots));
mgr->SetAttribute("TimeSlotInterval", DoubleValue(slotInterval));
mgr->SetAttribute("IslMaxDistanceKm", DoubleValue(5000.0));
mgr->SetAttribute("IslsFilePath",     StringValue(islsFilePath));
mgr->SetAttribute("EmaAlpha",         DoubleValue(0.3));
mgr->SetAttribute("ChangeThreshold",  DoubleValue(0.1));
mgr->SetAttribute("CooldownSeconds",  DoubleValue(slotInterval / 2.0));
mgr->SetAttribute("IslLinkRateBps",   DoubleValue(10.0e6));

// 必須在 CreateSatScenario() 之後呼叫
mgr->Initialize(islsFilePath);
mgr->PrecomputeAllTables();
mgr->ScheduleRoutingUpdates();
```

### 2.7 Load-aware 動態路由（行為說明）

每次 `ApplyRoutingTable(k)`（slot > 0）執行：

1. `UpdateLoadCosts()`：讀各 ISL queue bytes，依 `IslLinkRateBps` 換算為 queue delay proxy，以 EMA 平滑更新內部 load cost
2. `HasSignificantChange()`：任一 ISL 方向 load cost 相對變化超過 `ChangeThreshold=0.1`，且距上次重算超過 `CooldownSeconds=30s`，即判定需要重算
3. `RecomputeAffectedRoutes()`：定位受影響 ISL 邊，透過 `m_islSources[edgeIdx]` 找出受影響 source，對這些 source 重跑 Dijkstra（`BuildISLGraphWithLoad`，cost = propagation + load）

#### 執行期震盪抑制

| 機制 | 設定 |
|------|------|
| EMA | `smoothed = (1-α) × previous + α × current`，α=0.3 |
| Hysteresis | 新路徑 cost 低於現有路徑 δ（建議 0.5×avg_prop）允許切換 |
| Cooldown | `CooldownSeconds=30s`（預設），切換後不再重算 |

**驗證結論**：EMA load cost 計算正確（最高 ~0.84ms），但遠小於傳播延遲（~44ms，約 2%）。`HasSignificantChange` 與局部重算有觸發，但 audit/final 條件下未觀察到由負載主導的路由切換；若需驗證 load-driven 切換，須設計 ISL 飽和壓測場景。

---

## 3. SNS3 介面

### 3.1 相關類別

| 類別 | 用途 |
|------|------|
| `PointToPointIslNetDevice` | ISL device，內含 `DropTailQueue<Packet>` |
| `SatOrbiterNetDevice` | 衛星主 device，管理所有 ISL device 與 Arbiter |
| `SatIslArbiterUnicast` | 實作類，內部為 `map<destSatId, islInterfaceIndex>` |
| `SatSGP4MobilityModel` | 軌道位置計算，需新增 `GetGeoPositionAt(Time t)` |

### 3.2 InitOrbiterDevices()（在 Simulator::Run() 前呼叫）

```cpp
// 預先建立並安裝 66 個 arbiter，快取供排程使用
for (uint32_t i = 0; i < 66; i++) {
    Ptr<Node> sat = m_orbNodes[i];
    Ptr<SatOrbiterNetDevice> orbDev =
        DynamicCast<SatOrbiterNetDevice>(sat->GetDevice(0));
    Ptr<SatIslArbiterUnicast> arbiter =
        CreateObject<SatIslArbiterUnicast>(sat);   // 在有效上下文建立
    orbDev->SetArbiter(arbiter);
    m_arbiters[i] = arbiter;                       // 快取
}
```

> 排程內不可呼叫 `CreateObject<SatIslArbiterUnicast>()`，缺少有效 Node context 會觸發 NS_FATAL。
> 詳見 `Decisions/03_Arbiter lifecycle management.md`。

### 3.3 ApplyRoutingTable()（排程執行時呼叫）

```cpp
Ptr<SatIslArbiterUnicast> arbiter = m_arbiters[satId];
arbiter->ClearNextHopEntries();
for (auto& entry : routingTable[satId]) {
    arbiter->AddNextHopEntry(entry.destSatId, entry.islIfIndexOnA);
}
```

### 3.4 BuildISLGraph() 效能注意事項

位置計算必須在迴圈外預先快取，不可在 ISL 邊迴圈內逐次呼叫：

```cpp
// 正確：一次性快取所有衛星位置
std::vector<Vector> pos(m_numSatellites);
for (uint32_t i = 0; i < m_numSatellites; i++)
    pos[i] = m_orbNodes[i]->GetObject<SatSGP4MobilityModel>()
                           ->GetGeoPositionAt(tau_k).ToVector();
// 之後 ISL 邊迴圈直接用 pos[a], pos[b]
```

`PrecomputeAllTables` 以滾動快取傳遞 `graphNext`，只建 10 次圖（錯誤做法：10 個 slot 獨立呼叫實際建圖 19 次）。

### 3.5 UpdateLoadCosts() 實作

```cpp
std::vector<Ptr<PointToPointIslNetDevice>> islDevs =
    orbDev->GetIslsNetDevices();
for (uint32_t i = 0; i < islDevs.size(); i++) {
    uint32_t queuePackets = islDevs[i]->GetQueue()->GetNPackets();
    // 套用 EMA，計算 load_cost
}
```

### 3.6 ifIndex 對應方式

不依賴 `isls.txt` 順序，改用 peer nodeId 查 vector index：

```cpp
for (uint32_t j = 0; j < islDevs.size(); j++) {
    Ptr<Node> peer = islDevs[j]->GetDestinationNode();
    if (peer) peerNodeIdToIfIdx[i][peer->GetId()] = j;
}
```

---

## 4. E2E 測試框架（test-iridium-e2e.cc）

### 4.1 pathType 系統

E2E 框架以 `--pathType` 為唯一入口，自動決定：流量安裝方式、觀測 scope、Verdict 輸出。

| pathType | 語意 | 觀測段 | 主 Verdict |
|----------|------|-------|-----------|
| `gw2sat` | GW→SAT feeder 上行 | feeder（衛星側 RxFeeder） | FEEDER_LAYER |
| `sat2gw` | SAT→GW feeder 下行 | feeder（GW 側 SatNetDevice::Rx） | FEEDER_LAYER |
| `sat2sat` | SAT→SAT ISL 骨幹 | ISL 路徑上各 hop | ISL_LAYER |
| `sat2ut` | SAT→UT service 下行 | service（SAT 側 + UT 側） | SERVICE_LAYER |
| `gw2ut_e2e` | GW→UT 端到端（feeder + ISL + service） | feeder + ISL（若有）+ service | FEEDER_LAYER + SERVICE_LAYER |
| `gw2gw_e2e` | GW→GW 端到端（ISL 骨幹 + GW_user 交付） | 路由驗證 + scoped ISL + PacketSink Rx | ROUTING_LAYER + ISL_LAYER + PACKET_LAYER |

### 4.2 feeder 觀測來源切換

`ConnectLinkObserverTraces()` 依 pathType 決定觀測來源：

```cpp
bool useOrbiterFeeder = (pathType != "sat2gw" && pathType != "gw2gw_e2e");
bool useGwFeeder      = (pathType == "sat2gw");
bool useUtService     = (pathType == "sat2ut" || pathType == "gw2ut_e2e");
```

| pathType | feederSource（log 輸出） | 說明 |
|----------|------------------------|------|
| `gw2sat`, `gw2ut_e2e` | `orbiter_rxfeeder` | 衛星端 `SatOrbiterNetDevice::RxFeeder` |
| `sat2gw` | `gw_return_rx` | GW 端 `SatNetDevice::Rx`（return feeder） |
| `gw2gw_e2e` | `none` | feeder PHY bypass（見 Section 7.1） |
| `sat2sat`, `sat2ut` | `none` | 不觀測 feeder |

### 4.3 ObsScope（觀測範圍管控）

依 pathType 限制哪些 key 進入統計，避免大量零行輸出：

```cpp
struct ObsScope {
    std::set<std::string> feederKeys;
    std::set<std::string> serviceKeys;
    std::set<std::string> islKeys;
};
static ObsScope g_obsScope;        // 即時觀測（TakeObsSnapshot 使用）
static ObsScope g_obsVerdictScope; // 最終 PASS/FAIL 判定
```

`ConfigureObsScope()` 依 pathType 初始填入 scope；每 slot 切換後由 `UpdateObsScopeForSlot()` 重算：

| pathType | feederKeys | serviceKeys | islKeys |
|----------|-----------|------------|---------|
| `gw2sat` | entry sat（per slot GetGwVisibleSats） | — | — |
| `sat2gw` | GW key（固定） | — | — |
| `sat2sat` | — | — | TracePath 展開邊集合 |
| `sat2ut` | — | `ut<trafficUtUserId>`（固定） | — |
| `gw2ut_e2e` | entry sat | `ut<trafficUtUserId>` | 路徑邊集合（entry==serving 時為空） |
| `gw2gw_e2e` | — | — | 正反向 GW-GW route 的路徑邊集合（per slot 更新） |

### 4.4 Verdict 層級

`PrintE2EFinalVerdict()` 依 pathType 輸出各層：

| Verdict | 判定條件 | 適用 pathType |
|---------|---------|--------------|
| `FEEDER_LAYER` | `scopedRxPkts > 0`（scope 內 feeder key 有收包） | gw2sat, sat2gw, gw2ut_e2e |
| `SERVICE_LAYER` | `utRxPkts > 0`（UT-side 有收包） | sat2ut, gw2ut_e2e |
| `ISL_LAYER` | `scopedLinks > 0 && scopedRxPkts > 0` | sat2sat, gw2gw_e2e |
| `ISL_LAYER not_applicable` | `gw2ut_e2e` 且 entry==serving（無 ISL hop） | gw2ut_e2e |
| `ROUTING_LAYER` | `validSlots == numSlots`（全槽路由均可解析） | gw2gw_e2e |
| `PACKET_LAYER` | `PacketSink::Rx traceRxBytes > 0` | gw2gw_e2e |

### 4.5 UT Endpoint Selection（trafficUtUserId）

`sat2ut` / `gw2ut_e2e` 的 UT 端 routing ID 與 SNS3 scenario UT user index 不同：

- `cfg.utId`：邏輯 UT routing ID（`IslRoutingManager` 使用）
- `cfg.trafficUtUserId`：SNS3 scenario 中真正的 UT user node index（流量安裝 / OBS key / endpoint probe 使用）

`ResolveTrafficUtUserId()` 自動解析：

```
1. GetGwUtRoute(gwId, utId, first valid slot) → servingSatId
2. 掃描 SatTopology::GetUtUserNodes()
3. 對每個 user node: physicalUtNode = topo->GetUtNode(utUser)
4. GetUtSatId(physicalUtNode) == servingSatId → trafficUtUserId = i
```

輸出：
```
[UT_SELECT] logicalUtId=0 routeServingSat=15 trafficUtUserId=22 requestedUtUserId=0
```

### 4.6 Endpoint Probe（多層接收驗證）

安裝於 physical UT / GW node 上，逐層確認封包抵達：

| 層 | trace source | 意義 |
|----|-------------|------|
| PHY | `SatPhy::Rx` | 實體層接收 |
| MAC | `SatMac::Rx` | MAC 層接收 |
| Dev | `SatNetDevice::Rx` | NetDevice 層接收 |
| App | `PacketSink::Rx` | 應用層接收（port=9100，診斷用） |

| interpretation 值 | 意義 |
|-------------------|------|
| `device_rx_observed_probe_app_idle` | Dev 有收包，App sink 安裝但主流量未打到 port=9100（正常） |
| `no_endpoint_observed` | Dev 未收包（異常） |
| `device_rx_observed_app_not_installed` | Dev 有收包但 App sink 未安裝 |

---

## 5. 核心資料結構

### 5.1 isl-graph.h — 路由層

| 結構 | 說明 |
|------|------|
| `ISLEdge` | `{nodeB, propagation_cost, islIfIndexOnA, islIfIndexOnB}` |
| `ISLDef` | `{nodeA, nodeB}` 靜態 ISL 定義 |
| `RouteEntry` | `{destSatId, nextHopSatId, islIfIndexOnA, cost}` |
| `ISLGraph` | `vector<vector<ISLEdge>>` 66 節點鄰接表 |
| `RoutingTable` | `vector<vector<RouteEntry>>` 66 顆衛星各自路由表 |
| `SlotStats` | `{slotIndex, simTimeSec, applyWallMs, recomputeWallMs, recomputedSources, significantChange}` |
| `GwDef` | `{gwId, latDeg, lonDeg, name}` |
| `GwToGwRoute` | `{srcGwId, dstGwId, entrySatId, exitSatId, satPath, islCost, valid}` |
| `UtDef` | `{utId, latDeg, lonDeg, name}` |
| `GwToUtRoute` | `{gwId, utId, entrySatId, servingSatId, satPath, islCost, valid}` |

### 5.2 test-iridium-e2e.cc — E2E 框架

```cpp
struct E2EConfig {
    std::string pathType;         // gw2sat / sat2gw / sat2sat / sat2ut / gw2ut_e2e / gw2gw_e2e
    uint32_t gwId, gwSrc, gwDst;
    uint32_t utId;                // 邏輯 UT routing ID
    uint32_t trafficUtUserId;     // SNS3 scenario UT user index（自動解析）
    bool     trafficUtUserIdResolved;
    uint32_t satSrc, satDst;
    double   simTimeSec, utLatDeg, utLonDeg;
    std::string utName;
};

struct SegLinkStats {
    uint64_t rxPkts, rxBytes, dropPkts;
    double   sumDelayMs;
    double   DropRate() const;
    double   AvgDelayMs() const;
    double   WindowThroughputKbps(double nowSec) const;
};

struct TrafficConfig {
    bool     enableFwd{true}, enableRtn{true};
    uint32_t fwdIntervalMs{100}, rtnIntervalMs{500};
    uint32_t fwdPktBytes{1500}, rtnPktBytes{512};
    double   startSec{1.0}, stopSec{0.0};
};
```

Key factories（統一 key 格式）：

| 函式 | 回傳格式 | 用途 |
|------|---------|------|
| `MakeSatKey(satId)` | `"sat<N>"` | orbiter feeder / service obs |
| `MakeGwKey(gwId)` | `"gw<N>"` | GW-side return feeder obs |
| `MakeUtKey(utId)` | `"ut<N>"` | UT-side service obs |
| `MakeIslKey(src, dst)` | `"<src>-<dst>"` | ISL link obs |

---

## 6. CLI 參考

| 參數 | 型別 | 預設值 | 說明 |
|------|------|--------|------|
| `--pathType` | string | `gw2gw_e2e` | 唯一路徑語意入口 |
| `--simTime` | double | 120.0 | 模擬時長（秒） |
| `--slotInterval` | double | 60.0 | 路由更新間隔（秒） |
| `--gwId` | uint32 | 0 | GW preset ID（gw2sat / sat2gw / gw2ut_e2e） |
| `--gwSrc` / `--gwDst` | uint32 | 0 / 1 | GW pair（gw2gw_e2e） |
| `--utId` | uint32 | 0 | 邏輯 UT ID（sat2ut / gw2ut_e2e） |
| `--satSrc` / `--satDst` | uint32 | 0 / 33 | SAT pair（sat2sat） |
| `--islDropThreshPct` | double | 1.0 | ISL drop rate PASS 門檻（%） |
| `--obsLogPath` | string | `e2e_link_obs.csv` | Link Observer CSV 路徑 |
| `--obsInterval` | double | 10.0 | 快照週期（秒） |

**GW Preset 對照：**

| ID | 名稱 | 座標 |
|----|------|------|
| 0 | JP-Tokyo | 35.7°N, 139.7°E |
| 1 | IN-NewDelhi | 28.6°N, 77.2°E |
| 2 | CN-Shanghai | 31.2°N, 121.5°E |
| 3 | BR-SaoPaulo | 23.5°S, 46.6°W |
| 4 | IN-Mumbai | 19.1°N, 72.9°E |

---

## 7. 最終驗證結果

**驗證環境**：Iridium-66，simTime=120s（gw2gw_e2e=300s），slotInterval=60s，ISL 10Mbps/5000km
**GW Preset**：GW0=JP-Tokyo（35.7°N,139.7°E），GW1=IN-NewDelhi（28.6°N,77.2°E）

### 7.1 六種 pathType 總覽

| pathType | 場景 | 主 Verdict | ISL drop | wall (s) |
|----------|------|-----------|---------|---------|
| `sat2sat` | SAT0→SAT33，7-hop | ISL_LAYER **PASS** \| scopedRxPkts=173,595 | **0.240% PASS**（14-15 熱點 10.84%） | 723.9 |
| `gw2sat` | GW0=JP-Tokyo feeder up | FEEDER_LAYER **PASS** \| scopedRxPkts=32,087 | 0.000% PASS | 554.9 |
| `sat2gw` | GW0=JP-Tokyo feeder dn | FEEDER_LAYER **PASS** \| scopedRxPkts=5,148 | 0.000% PASS | 561.7 |
| `sat2ut` | UT-Taipei service link | SERVICE_LAYER **PASS** \| utRxPkts=1,189 | 0.000% PASS | 586.2 |
| `gw2ut_e2e` | JP-Tokyo→UT-Taipei（no ISL hop） | FEEDER+SERVICE **PASS** \| utRxPkts=1,189 | 0.000% PASS | 603.9 |
| `gw2gw_e2e` | JP-Tokyo→IN-NewDelhi，300s | ROUTING_LAYER+ISL_LAYER+PACKET_LAYER **全 PASS** \| traceRxPkts=2,979 | 0.000% PASS | 1293.0 |

---

### 7.2 sat2sat（SAT0 → SAT33，120s）

**Log**：`audit/final/sat2sat.log`

路由表（3 slots 相同）：

| slot | t(s) | full_path | route_cost |
|------|------|-----------|-----------|
| 0 | 0 | `0→1→2→57→46→35→34→33` | 0.078176 |
| 1 | 60 | `0→1→2→57→46→35→34→33` | 0.074919 |
| 2 | 120 | `0→1→2→57→46→35→34→33` | 0.072055 |

ISL drop 明細（僅列有 drop 的 link）：

| ISL | total_pkts | dropped | drop_rate |
|-----|-----------|---------|----------|
| 13-14 | 75,763 | 3 | 0.004% |
| **14-15** | **153,004** | **16,583** | **10.838%** |
| 3-14 | 75,851 | 43 | 0.057% |
| TOTAL | **6,926,118** | **16,629** | **0.240% [PASS]** |

> isl:14-15 高丟棄率為 aggressive background load 設計（製造 ISL 佇列壓力），overall 仍通過 1% 門檻。

Verdict：
```
[ISL_LAYER] PASS | scopedLinks=7 scopedRxPkts=173,595
```

---

### 7.3 gw2sat（GW0=JP-Tokyo feeder up，120s）

**Log**：`audit/final/gw2sat.log`

GW0 可見衛星：3 slots 均 2 sats（sat15, sat45）。feeder:sat45=0 為預期行為（Dijkstra 選 sat15 為最短路徑）。

Verdict：
```
[FEEDER_LAYER] PASS | scopedKeys=2 scopedRxPkts=32,087
```

---

### 7.4 sat2gw（GW0=JP-Tokyo feeder dn，120s）

**Log**：`audit/final/sat2gw.log`

運行事件 `POSSIBLE LINK FAILURE at t=70s` 為 slot boundary 邊緣效應假警報，非真實鏈路失效（feeder:gw0 全程 drop=0 確認）。

Verdict：
```
[FEEDER_LAYER] PASS | scopedKeys=1 scopedRxPkts=5,148
```

---

### 7.5 sat2ut（UT0=UT-Taipei，120s）

**Log**：`audit/final/sat2ut.log`

```
[UT_SELECT] logicalUtId=0 routeServingSat=15 trafficUtUserId=22 requestedUtUserId=0
```

UT0[UT-Taipei] 全程由 sat15 服務。`service:sat15`=2,704 pkts（衛星端），`service:ut22`=1,189 pkts（UT 端）。

Verdict：
```
[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2,704 utKeys=1 utRxPkts=1,189
```

---

### 7.6 gw2ut_e2e（GW0=JP-Tokyo → UT0=UT-Taipei，120s）

**Log**：`audit/final/gw2ut_e2e.log`

JP-Tokyo 與 UT-Taipei 地理相近，sat15 同時覆蓋兩端，entry==serving，無需 ISL hop。

Verdict：
```
[FEEDER_LAYER]  PASS | scopedKeys=1 scopedRxPkts=2,388
[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2,704 utKeys=1 utRxPkts=1,189
[ISL_LAYER]     not_applicable | valid route has no ISL hop
```

---

### 7.7 gw2gw_e2e（GW0=JP-Tokyo → GW1=IN-NewDelhi，300s）

**Log**：`audit/final/gw2gw_e2e.log`

obsFeederMode=ROUTING（feeder PHY 不觀測，原因見 Section 8.1）。

路由表（JP-Tokyo → IN-NewDelhi）：

| slot | t(s) | entry | ISL_path | exit | isl_cost(s) |
|------|------|-------|----------|------|------------|
| 0 | 0 | 45 | `45→46→35→34→33` | 33 | 0.047382 |
| 1 | 60 | 15 | `15→14→3→4` | 4 | 0.037122 ← ROUTE CHANGED |
| 2 | 120 | 45 | `45→34→33` | 33 | 0.029282 ← ROUTE CHANGED |
| 3 | 180 | 14 | `14→3→4` | 4 | 0.026127 ← ROUTE CHANGED |
| 4 | 240 | 45 | `45→34→33` | 33 | 0.027011 ← ROUTE CHANGED |
| 5 | 300 | 45 | `45→34→33` | 33 | 0.025810 |

封包層交付：
```
[GW2GW_APP] received: 1,525,248 bytes (~2,979 pkts)
[GW2GW_OBS][PACKET] traceRxPkts=2979 traceRxBytes=1525248
```

Verdict：
```
[ROUTING_LAYER] PASS | validSlots=6/6 gwSrc=0 gwDst=1
[ISL_LAYER]     PASS | scopedLinks=16 scopedRxPkts=968,854
[PACKET_LAYER]  PASS | traceRxPkts=2979 traceRxBytes=1525248
```

IslRoutingManager Stats（全槽 HasSignificantChange=YES）：

| slot | apply(ms) | recompute(ms) | recompSrc | changed |
|------|-----------|--------------|-----------|---------|
| 0 | 0 | 0 | 0 | NO |
| 1 | 0 | 0 | 3 | YES |
| 2 | 0 | 0 | 17 | YES |
| 3 | 2 | 2 | 23 | YES |
| 4 | 0 | 0 | 30 | YES |
| 5 | 2 | 2 | 33 | YES |

---

## 8. 已知架構限制

### 8.1 gw2gw_e2e feeder PHY 不可觀測

在 SNS3 `REGENERATION_NETWORK` 模式下，GW-to-GW application traffic 在衛星端被重新產生並經 ISL 骨幹轉送。`SatOrbiterNetDevice::RxFeeder` 與 GW-side `SatNetDevice::Rx` 不觸發，計數保持 0。

**這不是 link failure**，是 traffic model 的 trace 可見性限制。因此 `gw2gw_e2e` verdict 設計為：主判定 ROUTING_LAYER + ISL_LAYER + PACKET_LAYER，feeder PHY 不參與 verdict。

### 8.2 sat2gw 觀測視窗假警報

slot boundary 後約 10s 視窗，throughput window 計算為 0，觸發 `POSSIBLE LINK FAILURE`。根因為 obsInterval 視窗 baseline 不連續，非真實封包丟失。Grace period 修法已分析，刻意 deferred。

### 8.3 Load-aware 重算已觸發，但未觀察到負載主導切換

EMA load cost 最高約 0.84ms，傳播延遲約 44ms（佔比 ~2%）。幾何代價仍主導 Dijkstra。若需驗證 load-driven 切換，須設計 ISL 飽和壓測場景（約 80%+ link utilization）。

### 8.4 sat2ut / gw2ut_e2e Endpoint Probe App 計數

Endpoint probe 使用 port=9100（診斷用），主流量不打到該 port，故 `app rxPkts=0`。`dev rxPkts=1189` 確認 physical UT device 已收到封包，正確 interpretation 為 `device_rx_observed_probe_app_idle`。

---

## 8.5 驗證

**結果來源**：`Topology & ISL Routing/Outputs/audit/layer1/`


### 8.5.1 總覽

| case | 驗證重點 | 可以判斷什麼 | 關鍵結果 |
|------|----------|--------------|----------|
| `gw2sat` | GW 上行 feeder | feeder uplink 可達，且可量測 feeder delay | `FEEDER_LAYER PASS`，`avgDelay=4.62ms` |
| `sat2gw` | 衛星下行 feeder | feeder downlink 可達 | `FEEDER_LAYER PASS`，`scopedRxPkts=5148` |
| `sat2sat` | 純 ISL 骨幹 multi-hop | 可證明 hop-by-hop 轉送存在，並辨識骨幹 drop 熱點 | `ISL_LAYER PASS`，7-hop path，總 drop `0.241%` |
| `sat2ut` | 純 service segment | sat→UT service link 可達 | `SERVICE_LAYER PASS`，`satAvgDelay=63.07ms` |
| `gw2ut_e2e`（Taipei） | feeder + service，無 ISL hop | e2e 不一定經過 ISL，hop 數由幾何覆蓋決定 | `FEEDER PASS` + `SERVICE PASS` + `ISL not_applicable` |
| `gw2ut_e2e`（San Francisco） | feeder + ISL + service | e2e 會在需要時跨 ISL 骨幹，且可追出 entry / serving / hop 數 | `FEEDER PASS` + `SERVICE PASS` + `ISL PASS` |
| `gw2gw_e2e` | routing + ISL transit + packet delivery | GW-GW 路徑在多 slot 下持續有效，且 route 會隨幾何更新 | `ROUTING PASS` + `ISL PASS` + `PACKET PASS` |

### 8.5.2 `gw2sat`：GW 上行 feeder 已驗證


```text
[FEEDER_LAYER] PASS | scopedKeys=2 scopedRxPkts=32087 avgDelay=4.62ms
```

- 只測 feeder uplink，不混入 ISL 或 service
- `avgDelay=4.62ms` 為 feeder segment 的量測延遲，不代表端到端延遲。

### 8.5.3 `sat2gw`：衛星下行 feeder 已驗證

```text
[FEEDER_LAYER] PASS | scopedKeys=1 scopedRxPkts=5148
```

- 證明 satellite 到 gateway 的 feeder downlink 可達，且封包已到 GW device。
- `POSSIBLE LINK FAILURE at t=70s` 不能直接解讀為斷鏈；因為最終 `feeder:gw0` 仍持續收到 `5148` 個封包，因此這是 slot boundary 附近的觀測窗假警報，不是物理中斷。

### 8.5.4 `sat2sat`：純 ISL multi-hop 骨幹已驗證

3 個 slot 的 full path exist，而且都是 7-hop：

| slot | time(s) | full_path | route_cost(s) |
|------|---------|-----------|---------------|
| 0 | 0 | `0->1->2->57->46->35->34->33` | 0.078176 |
| 1 | 60 | `0->1->2->57->46->35->34->33` | 0.074919 |
| 2 | 120 | `0->1->2->57->46->35->34->33` | 0.072055 |

```text
[ISL_LAYER] PASS | scopedLinks=7 scopedRxPkts=173596
TOTAL: 6926030 pkts, 16689 dropped | drop_rate=0.241%
```

有 drop 的熱點 ISL：

| ISL | total_pkts | dropped | drop_rate |
|-----|-----------:|--------:|----------:|
| `13-14` | 75,746 | 3 | 0.004% |
| `14-15` | 152,989 | 16,646 | 10.881% |
| `3-14` | 75,855 | 40 | 0.053% |


- 可穩定建立並維持明確的 multi-hop ISL path。
- hop 數可以直接由 `full_path` 的衛星序列計算；此例是 7 顆衛星、6 條 ISL link 的骨幹轉送鏈。
- 指出骨幹在背景負載下存在 hotspot link，例如 `14-15` drop rate 高，但整體骨幹總 drop 仍只有 `0.241%`，低於 `1%` 門檻，因此整體 ISL 層仍判定 PASS。

### 8.5.5 `sat2ut`：純 service 段已驗證

```text
[UT_SELECT] logicalUtId=0 routeServingSat=15 trafficUtUserId=22 requestedUtUserId=0
[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2704 satAvgDelay=63.07ms utKeys=1 utRxPkts=1189
```

- 只驗 service link，不經 feeder 與 ISL，因此可以單獨證明 satellite 到 user terminal 的交付段成立。
- `routeServingSat=15` 表示由 `sat15` 提供服務。
- `satAvgDelay=63.07ms` 為 service 段的觀測延遲；它不是完整 e2e latency。

### 8.5.6 `gw2ut_e2e`（Taipei）：e2e 成立，但不需要 ISL hop

| slot | time(s) | entry | ISL_path | serving | isl_cost |
|------|---------|-------|----------|---------|----------|
| 0 | 0 | 15 | `15` | 15 | `no ISL hop` |
| 1 | 60 | 15 | `15` | 15 | `no ISL hop` |
| 2 | 120 | 15 | `15` | 15 | `no ISL hop` |

```text
[FEEDER_LAYER] PASS | scopedKeys=1 scopedRxPkts=2388 avgDelay=2.59ms
[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2704 satAvgDelay=63.07ms utKeys=1 utRxPkts=1189
[ISL_LAYER] not_applicable | valid route has no ISL hop
```

- 端到端成立不等於一定經過 ISL。
- `entry == serving == sat15`，表示 gateway 與 UT 同時被同一顆衛星覆蓋，所以不需要跨衛星轉送。
- hop 數不是固定架構參數，而是幾何覆蓋與可見性決定的動態結果。

### 8.5.7 `gw2ut_e2e`（San Francisco）：e2e 成立，且經過 ISL hop

| slot | time(s) | entry | ISL_path | serving | isl_cost(s) |
|------|---------|-------|----------|---------|-------------|
| 0 | 0 | 15 | `15->14->25->36->37` | 37 | 0.043969 |
| 1 | 60 | 45 | `45->46->57->2->1` | 1 | 0.041882 |
| 2 | 120 | 45 | `45->46->57->2->1` | 1 | 0.040067 |
| 3 | 180 | 14 | `14->25->36` | 36 | 0.024543 |

```text
[FEEDER_LAYER] PASS | scopedKeys=3 scopedRxPkts=1799 avgDelay=3.00ms
[SERVICE_LAYER] PASS | satKeys=3 satRxPkts=1022 satAvgDelay=94.60ms utKeys=1 utRxPkts=1189
[ISL_LAYER] PASS | scopedLinks=8 scopedRxPkts=287801
```

- GW→UT 端到端流量在需要時會橫跨 ISL 骨幹，再落到服務衛星。
- `entry` 是 feeder 進入骨幹的衛星，`serving` 是最後服務 UT 的衛星，`ISL_path` 中間節點則是 transit hop。
- `isl_cost` 在不同 slot 下降，說明幾何路徑變短；骨幹代價隨衛星位置改變，但不能直接說是壅塞改善。

### 8.5.8 `gw2gw_e2e`：GW-GW routing、transit、packet delivery 全部成立

| slot | time(s) | entry | ISL_path | exit | isl_cost(s) |
|------|---------|-------|----------|------|-------------|
| 0 | 0 | 45 | `45->46->35->34->33` | 33 | 0.047382 |
| 1 | 60 | 15 | `15->14->3->4` | 4 | 0.037122 |
| 2 | 120 | 45 | `45->34->33` | 33 | 0.029282 |
| 3 | 180 | 14 | `14->3->4` | 4 | 0.026127 |
| 4 | 240 | 45 | `45->34->33` | 33 | 0.027011 |
| 5 | 300 | 45 | `45->34->33` | 33 | 0.025810 |

```text
[GW2GW_APP] received: 1525248 bytes (~2979 pkts)
[ROUTING_LAYER] PASS | validSlots=6/6 gwSrc=0 gwDst=1
[ISL_LAYER] PASS | scopedLinks=16 scopedRxPkts=968854
[PACKET_LAYER] PASS | traceRxPkts=2979 traceRxBytes=1525248
```

- Layer 1 會計算path，也真的把 GW-GW 流量送到對端應用層。
- `ROUTE CHANGED` 的意義是 slot 更新後，entry / exit / ISL path 隨幾何與可見性調整；它代表動態路由能力，不代表中斷。
- `validSlots=6/6` 可以用來說明在整個 300 秒觀察窗內，每個 slot 都有有效路徑。
- `isl_cost` 可描述為 Layer 1 用來選路的骨幹總成本，包含 propagation cost 與 load cost；但從數值趨勢看，本輪主要還是幾何距離主導。

### 8.5.9 can & cannot

可以明確判斷：

- feeder 是否成立：看 `gw2sat` / `sat2gw` 的 `FEEDER_LAYER PASS`、scoped Rx 與 delay。
- service 是否成立：看 `sat2ut` / `gw2ut_e2e` 的 `SERVICE_LAYER PASS`、`routeServingSat`、UT device Rx。
- 是否經過 ISL：看 `ISL_path` 是否只有單一衛星，或 verdict 是否為 `ISL_LAYER PASS` / `not_applicable`。
- hop 數多少：直接由 `ISL_path` 的衛星序列計算。例如 `45->46->57->2->1` 代表 4 條 ISL hop。
- 路由是否隨時間變化：看 `ROUTE CHANGED`、不同 slot 的 `entry / exit / serving` 是否改變。
- 哪一層量到的 delay：`gw2sat` 是 feeder delay，`sat2ut` / `gw2ut` 的 sat-side service delay 是 service delay；這些都不是完整 e2e latency。
- 骨幹哪裡有壅塞熱點：看 `ISL Packet Drop Rate Summary` 與 `ISL Load Cost Summary`，例如 `sat2sat` 的 `14-15`。

不能宣稱：

- 不能把 feeder 或 service 的 `avgDelay` 直接說成端到端延遲。
- 不能把 `POSSIBLE LINK FAILURE` 直接說成真實斷鏈；必須回頭看最終 scoped Rx、drop 與 verdict。
- 不能因為 `HasSignificantChange=YES` 就宣稱已證明壅塞主導 reroute；這只證明 load-aware 更新鏈有觸發。

對 load-aware routing ：

- 已驗證 `queue bytes -> queue delay proxy -> EMA smoothing -> HasSignificantChange -> RecomputeAffectedRoutes` 這條控制鏈有實際運作。
- 已驗證 route update 會在 slot 切換時發生，且會重算受影響來源節點。
- 尚未驗證 `load cost` 足以壓過 propagation cost，進而主導最終改道。
- 本輪 log 中觀察到的 EMA load cost 大致在 `0.5ms` 到 `0.7ms` 級別，最高約 `0.70ms`；相較於 `gw2gw_e2e`、`gw2ut_e2e` 路徑上的 `isl_cost` 約 `24ms` 到 `47ms`，比例仍偏小。目前已驗證 load-aware 機制會工作，但本輪仍以幾何成本主導選路。

Endpoint probe 的正確判讀：

- `sat2ut` 與 `gw2ut_e2e` 的 endpoint probe 綁的是 `9100` port，而實際資料流未必走同一個 app port。
- 因此 `app rxPkts=0` 但 `dev rxPkts=1189` 代表 probe app 沒收到它監看的流，不代表 UT 沒收到資料。
- 對外說明時，應以 `dev/mac/phy` 接收結果與最終 verdict 為主。

### 8.5.10 理論延遲 vs 目前實作量測

外部參考圖中的理論值如下：

| Scenario | 定義 | 600 km | 1200 km | 解讀 |
|------|------|------|-------|------|
| Scenario C | transparent payload，包含 `service + feeder` | `25.77 ms` | `41.77 ms` | 以空中傳播延遲為主的端到端參考值 |
| Scenario D | regenerative payload，只看 `service` | `12.89 ms` | `20.89 ms` | 以空中傳播延遲為主的 service-only 參考值 |

這兩組數字和我們目前 log 中的 delay **不是同一種量測定義**。

目前 Layer 1 log 量到的是 segment-level observability delay：

| 目前案例 | 現在量到什麼 | 數值 | 正確解讀 |
|------|------|------|------|
| `gw2sat` | feeder segment observed delay | `4.62 ms` | feeder 單段 trace delay，不是 e2e |
| `sat2ut` | sat-side service observed delay | `63.07 ms` | service 單段 trace delay，不是純 propagation |
| `gw2ut_e2e` Taipei | feeder + service path | 沒有直接 e2e delay | 已證明 path 成立，但目前只有分段 delay |
| `gw2ut_e2e` San Francisco | feeder + ISL + service path | 沒有直接 e2e delay | 已證明 path / hop / cost 成立，但尚非 packet-level e2e latency |


- Scenario C / D 是依 payload 架構與星地距離推估的 propagation-dominant theoretical latency。
- 目前 Layer 1 log 是在模擬系統的特定 trace point 量 delay，因此可能包含 propagation 之外的 scheduler timing、queueing、MAC/PHY timing 與 delivery effects。
- 因此目前 log 中的 delay 適合做相對工程判讀，但不應直接宣稱為嚴格的端到端 latency。

目前可以合理主張：

- `gw2sat 4.62 ms` 顯示 feeder 段延遲量級小且合理。
- `sat2ut 63.07 ms` 顯示 service segment 可觀測且正常運作，但它不等於文獻中的 `12.89 / 20.89 ms` service-only 理論 propagation latency。
- `gw2ut_e2e` Taipei 沒有 ISL hop，因此其真實 e2e latency 理論上應比 San Francisco case 更接近 Scenario C。
- `gw2ut_e2e` San Francisco 含 ISL hop，因此其真實 e2e latency 理論上應高於單純的 Scenario C / D 單段參考值。

對先前 `sat2ut >1000 ms` 結果的正確說明：

- 舊版 `sat2ut` 異常值 `1396.82 ms` 來自較早期的 shared-edge 配置，當時 `utUsers=91`，不是目前的單一目標 UT 驗證。
- 該數字應解讀為 shared traffic 條件下的 service-side queueing / observation delay，不應視為可信的單一 UT 端到端延遲。
- 在流量縮減為 `utUsers=1` 後，service observed delay 回到 `63.07 ms`。

若要得到真正的端到端延遲，還缺：

- packet-level sender timestamp
- packet-level receiver timestamp
- 以同一 packet identity 計算 one-way delay 或 RTT

因此目前最精確的總結是：

> 文獻中的數值代表 theoretical propagation latency；我們目前 Layer 1 implementation 回報的是 segment-level observed delay。現階段已驗證 path correctness、hop structure 與 relative delay behavior，但尚未提供嚴格的 packet-level end-to-end latency measurement。

## 9. SNS3 修改清單

| 檔案 | 位置 | 修改內容 |
|------|------|----------|
| `satellite-sgp4-mobility-model.h/.cc` | `contrib/satellite/model/` | 新增 `GetGeoPositionAt(Time t)` — 支援與模擬時鐘解耦的軌道位置查詢 |
| `satellite-isl-arbiter-unicast.h` | `contrib/satellite/model/` | 新增 `ClearNextHopEntries()` — 支援跨 slot 清空覆寫 |

**CMakeLists.txt**（`contrib/satellite/CMakeLists.txt`）新增：
```cmake
# source_files
helper/isl-graph.cc
helper/ft-filter.cc

# header_files
helper/isl-graph.h
helper/ft-filter.h
```

---

## 10. Layer 2 介面

Layer 1 對 Layer 2（Beam Hopping Controller）提供的穩定介面：

| 介面 | API | 說明 |
|------|-----|------|
| 路由查詢 | `GetGwRoute(srcGwId, dstGwId, slot)` | GW→GW 路由（entry, exit, path, cost） |
| 路由查詢 | `GetGwUtRoute(gwId, utId, slot)` | GW→UT 路由（entry, serving, path, cost） |
| 路徑展開 | `TracePath(src, dst, slot)` | 取得完整跳數序列 |
| 可見性 | `GetGwVisibleSats(gwId, slot)` | GW 在指定槽可見衛星集合 |
| ISL cost | 需新增 public accessor `GetIslLoadCost(a, b)` | `m_loadCosts` 目前為 private，不應直接作為 Layer 2 API |
| 時間槽事件 | `ScheduleRoutingUpdates()` | 路由更新已納入 NS3 event scheduler |
| FT filter | `FtVisibilityFilter` | `ft-filter.h/.cc`，Layer 2 可直接呼叫 |

**Layer 2 開發約定**：
- `IslRoutingManager` 介面已穩定，不應修改 Layer 1 架構
- Layer 2 程式碼路徑：`Beam Hopping Controller/Codes`
- 若需存取衛星拓樸，使用 `Singleton<SatTopology>::Get()` — Layer 1 已驗證其 API

---

## 11. 設計決策文件索引

| 決策 | 文件 |
|------|------|
| Arbiter 機制取代 IP 層路由 | `Decisions/01_Arbiter mechanism replaces IP layer routing.md` |
| ISL 距離門檻設為 5000 km | `Decisions/02_ISL Distance Threshold.md` |
| Arbiter 預先建立（非排程內建立） | `Decisions/03_Arbiter lifecycle management (pre-creation vs. scheduled creation).md` |
| Beam Scheduler 開銷根本原因 | `Decisions/04_Beam Scheduler.md` |
