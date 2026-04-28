# 2026-04-27 今日工作日誌：
`gw-path-inspector.cc` 結構與

## 今日目標
- 釐清 `gw2gw` 與 `gw2ut` 兩種檢查視角的輸出內容。
- 釐清 `capacity` 模式目前是理論瓶頸估算，而非 packet-level 驗證。
- 記錄目前直接執行時的環境限制與後續可改進方向。

---

## 目標檔案

- `scratch/gw-path-inspector.cc`

---

## 程式定位

`gw-path-inspector.cc` 用來抽取 `IslRoutingManager` 已預先計算好的路徑結果，並整理成可讀的文字報告與 evidence index。

它目前回答的核心問題有兩類：

- `GW -> GW` 在指定 slot 會走哪條邏輯衛星路徑。
- `GW -> UT` 在指定 slot 會走哪條邏輯衛星路徑。
- 這條路徑在 feeder / service / ISL 假設下的理論瓶頸容量大概是多少。

它不負責的部分：

- 不直接驗證實際封包是否成功送達。
- 不直接量測 queue / delay / drop 的真實 runtime 行為。
- 不取代 `test-iridium-e2e.cc` 那類完整情境模擬與 verdict。

因此它比較接近：

```text
control-plane route inspection
+ theoretical capacity estimation
+ evidence-oriented reporting
```

---

## 主要資料結構

程式主要把設定、證據與容量分析拆成幾個結構：

- `InspectConfig`
  - 收納 CLI 參數與分析假設。
  - 包含 `mode`、`view`、`simTime`、`slotInterval`、`gwSrc/gwDst/gwId/utId`、`beamId`、`islRateMbps`、spectral efficiency 等設定。

- `GatewayPreset`
  - 內建 gateway 預設座標。
  - 目前支援 Tokyo、New Delhi、Shanghai、Sao Paulo、Mumbai。

- `EvidenceRecord`
  - 把路徑上的每一段整理成 `segment/status/source/detail`。
  - 方便後續輸出成 CSV 做檢查索引。

- `CapacityResult`
  - 收納容量分析結果。
  - 包含各 segment capacity、瓶頸名稱、瓶頸速率與估計 pass rate。

---

## 主程式流程

主程式從 `main()` 開始，流程可以整理成：

```text
1. 建立 InspectConfig 並解析 CLI
2. 用 SimulationHelper 載入 Iridium-66 scenario
3. 建立 IslRoutingManager 並設定 slot / ISL 參數
4. 依 view 加入 GW / UT 與對應 pair
5. 執行 PrecomputeAllTables()
6. 執行 PrecomputeGwRoutes() / PrecomputeGwUtRoutes()
7. 取出指定 slot 的 route
8. 生成 inspection report
9. 若 mode=capacity，附加理論容量分析
10. 輸出 report 與 evidence csv
```

關鍵初始化行為：

- `SimulationHelper("gw-path-inspector")`
- `LoadScenario("constellation-iridium-66-sats-fixed")`
- `SetBeamSet(std::set<uint32_t>{cfg.beamId})`
- `SetUserCountPerUt(1)`
- `CreateSatScenario()`

路由管理器的關鍵動作：

- `Initialize(islsFilePath)`
- `SetGwElevationThreshold(cfg.elevMinDeg)`
- `PrecomputeAllTables()`
- `PrecomputeGwRoutes()`
- `PrecomputeGwUtRoutes()`

---

## 檢查模式

### `inspect` 模式

`inspect` 模式只做路徑與拓樸整理，不做容量估算。

輸出重點：

- scenario summary
- slot 與 selected time
- gateway / UT visibility
- entry / exit / serving satellite
- satellite path
- logical path
- physical attachment summary
- evidence index

### `capacity` 模式

`capacity` 模式會在 inspection report 後面再附一段容量分析。

目前這個分析不是 packet simulation，而是理論估算：

- 先估 feeder capacity
- 視情況加入 ISL capacity
- `gw2ut` 時再加入 service capacity
- 取最小值當 bottleneck
- 再用簡化 binary search 找出符合 `goodputFloorPct` 的估計 pass rate

程式內也明確保留了 `dropThreshPct` 與 `delayThreshMs`，但目前仍屬 operator-facing limit，尚未真正接進 packet-level probe。

---

## `gw2gw` 視角輸出

`AppendGw2GwInspection()` 會整理：

- source gateway 可見衛星集合
- destination gateway 可見衛星集合
- route 是否有效
- `entrySatId`
- `exitSatId`
- `satPath`
- `logicalPath`

邏輯路徑會被格式化成：

```text
JP-Tokyo(gw0)
-> entry:satX
-> satX -> satY -> satZ
-> exit:satN
-> IN-NewDelhi(gw1)
```

另外也會附上 physical attachment path，說明 GW user node、beam 對應 GW node，以及中間的 sat network 是由 logical path 推導而來。

evidence index 中對應的 segment 包含：

- `logical_route`
- `feeder_uplink`
- `isl_path`
- `feeder_downlink`
- `packet_endpoint`

---

## `gw2ut` 視角輸出

`AppendGw2UtInspection()` 會整理：

- gateway visible sats
- UT visible sats
- common visible sats
- route 是否有效
- `entrySatId`
- `servingSatId`
- `satPath`
- `logicalPath`

邏輯路徑會被格式化成：

```text
GW preset
-> entry:satX
-> satX -> satY -> satZ
-> serving:satN
-> UT-Taipei(ut0)
```

這裡另外多了一步 `ResolveTrafficUtUserId()`，目的是根據實際 route 的 `servingSatId` 去找目前 scenario 中對應的 UT user node，避免只用 logical `utId` 造成 endpoint 映射錯誤。

evidence index 中對應的 segment 包含：

- `logical_route`
- `feeder_uplink`
- `isl_path`
- `service_downlink`
- `endpoint`

---

## 容量分析模型

目前容量分析採用簡化理論模型。

`gw2gw`：

- source feeder per beam
- destination feeder per beam
- 若有多顆衛星，加入 ISL rate
- 取最小值作為 bottleneck

`gw2ut`：

- feeder per beam
- 若有多顆衛星，加入 ISL rate
- service per beam
- 取最小值作為 bottleneck

對應公式概念為：

```text
feeder capacity
= fwdFeederBandwidthHz / fwdFeederChannels * feederSpectralEfficiency

service capacity
= fwdUserBandwidthHz / fwdUserChannels * userSpectralEfficiency

isl capacity
= islRateMbps * 1000
```

接著以 `goodputFloorPct` 為門檻，做最多 32 次的 binary search，找出估計可接受速率 `passRateKbps`。

這表示目前的 `capacity` 模式是：

- 用設定值推估理論上限
- 用 operator-facing 門檻換算保守可接受速率
- 不是以 runtime trace 驗證 throughput / drop / latency

---

## 輸出產物

每次執行會產生兩份檔案：

- `reportPrefix-<timestamp>.txt`
- `reportPrefix-<timestamp>-evidence.csv`

其中：

- `.txt` 用來給人讀，整理 scenario、logical path、physical path、capacity 結果。
- `.csv` 用來做 evidence index，讓每段路徑都有 `status/source/detail` 可追溯。

console 也會同步印出 report 內容與 artifact 路徑。

---

## 今日結論

 `gw-path-inspector.cc` 的定位整理與程式閱讀

目前定位成：

- Layer 1 logical route inspection tool
- GW / UT attachment path explanation tool
- theoretical bottleneck capacity prototype

它適合拿來做：

- slot-based 路徑檢查
- visibility / entry / exit / serving sat 驗證
- route evidence 報告輸出
- capacity 上限初步估算

但如果要回答：

- 實際封包有沒有到
- 某條流量在 runtime 會不會 drop
- throughput / delay 是否符合門檻

仍然需要搭配 `test-iridium-e2e.cc` 這類完整 packet-level simulation 與 observability 工具。

---

## 下一步建議

- 將 `ns3BasePath` 改為 CLI 參數或相對路徑，避免硬編碼環境依賴。
- 若要讓 `capacity` 模式更有驗證力，可加入實際 traffic probe 與 PacketSink / trace-based goodput 統計。
- 若要提升 physical path 可觀測性，可補上逐 hop trace 或與既有 OBS scope 結果對接。
---

## 補充整理：`gw preset`、`physical gw` 與封包通過節點分析

這次再次確認後，需要把 `gw preset` 與 `physical gw` 明確拆開看，因為它們在 `gw-path-inspector.cc` 中屬於不同層級的資訊。

### `gw preset` 的角色

`gw preset` 是程式內建的 gateway 定義表，內容包含：

- `gwId`
- latitude
- longitude
- gateway name

它的主要用途是提供 `IslRoutingManager` 做邏輯路由計算，不是直接代表 SNS3 場景中的實體 node。

執行邏輯為：

- `FindGatewayPreset(gwId)`
- `AddGatewayOrAbort()`
- `routingMgr->AddGateway(...)`
- `AddGwPair()` 或 `AddGwUtPair()`

因此 `gw preset` 負責的是：

- 定義 gateway 在地理上的位置
- 讓 routing manager 計算 gateway 在某個 slot 可見哪些衛星
- 決定 `entry sat`
- 決定 `exit sat` 或 `serving sat`
- 產生中間的 `satPath`

可以把它解讀為：

```text
logical routing endpoint
```

### `physical gw` 的角色

`physical gw` 不是從 preset table 來，而是從 `SatTopology` 查詢出來。

目前工具用到的 physical-side API 包含：

- `GetGwNodes()`
- `GetGwUserNodes()`
- `GetGwFromBeam(beamId)`

因此 `physical gw` 回答的是：

- 這個 `beamId` 在目前 SNS3 scenario 中附著到哪個實體 GW node
- ground side 有哪些 physical GW node / GW user node 可以作為 endpoint 說明

可以把它解讀為：

```text
scenario physical attachment
```

### 兩者的關係

目前這支工具不是把 `gw preset` 直接唯一映射到某個 physical GW node，而是把兩條資訊並列：

- `gw preset` 提供邏輯路由端點
- `physical gw` 提供 beam 與場景中 GW node 的附著摘要

也就是說，這支工具目前回答的是：

```text
路由理論上應該怎麼走
+ 這個 beam 在場景裡附著到哪個 GW node
```

而不是：

```text
gw0 / gw1 各自在場景中唯一且完整地對應到哪個 physical GW node
```

### 封包通過節點分析的語意

這次也確認，程式中的封包通過節點分析屬於推導式分析，不是 packet-level hop-by-hop trace。

`logicalPath` 來自：

- `GetGwRoute()`
- `GetGwUtRoute()`

這部分是真正的 route computation result，會列出：

- `entrySatId`
- `exitSatId` 或 `servingSatId`
- `satPath`

而 `physicalPath` 則是由 topology API 搭配 logical route 組出來的摘要。  
報告中已經明確寫出：

```text
satNetwork(derived from logical path)
```

這表示中間衛星網路那一段不是由封包 trace 直接觀測，而是依據 `satPath` 推導：

- 封包應從哪個 GW user node 出發
- 經過哪個 physical GW attachment
- 進入哪顆 entry satellite
- 經過哪些 satellite
- 從哪顆 exit / serving satellite 離開
- 最後到哪個 GW user / UT user endpoint

因此目前它能支持的結論是：

- logical route 存在且可列出
- endpoint node 可由 topology API 解析
- beam 與 physical GW node 的附著關係可被列出

但目前不能單獨證明：

- runtime 封包逐 hop 真正走過哪些節點
- 每一跳是否有被 trace callback 直接觀測
- actual packet forwarding 是否完全等同 logical route

### `gw2gw` case 的額外注意點

在 `gw2gw` 視角中，`physicalPath` 目前使用同一個：

- `mappedGw = topo->GetGwFromBeam(cfg.beamId)`

來描述 source 與 destination 兩端的 physical GW attachment。

所以它比較接近：

```text
gwUserSrc
-> gwNodeBeamMap
-> satNetwork(derived)
-> gwNodeBeamMap
-> gwUserDst
```

這比較適合解讀成：

- beam attachment summary
- logical 上下行附著位置的示意

不應直接解讀成：

- source GW 與 destination GW 的實際 physical GW node 已經被逐端精確驗證

因此在 `gw2gw` case 下，目前的 `physicalPath` 應視為：

```text
attachment summary
```

而不是：

```text
strict packet traversal evidence
```
