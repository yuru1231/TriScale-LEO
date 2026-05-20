# 工作日誌 2026-04-28

## 主題

釐清此 repo 中 `beam / sat / GW` 三者的建立關係，並說明為什麼原本 `test-iridium-e2e.cc` 抓不到其他 `physical GW node`。

---

## 問題背景

核心問題：
即使 routing preset 已經有 `gw0..gw4`，實際上 `SatTopology::GetGwNodes()` 卻只有部分 physical GW node，導致 `GetGwNodesById(gwId)` 對某些 `gwId` 回傳空結果？

---

## 先看 test-iridium 端怎麼使用 GW node

在 `test-iridium-e2e.cc` 中，GW physical node 的查法是：

```cpp
static NodeContainer
GetGwNodesById(uint32_t gwId)
{
    auto topo = Singleton<SatTopology>::Get();
    NodeContainer gwNodes = topo->GetGwNodes();
    if (gwId >= gwNodes.GetN())
    {
        return NodeContainer();
    }
    return NodeContainer(gwNodes.Get(gwId));
}
```

也就是說，`test-iridium` 並不是自己建立 GW node，而是完全依賴：

```text
SatTopology::GetGwNodes()
```

如果 `SatTopology` 裡沒有把某個 physical GW node 註冊進去，`GetGwNodesById(gwId)` 就會直接拿不到。

原版 main 流程如下：

```cpp
Config::SetDefault("ns3::SatHelper::GwUsers", UintegerValue(3));
simHelper->SetBeamSet(std::set<uint32_t>{beamId});
simHelper->CreateSatScenario();

auto topo = Singleton<SatTopology>::Get();
NodeContainer physicalGwNodes = topo->GetGwNodes();
```

這段流程代表兩件事：

- `GW user` 的數量被設成固定 `3`
- 實際建立 scenario 時只啟用單一 `beamId`

因此後面能不能看到 physical GW node，關鍵不是 preset table 有沒有列出 GW，而是 `CreateSatScenario()` 期間到底有哪些 GW 被真正註冊進 `SatTopology`。

---

## beam / sat / GW 在 code 裡的真正關係
```text
先決定哪些 beam 被啟用
  -> 對每個 enabled beam 讀取 beam configuration
  -> 由 beam configuration 決定這個 beam 要使用哪個 GW
  -> 再建立 satellite / GW / UT 的鏈路
  -> 建鏈過程中把 GW node 註冊進 SatTopology
```

`beam` 不只是 sat 上的編號；它同時也是一個啟動 access / feeder link 安裝流程的索引。

---

## code trace

### 1. `sat` 的建立

`sat` 是在 SNS3 scenario 建構時建立的 constellation node。每顆衛星 node 會被放進 `SatTopology` 作為 orbiter node。

這代表：

- `sat` 是真實 node
- `beam` 不是真實 node
- `beam` 是附著在某顆衛星上的邏輯 coverage / channel configuration

所以真正的 beam 語意不是單獨存在，而是：

```text
(satId, beamId)
```

這一對才有完整語意。

### 2. `beam` 的啟用

原版 `test-iridium-e2e.cc`：

```cpp
simHelper->SetBeamSet(std::set<uint32_t>{beamId});
```

修正版 `test-iridium-e2e-fix.cc`：

```cpp
const std::set<uint32_t> enabledBeamSet =
    BuildGatewayBootstrapBeamSet(rtnConfFilePath, GetGatewayPresets().size(), beamId);

simHelper->SetBeamSet(enabledBeamSet);
```

這個差異非常重要。原版只告訴 SNS3：

```text
只建立一個 requested beam
```

修正版則是：

```text
除了 requested beam 外，再額外補進每個 GW 至少一個代表 beam
```

### 3. `beam -> GW` 的來源

修正版裡有一段非常關鍵的註解：

```cpp
// In constellation scenarios, SatConf::Initialize rewrites each beam's GW id as
// (rowIndex % gwCount) + 1. Mirror that runtime mapping here so the bootstrap
// beams actually cover every physical GW that SNS3 will register.
```

也就是說，constellation 模式下，執行期真正生效的 `beam -> GW` 關係，不是單純照 `rtnConf.txt` 裡原始的 `gwIdFromFile` 使用，而是會依照 runtime 規則重寫成：

```text
gwIdOneBased = (rowIndex % gatewayCount) + 1
```

因此修正版的 `BuildGatewayBootstrapBeamSet()`刻意 mirror SNS3 runtime 的 GW 分配規則，去挑出能覆蓋每個 GW 的 representative beam。

### 4. 為什麼開 beam 會帶出 GW

關鍵不在於「beam 物理上直接接到 GW」，而在於：

```text
某個 beam 的 install 流程需要知道它該用哪個 GW
```

所以在 scenario 建構期間，對每個 enabled beam，SNS3 會：

```text
取 beam configuration
  -> 找出 beam 對應的 gwId
  -> 從 gwNodes 裡取出該 gwNode
  -> 用該 gwNode 建立 feeder link / user link
  -> 把這個 gwNode 註冊進 topology
```

因此：

- 不是 beam 自己變成 GW
- 而是 beam 的 configuration 指定了「這個 beam 的 satellite access / feeder chain 要走哪個 GW」
- 所以啟用 beam 時，對應的 GW 會一起被拉進 scenario

### 5. `SatTopology` 與 `SatBeamHelper` 各自保存什麼

今天釐清的另一個重點是兩套映射不同：

- `SatBeamHelper` 偏向真實安裝語意：`(satId, beamId) -> gwId`
- `SatTopology` 偏向查詢語意：`beamId -> gwNode` 與 `gwId -> physical gw node`

因此如果要問：

```text
某顆衛星上的某個 beam 實際接到哪個 gateway？
```

真正完整的答案應該依賴：

```text
(satId, beamId) -> gwId
```

如果只看 `GetGwFromBeam(beamId)`，那只是較簡化的查詢表，語意不如前者完整。

---

## 原本抓不到其他 physical GW node

這是今天最重要的結論。

### 1. 原版只開單一 beam

原版：

```cpp
simHelper->SetBeamSet(std::set<uint32_t>{beamId});
```

若預設 `beamId = 72`，那就代表：

```text
CreateSatScenario() 期間只會對 beam 72 走完整的 install 流程
```

### 2. physical GW node 不是 create 出來就存在於 topology

雖然 SNS3 內部會先建立一批 candidate GW nodes，但 `test-iridium` 端真正使用的是：

```text
SatTopology::GetGwNodes()
```

而這個容器裡的 physical GW node，必須是那些在 beam install 過程中被真正註冊進 topology 的 GW。

所以會出現下面這種狀況：

```text
routing preset: gw0..gw4 都存在
candidate GW nodes: 可能都先建好了
physical GW nodes in SatTopology: 只有被 enabled beam 實際用到的那些
```

### 3. 單一 beam 只會覆蓋單一對應 GW

如果只開 `{72}`，那就只有 beam 72 對應到的那個 GW 會在 install 過程中被用到，因此只有那個 GW 會被註冊成 physical GW node。

後果就是：

```text
topo->GetGwNodes().GetN() 只會等於已註冊的 physical GW 數量
```

而 `GetGwNodesById(gwId)` 的實作是：

```cpp
if (gwId >= gwNodes.GetN())
{
    return NodeContainer();
}
```

所以當 `gwId = 1..4` 但 `physicalGwNodes.GetN()` 還很小時，就會直接回傳空的 `NodeContainer()`，看起來像「抓不到其他 GW node」。

這不是因為 `gw1..gw4` 不存在於 routing 邏輯中，而是因為它們沒有被實際註冊成 `SatTopology` 裡可供存取的 physical GW node。

### 4. 修正版怎麼解決

修正版做了兩件對症處理：

第一，先算出覆蓋每個 GW 的 bootstrap beam set：

```cpp
const std::set<uint32_t> enabledBeamSet =
    BuildGatewayBootstrapBeamSet(rtnConfFilePath, GetGatewayPresets().size(), beamId);
```

第二，把整包 beam set 丟給 SNS3：

```cpp
simHelper->SetBeamSet(enabledBeamSet);
```

這樣 `CreateSatScenario()` 就不只會 install primary beam 72，而是會 install：

```text
primary beam 72
+ 每個 GW 至少一個 bootstrap beam
```

結果就是每個 GW 都至少會被某個 enabled beam 用到一次，於是每個 GW 都會被註冊進 `SatTopology::GetGwNodes()`。

這也是為什麼修正版會印出：

```text
[TOPO_BOOTSTRAP] enabledBeams={1,2,3,4,5,72} gatewayPresets=5 primaryBeamId=72
[TOPO] physicalGwNodes=5
```

這裡的真正意義是：

```text
不是 routing preset 變多了
而是 physical GW node 終於被完整 bootstrap 進 topology 了
```

---

## `GwUsers` 為什麼也要一起改

修正版另外把：

```cpp
Config::SetDefault("ns3::SatHelper::GwUsers",
                   UintegerValue(static_cast<uint32_t>(GetGatewayPresets().size())));
```

從原本固定 `3` 改成和 preset 數量一致。

這一改不是造成 physical GW 註冊的主因，但它讓三條東西對齊：

- routing preset 的 gateway 數量
- physical GW node 的數量
- GW user node 的數量

如果只修 beam set，不修 `GwUsers`，還是可能出現：

```text
physical GW node 已經有 5 個
但 GW user node 仍只有 3 個
```

那後面在 app endpoint 或觀測流程上又會出現另一種不一致。

---

## 一句話總結

code trace 確認：在這個 repo 的 SNS3 constellation 流程中，`beam` 不只是 sat 上的波束編號，而是用來決定哪個 satellite coverage / feeder chain 要接哪個 GW的建立入口。原版 `test-iridium-e2e.cc` 只啟用單一 beam，因此只有那個 beam 對應到的 GW 會在 install 過程中被註冊成 `SatTopology` 裡的 physical GW node；其他 GW 雖然可能存在於 routing preset 或 candidate node 集合中，但沒有被實際 bootstrap 進 topology，所以 `GetGwNodesById(gwId)` 會抓不到。修正版透過 `BuildGatewayBootstrapBeamSet()` 額外開啟每個 GW 的 representative beam，才讓所有 physical GW node 都被完整註冊進 `SatTopology`。

---

## 第二次修正：beam 數精簡（效能修正）

### 問題

修正版原本把 `bootstrapGwCount` 設成 `GetGatewayPresets().size()` = 5，導致 5 個 GW 全部被 bootstrap，啟用 5–6 個 beam。

每多一個 beam，SNS3 會對 66 顆衛星各多建一套 MAC/DAMA/RBDC/TDMA timer 事件。  
結果 wall time 從 ~34 分鐘膨脹到預估 ~2.9–3.4 小時。

### 修正邏輯

`gw2gw_e2e` 實際只需要 gwSrc + gwDst 兩個 physical GW node，其餘 GW 的 beam 對路由和封包結果毫無影響。  
因此改為依 `pathType` 動態決定需要幾個 GW：

**修改位置：** `test-iridium-e2e-fix.cc` `main()` 中 beam 設定段落

```cpp
// Before（啟動全部 5 個 GW 的 bootstrap beam）
const std::set<uint32_t> enabledBeamSet =
    BuildGatewayBootstrapBeamSet(rtnConfFilePath, GetGatewayPresets().size(), beamId);

// After（只啟動本次 path 實際需要的 GW 數量）
// Only activate beams for the GWs actually used in this path (1 for single-GW paths,
// 2 for gw2gw). Activating all 5 GW beams multiplies satellite MAC/DAMA/RBDC events
// by ~5x and inflates wall time from ~12 min to ~3 hours.
const uint32_t bootstrapGwCount = GetPathTypeSpec(pathType).needsGwPair ? 2 : 1;
const std::set<uint32_t> enabledBeamSet =
    BuildGatewayBootstrapBeamSet(rtnConfFilePath, bootstrapGwCount, beamId);
```

```cpp
// Before（GwUsers 固定等於全部 preset 數量）
Config::SetDefault("ns3::SatHelper::GwUsers",
                   UintegerValue(static_cast<uint32_t>(GetGatewayPresets().size())));

// After（與 bootstrapGwCount 對齊）
// Match GwUsers to bootstrapGwCount: only create user-side GW nodes for active GWs.
const uint32_t gwUsersNeeded = GetPathTypeSpec(pathType).needsGwPair ? 2 : 1;
Config::SetDefault("ns3::SatHelper::GwUsers",
                   UintegerValue(gwUsersNeeded));
```

### pathType 對應表

| pathType | needsGwPair | bootstrapGwCount | GwUsers |
|----------|------------|------------------|---------|
| gw2gw_e2e | true | 2 | 2 |
| gw2sat / sat2gw / gw2ut_e2e / sat2ut | false | 1 | 1 |
| sat2sat | false | 1 | 1 |

---

## 模擬驗證

**執行指令：**
```bash
stdbuf -oL ./ns3 run 'scratch/test-iridium --pathType=gw2gw_e2e --gwMode=physical \
  --simTime=120 --trafficStop=119 --delayCsvPath=real_delay_physical.csv' \
  2>&1 | tee run_fix.log
```

**Log 來源：** `Topology & ISL Routing/Outputs/run_fix.log`

### Topology Bootstrap 確認

```
[TOPO_BOOTSTRAP] enabledBeams={1,2,72}  gatewayPresets=5  primaryBeamId=72
[TOPO] physicalGwNodes=2  utUserNodes=91
[TOPO] logicalGwId=0  gwUserNode=present  physicalGwNode=present
[TOPO] logicalGwId=1  gwUserNode=present  physicalGwNode=present
```

修正正確生效：beam 數從 5–6 降為 3，physical GW node 只建了 gwSrc=0 + gwDst=1 兩個。

### 路由結果

GW0(JP-Tokyo) → GW1(IN-NewDelhi)，3 個 slot 全部有效路由：

| slot | time(s) | ISL path | isl_cost(s) |
|------|---------|----------|-------------|
| 0 | 0 | 45→46→35→34→33 | 0.047382 |
| 1 | 60 | 15→14→3→4 | 0.037122 |
| 2 | 120 | 45→34→33 | 0.029282 |

slot=1, slot=2 都觸發 `HasSignificantChange=YES`，routing table 動態切換，recompute 分別 16/66 和 19/66 顆衛星。

### 三層 Verdict

```
[ROUTING_LAYER] PASS | validSlots=3/3
[ISL_LAYER]     PASS | scopedLinks=16  scopedRxPkts=511250
[PACKET_LAYER]  PASS | traceRxPkts=1181  traceRxBytes=604672  delaySamples=1181
```

封包：GW0(40.1.0.1) → GW1(40.67.0.1)，1181 封包送達，ISL 全程零掉包（8,266,570 pkts, drop_rate=0.000%）。

### Wall Time 對比

| 版本 | Beam 數 | Wall Time |
|------|---------|-----------|
| 修正前（全部 GW bootstrap） | 5–6 | 預估 ~2.9–3.4 小時 |
| **修正後（只 bootstrap 需要的 GW）** | **3** | **實測 3205.704s ≈ 53.4 分鐘** |

### 待確認項目

`avgOneWayDelayMs=0.010ms`（最小 0ms，最大 12ms）對於 Tokyo→NewDelhi 多跳 ISL 路由偏低，實際衛星傳播延遲預期應在 ~150–300ms 範圍。  
需進一步確認 `gwMode=physical` 下 `Gw2GwTxTimeTag` 是否確實量測到完整的 satellite propagation delay，而非僅計算 NS-3 internal 轉發時間。
