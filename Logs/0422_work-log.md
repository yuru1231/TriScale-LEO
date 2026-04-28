# 2026-04-22 ：UT Endpoint Delivery 與 Logical/Scenario UT 對齊

## 目標

把 `sat2ut` / `gw2ut_e2e` 的 service-link endpoint delivery 從衛星端有觀測到封包推進到UT endpoint 確實收到封包。

核心目標：

- 修正 `SatTopology::GetUtUserNodes()` 與 physical UT node 的使用邊界。
- 讓 `sat2ut` / `gw2ut_e2e` 的 `SERVICE_LAYER` verdict 不再只靠 satellite-side service trace 判斷 PASS。
- 找出 `service:sat15 > 0` 但 `service:ut0 = 0` 的資料面原因。
- 修復 logical routing UT 與 SNS3 scenario UT user index 不一致的問題。
- 用 audit log 驗證 `sat2ut` 與 `gw2ut_e2e` 最終都能在 UT endpoint 收到封包。

---

## 修改檔案

主要修改：

- `Topology & ISL Routing/Codes/test-iridium-e2e.cc`

---

## 問題背景

一開始從 `SatTopology` API 使用點整理出幾個重要區分：

```text
GetUtUserNodes() / GetUtUserNode()
  -> 使用者端節點，適合安裝 app / traffic endpoint

GetUtNode(utUserNode) / GetUtNodes()
  -> physical UT terminal node，才有 SatNetDevice / MAC / PHY

GetGwUserNode()
  -> GW user endpoint

GetGwNodes()
  -> physical GW node，才有 SatNetDevice
```

早期 endpoint probe 直接在 UT user node 上找 `SatNetDevice`，這會導致 probe 掛不到 physical UT device。後續修成：

```cpp
Ptr<Node> utUserNode = GetUtUserNodeOrNull(utId);
Ptr<Node> physicalUtNode = Singleton<SatTopology>::Get()->GetUtNode(utUserNode);

ConnectEndpointDeviceProbe(physicalUtNode, ...);
InstallEndpointAppSink(simHelper, utUserNode, ...);
```

這個修正讓 probe 能成功掛到 physical UT node，但 v6 驗證後又暴露出更深一層的問題：probe 掛上了，UT endpoint 還是沒有收到封包。

---

## 觀測層級架構圖

目前 E2E observability 分成兩套互補機制：

```text
OBS / Verdict
  -> 判斷 path segment 是否 PASS
  -> 輸出 FEEDER_LAYER / SERVICE_LAYER / ISL_LAYER / PACKET_LAYER / ROUTING_LAYER

Endpoint Probe
  -> 診斷 ground endpoint 內部封包落點
  -> 輸出 PHY / MAC / NetDevice / App counters
```

整體層級由低到高可整理成：

```text
PHY
  -> SatPhy::Rx
  -> EndpointLayerRxCb
  -> endpoint summary: phy connected / rxPkts / rxBytes

MAC
  -> SatMac::Rx
  -> EndpointLayerRxCb
  -> endpoint summary: mac connected / rxPkts / rxBytes

NetDevice / Dev
  -> SatNetDevice::Rx
  -> EndpointLayerRxCb
  -> endpoint summary: dev connected / rxPkts / rxBytes

Feeder Link
  -> GW <-> SAT feeder segment
  -> verdict: FEEDER_LAYER

Service Link
  -> SAT <-> UT service segment
  -> verdict: SERVICE_LAYER

ISL Link
  -> SAT <-> SAT inter-satellite segment
  -> verdict: ISL_LAYER

Routing / Scope
  -> IslRoutingManager route lookup / TracePath
  -> 決定哪些 feeder/service/isl keys 算入 verdict
  -> verdict: ROUTING_LAYER, mainly gw2gw_e2e

Application / Packet
  -> PacketSink::Rx
  -> EndpointAppRxCb or Gw2GwAppRxCb
  -> verdict: PACKET_LAYER, mainly gw2gw_e2e
```

目前具體觀測點如下：

```text
Layer / Segment         Object                         Trace / Callback
--------------------------------------------------------------------------------
PHY                     SatPhy                         Rx / EndpointLayerRxCb
MAC                     SatMac                         Rx / EndpointLayerRxCb
NetDevice               SatNetDevice                   Rx / EndpointLayerRxCb
Feeder GW->SAT          SatOrbiterNetDevice             RxFeeder / OrbiterRxFeederCb
Feeder SAT->GW          GW physical SatNetDevice        Rx / GatewayRxFeederCb
Service satellite-side  SatOrbiterNetDevice             RxUser / OrbiterRxUserCb
Service UT endpoint     UT physical SatNetDevice        Rx / UtRxServiceCb
ISL                     ISL NetDevice                   PacketDropRateTrace / IslObsCb
Routing                 IslRoutingManager               GetGwUtRoute / GetGwRoute / TracePath
Application             PacketSink                      Rx / EndpointAppRxCb, Gw2GwAppRxCb
```

關鍵語意：

```text
service:satXX > 0
  -> 衛星側 service-link visibility 有看到封包
  -> 不能單獨證明 SAT->UT endpoint delivery 成功

service:utXX > 0
  -> physical UT SatNetDevice::Rx 有看到封包
  -> 對 sat2ut / gw2ut_e2e 是 SERVICE_LAYER PASS 的主要依據

endpoint phy/mac/dev rxPkts > 0
  -> endpoint probe 在 physical endpoint 逐層觀測到封包
  -> 可輔助確認封包已抵達正確 UT/GW endpoint

endpoint app rxPkts = 0
  -> 不一定代表資料面失敗
  -> diagnostic PacketSink 使用 endpointProbePort=9100
  -> 若主流量沒有打到該 port，app 會 idle
```

因此今天修正後，`sat2ut` / `gw2ut_e2e` 的核心判斷不再停在：

```text
service:sat15 > 0
```

而是要求：

```text
service:ut{trafficUtUserId} > 0
endpoint target ut{trafficUtUserId} dev/mac/phy rxPkts > 0
```

## v6 問題現象

`Topology & ISL Routing/Outputs/audit/v6/sat2ut.log`：

```text
service:sat15           2309
service:ut0             0

[SERVICE_LAYER] FAIL | satKeys=1 satRxPkts=2309 utKeys=1 utRxPkts=0 endpoint_dev_rx=not_observed

[target=ut0]
phy connected=1 rxPkts=0
mac connected=1 rxPkts=0
dev connected=1 rxPkts=0
interpretation=no_endpoint_observed
```

`Topology & ISL Routing/Outputs/audit/v6/gw2ut.log`：

```text
feeder:sat15            1199
service:sat15           2309
service:ut0             0

[FEEDER_LAYER] PASS | scopedKeys=1 scopedRxPkts=1199
[SERVICE_LAYER] FAIL | satKeys=1 satRxPkts=2309 utKeys=1 utRxPkts=0 endpoint_dev_rx=not_observed
```

這證明：

- UT device probe 已成功連接：`phy/mac/dev connected=1`
- 但選到的 `ut0` endpoint 沒收到封包：`rxPkts=0`
- 問題不是 probe 掛不上，而是 traffic endpoint 與 routing logical UT 沒有對齊

---

## 修正一：SERVICE_LAYER Verdict 收緊

原本 `SERVICE_LAYER` 只要 satellite-side 或 UT-side 有任一 service rx，就可能 PASS。

這對 `sat2ut` / `gw2ut_e2e` 太寬鬆，因為這兩個 path type 的 service-link 重點是：

```text
SAT -> UT endpoint delivery
```

修正後：

```cpp
bool servicePass = rxPkts > 0;
if (cfg.pathType == "sat2ut" || cfg.pathType == "gw2ut_e2e")
{
    servicePass = (utRxPkts > 0);
}
```

效果：

- `sat2ut` / `gw2ut_e2e` 必須 `utRxPkts > 0` 才能 PASS。
- 若只有 `satRxPkts > 0`、`utRxPkts == 0`，會輸出 FAIL。
- `endpoint_dev_rx=not_observed` 明確指出 endpoint 未觀測到封包。

---

## 修正二：Endpoint Probe Interpretation 語意修正

原本 device 有收包但 diagnostic app 沒收包時，會輸出：

```text
device_rx_observed_but_app_missing
```

這個說法不準，因為 app sink 其實有安裝，只是主流量沒有打到 diagnostic `probePort=9100`。

修正後：

```text
device_rx_observed_probe_app_idle
```

如果真的沒有 app sink，才輸出：

```text
device_rx_observed_app_not_installed
```

---

## 修正三：no-ISL-hop 不再誤報 Empty Scope Warning

`gw2ut_e2e` route report 顯示：

```text
entry = 15
serving = 15
isl_cost = no ISL hop
```

這代表 route valid，但不需要經過 ISL。因此 `islKeys=0` 是合理狀況，不應輸出：

```text
[OBS] WARNING: active segment has empty scope
```

修正後改為：

```text
[OBS] ISL scope empty because valid route has no ISL hop
```

並且不再把這種正常狀況當 warning。

---

## 根因：Logical UT 與 Scenario UT User Index 混用

真正資料面問題是 `cfg.utId` 被同時拿去做兩件不同的事：

```text
cfg.utId = 0

1. routingMgr 裡的 logical UT0 = UT-Taipei
2. SatTopology::GetUtUserNodes()[0] = SNS3 scenario 裡第 0 個 UT user
```

但 v6 log 顯示 `GetUtUserNodes()[0]` 並不是 route serving satellite 下會收到封包的那個 scenario UT user。

因此需要拆成：

```text
logicalUtId      = cfg.utId
trafficUtUserId  = 自動解析出的 scenario UT user index
```

---

## 修正四：自動解析 Traffic UT User

新增欄位：

```cpp
uint32_t trafficUtUserId{0};
bool     trafficUtUserIdResolved{false};
```

新增 resolver：

```cpp
static uint32_t
ResolveTrafficUtUserId(Ptr<IslRoutingManager> routingMgr,
                       const E2EConfig&       cfg,
                       uint32_t               numSlots)
```

解析流程：

1. 用 `routingMgr->GetGwUtRoute(cfg.gwId, cfg.utId, k)` 找第一個 valid route。
2. 讀出該 route 的 `servingSatId`。
3. 掃描 `Singleton<SatTopology>::Get()->GetUtUserNodes()`。
4. 對每個 UT user 反查 physical UT：

```cpp
Ptr<Node> utNode = topo->GetUtNode(utUsers.Get(i));
```

5. 用 physical UT 查當前 serving satellite：

```cpp
topo->GetUtSatId(utNode)
```

6. 找到 `GetUtSatId(utNode) == servingSatId` 的 scenario UT user index。

成功時輸出：

```text
[UT_SELECT] logicalUtId=0 routeServingSat=15 trafficUtUserId=22 requestedUtUserId=0
```

---

## 修正五：Traffic / Probe / OBS Key 改用 trafficUtUserId

修正後保留：

```text
cfg.utId
  -> routing logical UT
```

改用：

```text
cfg.trafficUtUserId
  -> SNS3 traffic endpoint
  -> endpoint probe target
  -> UT-side OBS key
```

主要替換：

```cpp
GetSelectedUtUser(cfg.trafficUtUserId)
ActivateUtEndpointProbe(simHelper, cfg.trafficUtUserId, stopSec)
MakeUtKey(cfg.trafficUtUserId)
```

E2E banner 也新增輸出：

```text
logicalUtId=0 trafficUtUserId=22
```

---

## 最終驗證：sat2ut PASS

最新驗證檔：

- `Topology & ISL Routing/Outputs/audit/v5/sat2ut.log`

關鍵輸出：

```text
[UT_SELECT] logicalUtId=0 routeServingSat=15 trafficUtUserId=22 requestedUtUserId=0

[E2E] pathType=sat2ut ... logicalUtId=0 trafficUtUserId=22

service:sat15           2683      147492
service:ut22            1189      1816792

[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2683 utKeys=1 utRxPkts=1189

[target=ut22]
phy connected=1 rxPkts=25163 rxBytes=30587358
mac connected=1 rxPkts=1189 rxBytes=1829871
dev connected=1 rxPkts=1189 rxBytes=1816792
interpretation=device_rx_observed_probe_app_idle
```

結論：

```text
sat2ut service endpoint delivery 已修復。
```

---

## 最終驗證：gw2ut_e2e PASS

最新驗證檔：

- `Topology & ISL Routing/Outputs/audit/v5/gw2ut_e2e.log`

關鍵輸出：

```text
[UT_SELECT] logicalUtId=0 routeServingSat=15 trafficUtUserId=22 requestedUtUserId=0

[E2E] pathType=gw2ut_e2e ... logicalUtId=0 trafficUtUserId=22

feeder:sat15            2388      4065650
service:sat15           2683      147492
service:ut22            1189      1816792

[FEEDER_LAYER] PASS | scopedKeys=1 scopedRxPkts=2388
[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2683 utKeys=1 utRxPkts=1189
[ISL_LAYER] not_applicable | valid route has no ISL hop

[target=ut22]
phy connected=1 rxPkts=25163 rxBytes=30587358
mac connected=1 rxPkts=1189 rxBytes=1829871
dev connected=1 rxPkts=1189 rxBytes=1816792
interpretation=device_rx_observed_probe_app_idle
```

結論：

```text
gw2ut_e2e feeder + service endpoint delivery 已修復。
```

---

## 其他清理

清掉兩個未使用 helper，避免編譯 warning 污染 log：

```text
SumScopedRxPkts
SumScopedRxPktsWithPrefix
```

驗證：

```powershell
git diff --check -- "Topology & ISL Routing/Codes/test-iridium-e2e.cc"
```

結果：

```text
通過，僅剩既有 LF/CRLF 提醒。
```

---

## 今日結論

今天完成的核心修復：

```text
logical UT routing id 與 SNS3 scenario UT user endpoint 已解耦。
```

修復前：

```text
logicalUtId=0
traffic endpoint=GetUtUserNodes()[0]
service:ut0 = 0
SERVICE_LAYER FAIL
```

修復後：

```text
logicalUtId=0
routeServingSat=15
trafficUtUserId=22
service:ut22 = 1189
SERVICE_LAYER PASS
```

最終狀態：

- `sat2ut`：PASS
- `gw2ut_e2e`：PASS
- UT physical device/MAC/PHY endpoint：有收包
- diagnostic app sink：idle，原因是主流量沒有送到 `probePort=9100`，不是錯誤
- no-ISL-hop：正確標示為 `not_applicable`

---

## 後續注意事項

- `trafficUtUserId` 目前以第一個 valid slot 的 serving satellite 為準。若長時間模擬中 serving satellite 切換，未來可擴充成 slot-aware traffic/probe mapping。
- `out-of-scope feeder hits` 目前仍作為診斷資訊保留，不影響 verdict。
- `sat2ut` / `gw2ut_e2e` 的 endpoint app probe 若要觀測 app 層，需要讓 diagnostic sender 實際打到 `probePort=9100`；目前它主要用來確認 app sink 可安裝，device/MAC/PHY counters 才是 service endpoint delivery 的主判斷。
