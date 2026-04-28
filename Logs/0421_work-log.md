# 2026-04-21 今日工作日誌：E2E Observability 與 Endpoint Probe

## 今日目標

今天的工作主軸是把 `test-iridium-e2e.cc` 的 E2E observability 從單一結果看起來 PASS/FAIL整理成可以分層定位問題的診斷流程。

核心目標：

- 修正 `gw2gw_e2e` verdict 語意，不再把 packet delivery 與 ISL transit 混在同一個 `LINK_LAYER` 判斷。
- 修正 route report 裡 `N/A` 的語意，把 valid route 但沒有 ISL hop 的情況明確標示成 `no ISL hop`。
- 新增 `--endpointProbe=1`，讓所有 `pathType` 都能輸出 endpoint/app delivery probe summary。
- 用六份 audit log 驗證 endpoint probe 的實際結果。
- 把目前尚未解決的 physical UT terminal API 查找列為下一步待辦。

---

## 修改檔案

主要修改：

- `Topology & ISL Routing/Codes/test-iridium-e2e.cc`
- `Topology & ISL Routing/Codes/isl-graph.cc`

今日也更新本工作日誌：

- `Logs/0421_work-log.md`

---

## Code 改變：`test-iridium-e2e.cc`

### 1. 新增 Endpoint Probe CLI

新增 CLI：

```text
--endpointProbe=0/1
--endpointProbePort=9100
```

用途：

- `--endpointProbe=1`：啟用 endpoint delivery probe。
- `--endpointProbePort=9100`：diagnostic `PacketSink` 使用的 UDP port，預設 `9100`。

`endpointProbePort` 加入合法性檢查：

```text
endpointProbePort must be in [1, 65535]
```

### 2. 新增 Endpoint Probe 統計結構

新增 endpoint probe state 與 target stats，用來分別紀錄 UT / GW / SAT target：

- PHY Rx packets / bytes
- MAC Rx packets / bytes
- NetDevice Rx packets / bytes
- App Rx packets / bytes
- PacketSink total Rx bytes
- trace 是否成功掛接
- app sink 是否成功安裝
- target 是否 not applicable
- interpretation

核心結構包含：

```cpp
EndpointLayerStats
EndpointAppStats
EndpointProbeTargetStats
EndpointProbeState
```

### 3. 新增 Trace Callback

新增 callback：

```cpp
EndpointLayerRxCb(...)
EndpointAppRxCb(...)
```

用途：

- `EndpointLayerRxCb`：統計 PHY / MAC / NetDevice layer 收到的 packets 與 bytes。
- `EndpointAppRxCb`：統計 diagnostic `PacketSink::Rx` 收到的 packets 與 bytes。

### 4. 新增 Device / MAC / PHY Probe 掛接

新增：

```cpp
ConnectEndpointDeviceProbe(...)
```

行為：

- 掃描 selected endpoint node 的所有 devices。
- 成功 `DynamicCast<SatNetDevice>` 後 best-effort 掛接：
  - `SatNetDevice::Rx`
  - `SatMac::Rx`
  - `SatPhy::Rx`
- 任一層缺失不 abort。
- 若該 node 沒有 `SatNetDevice`，summary 顯示：

```text
reason=satnetdevice_missing
interpretation=probe_connection_incomplete
```

### 5. 新增 Diagnostic App Sink

新增：

```cpp
InstallEndpointAppSink(...)
```

使用 SNS3 既有模式：

```cpp
PacketSinkHelper sink("ns3::UdpSocketFactory",
                      InetSocketAddress(satHelper->GetUserAddress(node), probePort));
```

設計決策：這個 sink 是 diagnostic-only。

它不會：

- 取代現有 traffic。
- 新增 sender。
- 改 routing。
- 改 traffic plan。
- 改任何既有 verdict。

因此如果現有流量沒有送到 `endpointProbePort`，app counter 可能是 0。這不是 probe failure，而是表示目前沒有 active traffic 打到 diagnostic sink port。

### 6. 新增 Endpoint Probe 安裝流程

新增：

```cpp
InstallEndpointProbe(...)
```

依 `pathType` 決定 probe target。

行為如下：

```text
gw2sat:
  probe GW source-side physical SatNetDevice/MAC/PHY
  no GW app sink
  SAT target marked satellite_orbiter_only

sat2gw:
  probe GW destination-side physical SatNetDevice/MAC/PHY
  install GW user PacketSink

sat2ut:
  install UT user PacketSink
  attempt UT endpoint device/MAC/PHY probe

gw2ut_e2e:
  install UT user PacketSink
  attempt UT endpoint device/MAC/PHY probe

gw2gw_e2e:
  install destination GW user PacketSink
  attempt physical GW probe if that physical GW node exists

sat2sat:
  mark endpoint_not_applicable reason=no_ground_endpoint
```

### 7. 新增 Endpoint Probe Summary

新增：

```cpp
PrintEndpointProbeSummary()
```

simulation end 時輸出：

```text
=== Endpoint Probe Summary ===
pathType=...
probePort=9100

[target=ut0]
phy connected=... rxPkts=... rxBytes=...
mac connected=... rxPkts=... rxBytes=...
dev connected=... rxPkts=... rxBytes=...
app installed=... traceConnected=... rxPkts=... traceRxBytes=... sinkTotalRxBytes=...
interpretation=...
```

目前支援 interpretation：

```text
app_delivery_observed
device_rx_observed_but_app_missing
mac_rx_observed_but_netdevice_rx_missing
phy_rx_observed_but_mac_rx_missing
device_rx_observed
endpoint_not_applicable
probe_connection_incomplete
no_endpoint_observed
```

### 8. SatStats Cross-Check 補強

`--satStats=1` 時，新增 broader endpoint-relevant SNS3 native stats：

```cpp
AddPerUtFwdAppThroughput
AddPerUtFwdUserPhyThroughput
AddPerUtFwdUserMacThroughput
AddPerUtFwdUserDevThroughput

AddPerUtRtnAppThroughput
AddPerUtRtnUserPhyThroughput
AddPerUtRtnUserMacThroughput
AddPerUtRtnUserDevThroughput

AddPerGwFwdAppThroughput
AddPerGwFwdUserMacThroughput
AddPerSatFwdAppThroughput
AddPerSatRtnAppThroughput
```

保留原本 feeder/user dev throughput stats，沒有移除既有 registrations。

### 9. `gw2gw_e2e` Verdict 語意修正

`gw2gw_e2e` 不再只輸出一個混合的 `LINK_LAYER` verdict。

現在拆成：

```text
[ROUTING_LAYER]
[ISL_LAYER]
[PACKET_LAYER]
```

理由：

- routing valid 是 routing 層證據。
- ISL scoped links / scoped Rx packets 是 ISL transit 證據。
- GW user PacketSink 收到 bytes 是 packet delivery 證據。

若 GW-to-GW route valid 但沒有 ISL hop，輸出：

```text
[ISL_LAYER] not_applicable | valid GW-GW route has no ISL hop
```

這避免把 no-hop valid route 誤判成 FAIL。

---

## Code 改變：`isl-graph.cc`

GW route report 中，route valid 但沒有 ISL transit path 時，將 `isl_cost` 顯示從：

```text
N/A
```

改成：

```text
no ISL hop
```

目的：

- `N/A` 容易被誤解成 route invalid 或 cost missing。
- `no ISL hop` 明確表示 route 是 valid，只是不需要經過 ISL transit。

---

## 設計決策

### Endpoint Probe 是 Diagnostic-Only

Endpoint probe 不參與既有 pass/fail semantics。

只回答：

- packet 有沒有到 PHY？
- packet 有沒有到 MAC？
- packet 有沒有到 NetDevice？
- packet 有沒有到 app sink？
- 如果沒有，第一個缺失層在哪裡？

不改變：

- `SERVICE_LAYER` verdict
- `FEEDER_LAYER` verdict
- `ISL_LAYER` verdict
- `PACKET_LAYER` verdict
- routing
- traffic plan
- sender 行為

### App Sink 不新增 Sender

這次沒有新增 active endpoint probe sender。

原因：

- 目前需求是診斷既有 pathType，不改 traffic plan。
- 如果新增 sender，可能改變 load、routing behavior 或 pass/fail interpretation。
- 因此 app sink 收不到 packet 時，不能直接視為 endpoint delivery FAIL；要搭配 PHY/MAC/DEV counters 解讀。

### UT Endpoint Probe 暫時暴露出 Node Selection 問題

目前 UT app sink 安裝在：

```cpp
SatTopology::GetUtUserNodes().Get(utId)
```

這對 app sink 是合理的，因為 user address 在 user node 上。

但 audit log 顯示此 node 沒有 `SatNetDevice`，所以不能直接在這個 node 掛 PHY/MAC/DEV trace。

因此下一步需要找 SNS3 physical UT terminal API，把 UT endpoint probe 拆成：

```text
appNode    = UT user node
deviceNode = physical UT terminal node
```

---

## Audit Log 驗證結果

本次檢查六份 log：

```text
Topology & ISL Routing/Outputs/audit/v4/sat2sat.log
Topology & ISL Routing/Outputs/audit/v4/sat2ut.log
Topology & ISL Routing/Outputs/audit/v4/sat2gw.log
Topology & ISL Routing/Outputs/audit/v4/gw2sat.log
Topology & ISL Routing/Outputs/audit/v4/gw2gw_e2e.log
Topology & ISL Routing/Outputs/audit/v4/gw2ut_e2e.log
```

共同結果：

- 六種 `pathType` 都成功輸出 `Endpoint Probe Summary`。
- 六種 `pathType` 都成功 register `--satStats=1` 的 broader endpoint stats。
- endpoint probe 沒有改變既有 E2E verdict。

### `sat2sat.log`

Verdict：

```text
[FEEDER_LAYER] not_applicable | not part of sat2sat
[SERVICE_LAYER] not_applicable | not part of sat2sat
[ISL_LAYER] PASS | scopedLinks=1 scopedRxPkts=23974
```

Endpoint summary：

```text
[target=sat]
endpointProbe=not_applicable reason=no_ground_endpoint
interpretation=endpoint_not_applicable
```

結果：符合預期。`sat2sat` 沒有 ground endpoint，不應被 endpoint probe 判成 FAIL。

### `gw2sat.log`

Verdict：

```text
[FEEDER_LAYER] PASS | scopedKeys=2 scopedRxPkts=32087
```

Endpoint summary 重點：

```text
[target=gw0]
phy connected=1 rxPkts=10347 rxBytes=2821498
mac connected=1 rxPkts=10347 rxBytes=2821498
dev connected=1 rxPkts=5148 rxBytes=2779920
app installed=0
interpretation=device_rx_observed

[target=sat]
endpointProbe=not_applicable reason=satellite_orbiter_only
interpretation=endpoint_not_applicable
```

結果：GW physical endpoint 可觀測，SAT target 正確標示 orbiter-only。

### `sat2gw.log`

Verdict：

```text
[FEEDER_LAYER] PASS | scopedKeys=1 scopedRxPkts=5148
```

Endpoint summary 重點：

```text
[target=gw0]
phy connected=1 rxPkts=10347 rxBytes=2821498
mac connected=1 rxPkts=10347 rxBytes=2821498
dev connected=1 rxPkts=5148 rxBytes=2779920
app installed=1 traceConnected=1 rxPkts=0 traceRxBytes=0 sinkTotalRxBytes=0
interpretation=device_rx_observed_but_app_missing
```

結果：packet 到了 GW physical NetDevice，但沒有進入 diagnostic app sink。

解讀：這不直接等於資料面 FAIL，因為 diagnostic sink 使用 `9100`，而目前沒有新增 sender 保證 traffic 打到該 port。

### `sat2ut.log`

Verdict：

```text
[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2304 utKeys=1 utRxPkts=0 endpoint_dev_rx=not_observed
```

Endpoint summary 重點：

```text
[target=ut0]
phy connected=0 rxPkts=0 rxBytes=0
mac connected=0 rxPkts=0 rxBytes=0
dev connected=0 rxPkts=0 rxBytes=0
app installed=1 traceConnected=1 rxPkts=0 traceRxBytes=0 sinkTotalRxBytes=0
reason=satnetdevice_missing
interpretation=probe_connection_incomplete
```

結果：UT app sink 安裝成功，但同一個 UT user node 沒有 `SatNetDevice`。

結論：目前 UT endpoint probe 的 node selection 不完整，需要找到 physical UT terminal node。

### `gw2ut_e2e.log`

Verdict：

```text
[FEEDER_LAYER] PASS | scopedKeys=1 scopedRxPkts=1199
[SERVICE_LAYER] PASS | satKeys=1 satRxPkts=2304 utKeys=1 utRxPkts=0 endpoint_dev_rx=not_observed
[ISL_LAYER] not_applicable | valid route has no ISL hop
```

Endpoint summary 重點：

```text
[target=ut0]
phy connected=0
mac connected=0
dev connected=0
app installed=1 traceConnected=1
reason=satnetdevice_missing
interpretation=probe_connection_incomplete
```

結果：和 `sat2ut` 一樣，問題集中在 UT physical device node 沒被取得。

### `gw2gw_e2e.log`

Verdict：

```text
[ROUTING_LAYER] PASS | validSlots=3/3 gwSrc=0 gwDst=1 obsFeederMode=ROUTING
[ISL_LAYER] PASS | scopedLinks=16 scopedRxPkts=387851
[PACKET_LAYER] PASS | appInstalled=1 traceConnected=1 traceRxPkts=1179 traceRxBytes=603648 reported=1 sinkTotalRxBytes=603648 sinkEstPkts=1179
```

Endpoint summary 重點：

```text
[target=gw1]
phy connected=0
mac connected=0
dev connected=0
app installed=1 traceConnected=1 rxPkts=0 traceRxBytes=0 sinkTotalRxBytes=0
reason=physical_gw_node_missing
interpretation=probe_connection_incomplete
```

Topology log：

```text
physicalGwNodes=1
logicalGwId=0 gwUserNode=present physicalGwNode=present
logicalGwId=1 gwUserNode=present physicalGwNode=not_present
```

結果：logical GW1 有 user node，但沒有 physical GW node。這不影響 `PACKET_LAYER` verdict，因為 GW-to-GW app delivery 已經 PASS。

---

## 今日結論

1. Endpoint probe 啟用條件已達成：所有 `pathType` 都印出 `Endpoint Probe Summary`。
2. `sat2sat` 正確標示 endpoint 不適用，沒有被誤判 FAIL。
3. GW physical endpoint probe 可用，`gw2sat` 與 `sat2gw` 都能看到 PHY/MAC/DEV counters。
4. `sat2gw` 顯示 packet 到 GW NetDevice，但 diagnostic app sink 沒收到；這與 sink port / no active sender 設計有關，不直接視為 verdict FAIL。
5. `sat2ut` 與 `gw2ut_e2e` 的第一個明確問題是 `satnetdevice_missing`：目前掛 probe 的 UT user node 沒有 `SatNetDevice`。
6. `gw2gw_e2e` 已能同時證明 routing、ISL scoped observation 與 packet delivery；physical GW endpoint probe 對 logical GW1 不適用，因為 topology 沒有 physical GW1 node。

---

## 待辦：Physical UT API

下一步要找 SNS3 physical UT terminal API。

目前 workspace 搜尋只看到：

```cpp
SatTopology::GetUtUserNodes()
```

尚未找到明確 physical UT API，例如：

```cpp
GetUtNodes()
GetUtNode(...)
GetUtNodes(utUserNode)
GetUtNodeFromUserNode(...)
```

建議在 SNS3 環境執行：

```bash
rg -n "GetUt.*Node|UtNodes|UtUserNodes|GetGwNodes|GetGwUserNode" contrib/satellite src scratch
```

找到 API 後，UT endpoint probe 應改成：

```text
appNode    = Singleton<SatTopology>::Get()->GetUtUserNodes().Get(utId)
deviceNode = physical UT terminal node for utId
```

然後：

- app sink 安裝在 `appNode`。
- PHY/MAC/DEV trace 掛在 `deviceNode`。

驗證目標：

- `sat2ut` / `gw2ut_e2e` 不再顯示 `satnetdevice_missing`。
- UT PHY/MAC/DEV 至少能成功 `connected=1`。
- 若 counters 仍為 0，再判斷是 traffic 未到 endpoint、trace source 不在該層、或 app delivery port 不一致。

---

## 後續可能選項

### Option A：保持 diagnostic-only

維持目前設計，不新增 sender。

優點：

- 不改 traffic plan。
- 不影響 verdict。
- 適合定位現有 traffic path。

限制：

- app sink counter 可能為 0，因為沒有流量打到 `endpointProbePort`。

### Option B：新增 optional active endpoint probe sender

新增類似：

```text
--endpointProbeActive=1
```

讓 probe 自己送少量 UDP 到 diagnostic sink。

優點：

- app delivery 可以被主動驗證。

風險：

- 會改變 traffic load。
- 可能影響 routing / queue / verdict interpretation。
- 必須明確標示 active probe 不參與原 verdict。

目前決策：先保持 Option A，等 physical UT API 找到後再決定是否需要 Option B。

---

## 本地檢查狀態

已執行：

```powershell
git diff --check -- 'Topology & ISL Routing/Codes/test-iridium-e2e.cc' 'Topology & ISL Routing/Codes/isl-graph.cc'
```

結果：

- 沒有 whitespace error。
- 只有 LF/CRLF warning。

本機 workspace 沒有找到：

```text
ns3
ns3.exe
waf
CMakeLists.txt
```

因此今天的 simulation 結果是根據使用者提供的 SNS3 audit logs 分析。
