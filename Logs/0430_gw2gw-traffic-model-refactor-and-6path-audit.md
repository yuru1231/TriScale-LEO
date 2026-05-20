# 2026-04-30 工作日誌 
gw2gw_e2e 流量模型重構 & 6-PathType 全面稽核

## 今日目標

1. 將 `gw2gw_e2e` 的流量發送模型從自訂 `UdpSocket` CBR loop 改為與其他 path 同型的 `OnOffHelper` + `PacketSinkHelper`
2. 對所有 6 個 PathType 進行層次一致性、PHY/MAC 問題、傳播延遲準確性、ns-3 事件真實性的全面稽核

---

## 完成項目

### 1. gw2gw_e2e 流量模型重構（過渡狀態紀錄）

> ⚠️ **設計決策說明**：本系統以 **measurement correctness（量測正確性）為優先**，
> 因此允許 `gw2gw_e2e` 使用與其他 path 不同的 custom traffic model（raw socket + tagged CBR）。
> 流量模型一致性目標退讓給 per-packet delay 量測需求。
> Session 1 嘗試改為 OnOffHelper（為了模型一致性），但 Session 2 基於上述原則撤銷並還原。
> 以下紀錄為中間過渡狀態（**非最終狀態**），**最終 code 見 Session 2 修正 B**。

#### 修改前（Session 1 起點）

`InstallGw2GwApplicationTraffic` 使用：
- 手動建立 `UdpSocketFactory` raw socket
- `Simulator::Schedule` 遞迴驅動的 CBR loop（`SendGw2GwTaggedPacket`）
- 自訂 `Gw2GwTxTimeTag` 加在每個封包上記錄發送時間

此做法與 `gw2ut_e2e`、`sat2ut`、`sat2gw` 等 path 的 `OnOffHelper` 模式不一致，
造成流量特性不可比較、debug 行為不一致。

#### Session 1 修改（過渡）→ Session 2 撤銷

臨時改為 `OnOffHelper` + `PacketSinkHelper`，與其他 path 同型：

```cpp
// Sink（目的 GW）
PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
ApplicationContainer sinkApps = sink.Install(dstNode);
sinkApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&Gw2GwAppRxCb));

// Source（OnOffHelper，永遠開啟）
OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstAddr, port));
onoff.SetAttribute("PacketSize", UintegerValue(512));
onoff.SetAttribute("DataRate",   DataRateValue(DataRate("40960bps")));  // 512B × 10pkt/s
onoff.SetAttribute("OnTime",     StringValue("ns3::ConstantRandomVariable[Constant=1000000]"));
onoff.SetAttribute("OffTime",    StringValue("ns3::ConstantRandomVariable[Constant=0]"));
onoff.SetAttribute("Local",      AddressValue(InetSocketAddress(srcAddr, 0)));  // 綁定 feeder IP

ApplicationContainer srcApps = onoff.Install(srcNode);
srcApps.Start(Seconds(startSec));
srcApps.Stop(Seconds(stopSec));
```

`Local` attribute 綁定 srcAddr（40.x.x.x，feeder link IP），確保 routing 選擇 feeder interface 而非 backbone。

---

#### Session 1 同步移除的 Dead Code（部分於 Session 2 還原）

| 移除項目 | 原因 | 最終狀態 |
|----------|------|----------|
| `SendGw2GwTaggedPacket` 函式整體 | OnOffHelper 化後此函式不再被呼叫（Session 1 暫時移除） | ⚠️ **Session 2 還原**：raw socket 模式下必須存在以注入 `Gw2GwTxTimeTag` |
| `for (uint32_t i = 0; false && ...)` routing 搜尋 loop | 原為搜尋 feeder gateway IP 的廢棄邏輯，以 `false &&` 停用但未刪除；ARP prefill 方案已取代此做法 | ✅ 永久移除 |
| `feederGatewayIp`、`feederGatewayFound` 變數 | 宣告後從未被使用，是已廢棄方案的殘留 | ✅ 永久移除 |

保留的 ARP 修正邏輯
```cpp
// ARP prefill 讓 AddHostRouteTo(gw=0.0.0.0) 不觸發 ARP request crash
PrefillArpEntry(srcNode, gwSrc, srcFeeder, dstAddr, dstGwMac);
staticRouting->AddHostRouteTo(dstAddr, feederIfIndex);
```

---

### 2. 6-PathType 全面稽核

對 `gw2gw_e2e`、`gw2ut_e2e`、`sat2ut`、`sat2gw`、`isl_background`、`sat2ut_app` 六個 path type 進行稽核，
稽核項目包含：流量模型層次、observer 所在層、PHY/MAC 一致性、傳播延遲來源、ns-3 事件真實性。

#### 稽核結果摘要

| PathType | 流量模型 | Observer 層 | PHY→MAC 問題 | 傳播延遲 | 已知問題 |
|----------|----------|------------|--------------|---------|----------|
| `gw2gw_e2e` | raw socket + `SendGw2GwTaggedPacket`（最終狀態；Session 1 改過 OnOffHelper 後 Session 2 撤銷） | App-layer Rx | 無 | ISL: `PropagationDelayTrace`（channel-level，真實）；Feeder: `RxFeederLinkDelay` tag | E2E delay 已可量測（Session 2 恢復 `Gw2GwTxTimeTag`）；FEEDER / PACKET 待驗證 |
| `gw2ut_e2e` | OnOffHelper | App-layer Rx + orbiter RxUser | 無 | Feeder: `RxFeederLinkDelay` tag；Service: `RxUserLinkDelay` tag | FWD service delay gap：`OrbiterUserDelayCb` 只在 RTN 方向 fire |
| `sat2ut` | SatTrafficHelper::AddCbrTraffic | `SatNetDevice::Rx` at UT (FWD) | 無 | Service FWD: `RxUserLinkDelay` tag at orbiter（RTN only），FWD UT-side 無 delay tag | FWD service delay 無法在 UT 端量到 |
| `sat2gw` | SatTrafficHelper::AddCbrTraffic | `GatewayRxFeederCb`（feeder downlink） | 無 | Feeder RTN: `RxFeederLinkDelay` tag | **流量方向問題**：預設 traffic 可能是 FWD，`GatewayRxFeederCb` 觀察 RTN feeder downlink，若未啟用 RTN traffic 則 callback 永遠不 fire |
| `isl_background` | OnOffHelper（P2P ISL）| `PointToPointIslChannel::PropagationDelayTrace` | 無 | ISL: channel-level 真實 propagation delay | Observer 為 aggregate rate（`PacketDropRateTrace`），非 per-packet |
| `sat2ut_app` | 自訂 App（SNS3 application layer） | App-layer Rx at UT | 無 | 依 SNS3 saturation helper | 無 per-packet delay；aggregate 統計 |

#### PHY/MAC 問題結論

**6 個 path type 均無 PHY→MAC 層次混用問題。**

- Feeder 觀測點：`SatOrbiterNetDevice::RxFeeder`（L2 MAC receive，非 PHY raw bits）
- Service 觀測點：`SatNetDevice::Rx` at UT / `SatNetDevice::Rx` at GW（L2 MAC receive）
- ISL 觀測點：`PointToPointIslChannel::PropagationDelayTrace`（channel 事件，真實模擬觸發）
- App 觀測點：`PacketSinkHelper::Rx`（app layer，L4 以上）

per-packet 觀測點均在 ns-3 事件系統中真實觸發，非模擬推算值。
例外：`PacketDropRateTrace`（isl_background）為 timer-based aggregate，非 packet-level event，
不適用於 per-hop delay 分析。

#### 傳播延遲準確性結論

| 觀測源 | 準確性 | 說明 |
|--------|--------|------|
| `RxFeederLinkDelay` tag | ✅ 真實 | SNS3 channel 層在封包進入 satellite 時加上，來自 SGP4 mobility model 計算的當前距離 |
| `RxUserLinkDelay` tag | ✅ 真實（RTN only） | 同上，但僅在 RTN 方向（UT→SAT）有效；FWD 方向 orbiter 不 fire |
| `PropagationDelayTrace` at ISL | ✅ 真實 | `PointToPointIslChannel` 每次 packet 傳輸計算 `GetDelay()`，channel-level event |
| `Gw2GwTxTimeTag` | ✅ 真實 | Session 1 因 OnOffHelper 化而移除（`delaySamples` 恆 0）；**Session 2 已還原**（raw socket 模式），delay 量測現在可用 |
| `PacketDropRateTrace` | ⚠️ Aggregate | 每 interval 統計，非 per-packet，不能做 per-hop delay 分析 |

---

## Bug 列表與修正狀態

### Bug 1：gw2gw_e2e 流量模型與其他 path 不同型 ✅ 已修正

**問題**：`InstallGw2GwApplicationTraffic` 使用 raw socket + `SendGw2GwTaggedPacket` CBR loop，
與其他 path type 使用 `OnOffHelper` 的模式不一致。

**修正**：改為 `OnOffHelper` + `PacketSinkHelper`，統一流量模型。

**影響檔案**：`Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc`

> ⚠️ **注意（Session 2 撤銷）**：此 OnOffHelper 改動已於 Session 2「修正 B」撤銷，
> 還原為 raw socket + `SendGw2GwTaggedPacket` 模式，以恢復 per-packet E2E delay 量測能力。
> 本節記錄為中間過渡狀態，**非最終狀態**，請見 Session 2 修正 B。

---

### Bug 2：Dead code 殘留 ✅ 已修正

**問題**：`SendGw2GwTaggedPacket`、`for (false && ...)` loop、`feederGatewayIp` 變數
均為廢棄邏輯殘留，造成 compiler warning 與閱讀雜訊。

**修正**：全部移除。

**影響檔案**：`Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc`

---

## 驗證方式

### 重構後基本編譯確認

```bash
cd $NS3_ROOT
./ns3 build test-iridium-e2e-fix 2>&1 | tee build.log
```

預期：無 `SendGw2GwTaggedPacket` 相關 warning，無 unused variable warning。

### gw2gw_e2e 功能確認

```bash
./ns3 run "test-iridium-e2e-fix --pathType=gw2gw_e2e --simTimeSec=120" 2>&1 | tee Logs/run_gw2gw_onoff.log
```

預期 log 中出現：
```
[GW2GW_APP] OnOffHelper installed: GW0=40.1.0.1 -> GW1=40.67.0.1
[GW2GW_OBS][PACKET] PacketSink::Rx trace connected on GW1 (physical)
```

不應出現：
```
[GW2GW_APP] SendGw2GwTaggedPacket
```

---

## 待辦（下次 session）

1. **修正 `sat2gw` 流量方向**（高優先）：強制 `enableRtn=true, enableFwd=false` → ✅ 已於 Session 2 修正
2. **決策 gw2gw_e2e E2E delay 量測方案**（中優先）：接受缺口或實作替代方案 → ✅ 已於 Session 2 決策並修正
3. **確認 ARP prefill 修正在真實 SNS3 run 生效**（驗證）：首次跑 `gw2gw_e2e` OnOffHelper 版本，確認不發生 ARP crash（t=1.008s `NS_ASSERT(device)` 不出現）

---

## Session 2 補充修正


### 修正 A：sat2gw 流量方向

**問題**：`GatewayRxFeederCb` 僅在 feeder downlink（SAT→GW，RTN 方向）fire。
若 `enableFwd=true, enableRtn=false`，observer 恆回報 `rxPkts=0`，
`[FEEDER_LAYER]` verdict 永遠 FAIL。

**修正**：在 `BuildPathTypePlan` 中新增 `sat2gw` 專屬分支，
強制覆蓋 TrafficConfig，無論 caller 如何設定：

```cpp
else if (cfg.pathType == "sat2gw")
{
    // GatewayRxFeederCb fires ONLY on SAT→GW feeder downlink (RTN).
    // FWD traffic never produces packets at GW feeder RX side.
    cfg.feederlink.traffic.enableFwd = false;
    cfg.feederlink.traffic.enableRtn = true;
    plan.trafficKind = TrafficKind::GW_UT_ALL;
}
```

**影響**：`sat2gw` 現在確保 RTN 流量存在，`GatewayRxFeederCb` 可正確計數。

---

### 修正 B：gw2gw_e2e E2E delay 量測

**決策**：Session 1 改為 OnOffHelper 造成 `Gw2GwTxTimeTag` 無法附加（OnOffHelper 不支援 PacketTag 注入），
E2E delay 量測功能喪失（`delaySamples` 恆 0）。

**修正**：將 `InstallGw2GwApplicationTraffic` 改回 raw socket 模式，
新增 `SendGw2GwTaggedPacket` 函式，每個封包發送前注入 `Gw2GwTxTimeTag`：

```cpp
static void
SendGw2GwTaggedPacket(Ptr<Socket>  socket,
                      Ipv4Address  dstAddr,
                      uint16_t     port,
                      uint32_t     packetSize,
                      Time         interval,
                      Time         stopTime)
{
    if (!socket || Simulator::Now() > stopTime) return;
    Ptr<Packet> pkt = Create<Packet>(packetSize);
    Gw2GwTxTimeTag txTag;
    txTag.SetTxTime(Simulator::Now());
    pkt->AddPacketTag(txTag);
    socket->SendTo(pkt, 0, InetSocketAddress(dstAddr, port));
    Simulator::Schedule(interval, &SendGw2GwTaggedPacket,
                        socket, dstAddr, port, packetSize, interval, stopTime);
}
```

`Gw2GwAppRxCb` 端已有 `PeekPacketTag(txTag)` 邏輯，Rx side 不需改動。

**設計說明**：此模式與現有 `SendSat2UtTaggedPacket` 相同，為專案內標準 tagged-CBR 做法。
OnOffHelper 統一化目標退讓給 per-packet delay 量測需求，屬功能正確性優先決策。

---

### 修正 C：feeder_dn 傳播延遲量測

**問題**：`GatewayRxFeederCb` 計數 feeder downlink 封包，但從未量測 feeder_dn 的傳播延遲，
`g_feederObsStats[key].delaySamples` 永遠為 0。

**來源確認**：`SatNetDevice::m_rxLinkDelayTrace`（`satellite-net-device.h:249`），
trace name = `"RxLinkDelay"`，signature = `TracedCallback<const Time&, const Address&>`。

**修正**：新增 `GatewayFeederLinkDelayCb`，在 `ConnectLinkObserverTraces` 的
`useGwFeeder` block 中連接至 GW 端 `SatNetDevice`：

```cpp
static void
GatewayFeederLinkDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // feeder_dn (SAT→GW) link propagation delay.
    // Source: SatNetDevice::RxLinkDelay on GW (m_rxLinkDelayTrace, satellite-net-device.h:249).
    auto& s = g_feederObsStats[key];
    s.sumDelayMs += delay.ToDouble(Time::MS);
    s.delaySamples++;
}
```

連接位置（`useGwFeeder` block，緊接 `GatewayRxFeederCb` 之後）：
```cpp
satDev->TraceConnectWithoutContext(
    "RxLinkDelay",
    MakeBoundCallback(&GatewayFeederLinkDelayCb, gwKey));
```

---

### 修正 D：service FWD 傳播延遲量測

**問題**：`UtRxServiceCb` 計數 service FWD 封包，但從未量測 SAT→UT 的傳播延遲，
`g_serviceObsStats[key].delaySamples` 永遠為 0。

**來源確認**：同為 `SatNetDevice::m_rxLinkDelayTrace`（`satellite-net-device.h:249`），
連接對象為 UT 端的 `SatNetDevice`（service link FWD 接收端）。

**修正**：新增 `UtServiceLinkDelayCb`，在 `ConnectLinkObserverTraces` 的
`useUtService` block 中連接至 UT 端 `SatNetDevice`：

```cpp
static void
UtServiceLinkDelayCb(std::string key, const Time& delay, const Address& /*addr*/)
{
    // service FWD (SAT→UT) link propagation delay.
    // Source: SatNetDevice::RxLinkDelay on UT (m_rxLinkDelayTrace, satellite-net-device.h:249).
    auto& s = g_serviceObsStats[key];
    s.sumDelayMs += delay.ToDouble(Time::MS);
    s.delaySamples++;
}
```

連接位置（`useUtService` block，緊接 `UtRxServiceCb` 之後）：
```cpp
satDev->TraceConnectWithoutContext(
    "RxLinkDelay",
    MakeBoundCallback(&UtServiceLinkDelayCb, utKey));
```

**待確認**：GW 端的 `satDev` 已設定 `SetAttribute("EnableStatisticsTags", BooleanValue(true))`（~line 1411），
但 UT 端 `satDev` 在 `useUtService` block 中未設定此 attribute。
若 `RxLinkDelay` 需要 `EnableStatisticsTags=true` 才能 fire，需在 UT 端補加此行。
**請使用者於實機測試時確認 `UtServiceLinkDelayCb` 是否被呼叫。**

---

### 修正 E：gw2gw_e2e feeder 行為描述 comment 錯誤

**問題**：code 中約 line 5278 的 comment 寫：
> "gw2gw_e2e excludes feeder PHY traces from its main verdict"
> "gw2gw_e2e does not connect feeder PHY traces for its main verdict"

但實際 code 設定 `obsFeederMode=PHY`、`useOrbiterFeeder=true`，
且 `[FEEDER_LAYER]` verdict 確實由 `OrbiterRxFeederCb` 驅動。

**修正**：更新 comment 為：
```
//   gw2gw_e2e has obsFeederMode=PHY; its FEEDER_LAYER verdict uses OrbiterRxFeederCb
//   counts on the entry satellite. useOrbiterFeeder=true for gw2gw_e2e.
// ...
//   gw2gw_e2e feeder_dn (exitSat→dstGW) is scoped but not connected here;
//   only the entry-satellite OrbiterRxFeeder drives the gw2gw_e2e FEEDER verdict.
```

---

## Session 2 檔案變更摘要

| 檔案 | 修改內容 |
|------|----------|
| `Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc` | (A) `BuildPathTypePlan`：新增 `sat2gw` branch 強制 RTN；(B) `InstallGw2GwApplicationTraffic`：OnOffHelper → raw socket + `SendGw2GwTaggedPacket`；(C) 新增 `GatewayFeederLinkDelayCb` + 連接；(D) 新增 `UtServiceLinkDelayCb` + 連接；(E) 修正 line 5278 附近 comment 錯誤描述 |

---

## Session 2 驗證方式

```bash
# sat2gw 流量方向修正驗證
./ns3 run "test-iridium-e2e-fix --pathType=sat2gw --simTimeSec=120" 2>&1 | tee Logs/run_sat2gw_rtn.log
# 預期：[FEEDER_LAYER] rx_pkts > 0

# gw2gw E2E delay 驗證
./ns3 run "test-iridium-e2e-fix --pathType=gw2gw_e2e --simTimeSec=180" 2>&1 | tee Logs/run_gw2gw_tagged.log
# 預期：[GW2GW_OBS] delay_avg_ms > 0（delaySamples > 0）

# feeder_dn / service FWD delay 驗證
./ns3 run "test-iridium-e2e-fix --pathType=sat2gw --simTimeSec=120" 2>&1 | grep "feeder.*delay"
./ns3 run "test-iridium-e2e-fix --pathType=sat2ut --simTimeSec=120" 2>&1 | grep "service.*delay"
# 預期：delaySamples > 0（若 EnableStatisticsTags 正常）
```

---

## Session 3 更新（2026-04-30）

### 已完成的程式修正

**影響檔案**
- `Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc`

**修改內容**
- 將 `sat2ut` 對齊為真正的 `SAT_UT_APPLICATION`，不再借用 `GW_UT_SELECTED`
- 新增定向的 `SAT_SAT_APPLICATION`，讓 `sat2sat` 不再依賴 `GW <-> all UTs` 背景流量冒充指定 SAT→SAT packet path
- 新增 `Sat2SatAppRxCb()` 與 `InstallSat2SatApplicationTraffic()`，使 `sat2sat` 具備真正的 `PACKET_LAYER` verdict
- 重做 `ISL_PHY_LAYER` 驗證為 slot-aware：
  - 新增 `g_islPropagationStatsBySlot`
  - 在 `IslPropagationDelayCb()` 內依 slot 分桶 `PropagationDelayTrace` sample
  - 新增 `EvaluateScopedIslPhyDelay()`，用同 slot 的 observed hop propagation 對比同 slot 的 theoretical hop propagation
- 將 delay matrix 中觀測到的 ISL PHY 欄位改為使用 per-slot propagation stats，而不是跨 slot aggregate
- 移除 observer hookup 內誤導性的 per-GW `EnableStatisticsTags=true` 重設，保留全域 default 作為單一設定來源
- 修正 ARP prefill 狀態機處理：
  - 舊作法：`MarkAlive(dstMac)`
  - 新作法：`SetMacAddress(dstMac)` + `MarkPermanent()`
  - 原因：手動靜態 ARP prefill 不應走 `WAIT_REPLY -> ALIVE` 的 ARP 回覆狀態機

### gw2gw 驗證結果

**來源 log**
- `Topology & ISL Routing/Outputs/0430/gw2gw.log`

**已確認 PASS**
- ARP prefill 不再 crash
- `ISL_TRANSIT_LAYER` PASS
- `ISL_PHY_LAYER` PASS

**關鍵 verdict**
- `[ISL_TRANSIT_LAYER] PASS | scopedLinks=8 scopedRxPkts=389101`
- `[ISL_PHY_LAYER] PASS | channelDelaySamples=389101 channelAvgDelay=12.57ms observedHopPropDelay=12.572ms theoreticalHopPropDelay=12.951ms errorMs=0.379 toleranceMs=4.533 slotsWithSamples=3 slotsWithTheory=3`

**解讀**
- propagation-delay 的驗證邏輯現在已經能正確比較「同 slot 理論值」與「同 slot 觀測到的 PHY propagation」
- 這次 run 顯示 ISL PHY propagation 結果落在容忍範圍內
- **重要說明**：`ISL_PHY_LAYER` 的 389101 samples 來自 **background ISL traffic**（`isl_background` path type 的 P2P OnOff flow），
  **不是 gw2gw_e2e 自身的 packet**。ISL 通道層的 propagation delay 機制本身是正確的；
  但 gw2gw_e2e 的 packet 目前尚未成功到達 ISL 層（因 FEEDER_LAYER 仍 FAIL）。
- 因此，`gw2gw_e2e` 目前的主要 blocker 是 feeder egress，而非 ISL propagation 機制。

### gw2gw 仍然失敗的部分

**目前仍為 FAIL**
- `[FEEDER_LAYER] FAIL | scopedKeys=4 scopedRxPkts=0 avgDelay=--`
- `[PACKET_LAYER] FAIL | appInstalled=1 traceConnected=1 traceRxPkts=0 traceRxBytes=0 ...`

**診斷證據**
- `[OBS][FEEDER-DIAG] total OrbiterRxFeeder callbacks (all sats, unscoped): 0`
- packet summary 顯示 `received: 0 bytes (~0 pkts)`

**解讀**
- source GW 發出的 packet 沒有真正進到 entry-satellite 的 feeder receive path
- packet-layer 失敗發生在 ISL propagation 之前
- 後續 debug 重點應從 ISL PHY delay 移到 GW feeder egress / entry feeder bring-up

### 目前狀態總結

**已解決**
- ARP prefill 狀態機 crash
- 錯誤的 cross-slot vs slot-0 PHY propagation 比較
- `gw2gw_e2e` 的 propagation delay 驗證

**尚未解決**
- `gw2gw_e2e` source GW 的 feeder egress
- `gw2gw_e2e` packet 送達 destination GW

### 下一步追查目標

- 為什麼 `GW0` 的 direct host-route + prefilled ARP packet 仍然完全沒有觸發 `OrbiterRxFeederCb`

---

## Session 4 補充修正（2026-04-30）

### 修正 F：Compiler warning — `GetGwFeederMacAddress` 未使用

**問題**：函式 `GetGwFeederMacAddress` 已宣告且有定義，但 call site 已在前次 session 移除，
造成 `defined but not used` compiler warning。

**修正**：移除 forward declaration 及函式定義。

**影響檔案**：`Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc`

---

### 修正 G：ARP crash — `NS_ASSERT(device)` at `arp-l3-protocol.cc:384`（t=1.008s）

**症狀**：模擬在 t=1.008s crash，`NS_ASSERT failed, cond="device"` at `arp-l3-protocol.cc, line=384`

**根因**：`PrefillArpEntry` 在 `iface->GetArpCache()` 回傳 null 時建立新的 `ArpCache` 物件，
但未呼叫 `SetDevice()`，導致 `cache->GetDevice()` 回傳 null。
當 ARP stack 對任何未 prefill 的 IP 嘗試發出 ARP request 時，
`ArpL3Protocol::SendArpRequest` 內的 `NS_ASSERT(device)` 即 crash。

**修正**：在 `PrefillArpEntry` 建立 `ArpCache` 後立即呼叫 `SetDevice()`：

```cpp
if (!arpCache)
{
    arpCache = CreateObject<ArpCache>();
    // SetDevice is required: without it cache->GetDevice() returns null, which causes
    // NS_ASSERT(device) crash in ArpL3Protocol::SendArpRequest if ARP is ever triggered
    // on this interface for any IP that doesn't have a permanent entry.
    arpCache->SetDevice(srcFeeder.netDevice, iface);
    iface->SetArpCache(arpCache);
}
```

**影響檔案**：`Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc`（`PrefillArpEntry`，約 line 2455）

---

### 修正 H：ISL arbiter NS_FATAL — `satellite-isl-arbiter.cc:67`（主要修正）

**症狀**：模擬 crash，log 顯示：
```
ns-3.43/contrib/satellite/model/satellite-isl-arbiter.cc, line=67
NS_FATAL, terminating
```

**根因分析**

前次 session 以 `FindSatelliteFeederMac` 取得 satellite 的 `SatOrbiterNetDevice` MAC，
作為 L2 destination 送入 feeder uplink packet。

但 SNS3 REGENERATION_NETWORK 模式下，`SatIslArbiterUnicast::GetSatIdWithMacIsl(mac)` 
的 MAC 查找表內只有 **destination GW 的 `SatNetDevice` MAC**，不含任何衛星的 `SatOrbiterNetDevice` MAC。
Arbiter 查不到傳入的 MAC 就呼叫 `NS_FATAL_ERROR`。

**正確的 L2 dst 設計**：
- L2 dst = **目的 GW 的 `SatNetDevice` MAC**
- SatGwLlc 透過 `SatGroundStationAddressTag` 讀取 L2 dst
- ISL arbiter 用 `GetSatIdWithMacIsl(dstGwMac)` 映射到 exit satellite ID
- 路由不需要 gateway IP，ARP 直接解析 `dstAddr → dstGwMac`

**修正內容**

#### (1) 移除 `FindSatelliteFeederMac`，新增 `GetDstGwMac`

```cpp
// 掃描 dstNode（目的 GW 節點）的 SatNetDevice，取其 MAC48 地址。
// ISL arbiter 的 MAC 表只含 GW MAC，傳入 satellite MAC 會 NS_FATAL。
static bool
GetDstGwMac(Ptr<Node> dstNode, Mac48Address& outMac)
{
    for (uint32_t d = 0; d < dstNode->GetNDevices(); ++d)
    {
        Ptr<SatNetDevice> satDev = DynamicCast<SatNetDevice>(dstNode->GetDevice(d));
        if (!satDev) continue;
        Address addr = satDev->GetAddress();
        if (Mac48Address::IsMatchingType(addr))
        {
            outMac = Mac48Address::ConvertFrom(addr);
            return true;
        }
    }
    return false;
}
```

#### (2) 修正 `InstallGw2GwApplicationTraffic` — ARP + route block

```cpp
// 舊（錯誤）：satGwIp → satellite OrbiterNetDevice MAC
Mac48Address satFeederMac;
if (FindSatelliteFeederMac(route0.entrySatId, satFeederMac))
    PrefillArpEntry(srcNode, gwSrc, srcFeeder, satGwIp, satFeederMac);
staticRouting->AddHostRouteTo(dstAddr, satGwIp, srcFeeder.ifIndex);  // gateway=satGwIp

// 新（正確）：dstAddr → 目的 GW SatNetDevice MAC
Mac48Address dstGwMac;
NS_ABORT_MSG_IF(!GetDstGwMac(dstNode, dstGwMac), "[GW2GW_APP] GetDstGwMac failed for GW" << gwDst);
PrefillArpEntry(srcNode, gwSrc, srcFeeder, dstAddr, dstGwMac);
staticRouting->AddHostRouteTo(dstAddr, srcFeeder.ifIndex);  // no explicit gateway
```

#### (3) 修正 `UpdateGw2GwHostRoute` — 新增 `dstGwMac` 參數，同步修正

```cpp
// 舊 signature（缺 dstGwMac）：
static void UpdateGw2GwHostRoute(..., Ipv4Address dstAddr)

// 新 signature：
static void UpdateGw2GwHostRoute(..., Ipv4Address dstAddr, Mac48Address dstGwMac)
```

函式內 route 改為 no-gateway，ARP 改為 `dstAddr → dstGwMac`，不再使用 `newSatGwIp` / `FindSatelliteFeederMac`。

#### (4) 更新 slot schedule 呼叫

```cpp
Simulator::Schedule(Seconds(slotTime),
                    &UpdateGw2GwHostRoute,
                    srcNode, routingMgr, gwSrc, gwDst, slot,
                    dstAddr, dstGwMac);  // dstGwMac 新增傳入
```

**影響檔案**：`Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc`

---

### gw2gw_e2e 最終 Routing / ARP 設計總結

經歷多次修正後，gw2gw_e2e 的 L3/L2 routing model 定義如下：

| 層 | 設計 | 說明 |
|----|------|------|
| **L3 Route** | `AddHostRouteTo(dstAddr, feederIfIndex)` — 無 gateway | ARP 直接解析 dstAddr；feederIfIndex = entrySatId + 1 |
| **L2 ARP prefill** | `dstAddr → dstGwMac`（目的 GW 的 `SatNetDevice` MAC） | 讓 feeder uplink packet L2 dst = GW1 MAC |
| **ISL arbiter** | `GetSatIdWithMacIsl(dstGwMac)` → exit satellite | 必須是 GW MAC；satellite OrbiterNetDevice MAC 不在查找表 |
| **SatGwLlc** | 從 L2 dst 讀取 `SatGroundStationAddressTag` | tag 攜帶 dstGwMac，驅動 ISL arbiter 做路徑決定 |

> 任何試圖使用 satellite MAC（`SatOrbiterNetDevice`）或 gateway IP 作為 gateway 參數的方案
> 均會導致 ISL arbiter `NS_FATAL` 或 ARP crash，已完整排除。

---

### Session 4 後當前狀態

| 問題 | 狀態 |
|------|------|
| Compiler warning `GetGwFeederMacAddress` | ✅ 已修正 |
| ARP crash `NS_ASSERT(device)` @ t=1.008s | ✅ 已修正 |
| ISL arbiter `NS_FATAL` @ arbiter.cc:67 | ✅ 已修正 |
| `FEEDER_LAYER` FAIL（rxPkts=0） | ⏳ 待驗證 |
| `PACKET_LAYER` FAIL（received=0） | ⏳ 待驗證 |

---

### 驗證方式（Session 4 後）

```bash
cd $NS3_ROOT
./waf --run "test-iridium-e2e-fix --simTime=180 --scenario=gw2gw_e2e" 2>&1 | tee run_gw2gw.log
```

預期：
- 不再出現 `NS_FATAL` at `satellite-isl-arbiter.cc:67`
- `[GW2GW_APP] ARP prefill: GW0 if=<N> <dstAddr> -> <dstGwMac>` 出現，MAC 應為 GW1 的 SatNetDevice MAC
- `[GW2GW_APP] static host route: dst=<dstAddr> if=<N> (no gateway)` 出現
- 後續觀察 `FEEDER_LAYER` 與 `PACKET_LAYER` verdict
