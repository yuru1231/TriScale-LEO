# 2026-05-11 更動日誌

## 今日目標

- 整理 GW feeder 過濾、`gw2gw_e2e` 預測式 beam 啟用、E2E 診斷輸出與文件入口調整。

---

## 今日更動

### 1. 變更摘要

- 在 `IslRoutingManager` 補強 GW 路由前置設定，新增 `SetGwFeederSats()`、`GetRequiredGwFeederSats()`，讓 `PrecomputeGwRoutes()` 可把候選 entry/exit satellite 限縮到 `rtnConf` 指派的 feeder sat 集合，並與仰角可見集合取交集。
- `PrecomputeGwRoutes()` / GW→UT 路由流程改為優先使用 feeder-filtered 候選集合，降低選到實際未對應 GW feeder channel 的衛星機率。
- `test-iridium-e2e-fix.cc` 新增 `PredictGw2GwRequiredBeams()` 兩階段流程：先用輕量 phase 1 預算各 slot 實際需要的 entry/exit sats，再只啟用必要 beams 建立 phase 2 正式場景，避免 `gw2gw_e2e` 一次開太多 feeder beams 造成大量 MAC/DAMA/RBDC 事件。
- `test-iridium-e2e-fix.cc` 補入更多 CLI/診斷能力，包括 `packetHopTrace`、`packetHopTraceMaxPackets`、`scenarioName`、`numSats`、`strict3gppScenarioD`，並新增 packet path trace、propagation delay audit、reroute summary 輸出。
- 調整 GW bootstrap / logical GW node 映射流程，加入 `RefreshLogicalGwNodeMap()` 與 target-GW 導向的 beam 啟用邏輯，降低 Phase 2 建錯實體 GW node 導致目的 IP 對錯節點的風險。
- `Readme.md` 由舊版技術長文改為專案導覽入口，將細節導向 `TechRef.md`、`Report/chapter3_sns3.md`、`Layer1.md` 與 `Logs/`。

### 2. 影響檔案

| 檔案 | 變更內容 |
|------|----------|
| `Topology & ISL Routing/Codes/isl-graph.h` | 宣告 `SetGwFeederSats()`、`GetRequiredGwFeederSats()`，補充 GW routing API 註解與 feeder sat 狀態欄位。 |
| `Topology & ISL Routing/Codes/isl-graph.cc` | 實作 feeder sat 過濾、required sat 蒐集、GW/UT entry sat filter，並調整 `PrecomputeGwRoutes()` 候選策略。 |
| `Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc` | 新增 phase 1 beam 預測流程、更多 CLI 選項、packet hop trace、route/propagation audit、GW node map refresh 等邏輯。 |

### 3. 驗證結果

執行情境：`gw2gw_e2e`，GW0=JP-Tokyo → GW1=IN-NewDelhi，Iridium-66，6 slots × 60s，simTime=300s

#### 3.1 Phase 1 Beam 預測

| 項目 | 結果 |
|------|------|
| Phase 1 預算所需衛星 | sat3, sat4, sat5（共 3 顆） |
| 啟用 beams（Phase 2） | {4, 5, 6}（原 includeAllFeederBeams ≈ 26） |
| 縮減效果 | beam 數從 ~26 → 3，physicalGwNodes = 2 |

來源：`[PREDICT] requiredBeams={4,5,6} (was ~26 with includeAllFeederBeams; physicalGwNodes in Phase 2 = 2)`

---

#### 3.2 Routing Layer — PASS

| slot | t(s) | entry | ISL path | exit | isl_cost(s) |
|------|------|-------|----------|------|-------------|
| 0 | 0 | sat3 | 3→4→5 | sat5 | 0.02635 |
| 1 | 60 | sat3 | 3→4 | sat4 | 0.01317 ← ROUTE CHANGED |
| 2–5 | 120–300 | sat3 | 3→4 | sat4 | 0.01317 |

- slot 0：2-hop（sat3→sat4→sat5），因 GW1 在 slot 0 可見兩顆衛星，exit 選到 sat5
- slot 1 起：GW1 可見集合調整，縮短為 1-hop（sat3→sat4），isl_cost 減半

---

#### 3.3 ISL Transit Layer — PASS

| 來源 | 數值 |
|------|------|
| ISL scoped rx_pkts | 664,449 |
| ISL drop rate | 0 drops / 28,561,089 pkts → 0.000% |
| ISL phy channel avg delay | 13.43 ms（理論 13.352 ms，誤差 0.076 ms） |

ISL channel 正常轉發，delay 與理論值吻合，無封包掉落。

---

#### 3.4 Feeder Layer — FAIL

- `[FEEDER_LAYER] FAIL | scopedKeys=5 scopedRxPkts=0`
- `[OBS][FEEDER-DIAG] total OrbiterRxFeeder callbacks (all sats, unscoped): 0`
- feeder 收不到任何 packet，但 ISL 有大量流量（664K pkts），代表 ISL 有在跑，但封包沒有真正從 GW0 進入 feeder uplink

---

#### 3.5 Packet Layer — FAIL

- `received: 0 bytes (~0 pkts)`
- GW1 endpoint probe：`phy_rx_observed_but_mac_rx_missing`
  - PHY 層有收到（rxPkts=4025），但 MAC 層 rxPkts=0
  - 推斷：封包有到達 GW1 的 PHY，但無法被 MAC 層接收（可能是 ARP 或 feeder beam MAC 配對問題）
- `[GW2GW_ROUTE] slot=1 dstAddr 從 40.6.0.1 改為 40.5.0.1`（reroute 觸發後 GW IP 切換）

---

#### 3.6 待查問題（已更新，見 §4）

| 問題 | 狀態 |
|------|------|
| GW1 feeder 收不到封包（OrbiterRxFeeder=0） | 已查明原因，見 §4.1 |
| MAC rx=0 但 PHY rx=4025 | 非主因，已排除 |
| slot 1 dstAddr 從 40.6.0.1 → 40.5.0.1 | 正常 reroute 行為，非 bug |

---

## 4. Fix 嘗試記錄

### 4.1 修正嘗試一：beam override {1,2,72}（gw2gw_fix2.log）

**動機**：原始 run 中 gwSrc=0 與 gwDst=1 均指向 nodeId=66（同一個 ns-3 實體節點），懷疑封包被 local delivery 截斷，未真正進入 feeder uplink。嘗試改用 beam {1,2,72} 以產生兩個不同的實體 GW node。

執行情境：`gw2gw_e2e`，gwSrc=0, gwDst=1，simTime=120，slotInterval=60，beamOverride={1,2,72}

#### 4.1.1 GW_MAP 結果

```
[GW_MAP] logicalGwId=1 -> nodeId=66  matchedFeederIfs=15   (Tier 1)
[GW_MAP] logicalGwId=4 -> nodeId=67  matchedFeederIfs=6    (Tier 1, beam72)
[GW_MAP] fallback ordered map logicalGwId=0 -> nodeId=66   (Tier 3)
physicalGwNodes=2
```

beam72 屬於 rtnConf 中的 gwIdx=4，不是 gwDst=1（IN-NewDelhi）。gwSrc=0 與 gwDst=1 **仍雙雙指向 nodeId=66**，物理節點衝突未解。

#### 4.1.2 路由選到的 entry sat 不在 enabledBeamSet

```
[GW2GW_APP] slot0 entrySat=3 GW0 feeder ifIndex=4 gwIp=40.4.0.1
```

`PrecomputeGwRoutes()` 根據仰角可見性選出 entry=sat3（beam4），但 beam4 **不在 {1,2,72}**。SNS3 未啟用 beam4 的 feeder DAMA scheduler，sat3 feeder 接收端不存在。

#### 4.1.3 觀測結果

| 層次 | 結果 |
|------|------|
| ROUTING_LAYER | PASS（路由計算正常，3/3 slots） |
| FEEDER_LAYER | **FAIL**（scopedRxPkts=0，OrbiterRxFeeder=0） |
| ISL_TRANSIT_LAYER | PASS（scopedRxPkts=141,592，但為背景流量） |
| PACKET_LAYER | **FAIL**（received=0 bytes） |

```
[OBS][FEEDER-DIAG] total OrbiterRxFeeder callbacks (all sats, unscoped): 0
```

#### 4.1.4 PKT_HOP 異常（spurious ISL trace）

```
[PKT_HOP] t=1.000s stage=app_tx → dst=40.6.0.1:9001
[PKT_HOP] t=1.021s stage=isl_hop link=3-2 delayMs=13.173
[PKT_HOP] t=1.034s stage=isl_hop link=2-1 delayMs=13.174
[PKT_HOP] t=1.048s stage=isl_hop link=1-0 delayMs=13.173
```

- app_tx → 21ms 後出現 isl_hop，但 feeder uplink 單程延遲約 12–13ms，時序不符
- ISL 方向為 sat3→sat2→sat1→sat0，與路由計畫（3→4→5）反向
- OrbiterRxFeeder=0 的前提下不應有 feeder 觸發的 ISL 流量
- 研判：PKT_HOP ISL 紀錄捕捉的是**背景流量**，並非 gw2gw 標記封包

#### 4.1.5 根本原因確認

| 問題 | 原因 |
|------|------|
| OrbiterRxFeeder=0 | beam4 不在 enabledBeamSet，feeder DAMA scheduler 未啟動 |
| GW node 仍衝突 | beam72=gwIdx=4≠gwDst=1，gwIdx=0 與 gwIdx=1 在 Iridium-66 rtnConf 中皆對應 nodeId=66 |
| PACKET received=0 | 上游 feeder 未通，封包無法進入衛星鏈路 |

#### 4.1.6 後續方向

- gwIdx=0（Tokyo）與 gwIdx=1（New Delhi）在 Iridium-66 SNS3 設定中共用同一實體 ns-3 node（nodeId=66），透過切換 beam set 無法分離
- 已確認 code 中存在 `static host route: dst=GW1_IP → if=entrySat_feeder_if` 及 ARP prefill，可強制封包走 feeder uplink 而非 local delivery
- 下一步：**改回 enabledBeamSet={4,5,6}**，使路由選用的 sat3/4/5 具備有效的 feeder channel，驗證 OrbiterRxFeeder 是否 > 0

---


