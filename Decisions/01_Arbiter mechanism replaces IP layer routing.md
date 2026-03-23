# DEC-001：SNS3 接口對齊（Arbiter 機制取代 IP 層路由）

**日期**：2026-03-19
**狀態**：已確認

---

## 背景

原始設計假設 SNS3 的 ISL routing 走標準 ns-3 IP 層，使用 `Ipv4StaticRouting` 寫入路由、使用 `FqCoDelQueueDisc` 讀取負載。閱讀 SNS3 原始碼後發現三個假設均不正確。

---

## 決策一：路由表寫入方式

**原設計**：`Ipv4StaticRouting::AddHostRouteTo(destAddr, nhAddr, ifIdx)`

**實際接口**：`SatOrbiterNetDevice::SetArbiter(newArbiter)`

**原因**：SNS3 的 ISL 封包轉發繞過 IP 層，由 `SatOrbiterNetDevice` 上掛載的 `SatIslArbiterUnicast` 查表決定出口。Arbiter 內部為 `map<destSatId, islInterfaceIndex>`，轉發時呼叫 `Decide(sourceSatId, targetSatId, pkt)`。

**採用方案**：整個替換 Arbiter（`SetArbiter(newArbiter)`），不做 diff-based 更新。

**理由**：整體替換比逐項修改更簡單，且不會有殘留舊路由的問題。未來如需增量更新，修改點只有 `ApplyRoutingTable()` 內部邏輯。

---

## 決策二：Queue 讀取方式

**原設計**：`FqCoDelQueueDisc`（TC 層）

**實際接口**：`PointToPointIslNetDevice::GetQueue()->GetNPackets()`（device 層 `DropTailQueue<Packet>`）

**原因**：SNS3 的 `PointToPointIslNetDevice` 直接使用 device-level queue，未安裝 TC 層 QueueDisc。

**採用方案**：直接讀 `islDev->GetQueue()->GetNPackets()`。

---

## 決策三：islInterfaceIndex 的定義

**原設計**：Ipv4 interface index

**實際定義**：`SatOrbiterNetDevice::GetIslsNetDevices()` 回傳的 vector index

**原因**：Arbiter 的查表結果是 ISL device vector 的 index，與 Ipv4 interface 編號無關。

**採用方案**：建立 `peerNodeId → vector index` 的對應表，在 `BuildISLGraph` 時填入 `ISLState.ifIdxOnA` 和 `ifIdxOnB`。

---

## 影響範圍

| 模組 | 影響 |
|------|------|
| `ApplyRoutingTable()` | 改用 `SetArbiter()` |
| `UpdateLoadCosts()` | 改讀 device queue |
| `ISLState` 結構 | `islInterfaceIndex` 改為 vector index |
