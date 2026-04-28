# 工作日誌 2026-04-17

## 目標
診斷並修復 OBS（Link Observability）框架的 feeder preset 觀測問題，輸出存放於 `Outputs/preset_fix/`

---

## 完成事項

### 1. 診斷 gw2gw_e2e feeder 欄位全 0 問題（非 bug，屬正常行為）

**現象**：`gw2gw.log` 與 `feeder_diag_gw2gw.log` 中，OBS summary 的 `feeder:gw0`、`feeder:sat15` 等欄位顯示 `rx_pkts=0, drop_rate=0.00%`，乍看像未收到封包，實為誤導。

**原因**：`GW2GW_DIRECT` 流量在 REGENERATION_NETWORK 模式下直接走 IP routing，完全 bypass feeder PHY layer，`OrbiterRxFeeder` trace 從未觸發（`g_totalOrbiterFeederRxCalls=0`）。舊 OBS 不區分「真正 0 封包」與「trace 不適用」，兩者皆顯示 0。

**修正**：
- 新增 `g_totalOrbiterFeederRxCalls` 全域計數器，統計所有 OrbiterRxFeeder callback 觸發次數（不受 scope 限制）
- 當計數器為 0 時，feeder 欄位改顯示 `N/A (bypasses feeder PHY)` 而非誤導性的 0
- 新增 `[OBS][FEEDER-DIAG]` log 行，明確說明原因

**驗證**：`feeder_na_gw2gw.log` 輸出：
```
feeder:gw0    N/A (bypasses feeder PHY)
feeder:sat15  N/A (bypasses feeder PHY)
feeder:sat33  N/A (bypasses feeder PHY)
feeder:sat4   N/A (bypasses feeder PHY)
feeder:sat45  N/A (bypasses feeder PHY)
[OBS][FEEDER-DIAG] NOTE: GW2GW_DIRECT bypasses feeder PHY in REGENERATION_NETWORK mode.
```
ISL 封包交付正常，TOTAL: 6340555 pkts, 0 dropped [PASS]

---

### 2. 診斷 gw2sat feeder out-of-scope 問題

**現象**：`gw2sat.log` 的 OBS scope 設定為 `feeder=3`（sat33, sat4, sat5），但 `feeder_diag_gw2sat.log` 中新增的 FEEDER-DIAG 顯示大量 out-of-scope satellites 也在接收 feeder 封包：

```
[OBS][FEEDER-DIAG] total OrbiterRxFeeder callbacks (all sats, unscoped): 127339
[OBS][FEEDER-DIAG] out-of-scope feeder hits:
  sat1  rxPkts=13082   sat15 rxPkts=32087   sat17 rxPkts=3575
  sat20 rxPkts=2387    sat21 rxPkts=4763    sat22 rxPkts=4763
  sat23 rxPkts=9515    sat24 rxPkts=3575    sat37 rxPkts=4763
```

**原因**：`ObsScope::feederKeys` 只涵蓋 GW 的 primary serving satellite，但 SNS3 feeder 封包實際落在多顆 satellite（衛星切換 + 多 beam 覆蓋），造成大量封包落在 scope 外而未被追蹤。SNS3 的 `OrbiterRxFeeder` trace 本質上是全域的（所有 GW 的 feeder 交易共用同一個 callback），out-of-scope hits 包含其他 GW 的流量，無法單純靠 scope 管理解決。

**修正**：
- 新增 `ObsScope` struct（含 `feederKeys`、`serviceKeys`、`islKeys` 以及 `activeFeeder/activeService/activeIsl` flag）
- 新增 `IsObsKeyInScope()` 函式，統一判斷某 link key 是否應納入 OBS 統計
- 新增 `g_obsDebug` flag，啟用時輸出 out-of-scope hits 明細（即 `feeder_diag_*` 的行為）
- 實作動態 scope（Option C）：新增 `UpdateObsScopeForSlot()`，在每個 slot 切換後（由 NS-3 FIFO scheduler 保證在 `ApplyRoutingTable` 之後執行）重算各 pathType 的 feeder/service/ISL keys
- 新增 `g_feederNaExpected` flag：僅對 gw2gw_e2e 設為 true，修正 sat2gw 錯誤顯示 N/A 的 bug

**驗證**：
- `gw2sat_dynamic_scope.log`：`[OBS][SCOPE_UPDATE]` 在 slot=1/2 正確更新 feeder scope，out-of-scope hits 仍顯示於 FEEDER-DIAG（trace 架構限制，非 bug）
- `sat2gw_na_fix.log`：`feeder:gw0 rx_pkts=5148`，N/A 誤顯示已消除；`gw2gw_na_check.log`：`feeder:* N/A (bypasses feeder PHY)` 行為保留正確

---

### 3. 診斷 sat2gw OBS 假警報根因

**現象**：`sat2gw.log` 在 t=70s 觸發 `[OBS][EVENT] POSSIBLE LINK FAILURE`；`sat2gw_debug.log`（snapshot interval=1s）顯示警報在 t=61s 觸發，即 slot=1 切換（t=60s）後的第一個 snapshot 視窗。

**原因**：
- OBS snapshot interval=10s 時，t=60s slot 切換後第一個視窗（60s～70s）：衛星切換導致舊 key `feeder:gw0` 在切換瞬間 tput 歸零
- OBS 的 throughput 警報條件為 `rxPkts>0 && tput<1e-9`，只要歷史上有流量且本視窗歸零即觸發，無法區分「衛星切換正常中斷」與「真實鏈路失效」
- 以 snapshot=1s debug 跑可明確確認警報在 t=61s（切換後第 1 秒）觸發

**修正方向**（根因分析完成，patch 刻意 deferred）：
- 已實作 `g_prevObsThroughputKbps` map，改為邊緣偵測（`prevTput > threshold && curTput ≈ 0`）
- 邊緣偵測仍無法消除 slot boundary 觸發（prevTput>0 + curTput=0 條件仍成立）；完整修法需加入 slot boundary grace period
- 決策：grace period 實作延後，問題二暫不解決

**驗證**：`sat2gw_debug.log` 確認警報在 t=61s 觸發，`sat2gw.log`（10s interval）在 t=70s 觸發，與分析一致。`sat2gw_no_alarm.log` 為動態 scope 驗證 run（假警報仍存在，符合預期）。

---

### 4. isl-graph.cc 標籤修正

**修改**：`PrintGwUtRouteReport()` 中，entry satellite == serving satellite（無 ISL 跳）時，`isl_cost` 欄位從 `"N/A"` 改為 `"no ISL hop"`，語意更明確。

---

### 5. test-iridium-e2e.cc 架構清理與新功能

**修改**：
- 新增全域變數：`g_gwDeviceRxHits`, `g_gwDeviceTypes`, `g_obsDebug`, `g_totalOrbiterFeederRxCalls`, `g_prevObsThroughputKbps`, `g_feederNaExpected`
- 新增 helper：`MakeSatKey()`, `MakeGwKey()`, `MakeGwTxKey()`, `MakeIslKey()`, `IsObsKeyInScope()`
- 新增 `UpdateObsScopeForSlot()`：動態 scope 更新，支援 gw2sat / sat2ut / sat2sat / gw2ut_e2e / gw2gw_e2e 五種 pathType
- 主函式新增 slot 1..N-1 的 scope update scheduling loop（FIFO 保證在 `ApplyRoutingTable` 之後執行）

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `Topology & ISL Routing/Codes/test-iridium-e2e.cc` | 新增 ObsScope struct、FEEDER-DIAG 診斷機制、g_totalOrbiterFeederRxCalls、IsObsKeyInScope()、英文化 comments |
| `Topology & ISL Routing/Codes/isl-graph.cc` | `PrintGwUtRouteReport()` 無 ISL 跳時標籤 `"N/A"` → `"no ISL hop"` |
| `Topology & ISL Routing/Outputs/preset_fix/gw2gw.log` | 新增：gw2gw_e2e 初版 run（feeder 全 0 問題原始記錄） |
| `Topology & ISL Routing/Outputs/preset_fix/feeder_diag_gw2gw.log` | 新增：gw2gw_e2e 診斷 run（FEEDER-DIAG 輸出）|
| `Topology & ISL Routing/Outputs/preset_fix/feeder_na_gw2gw.log` | 新增：gw2gw_e2e 修正後 run（feeder 欄位顯示 N/A） |
| `Topology & ISL Routing/Outputs/preset_fix/gw2sat.log` | 新增：gw2sat 一般 run |
| `Topology & ISL Routing/Outputs/preset_fix/feeder_diag_gw2sat.log` | 新增：gw2sat 診斷 run（out-of-scope hits 明細） |
| `Topology & ISL Routing/Outputs/preset_fix/feeder_na_gw2sat.log` | 新增：gw2sat 修正後 run |
| `Topology & ISL Routing/Outputs/preset_fix/sat2gw.log` | 新增：sat2gw run（假警報原始記錄，snapshot=10s）|
| `Topology & ISL Routing/Outputs/preset_fix/sat2gw_debug.log` | 新增：sat2gw debug run（snapshot=1s，確認警報在 t=61s）|
| `Topology & ISL Routing/Outputs/preset_fix/gw2gw_tokyo_newdelhi.log` | 新增：gw2gw_e2e Tokyo→NewDelhi 場景驗證（ISL routing 正常） |
| `Topology & ISL Routing/Outputs/preset_fix/gw2sat_dynamic_scope.log` | 新增：gw2sat 動態 scope 驗證 run（SCOPE_UPDATE 正確更新） |
| `Topology & ISL Routing/Outputs/preset_fix/sat2gw_no_alarm.log` | 新增：sat2gw 動態 scope run（假警報仍在，問題二 deferred） |
| `Topology & ISL Routing/Outputs/preset_fix/sat2gw_na_fix.log` | 新增：sat2gw N/A bug 修正驗證（feeder:gw0 顯示實際 rx_pkts） |
| `Topology & ISL Routing/Outputs/preset_fix/gw2gw_na_check.log` | 新增：gw2gw_e2e N/A 行為保留驗證（feeder:* 仍顯示 N/A） |

---

## 未解決問題

| 問題 | 狀態 |
|------|------|
| sat2gw OBS 假警報 patch（slot boundary grace period） | 刻意 deferred，暫不修 |
| gw2sat out-of-scope feeder hits（其他 GW 流量滲入） | trace 架構限制，FEEDER-DIAG 可觀測，不修 |

---

## 今日完成小結

- OBS feeder N/A 誤顯示 bug：**已修正並驗證**（`g_feederNaExpected` flag）
- 動態 feeder scope（Option C）：**已實作並驗證**（`UpdateObsScopeForSlot()`）
- gw2gw_e2e / sat2gw 兩組 preset 驗證通過
