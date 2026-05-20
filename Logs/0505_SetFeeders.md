# 工作日誌 2026-05-05

## 目標
驗證昨日修正 I（SetGwFeederSats 實作）與修正 J（feeder 候選衛星 IP scan → rtnConf）是否使 gw2gw_e2e 三層 verdict 全數轉為 PASS。

---

## 完成事項

### 1. gw2gw_e2e 首次全層 PASS 驗證

**現象**：昨日（05-04）修正 I + J 後，尚未完成實機驗證。修正 I 新增了 `SetGwFeederSats()` 實作並在 `PrecomputeGwRoutes()` 中加入 `feederFilter` lambda；修正 J 將 feeder 候選衛星建構從 IP 掃描改為直接讀取 `rtnConf.txt`，解決 `gwIdx=0 feederSats=255` / `gwIdx=1..4 feederSats=0` 的誤判問題。

**原因**：IP 掃描方式在 `includeAllFeederBeams=true` 時，所有 GW 的 feeder 介面集中在 GW0 物理節點，`o2` 值 1–255 涵蓋全部 66 衛星以上，造成 GW0 誤收全部衛星、其餘 GW 無任何候選。改用 `rtnConf.txt` 直接映射（`satId = beam - 1`，`gwIdx = gwIdFromFile - 1`）後，每個 GW 的 feeder 衛星集合正確分離。

**修正**：昨日已完成（見 `Logs/0504_gw2gw-elev-debug.md`）。今日執行實機驗證指令：
```
./ns3 run "test-iridium --pathType=gw2gw_e2e --gwSrc=0 --gwDst=1 --simTime=120 --numSlots=3 --slotInterval=60" 2>&1 | tee Outputs/run_fix.log
```

**驗證**：來源 `Topology & ISL Routing/Outputs/run_fix.log`（simTime=120s，3 slots）。

三層 verdict 確認結果：

```
[ROUTING_LAYER] PASS | validSlots=3/3 gwSrc=0 gwDst=1 obsFeederMode=ROUTING
[ISL_LAYER]     PASS | scopedLinks=16 scopedRxPkts=511250
[PACKET_LAYER]  PASS | appInstalled=1 traceConnected=1 traceRxPkts=1181
                        traceRxBytes=604672 delaySamples=1181 avgOneWayDelayMs=0.010
```

封包路徑（ROUTING 層）逐 slot 路由切換正常：

| slot | time(s) | ISL path | isl_cost(s) |
|------|---------|----------|-------------|
| 0 | 0 | 45->46->35->34->33 | 0.047382 |
| 1 | 60 | 15->14->3->4 | 0.037122（ROUTE CHANGED）|
| 2 | 120 | 45->34->33 | 0.029282（ROUTE CHANGED）|

ISL drop rate：0 drops / 8266570 pkts（drop_rate=0.000%），整體 ISL PASS。

E2E delay matrix：`appAvgDelayMs=0.010`，feeder/service delay 另由昨日 OBS 框架量測（feeder `OrbiterRxFeeder` callbacks 本次 run 仍為 0，obsFeederMode=ROUTING，FEEDER 層由 routing 驗證替代）。

---



## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\isl-graph.cc` | 新增 `SetGwFeederSats()` 實作；在 `PrecomputeGwRoutes()` 中加入 `feederFilter` lambda，將候選衛星限縮為仰角可見且有 feeder channel 的交集（修正 I） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\isl-graph.h` | 新增 `SetGwFeederSats()` 方法宣告；新增 `m_gwFeederSats` 成員（`std::map<uint32_t, std::set<uint32_t>>`） |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\test-iridium-e2e-fix.cc` | 將 feeder 候選衛星建構改為讀取 `rtnConf.txt`（修正 J）；新增 `E2EConfig::rtnConfFilePath` 欄位；移除原有 IP scan 邏輯 |


---

## 明日計畫

- 調查 `OrbiterRxFeeder` callbacks 仍為 0 的根因（obsFeederMode=ROUTING 時以 routing 驗證替代，但真實 feeder 收包尚未確認）
- 若要轉換 `obsFeederMode` 為 PHY 驗證，需確認 feeder uplink trace 是否需要 `includeAllFeederBeams=true` 以外的條件
- 調查 `sat2sat` NS_ABORT 問題（低優先）
- 確認 `UtServiceLinkDelayCb` 在實機的 `delaySamples` 是否 > 0（需 UT 端 `EnableStatisticsTags=true`）