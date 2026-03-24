# Diagnostic simulation execution time is too long
**工作日誌 2026-03-24**

## 目標
診斷模擬執行時間過長（2760s wall time），並完成 `isl-graph` 效能優化（v3）。

---

## 完成事項

### 1. 效能瓶頸診斷：計時拆解確認根本原因

**現象**：模擬 200s 場景（simTime=200, slotInterval=60）的計時拆解結果：

```
Initialize         :     0 ms
PrecomputeAllTables:     2 ms
ScheduleRoutes     :     0 ms
Simulator::Run     : 1026865 ms   ← 1026 秒
Total Wall         :  1026.87 s
Total scheduled events processed: 4294967295
```

**原因確認**：`PrecomputeAllTables` 僅 2 ms，完全不是瓶頸。`Simulator::Run` 佔 99.9% 的執行時間，而 event count 為 `4294967295`（即 `UINT32_MAX`），代表計數器溢位——實際事件數遠超預期。

**根本原因**：SNS3 的 DVB MAC beam scheduler 持續產生排程事件。66 顆衛星各自有 forward + return link scheduler，superframe 週期約 250ms，200s 模擬時間內產生的排程事件量遠超 ISL 本身的事件量。這不是 `PrecomputeAllTables` 或 `BuildISLGraph` 的問題，而是 SNS3 beam scheduler 的固有開銷。

→ 詳細決策見 [`Decisions/DEC-004-beam-scheduler-overhead.md`]()

---

### 2. v3 效能優化：Fix 1–4

#### Fix 1：滾動圖快取（`PrecomputeAllTables`）

**現象**：`graphNext` 在 k 輪計算完後被丟棄，k+1 輪重新建立相同的圖。

**原因**：原始實作每輪獨立呼叫 `BuildISLGraph`，未保留上一輪的結果。

**修正**：將 `graphNext` 以 `std::move` 傳入下一輪作為 `graphCurr`，`BuildISLGraph` 呼叫次數從 19 次降為 10 次。

**驗證**：`PrecomputeAllTables` 輸出 slot 數與路由數量不變 ✅

---

#### Fix 2：SGP4 位置快取（`BuildISLGraph`）

**現象**：相同 `tau_k` 下，同一顆衛星的 `GetGeoPositionAt` 被呼叫多次。

**原因**：原始實作在 ISL 邊迴圈內對每條邊的兩端各呼叫一次，衛星出現次數決定呼叫次數。

**修正**：迴圈前預先快取所有衛星位置：
```cpp
std::vector<Vector> pos(m_numSatellites);
for (uint32_t i = 0; i < m_numSatellites; i++)
    pos[i] = m_orbNodes[i]->GetObject<SatSGP4MobilityModel>()
                           ->GetGeoPositionAt(tau_k).ToVector();
```
SGP4 呼叫次數從每次建圖 264 次降為 66 次，10 個 time slot 合計從 2640 降為 660（約 4× 改善）。

**驗證**：各時間點路由結果與 v2 一致 ✅

---

#### Fix 3：Dijkstra 直接記錄第一跳（`ComputeBaseRoutes`）

**現象**：Dijkstra 完成後，用 `prevHop` 逆向追蹤找第一跳，每個 dest 最多走 N-1 步，全部 `ComputeBaseRoutes` 約 O(N³)。

**原因**：原始實作只記錄直接前驅，未在鬆弛時傳播第一跳資訊。

**修正**：鬆弛時同步傳播 `firstHopNode` 與 `firstHopIf`：
```cpp
if (u == src) {
    firstHopNode[e.nodeB] = e.nodeB;
    firstHopIf[e.nodeB]   = e.islIfIndexOnA;
} else {
    firstHopNode[e.nodeB] = firstHopNode[u];
    firstHopIf[e.nodeB]   = firstHopIf[u];
}
```
路徑回溯從 O(N²) 降為 O(1) per dest，整體降為 O(N·E log V)。

**驗證**：SAT0 各 dest 的 nextHop 與 v2 結果一致 ✅

---

#### Fix 4：`ApplyTiebreaker` 改用 `unordered_set`

**現象**：`nextEligible.count({u, v})` 每次 O(log N)，在 tiebreaker 比對時反覆查詢。

**原因**：原始實作用 `std::set`，查詢複雜度 O(log N)。

**修正**：改用 `unordered_set` 配合自定義 hash，查詢降為 O(1) 均攤。

**驗證**：tiebreaker 結果與 v2 一致 ✅

---

### 3. `ApplyRoutingTable` 中的 `CreateObject` 問題（待處理）

**現象**：從程式碼中發現 `ApplyRoutingTable` 仍在排程內呼叫 `CreateObject<SatIslArbiterUnicast>()`，與 DEC-003 決策不符。

**原因**：v3 的效能優化未一併修正 arbiter 生命週期，`ClearNextHopEntries` 方案尚未套用。

**狀態**：⏳ 待修正，列入明日計畫。

---

## 修改的檔案

| 檔案 | 位置 | 異動 |
|------|------|------|
| `isl-graph.cc` | `contrib/satellite/helper/` | Fix 1–4 全部套用 |
| `isl-graph.h` | `contrib/satellite/helper/` | 新增 `GetPositionsAt` 宣告（如有拆出） |

---

## 明日計畫

1. **修正 `ApplyRoutingTable`**：套用 DEC-003 方案，改用預先建立的 arbiter + `ClearNextHopEntries`
2. **處理 beam scheduler 開銷**：評估 DEC-004 中的優化方向（降低 superframe 週期、關閉未使用 beam）
3. **實作動態路由新方法**：`UpdateLoadCosts`、`HasSignificantChange`、`RecomputeAffectedRoutes`
4. **整合進 `my-simulation.cc`**：加入實際流量，驗證端對端延遲與封包遺失率
