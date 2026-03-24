# DEC-004：Beam Scheduler 開銷（模擬速度瓶頸）

**日期**：2026-03-24
**狀態**：已確認根本原因，優化方向待決定

---

## 診斷數據

simTime=200s, slotInterval=60s 的計時拆解：

```
Initialize         :     0 ms
PrecomputeAllTables:     2 ms
ScheduleRoutes     :     0 ms
Simulator::Run     : 1026865 ms
Total Wall         :  1026.87 s
Total scheduled events processed: 4294967295  ← UINT32_MAX，計數器溢位
```

wall / sim 比值：1026s / 200s = 5.13×。

---

## 根本原因

`Simulator::Run` 佔 99.9% 執行時間，且 event count 為 `UINT32_MAX`（`uint32_t` 溢位），代表實際排程事件數超過 42 億。

來源是 SNS3 的 DVB MAC beam scheduler：

- 66 顆衛星各有 forward + return link scheduler
- 每個 scheduler 的 superframe 週期約 250ms
- 即使沒有使用者流量，scheduler 仍持續產生 timing 事件
- 200s 模擬時間內理論事件量：66 × 4 × (200 / 0.25) = ~211,200，但實際更多（控制訊息、ACK、beam 管理等）

`PrecomputeAllTables` 僅 2ms，Fix 1–4 的效能優化對整體 wall time 影響可以忽略，問題完全在 `Simulator::Run`。

---

## 優化方向評估

| 方案 | 預期效果 | 代價 | 狀態 |
|------|---------|------|------|
| 降低 superframe 週期（250ms → 1s） | 減少約 4× 排程事件 | MAC 精度略降 | 待測試 |
| 關閉未使用 beam 的 scheduler | 最大效果（只開 beam 1） | 需修改 `SatBeamHelper` | 待評估 |
| 換用 `REGENERATION_PHY` 而非 `NETWORK` | 減少逐層處理開銷 | 改變模擬語意，影響 ISL 行為 | 風險高 |
| 限制模擬時間（先用 200s 驗證正確性） | 立即可行 | 無法做完整 600s 場景 | 目前採用 |

---

## 目前立場

優先確保功能正確性（`UpdateLoadCosts`、`HasSignificantChange`、`RecomputeAffectedRoutes` 實作完成），效能優化在功能驗證後再處理。短期以 simTime=200s 進行開發迭代。
