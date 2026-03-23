# DEC-003：Arbiter 生命週期管理（預先建立 vs 排程內建立）

**日期**：2026-03-23
**狀態**：已確認

---

## 背景

`ApplyRoutingTable()` 在每個排程時間點需要更新 66 顆衛星的 `SatIslArbiterUnicast`。最直覺的做法是每次建立新的 arbiter 物件再呼叫 `SetArbiter()` 替換。

---

## 問題

**現象**：排程內呼叫 `CreateObject<SatIslArbiterUnicast>()` 時，模擬 crash，輸出 fatal error。

**原因**：`CreateObject<T>()` 要求 `T` 有可用的 default constructor。`SatIslArbiterUnicast` 的 default constructor 缺少必要的初始化參數（衛星節點指標），導致物件建立後處於無效狀態，在後續呼叫時觸發 fatal。

---

## 評估的方案

| 方案 | 問題 |
|------|------|
| 排程內 `CreateObject` 建立新 arbiter | Fatal crash，default constructor 無效 |
| 排程內用 `new` 直接建立 | 繞過 ns-3 物件系統，不符合 ns-3 記憶體管理規範 |
| 預先建立並快取，排程只更新內容 | 可行，物件在正確上下文建立 |

---

## 決策

採用「預先建立並快取」方案：

1. 新增 `InitOrbiterDevices()`，在 `Simulator::Run()` 之前建立並安裝 66 個 arbiter
2. 排程執行時只呼叫 `ClearNextHopEntries()` + `AddNextHopEntry()`，不重新建立物件
3. 新增 `ClearNextHopEntries()` 至 `SatIslArbiterUnicast`，允許原地清空後重填

**理由**：arbiter 物件在有效的衛星節點上下文中建立，生命週期與模擬一致，排程內只做資料更新，無物件建立開銷。

---

## 影響範圍

| 位置 | 修改 |
|------|------|
| `satellite-isl-arbiter-unicast.h` | 新增 `ClearNextHopEntries()` |
| routing helper | 新增 `InitOrbiterDevices()`，快取 arbiter 指標 |
| `ApplyRoutingTable()` | 改為 `ClearNextHopEntries()` + `AddNextHopEntry()` |
