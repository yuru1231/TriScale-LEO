# Beam Hopping dynamic timeplan— 工作日誌 / 筆記

## 1. Phase G 架構概述



 |模組 | 特性 |
 |- | - |
 |`SatGreedyBstpProvider` | demand-greedy top-K，輕量動態 |

**啟動條件：** `enableDynamicBstp=true` 且 `enableScheduler=false`

**資料流：**

```
SatGreedyBstpProvider::GetNextConf(t)
    ↓ (每 T_p)
RunDynamicBstpCycle()
    ↓ ConfToTimePlan(conf, now + T_prop)
SatBhObc::ReceiveNewPlan(plan, T_prop)
    ↓ EnterSlot() / OnSlotServiceEnd()
SatNetDevice::ToggleState(true/false)   ← GW 物理開關
SatBhMetrics::OnSlotActivated/Deactivated
```

**demand feeding：**

```
SatBeamScheduler::BacklogRequestsTrace
    → OnBacklogRequestTrace()
    → m_dynamicProvider->UpdateBeamDemand(beamId, rbdcKbps)
```

---

## 2. 工作日誌

### Round 1 — 功能缺口

1. **Dynamic BHTP 沒有控制原本 SNS3 SatBstpController**  
   OBC callbacks 從未呼叫 `ToggleState(true/false)`，beam 物理狀態不變。

2. **beam metadata 使用 placeholder (0, beamId, 0)**  
   `AddEnabledBeamInfo` 的 freq/GW ID 均為 0，驗證器無法真正檢查 feederFreq 衝突。


3. **Propagation delay 在動態路徑被忽略**  
   `ConfToTimePlan(conf, now)` 直接用裸 `now` 作為 periodStart，OBC 立即排程 `EnterSlot(0)`，沒有加上 T_prop。

4. **`GetCurrentPlan()` 回傳過時的靜態計畫**  
   新 plan 送給 OBC，但 `m_staticPlan` 不更新，`GetCurrentPlan()` 永遠回傳初始靜態計畫。

5. **`SatDynamicBstpProvider::GetTypeId()` 放在 greedy `.cc`**  
   純虛擬基底類別的 TypeId 被放在具體實作的編譯單元，結構不清。

6. **過時的 doc comment**  
   `BuildBeamToggleMap()` 內部仍寫著 `SatNetDevice::GetBeamId()` 但程式已改用 `SatMac::GetBeamId()`。

**修正（Round 1 完成）：**
- 在 `SetupObc()` callbacks 裡加上 `ToggleState` 呼叫
- 在 `RunDynamicBstpCycle()` 加入 `periodStart = now + T_prop`
- 新增 `m_lastDynamicPlan`，修正 `GetCurrentPlan()`
- 更新 doc comment

---

### Round 2 — BuildBeamToggleMap 解耦

**問題：**

`BuildBeamToggleMap()` 只在 `ConnectTracesPhaseE()` 內部被呼叫，而 ResourceManager 需要 `enableResourceManager=true` AND `enablePhaseE=true` 才會啟動。

這導致 `enableObc=true` 但 `enableResourceManager=false` 時，toggle map 永遠是空的，OBC callbacks 找不到任何 `SatNetDevice`，`ToggleState` 從未執行。

**修正：**  
將 `BuildBeamToggleMap()` 從 `ConnectTracesPhaseE()` 內移到 `Install()` 的 `if (m_cfg.enableObc)` 區塊裡，直接在 `SetupObc()` 之後呼叫：

```cpp
if (m_cfg.enableObc)
{
    SetupObc();
    BuildBeamToggleMap();  // 與 ResourceManager 完全無關
}
```

OBC callbacks 以 lambda 捕捉 `this`，在事件發生時才讀 map，所以 map 只需在模擬開始前填好即可。

---
### Round 3 — 最終修正


**[1] `StringValue("BH_OFF")` 錯誤**

ns-3.43 的 `SatBstpController` enum 定義：
```cpp
BH_UNKNOWN = 0
BH_STATIC  = 1
BH_DYNAMIC = 2
// ← 沒有 BH_OFF
```

`MakeEnumChecker()` 只接受 `"Static"` 和 `"Dynamic"` 兩個字串。`StringValue("BH_OFF")` 在 runtime 會 abort。

Round 3 已修正為 `BooleanValue(false)` 

**[2] Toggle map key 多星碰撞**

原始 key = `beamId`（uint32_t），不同衛星可能有相同 beamId，會互相覆蓋。

修正：key 改為 `std::pair<uint32_t, uint32_t>` (`{satId, beamId}`)：
```cpp
// sat-bh-helper.h
#include <utility>
std::map<std::pair<uint32_t,uint32_t>, Ptr<SatNetDevice>> m_beamToggleMap;

// BuildBeamToggleMap()
uint32_t satId  = mac->GetSatId();   // SatMac::GetSatId() ✓ 存在於 ns-3.43
uint32_t beamId = mac->GetBeamId();  // SatMac::GetBeamId() ✓
auto key = std::make_pair(satId, beamId);
m_beamToggleMap[key] = dev;

// OBC callbacks (三處)
auto toggleIt = m_beamToggleMap.find({satId, beamId});
```

**[3] 過時 comment 仍寫 `SatNetDevice::GetBeamId()`**

```cpp
// 修正前：
// NOTE: Uses SatNetDevice::GetBeamId()

// 修正後：
// NOTE: Uses SatMac::GetSatId() and SatMac::GetBeamId()
//       SatNetDevice has no GetBeamId().
```

---

## 3. 修改摘要表

### `sat-bh-helper.h`

| 修改 | 說明 |
|---|---|
| `#include <utility>` 新增 | `std::pair` for toggle map key |
| `BhExperimentConfig` Phase G 欄位 | `enableDynamicBstp`, `bhDemandBacklogWeight`, `bhFairnessWeight`, `bhValiditySuperframes`, `bhStarvationThreshold` |
| `BuildBeamToggleMap()` private method 宣告 | 獨立於 ResourceManager |
| `SetupDynamicBstp()`, `RunDynamicBstpCycle()`, `ConfToTimePlan()` 宣告 |dynamic 核心 |
| `m_dynamicProvider`, `m_dynamicPlanId`, `m_lastDynamicPlan` | Phase G state |
| `m_beamToggleMap` 型別 | `map<uint32_t,…>` → `map<pair<uint32_t,uint32_t>,…>` |

### `sat-bh-helper.cc`

| 修改 | 說明 |
|---|---|
| `#include "sat-greedy-bstp-provider.h"` | 具體實作類別 |
| `#include "ns3/satellite-mac.h"` | `SatMac::GetBeamId()` / `GetSatId()` |
| `Install()` 啟動區塊 | `enableDynamicBstp && !enableScheduler` → `SetupDynamicBstp()` |
| `Install()` OBC 區塊 | `SetupObc()` 後立即 `BuildBeamToggleMap()`（與 RM 解耦） |
| `GetCurrentPlan()` | 加 `if (m_lastDynamicPlan) return m_lastDynamicPlan` |
| `BuildBeamToggleMap()` | key 改 `{satId, beamId}`；加 initial `ToggleState(false)` |
| `SetupObc()` callbacks | `find({satId, beamId})` |
| `SetupCacheQueue()` callback override | `find({satId, beamId})` |
| `RunDynamicBstpCycle()` | `periodStart = now + T_prop`；更新 `m_lastDynamicPlan` |
| `OnBacklogRequestTrace()` | 末尾加 `m_dynamicProvider->UpdateBeamDemand()` |
| `SetupDynamicBstp()` (新) | 建立 `SatGreedyBstpProvider`，從 `SatTopology` 讀 beam |
| `ConfToTimePlan()` (新) | 將 `Conf::activeBeams` round-robin 分配到 M 個 slot |

### `sat-bh-example.cc`

| 修改 | 說明 |
|---|---|
| `ParseConfig()`  CLI 參數（5 個）| `--enableDynamicBstp` 等 |
| BSTP conflict 防護 | `EnableFwdLinkBeamHopping=false`（取代錯誤的 `BH_OFF`） |
| Step 7 startup print | Phase G 資訊列印 |
| USAGE comment | 新增 Phase G 使用範例 |

---


## 5. 首次輸出（bh_dynamic）

**執行指令：**
```bash
./ns3 run "Codes/sat-bh-example --enableObc=1 --enableDynamicBstp=1 --simTime=30"
```

**輸出檔案：**
- `bh-timeplan.csv`：初始靜態計畫（Install 時寫出，不含動態計畫）
- `bh-metrics.csv`：OBC 執行期間的 beam KPI

### 5.1 bh-timeplan.csv

19 slots（M = ⌈503/26.5⌉），K=2，3 hotspot (beam 1–3) + 4 non-hotspot (beam 4–7)：

```
slot 0:  beam1(SMALL) + beam4(LARGE)
slot 1:  beam2(SMALL) + beam5(LARGE)
slot 2:  beam3(SMALL) + beam6(LARGE)
slot 3:  beam1(SMALL) + beam7(LARGE)
...（hotspot round-robin + nonHotspot round-robin）
```

### 5.2 bh-metrics.csv

**關鍵發現：beam 活動集合每 T_p 週期不同，以 7 個週期為一循環。**

| 週期 | t (s) | 高活躍 beams (17 slots) | 低活躍 beams (2 slots) |
|---|---|---|---|
| 1 | 10.503 | 6, 7 | 4, 5 |
| 2 | 11.006 | 1, 2 | 6, 7 |
| 3 | 11.509 | 3, 4 | 1, 2 |
| 4 | 12.012 | 5, 6 | 3, 4 |
| 5 | 12.515 | 1, 7 | 5, 6 |
| 6 | 13.018 | 2, 3 | 1, 7 |
| 7 | 13.521 | 4, 5 | 2, 3 |
| **8** | **14.024** | **6, 7 ← 第 1 週期重現** | **4, 5** |

7 週期 × 503ms = 3521ms；t=10.503 + 3.521 = 14.024 ✓ 

### 5.3 dwell_time 數值解讀

- `T_s = 26.5ms`，`T_sw = 2ms`，`usable = 24.5ms`
- SatBhMetrics 以整數 ms 記錄 → 24.5ms 截斷為 24ms
- `dwell_time_ms = 48ms` = 2 slots × 24ms ← beam 低活躍
- `dwell_time_ms = 408ms` = 17 slots × 24ms ← beam 高活躍
- 2 + 17 + 2 + 17 = 38 = M × K = 19 × 2 ✓

`slot_util_pct = 94.117% = 16/17`（17-slot beams）：最後一個 slot 的 `OnSlotDeactivated` 恰好落在 metrics period flush 邊界，只計到 16 個完整的 usedDur，屬正常邊界行為。

### 5.4 throughput = 0 的原因

此次執行未啟用 `--enablePhaseF`，`SatBhMetrics::OnPacketReceived()` 從未被呼叫（沒有 SNS3 真實封包注入 metrics）。**Phase G OBC 的 `ToggleState` 控制已正確執行，只是 metrics 封包計數需接入後才成立**

### 5.5 t=0 暖機資料

OBC 在 t=0 就開始發 `OnSlotActivated` 事件，metrics 暫存後在 warmup 結束時一次性輸出，timestamp 仍為 0。

**修正方向：** 在 `SatBhMetrics::Flush()` 加 `if (time_s < m_warmUpSec) continue;` 過濾。

### 5.6 整體評估

| 項目 | 結果 |
|---|---|
| OBC 執行 | beam 集合每週期動態切換 |
| Fairness-based 輪轉（無 demand） | 7 週期循環確認 |
| 多週期 ToggleState 驅動 |（build log 需確認 `BuildBeamToggleMap: X mapped`）|
| throughput 統計 | 0，需接入後才有真實數值 |
| t=0 暖機前資料洩漏 | 需 metrics flush 加 guard |

---

## 6. 已知未解問題

| # | 問題 | 影響 | 解法方向 |
|---|---|---|---|
| 1 | beam metadata freq/GW ID = 0（placeholder） | `ValidateConf` 的 feederFreq 衝突檢查失效 | Phase E 啟動後 call `UpdateBeamInfo()` API（未實作） |
| 2 | `SatDynamicBstpProvider::GetTypeId()` 在 greedy `.cc` | 架構不乾淨 | 移至 base class `.cc` |
| 3 | 未整合進 `SatBstpController::BH_DYNAMIC` 正式流程 | Phase G 是 parallel controller，非 SNS3 原生 dynamic mode | 需修改 `contrib/satellite/` 原始碼（目前不在允許範圍） |
| 4 | `t=0` metrics 資料洩漏到 CSV | 分析時需手動過濾 | `SatBhMetrics::Flush()` 加 warmup guard |
| 5 | Phase G 無 real demand 時退化為 fairness round-robin | 無 Phase F 時無法驗證 demand-aware 選 beam | 需配合 Phase F 跑一次完整實驗 |

---

