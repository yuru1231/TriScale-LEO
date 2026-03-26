# 工作日誌 2026-03-25

## 目標
加入 UDP 流量驗證動態路由（v5 OOP），並確認 FtVisibilityFilter 與 BeamHoppingManager 背景整合行為正確。

---

## 完成事項

### 1. 修正 ApplyRoutingTable（DEC-003 實作）

**現象**：每次呼叫 `ApplyRoutingTable()` 都 `CreateObject<SatelliteIslArbiterUnicast>()` 重建 arbiter，高頻觸發時造成不必要的物件分配。

**原因**：原始設計未考慮重複套用場景，DEC-003 決定改用預先建立的 arbiter 物件並透過 `ClearNextHopEntries()` 清空再填入，避免每槽重複建立。

**修正**：在 `InitOrbiterDevices()` 中為 66 顆衛星預先建立 arbiter；`ApplyRoutingTable()` 改呼叫 `ClearNextHopEntries()` + `AddNextHopEntry()`，不再呼叫 `CreateObject`。

**驗證**：各槽 `apply ≤ 5ms`，無記憶體分配異常 ✅

→ 若有重要設計決策，見 `Decisions/DEC-003-Arbiter lifecycle management.md`

---

### 2. v5 OOP 重構完成

**現象**：v4 版本所有邏輯集中於全域函式，參數以 hard-code 常數散佈各處，難以在測試腳本中切換設定。

**原因**：缺乏封裝。依照 OOP 設計規則，需以 ns-3 `Object` 為基底類別，透過 Attribute 機制統一管理參數。

**修正**：新增 `IslRoutingManager`（繼承 `ns3::Object`），所有參數改以 ns-3 Attribute 設定（`NumSatellites`、`IslMaxDistanceKm`、`NumTimeSlots`、`TimeSlotInterval`、`EmaAlpha`、`ChangeThreshold`、`CooldownSeconds`、`IslLinkRateBps`）。新增診斷 API：`TracePath()`、`PrintRouteReport()`、`BlockISL()`、`UnblockISL()`、`RunAvoidanceTest()`。新增 `SlotStats` 結構記錄每槽 apply/recompute wall time 與 `significantChange` 旗標，由 `PrintStats()` 輸出完整統計表。

**驗證**：測試腳本 `v5_test-iridium.cc` 使用新 OOP API，模擬正常啟動並完整輸出統計表 ✅

#### 程式碼比對：v4 → v5 主要差異

**1. 參數定義方式（hard-code → Attribute）**

v4（`v4_isl-graph.h`）：
```cpp
// private 成員，無外部可見性，直接用預設值
uint32_t m_numSatellites;
double   m_islMaxDistanceKm;
uint32_t m_numTimeSlots;
// 無 GetTypeId 或 Attribute 機制
```

v5（`v5_isl-graph.h`）：
```cpp
static TypeId GetTypeId();  // ← 新增：ns3 Attribute 系統接入點
// 在 .cc 中：
TypeId IslRoutingManager::GetTypeId() {
    static TypeId tid = TypeId("ns3::IslRoutingManager")
        .SetParent<Object>()
        .AddAttribute("NumSatellites", ..., UintegerValue(66), ...)
        .AddAttribute("IslMaxDistanceKm", ..., DoubleValue(5000.0), ...)
        // ... 8 個 Attribute 全部可在腳本中覆寫
    return tid;
}
```

**2. 診斷 API（v5 新增，v4 無）**

```cpp
// v5 新增（v4_isl-graph.h 無以下宣告）
std::vector<uint32_t> TracePath(uint32_t src, uint32_t dst, uint32_t slotIndex) const;
void PrintRouteReport(const std::vector<std::pair<uint32_t,uint32_t>>& pairs) const;
void BlockISL(uint32_t nodeA, uint32_t nodeB);
void UnblockISL(uint32_t nodeA, uint32_t nodeB);
void RunAvoidanceTest(uint32_t testSrc, uint32_t testDst, uint32_t slotIndex);
```

**3. 跨層 Accessor（v5 新增，供 Layer 2/3 使用）**

```cpp
// v5 新增（v4 無）
uint32_t GetNumTimeSlots()     const { return m_numTimeSlots; }
uint32_t GetNumSatellites()    const { return m_numSatellites; }
double   GetTimeSlotInterval() const { return m_timeSlotInterval; }
double   GetRouteCost(uint32_t src, uint32_t dst, uint32_t slotIndex) const;
```

Layer 2（`BeamHoppingManager`）透過 `GetNumTimeSlots()` 與 `GetTimeSlotInterval()` 讓 BH 排程的時槽定義與 ISL 路由完全一致。

**4. ISL 封鎖集合（v5 新增，供 `RunAvoidanceTest` 使用）**

```cpp
// v5 新增（v4 無）
std::set<std::pair<uint32_t, uint32_t>> m_blockedEdges;
// BuildISLGraph 中新增過濾邏輯：
if (m_blockedEdges.count({nodeA, nodeB})) continue;  // 跳過被封鎖的 ISL
```

---

### 3. 加入 UDP 流量，觸發動態路由

**現象**：無流量時，`PrintStats` 所有槽均顯示 `changed=NO`（ISL queue delay 永遠為 0，EMA 不變）。

**原因**：`UpdateLoadCosts()` 依賴 ISL 裝置的佇列延遲讀取，無流量時延遲恆為 0，`HasSignificantChange()` 永遠回傳 false。

**修正**：在 E2E 腳本中加入 UDP 流量源，讓 ISL 有實際佇列延遲可供 `UpdateLoadCosts()` 讀取。

**驗證**：slot 1–10 全部觸發 `HasSignificantChange=YES`，`recomputedSrc` 從 3 增長到 51 ✅

| slot | recomputedSrc | 說明 |
|------|---------------|------|
| 0 | 0 | 初始槽不觸發（slot=0 跳過 UpdateLoadCosts） |
| 1 | 3 | 流量剛開始，只有少數 ISL 受影響 |
| 5 | 33 | 流量持續累積，受影響 source 增加 |
| 10 | 51 | 模擬末期，超過半數 satellite 路由受影響 |

---

### 4. FtVisibilityFilter 可見性驗證

**現象**：需確認 12 個時槽的仰角過濾結果是否正確反映星座移動。

**原因**：FtVisibilityFilter 依賴 `GetGeoPositionAt()` 查詢各槽衛星位置，若時間參數錯誤會導致全槽結果相同。

**修正**：無需修正，驗證既有實作。

#### 程式碼比對：FtVisibilityFilter（新增模組，無前一版本）

`ft-filter.h/cc` 是全新模組，與 `IslRoutingManager` 解耦，透過 `SetRoutingManager()` 注入相依。

**新增資料結構**：
```cpp
struct FtDef { uint32_t id; double latDeg; double lonDeg; std::string name; };

struct FtTransitRoute {
    uint32_t ftSrc, ftDst;
    uint32_t entrySatId;  // ftSrc 可見、作為路徑入口的衛星
    uint32_t exitSatId;   // ftDst 可見、作為路徑出口的衛星
    double   islCost;     // entrySat→exitSat 的 ISL 路由代價（秒）
    bool     valid;
};
```

**核心查詢流程（`GetBestTransit`）**：
```cpp
FtTransitRoute FtVisibilityFilter::GetBestTransit(uint32_t ftI, uint32_t ftJ, uint32_t slotIndex)
{
    // 1. 取 ftI 可見衛星集合（已在 PrecomputeVisibility 離線計算）
    const auto& visI = GetAccessSats(ftI, slotIndex);
    const auto& visJ = GetAccessSats(ftJ, slotIndex);

    // 2. 暴力枚舉所有 (entryS ∈ visI, exitS ∈ visJ) 組合
    // 3. 對每對呼叫 m_routingMgr->GetRouteCost(entryS, exitS, slotIndex)
    // 4. 回傳最小 cost 的組合
}
```

**與 `IslRoutingManager` 的耦合點**（v5 新增 Accessor 後才能運作）：
```cpp
m_numSlots      = m_routingMgr->GetNumTimeSlots();      // v5 新增
m_numSatellites = m_routingMgr->GetNumSatellites();     // v5 新增
const double slotInterval = m_routingMgr->GetTimeSlotInterval(); // v5 新增
// cost 查詢：
double cost = m_routingMgr->GetRouteCost(entryS, exitS, k); // v5 新增
```

> 若使用 v4 版本（無 Accessor），`FtVisibilityFilter` 無法編譯。這是 v4→v5 升級的直接驅動原因之一。

**驗證**：
```
FT0[TW-Taipei]:       各槽 1–2 顆可見衛星
FT1[JP-Tokyo]:        各槽 2–3 顆可見衛星
FT2[US-SanFrancisco]: 各槽 1–2 顆可見衛星
```
Slot 0 最佳路徑（合約對）：
```
FT0 → FT1: entry=sat15 exit=sat15 cost=0.0000s（同星中繼）
FT0 → FT2: entry=sat15 exit=sat37 cost=0.0440s（跨星路徑，44ms propagation）
```
各槽可見衛星隨時間變動，確認仰角過濾正確運作 ✅

---

### 5. BeamHoppingManager 背景運行確認

**現象**：需確認 BH 模組在 E2E 腳本中作為背景模組運行時，不干擾 ISL routing 事件排程。

**原因**：兩模組均向 ns-3 Simulator 排程事件，需確認時間軸上無衝突。

**修正**：無需修正，驗證既有隔離設計。

**驗證**：
```
[BeamHoppingManager] LoadOrbiterNodes: 66 nodes loaded
[BeamHoppingManager] ComputeBhSchedule: 64 events across 12 slots
[BeamHoppingManager] ScheduleBhUpdates: 64 events registered
```
64 個 BH 事件全程不干擾 ISL routing 邏輯 ✅

---

### 6. Wall Time 確認（DEC-004）

**現象**：`simTime=630s` 的模擬 wall time 為 2775.79s（約 4.4× 放大）。

**原因**：SNS3 DVB MAC beam scheduler 固有開銷，與 ISL 邏輯無關。`PrecomputeAllTables` 僅 5ms，ISL 完全不是瓶頸。詳見 `Decisions/DEC-004-Beam Scheduler.md`。

**修正**：無需修正，確認為已知行為，暫不優化。

**驗證**：`PrecomputeAllTables` wall=5ms，`ApplyRoutingTable` 各槽 ≤ 5ms，wall time 來源已明確定位至 SNS3 DVB MAC 層 ✅

→ 若有重要設計決策，見 `Decisions/DEC-004-Beam Scheduler.md`

---

## 驗證結果總表

| 驗證項目 | 輸出 | 結論 |
|----------|------|------|
| LoadISLDefs | `loaded=132 ISLs` | ISL 定義正確 ✅ |
| InitOrbiterDevices | `satellites=66` | 衛星裝置快取正確 ✅ |
| PrecomputeAllTables | 12 個時槽，SAT0_routes=65，wall=5ms | 離線預計算完整 ✅ |
| ScheduleRoutingUpdates | `12 events scheduled` | 排程事件正確 ✅ |
| FtFilter 可見性 | 12 槽動態可見衛星數 1–3 顆 | 仰角過濾正確 ✅ |
| HasSignificantChange | slot 1–10 全部 YES（UDP 流量下） | 動態路由觸發正確 ✅ |
| RecomputeAffectedRoutes | 局部重算 3–51 源，不全量重算 | 局部 Dijkstra 正確 ✅ |
| ApplyRoutingTable | 各槽 apply ≤ 5ms | Arbiter 更新效能正常 ✅ |
| BH 背景運行 | 64 BH 事件，不干擾 ISL | 模組隔離正確 ✅ |
| Wall time | 2775.79s（DEC-004 已知） | 暫不處理，行為合理 ✅ |

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v5_isl-graph.h` | 新增 `IslRoutingManager` class（OOP），新增 `TracePath`、`PrintRouteReport`、`BlockISL`、`UnblockISL`、`RunAvoidanceTest` API，新增 `SlotStats` 結構 |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v5_isl-graph.cc` | 完整 OOP 實作，Fix 1–4 全部保留；`InitOrbiterDevices()` 預建 66 個 arbiter；`ApplyRoutingTable()` 改用 `ClearNextHopEntries()` + `AddNextHopEntry()` |
| `C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\v5_test-iridium.cc` | 驗證腳本，使用新 OOP API 建立 `IslRoutingManager` 並執行完整驗證 |
| `C:\Users\wenj\Desktop\TriScale-LEO\E2E\Codes\v1_e2e-iridium.cc` | E2E 整合腳本，加入 UDP 流量源，整合 Layer 1（ISL + FtFilter）與 Layer 2（BH）背景運行 |
| `C:\Users\wenj\Desktop\TriScale-LEO\E2E\Outputs\v1_output.md` | E2E 完整執行輸出記錄 |

---

## 備註

- E2E v1 尚未確認為最終版本，**不整合**。
- 架構保持分層：Layer 1（ISL + FT Filter）/ Layer 2（BH）獨立開發與驗證。
- 完整輸出存於 `C:\Users\wenj\Desktop\TriScale-LEO\E2E\Outputs\v1_output.md`。

---

## 明日計畫

- 確認 Layer 2（Beam Hopping）獨立驗證計畫
