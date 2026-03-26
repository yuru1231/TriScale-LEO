# Complete all offline pre-computed modules and verify schedule execution.
**工作日誌 2026-03-23**

## 目標
完成所有離線預計算模組並驗證排程執行。\
[test-iridium](https://github.com/bmw-ntust-internship/Lucy/blob/5956a66e1a585e8ff33a1587cd895dddf57394c6/TriScale-LEO/Topology%20%26%20ISL%20Routing/Predict/Codes/v1_test-iridium.cc)
---

## 完成事項

### 1. `GetGeoPositionAt()` 加入 SNS3

**現象**：離線預計算階段需要在 `Simulator::Run()` 之前查詢任意 τ_k 的衛星位置，但呼叫現有 `GetGeoPosition()` 回傳的永遠是 `Simulator::Now()`（模擬尚未開始時為 t=0）的位置。

**原因**：`DoGetGeoPosition()` 內部綁定 `Simulator::Now()`，無公開接口接受任意時間參數。

**修正**：新增 `GetGeoPositionAt(Time t)` public 方法至 `satellite-sgp4-mobility-model.h/.cc`，直接代入 τ_k 執行 SGP4 解算，不依賴模擬時鐘。

**驗證**：`BuildISLGraph(t=0)` 可正確取得 66 顆衛星位置。

---

### 2. `LoadISLDefs()` + `BuildISLGraph(τ_k)`

**現象**：初版以 2500 km 為距離門檻時，SAT0 在 t=0 的 eligible ISL 數量為 0，拓樸圖斷裂。

**原因**：跨軌道面 ISL 瞬時距離可達 ~4800 km，2500 km 門檻過嚴。詳見 `decisions/DEC-002-isl-distance-threshold.md`。

**修正**：距離門檻改為 5000 km。

**現象（第二個）**：確認 `ifIndex` 對應時，發現直接用 `isls.txt` 讀入順序與 `GetIslsNetDevices()` vector 順序不一致。

**原因**：SNS3 的 device 掛載順序由建立時機決定，非 `isls.txt` 定義順序。

**修正**：改用 `peerNodeIdToIfIdx`，以 peer nodeId 直接查 vector index。

**驗證**：SAT0 在 t=0 有 2 條 eligible ISL（peer=1, peer=10），`ifIdxOnA` 對應正確 ✅

---

### 3. `ComputeBaseRoutes()`

**現象**：無異常，直接實作通過。

**原因**：純內部 Dijkstra，不依賴 SNS3 API。

**修正**：per-source Dijkstra，輸入 `ISLGraph`，每顆衛星輸出 65 條路由。

**驗證**：SAT0 路由表 nextHop=1 與 nextHop=10 分別覆蓋不同方向 ✅

---

### 4. `ApplyTiebreaker()`

**現象**：無異常，直接實作通過。

**修正**：比對 τ_k+1 的 eligible ISL 集合，等 cost 路徑優先選下一時間點仍 eligible 的 ISL。

**驗證**：兩次遍歷正確，τ_9（t=540）無下一個時間點，不執行 tiebreaker ✅

---

### 5. `PrecomputeAllTables()`

**現象**：無異常，直接實作通過。

**修正**：遍歷 10 個時間點（t=0, 60, ..., 540s），每個時間點執行 `BuildISLGraph` → `ComputeBaseRoutes` → `ApplyTiebreaker`。

**驗證**：SAT0 每個時間點均有 65 條路由 ✅

---

### 6. `InitOrbiterDevices()` + `ApplyRoutingTable()` + `ScheduleRoutingUpdates()`

**現象**：排程執行時呼叫 `CreateObject<SatIslArbiterUnicast>()` 建立新 arbiter，模擬 crash，輸出 fatal error。

**原因**：`CreateObject` 走 default constructor，`SatIslArbiterUnicast` 的 default constructor 缺少必要的初始化參數，導致 fatal。詳見 `decisions/DEC-003-arbiter-lifecycle.md`。

**修正**：
- 新增 `InitOrbiterDevices()`，在 `RunSimulation()` 前預先建立並安裝 66 個 arbiter
- 排程執行時只呼叫 `ClearNextHopEntries()` + `AddNextHopEntry()`，不再重新建立 arbiter
- 新增 `ClearNextHopEntries()` 至 `SatIslArbiterUnicast`

**驗證**：排程執行正常，`ApplyRoutingTable: done` 確認 ✅

---

## 修改的檔案

| 檔案 | 位置 | 異動 |
|------|------|------|
| `isl-graph.h` | `contrib/satellite/helper/` | 新增 |
| `isl-graph.cc` | `contrib/satellite/helper/` | 新增 |
| `satellite-sgp4-mobility-model.h/.cc` | `contrib/satellite/model/` | 加入 `GetGeoPositionAt()` |
| `satellite-isl-arbiter-unicast.h` | `contrib/satellite/model/` | 加入 `ClearNextHopEntries()` |
| `test-iridium.cc` | `scratch/` | 驗證腳本更新 |

以下是所有修改的詳細說明：

---

### 1. `satellite-sgp4-mobility-model.h`
**位置**：`contrib/satellite/model/`

**加入位置**：`SetTleInfo()` 宣告下方的 public 區塊

**新增內容**：
```cpp
/**
 * @brief Retrieve satellite's position at an arbitrary simulation time.
 * @param t Simulation time (e.g., Seconds(60)).
 * @return GeoCoordinate in ITRF frame, or default GeoCoordinate() if not initialized.
 */
GeoCoordinate GetGeoPositionAt(Time t) const;
```

**功能**：允許在任意時間點 τ_k 查詢 SGP4 位置，不受 `Simulator::Now()` 限制，是離線預計算的基礎。

---

### 2. `satellite-sgp4-mobility-model.cc`
**位置**：`contrib/satellite/model/`

**加入位置**：`DoGetGeoPosition()` 實作後方

**新增內容**：
```cpp
GeoCoordinate
SatSGP4MobilityModel::GetGeoPositionAt(Time t) const
{
    if (!IsInitialized())
        return GeoCoordinate();

    JulianDate cur = m_start + t;
    double delta = (cur - GetTleEpoch()).GetMinutes();

    double r[3], v[3];
    sgp4(WGeoSys, m_sgp4_record, delta, r, v);

    if (m_sgp4_record.error != 0)
        return GeoCoordinate();

    return rTemeTorItrf(Vector3D(r[0], r[1], r[2]), cur) * 1000;
}
```

**功能**：實作任意時間點的 SGP4 位置計算。流程為 `sgp4() → rTemeTorItrf() → ×1000 → GeoCoordinate`，回傳 ITRF 座標系的地理座標。

---

### 3. `satellite-isl-arbiter-unicast.h`
**位置**：`contrib/satellite/model/`

**加入位置**：`AddNextHopEntry()` 宣告下方的 public 區塊

**新增內容**：
```cpp
/**
 * Clear all next hop entries.
 */
void ClearNextHopEntries() { m_nextHopMap.clear(); }
```

**功能**：允許在不重建 arbiter 物件的情況下清空路由表，供 `ApplyRoutingTable()` 更新路由時使用。

---

### 4. `isl-graph.h`
**位置**：`contrib/satellite/helper/`（新增檔案）

**完整內容**：
```cpp
#ifndef ISL_GRAPH_H
#define ISL_GRAPH_H

#include "ns3/nstime.h"
#include "ns3/singleton.h"
#include "ns3/satellite-sgp4-mobility-model.h"
#include "ns3/satellite-topology.h"

#include <array>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

struct ISLEdge
{
    uint32_t nodeB;
    double   propagation_cost;  // dist/c，單位：秒
    uint32_t islIfIndexOnA;     // A 的 GetIslsNetDevices() index
    uint32_t islIfIndexOnB;     // B 的 GetIslsNetDevices() index
};

struct ISLDef
{
    uint32_t nodeA;
    uint32_t nodeB;
};

struct RouteEntry
{
    uint32_t destSatId;
    uint32_t nextHopSatId;
    uint32_t islIfIndexOnA;
    double   cost;
};

using ISLGraph          = std::vector<std::vector<ISLEdge>>;
using RoutingTable      = std::vector<std::vector<RouteEntry>>;
using PrecomputedTables = std::array<RoutingTable, 10>;

void         LoadISLDefs(const std::string& islsFilePath);
void         InitOrbiterDevices();
ISLGraph     BuildISLGraph(Time tau_k);
RoutingTable ComputeBaseRoutes(const ISLGraph& graph);
RoutingTable ApplyTiebreaker(const RoutingTable& routes,
                              const ISLGraph&     graphNext);
void         PrecomputeAllTables(PrecomputedTables& tables);
void         ApplyRoutingTable(uint32_t slotIndex,
                               const PrecomputedTables& tables);
void         ScheduleRoutingUpdates(const PrecomputedTables& tables);

} // namespace ns3

#endif // ISL_GRAPH_H
```

---

### 5. `isl-graph.cc`
**位置**：`contrib/satellite/helper/`（新增檔案）

各函式功能說明：

| 函式 | 功能 |
|------|------|
| `LoadISLDefs()` | 讀取 `isls.txt`，建立 `islDefs[]` 和 `perSatISLOrder[]`，記錄每顆衛星的 ISL interface index 對應關係 |
| `InitOrbiterDevices()` | 快取 66 顆衛星的 `Node` 和 `SatOrbiterNetDevice`，供後續操作使用 |
| `BuildISLGraph(τ_k)` | 在時間點 τ_k 計算所有 ISL 的距離，過濾距離 > 5000 km 的連線，建立帶 propagation_cost 的 adjacency list |
| `ComputeBaseRoutes()` | 對 66 顆衛星各跑一次 Dijkstra，產生完整路由表，包含 nextHopSatId 和 islIfIndexOnA |
| `ApplyTiebreaker()` | 比對 τ_k+1 的 eligible ISL 集合，等 cost 路徑優先選下一時間點仍 eligible 的 ISL |
| `PrecomputeAllTables()` | 遍歷 10 個時間點，依序呼叫 `BuildISLGraph` → `ComputeBaseRoutes` → `ApplyTiebreaker` |
| `ApplyRoutingTable()` | 對每顆衛星建立新的 `SatIslArbiterUnicast`，填入路由表後呼叫 `SetArbiter()` 注入 |
| `ScheduleRoutingUpdates()` | 在 `Simulator::Run()` 前排入 10 個排程事件，每 60 秒觸發一次 `ApplyRoutingTable()` |

---

### 6. `CMakeLists.txt`
**位置**：`contrib/satellite/`

**修改內容**：在 `source_files` 加入：
```cmake
helper/isl-graph.cc
```
在 `header_files` 加入：
```cmake
helper/isl-graph.h
```

---

## 驗證結果

| 驗證項目 | 輸出 | 結論 |
|----------|------|------|
| `GetGeoPositionAt(Seconds(0))` | SAT0 ECEF: `-1.2146e+06 -6.90455e+06 -14927.1` | SGP4 位置查詢正確 ✅ |
| SAT0-SAT1 距離 | `3951.79 km` | ECEF 距離計算正確，prop_cost=0.01317s ✅ |
| islInterfaceIndex 對應 | SAT0 islDev[0]→peer=1, islDev[2]→peer=10 與 graph 一致 | index 對應正確 ✅ |
| `PrecomputeAllTables()` | 10 個時間點各 SAT0 routes=65 | 所有時間點路由完整 ✅ |
| `ApplyRoutingTable()` 排程 | slot=0 t=0s done, slot=1 t=60s done ... slot=9 t=540s done | 10 次排程全部正常觸發 ✅ |
---

## non-OOP n.s OOP
[non-OOP isl-graph.cc](https://github.com/bmw-ntust-internship/Lucy/blob/5956a66e1a585e8ff33a1587cd895dddf57394c6/TriScale-LEO/Topology%20%26%20ISL%20Routing/Predict/Codes/v1_isl-graph.cc)
[OOP isl-graph.cc](https://github.com/bmw-ntust-internship/Lucy/blob/5956a66e1a585e8ff33a1587cd895dddf57394c6/TriScale-LEO/Topology%20%26%20ISL%20Routing/Predict/Codes/v2_isl-graph.cc)
兩份程式碼， **`LoadISLDefs` 和 `InitOrbiterDevices` 的呼叫時機**，以及 **`m_numSatellites` 尚未透過 Config 設定就被使用**。

---

## 問題一：`m_numSatellites` 在 `Initialize()` 之前就被 Attribute 預設值固定了，但 `LoadISLDefs` 內部直接用它

```cpp
// OOP 版本的 LoadISLDefs
m_perSatISLOrder.assign(m_numSatellites, ...);  // 如果 Config 還沒設，這裡是預設值 66
```

當你用 `Config::Set(...)` 或 `CreateObjectWithAttributes<>()` 設定 `NumSatellites` **之後**才呼叫 `Initialize()`，這本身沒問題。

但如果 `Initialize()` 在物件建構子裡或 `Config` 尚未生效前被呼叫，`m_numSatellites` 就是建構子預設值 `66`，**不是 Config 設定值**。

---

## 問題二：`PrecomputeAllTables()` 裡 `m_tables.resize(m_numTimeSlots)` 和迴圈上界都依賴 Attribute，但 ns-3 的 Attribute 是**物件建立後才可設定**

```cpp
void IslRoutingManager::PrecomputeAllTables()
{
    m_tables.resize(m_numTimeSlots);  // ← 此時 m_numTimeSlots 是否已是 Config 值？
    for (uint32_t k = 0; k < m_numTimeSlots; k++)
```

ns-3 的 `Config::Set("...NumTimeSlots", ...)` 透過 TypeId Attribute 機制設定，**只在物件已存在且路徑正確時生效**。若呼叫順序是：

```
CreateObject → PrecomputeAllTables → Config::Set   ← 錯誤順序
```

那 `m_numTimeSlots` 永遠是預設值。

---

## 問題三：non-OOP 版本的全域函式沒有這個問題的原因

non-OOP 版的參數都是 **hardcode 常數或函式參數**，不依賴 ns-3 Attribute 機制，所以呼叫順序不影響行為：

```cpp
// 永遠是這個值，不受 Config 影響
for (uint32_t k = 0; k < 10; k++)
```

---

## 解法建議

**在 `Initialize()` 開頭加上防禦性斷言**，確認 Attribute 已正確設定：

```cpp
void IslRoutingManager::Initialize(const std::string& islsFilePath)
{
    NS_ASSERT_MSG(m_numSatellites > 0, "NumSatellites not configured");
    NS_ASSERT_MSG(!islsFilePath.empty(), "IslsFilePath not configured");
    // ...
}
```

**並確保呼叫順序**：

```cpp
Ptr<IslRoutingManager> mgr = CreateObject<IslRoutingManager>();
mgr->SetAttribute("NumSatellites", UintegerValue(66));   // ← 先設
mgr->SetAttribute("NumTimeSlots",  UintegerValue(10));   // ← 先設
mgr->Initialize(islsFilePath);                           // ← 後呼叫
mgr->PrecomputeAllTables();
mgr->ScheduleRoutingUpdates();
```

---

## 總結

| 問題點 | non-OOP | OOP (Config) |
|--------|---------|--------------|
| 參數來源 | hardcode，穩定 | Attribute，依賴設定時機 |
| `resize` 大小 | 固定 66/10 | 依 `m_numSatellites` 當下值 |
| 初始化順序敏感 | 否 | **是** |

**ns-3 Attribute 的賦值是非同步/順序依賴的，不像建構子參數那樣保證在物件可用前已完成設定**。

## 明日計畫
1. 修正參數，改為傳值呼叫
2. 實作 `UpdateLoadCosts()`：EMA + load_cost 計算
3. 實作 `HasSignificantChange()`：Hysteresis + Cooldown
4. 實作 `RecomputeAffectedRoutes()`：局部 Dijkstra
5. 整合進 `my-simulation.cc`，加入實際流量
