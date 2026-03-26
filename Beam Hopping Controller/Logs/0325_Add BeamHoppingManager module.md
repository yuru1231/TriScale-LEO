# 工作日誌 2026-03-25

## 目標
建立 Layer 2 `BeamHoppingManager` 模組，實作衛星波束跳頻排程（Beam Hopping），並整合進 E2E 腳本作為背景模組驗證。

---

## 新增模組總覽

| 檔案 | 說明 |
|------|------|
| `beam-hopping-manager.h` | Layer 2 BH 管理器介面（OOP，繼承 `ns3::Object`） |
| `beam-hopping-manager.cc` | 完整實作：離線排程計算 + NS3 事件注入 |

本模組由零開始建立，無前一版本可比對。

---

## 新增結構與類別說明

### 1. 資料結構

#### `CellDef` — 地面 Cell 定義
```cpp
// 新增（無前一版本）
struct CellDef {
    uint32_t    id;
    double      latDeg;
    double      lonDeg;
    std::string name;
};
```
**功能**：描述一個地面 Cell 的地理位置（緯度/經度）與識別碼，供衛星可視性計算使用。

---

#### `BhEvent` — 波束切換事件
```cpp
// 新增（無前一版本）
struct BhEvent {
    Time     t;        // 事件觸發時間（NS3 Time）
    uint32_t satId;    // 觸發切換的衛星 ID
    uint32_t cellId;   // 目標 Cell ID
};
```
**功能**：代表「某時刻，某衛星切換到某 Cell」的排程事件，最終注入 NS3 Simulator。

---

#### `TrafficDemandProvider` — 流量需求介面（可插拔）
```cpp
// 新增（無前一版本）
class TrafficDemandProvider {
  public:
    virtual double GetDemandMbps(uint32_t cellId, Time t) const = 0;
};

class UniformDemandProvider : public TrafficDemandProvider {
    // 所有 Cell 回傳相同固定需求（預設 1.0 Mbps）
    double GetDemandMbps(uint32_t, Time) const override { return m_mbps; }
};
```
**功能**：抽象介面，讓 BH 排程演算法可依各 Cell 的實際流量需求分配 superframe 時槽。預設實作 `UniformDemandProvider` 等量分配。之後可替換為真實流量感知實作。

---

### 2. 主類別 `BeamHoppingManager`（繼承 `ns3::Object`）

#### NS3 Attribute 參數
| Attribute | 預設值 | 說明 |
|-----------|--------|------|
| `NumSatellites` | 66 | 星座衛星數 |
| `SuperframeDurationSec` | 0.25 | DVB-S2 superframe 週期（秒） |
| `ElevationThresholdDeg` | 5.0 | 最小仰角（度），低於此值的 Cell 視為不可見 |

所有參數均可在腳本中透過 `CreateObjectWithAttributes` 或 `SetAttribute` 設定，不 hard-code。

---

### 3. 核心方法說明

#### `ComputeVisibleCells(satEcef)` — 計算可見 Cell

**功能**：對每顆衛星的 ECEF 位置，遍歷所有 Cell，呼叫 `FtVisibilityFilter::ComputeElevationDeg()` 計算仰角，過濾出仰角 ≥ 閾值的 Cell。

**依賴**：`ft-filter.h` 提供的 `ComputeElevationDeg` 靜態工具函式（Layer 1 共用）。

---

#### `ComputeBhSchedule(numSlots, slotIntervalSec)` — 離線排程計算

**舊**：無（本模組新增）

**新**：
```cpp
void BeamHoppingManager::ComputeBhSchedule(uint32_t numSlots, double slotIntervalSec)
{
    LoadOrbiterNodes();  // 載入 66 顆衛星的 SatSGP4MobilityModel
    // 對每個時間槽 k：
    //   取得衛星 ECEF 位置（SGP4）
    //   計算可見 Cell 清單
    //   依 TrafficDemandProvider 取得各 Cell 需求
    //   依需求比例分配 superframe 數量 → 生成 BhEvent
    // 最後對 m_schedule 按時間排序
}
```

**演算法**：
- 對每顆衛星，在時間槽 `k` 取衛星位置（SGP4）
- 找出所有仰角 ≥ 閾值的可見 Cell
- 將可見 Cell 依需求由高到低排序
- 按比例分配 superframe 時槽數 `numSF = round(demand/total * sfPerSlot)`
- 每次切換產生一個 `BhEvent{t, satId, cellId}`
- 全部事件排序後存入 `m_schedule`

**輸出**：
```
[BeamHoppingManager] ComputeBhSchedule: 64 events across 12 slots
```

---

#### `ScheduleBhUpdates()` — NS3 事件注入

**舊**：無

**新**：
```cpp
void BeamHoppingManager::ScheduleBhUpdates()
{
    for (const auto& ev : m_schedule)
        Simulator::Schedule(ev.t, &BeamHoppingManager::ApplyBhEvent,
                            this, ev.satId, ev.cellId);
}
```
**功能**：將離線計算好的 `m_schedule` 全部注入 NS3 Simulator 排程。與 Layer 1 的 `ScheduleRoutingUpdates()` 模式一致，兩者事件互不干涉。

---

#### `ApplyBhEvent(satId, cellId)` — 執行波束切換

```cpp
void BeamHoppingManager::ApplyBhEvent(uint32_t satId, uint32_t cellId)
{
    m_currentCell[satId] = cellId;   // 更新當前 Cell 狀態（供 Layer 3 查詢）
    if (m_switchCallback)
        m_switchCallback(satId, cellId, Simulator::Now());
    // TODO SNS3_BH_INJECT: 呼叫實際 SNS3 beam switch API
}
```
**功能**：更新 `m_currentCell` 映射表，並觸發可插拔的 `BhSwitchCallback`。實際 SNS3 API 呼叫以 TODO 標記待確認。

---

#### `GetCurrentCell(satId)` — Layer 3 查詢介面

```cpp
uint32_t BeamHoppingManager::GetCurrentCell(uint32_t satId) const
{
    auto it = m_currentCell.find(satId);
    return (it != m_currentCell.end()) ? it->second : UINT32_MAX;
}
```
**功能**：供 Layer 3（QoS 排程）查詢當前某衛星服務的 Cell，以決定封包排程優先級。UINT32_MAX 表示尚未分配。

---

## 與 Layer 1 的整合界面

| Layer 1 提供 | Layer 2 使用方式 |
|-------------|----------------|
| `IslRoutingManager::GetNumTimeSlots()` | 傳入 `ComputeBhSchedule(numSlots, ...)` |
| `IslRoutingManager::GetTimeSlotInterval()` | 傳入 `ComputeBhSchedule(..., slotIntervalSec)` |
| `FtVisibilityFilter::ComputeElevationDeg()` | `ComputeVisibleCells()` 內部呼叫 |

Layer 2 的時槽數與間距與 Layer 1 完全一致，確保排程事件時間對齊。

---

## E2E 驗證結果

在 `v5_test-iridium.cc` 中以背景模組運行，結果：

```
[BeamHoppingManager] LoadOrbiterNodes: 66 nodes loaded
[BeamHoppingManager] ComputeBhSchedule: 64 events across 12 slots
[BeamHoppingManager] ScheduleBhUpdates: 64 events registered
```

- 64 個 BH 事件在 12 個時槽內正確分佈 ✅
- BH 事件與 ISL routing 事件（12 個）完全獨立，無衝突 ✅
- `GetCurrentCell()` 在 BH 事件後可正確回傳對應 cellId ✅

---

## 待處理

| 項目 | 說明 |
|------|------|
| `TODO SNS3_BH_INJECT` | 確認並呼叫 SNS3 實際 beam switch API（`SetActiveBeam` 或 `SatBeamScheduler::EnableBeam`） |
| `TrafficDemandProvider` 實作 | 目前為 `UniformDemandProvider`，之後替換為基於 UDP 佇列統計的實作 |
| Layer 3 整合驗證 | 確認 `GetCurrentCell()` 與 QoS 封包排程的連接行為 |

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `Beam Hopping Controller/Codes/beam-hopping-manager.h` | 新建：定義 `CellDef`、`BhEvent`、`TrafficDemandProvider`、`BeamHoppingManager` |
| `Beam Hopping Controller/Codes/beam-hopping-manager.cc` | 新建：完整實作，包含離線排程計算與 NS3 事件注入 |
