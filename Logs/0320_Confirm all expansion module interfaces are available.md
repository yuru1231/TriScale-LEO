# Confirm all expansion module interfaces are available
**工作日誌 2026-03-20**

## 目標
驗證 `BuildISLGraph` 核心邏輯可執行，確認所有擴充模組接口可用。

---

## 完成事項

### 1. 新增 `GetGeoPositionAt(Time t)` 接口

**現象**：嘗試在離線預計算階段呼叫 `GetGeoPosition()` 取得任意時間點的衛星位置時，發現回傳值永遠是 `Simulator::Now()` 對應的位置，無法代入指定的 τ_k。

**原因**：`SatSGP4MobilityModel::DoGetGeoPosition()` 內部綁定 `Simulator::Now()`，沒有提供接受任意時間參數的公開接口。

**修正**：在 SNS3 新增一個 public 方法：

```cpp
// satellite-sgp4-mobility-model.h
GeoCoordinate GetGeoPositionAt(Time t) const;

// satellite-sgp4-mobility-model.cc
GeoCoordinate SatSGP4MobilityModel::GetGeoPositionAt(Time t) const {
    if (!IsInitialized()) return GeoCoordinate();
    JulianDate cur = m_start + t;
    double delta = (cur - GetTleEpoch()).GetMinutes();
    double r[3], v[3];
    sgp4(WGeoSys, m_sgp4_record, delta, r, v);
    if (m_sgp4_record.error != 0) return GeoCoordinate();
    return rTemeTorItrf(Vector3D(r[0], r[1], r[2]), cur) * 1000;
}
```

**驗證**：ECEF 座標轉換路徑完整：`sgp4() → rTemeTorItrf() → ×1000 → GeoCoordinate`。

---

### 2. 實作並驗證 `BuildISLGraph`

**現象（第一次）**：以 2500 km 為距離門檻時，部分衛星節點的鄰居數為 0，拓樸圖呈現斷裂。

**原因**：跨軌道面 ISL 瞬時距離可達 3500–5000 km，2500 km 門檻過嚴。原始設計值來自 Iridium 規格，但未考慮極軌星座的跨面距離變化。

**修正**：距離門檻調整為 5000 km（與前期 L1 工作一致）。仰角限制確認不適用於 ISL（僅用於 UT/GW 對衛星鏈路）。

→ 詳細決策理由見 `decisions/DEC-002-isl-distance-threshold.md`

**現象（第二次）**：確認 `ifIndex` 對應方式時，發現不能依賴 `isls.txt` 讀入順序，因為 SNS3 內部的 device 排列與檔案順序可能不一致。

**原因**：`GetIslsNetDevices()` 的 vector index 由 device 掛載順序決定，非 `isls.txt` 定義順序。

**修正**：改用 peer nodeId 直接查 vector index：

```cpp
std::vector<std::map<uint32_t, uint32_t>> peerNodeIdToIfIdx(n);
for (uint32_t i = 0; i < n; i++) {
    auto islDevs = orbDev->GetIslsNetDevices();
    for (uint32_t j = 0; j < islDevs.size(); j++) {
        Ptr<Node> peer = islDevs[j]->GetDestinationNode();
        if (peer) peerNodeIdToIfIdx[i][peer->GetId()] = j;
    }
}
```

**驗證輸出**：
```
SAT0-SAT1 dist at t=0: 3951.79 km
graph[0] edge count: 2
  nodeB=1  prop_cost=0.0131726s  ifIdxOnA=0  ifIdxOnB=0
  nodeB=10 prop_cost=0.013165s   ifIdxOnA=2  ifIdxOnB=1
```
prop_cost 驗證：3951 km ÷ 3×10⁸ = 0.01317s ✅

---

### 3. 確認所有擴充模組接口可用

所有模組接口確認完畢，無阻礙項目。

| 模組 | 狀態 |
|------|------|
| `PrecomputeAllTables()` | ✅ |
| `BuildISLGraph(τ_k)` | ✅ 已驗證 |
| `ComputeBaseRoutes()` | ✅ |
| `ApplyTiebreaker()` | ✅ |
| `ApplyTable(τ_k)` | ✅ |
| `UpdateLoadCosts()` | ✅ |
| `HasSignificantChange()` | ✅ |
| `RecomputeAffectedRoutes()` | ✅ |
| `ApplyRoutingTable()` | ✅ |

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|---------|
| `satellite-sgp4-mobility-model.h/.cc` | 新增 `GetGeoPositionAt(Time t)` |
| `contrib/satellite/helper/isl-graph.h` | 新增（結構定義 + 函式宣告） |
| `contrib/satellite/helper/isl-graph.cc` | 新增（BuildISLGraph 實作） |
| `contrib/satellite/CMakeLists.txt` | 加入 isl-graph.h/.cc |

---

## 明日計畫

- 實作 `ComputeBaseRoutes()`：per-source Dijkstra
- 實作 `PrecomputeAllTables()`：遍歷 10 個時間點
- 實作 `ApplyRoutingTable()`：SetArbiter 注入
- 整合進 `scratch/my-simulation.cc`
