# Confirm interfaces required for the expansion module 
**工作日誌 2026-03-19**

## 目標
確認 SNS3 Iridium 星座場景可正確建立 66 顆衛星與 132 條 ISL。

---

## 完成事項

### 1. 修正 `tles.txt` header 錯誤

**現象**：執行測試腳本後，只建立了 6 顆衛星，而非預期的 66 顆。

**原因**：`tles.txt` 第一行為 `6 11`，`LoadTles()` 只讀第一個數字作為衛星總數，導致只建 6 顆衛星。

**修正**：
```bash
sed -i 's/^6 11$/66 11/' \
    contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/tles.txt
```

**驗證**：衛星數量回到 66 顆。

---

### 2. 修正 `isls.txt` 缺少 header

**現象**：衛星建立正確後，每顆衛星仍只有 1 個 device（`SatOrbiterNetDevice`），ISL device 全部缺失。

**原因**：`isls.txt` 第一行直接是 `0 1`，`LoadIsls()` 把它當 header 解析，讀到 `size=0`，132 條 ISL 全部未建立。

**修正**：
```bash
sed -i '1s/^/132\n/' \
    contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
```

**驗證**：
```
SAT 0 nDevices=5
SAT 1 nDevices=5
SAT 2 nDevices=5
```
每顆衛星 5 個 device = 1 個 `SatOrbiterNetDevice` + 4 個 `PointToPointIslNetDevice`，ISL 建立成功。

---

### 3. 確認 SNS3 ISL Routing 架構與設計假設不符

**現象**：閱讀原始碼後，發現三個設計假設與實際 SNS3 接口不一致。

**原因與修正方向**：

| 原設計假設 | 實際 SNS3 接口 | 觸發原因 |
|-----------|--------------|---------|
| `FqCoDelQueueDisc` on ISL | `DropTailQueue<Packet>` 直接掛在 `PointToPointIslNetDevice` | SNS3 不走 TC 層，直接用 device queue |
| `Ipv4StaticRouting::AddHostRouteTo` | `SatOrbiterNetDevice::SetArbiter()` | SNS3 的 ISL routing 繞過 IP 層，用 Arbiter 機制查表 |
| islInterfaceIdx = Ipv4 interface index | `GetIslsNetDevices()` vector index | Arbiter 查的是 ISL device vector，非 IP interface |

→ 詳細決策理由見 `decisions/DEC-001-interface-alignment.md`

---

### 4. 建立測試腳本

建立 `scratch/test-iridium.cc`，最小可行配置驗證環境：

```cpp
simulationHelper->LoadScenario("constellation-iridium-66-sats-fixed");
simulationHelper->SetSimulationTime(Seconds(10));
std::set<uint32_t> beamSet = {1};
simulationHelper->SetBeamSet(beamSet);
simulationHelper->SetUserCountPerUt(1);
simulationHelper->CreateSatScenario();
```

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|---------|
| `positions/tles.txt` | header `6 11` → `66 11` |
| `positions/isls.txt` | 補上 header `132` |
| `scratch/test-iridium.cc` | 新增驗證腳本 |

---

## 明日計畫

- 確認 `SatSGP4MobilityModel` 的任意時間點位置查詢接口
- 實作 `BuildISLGraph(τ_k)` 骨架，驗證 SGP4 位置查詢正確
- 設計 `islInterfaceIndex` 與 `GetIslsNetDevices()` vector index 的對應方式
