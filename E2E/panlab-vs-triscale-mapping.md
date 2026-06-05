# PanLAB × TriScale-LEO 對應關係

> **目的：** 記錄 (1) PanLAB tools如何接入 TriScale-LEO 三層架構，(2) PanLAB 的真實 Starlink 環境在結構上與 TriScale-LEO 的 Iridium 模擬有何差異——以便設計實驗計畫，驗證該架構是否適用於 Starlink 等級的參數。

---

## 1. PanLAB 工具 × TriScale-LEO 

### 1.1 總覽表

| PanLAB 工具 | TriScale 層次 | 在 TriScale 的用途 | 整合複雜度 |
|---|---|---|---|
| `python-skyfield` | Layer 1（ISL Routing） | 替代或驗證 SNS3 的 SGP4 衛星軌跡模型；更精確地計算衛星位置與 ISL 距離 | 需要轉接層將輸出轉為 SNS3 相容格式 |
| `LEOViz`（satellite_matching_estimation.py） | Layer 1 | 驗證 SNS3 的衛星軌跡模型是否與真實 Starlink 過境觀測資料一致 | 需離線比較 |
| `starlink-lens` | Layer 3（QoS） | 提供真實 RTT／延遲 trace，可作為 NS-3 流量模型輸入或 E2E 延遲的基準對比 | 將 CSV trace 匯入 NS-3 流量模型即可 |
| `starlink_exporter` | Layer 3 | 真實 KPI 參考（吞吐量、PRB 等效值、UE 數量）— 作為模擬輸出指標的對比目標 | 需離線比較 |
| `GeoIP Map` | Layer 1 | 地面站（閘道器）拓撲：可使用真實 Starlink PoP 位置定義 Starlink 情境中的閘道器位置 | 從資料流中提取經緯度即可 |

---

### 1.2 `python-skyfield` vs SNS3 SGP4

**問題：** Skyfield 是否明顯比 SNS3 內建的 SGP4 更準確？

#### SNS3 使用的方法

SNS3 使用 `satellite-sgp4-mobility-model.cc`，這是一個由 TLE 驅動的直接 SGP4 傳播器，在每個模擬時間步呼叫 `GetPosition()`，與 `Simulator::Now()` 綁定。

#### Skyfield 的額外能力

| 功能 | SNS3 SGP4 | python-skyfield |
|---|---|---|
| 傳播器 | SGP4（WGS72） | 透過 `python-sgp4` 使用 SGP4（WGS72）— 核心演算法相同 |
| 地球模型 | WGS72 | WGS84（預設），可切換 |
| 極移 / IERS 修正 | 未套用 | 已套用`load.timescale(builtin=True)` |
| 光行差 / 大氣折射 | 未套用 | 可使用 |
| 地心觀測幾何 | 不支援 | 支援從觀測點計算方位角/仰角/距離 |
| TLE 曆元處理 | 直接處理 | 相同 |

**結論：** 兩者使用相同的底層 SGP4 演算法，差異在於：
- Skyfield 套用 IERS 修正（極移、ΔUT1）——對絕對位置精度影響約 10–100 公尺，在 1000 公里的 ISL 距離計算尺度上並不重要。
- Skyfield 支援地心觀測幾何（從地面觀測點計算方位角/仰角）——有助於驗證 SNS3 衛星位置相對於 UT 是否正確。

**觀察：** 軌道位置本身幾乎相同。Skyfield 在本專案的價值**不是**替換 SNS3 的傳播器，而是**驗證** SNS3 的位置輸出是否與 LEOViz 的真實 Starlink 軌跡資料吻合。以相同 TLE 快照同時執行兩者，比較 t = 0、30、60 秒時的 ISL 距離。若最大偏差 < 50 公里（< 1000 公里 ISL 的 5%），則 SNS3 SGP4 已足夠，Skyfield 僅作為驗證工具使用。

**建議的驗證實驗：**
1. 取得一份真實 Starlink TLE 快照（來自 LEOViz 資料）。
2. 用 Skyfield 執行：計算 t = 0、30、60、90 秒時的衛星位置。
3. 用 SNS3 SGP4 執行相同 TLE：提取位置 log。
4. 比較距離矩陣（每顆衛星的歐幾里得距離）。
5. 若最大偏差 < 50 公里 → 保留 SNS3 SGP4；否則 → 調查原因。

---

## 2. PanLAB（Starlink）vs TriScale-LEO（Iridium）結構差異

### 2.1 星座參數比較

| 參數 | Iridium（TriScale-LEO） | Starlink Shell 1（真實世界） | 對架構的影響 |
|---|---|---|---|
| 軌道高度 | 780 公里 | 550 公里 | 高度較低 → 傳播延遲更短、ISL 距離門檻更短 |
| 傾斜角 | 86.4°（近極軌道） | 53°（中傾角） | Starlink 極區覆蓋稀疏，赤道與極區的拓撲密度不同 |
| 衛星總數 | 66（6 軌道面 × 11） | ~3,200+（Shell 1：72 軌道面 × 22 顆） | 節點數約多 50 倍 → 路由表大小、Dijkstra 計算成本、BH 排程器的波束數量均大幅增加 |
| 軌道面數 | 6 | 72 | 跨軌道面 ISL 幾何結構根本不同——Starlink 為 +Grid 拓撲 vs Iridium 的近極網格 |
| ISL 類型 | 射頻（Ka 波段） | 雷射（光學） | 不同的鏈路容量模型；Starlink ISL 無需射頻干擾建模 |
| 每顆衛星 ISL 數 | 4（2 條同軌道面，2 條跨軌道面） | 4（結構相同，但跨軌道面 ISL 在極區停用） | Starlink 在高緯度停用跨軌道面 ISL——極區拓撲退化為一維鏈狀 |
| 地面閘道器 | ~12 | 全球數千個 PoP | Starlink 的路由進出點密度高出數個數量級 |

---

### 2.2 各層影響分析

#### Layer 1：ISL Routing

| 面向 | Iridium（現行） | Starlink（目標） | 變更內容 |
|---|---|---|---|
| 圖規模 | 66 個節點，132 條 ISL | ~3,200 個節點，~12,800 條 ISL | 離線 Dijkstra 前置計算成本增加——但仍為離線計算，仍可行 |
| ISL 距離門檻 | 5,000 公里（對應 Iridium 約 4,800 公里最大跨軌道面距離） | ~2,500–3,000 公里（550 公里高度下的 Starlink ISL 幾何） | `IslDistanceThreshold` 參數需重新校正 |
| 拓撲更新週期（τ） | 60 秒 | 15–30 秒較合理（低軌高度的拓撲變化更快） | 可能需要將 τ 縮短一半以維持路由準確性 |
| 跨軌道面 ISL 可用性 | 始終可用 | 極區停用（高緯度缺口） | 拓撲不再是穩定的 +Grid——需依緯度加入條件式 ISL 剪枝邏輯 |
| 代價函數 | 負載感知 Dijkstra | 同一方法可用 | 無結構性變更——僅需重新校正參數 |
| FtVisibilityFilter | 基於衛星仰角 | 原理相同，仰角分佈不同 | 可用——門檻值可能需要調整 |

**Layer 1 結論：** 架構適用。參數需重新校正。新增一個案例：極區 ISL 剪枝。

---

#### Layer 2：Beam Hopping

| 面向 | Iridium（現行） | Starlink（目標） | 變更內容 |
|---|---|---|---|
| 每衛星波束數 | 小型固定集合（SNS3 預設） | 每顆衛星 8 個主動波束（相位陣列，可電子轉向） | BH 排程器的 K（同時主動波束數）參數變更 |
| 波束轉向 | 固定波束位置 | 電子可轉向至任意地面點 | BHTP 時隙結構仍可用，但波束分配邏輯從固定 ID 改為經緯度目標 |
| 超幀結構 | DVB-S2X（T_s = 26.5ms，T_p = 503ms） | Starlink 使用專有幀結構——非 DVB-S2X | BHTP 時間模型建立在 DVB-S2X 上，這是最大的結構性假設——需要明確處理 |
| 需求估計（EM） | 每波束 EMA | 同一原理可用 | 無結構性變更 |
| 干擾模型 | 同頻波束隔離 | 雷射 ISL 無干擾；地面波束對波束干擾仍適用 | ISL 干擾模型可簡化 |

**Layer 2 結論：** 架構部分適用。DVB-S2X 超幀假設為 Iridium／GEO 特定假設。針對 Starlink，時隙模型需從 DVB-S2X 中抽象出來。排程邏輯（K 個主動波束、EM 需求估計）可移植。

---

#### Layer 3：QoS Scheduling

| 面向 | Iridium（現行） | Starlink（目標） | 變更內容 |
|---|---|---|---|
| MAC 排程器 | SNS3 SatBeamScheduler（CRA／RBDC／VBDC） | Starlink MAC 為專有協議——非 DVB-RCS2 | 與 Layer 2 相同問題：排程器建立在 DVB-RCS2 基元上 |
| QoS 類別 | 4 種 SNS3 流量類別 | Starlink 有商務／住宅／海事方案——不同的 QoS 模型 | QoS 類別對應需重新定義 |
| WFQ 權重 | 可配置 | 同一機制可用 | 可移植 |
| 佇列模型 | SNS3 中的每流佇列 | 概念相同 | 可移植 |

**Layer 3 結論：** 架構在概念上適用（WFQ、優先順序佇列、每流排程）。SNS3 使用的 DVB-RCS2 MAC 基元並非真實 Starlink 基元——這是已知的模擬抽象，並非設計缺陷。

---

### 2.3 PanLAB 量測項目 vs TriScale-LEO 模擬項目

| 量測項目 | PanLAB 工具 | TriScale-LEO 等效項目 | 可比較性 |
|---|---|---|---|
| RTT（ping 至閘道器） | `starlink-lens` | NS-3 模擬中的 E2E 封包延遲 | 可——但 SNS3 延遲含排隊延遲；ping 量測的是傳播 + Starlink 內部處理時間 |
| 服務衛星識別 | `LEOViz` | `m_tables[slotIndex][satId]` 路由表 | 間接可——可檢查 SNS3 選擇的中繼衛星是否與 LEOViz 估計相同 |
| 遮蔽 / 交接 | `LEOViz` 遮蔽地圖 | ISL Routing 中的拓撲變更事件 | 部分可——LEOViz 量測波束遮蔽；SNS3 建模鏈路狀態 |
| 吞吐量 | `starlink_exporter` | SNS3 中的 `DRB.UEThpDL` 等效值 | 大致可——通道模型不同 |
| PoP／閘道器 | `GeoIP Map` | SNS3 拓撲中的 GW 節點位置 | 可——可從真實 PoP 座標植入 SNS3 的 GW 位置 |

---

## 3. 後續步驟

TriScale-LEO 是否適用於 Starlink 等級參數實作：

| 步驟 | 行動 | 預期產出 |
|---|---|---|
| S1 | 執行 Skyfield vs SGP4 驗證實驗 | 確認 SNS3 傳播器已足夠使用 |
| S2 | 校正 Layer 1 的 Starlink 參數：高度=550km，門檻=2500km，τ=30s | 新的 SNS3 情境配置檔 |
| S3 | 加入極區 ISL 剪枝條件（在 ±70° 緯度以上停用跨軌道面 ISL） | ISL 拓撲建構器的程式碼修改 |
| S4 | 將星座擴展至 Starlink Shell 1 子集（先從 1 個軌道面 × 22 顆開始，再擴展至完整 72×22） | 漸進式負載測試 |
| S5 | 將 Layer 2 的 DVB-S2X 超幀假設替換為抽象時隙模型 | 需要設計決策 |
| S6 | 在 Starlink 參數情境下執行 Routing + BH + QoS，將 E2E 延遲與 `starlink-lens` 真實 trace 對比 | 論文結果 |

---

