# NTN 模擬器專題報告


## 目錄

- [第 1 章：緒論](#第-1-章緒論)
- [第 2 章：相關研究與模擬需求分析](#第-2-章相關研究與模擬需求分析)
- [第 3 章：SNS3 模擬器分析](#第-3-章sns3-模擬器分析)
  - [3.11 Slot 容量與 Beam 覆蓋分析](#311-slot-容量與-beam-覆蓋分析)
  - [3.12 小結](#312-小結)
- [第 4 章：Hypatia 模擬器分析](#第-4-章hypatia-模擬器分析)
- [第 5 章：跨模擬器比較](#第-5-章跨模擬器比較)
- [第 6 章：討論](#第-6-章討論)
- [第 7 章：結論與未來工作](#第-7-章結論與未來工作)

---

## 第 1 章：緒論

### 1.1 研究背景

#### 1.1.1 NTN（Non-Terrestrial Networks）介紹

3GPP 將 Non-Terrestrial Networks（NTN）定義為使用非地面平台承載傳輸中繼節點或基地台功能的網路或網路片段，其平台可包含不同軌道高度的衛星系統，以及高空平台（HAPS）。依軌道型態區分，LEO（Low Earth Orbit）通常位於約 500 至 2,000 km 高度，具較低延遲與較佳鏈路預算，但必須依賴較多衛星維持連續覆蓋；MEO（Medium Earth Orbit）大致位於 8,000 至 20,000 km；GEO（Geostationary Earth Orbit）則位於赤道上空約 35,786 km，可提供穩定覆蓋但延遲較高。對本研究而言，這些分類不僅是軌道高度差異，更直接決定了傳播延遲、可見時間、切換頻率與拓樸變動速度，因此也決定模擬器所需支援的動態能力。

在 3GPP 的 NTN 架構中，另一個重要區分是 transparent payload 與 regenerative payload。前者又稱 bent-pipe 或 transparent mode，衛星主要扮演類比中繼角色，將上行訊號經過頻率轉換、濾波與放大後再下行發送，基站主要功能仍位於地面；後者則具備部分或完整的 onboard processing 能力，能在星上執行解調、解碼、交換或路由等功能。這個區分與本研究的關聯在於：一旦系統朝 regenerative payload 發展，便更容易延伸到星間鏈路（ISL）、星上轉送與更複雜的控制平面，因此 routing、資源調度與端對端服務連續性的分析也會變得更重要。

從標準化脈絡來看，3GPP 在 Rel-17 首次將 NTN 以具體的規範型態納入 5G 系統，形成後續研究與實作的重要基線。Rel-17 的 NR NTN 工作建立了一組相對保守但明確的假設，例如 transparent payload、FR1 FDD 操作、GNSS-capable UE，以及 earth-fixed tracking area 等，目標是先讓衛星接取具備可落地的基本架構。進入 Rel-18 之後，3GPP 將焦點由「可接入」推進至「可優化」，加入 coverage enhancement、network-verified UE location、NTN-TN 與 NTN-NTN mobility / service continuity enhancement，以及更完整的頻段與效能增強議題。Rel-19 與 Rel-20 目前仍屬 open release，本研究不將其視為既定要求，而是作為技術演進方向加以觀察；其中 Rel-19 已明確往 regenerative payload、store-and-forward、UE-satellite-UE communication、GNSS-independent operation，以及更完整的架構、管理與安全議題延伸，Rel-20 則可視為 5G-Advanced 持續深化衛星整合能力的開放性方向。

因此，本研究在第 1 章引用 3GPP 並非為了宣稱本專案直接實作某一版標準，而是將 3GPP 作為需求背景與技術參照系：一方面說明 NTN 尤其是 LEO NTN 已成為 B5G/6G 網路演進的重要組成，另一方面指出在動態拓樸、資源調度與 QoS 行為的研究驗證上，現有單一模擬平台仍難以完整支撐標準脈絡下的問題分析。這也正是本專題從模擬器角度切入的出發點。

**圖 1-1：NTN 系統架構圖**

![](figures/fig01-ntn-system-architecture.drawio.png)

#### 1.1.2 LEO 衛星系統發展

在各類 NTN 平台中，LEO 衛星系統近年特別受到重視，原因在於其兼具較低延遲、較佳鏈路條件與大規模星座部署的可行性。相較於 GEO 系統，LEO 衛星因軌道高度較低，單程傳播延遲顯著下降，較適合承載需要互動性與服務連續性的資料服務；然而，這項優勢的代價是衛星覆蓋範圍較小、可見時間較短，必須以大量衛星協同運作才能形成接近連續的全球覆蓋。

近年的代表性系統顯示，LEO 網路已從概念驗證逐漸邁向大規模部署與系統化營運。例如 Starlink 強調大規模星座與全球寬頻覆蓋能力，OneWeb 聚焦企業與區域連接場景，而 Iridium 則是較早建立成熟低軌通訊網路並具備衛星間連線特性的代表案例。雖然這些系統的商業模式與部署規模並不相同，但它們共同反映出一個趨勢：未來衛星網路研究不能只停留在靜態覆蓋分析，而必須面對由高速移動星座所引發的拓樸時變、鏈路切換、路徑重組與資源重分配問題。

對本專題而言，LEO 系統的關鍵意義不只在於「低延遲」，更在於其帶來的高動態性。衛星位置會隨時間持續變動，gateway 與 user terminal 的可見衛星集合也會改變；若再進一步考慮 ISL，則端到端路徑將隨時受到衛星幾何、鏈路可用性與負載狀態影響。這意味著 routing 不再是一次性離線設定，而必須具備與時間同步的更新能力；同時，beam 資源與封包排程也必須能配合衛星覆蓋與服務時間視窗調整。換言之，LEO 的系統特性正是本研究同時關注 ISL Routing、Beam Hopping 與 QoS Scheduling 三個主題的直接原因。

**圖 1-2：LEO 拓樸隨時間變化圖**

![](figures/fig02-leo-topology-time-variation.drawio.png)

---

### 1.2 問題定義

#### 1.2.1 動態拓樸（Dynamic Topology）

3GPP 在 NTN 討論中多次指出，非地面網路與傳統地面網路最大的差異之一，在於衛星與波束覆蓋本身具有可預測但持續變動的移動性。對於 GEO 而言，此問題相對有限；但在 LEO 與 MEO 場景中，衛星相對地面高速移動，moving cells、satellite motion，以及 NTN-TN / NTN-NTN mobility 便成為系統設計的核心議題。以 LEO 為例，衛星軌道速度約可達 7.5 km/s，若將時間尺度拉到數十秒甚至分鐘級，衛星間相對距離、可見地面節點集合以及服務中的覆蓋波束都可能發生顯著變化。

這種拓樸時變性對網路層的直接衝擊是：鏈路圖不是固定不變的。若系統包含 ISL，則某一時刻可用的衛星間邊集合可能在下一個時間槽便改變；即使不考慮實體斷鏈，最短路徑與最佳轉送路徑也會因幾何位置改變而重組。換言之，傳統地面網路中依賴相對穩定拓樸、較慢控制平面更新週期、或僅在故障時重新計算路由的假設，並不適合直接套用到 LEO NTN。

因此，本研究將動態拓樸視為第一個核心問題，並將其轉化為模擬器需求：系統必須能在 slot / epoch 層級追蹤衛星位置、重建當前有效拓樸，並據此更新路由決策。這也是本專題 Layer 1 聚焦於 topology-aware path computation 與 runtime routing update 的原因。

#### 1.2.2 高延遲與長距離傳輸

除了拓樸會隨時間變化之外，NTN 的另一項根本特性是傳播距離遠、延遲顯著，且延遲會與衛星幾何位置一同變動。3GPP 在 NTN 研究中即指出，較大的 propagation delay 會連帶影響 timing advance、HARQ、RACH、功率控制與服務連續性等設計。對 LEO 系統而言，雖然其延遲遠低於 GEO，但由於衛星與地面之間距離仍遠大於一般地面蜂巢網路，因此單程傳播延遲通常仍落在數毫秒到十餘毫秒量級；若進一步考慮 gateway 到 gateway 或 gateway 到 user terminal 的多跳 ISL 路徑，端到端延遲便不再只是單一空間鏈路的問題，而是多段 service link、feeder link 與 ISL cost 的總和。

這使得 LEO NTN 的效能分析不能簡化為「比 GEO 更低延遲」即可。實際系統中，某些路徑雖然在衛星幾何上仍可達，但可能因多跳轉送、gateway 繞送、隊列堆積或資源競爭而造成明顯的端到端延遲差異。對研究者而言，若只從抽象圖論角度比較最短路徑，往往無法回答封包層級上實際會觀察到的 latency、throughput、drop rate 或 queueing behavior。

因此，本研究將高延遲與長距離傳輸問題進一步連結到 QoS 與可觀測性需求：在模擬器中，不僅要能估計 propagation delay，還必須能觀測 packet-level 的資料面行為，才能分析路由與排程對端到端服務品質的實際影響。這也是本研究後續同時重視 Layer 1 路由成本設計與 Layer 3 QoS 驗證能力的原因。

---

### 1.3 三大研究主題

本研究以模擬器為核心工具，針對 LEO 衛星網路中三個相互關聯的挑戰進行分析：

| 主題 | 核心問題 | 本研究對應 |
|------|---------|-----------|
| **ISL Routing** | 動態拓樸下如何有效計算與更新路由 | `IslRoutingManager` + pre-slot Dijkstra（Layer 1） |
| **Beam Hopping** | 有限資源下如何動態調度多 beam 覆蓋 | `SatBhScheduler` + BHTP（Layer 2） |
| **QoS Scheduling** | beam 服務時間內如何對 UE 做差異化排程 | SNS3 `SatBeamScheduler` 擴充（Layer 3） |

上述三個主題並非彼此獨立，而是對應到 3GPP NTN 背景下不同層級的需求映射。首先，ISL Routing 對應的是高移動性星座中的 mobility、service continuity，以及逐步朝 regenerative payload 與 ISL 架構延伸的網路演進方向；在本研究中，它具體落實為動態拓樸下的 route computation、per-slot table precomputation 與 runtime update。其次，Beam Hopping 對應的是 NTN 在有限覆蓋與能量條件下的資源效率與 coverage management 需求；在本研究中，它被具體化為 time-slot based beam resource allocation 問題。最後，QoS Scheduling 對應的是不同服務需求下對 latency、throughput、continuity 與 fairness 的追求；在本研究中，這一主題被轉化為 beam service window 內的 priority / WFQ 式排程與 packet-level KPI 觀測。

因此，本專題的目標不是重新制定或取代 3GPP 規格，而是在模擬器層面建立一套可驗證這些需求映射的研究工具鏈。換言之，3GPP 提供的是問題背景與能力邊界，而本研究試圖回答的是：現有模擬平台能否支撐這些需求的驗證？若不能，又應如何擴充才能使 routing、beam 與 QoS 行為在 LEO NTN 場景中被系統化分析。

---

### 1.4 研究動機

#### 1.4.1 為何需要模擬器

若從 3GPP 的角度觀察，標準文件已對 NTN 的需求、架構與演進方向提供了明確脈絡；然而，標準本身並不等於研究驗證工具。它能說明系統應支援哪些場景、應考量哪些限制，卻無法直接提供研究者 packet-level、routing-level 或 beam-level 的可操作驗證環境。對於本研究這類同時涉及動態拓樸、星間路由、資源切換與 QoS 行為的問題而言，若僅閱讀標準條文，仍不足以回答特定設計在實際流量、特定星座與特定時間尺度下會如何表現。

另一方面，實體衛星系統的驗證成本極高。真實衛星、地面閘道站與終端設備的建置與操作涉及昂貴設備、長週期部署、受限的可控實驗條件，以及難以重現的外部環境因素。即使研究者能取得商用系統資料，也通常難以任意控制拓樸、切換頻率、流量模式或內部排程策略。這使得模擬器在 NTN 研究中具有不可替代性：它能提供可控制、可重現、可量測的實驗環境，使研究者得以系統性比較不同設計方案。

本研究之所以同時關注 SNS3 與 Hypatia，正是因為不同模擬器位於不同抽象層級。Hypatia 偏重拓樸與路由層級的分析，適合大規模星座與時間演進的 network graph 研究；SNS3 則建立於 NS-3 的 packet-level 模型之上，更適合觀察資料面行為、排程影響與端到端效能。也因此，本研究不將模擬器視為單純的實驗替代品，而是視為連接 3GPP 需求背景與工程驗證之間的關鍵橋梁。

#### 1.4.2 模擬器在研究中的角色

在本研究脈絡下，模擬器至少扮演四種角色。第一，它是演算法與系統設計的驗證平台，用來檢查特定 routing、beam 或 scheduling 設計是否能在 3GPP 所勾勒的 NTN 問題背景下運作。第二，它是量化效能的工具，研究者可透過模擬器觀測 latency、throughput、drop rate、queue occupancy 與 route switching 等 KPI，而不必只停留在理論最短路徑或靜態容量估算。第三，它是比較不同設計方案與不同模擬平台的基準環境，使研究者能分析 topology-level、packet-level、MAC/PHY-level 抽象之間的差異與取捨。第四，它是學術研究可重現性的基礎，因為可重現的模型、輸入參數與實驗流程，是論文結果能被驗證與延續的前提。

對本專題而言，模擬器還有一層更具體的角色：它不只是用來「跑結果」，而是用來找出標準需求與研究驗證之間的落差。例如 3GPP 可以指出 NTN 需要 service continuity、mobility enhancement 與 resource management，但若模擬器無法支援動態拓樸、beam time-slot 切換或封包層級 QoS 觀測，研究者就很難將這些需求落成可檢驗的工程問題。因此，本研究後續將先從相關研究與模擬需求出發，整理 Beam Hopping、Routing 與 Scheduling 對模擬器功能的要求，再進一步分析 SNS3 與 Hypatia 分別能支援哪些能力、又有哪些能力仍需擴充。這也是第 2 章與第 3 章之所以緊密相連的原因。

---

## 第 2 章：相關研究與模擬需求分析

### 2.1 Beam Hopping

#### 2.1.1 論文分類與概述

> **[TODO]** 依以下分類整理 Beam Hopping 相關論文（共 9–15 篇）：
> - Survey（2–3 篇）：整體 BH 技術綜述
> - Method（5–8 篇）：具體 BH 排程演算法
> - System（2–4 篇）：系統實作或模擬框架

#### 2.1.2 代表論文分析（挑 3 篇深入分析）

> **[TODO]** 對每篇說明：
> 1. 問題定義
> 2. 提出方法
> 3. 模擬方式（使用何種模擬器或工具）
> 4. 與本研究的關聯

#### 2.1.3 論文比較

**表 2-1：Beam Hopping 論文整理表**

| 編號 | 論文標題 | 方法 | 模擬器 / 工具 | 需求功能 |
|------|---------|------|--------------|---------|
| BH-01 | [TODO] | | | |
| BH-02 | [TODO] | | | |
| BH-03 | [TODO] | | | |

#### 2.1.4 模擬需求分析

從上述論文中歸納 Beam Hopping 研究對模擬器的需求：

| 需求功能 | 說明 | 重要性 |
|---------|------|--------|
| Beam model | 可變 beam 覆蓋範圍與功率 | 高 |
| Time-slot simulation | super-frame 級時間切換 | 高 |
| Interference model | 同時活動 beam 的干擾計算 | 中 |
| Power allocation | 動態功率分配支援 | 中 |

> **建議圖表**：
> - 圖 2-1：Beam hopping 時間切換圖（BHTP slot 分配示意）
> - 圖 2-2：Beam footprint 示意圖（衛星覆蓋地面 beam 分佈）

---

### 2.2 Routing

#### 2.2.1 論文分類與概述

> **[TODO]** 依以下分類整理 ISL Routing 相關論文（共 9–15 篇）：
> - Survey（2–3 篇）
> - Method（5–8 篇）：Dijkstra、Dynamic Routing、Load-aware
> - System（2–4 篇）

#### 2.2.2 代表論文分析（挑 3 篇深入分析）

> **[TODO]**

#### 2.2.3 論文比較

**表 2-2：Routing 論文整理表**

| 編號 | 論文標題 | 方法 | 模擬器 / 工具 | 需求功能 |
|------|---------|------|--------------|---------|
| RT-01 | [TODO] | | | |
| RT-02 | [TODO] | | | |
| RT-03 | [TODO] | | | |

#### 2.2.4 模擬需求分析

| 需求功能 | 說明 | 重要性 |
|---------|------|--------|
| Dynamic topology | 每數十秒更新 ISL 圖 | 高 |
| ISL modeling | ISL 建立 / 斷開條件（距離 / 仰角） | 高 |
| Routing API | 可寫入 per-sat forwarding table | 高 |
| Traffic model | 多種流量類型驅動 routing 壓力 | 中 |

> **建議圖表**：
> - 圖 2-3：LEO network graph（6×11 mesh 拓樸）
> - 圖 2-4：Routing path 變化圖（跨 slot 路由切換示意）

**圖 2-3：LEO Network Graph（6×11 mesh 拓樸）**

![](figures/fig05-leo-network-graph.drawio.png)

**圖 2-4：Routing Path 變化圖（跨 slot 路由切換示意）**

![](figures/fig06-routing-path-change.drawio.png)

---

### 2.3 Scheduling

#### 2.3.1 論文分類與概述

> **[TODO]** 依以下分類整理 QoS Scheduling 相關論文（共 9–15 篇）：
> - Survey（2–3 篇）
> - Method（5–8 篇）：WFQ、DRR、priority queuing
> - System（2–4 篇）

#### 2.3.2 代表論文分析（挑 3 篇深入分析）

> **[TODO]**

#### 2.3.3 論文比較

**表 2-3：Scheduling 論文整理表**

| 編號 | 論文標題 | 方法 | 模擬器 / 工具 | 需求功能 |
|------|---------|------|--------------|---------|
| SC-01 | [TODO] | | | |
| SC-02 | [TODO] | | | |
| SC-03 | [TODO] | | | |

#### 2.3.4 模擬需求分析

| 需求功能 | 說明 | 重要性 |
|---------|------|--------|
| MAC scheduler | 可攔截 / 替換 SNS3 原生排程邏輯 | 高 |
| Queue model | 多佇列 per UE 建模 | 高 |
| QoS metrics | Throughput / delay / jitter / fairness 量測 | 高 |

> **建議圖表**：
> - 圖 2-5：Resource block 分配圖
> - 圖 2-6：Queue 模型圖（multi-queue per UE）

---

### 2.4 模擬需求總整理

#### 2.4.1 功能彙整

**表 2-4：跨研究主題模擬功能需求對照表**

| 功能 | ISL Routing | Beam Hopping | Scheduling |
|------|:-----------:|:------------:|:----------:|
| LEO mobility（SGP4） | ✅ | ✅ | — |
| 動態 ISL 拓樸 | ✅ | ✅ | — |
| Per-sat routing API | ✅ | — | — |
| Beam model | — | ✅ | ✅ |
| Time-slot simulation | ✅ | ✅ | — |
| MAC scheduler API | — | — | ✅ |
| Queue model | ✅（load-aware） | — | ✅ |
| Packet-level metrics | ✅ | ✅ | ✅ |

#### 2.4.2 關鍵觀察

> **[TODO]** 說明哪些功能在三個研究主題中都高度重視（如 packet-level simulation、動態拓樸），以及哪些是特定主題的關鍵能力。

---

## 第 3 章：SNS3 模擬器分析

### 3.1 Overview

本章整理本研究在 SNS3 上完成的主要工程工作，聚焦於 LEO 星座的 ISL 路由擴充、端對端路徑分析，以及執行期可觀測性的建立。相較於原生 SNS3 主要依賴既有的 Global Routing 與 IP 層轉送，本研究導入以 `IslRoutingManager` 與 `SatIslArbiterUnicast` 為核心的控制平面，使衛星節點可依 time slot 套用預先計算之 ISL next-hop table，並在必要時根據佇列負載進行局部重算。

本章的重點包括：

- 使用 TLE 與 `isls.txt` 建立動態 LEO ISL 拓樸
- 以 propagation delay 與 EMA queue delay 建立路由成本
- 在 runtime 以 Arbiter 方式套用 per-slot 路由
- 以 `sat2sat`、`gw2gw`、`gw2ut` 三種模式驗證路由與資料面行為

---

### 3.2 System Architecture

本研究的系統可分為四個層次：離線輸入與拓樸基礎、路由控制平面、執行期資料平面與觀測機制、以及輸出與驗證文件。

**圖 3-1：SNS3 ISL 路由系統架構圖**

![](figures/fig03-sns3-isl-system-architecture.drawio.png)

上圖說明本研究的整體資料流與模組依賴關係：

- `TLE` 與 `isls.txt` 構成離線輸入，其中 TLE 提供衛星位置演化，`isls.txt` 提供候選 ISL 鄰接關係
- `SatSGP4MobilityModel` 於指定時間 `t` 取得衛星位置，供 `IslRoutingManager::BuildISLGraph()` 建立當前 slot 的有效圖
- `IslRoutingManager` 以 Dijkstra 針對所有 source 衛星預先計算路由，並將結果快取於 `m_tables[slot]`
- 執行期中，`SatIslArbiterUnicast` 依 slot 將 next-hop table 寫入各衛星節點，控制實際的 ISL 轉送
- `PointToPointIslNetDevice` 與其 `DropTailQueue<Packet>` 提供資料平面佇列與負載來源，trace 模組則輸出 drop、load、route report 與端對端觀測結果

---

### 3.3 Simulation Model

#### 3.3.1 事件驅動架構

SNS3 建立於 NS-3 的事件驅動模擬機制之上。所有路由更新、流量安裝、trace 掛接與 slot 邊界觸發事件，皆以 `Simulator::Schedule()` 安排於模擬時間軸中執行。這使得控制平面更新可與資料平面封包事件共存，且每次實驗具有可重現性。

#### 3.3.2 封包層級仿真

本研究保留 SNS3 的 packet-level 模型，因此 route path、queue occupancy、drop、delay、throughput 等結果皆可直接由封包事件累積得到，而非僅為離線 shortest-path 推估。這也是本研究能進一步驗證 ISL routing 與 E2E forwarding 的原因。

#### 3.3.3 移動性模型

LEO 星座的衛星位置由 `SatSGP4MobilityModel` 依 TLE 計算。對於每個 time slot，系統透過 `GetGeoPositionAt(t)` 取得 66 顆衛星的 ECEF 座標，作為當前 slot 建圖與路由計算的基礎。此介面與模擬時鐘解耦，確保離線預計算可重現。

---

### 3.4 Functional Modules

本研究以 `IslRoutingManager` 為核心路由控制平面，搭配 SNS3 原生的 channel、queue 與 mobility 基礎設施。相較於原生 SNS3，有三個關鍵差異：

1. 路由不再依賴 IP 層 static/global routing table，而是由 `SatIslArbiterUnicast` 直接決定 ISL next hop
2. 路由成本以 propagation delay 為 primary cost，加入 EMA queue delay 作為 secondary cost
3. Runtime 若偵測到顯著負載變化，僅對受影響 source 重跑 Dijkstra，而非全量重算

ISL 佇列位於 `PointToPointIslNetDevice` 的 device-level `DropTailQueue<Packet>`，由 queue bytes 推估 queue delay 後以 EMA 平滑。

模組 API 與 Attribute 完整規格見 [`TechRef.md`](../TechRef.md) Section 2。

---

### 3.5 程式目錄架構

**表 3-1：SNS3 相關程式目錄結構**

| 路徑 | 說明 |
|------|------|
| `contrib/satellite/helper/isl-graph.h/.cc` | `IslRoutingManager` 核心實作（路由計算、Arbiter 寫入） |
| `contrib/satellite/helper/ft-filter.h/.cc` | `FtVisibilityFilter`（Layer 2 FT 可見性過濾） |
| `scratch/test-iridium-e2e.cc` | 主測試程式（6 種 pathType，流量 / trace / verdict） |
| `contrib/satellite/model/satellite-sgp4-mobility-model.h/.cc` | 已修改：新增 `GetGeoPositionAt(Time t)` |
| `contrib/satellite/model/satellite-isl-arbiter-unicast.h` | 已修改：新增 `ClearNextHopEntries()` |
| `Topology & ISL Routing/Outputs/` | 各類驗證輸出與路由紀錄 |

---

### 3.6 主程式流程

**圖 3-2：test-iridium-e2e 與 IslRoutingManager 主程式流程圖**

![](figures/fig04-test-iridium-program-flow.drawio.png)

圖 3-2 對應 `test-iridium-e2e.cc` 的主要執行順序。程式以 `--pathType` 作為唯一入口，依序完成：SNS3 scenario 建立、`IslRoutingManager` 初始化與離線預計算、trace hooks 掛接、traffic 安裝，最後以 `ScheduleRoutingUpdates()` 在每個 slot 邊界套用路由表，並於模擬結束後輸出各層 PASS/FAIL verdict。

**圖 3-3：路由更新時序圖（以 Slot 為主軸）**

![](figures/fig07-routing-update-sequence-timing.drawio.png)

圖 3-3 以 slot 為 x 軸，呈現每個 slot 內的執行順序：Pre-Sim 期間完成全 slot 預計算（`ComputeAllSlots()`），每個 slot 邊界依序執行 `ApplyRoutingTable(k)`，slot 期間流量持續傳輸，slot 末端更新 `LoadCosts()`，若負載變化超過閾值 θ 則觸發局部重算 `RecomputeAffectedRoutes(k)`。底部標示三個 wall-clock timing metric：`dijkstraMs`、`applyWallMs`、`recomputeWallMs`。

完整的執行步驟、pathType 定義、ObsScope 邏輯與 function mapping 見 [`TechRef.md`](../TechRef.md) Section 3–5。

---

### 3.7 功能支援分析

**表 3-2：SNS3 功能支援對照表**

| 功能 | 支援情況 | 說明 |
|------|----------|------|
| LEO mobility | ✅ 完整支援 | 由 TLE 與 SGP4 驅動，`GetGeoPositionAt(t)` 解耦時鐘 |
| 動態 ISL 拓樸 | ✅ 完整支援 | 每個 slot 重新建圖，`BuildISLGraph()` |
| Propagation-delay routing | ✅ 完整支援 | `dist / c` 作為主要成本 |
| Queue-aware routing | ✅ 完整支援 | EMA queue delay 作為次要成本，`UpdateLoadCosts()` |
| Satellite-to-satellite routing | ✅ 完整支援 | `sat2sat` pathType 驗證 |
| Gateway-to-gateway E2E | ✅ 完整支援 | `gw2gw_e2e` pathType 驗證 |
| Gateway-to-UT E2E | ✅ 完整支援 | `gw2ut_e2e` pathType 驗證 |
| Packet-level observability | ✅ 完整支援 | drop、delay、throughput、OBS scope、endpoint probe |
| Beam Hopping | ⏳ 架構就緒 | Layer 1 介面已提供，Layer 2 實作中 |
| QoS Scheduling | ⏳ 架構就緒 | `ConfigureQoS()` 目前為 no-op，待實作 |

---

### 3.8 與論文需求對應

**表 3-3：SNS3 功能與研究需求對照**

| 需求 | 來自 §2 分析 | 對應實作 | 模擬器支援 | 是否需修改 |
|------|------------|---------|-----------|----------|
| 動態 LEO 路由 | 2.2.4 Routing | `IslRoutingManager::BuildISLGraph()` + per-slot tables | ✅ | 是（新增模組） |
| 最短路徑計算 | 2.2.4 Routing | Dijkstra per source，`ComputeBaseRoutes()` | ✅ | 是（新增） |
| 執行期更新 | 2.2.4 Routing | `ScheduleRoutingUpdates()` + `ApplyRoutingTable()` | ✅ | 是（新增） |
| 負載感知路由 | 2.2.4 Routing | `UpdateLoadCosts()` + EMA queue delay | ✅ | 是（新增） |
| E2E 路徑分析 | 2.2.4 Routing | `PrecomputeGwRoutes()` / `PrecomputeGwUtRoutes()` | ✅ | 是（新增） |
| Packet-level metrics | 全三主題 | route report、ISL drop trace、OBS summary | ✅ | 是（新增） |
| LEO mobility（SGP4） | 2.2.4, 2.1.4 | `SatSGP4MobilityModel::GetGeoPositionAt(t)` | ✅ | 是（修改 SNS3） |
| Beam model | 2.1.4 Beam Hopping | SNS3 原生 beam 模型 | ✅（原生） | 否（待 Layer 2 驗證） |

---

### 3.9 Limitations

#### 3.9.1 Feeder Link PHY 繞過

在 SNS3 的 `REGENERATION_NETWORK` 模式下，GW-to-GW application traffic 在衛星端被重新產生並經 ISL 骨幹轉送。`SatOrbiterNetDevice::RxFeeder` 與 GW-side `SatNetDevice::Rx` 不觸發，因此 `gw2gw_e2e` pathType 的 feeder PHY 在 packet trace 上的可見性有限。這屬於底層架構限制，而非 `IslRoutingManager` 的路由邏輯錯誤，對應 verdict 設計改為以 ROUTING_LAYER + ISL_LAYER + PACKET_LAYER 為主判定依據。

#### 3.9.2 路由計算規模

雖然 66 顆衛星的 Iridium-66 星座可輕易完成 per-slot Dijkstra 計算，但若未來擴展至數百或數千顆衛星的大型星座，預先計算與 runtime 局部重算的成本仍需進一步優化。

#### 3.9.3 全量覆寫路由表

目前 `ApplyRoutingTable()` 在 slot 邊界仍會重寫該 slot 的 Arbiter next-hop entries。雖然成本低於全量重建 IP routing table，但未來仍可考慮引入更細粒度的 diff-based update，進一步降低 slot boundary 開銷。

---

### 3.10 Extensibility

本研究的設計具備良好的擴充性：

- 可替換成本函數，例如引入 link reliability、predicted congestion 或 QoS class
- 可擴充不同 pathType 與 observer 模組，不需修改 routing core
- 可與 Beam Hopping（Layer 2）或 QoS Scheduling（Layer 3）整合，Layer 1 介面已穩定提供 `GetGwRoute()`、`GetGwUtRoute()`、`FtVisibilityFilter` 等 API
- 可在既有 `SatOrbiterNetDevice` / `SatIslArbiterUnicast` 架構上繼續擴充，而不必改寫整個 SNS3 stack

---

### 3.11 Slot 容量與 Beam 覆蓋分析

**圖 3-4：Slot 容量與 Beam 覆蓋分析（T_slot = 60 s）**

![](figures/fig08-slot-capacity-beam-coverage.drawio.png)

圖 3-4 從三個角度量化一個 routing slot 的系統特性：

- **ISL 容量**（左欄）：10 Mbps × 60 s = 600 Mb/slot，可承載 50,000 FWD packets（1500 B）或約 83 concurrent flows（FWD rate 120 kbps/flow）
- **Beam 覆蓋偏移**（中欄）：Iridium 衛星在 60 s 內移動約 450 km，beam 直徑約 670 km，導致 footprint 中心偏移 67% beam 寬，兩端重疊面積僅約 21%。這說明 T_slot = 60 s 期間 beam **不在同一 cell**，Layer 2 beam-hop frame 必須遠小於 T_slot 才能維持 per-cell 覆蓋
- **時間尺度層次**（右欄）：封包 TX（~1.2 ms）→ Beam hop frame（~1–5 s）→ Routing slot（60 s）→ 軌道週期（~100 min），各層解耦、可獨立設計

---

### 3.12 小結

本章完成了本研究在 SNS3 上的工程主體描述。核心成果是：在 SNS3 上建立一套可處理 LEO 動態拓樸的 ISL routing framework，能依 slot 預先計算路由、在執行期套用 Arbiter next-hop entries、並在負載變化時進行局部重算。透過 `sat2sat`、`gw2gw` 與 `gw2ut` 的驗證，此架構不僅能正確輸出理論路徑，也能在 packet-level 模擬環境中觀測其資料面效果，6 種 pathType 全數通過 PASS 驗證。

---

## 第 4 章：Hypatia 模擬器分析

> **[TODO]** 本章結構與第 3 章完全相同（4.1–4.11），依序完成 Hypatia 的分析。

### 4.1 Overview

> **[TODO]** 簡介 Hypatia 模擬器的設計背景、與 SNS3 的定位差異。

---

### 4.2 System Architecture

> **[TODO]**
>
> **建議圖表**：圖 4-1：Hypatia 架構圖
>
> **建議圖表**：圖 4-2：主程式流程圖

**圖 4-1：Hypatia 架構圖**

![](figures/fig11-hypatia-architecture.drawio.png)

**圖 4-2：Hypatia 主程式流程圖**

![](figures/fig12-hypatia-main-pipeline.drawio.png)

---

### 4.3 Simulation Model

> **[TODO]** 說明 Hypatia 的事件模型、封包模型與移動性模型。

---

### 4.4 Functional Modules

> **[TODO]**

---

### 4.5 程式目錄架構

**表 4-1：Hypatia 程式目錄結構**

| 路徑 | 說明 |
|------|------|
| [TODO] | |

---

### 4.6 主程式流程

> **[TODO]**

---

### 4.7 功能支援分析

**表 4-2：Hypatia 功能支援對照表**

| 功能 | 支援情況 | 說明 |
|------|----------|------|
| [TODO] | | |

---

### 4.8 與論文需求對應

**表 4-3：Hypatia 功能與研究需求對照**

| 需求 | 來自 §2 分析 | 對應實作 | 模擬器支援 | 是否需修改 |
|------|------------|---------|-----------|----------|
| [TODO] | | | | |

---

### 4.9 Limitations

> **[TODO]**

---

### 4.10 Extensibility

> **[TODO]**

---

### 4.11 小結

> **[TODO]**

---

## 第 5 章：跨模擬器比較

### 5.1 功能比較

**表 5-1：SNS3 vs. Hypatia 功能對照**

| 功能 | SNS3 | Hypatia | 說明 |
|------|:----:|:-------:|------|
| LEO mobility | ✅ | ✅ | |
| 動態 ISL 拓樸 | ✅ | ✅ | |
| Packet-level simulation | ✅ | [TODO] | |
| Beam model | ✅ | [TODO] | |
| QoS / MAC scheduler | ✅ | [TODO] | |
| Routing API 可擴充性 | ✅（Arbiter） | [TODO] | |
| 易用性 / 學習曲線 | [TODO] | [TODO] | |

---

### 5.2 架構層級比較

> **[TODO]** 說明兩個模擬器在架構層級的差異：
>
> - Hypatia 層級：Topology layer（衛星軌道、ISL 圖）
> - SNS3 層級：Network layer（MAC、PHY、Queue、Application）
>
> **建議圖表**：圖 5-1：模擬器架構層級比較圖

**圖 5-1：模擬器架構層級比較圖**

![](figures/fig15-simulator-layer-comparison.drawio.png)

---

### 5.3 與論文需求對應

**表 5-2：跨模擬器與論文需求對照**

| 需求 | SNS3 | Hypatia | 建議選擇 |
|------|:----:|:-------:|---------|
| ISL Routing 驗證 | ✅ | ✅ | |
| Beam Hopping 驗證 | ✅ | [TODO] | |
| QoS Scheduling | ✅ | [TODO] | |
| 大規模星座擴展性 | 中 | [TODO] | |

---

### 5.4 討論

> **[TODO]** 說明：
> - 哪個模擬器適合哪種研究場景
> - 兩者的互補性（SNS3 做精細 packet-level，Hypatia 做大規模 topology）
> - 本研究選擇 SNS3 的理由（Beam Hopping + QoS 需要 MAC/PHY 層支援）

---

## 第 6 章：討論

### 6.1 模擬器選擇策略

> **[TODO]** 根據第 2 章的模擬需求分析與第 5 章的比較，給出研究者選擇模擬器的建議原則。

---

### 6.2 本研究的工程貢獻與侷限

> **[TODO]** 討論本研究在 SNS3 上的擴充對後續 Beam Hopping 與 QoS 研究的價值，以及 §3.9 各項限制對結論的影響程度。

---

### 6.3 與論文需求的符合程度

**表 6-1：論文需求符合度評估**

| 需求（來自 §2.4） | 是否達成 | 備註 |
|-----------------|---------|------|
| [TODO] | | |

---

### 6.4 未解決問題與後續方向

> **[TODO]** 列出：
> - load-driven 路由切換驗證（需 ISL 飽和場景）
> - Beam Hopping Phase 3/4 實作
> - QoS attribute 路徑驗證
> - 大規模星座擴展測試

---

## 第 7 章：結論與未來工作

### 7.1 結論

本研究以 SNS3 為核心模擬平台，完成了三項工程主體：

1. **Layer 1 ISL Routing**（已完成）：在 Iridium-66 LEO 星座上實作離線預計算 + runtime 局部重算的 ISL routing framework，6 種端對端 pathType 全數通過封包層驗證
2. **Layer 2 Beam Hopping**（進行中）：BHTP-based 多 beam 動態調度架構，Phase 2 完成，Phase 3/4 待實作
3. **Layer 3 QoS Scheduling**（進行中）：架構完成，attribute 路徑待驗證

三個模擬器（SNS3、Hypatia、[TODO]）定位不同，應依研究需求選擇：SNS3 適合需要 MAC/PHY/Queue 精細建模的研究，Hypatia 適合大規模拓樸分析。

---

### 7.2 未來工作

> **[TODO]（必寫）** 說明：

1. **模擬器整合**：SNS3（精細 packet-level）+ Hypatia（大規模拓樸）的混合模擬架構
2. **Routing + Scheduling 聯合優化**：Layer 1 load-aware routing 與 Layer 3 QoS priority 的協同
3. **大規模星座適應性**：將現有架構擴展至 Starlink 級別（~1000+ 顆）星座的可行性分析
4. **ISL 飽和壓測**：設計高負載場景驗證 load-driven 路由切換行為

> **建議圖表**：圖 7-1：Hybrid simulation 架構圖（SNS3 精細層 + Hypatia 拓樸層整合示意）
