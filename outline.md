# NTN模擬器研究報告 — 大綱

> 日期：2026-04-22

---

## 1. 緒論（Introduction）

### 1.1 研究背景

#### 1.1.1 NTN（Non-Terrestrial Networks）介紹
- NTN 的定義與範疇（衛星、HAPS、UAV）
- 3GPP NTN 標準化進程（Rel-17/18）
- NTN 相較 TN 的核心差異：高延遲、高移動性、大覆蓋範圍

#### 1.1.2 LEO 衛星系統發展
- LEO 定義與軌道特性（高度 550–1200 km）
- 代表星座：Starlink、OneWeb、Telesat
- 技術趨勢：星間鏈路（ISL）、再生酬載（Regenerative Payload）

---

### 1.2 問題定義

#### 1.2.1 Dynamic Topology
- 衛星高速移動導致拓樸頻繁變化
- 鏈路可用性視窗短暫

#### 1.2.2 高延遲 / 長距離傳輸
- 傳播延遲對 QoS 的影響
- 多跳路由路徑規劃的複雜度

---

### 1.3 三大研究主題

| 主題 | 對應層 | 核心問題 |
|------|--------|----------|
| Routing | 網路層 | 如何在動態拓樸下選最短/最優路徑 |
| Beam Hopping | MAC/PHY 層 | 如何在時槽內動態調配 beam 資源 |
| Scheduling | MAC 層 | 如何分配頻寬與 RB 以滿足 QoS |

---

### 1.4 研究動機

#### 1.4.1 為何需要模擬器
- 實際衛星部署成本極高
- 模擬器可驗證演算法正確性與效能
- 重現性（Reproducibility）對學術研究的重要性

#### 1.4.2 模擬器在研究中的角色
- 作為 Proof-of-Concept 工具
- 作為 Benchmark 平台
- 與真實系統的差距（Gap Analysis）

---

### 本章必要圖表

| 編號 | 圖表名稱 | 說明 |
|------|----------|------|
| 圖1 | NTN系統架構圖 | 展示 GW / UT / SAT / ISL 各節點關係 |
| 圖2 | LEO拓樸隨時間變化圖 | 展示 t=0s / t=30s / t=60s 拓樸差異 |

---

---

## 2. 相關研究與模擬需求分析

---

### 2.1 Beam Hopping

#### 2.1.1 論文分類與概述

| 類別 | 數量 | 描述 |
|------|------|------|
| Survey | 2–3 篇 | 綜述 Beam Hopping 技術演進 |
| Method | 5–8 篇 | 演算法提案（優化、學習等） |
| System | 2–4 篇 | 系統設計或實作驗證 |

#### 2.1.2 代表論文深入分析（挑3篇）

**論文 A**
- 問題定義
- 方法（演算法/架構）
- 模擬方式（工具、情境、指標）

**論文 B**（同上格式）

**論文 C**（同上格式）

#### 2.1.3 論文比較

**表2-1：Beam Hopping 論文整理表**

| 編號 | 論文 | 方法 | 模擬器 | 需求功能 |
|------|------|------|--------|----------|
| BH-1 | | | | |
| BH-2 | | | | |
| BH-3 | | | | |

#### 2.1.4 模擬需求分析（核心）

- **Beam Model**：beam 數量、footprint、頻率複用
- **Time-slot Simulation**：slot 長度、切換延遲
- **Interference Model**：co-channel / adjacent-beam 干擾
- **Power Allocation**：per-beam 功率控制介面

---

### 本節必要圖表

| 編號 | 圖表名稱 | 說明 |
|------|----------|------|
| 圖3 | Beam Hopping 時間切換圖 | 展示 slot 與 beam 的時間對應關係 |
| 圖4 | Beam Footprint 示意圖 | 展示地面覆蓋與頻率複用模式 |

---

### 2.2 Routing

#### 2.2.1 論文分類與概述

| 類別 | 數量 | 描述 |
|------|------|------|
| Survey | 2–3 篇 | 衛星路由技術綜述 |
| Method | 5–8 篇 | 路由演算法提案 |
| System | 2–4 篇 | 系統整合與實驗 |

#### 2.2.2 代表論文深入分析（挑3篇）

**論文 A / B / C**（同 2.1.2 格式）

#### 2.2.3 論文比較

**表2-2：Routing 論文整理表**

| 編號 | 論文 | 方法 | 模擬器 | 需求功能 |
|------|------|------|--------|----------|
| RT-1 | | | | |
| RT-2 | | | | |
| RT-3 | | | | |

#### 2.2.4 模擬需求分析

- **Dynamic Topology**：拓樸更新週期、圖結構表示
- **ISL Modeling**：鏈路容量、傳播延遲計算
- **Routing API**：路徑查詢介面、cost function 可注入性
- **Traffic Model**：流量產生、端對端路徑追蹤

---

### 本節必要圖表

| 編號 | 圖表名稱 | 說明 |
|------|----------|------|
| 圖5 | LEO Network Graph | 展示衛星節點、ISL、GW 間拓樸 |
| 圖6 | Routing Path 變化圖 | 展示相同 src/dst 在不同時間的路徑差異 |

---

### 2.3 Scheduling

#### 2.3.1 論文分類與概述

| 類別 | 數量 | 描述 |
|------|------|------|
| Survey | 2–3 篇 | MAC 排程技術綜述 |
| Method | 5–8 篇 | QoS-aware 排程演算法 |
| System | 2–4 篇 | 系統實作或跨層設計 |

#### 2.3.2 代表論文深入分析（挑3篇）

**論文 A / B / C**（同上格式）

#### 2.3.3 論文比較

**表2-3：Scheduling 論文整理表**

| 編號 | 論文 | 方法 | 模擬器 | 需求功能 |
|------|------|------|--------|----------|
| SC-1 | | | | |
| SC-2 | | | | |
| SC-3 | | | | |

#### 2.3.4 模擬需求分析

- **MAC Scheduler**：RR / PF / QoS-aware 排程器介面
- **Queue**：每流佇列、優先權佇列、緩衝區管理
- **QoS Metrics**：吞吐量、延遲、Jitter、封包遺失率

---

### 本節必要圖表

| 編號 | 圖表名稱 | 說明 |
|------|----------|------|
| 圖7 | Resource Block 分配圖 | 展示時頻域 RB 分配與流量對應 |
| 圖8 | Queue 模型圖 | 展示每 UT 的多佇列結構 |

---

### 2.4 模擬需求總整理

#### 2.4.1 功能彙整

**表2-4：各研究主題模擬功能需求對應表**

| 功能 | Routing | Beam Hopping | Scheduling |
|------|---------|--------------|------------|
| Dynamic Topology | ✅ | ✅ | ❌ |
| ISL Modeling | ✅ | ❌ | ❌ |
| Beam Model | ❌ | ✅ | ❌ |
| Time-slot Sim | ❌ | ✅ | ✅ |
| MAC Scheduler | ❌ | ❌ | ✅ |
| Queue Model | ❌ | ❌ | ✅ |
| QoS Metrics | ✅ | ✅ | ✅ |
| Traffic Model | ✅ | ✅ | ✅ |
| Interference | ❌ | ✅ | ❌ |

#### 2.4.2 關鍵觀察

- 哪些功能在三個主題中均出現（共同基礎需求）
- 哪些功能是特定主題的關鍵能力（差異化需求）
- 哪些功能在現有模擬器中尚未完整支援（缺口分析）

---

---

## 3. SNS3 模擬器分析

### 3.1 Overview
- 開發背景（ESA 資助，NS-3 延伸）
- 版本與維護狀態
- 主要應用場景：GEO/LEO 衛星網路端對端模擬

### 3.2 System Architecture
- 系統分層：應用層 → 傳輸層 → IP 層 → MAC/PHY
- Satellite-specific 模組與原生 NS-3 模組的邊界

**圖9：SNS3 架構圖**

### 3.3 Simulation Model
- 事件驅動（event-driven discrete time）
- 封包層級（packet-level fidelity）
- 移動模型：SGP4 / 軌道預測

### 3.4 Functional Modules
- **Routing 模組**：arbiter、routing table 更新
- **Channel 模組**：前向 / 返回鏈路通道模型
- **Queue 模組**：per-flow 佇列、優先權機制
- **Mobility 模組**：衛星軌跡計算

### 3.5 程式目錄架構 

**表3-1：SNS3 目錄結構**

| 路徑 | 說明 |
|------|------|
| `contrib/satellite/model/` | 核心模型類別 |
| `contrib/satellite/helper/` | 場景建構輔助類別 |
| `contrib/satellite/examples/` | 範例模擬腳本 |
| `contrib/satellite/utils/` | 工具函式 |

### 3.6 主程式流程 

**圖10：SNS3 主程式流程圖**

```
main()
  └─ SatHelper::CreatePredefinedScenario()
       ├─ Topology 建構（衛星、GW、UT 節點）
       ├─ Channel 安裝（前向 / 返回）
       ├─ IP 位址配置
       ├─ Routing 安裝（Arbiter）
       └─ Application 安裝（CBR / OnOff）
            └─ Simulator::Run()
```

### 3.7 功能支援分析

**表3-2：SNS3 功能支援表**

| 功能 | 支援狀態 | 備註 |
|------|----------|------|
| LEO Mobility | ✅ | SGP4 整合 |
| ISL | ⚠️ 部分 | 需擴充 |
| Dynamic Routing | ⚠️ 部分 | Arbiter 可擴充 |
| Beam Hopping | ❌ | 未原生支援 |
| QoS Scheduler | ⚠️ 部分 | 基礎 PF 已有 |
| Multi-gateway | ✅ | 原生支援 |

### 3.8 與論文需求對應（核心）

**表3-3：SNS3 與論文需求對應**

| 功能 | 論文需求 | 模擬器支援 | 是否需修改 |
|------|----------|------------|------------|
| Dynamic Topology | 高 | 部分 | 是 |
| ISL Modeling | 高 | 部分 | 是 |
| Beam Model | 高 | 基礎 | 是 |
| MAC Scheduler | 中 | 基礎 | 是 |
| QoS Metrics | 高 | 部分 | 否 |

### 3.9 Limitations
- ISL 支援尚不完整（需擴充 arbiter）
- Beam Hopping 無原生 time-slot 切換機制
- PHY 層抽象化程度高，干擾模型簡化

### 3.10 Extensibility
- NS-3 繼承架構允許模組替換
- Arbiter 介面可注入自訂路由邏輯
- Helper 層可擴充多場景支援

### 3.11 小結
- SNS3 強項：端對端封包模擬、多節點場景
- SNS3 弱項：Beam Hopping、ISL 路由精細度
- 適用研究：Scheduling、基礎 Routing 驗證

---

---

## 4. Hypatia 模擬器分析

### 4.1 Overview
- 開發背景（MIT / Amazon 合作）
- 版本與維護狀態
- 主要應用場景：LEO 星座拓樸分析與路由研究

### 4.2 System Architecture
- 拓樸層與網路模擬層的分離設計
- 與 NS-3 的整合介面

**圖11：Hypatia 架構圖**

### 4.3 Simulation Model
- 基於 NS-3 core，拓樸由 Python 預計算
- 靜態 snapshot 與動態更新機制

### 4.4 Functional Modules
- **Topology Generator**：衛星位置、ISL 圖構建
- **Routing 模組**：OSPF / 自訂 shortest path
- **Traffic Model**：flow-level / packet-level
- **Visualization**：拓樸可視化工具

### 4.5 程式目錄架構 

**表4-1：Hypatia 目錄結構**

| 路徑 | 說明 |
|------|------|
| `satgenpy/` | 衛星軌跡與拓樸生成（Python） |
| `ns3-sat-sim/` | NS-3 模擬主體 |
| `paper/` | 論文範例腳本 |
| `visualization/` | 拓樸可視化 |

### 4.6 主程式流程 

**圖12：Hypatia 主程式流程圖**

```
satgenpy (Python)
  ├─ 生成軌道資料（TLE）
  ├─ 計算每 epoch ISL 連通圖
  └─ 輸出 routing / forwarding table

NS-3 模擬
  ├─ 載入拓樸 snapshot
  ├─ 安裝靜態路由（per epoch 更新）
  ├─ 安裝 Application
  └─ Simulator::Run()
```

### 4.7 功能支援分析

**表4-2：Hypatia 功能支援表**

| 功能 | 支援狀態 | 備註 |
|------|----------|------|
| LEO Mobility | ✅ | TLE-based |
| ISL | ✅ | 核心功能 |
| Dynamic Routing | ✅ | Epoch-based 更新 |
| Beam Hopping | ❌ | 無 PHY 層 |
| QoS Scheduler | ❌ | 無 MAC 層 |
| Multi-gateway | ✅ | 支援 |

### 4.8 與論文需求對應（核心）

**表4-3：Hypatia 與論文需求對應**

| 功能 | 論文需求 | 模擬器支援 | 是否需修改 |
|------|----------|------------|------------|
| Dynamic Topology | 高 | 完整 | 否 |
| ISL Modeling | 高 | 完整 | 否 |
| Beam Model | 高 | 無 | 是 |
| MAC Scheduler | 中 | 無 | 是 |
| QoS Metrics | 高 | 部分 | 是 |

### 4.9 Limitations
- 無 MAC/PHY 層，無法模擬 Beam Hopping
- 排程功能缺失，無 QoS 機制
- 拓樸更新為 epoch-based，非連續動態

### 4.10 Extensibility
- Python 拓樸生成層易於修改
- 可與 SNS3 整合補足 MAC 層

### 4.11 小結
- Hypatia 強項：ISL 拓樸、動態路由
- Hypatia 弱項：PHY/MAC 層缺失
- 適用研究：Routing、拓樸分析

---

---

## 5. OAI NTN 模擬器分析

### 5.1 Overview
- 開發背景（OpenAirInterface Software Alliance）
- 版本與維護狀態
- 主要應用場景：5G NR NTN PHY/MAC 實驗

### 5.2 System Architecture
- 基於 OAI 5G NR 架構延伸至 NTN
- 主要修改：TA 補償、Doppler、延遲補償

**圖13：OAI NTN 架構圖**

### 5.3 Simulation Model
- 基於 OAI 軟體無線電框架
- 可連接真實 RF 前端或模擬通道

### 5.4 Functional Modules
- **PHY 層**：波形生成、通道估測、解碼
- **MAC 層**：排程器（gNB side）、HARQ
- **RRC 層**：連線管理
- **NTN 擴充**：TA 補償、衛星軌跡整合

### 5.5 程式目錄架構 

**表5-1：OAI NTN 目錄結構**

| 路徑 | 說明 |
|------|------|
| `openair1/` | PHY 層實作 |
| `openair2/` | MAC / RLC / PDCP |
| `openair3/` | RRC / NAS |
| `ntn/` | NTN 擴充模組 |

### 5.6 主程式流程 

**圖14：OAI NTN 主程式流程圖**

```
gNB 主程式
  ├─ PHY 初始化（波形、通道）
  ├─ MAC Scheduler 啟動
  ├─ NTN TA 補償啟用
  └─ 模擬迴圈（subframe-level）
       ├─ 排程決策
       ├─ 封包傳送
       └─ 統計收集
```

### 5.7 功能支援分析

**表5-2：OAI NTN 功能支援表**

| 功能 | 支援狀態 | 備註 |
|------|----------|------|
| LEO Mobility | ✅ | TA 補償 |
| ISL | ❌ | 無 |
| Dynamic Routing | ❌ | 無 |
| Beam Hopping | ⚠️ 部分 | 需擴充 |
| QoS Scheduler | ✅ | MAC 排程器完整 |
| Multi-gateway | ❌ | 不適用 |

### 5.8 與論文需求對應（核心）

**表5-3：OAI NTN 與論文需求對應**

| 功能 | 論文需求 | 模擬器支援 | 是否需修改 |
|------|----------|------------|------------|
| Dynamic Topology | 高 | 無 | 是 |
| ISL Modeling | 高 | 無 | 是 |
| Beam Model | 高 | 基礎 | 是 |
| MAC Scheduler | 中 | 完整 | 否 |
| QoS Metrics | 高 | 完整 | 否 |

### 5.9 Limitations
- 無網路層路由功能
- ISL 完全缺失
- 動態拓樸不支援

### 5.10 Extensibility
- PHY/MAC 層高度可擴充
- 可作為 SNS3 的 PHY backend

### 5.11 小結
- OAI NTN 強項：PHY/MAC 精細度、QoS 排程
- OAI NTN 弱項：網路層以上全缺
- 適用研究：Scheduling、Beam Hopping PHY 驗證

---

---

## 6. 跨模擬器比較

### 6.1 功能比較

**表6-1：三模擬器功能比較**

| 功能 | Hypatia | SNS3 | OAI NTN |
|------|---------|------|---------|
| LEO Mobility | ✅ | ✅ | ✅ |
| ISL | ✅ | ⚠️ | ❌ |
| Dynamic Routing | ✅ | ⚠️ | ❌ |
| Beam Hopping | ❌ | ❌ | ⚠️ |
| MAC Scheduler | ❌ | ⚠️ | ✅ |
| QoS Metrics | ⚠️ | ⚠️ | ✅ |
| Packet-level Sim | ✅ | ✅ | ✅ |
| End-to-End Sim | ✅ | ✅ | ❌ |

### 6.2 架構層級比較

**圖15：三模擬器層級定位圖**

```
┌─────────────────────────────────────────┐
│            Application Layer            │
├─────────────────────────────────────────┤
│  Network Layer (Routing / Topology)     │  ← Hypatia 強項
├─────────────────────────────────────────┤
│  Transport / IP Layer                   │
├─────────────────────────────────────────┤
│  MAC / Scheduling / Queue               │  ← SNS3 中間層
├─────────────────────────────────────────┤
│  PHY / Air Interface / Beam             │  ← OAI NTN 強項
└─────────────────────────────────────────┘
```

### 6.3 與論文需求對應

**表6-2：三模擬器與研究主題適配度**

| 研究主題 | 最適模擬器 | 次適 | 原因 |
|----------|------------|------|------|
| Routing | Hypatia | SNS3 | ISL 拓樸支援完整 |
| Beam Hopping | SNS3（擴充）| OAI NTN | 需跨 MAC/PHY 層 |
| Scheduling | OAI NTN | SNS3 | MAC 排程器完整 |

### 6.4 討論

- 單一模擬器無法滿足三大研究主題全部需求
- Hypatia + SNS3 組合可覆蓋 Routing + Scheduling
- OAI NTN 適合需要 PHY 精細度的 Beam Hopping 研究
- 整合三者面臨的主要挑戰：介面標準化、時間同步

---

---

## 7. 結論與未來工作

### 7.1 結論

- **Hypatia**：最適合 Routing 研究，ISL 拓樸完整，但缺乏 MAC/PHY 層
- **SNS3**：全層次封包模擬，適合 E2E 場景，但 ISL 與 Beam Hopping 需擴充
- **OAI NTN**：PHY/MAC 最精細，適合 Scheduling 研究，但無網路層
- 三種模擬器在研究定位上互補，非競爭關係

### 7.2 未來工作 

#### 7.2.1 模擬器整合
- 定義統一的拓樸資料格式（Topology API）
- 建立 Hypatia → SNS3 的路由表同步機制
- 探討 OAI NTN 作為 SNS3 PHY backend 的可行性

#### 7.2.2 Routing + Scheduling 聯合設計
- 在 SNS3 中實作 ISL-aware routing
- 將路由 cost 回饋至 MAC scheduler 決策
- 驗證跨層設計對 QoS 的提升效果

#### 7.2.3 Beam Hopping 整合
- 在 SNS3 中實作 time-slot 切換機制
- 結合 OAI NTN 干擾模型進行聯合最佳化

---

### 圖16：Hybrid Simulation 架構圖（加分）

```
┌──────────────┐     Topology/Route     ┌──────────────┐
│   Hypatia    │ ─────────────────────► │    SNS3      │
│ (Topology)   │                        │ (Network)    │
└──────────────┘                        └──────┬───────┘
                                               │ MAC primitives
                                               ▼
                                        ┌──────────────┐
                                        │  OAI NTN     │
                                        │ (PHY/MAC)    │
                                        └──────────────┘
```

---

---

## 附錄

### A. 論文清單（每人 10–15 篇）

| 編號 | 作者 | 年份 | 標題 | 類別 | 主題 |
|------|------|------|------|------|------|
| 1 | | | | Survey | Routing |
| 2 | | | | Method | Routing |
| ... | | | | | |

### B. 縮寫對照表

| 縮寫 | 全名 |
|------|------|
| NTN | Non-Terrestrial Network |
| LEO | Low Earth Orbit |
| ISL | Inter-Satellite Link |
| GW | Gateway |
| UT | User Terminal |
| RB | Resource Block |
| HARQ | Hybrid Automatic Repeat reQuest |
| TA | Timing Advance |
| TLE | Two-Line Element |

---

> **撰寫進度追蹤**
>
> - [ ] 第1章 緒論
> - [ ] 第2章 相關研究（2.1 BH / 2.2 RT / 2.3 SC / 2.4 總整理）
> - [ ] 第3章 SNS3 分析
> - [ ] 第4章 Hypatia 分析
> - [ ] 第5章 OAI NTN 分析
> - [ ] 第6章 跨模擬器比較
> - [ ] 第7章 結論
> - [ ] 圖表製作（圖1–16）
> - [ ] 論文清單補齊（每人 10–15 篇）
