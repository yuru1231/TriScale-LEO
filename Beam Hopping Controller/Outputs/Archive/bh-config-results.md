# Beam Hopping Controller — 設定與結果說明

> **資料來源：** `Beam Hopping Controller/Outputs/66_bhtp/`  
> **程式碼：** `Beam Hopping Controller/Codes/sat-bh-example.cc`  
> **更新日期：** 2026-05-18

---

## 0. 設計目標 vs 現階段進度

### 0.1 設計目標圖（尚未達成）

圖為 Beam Hopping 控制器的**設計規格示意**，呈現最終預計達到的排程行為：


> **說明：** 此圖對應 `leo2sat` 場景（N=7），展示三項設計機制同時運作：
> 1. **K=3 多波束並排**：每 Slot 同時開啟 3 個波束（B1、B3、B6），其餘 4 個靜默
> 2. **動態波束半徑**：熱點 B1 使用 SMALL（高密度），B3 使用 MIDDLE，非熱點 B6 使用 LARGE（廣覆蓋）
> 3. **Cluster 干擾控制**：選定的三個波束彼此距離夠遠（ω < κ），不合入同一 Cluster，確保同 Slot 無干擾

**此場景的完整驗證條件（尚未執行）：**

```bash
# 待執行：leo2sat + Phase F（N=7, K=2, 含真實 DAMA 流量）
./ns3 run "sat-bh-example --satId=1 --simTime=120 \
           --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --enablePhaseF=1" \
  2>&1 | tee Outputs/leo2sat_phasef/run.log
```

---

### 0.2 現階段已完成的驗證

目前所有有效 KPI 結果來自 **`iridium66` 場景**（N=2，K=1），屬於**基礎功能驗證階段**，目的是確認各模組介面接線正確，而非驗證上圖的多波束排程機制。

| 驗證目標 | 對應場景 | 狀態 |
|----------|----------|------|
| BHTP 生成與 Slot 時序正確 | iridium66，Phase 1 | ✅ 完成 |
| KPI 指標收集（吞吐量、延遲、公平性） | iridium66，Phase F | ✅ 完成 |
| ResourceManager 每 503 ms 執行 frame optimization | leo2sat，Phase E | ✅ 完成（log 確認） |
| SNS3 API 接線（MoveUtBetweenBeams、SetTxMaxPowerDbw） | leo2sat，Phase E | ✅ 完成（log 確認） |
| DAMA Backlog trace → ResourceManager | iridium66，Phase F | ✅ 完成 |
| **N=7, K=3 多波束排程 + 動態 radius + 干擾控制** | leo2sat，Phase F | ❌ **尚未執行** |

---

## 1. 模擬設定

### 1.1 關鍵參數說明

| 參數 | 全名 | 意義 |
|------|------|------|
| **N** | 每衛星波束數（numBeams） | 一顆衛星上的波束總數；決定可排程的資源池大小 |
| **K** | 同時啟動波束上限（maxActiveBeams） | 每個 Slot 最多同時照射幾個波束；**K < N 才構成 Beam Hopping**，K = N 等同全開無需 hopping |
| **M** | 每幀 Slot 數 | M = round(T_p / T_s) = round(503 / 26) = **19**；EM 演算法將 M×K 個 Slot 資源分配給 N 個波束 |

### 1.2 場景設定

| 參數 | `leo2sat` | `iridium66` |
|------|-----------|-------------|
| 場景名稱 | `constellation-leo-2-satellites` | `constellation-iridium-66-sats` |
| 衛星數量 | 2 | 66 |
| 每衛星波束數 N | **7** | **2**（72 beams ÷ 66 sats） |
| K（Layer2.md 規格值） | 3 | 3 |
| K（實際執行值） | **2** | **1**（程式碼強制覆蓋）† |
| 熱點波束數 | 3 | — |
| GW 節點 | 2（GW2、GW3） | — |
| UT 節點 | 5 | 每波束 1 個 |
| 換手 (Handover) | 啟用 | 啟用 |
| 再生模式 | NETWORK | NETWORK |
| **目前有有效 KPI？** | ❌（Phase E 流量為 0） | ✅（Phase F） |

> **† iridium66 K 強制覆蓋原因：** N=2 時若 K=2，每 Slot 同時開全部波束，Beam Hopping 失效。  
> 強制 K=1 使每 Slot 嚴格交替單一波束，維持 hopping 可驗證性。  
> 這是目前 N=2 星座下的 workaround，規格目標 K=3 需 N ≥ 4 才能正確驗證。

### 1.3 時序參數（兩個場景共用）

| 參數 | 數值 | 說明 |
|------|------|------|
| T_p | 503 ms | BHTP 週期（Frame 時長）；EM 演算法每 T_p 重新計算一次 BHTP |
| T_s | 26 ms | 每個 Slot 時長；對應 DVB-S2X Super-frame 長度 |
| T_sw | 0 ms | 波束切換 dead-time；現階段設為 0，實際部署時約 2 ms |
| T_prop | 270 ms | NCC → 衛星 OBC 的指令傳播延遲；影響 BHTP 下發時機 |
| Warm-up | 10 s | 前 10 s 資料不計入 KPI，讓連線建立與 DAMA 協商完成 |

### 1.4 演算法參數（預設值）

| 參數 | 預設值 | 公式 | 說明 |
|------|--------|------|------|
| α | 2.0 | `A_n = demand_n × α × (1 + 1/W)` | 虛擬流量延遲敏感係數；α 越大，delay 敏感波束獲得越多 Slot |
| κ | 0.08 | `ω_{i,j} = exp(−2.77×(d_ij/r)²) ≥ κ` → 同 Cluster | Cluster 干擾合併門檻；κ 越大越保守，同 Slot 排程的波束間距要求越遠 |
| W | 5 | `λ_n = max(1, (1/W×M) × Σ x_{n,t})` | EM Observation Window；窗口大小 = W×M = 95 Slot |
| nonHotspotPercentile | 0.25 | `d_n ≤ 0.25 × max(d_n)` → LARGE beam | d_n 低於最大值 25% 的波束歸為非熱點，改用大半徑廣覆蓋 |
| demandChangeThreshold | 0.20 | `\|curr−prev\|/prev > 0.20` 連續 2 次 → 提前觸發 | 需求突變超過 20% 連續兩次時，不等 T_p 就立即重排程 |
| schedulingMode | 0（WFQ） | `weight = (p+1) × (1 + bufFill + demFrac)` | UT-波束分配模式：0=WFQ / 1=Priority / 2=RoundRobin |
| maxReassignment | 5 | — | 每 Frame MoveUtBetweenBeams 次數上限；優先執行 deadline 違規的換手 |

**各參數詳細說明：**

**α — 虛擬流量公式**（[sat-bh-scheduler.cc:551](../Codes/sat-bh-scheduler.cc)）
```cpp
// A_n = demand_n × α × (1 + 1/W)
double urgency = 1.0 + 1.0 / static_cast<double>(m_observationWindow);
m_virtualTraffic[n] = demand * m_alphaDelaySensitivity * urgency;
```
A_n 決定 BuildPlan 時熱點波束的排程優先序，A_n 越大越優先取得 Slot。

---

**κ — Cluster 干擾模型**（[sat-bh-scheduler.cc:592](../Codes/sat-bh-scheduler.cc)）
```cpp
// ω_{i,j} = exp(−2.77 × (d_ij / r_MIDDLE)²)
// r_MIDDLE = 20 km；六角格間距 70 km → 相鄰 beam ω ≈ 0.02
double omega = std::exp(-2.77 * (dij / rMiddle) * (dij / rMiddle));
// 若 ω >= κ，beam i 與 beam j 合入同一 Cluster，不可同 Slot 排程
```

---

**W — EM 需求估計**（[sat-bh-scheduler.cc:440](../Codes/sat-bh-scheduler.cc)）
```cpp
// λ_n = (1 / W×M) × Σ_t x_{n,t}
double windowSz = static_cast<double>(m_observationWindow * M); // W×M = 95
lambdaNew[n] = std::max(1.0, sum / windowSz);
```
λ_n 是每 Slot 平均需求量（bytes），再轉換為 d_n = max(1, round(λ_n / Σλ × M×K))。

---

**schedulingMode — WFQ 權重公式**（[sat-bh-user-associator.cc:212](../Codes/sat-bh-user-associator.cc)）
```cpp
// weight = (priorityClass+1) × (1 + bufferFill + demandFraction)
double bufferFill = ut.bufferBytes / 1500.0;          // 以 MTU 正規化
double demandFrac = std::max(ut.demandKbps, m_minRateKbps) / m_minRateKbps;
return static_cast<double>(ut.priorityClass + 1) * (1.0 + bufferFill + demandFrac);
```
priority class 高 → weight 乘數大；buffer 積壓越多 → 越急需分配到波束。

---

**demandChangeThreshold — 早觸發邏輯**（[sat-bh-scheduler.cc:263](../Codes/sat-bh-scheduler.cc)）
```cpp
double changeRatio = std::abs(curr - prev) / prev;
if (changeRatio > m_demandChangeThreshold) {
    m_consecutiveChanges[idx]++;
    if (m_consecutiveChanges[idx] >= 2)   // 連續 2 次才觸發，避免噪聲誤觸
        RunSchedulingCycle(Simulator::Now());
}
```

### 1.5 功率分配參數（Phase D / E / F）

| 參數 | 數值 | 說明 |
|------|------|------|
| P_total | 43.00 dBm | 總 TX 功率預算；IWFA 在此預算內對各波束做 Water-filling |
| N₀ | −126.47 dBW | 熱雜訊底限；決定每個波束的 SNR 下界 |
| ICI 係數 | 0.01 | 跨波束干擾洩漏比例；同 Cluster 內波束間的功率串擾估計值 |
| IWFA 最大迭代 | 30 | Water-filling 收斂迭代上限 |
| IWFA ε | 0.001 W | 收斂門檻；前後迭代功率差 < ε 時停止 |

---

## 2. Beam Hopping Time Plan（BHTP）

### 2.1 leo2sat — N=7 波束，K=2（介面驗證用）

```
T_p = 503 ms，19 個 Slot，每 Slot 26 ms
波束對輪替（K=2，Round-Robin，避免相鄰波束同時排程）：
  Slot 0:  {1,4}   Slot 1:  {2,5}   Slot 2:  {3,6}
  Slot 3:  {1,7}   Slot 4:  {2,4}   Slot 5:  {3,5}
  Slot 6:  {1,6}   Slot 7:  {2,7}   Slot 8:  {3,4}
  Slot 9:  {1,5}   Slot 10: {2,6}   Slot 11: {3,7}
  Slot 12: {1,4}   Slot 13: {2,5}   Slot 14: {3,6}
  Slot 15: {1,7}   Slot 16: {2,4}   Slot 17: {3,5}
  Slot 18: {1,6}
```

**每波束駐留時間（每週期）：**

| 波束 | 駐留時間 (ms) | 駐留時間 / T_p | 類型 |
|------|--------------|---------------|------|
| 1 | 182 ms | 36.2% | 熱點（最高優先） |
| 2 | 156 ms | 31.0% | 熱點 |
| 3 | 156 ms | 31.0% | 熱點 |
| 4 | 130 ms | 25.8% | 非熱點 |
| 5 | 130 ms | 25.8% | 非熱點 |
| 6 | 130 ms | 25.8% | 非熱點 |
| 7 | 104 ms | 20.7% | 非熱點（最低優先） |

### 2.2 iridium66 — N=2 波束，K=1（基礎功能驗證用）

```
T_p ≈ 494 ms，19 個 Slot，每 Slot 26 ms
嚴格交替（K=1）：波束 1 → 波束 2 → 波束 1 → …
  波束 1 駐留：240 ms（10 個 Slot）
  波束 2 駐留：216 ms（9 個 Slot）
```

> 此 BHTP 為靜態設計，無法展示動態 radius 選擇或多波束干擾控制。  
> Phase E 雖有 radius 差異化（波束 1=SMALL，波束 2=LARGE），但因 K=1 每次只有一個波束啟動，Cluster 機制未觸發。

---

## 3. Phase 啟用狀態總覽

| Phase | 啟用功能 | 場景 | 模擬時長 | 輸出資料夾 | 驗證目的 |
|-------|----------|------|----------|-----------|----------|
| **Phase 1** | TimePlan + Metrics | iridium66 | 60 s | `66_bhtp/phase1/` | BHTP 時序與 KPI 框架正確性 |
| **Phase 1 + 流量** | + CBR FWD/RTN | iridium66 | 60 s | `66_bhtp/traffic/` | 有流量時 KPI 是否正常收集 |
| **Phase 2** | + Scheduler + OBC stubs | iridium66 | 60 s | `66_bhtp/phase2/` | 介面接線（stub 層） |
| **Phase C** | + ResourceManager + UserAssociator | leo2sat | 300 s | `bh_phasec.log` | Frame 主控迴圈運作 |
| **Phase D** | + IWFA 功率分配 | leo2sat | 60 s | `bh_phased_v1.log` | IWFA Water-filling 迭代 |
| **Phase E** | + 真實 SNS3 API 接線 | leo2sat（satId=1） | 60 s | `bh_phasee_v2.log` | MoveUtBetweenBeams / SetTxMaxPowerDbw callback |
| **Phase E (iridium66)** | Phase E 套用至完整星座 | iridium66 | 60 s | `66_bhtp/phasee/` | 66 衛星規模下 API 接線穩定性 |
| **Phase F** | + BacklogRequestsTrace → DAMA | iridium66 | 120 s | `66_bhtp/phasef/` | 真實需求驅動排程端到端流程 |

---

## 4. KPI 結果

> **注意：** 結果來自 **iridium66（N=2，K=1）**。  
> 驗證**基礎流程正確性**（BHTP 執行、KPI 收集、DAMA 接線），  

每 T_p = 503 ms 輸出一筆 (時間, sat_id, beam_id) 記錄。
Warm-up = 10 s → 第一筆有效資料約在 t ≈ 10.5 s。

### 4.1 Phase 1 + 流量 — `66_bhtp/phase1/`（iridium66，CBR 合成流量）

**代表什麼：** BHTP 時序正確、KPI 指標可正常輸出、靜態交替排程下的基準吞吐量。

| 指標 | 波束 1 | 波束 2 |
|------|--------|--------|
| 吞吐量 | 0.109 Mbps | 0.098 Mbps |
| 平均延遲 | 10 ms | 10 ms |
| 最大延遲 | 10 ms | 10 ms |
| 駐留時間 | 240 ms | 216 ms |
| Slot 使用率 | 79.2% | 79.2% |
| 丟包率 | 1.96% | 0% |
| Jain 公平性指數 | 0.9972 | 0.9972 |

> 66 顆衛星 KPI 完全相同，因合成流量對每顆衛星均等分配。  
> 波束 1 丟包率 1.96%：CBR 速率略超過波束每週期容量上限，在現階段可接受。

---

### 4.2 Phase 2 — `66_bhtp/phase2/`（iridium66，Scheduler + OBC stubs）

**代表什麼：** Scheduler 與 OBC 的介面框架已接線，等待真實 EM 演算法填入。

| 指標 | 全部波束 |
|------|----------|
| 吞吐量 | 0 Mbps |
| Slot 使用率 | 100% |
| 丟包率 | 0% |
| Jain 公平性指數 | 1.0 |

> 吞吐量為 0 是預期結果：OBC stub 只切換狀態機，不注入真實封包。  
> 100% Slot 使用率確認 OBC 狀態機週期運作正常。

---

### 4.3 Phase E — `bh_phasee_v2.log`（leo2sat，真實 SNS3 API 接線）

**代表什麼：** `MoveUtBetweenBeams` 與 `SetTxMaxPowerDbw` 已成功掛接到 SNS3，ResourceManager 每 frame 正常執行。

| 指標 | 全部波束 |
|------|----------|
| 吞吐量 | 0 Mbps |
| Slot 使用率 | 100% |

> 吞吐量為 0 是預期結果：此次執行沒有加入應用層流量來源。  
> Log 確認：ResourceManager 每 503 ms 執行一次，5 個 UT 均有追蹤，IWFA 功率分配每 frame 執行。

---

### 4.4 Phase F — `66_bhtp/phasef/`（iridium66，真實 DAMA demand，120 s）

**代表什麼：** BacklogRequestsTrace 成功接入 ResourceManager，真實需求驅動排程端到端流程可運作。

**早期（t = 10.503 s，warm-up 後第一筆）：**

| 指標 | 波束 1 | 波束 2 |
|------|--------|--------|
| 吞吐量 | 0.057 Mbps | 0.051 Mbps |
| 平均延遲 | 10 ms | 10 ms |
| Slot 使用率 | 79.2% | 79.2% |
| 丟包率 | 1.96% | 0% |
| Jain 公平性指數 | 0.9972 | 0.9972 |

**中期（t = 14.024 s，需求累積後）：**

| 指標 | 波束 1 | 波束 2 |
|------|--------|--------|
| 吞吐量 | 1.193 Mbps | 1.074 Mbps |
| 平均延遲 | 10 ms | 10 ms |
| Slot 使用率 | 79.2% | 79.2% |
| 丟包率 | 1.96% | 0% |
| Jain 公平性指數 | 0.9972 | 0.9972 |

> 吞吐量隨時間增長，確認 DAMA 需求累積後 ResourceManager 正確回應。  
> 延遲穩定在 10 ms，受 T_s = 26 ms 粒度約束。

---

## 5. 各 Phase 比較

| Phase | 場景 | 吞吐量（波束 1） | Slot 使用率 | 驗證了什麼 |
|-------|------|----------------|------------|-----------|
| Phase 1 + 流量 | iridium66 | 0.109 Mbps | 79.2% | BHTP 時序、KPI 收集基準 |
| Phase 2（stubs） | iridium66 | 0 Mbps | 100% | Scheduler/OBC 介面框架 |
| Phase E（無流量） | leo2sat | 0 Mbps | 100% | SNS3 API callback 接線 |
| **Phase F（DAMA）** | iridium66 | **0.057→1.19 Mbps** | **79.2%** | **端到端 DAMA 驅動排程流程** |
| **待執行：Phase F** | **leo2sat** | **—** | **—** | **N=7 多波束排程、動態 radius、Cluster 干擾控制** |

---

## 6. 已知問題與待辦

| 問題 | 狀態 |
|------|------|
| **iridium66 K=1 與規格 K=3 不符** | Workaround：N=2 下 K 只能為 1。規格驗證需改用 leo2sat + Phase F（N=7，K=2） |
| **設計圖機制（多波束、動態 radius、Cluster）未有 KPI 驗證** | 待執行 leo2sat Phase F 實驗 |
| Phase 2 吞吐量為 0 | 預期：OBC stub，待補真實 EM 演算法 |
| 波束 1 丟包率 1.96% | CBR 速率略超容量，現階段可接受 |
| 全部衛星 KPI 完全相同 | iridium66 均等負載；需異質需求看 per-sat 差異 |
| T_prop / T_sw 未納入 Slot 時序 | 傳播延遲尚未從駐留時間扣除，影響鏈路預算驗證 |

---

## 7. 執行指令

```bash
# Phase 1（iridium66，含 CBR 流量）
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=60" \
  2>&1 | tee Outputs/66_bhtp/traffic/run.log

# Phase 2（Scheduler + OBC stubs）
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=60 \
           --enableScheduler=1 --enableObc=1" \
  2>&1 | tee Outputs/66_bhtp/phase2/run.log

# Phase E（真實 SNS3 接線，leo2sat）
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --satId=1 --simTime=60" \
  2>&1 | tee bh_phasee_v2.log

# Phase F（iridium66，DAMA demand）
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=120 \
           --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --enablePhaseF=1" \
  2>&1 | tee Outputs/66_bhtp/phasef/run.log

# ── 待執行：leo2sat Phase F（對應設計目標圖）──────────────────────
./ns3 run "sat-bh-example --satId=1 --simTime=120 \
           --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --enablePhaseF=1" \
  2>&1 | tee Outputs/leo2sat_phasef/run.log
```
