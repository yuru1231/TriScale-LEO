# BH Dynamic BSTP

> **適用範圍：** `enableDynamicBstp=1, enableObc=1`  
> **測試情境：** `starlink25` — 25 beams，10 顆 helper satellite，T_p = 80 ms，T_s = 10 ms，K_b = 2

---

## 0. Phase 代表意義

| Phase | 主要模組 | 功能 | 啟動旗標 | 狀態 |
|-------|---------|------|---------|:----:|
| 1 | `SatBhTimePlan`、`SatBhMetrics` | 靜態 BHTP 資料模型 + Packet-level KPI CSV | 預設啟動 | ✅ |
| 2 | `SatBhScheduler`、`SatBhObc` | NCC EM 演算法產生 BHTP；OBC 狀態機執行 slot | `--enableScheduler=1 --enableObc=1` | ✅ |
| 3 | `SatGwCacheQueue`、`SatBhPrecoder` | Beam inactive 時 GW 封包緩衝；MMSE 預編碼 | `--enableCacheQueue=1` | ✅ |
| B | `SatL1RoutingInterface` | Layer 1 ISL routing 預留介面（stub） | 隨 Phase C 載入 | ✅ |
| C | `SatUserAssociator`、`SatResourceManager` | WFQ/Priority/RR UT-beam 分配；自我排程主控迴圈 | `--enableResourceManager=1 --enableUserAssociation=1` | ✅ |
| D | `SatPowerAllocator` | TX 功率最佳化 | `--enablePowerAllocation=1` | ✅ |
| E | `SatBhHelper::ConnectTracesPhaseE` | 接入真實 SNS3 API（`MoveUtBetweenBeams` / `SetTxMaxPowerDbw`） | `--enablePhaseE=1` | ✅ |
| F | `OnBacklogRequestTrace` | 接入真實 DAMA demand trace（`BacklogRequestsTrace → UpdateBeamDemand`） | `--enablePhaseF=1` | ✅ |
| **G** | `SatGreedyBstpProvider`、`RunDynamicBstpCycle` | 每 T_p 動態 greedy top-K beam 選擇，取代靜態 BHTP | `--enableDynamicBstp=1 --enableObc=1` | ✅ |

### Phase G 依賴關係

```
Phase 1–3 ──┐
             │  提供 OBC 狀態機、Metrics、CacheQueue
Phase G ─────┤  用 SatGreedyBstpProvider 取代 SatBhScheduler
             │  RunDynamicBstpCycle 每 T_p 呼叫 GetNextConf → ConfToTimePlan → OBC
Phase F ─────┘  (可選) 接入 RBDC demand → UpdateBeamDemand → 分數計算啟用
```

- Phase G **不需要** Phase C/D/E
- Phase G **需要** Phase 2（`enableObc=1`）執行 slot 狀態機
- Phase F 可選；未接線時 demand 全為 0 → 退回 round-robin
- `enableDynamicBstp` 與 `enableScheduler` 同時開啟時，**Scheduler 優先，Phase G 被跳過**

---

## 1. Config 與命令列參數

### 1.1 BhExperimentConfig

實驗設定物件，定義於 `sat-bh-helper.h`，控制：

- scenario、模擬時長
- 波束數量與每 slot 最多同時啟用數
- slot、切換與傳播延遲
- 排程演算法選擇
- 是否啟用 Resource Manager、OBC、功率分配

程式先建立預設 `cfg`，再由 CLI 覆蓋：

```cpp
BhExperimentConfig cfg;
cmd.AddValue("simTime", ..., cfg.simTimeSec);
cmd.AddValue("maxActiveBeams", ..., cfg.maxActiveBeams);
cmd.Parse(argc, argv);
```

### 1.2 Config::SetDefault()

ns-3 / SNS3 物件的全域預設值，例如：

```cpp
Config::SetDefault("ns3::SatHelper::HandoversEnabled", BooleanValue(true));
```

控制 SNS3 底層物件建立時的屬性（handover、ISL、queue、RBDC/CRA、原生 BH controller 等）。  
**必須在 `CreateSatScenario()` 之前設定**，物件建立後再改預設值不會生效。

### 1.3 bh-attributes.xml 快照

```cpp
ConfigStore outputConfig;
outputConfig.ConfigureDefaults();
```

這份 XML 是**本次執行的 ns-3 屬性快照**，目前僅用於記錄，不是輸入設定檔。

### 1.4 命令列參數

#### 情境與規模

| 參數 | 用途 | 值 |
|---|---|---:|
| `scenario` | 情境 | `starlink25` |
| `simTime` | 模擬秒數 | 300 |
| `numPeriods` | 執行幾個 BHTP period；非 0 時覆蓋 simTime | 0 |
| `warmUp` | KPI 暖機時間（不計入統計） | 10s |
| `numBeams` | 每顆衛星的邏輯波束數 | 25 |
| `maxActiveBeams` | 每時槽最多同時啟用波束數 K | 2 |
| `numHotspot` | Hotspot 波束數 | 5 |

`numPeriods` 設定後，模擬時間自動計算：

```
simTime = warmUp + numPeriods × T_p
例：--numPeriods=100 --warmUp=10 --periodMs=80 → simTime = 10 + 100×0.08 = 18 s
```

#### 時間參數

| 參數 | 意義 | 預設 |
|---|---|---|
| `slotMs` | 一個 slot 長度 T_s | 10 ms |
| `periodMs` | 一個 BHTP frame 長度 T_p | 80 ms |
| `switchMs` | 波束切換 dead time T_sw（不能傳輸） | 2 ms |
| `propMs` | 新計畫送到衛星的傳播延遲 T_prop | 10 ms |

預設情況：`F = T_p / T_s = 8 slots`，每 slot 可服務時間 = `T_s − T_sw = 8 ms`

#### 功能開關

| 開關 | 功能 |
|---|---|
| `enableScheduler` | Phase 2 EM 動態排程器 |
| `enableObc` | 依 time plan 切換波束 |
| `enableCacheQueue` | beam inactive 時 GW 封包暫存 |
| `enableResourceManager` | 每 frame 資源最佳化 |
| `enableUserAssociation` | UT 重新分配波束 |
| `enablePowerAllocation` | IWFA 功率分配 |
| `enablePhaseE` | 資源決策接入真實 SNS3 API |
| `enablePhaseF` | 從 DAMA/RBDC trace 取得真實需求 |
| `enableDynamicBstp` | Phase G greedy top-K 排程 |

使用 `0/1`

### 1.5 執行指令範例

```bash
# 先編譯
./ns3 build sat-bh-example

# 最小測試：確認程式與輸出正常
./ns3 run "sat-bh-example --scenario=leo2sat --simTime=15 --warmUp=2"

# 觀察動態 Beam Hopping（demand=0，RR fallback）
./ns3 run "sat-bh-example \
  --scenario=starlink25 --enableObc=1 --enableDynamicBstp=1 \
  --maxActiveBeams=2 --simTime=30 --warmUp=5" \
  2>&1 | tee bh_run.log

# 加入真實 RBDC demand（Phase F）
./ns3 run "sat-bh-example \
  --scenario=starlink25 --enableObc=1 --enableDynamicBstp=1 \
  --enablePhaseF=1 --maxActiveBeams=2 --simTime=120" \
  2>&1 | tee bh_run_rbdc.log
```

---

## 2. 主程式工作流程

### 2.1 sat-bh-example 概述

`sat-bh-example.cc` 負責：

1. 建立衛星、Gateway、UT 與網路拓樸
2. 建立 Forward / Return Link 流量
3. 根據設定啟用靜態或動態的波束排程
4. 在每個 slot 切換啟用中的波束
5. 蒐集 throughput、delay、slot utilization、公平性等 KPI
6. 輸出 CSV、SNS3 statistics 與設定快照

BH 功能主要實作於 `sat-bh-helper.cc`；`SatBhHelper` 是整套 BH 系統的組裝者，主程式不直接操作 Scheduler、OBC 或 Resource Manager。

執行順序：

```
Step 0  ParseConfig()           讀取 CLI，建立 BhExperimentConfig
Step 1  Config::SetDefault()    設定 SNS3 全域屬性
Step 2  CreateSatScenario()     建立衛星、GW、UT、PHY、MAC、channel
Step 3  建立流量               Forward / Return Link CBR / RBDC
Step 4  Install SatBhHelper     每顆被監控的衛星安裝 BH Helper
Step 5  建立 SNS3 統計收集器    per-beam / per-sat throughput、service time
Step 6  ConfigStore             輸出 bh-attributes.xml 快照
Step 7  RunSimulation()         模擬事件依序觸發
Step 8  FinalFlush()            確保最後一個未結束的 period 也輸出 KPI
```

`starlink25` Forward Link 流量設定：

- 5 個 hotspot UT（beam 1,4,13,19,22）：~600 kbps/UT（封包間隔 20 ms，1500 bytes）
- 20 個一般 UT：~120 kbps/UT（封包間隔 100 ms，1500 bytes）

### 2.2 元件關係圖

```
┌───────────────────────────────────────────────────────────────┐
│  SatBhHelper（Install / ConnectTraces）                       │
│                                                               │
│  ┌──────────────────────┐    T_p     ┌───────────────────┐   │
│  │ SatGreedyBstpProvider│ ◄──────── │ RunDynamicBstpCycle│  │
│  │  （Phase G 排程器）  │            │  （自我循環排程）  │   │
│  └──────────┬───────────┘            └────────┬──────────┘   │
│             │ GetNextConf(now)                 │ FlushMetrics │
│             ▼                                 ▼               │
│  ┌──────────────────────┐   plan    ┌──────────────────┐     │
│  │  ConfToTimePlan()    │ ─────────►│  SatBhObc        │     │
│  │  （Conf → TimePlan） │           │  （狀態機）       │     │
│  └──────────────────────┘           └────────┬─────────┘     │
│                                              │ BeamActivateCb │
│                                              ▼               │
│                                    ┌──────────────────┐      │
│                                    │  SatBhMetrics    │      │
│                                    │  （KPI 收集器）  │      │
│                                    └────────┬─────────┘      │
│                                             │ FlushMetrics    │
│                                             ▼               │
│                                    bh-metrics_*.csv          │
└───────────────────────────────────────────────────────────────┘
```

### 2.3 執行模式

#### Phase 1：靜態 BHTP（預設）

```bash
./ns3 run "sat-bh-example"
```

建立固定 Time Plan，使用 synthetic slot driver 模擬波束服務。  
用於驗證 Time Plan 格式、slot 排列、KPI 輸出、基本資料結構。不代表真實封包驅動的 Beam Hopping。

#### Phase 2：Scheduler + OBC

```bash
./ns3 run "sat-bh-example --enableScheduler=1 --enableObc=1"
```

```
Synthetic demand → SatBhScheduler（EM 演算法）→ SatBhTimePlan
    → (+ T_prop) → SatBhObc → ToggleState(SatNetDevice) → Metrics
```

Scheduler 每個 T_p 重新計算計畫；OBC 依 slot 執行波束切換。

#### Phase C–F：完整 Resource Manager 路徑

```bash
./ns3 run "sat-bh-example \
  --enableResourceManager=1 --enableUserAssociation=1 \
  --enablePowerAllocation=1 --enablePhaseE=1 --enablePhaseF=1 \
  --simTime=120"
```

每個 frame 的 Resource Manager 流程：

```
取得 active beams → 估算 beam capacity → 讀取 UT demand
    → WFQ/Priority/RR 關聯 → UT 換 beam 決策
    → 功率計算 → Phase E 套用到 SNS3 物件
```

`schedulingMode`：`0` = WFQ、`1` = Priority、`2` = Round Robin

**Phase F 特殊設定（`enablePhaseF=1` 效果）：**

```
CRA = 關閉
RBDC = 開啟（MinimumServiceRate=10 kbps，MaximumServiceRate=500 kbps）
VBDC = 關閉
ControlSlotInterval = 1 s
```

UT 依 Return Link backlog 發出 RBDC capacity request，`OnBacklogRequestTrace()` 呼叫 `UpdateBeamDemand()` 更新 beam demand。

#### Phase G：Greedy Dynamic BSTP

```bash
./ns3 run "sat-bh-example \
  --scenario=starlink25 --enableObc=1 --enableDynamicBstp=1 \
  --maxActiveBeams=4 --simTime=60"
```

每個 T_p 執行：

```
每個 beam 的 demand → 分數計算（需求 + 等待時間公平性）
    → starvation beam 強制選入 → top-K greedy 選 beam
    → ConfToTimePlan → OBC 切換波束 → KPI 輸出
```

demand 尚未到達時退回 Round Robin。

### 2.4 輸出檔案

```
bh-metrics_<scenario>_<YYYYMMDD>_<HHMMSS>.csv   ← 每 period/衛星/beam 的 KPI
bh-timeplan_<scenario>_<YYYYMMDD>_<HHMMSS>.csv  ← 靜態備援計畫（動態計畫不寫入）
bh-attributes.xml                                ← ns-3 屬性快照
data/                                            ← SNS3 原生 throughput/service-time 統計
```

注意：`bh-timeplan` CSV 在 `Install()` 時寫入靜態 fallback 計畫，Phase G 每 80 ms 產生的動態計畫**不寫入**此檔。10 個 Helper 共用同一檔名，最後一個 Helper 的計畫覆蓋之前的。

---

## 3. 時間軸

### 3.1 時間常數

| 符號 | 值 | 意義 |
|---|---|---|
| `T_s` | 10 ms | 一個 slot 的總時間窗口 |
| `T_sw` | 2 ms | 波束切換 dead-time（無資料傳輸） |
| `T_usable` | 8 ms | 實際可傳資料的時間（= T_s − T_sw） |
| `T_p` | 80 ms | 一個 BHTP period（= M × T_s = 8 × 10 ms） |
| `T_prop` | 10 ms | NCC → 衛星 OBC 的命令傳播延遲 |
| `K` | 2 | 每個 slot 最多同時啟用波束數 |
| `M` | 8 | 每個 period 的 slot 數 |

### 3.2 NCC 與 OBC 時序對照（一個 T_p）

```
NCC 端（SatGreedyBstpProvider + RunDynamicBstpCycle）
──────────────────────────────────────────────────────────────────────────────
Period N 開始
t=0ms   → RunDynamicBstpCycle()
          GetNextConf() → Conf{activeBeams=[b1,b2]}
          ConfToTimePlan(periodStart = 0 + T_prop = 10ms)
          OBC.ReceiveNewPlan(plan, T_prop=10ms)
          Simulator::Schedule(T_p=80ms, RunDynamicBstpCycle)   ← 預約下一輪

t=80ms  → RunDynamicBstpCycle()                                ← Period N+1 開始
          FlushMetrics() → 寫出 Period N 的 KPI CSV
          GetNextConf() → Conf{activeBeams=[b3,b4]}
          ConfToTimePlan(periodStart = 80 + 10 = 90ms)
          OBC.ReceiveNewPlan(plan, T_prop=10ms)
──────────────────────────────────────────────────────────────────────────────

衛星 OBC 端（SatBhObc 狀態機）
──────────────────────────────────────────────────────────────────────────────
t=0ms         WAIT_PLAN（尚未收到計畫）

t=10ms        收到 BHTP[N]（T_prop 到達）
              WAIT_PLAN → ACTIVE
              EnterSlot(0)：beam {b1,b2} 開啟，usable=8ms
              BeamActivateCallback(satId, b1, 8ms)
              BeamActivateCallback(satId, b2, 8ms)

t=18ms        OnSlotServiceEnd(0)：可用時間到
              BeamDeactivateCallback(satId, b1, 8ms)
              BeamDeactivateCallback(satId, b2, 8ms)
              ACTIVE → SWITCHING

t=20ms        OnSwitchingDone(1)：T_sw=2ms 結束
              SWITCHING → ACTIVE
              EnterSlot(1)：beam {b2,b1}（round-robin 輪換）

  ... 每 10ms 重複一個 slot，共 8 個 slot ...

t=88ms        OnSlotServiceEnd(7)：最後一個 slot 服務結束
              ACTIVE → SWITCHING

t=90ms        OnSwitchingDone(8)：無下一 slot
              收到 BHTP[N+1]（t=80+T_prop=90ms 到達）
              SWITCHING → ACTIVE（直接銜接，無 WAIT_PLAN gap）
              EnterSlot(0 of Period N+1)
──────────────────────────────────────────────────────────────────────────────
```

### 3.3 單一 Slot 內部放大

```
         ┌──────────────── T_s = 10 ms ────────────────┐
         │                                              │
         ├──── T_usable = 8 ms ─────────────────►│ T_sw │
         │                                       │ 2 ms │
         │  beam {b1, b2} 同時傳資料              │ 切換  │
         │                                       │dead  │
    BeamActivateCb                     BeamDeactivateCb │
    (satId, bx, 8ms)                   (satId, bx, 8ms)│
         │                                       │      │
    ACTIVE state                        SWITCHING state │
         │◄──── Metrics 計入 dwell_time_ms ─────►│      │
         └───────────────────────────────────────┴──────┘
```

### 3.4 跨 Period 銜接與 WAIT_PLAN gap

```
                T_prop=10ms
              ◄────────────►
t:  0ms      10ms          80ms      90ms         160ms
    │         │             │         │             │
NCC ├─Compute─┼──Send───────┼─Compute─┼──Send───────┤
    │ BHTP[N] │             │BHTP[N+1]│             │
    │         │             │         │             │
OBC │WAIT_PLAN│◄──BHTP[N]──►│         │◄──BHTP[N+1]►│
    │         │  slot 0~7   │         │  slot 0~7   │
    │         │  執行中      │         │  執行中      │

若 NCC 計算延遲 > T_s，OBC 在 t=88ms 後無計畫可執行：
  → ACTIVE → WAIT_PLAN（gap）→ 收到 BHTP[N+1] 後 WAIT_PLAN → ACTIVE
  → gap 期間無任何 beam 啟用，dwell_time 損失
```

### 3.5 關鍵注意點

| 問題 | 說明 |
|---|---|
| OBC 在 t=10ms 才開始 | T_prop=10ms，NCC 下發命令需傳播時間；OBC 必須等收到計畫後才執行 |
| `periodStart` 是誰的時間？ | NCC 視角的絕對模擬時間；`BhSlotEntry::startTime` 是相對 periodStart 的 offset |
| T_sw 不計入可用傳輸時間 | 每 slot 只有 8ms 可傳資料；OBC 在 `OnSlotServiceEnd()` 後才進入 SWITCHING |
| Period 交界若 BHTP 遲到 | OBC 進入 WAIT_PLAN，期間無 beam 啟用，dwell_time_ms 損失但 slot_util_pct 不變 |
| `dwell_time_ms` 的計算 | 由 `BeamActivateCb → BeamDeactivateCb` 之間的時間累積，不含 T_sw |

### 3.6 完整流程圖

![bhtp-dynamic](.\drawio\bh-dynamic.png)

---

## 4. 排程邏輯

### 4.1 RunDynamicBstpCycle() — `sat-bh-helper.cc`

每隔 T_p（80 ms）由 `Simulator::Schedule` 自我循環呼叫。每次呼叫順序：

1. **Flush KPI** — `m_metrics->FlushMetrics()` — 將剛完成的 T_p 寫入 CSV
2. **取得下一個 conf** — `m_dynamicProvider->GetNextConf(now)` — 詢問 greedy provider 要啟動哪些 beam
3. **驗證** — `ValidateConf(conf)` — 排除非法 conf（beam 數超限、feeder 頻率衝突）
4. **建構 plan** — `ConfToTimePlan(conf, now + T_prop)` — 將 beam ID 包成 `SatBhTimePlan`
5. **推送給 OBC** — `m_obc->ReceiveNewPlan(plan, T_prop)` — OBC 在 `now + T_prop` 開始執行 slot
6. **重新排程** — `Simulator::Schedule(T_p, RunDynamicBstpCycle, now + T_p)`

```cpp
void
SatBhHelper::RunDynamicBstpCycle(Time now)
{
    NS_ASSERT_MSG(m_dynamicProvider,
                  "SatBhHelper::RunDynamicBstpCycle called without a provider");

    if (m_metrics)
        m_metrics->FlushMetrics();

    SatDynamicBstpProvider::Conf conf = m_dynamicProvider->GetNextConf(now);

    if (!m_dynamicProvider->ValidateConf(conf))
    {
        NS_LOG_WARN("SatBhHelper::RunDynamicBstpCycle: ValidateConf failed at t="
                    << now.GetSeconds() << "s — keeping previous plan");
    }
    else if (conf.activeBeams.empty())
    {
        NS_LOG_WARN("SatBhHelper::RunDynamicBstpCycle: provider returned empty activeBeams"
                    " — keeping previous plan");
    }
    else
    {
        Time periodStart = m_obc
            ? (now + MilliSeconds(m_cfg.propagationDelayMs))
            : now;

        Ptr<SatBhTimePlan> plan = ConfToTimePlan(conf, periodStart);

        if (plan)
        {
            m_lastDynamicPlan = plan;

            if (m_obc)
                m_obc->ReceiveNewPlan(plan, MilliSeconds(m_cfg.propagationDelayMs));
            else
                m_staticPlan = plan;
        }
    }

    Time nextT = now + MilliSeconds(m_cfg.bhtpPeriodMs);
    Simulator::Schedule(MilliSeconds(m_cfg.bhtpPeriodMs),
                        &SatBhHelper::RunDynamicBstpCycle, this, nextT);
}
```

### 4.2 GetNextConf() — Greedy 排程 (`sat-greedy-bstp-provider.cc`)

**輸入：** 當前模擬時間 `now`、每個 beam 的需求狀態（`m_beams`）

```cpp
SatDynamicBstpProvider::Conf
SatGreedyBstpProvider::GetNextConf(Time now)
{
    m_cycleCount++;

    if (m_beams.empty())
    {
        Conf c; c.validityInSuperframes = m_validityInSuperframes; return c;
    }

    // Step 1 — 判斷是否有 demand
    bool anyDemand = false;
    for (const auto& kv : m_beams)
        if (kv.second.demandKbps > 0.0) { anyDemand = true; break; }

    if (!anyDemand)
        return FallbackRoundRobin();  // demand=0 → RR

    // Step 2 — 收集 starvation 強制選入 beam
    std::vector<uint32_t> forced, candidates;
    for (auto& kv : m_beams)
    {
        if (kv.second.starvationCount >= m_starvationThreshold)
            forced.push_back(kv.first);
        else
            candidates.push_back(kv.first);
    }
    if (forced.size() > m_maxActiveBeams)
        forced.resize(m_maxActiveBeams);

    // Step 3 — 為剩餘候選 beam 計算分數，依分數降序排列
    // score = backlogWeight × demandKbps + fairnessWeight × ageSec
    std::sort(candidates.begin(), candidates.end(),
              [&](uint32_t a, uint32_t b)
              { return ComputeScore(m_beams.at(a), now) > ComputeScore(m_beams.at(b), now); });

    // Step 4 — 填入 active set：forced 優先，再依分數填到 K（feeder 頻率衝突跳過）
    std::vector<uint32_t> selected = forced;
    using GwFreqKey = std::pair<uint32_t, uint32_t>;
    std::set<GwFreqKey> usedGwFreq;
    for (uint32_t bid : selected)
        usedGwFreq.insert({m_beams.at(bid).info.gwId, m_beams.at(bid).info.feederFreqId});

    for (uint32_t bid : candidates)
    {
        if (selected.size() >= m_maxActiveBeams) break;
        GwFreqKey key{m_beams.at(bid).info.gwId, m_beams.at(bid).info.feederFreqId};
        if (usedGwFreq.count(key)) continue;
        selected.push_back(bid);
        usedGwFreq.insert(key);
    }

    // Step 5 — 更新服務狀態
    std::unordered_set<uint32_t> selectedSet(selected.begin(), selected.end());
    for (auto& kv : m_beams)
    {
        if (selectedSet.count(kv.first))
            { kv.second.lastServedTime = now; kv.second.starvationCount = 0; }
        else
            kv.second.starvationCount++;
    }

    return MakeConf(selected);
}
```

**關鍵參數（預設值）：**

| 參數 | 預設值 | 作用 |
|---|---|---|
| `maxActiveBeams`（K_b） | 2 | 每個 T_p 最多同時啟動的 beam 數 |
| `backlogWeight`（W_d） | 1.0 | demand kbps 在分數中的權重 |
| `fairnessWeight`（W_f） | 0.5 | 距離上次服務時間（秒）的權重 |
| `starvationThreshold` | 5 | 連續被跳過 N 次後強制選入 |

### 4.3 分數公式

```
score_j = 1.0 × demandKbps_j + 0.5 × ageSec_j
```

- `demandKbps_j` — 由 `OnBacklogRequestTrace()` 設定（Phase F）；Phase F 未接線時 demand=0 → 退回 round-robin
- `ageSec_j = (now - lastServedTime).GetSeconds()` — 公平性項，每跳過一個 T_p 就增加

**starlink25 Phase F 情境的量化預測（satId=498）：**

```
score_hotspot  = 1.0 × demandKbps_j + 0.5 × age 
score_non-hot  = 1.0 × demandKbps_j + 0.5 × age  

差距 = x；age 需累積 ts 才能讓非 hotspot 追上 hotspot
→ 非 hotspot beam 只靠 starvationCount 在每 T 個 T_p被 forced-in 一次
→ 非 hotspot 服務頻率 ≈（per beam）
→ hotspot 服務頻率 ≈ 100%（每 T_p 都被選中）

```

### 4.4 Starvation Forced-in 機制

| | Hardcode hot cell | Starvation forced-in |
|---|---|---|
| 觸發條件 | 永遠固定 | 連續 N 個 cycle 未被選到才觸發 |
| 哪個 beam 被強制選入 | 你指定的 | 動態決定（誰未被服務就是誰） |
| 重置條件 | 永遠不重置 | 被選到後 `starvationCount` 歸零 |
| 選入頻率 | 每個 T_p | 最多每 N+1 個 T_p 一次 |

**Threshold 選擇依據：**

```
最大允許服務等待時間 ≤ N × T_p
例：T_p = 80 ms，最大等待 ≤ 400 ms → N = 400 / 80 = 5（預設值）
```

| N 較小 | N 較大 |
|---|---|
| 低需求 beam 更頻繁被強制選入 | 低需求 beam 等待時間更長 |
| 浪費更多高需求 beam 的容量 | 高需求 beam 獲得更多容量 |
| 服務延遲保證更嚴格 | 服務延遲保證更寬鬆 |

建議範圍：`N = ⌈max_wait_ms / T_p⌉`，典型值 3–10。


### 4.5 ConfToTimePlan() — `sat-bh-helper.cc`

將 `Conf { activeBeams=[b1,b2] }` 轉換為 `SatBhTimePlan`：

- 每個選入的 beam 分配到 **F = T_p / T_s = 8 個 slot**（整個週期都服務）
- Slot 的絕對時間從 `periodStart = now + T_prop` 開始偏移
- 所有選入 beam 在每個 slot 同時啟動（不做干擾分群）

```cpp
Ptr<SatBhTimePlan>
SatBhHelper::ConfToTimePlan(const SatDynamicBstpProvider::Conf& conf, Time periodStart)
{
    if (conf.activeBeams.empty()) return nullptr;

    Ptr<SatBhTimePlan> plan = CreateObject<SatBhTimePlan>();
    plan->SetPlanId(++m_dynamicPlanId);

    const Time T_s = MilliSeconds(m_cfg.slotDurationMs);
    const Time T_p = MilliSeconds(m_cfg.bhtpPeriodMs) *
                     static_cast<double>(conf.validityInSuperframes);

    plan->SetPeriodBounds(periodStart, periodStart + T_p);

    const uint32_t M = static_cast<uint32_t>(
        std::ceil(m_cfg.bhtpPeriodMs * conf.validityInSuperframes / m_cfg.slotDurationMs));
    const uint32_t K = m_cfg.maxActiveBeams;

    const std::vector<uint32_t>& beams = conf.activeBeams;
    const uint32_t               B     = static_cast<uint32_t>(beams.size());

    // Distribute activeBeams across M slots using round-robin.
    // Example: activeBeams={1,4,7}, K=2, M=3 → slot0:{1,4}, slot1:{7,1}, slot2:{4,7}
    uint32_t beamOffset = 0;
    for (uint32_t s = 0; s < M; s++)
    {
        BhSlotEntry slot;
        slot.startTime = s * T_s;
        slot.duration  = T_s;
        slot.modcod    = 5;

        uint32_t count = std::min(K, B);
        for (uint32_t k = 0; k < count; k++)
        {
            uint32_t bid = beams[(beamOffset + k) % B];
            slot.beamIds.push_back(bid);
            slot.clusterIds.push_back(bid);
            slot.SetBeamPattern(bid, (k == 0) ? BeamRadiusType::SMALL : BeamRadiusType::LARGE);
        }
        beamOffset = (beamOffset + 1) % B;
        plan->AddSlot(slot);
    }

    if (!plan->Validate(K))
        NS_LOG_WARN("SatBhHelper::ConfToTimePlan: Validate() failed for planId="
                    << plan->GetPlanId());

    return plan;
}
```

### 4.6 OBC Slot 執行 — `sat-bh-obc.cc`

每個 slot（T_s = 10 ms）由三個函式串聯：

```cpp
void SatBhObc::EnterSlot(int32_t slotIdx)
{
    // 進入 ACTIVE，發出 BeamActivateCallback，排程 OnSlotServiceEnd(usable=8ms 後)
    m_state = ObcState::ACTIVE;
    const BhSlotEntry& slot = m_activePlan->GetSlots()[slotIdx];
    Time usableDur = slot.duration - m_switchingTime;

    if (m_activateCb)
        for (uint32_t beamId : slot.beamIds)
            m_activateCb(m_i, beamId, usableDur);

    Simulator::Schedule(usableDur, &SatBhObc::OnSlotServiceEnd, Ptr<SatBhObc>(this), slotIdx);
}

void SatBhObc::OnSlotServiceEnd(int32_t slotIdx)
{
    // 可用窗口結束：發出 BeamDeactivateCallback，進入 SWITCHING，排程 OnSwitchingDone(T_sw 後)
    if (m_deactivateCb)
        for (uint32_t beamId : slot.beamIds)
            m_deactivateCb(m_i, beamId, usedDur);

    m_state = ObcState::SWITCHING;
    Simulator::Schedule(m_switchingTime, &SatBhObc::OnSwitchingDone, Ptr<SatBhObc>(this), slotIdx + 1);
}

void SatBhObc::OnSwitchingDone(int32_t nextSlotIdx)
{
    // (a) 還有 slot → EnterSlot(next)
    // (b) 有排隊的 pending plan → 切換並 EnterSlot(0)
    // (c) 無計畫 → WAIT_PLAN
    if (m_activePlan && nextSlotIdx < m_activePlan->GetNumSlots())
        EnterSlot(nextSlotIdx);
    else if (m_pendingPlan)
    {
        m_activePlan = m_pendingPlan; m_pendingPlan = nullptr;
        Time delay = (planStart > now) ? (planStart - now) : Seconds(0.0);
        Simulator::Schedule(delay, &SatBhObc::EnterSlot, Ptr<SatBhObc>(this), 0);
    }
    else
        m_state = ObcState::WAIT_PLAN;
}
```

---

## 5. 驗證結果

### 5.1 CSV 欄位說明

```
bh-metrics_<scenario>_<YYYYMMDD>_<HHMMSS>.csv
```

| 欄位 | 單位 | 資料來源 |
|---|---|---|
| `time_s` | s | FlushMetrics 呼叫時的 `Simulator::Now()` |
| `sat_id` | — | Helper satellite 索引（0-indexed） |
| `beam_id` | — | 1-indexed beam ID（SNS3 慣例） |
| `throughput_mbps` | Mbps | `rxBytes × 8 / windowSec / 1e6` |
| `avg_delay_ms` | ms | `totalDelayMs / pktCount` |
| `max_delay_ms` | ms | 該視窗內最大封包延遲 |
| `dwell_time_ms` | ms | 該 T_p 視窗內 beam 累積開啟的可用時間 |
| `slot_util_pct` | % | `slotUsedMs / slotAllocMs × 100` |
| `drop_rate_pct` | % | `droppedPkts / totalOfferedPkts × 100` |
| `jain_fairness_index` | — | 所有 active beam 的 Jain 公平性指數，flush 時計算 |

### 5.2 行數結構

`numPeriods=N, maxHelperSats=H, warmUp=0` 的預期行數：

```
預期行數 ≈ N × (每個 T_p 每顆衛星的 active 唯一 beam 數) × H

starlink25，K_b=2，RR fallback：
  ≈ N × 2 beams/sat × 10 sats = N × 20 行（資料）
  + 1 行 header
```

**線性比例驗證：** `rows(numPeriods=20) / rows(numPeriods=10) ≈ 2.0 ✓`

2026-06-23 實測：numPeriods=10 → 380 data rows；numPeriods=20 → 780 data rows；比例 2.05 ✓

### 5.3 欄位含義

#### `dwell_time_ms`

```
T_p=80ms，F=8 slots，T_s=10ms，T_sw=2ms，usable=8ms

beam 在全部 8 個 slot 都被啟動：dwell_time_ms = 8 × 8ms = 64ms
beam 只在 4 個 slot 被啟動：   dwell_time_ms = 4 × 8ms = 32ms
```

RR fallback（demand=0）情境下，K_b=2 個 beam 各佔全部 8 個 slot → `dwell_time_ms ≈ 64 ms`

#### `slot_util_pct`

- `slotAllocMs` = 分配給此 beam 的總可用時間（由 `BeamActivateCallback` 累積）
- `slotUsedMs` = 實際使用的時間（由 `BeamDeactivateCallback` 累積）
- 無封包流量（demand=0）情境：`slot_util_pct = 0.0%`，`dwell_time_ms = 64ms`

#### `throughput_mbps` 與 `avg_delay_ms`

只有在 `SatBhMetrics::OnPacketReceived()` 被呼叫時才有數值（來自封包 trace）。  
Phase G + demand=0，或 Phase G + Phase F（Phase F 停用 synthetic injection）情境下：均為 0。  
真實 throughput 請查閱 `data/` 目錄中 SNS3 原生統計資料。

#### `jain_fairness_index`

flush 時依 `m_kpiTable` 中所有 beam 的 throughput 計算。  
所有 throughput 為 0 時回傳 1.0（退化情況，視為完全公平）。


### 5.4 Hotspot 服務品質分析（dwell_time 代理指標）

目前 `throughput_mbps` 全部為 0（`SatBhMetrics::OnPacketReceived()` 尚未串接真實 SNS3 封包 trace）。`dwell_time_ms` 是目前唯一能直接從 CSV 取得、且能反映**排程器決策**的欄位。

**資料結構前提：**
- 只有 `satId=498` 有物理意義（唯一有真實 UT 的衛星，starlink25 情境）
- `satId=490–497, 499`：`BuildBeamToggleMap()` 找不到可控裝置，無 RBDC trace 連接，Phase G 永遠退回 RR fallback
- Hotspot beam IDs：**{1, 4, 13, 19, 22}**；Non-hotspot：**{2,3,5–12,14–18,20,21,23–25}**

| 面向 | satId=498 | satId=490–497,499 |
|---|---|---|
| 物理 beams | 25 個真實 beams（starlink25 UT） | 無（虛擬） |
| Phase F RBDC | 連接真實 RBDC trace，demand=141~480 kbps | 無 trace 連接，demand=0 |
| Phase G 排程 | demand-based，hotspot {1,4,13,19,22} 優先 | RR fallback，永遠循環 |
| OBC ToggleState | 真實切換 GW SatNetDevice | 無物理效果（無映射裝置） |

**直接觀察（warmUp 後最初幾個 T_p，satId=498）：**

| time_s | sat_id | beam_id | dwell_time_ms | 角色 |
|--------|--------|---------|--------------|------|
| 10.080 | 498 | 1 | **56** | hotspot current |
| 10.080 | 498 | 4 | **56** | hotspot current |
| 10.080 | 498 | 19 | 8 | hotspot carry-over |
| 10.080 | 498 | 22 | 8 | hotspot carry-over |
| 10.160 | 498 | 13 | **56** | hotspot current |
| 10.160 | 498 | 22 | **56** | hotspot current |

warmUp 結束後（t=10s），satId=498 的 current beam（dwell=56ms）全部來自 hotspot set {1,4,13,19,22}，非 hotspot beam 在此時段**完全未出現**。


**分析腳本**（存放：`Beam Hopping Controller/Outputs/bh-dynamic_phaseG/bh_dwell_analysis.py`）：

```python
"""
用 dwell_time_ms 分析 hotspot vs non-hotspot beam 服務分配。
只分析 satId=498（唯一有物理意義的衛星，starlink25 情境）。
執行：python bh_dwell_analysis.py bh-metrics_starlink25_*.csv
"""
import sys
import pandas as pd
import matplotlib.pyplot as plt

HOTSPOT_BEAMS = {1, 4, 13, 19, 22}
TARGET_SAT    = 498

def main(csv_path):
    df = pd.read_csv(csv_path)
    df498 = df[df["sat_id"] == TARGET_SAT].copy()
    if df498.empty:
        print(f"No rows for sat_id={TARGET_SAT}."); return

    df498["beam_type"] = df498["beam_id"].apply(
        lambda b: "hotspot" if b in HOTSPOT_BEAMS else "non-hotspot")

    beam_stats = (df498.groupby(["beam_id", "beam_type"])
                       .agg(count=("dwell_time_ms", "count"),
                            total_dwell=("dwell_time_ms", "sum"),
                            mean_dwell=("dwell_time_ms", "mean"))
                       .reset_index()
                       .sort_values("total_dwell", ascending=False))
    print("\n=== Per-beam Statistics (sat_id=498) ===")
    print(beam_stats.to_string(index=False))

    summary = (df498.groupby("beam_type")
                    .agg(total_rows=("dwell_time_ms", "count"),
                         total_dwell=("dwell_time_ms", "sum"),
                         mean_dwell=("dwell_time_ms", "mean"))
                    .reset_index())
    print("\n=== Hotspot vs Non-hotspot Summary (sat_id=498) ===")
    print(summary.to_string(index=False))

    n_hot, n_cold = len(HOTSPOT_BEAMS), 25 - len(HOTSPOT_BEAMS)
    hot_row  = summary[summary["beam_type"] == "hotspot"].iloc[0]
    cold_row = summary[summary["beam_type"] == "non-hotspot"]
    if not cold_row.empty and cold_row.iloc[0]["total_rows"] > 0:
        cold_row = cold_row.iloc[0]
        print(f"\n服務頻率比 hotspot/non-hotspot = {(hot_row['total_rows']/n_hot)/(cold_row['total_rows']/n_cold):.2f}x")
        print(f"總 dwell 比  hotspot/non-hotspot = {(hot_row['total_dwell']/n_hot)/(cold_row['total_dwell']/n_cold):.2f}x")
    else:
        print("\nnon-hotspot beam 未出現（排程完全偏向 hotspot）")

    # current beam：dwell_time_ms >= 48 (= 6 slots × 8ms)，排除 carry-over 的 8ms
    df498["is_current"] = df498["dwell_time_ms"] >= 48
    current = df498[df498["is_current"]]
    per_period = (current.groupby("time_s")
                         .agg(total_current=("beam_id", "count"),
                              hotspot_current=("beam_type", lambda x: (x == "hotspot").sum()))
                         .reset_index())
    per_period["hotspot_pct"] = per_period["hotspot_current"] / per_period["total_current"] * 100
    print(f"\nMean hotspot% in active selection: {per_period['hotspot_pct'].mean():.1f}%")
    print(f"(Random baseline: {n_hot/25*100:.1f}%,  K=2 all-hotspot ceiling: 100%)")

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    colors = ["tab:orange" if b in HOTSPOT_BEAMS else "tab:blue" for b in beam_stats["beam_id"]]
    axes[0].barh(beam_stats["beam_id"].astype(str), beam_stats["total_dwell"], color=colors)
    axes[0].set_xlabel("Total Dwell Time (ms)")
    axes[0].set_title("Per-beam Total Dwell Time\n(orange=hotspot, sat_id=498)")
    axes[1].plot(per_period["time_s"], per_period["hotspot_pct"], alpha=0.7)
    axes[1].axhline(n_hot/25*100, color="gray", linestyle="--", label=f"random ({n_hot/25*100:.0f}%)")
    axes[1].set_xlabel("Time (s)"); axes[1].set_ylabel("Hotspot beams in active selection (%)")
    axes[1].set_title("Hotspot Selection Rate per T_p (sat_id=498)")
    axes[1].legend(); axes[1].set_ylim(0, 110)
    plt.tight_layout()
    out = csv_path.replace(".csv", "_dwell_analysis.png")
    plt.savefig(out, dpi=150)
    print(f"\nFigure saved: {out}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python bh_dwell_analysis.py <bh-metrics_starlink25_*.csv>"); sys.exit(1)
    main(sys.argv[1])
```

---

### 5.5 驗證紀錄

#### 2026/06/23 — Phase G starlink25 全面驗證（simTime=120s）

```bash
./ns3 run "sat-bh-example --scenario=starlink25 \
  --enableObc=1 --enableDynamicBstp=1 \
  --simTime=120" \
  2>&1 | tee bh_run_0623.log
```

| 項目 | 預期 | 實測 | 結果 |
|------|------|------|------|
| 總行數 | 54,961 | 54,961 | ✓ |
| 唯一 timestamps（T_p 數） | 1,374 | 1,374 | ✓ |
| rows / T_p | 40 | 40 | ✓ |
| 10 sats × (2 current + 2 carry-over) | 40 | 40 | ✓ |
| 時間涵蓋（warmUp=10s） | 10s – ~120s | 10s – 119.92s | ✓ |

Row count 計算：`1374 × 40 + 1 header = 54,961`；40 rows/T_p = 10 helper sats × 4 beams（2 current + 2 carry-over）。

**結論：Phase G row count fix 通過全面驗證。** 詳細過程見 [`Logs/0623_metrics-rowcount-fix.md`](../../Logs/0623_metrics-rowcount-fix.md)。

#### 2026/06/23 — Phase G + Phase F starlink25 120s 完整執行

```bash
./ns3 run "scratch/bh_dynamic/Codes/sat-bh-example \
  --scenario=starlink25 --enableObc=1 --enableDynamicBstp=1 \
  --enablePhaseF=1 --maxActiveBeams=2 --simTime=120" \
  2>&1 | tee bh_phaseg_f.log
```

啟用旗標：`enableObc=1, enableDynamicBstp=1, enablePhaseF=1`；其餘 Phase C/D/E/Scheduler 均關閉。

**關鍵觀察：**

- `t < 1s`：所有 beam demand=0 → RR fallback，每 T_p 選 {1,2}→{3,4}→...
- `t = 1s`：Phase F RBDC 開始到達（UT 每秒一次 request）；hotspot boost 每 80ms 呼叫 `UpdateBeamDemand({1,4,13,19,22}, 480)`
- `t > 1s`（約第 13 個 T_p 後）：demand ≠ 0 → score-based 選擇啟動；satId=498 開始以 hotspot beam 為主

**Phase F demand 設定：**

```
hotspot UT demand boost：480 kbps（每 80ms 覆寫）
RBDC return link demand：~141 kbps/UT
注意：UpdateBeamDemand() 為覆寫，兩者以最近一次呼叫為準
```

**CSV 輸出（bh-metrics）：**

| 欄位 | 值 | 說明 |
|---|---|---|
| `throughput_mbps` | 0.000000 | Phase F 停用 synthetic injection |
| `dwell_time_ms` | 56.000000 / 8.000000 | current / carry-over beam |
| `slot_util_pct` | 100.000000 | beam 有開啟，無論有無封包 |
| `jain_fairness_index` | 1.000000 | 所有 throughput=0 → 退化為 1.0 |

真實吞吐量請查 `data/` 目錄中 SNS3 原生統計。

---

## 已知限制與 Gap

| # | 問題 | 現況 | 影響 |
|---|---|---|---|
| 1 | **Starvation 機制在 demand=0 時不觸發** | Phase F 未接線時所有 demand=0 → `FallbackRoundRobin()`，`starvationCount` 永遠不遞增 | 純 RR fallback 情境無法驗證 forced-in 邏輯 |
| 2 | **throughput_mbps 永遠為 0** | `SatBhMetrics::OnPacketReceived()` 尚未串接真實 SNS3 封包 trace（需要 Phase E） | dwell_time 是目前唯一可用的排程決策代理指標 |
| 3 | **Phase F demand 覆寫問題** | RBDC（~141 kbps）與 hotspot boost（480 kbps）共用 `UpdateBeamDemand()`（覆寫，非累加） | non-hotspot beam 的實際 demand 值需從 log 逐行確認 |
| 4 | **satId=490–497,499 無物理效果** | 10 個 helper sats 中只有 satId=498 有真實 UT；其餘 `BuildBeamToggleMap()` 找不到可控裝置 | 這些 helper sats 的 metrics 只驗證 OBC/Metrics 狀態機，不代表實際 beam 服務 |
| 5 | **SMALL/LARGE beam pattern 無物理效果** | `SetBeamPattern()` 僅寫入 TimePlan 資料結構，OBC 只讀 `beamIds + duration` | 目前僅為殘留 metadata，不影響 antenna pattern、SNS3 物理層或分數計算 |
| 6 | **bh-timeplan CSV 僅記錄靜態備援計畫** | `Install()` 時寫入靜態 fallback BHTP；Phase G 動態計畫不寫入 CSV | 無法從 CSV 直接追蹤每個 T_p 的動態選擇結果，需透過 log 或另行 trace |
