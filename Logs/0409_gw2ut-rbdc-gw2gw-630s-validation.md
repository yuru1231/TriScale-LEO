# 工作日誌 2026-04-09

## 目標
修正 GW Preset 越界錯誤；以 GW2UT+RBDC trace（120s）與 GW2GW_DIRECT 長時驗證（630s/11 slots）確認 IslRoutingManager 在真實流量場景下的動態路由穩定性與資料平面完整性；同步更新 Layer 1 README 。

---

## 完成事項

### 1. GW2GW 長時驗證（630s / 11 slots）

**測試指令**：`--mode=gw2gw_direct --trafficProfile=gw2gw_direct --simTime=630 --gwSrc=0 --gwDst=2`

**現象**：執行 TW→USA 端到端 CBR 資料平面驗證。確認動態路由在完整時間窗口內的切換行為與封包送達，資料平面在切換期間無中斷。

**驗證**：

ISL 路由切換記錄（TW→USA，11 slots）：

| slot | time(s) | ISL path | isl_cost(s) | 備註 |
|------|---------|----------|-------------|------|
| 0 | 0 | 15→14→25→36→37 | 0.044 | |
| 1 | 60 | 15→14→25→36→37 | 0.046 | |
| 2 | 120 | 15→14→13→2→1 | 0.048 | ROUTE CHANGED |
| 3 | 180 | 15→14→25→36 | 0.038 | ROUTE CHANGED |
| 4 | 240 | 15→14→25→36 | 0.040 | |
| 5 | 300 | 15→14→25→36 | 0.042 | |
| 6 | 360 | 44→45→46→35→36 | 0.044 | ROUTE CHANGED |
| 7 | 420 | 14→13→24→35→36 | 0.040 | ROUTE CHANGED |
| 8 | 480 | 14→13→24→35→36 | 0.042 | |
| 9 | 540 | 44→45→56→1→0 | 0.042 | ROUTE CHANGED |
| 10 | 600 | 44→45→56→1→0 | 0.044 | |

- 活躍 ISL：79 條，最高負載 sat41→52 = 1.1719ms
- `PacketSink::GetTotalRx() = 3,214,848 bytes`（約 6279 pkts）**[PASS]**

---

### 2. GW2UT + RBDC Trace 驗證（120s / 3 slots）

**測試指令**：`--mode=gw2ut --trafficProfile=gw2ut_app --simTime=120`
GW0（TW-Taipei）→ UT1（US-SanFrancisco）

**現象**：91 UT 服務流量（FWD 120kbps/flow，RTN 8.192kbps/flow）。RBDC trace 路徑 `SatLlc/SatRequestManager/RbdcTrace` 成功觸發並有輸出。

**修正**：`test-iridium.cc` 目前對 RBDC trace 沒有頻率限制或 sample-only 過濾機制。需在 RBDC callback 中加入頻率限制，控制輸出量。

**驗證**：
- RBDC trace 路徑可正常掛接，callback 有被觸發（輸出非空）
- ISL 路由路徑：
  - Slot 0～1（t=0～60s）：`15→14→25→36→37`（serving sat=37），isl_cost ≈ 0.044～0.046s
  - Slot 2（t=120s）：`15→14→13→2→1`（serving sat=1），isl_cost = 0.047600s（ROUTE CHANGED）
- 活躍 ISL：19 條，最高負載 sat54→65 = 0.4517ms
- Wall time：560.9s

---

### 4. GW2UT 630s 長時驗證

**修正**：在 `test-iridium.cc` 新增 `--rbdcVerbose` flag（預設 `false`）。當 `rbdcVerbose=false` 時，不輸出任何 RBDC log，避免洗版。

```cpp
// test-iridium.cc — 新增 flag
cmd.AddValue("rbdcVerbose", "Enable per-callback RBDC trace output (default: false)", rbdcVerbose);

// RBDC callback 內部加入判斷
if (rbdcVerbose) {
    NS_LOG_UNCOND("RBDC trace: UT=" << utId << " rate=" << rateKbps << "kbps");
}
```

**驗證**：以 `--mode=gw2ut --trafficProfile=gw2ut --simTime=630 --gwId=0 --utId=1 --utLatDeg=37.8 --utLonDeg=-122.4 --utName=UT-SanFrancisco` 執行，輸出存於 `Topology & ISL Routing/Outputs/gw2ut_630sec.md`。

路由路徑（11 slots，5 次路由切換）：

| slot | time(s) | entry | ISL path | serving | 備註 |
|------|---------|-------|----------|---------|------|
| 0 | 0 | 15 | 15→14→25→36→37 | 37 | |
| 1 | 60 | 15 | 15→14→25→36→37 | 37 | |
| 2 | 120 | 15 | 15→14→13→2→1 | 1 | ROUTE CHANGED |
| 3 | 180 | 15 | 15→14→25→36 | 36 | ROUTE CHANGED |
| 4 | 240 | 15 | 15→14→25→36 | 36 | |
| 5 | 300 | 15 | 15→14→25→36 | 36 | |
| 6 | 360 | 44 | 44→45→46→35→36 | 36 | ROUTE CHANGED |
| 7 | 420 | 14 | 14→13→24→35→36 | 36 | ROUTE CHANGED |
| 8 | 480 | 14 | 14→13→24→35→36 | 36 | |
| 9 | 540 | 44 | 44→45→56→1→0 | 0 | ROUTE CHANGED |
| 10 | 600 | 44 | 44→45→56→1→0 | 0 | |

120s vs 630s 對比：

| 項目 | 120s | 630s |
|------|------|------|
| Slots | 3 | 11 |
| ROUTE CHANGED 次數 | 1 | 5 |
| 活躍 ISL links | 19 / 132 | 95 / 132 |
| 最高 ISL load | 0.4517ms (sat54→65) | 1.6273ms (sat30→41) |
| Wall time | 560.9s | 3874.5s |

路徑與 `gw2gw_direct_tw2usa_630s.md` 完全一致：兩者同為 TW-Taipei → SanFrancisco 方向，幾何相同，路由結果互相驗證通過。

---

### 5. 更新 Layer 1 README（Layer1.md）

**修正**：對 `Topology & ISL Routing/Layer1.md` 進行以下更新：

1. **架構流程圖**：在 `CreateSatScenario()` 後新增 `ConnectIslDropTrace()` 步驟；在 `RunSimulation()` 後新增 `PrintLoadStats()` 和 `PrintIslDropStats()` 步驟。
2. **核心資料結構**：拆分為 `isl-graph.h 定義` 與 `test-iridium.cc 驗證輔助` 兩子區段；新增 `IslDropStats`、`g_nodeToSatId`、`g_islDropStats` 說明。
3. **命令列參數**：新增 `test-iridium.cc 命令列參數` 表格，含 `rbdcVerbose` 和 `islDropThreshPct`。
4. **GW Preset 對照表**：明確標示只有 0/1/2 三個有效值。
5. **Public Methods**：重組為五大類別（Lifecycle、統計輸出、路由查詢/診斷、v6 GW-to-GW API、v7 GW-to-UT API）。
6. **新增ISL 驗證機制**：說明 `ConnectIslDropTrace()`、`IslPacketDropCallback()`、`PrintIslDropStats()` 行為。
7. **FtVisibilityFilter 備注**：新增「已實作但 test-iridium.cc 尚未整合使用」狀態標記。
8. **驗證基準與版本表**：新增 `trafficProfile=gw2gw_direct（TW-Taipei → US-SanFrancisco，630s）` 實際量測結果
---

## 問題

### 1. RBDC log 輸出冗長
- 狀態：**已解決**
- 方案：新增 `--rbdcVerbose` flag，預設關閉 RBDC callback 輸出

### 2. GW2GW 630s Wall time 偏長（2803s）
- 狀態：**暫緩**

### 3. GW2UT 驗證時長不足
- 狀態：**已完成**
- 結果：630s / 11 slots / 5 次路由切換，與 gw2gw 路徑一致，驗證通過

---

## 新增 / 修改的程式碼

### test-iridium_baseline.cc — ISL Drop Rate 驗證機制（全新新增）

**新增位置**：anonymous namespace（`main()` 之前）

```cpp
// ── 資料結構 ───────────────────────────────────────────
struct IslDropStats {
    uint64_t total{0};    // 嘗試傳送的封包總數（含成功與丟棄）
    uint64_t dropped{0};  // 被丟棄的封包數
};

static std::map<Ptr<Node>, uint32_t>       g_nodeToSatId;   // Node → satId 查找表
static std::map<std::string, IslDropStats> g_islDropStats;  // key = "srcId-dstId"

// ── Callback ───────────────────────────────────────────
// PacketDropRateTrace 每次 PointToPointIslNetDevice 入列嘗試時觸發
static void IslPacketDropCallback(uint32_t /*pktSize*/,
                                  Ptr<Node> srcNode, Ptr<Node> dstNode, bool dropped);

// ── ConnectIslDropTrace() ──────────────────────────────
// 在 CreateSatScenario() 之後呼叫；建立 g_nodeToSatId，掛接所有 ISL 的 trace
// 回傳成功掛接的介面數（每條雙向 link = 2 個介面，unique link 數 = 回傳值 / 2）
static uint32_t ConnectIslDropTrace();

// ── PrintIslDropStats() ────────────────────────────────
// 輸出丟棄率明細與 PASS/FAIL 判定（門檻 = islDropThreshPct，預設 1.0%）
// 特殊 FAIL 情況：
//   - connectedInterfaces == 0 → trace connection failed
//   - stats empty but interfaces > 0 → 0 events recorded（無流量通過 ISL）
static void PrintIslDropStats(double threshPct, uint32_t connectedInterfaces);
```

**呼叫點（main() 內）**：
```cpp
// 在 CreateSatScenario() 之後
uint32_t islConnected = ConnectIslDropTrace();

// 在 RunSimulation() + PrintLoadStats() 之後
PrintIslDropStats(islDropThreshPct, islConnected);
```

---

### test-iridium_baseline.cc — RBDC Trace 控制機制（新增）

**新增命令列參數**：
```cpp
bool   rbdcVerbose     = false;  // 預設靜默，避免 91 UT × 26ms 週期大量 log
double islDropThreshPct = 1.0;   // ISL 整體丟棄率 PASS 門檻（%）

cmd.AddValue("rbdcVerbose",      "Print each RBDC request (1=on, 0=off, default=0)", rbdcVerbose);
cmd.AddValue("islDropThreshPct", "ISL overall drop rate PASS threshold (%, default=1.0)", islDropThreshPct);
```

**RBDC trace 掛接邏輯**：
```cpp
if (rbdcVerbose) {
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/SatLlc/SatRequestManager/RbdcTrace",
        MakeCallback(&RbdcTraceCallback));
} else {
    std::cout << "[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)\n";
}
```

設計原因：RBDC 每 26ms 觸發一次，91 UT × 630s = 約 221,000 筆 log，預設關閉避免洗版。需要觀察 RBDC 行為時加 `--rbdcVerbose=1` 即可。

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `Topology & ISL Routing/Codes/test-iridium_baseline.cc` | 新增 ISL Drop Rate 驗證機制（`IslDropStats`、`ConnectIslDropTrace()`、`IslPacketDropCallback()`、`PrintIslDropStats()`）；新增 `--rbdcVerbose` 與 `--islDropThreshPct` 命令列參數；`--gwDst=3` 改為 `--gwDst=2` |
| `Topology & ISL Routing/Layer1.md` | 更新架構流程圖、資料結構、GW Preset 對照表、Public Methods 分類、新增 ISL 驗證機制章節與驗證基準 |
| `Topology & ISL Routing/Outputs/gw2ut_rbdc.md` | 新增 GW2UT+RBDC 120s 驗證輸出記錄 |
| `Topology & ISL Routing/Outputs/gw2gw_direct_tw2usa_630s.md` | 新增 GW2GW TW→USA 630s 驗證輸出記錄（11 slots，5 次路由切換） |
| `Topology & ISL Routing/Outputs/gw2ut_630sec.md` | 新增 GW2UT TW→USA 630s 驗證輸出記錄（11 slots，5 次路由切換） |

---

## Layer 1 驗證里程碑總表

| 項目 | 狀態 |
|------|------|
| ISL 拓樸 132 條邊建立 | 完成 |
| Dijkstra 路由 + slot 更新 | 完成 |
| entry/exit 衛星仰角篩選（5°） | 完成 |
| GW-to-GW 控制平面路由報告 | 完成 |
| GW_user→GW_user 封包端到端送達 | 完成（0408） |
| 跨洲多跳 ISL（TW→USA 4 跳） | 完成（0408） |
| 動態 routing 切換不中斷流量 | 完成（0408） |
| TW→JP 630s 動態路由（11 slots） | 完成（0408） |
| TW→USA 630s 動態路由（11 slots，5 次切換） | 完成（今日） |
| GW2UT + RBDC trace 掛接確認 | 完成（今日，路徑驗證通過） |
| GW2UT 630s 長時驗證 | 完成（今日） |
| RBDC log 輸出控制 | 完成（今日，`--rbdcVerbose` flag） |
| ISL Drop Rate 驗證機制實作 | 完成（今日，`ConnectIslDropTrace` + `PrintIslDropStats`） |
| ISL load-aware rerouting（HOL delay） | 待實作 |

---

## 明日計畫

- 以修正後指令（`--gwDst=2`）重新執行模擬，確認 `PrintIslDropStats` 輸出格式與門檻過濾行為
- 調查 gw2gw 630s Wall time 2803s 原因（P2：疑似 SNS3 DVB MAC beam scheduler 所致，待確認）
- 評估是否可進入 Layer 2 Beam Hopping 設計階段（前提：Layer 1 驗證基準已穩定）
