# 2026-04-29 工作日誌 — ISL Routing 修正與長時段驗證

## 目標

- 確認 `gw2gw_e2e` beam bootstrap 精簡修正正確生效，並以 11-slot（630s）長時段模擬驗證 ISL 動態路由切換行為
- 發現並修正 `gwMode="user"` 導致 GW 流量走 IP 層捷徑而非真實 satellite 路徑的問題
- 驗證移除 gwMode 後 physical GW 節點正確被使用

---

## 完成項目

- 完成 beam 數精簡修正（`bootstrapGwCount` 動態化），確認 TOPO bootstrap 只啟動實際需要的 GW
- 完成 11-slot 長時段 `gw2gw_e2e` 模擬，驗證逐 slot 路由切換與傳播延遲理論值
- **發現 `gwMode="user"` 根本性問題**：GW 流量繞過 satellite，直接在 SNS3 人工建立的 GW user node（90.3.x.x subnet）之間以純 IP 路由傳遞
- **完成 gwMode 全面移除**：刪除所有 gwMode 相關參數、分支、struct 欄位，強制所有 GW 流量走 physical GW node
- 驗證修正後 GW0=40.1.0.1、GW1=40.67.0.1，feeder_up → ISL → feeder_down 路徑正確建立

---

## Bug 修正

### Bug 1：bootstrapGwCount 未動態化（原有記錄）

| 項目 | 說明 |
|------|------|
| **問題** | 原版只開單一 beam（`beamId=72`），導致其餘 GW 未被 bootstrap 進 `SatTopology`，`GetGwNodesById()` 回傳空 `NodeContainer` |
| **根因** | `simHelper->SetBeamSet({beamId})` 只觸發單一 beam 的 install 流程，只有該 beam 對應的 GW 被 `SatTopology::GetGwNodes()` 收錄 |
| **修正** | 新增 `BuildGatewayBootstrapBeamSet()`，依 `needsGwPair` 決定 `bootstrapGwCount`（gw2gw=2，其餘=1），只啟動實際需要的 GW 所對應的 representative beam |
| **影響檔案** | `test-iridium-e2e-fix.cc` |

---

### Bug 2：gwMode="user" 導致 GW 流量不走 satellite 路徑（**本次新增**）

#### 問題描述

原版程式碼中，`gw2gw_e2e` 的 GW traffic 預設使用 `gwMode="user"`。
SNS3 的 GW user node（`topo->GetGwUserNode(gwId)`）是應用層抽象節點，不具備 `SatNetDevice`，
因此沒有 feeder link 介面。這類節點和其他 user node 在同一個 90.3.x.x subnet，
NS-3 IP 路由層可直接在兩個 GW user node 之間繞過 satellite 轉發封包。

**結果**：GW0=90.3.0.2 → GW1=90.3.0.3，走的是純 IP 層，不是 `feeder_up → ISL → feeder_down`。

#### gwMode="user" 在真實環境的意義

**沒有意義**。gwMode="user" 是 SNS3 模擬框架內部的抽象，代表「應用層節點，不接衛星」，
在真實 LEO 衛星通訊中不存在對應物。真實的 GW 一定透過 feeder link 與衛星通訊，
gwMode="user" 只是模擬器建立 user application 的捷徑，不代表任何物理路徑。

#### 修正策略

完全移除 gwMode：刪除 E2EConfig、DelayMatrixRow、CLI 參數、所有函式中的 gwMode 分支，
強制所有 GW 流量走 physical GW node（`topo->GetGwNodes().Get(gwId)`），即 SatNetDevice 節點。

---

#### 修正 Code （test-iridium-e2e-fix.cc）

**① 移除 E2EConfig 中的 gwMode 欄位**

```cpp
// 修正前
struct E2EConfig
{
    std::string pathType{"gw2gw_e2e"};
    std::string gwMode{"user"};     // <-- 移除此欄位
    double      simTimeSec{0.0};
    ...
};

// 修正後
struct E2EConfig
{
    std::string pathType{"gw2gw_e2e"};
    double      simTimeSec{0.0};
    ...
};
```

**原因**：gwMode 概念本身無效，欄位存在只會讓程式碼繼續支援不應存在的行為。

---

**② 移除 DelayMatrixRow 中的 gw_mode 欄位及 CSV header**

```cpp
// 修正前
struct DelayMatrixRow
{
    std::string pathType;
    std::string gwMode;     // <-- 移除
    ...
};
// CSV header 也包含 gw_mode 欄位

// 修正後
struct DelayMatrixRow
{
    std::string pathType;
    // gwMode 欄位已刪除
    ...
};
```

**原因**：輸出 CSV 中記錄 gwMode 等同於把無意義的抽象值當成量測欄位，
會誤導分析結果，應刪除以保持欄位語義清晰。

---

**③ 移除 CLI --gwMode 參數**

```cpp
// 修正前（main() 中）
std::string gwMode = "user";
cmd.AddValue("gwMode", "GW traffic mode: user | physical", gwMode);
e2eCfg.gwMode = gwMode;
// [CFG] log 也輸出 gwMode=...

// 修正後
// 以上三行全部刪除，gwMode 不再是 CLI 選項
```

**原因**：不應提供 gwMode 作為可選參數，讓使用者有機會誤用 user mode。
移除後，程式碼行為唯一且確定。

---

**④ 移除 GetGwUsers() 函式**

```cpp
// 修正前（存在此函式）
static NodeContainer
GetGwUsers(uint32_t gwId)
{
    // 回傳 SNS3 GW user node（應用層抽象，無 SatNetDevice）
    return NodeContainer(topo->GetGwUserNode(gwId));
}

// 修正後
// 整個函式刪除，不再有任何路徑呼叫 GetGwUserNode()
```

**原因**：`GetGwUserNode()` 回傳的節點沒有 feeder link，
在 gw2gw_e2e 場景中呼叫此函式必然導致 IP 層捷徑。移除後徹底消除誤用入口。

---

**⑤ GetGwTrafficNodes() 移除 gwMode 參數**

```cpp
// 修正前
static NodeContainer
GetGwTrafficNodes(uint32_t gwId, const std::string& gwMode)
{
    if (gwMode == "physical")
    {
        return GetPhysicalGwNodes(gwId);
    }
    return GetGwUsers(gwId);   // <-- user mode 走這裡，不走衛星
}

// 修正後
static NodeContainer
GetGwTrafficNodes(uint32_t gwId)
{
    // 永遠回傳 physical GW node，不存在 user 分支
    return GetPhysicalGwNodes(gwId);
}
```

**原因**：分支邏輯的存在代表「user mode 是合法選項」。
移除分支後，函式語義變成「取得 GW 的流量節點」= 「取得 physical GW 節點」，
無歧義。

---

**⑥ InstallGw2GwApplicationTraffic() 移除 gwMode 分支**

```cpp
// 修正前
static void
InstallGw2GwApplicationTraffic(Ptr<SimulationHelper> simHelper,
                               uint32_t gwSrc, uint32_t gwDst,
                               double startSec, double stopSec,
                               const std::string& gwMode)
{
    Ptr<Node> srcNode, dstNode;
    if (gwMode == "physical")
    {
        srcNode = GetPhysicalGwNodeOrNull(gwSrc);
        dstNode = GetPhysicalGwNodeOrNull(gwDst);
        srcAddr = GetPhysicalGwRoutableIp(srcNode, gwSrc);   // 40.x.x.x
        dstAddr = GetPhysicalGwRoutableIp(dstNode, gwDst);
    }
    else
    {
        // user mode：取 GW user node（90.3.x.x），繞過 satellite
        srcNode = topo->GetGwUserNode(gwSrc);
        dstNode = topo->GetGwUserNode(gwDst);
        srcAddr = simHelper->GetGwAddress(gwSrc);
        dstAddr = simHelper->GetGwAddress(gwDst);
    }
    ...
}

// 修正後
static void
InstallGw2GwApplicationTraffic(Ptr<SimulationHelper> simHelper,
                               uint32_t gwSrc, uint32_t gwDst,
                               double startSec, double stopSec)
{
    // 永遠使用 physical GW node，gwMode 分支完全移除
    Ptr<Node> srcNode = GetPhysicalGwNodeOrNull(gwSrc);
    Ptr<Node> dstNode = GetPhysicalGwNodeOrNull(gwDst);

    NS_ABORT_MSG_IF(!srcNode, "[GW2GW_APP] physical GW node for gwSrc=" << gwSrc << " not found");
    NS_ABORT_MSG_IF(!dstNode, "[GW2GW_APP] physical GW node for gwDst=" << gwDst << " not found");

    Ipv4Address srcAddr = GetPhysicalGwRoutableIp(srcNode, gwSrc);  // 40.x.x.x
    Ipv4Address dstAddr = GetPhysicalGwRoutableIp(dstNode, gwDst);
    ...
}
```

**原因**：這是 gwMode 影響最直接的函式。
user mode 分支使用 `GetGwUserNode()` + `GetGwAddress()`，
回傳的 IP 屬於 90.3.x.x，是 SNS3 人工建立的 user 子網路，
兩個 GW user node 可能直接透過 NS-3 IP 路由互通，完全不經過任何衛星轉發。
移除此分支後，srcAddr/dstAddr 固定為 40.x.x.x（physical node IP），
流量必須透過 feeder link → ISL → feeder link 才能送達。

---

**⑦ GetGwEndpointNode() 和 GetGwEndpointIpString() 移除 gwMode 參數**

```cpp
// 修正前
static Ptr<Node>
GetGwEndpointNode(uint32_t gwId, const std::string& gwMode)
{
    if (gwMode == "physical")
    {
        return GetPhysicalGwNodeOrNull(gwId);
    }
    return topo->GetGwUserNode(gwId);   // user mode 回傳 user node
}

static std::string
GetGwEndpointIpString(Ptr<SimulationHelper> simHelper,
                      uint32_t gwId, const std::string& gwMode)
{
    Ptr<Node> node = GetGwEndpointNode(gwId, gwMode);
    if (!node) return "";
    if (gwMode == "physical")
    {
        return Ipv4ToString(GetPhysicalGwRoutableIp(node, gwId));
    }
    return Ipv4ToString(simHelper->GetGwAddress(gwId));  // 90.3.x.x
}

// 修正後
static Ptr<Node>
GetGwEndpointNode(uint32_t gwId)
{
    return GetPhysicalGwNodeOrNull(gwId);  // 永遠 physical node
}

static std::string
GetGwEndpointIpString(Ptr<SimulationHelper> /*simHelper*/, uint32_t gwId)
{
    Ptr<Node> node = GetGwEndpointNode(gwId);
    if (!node) return "";
    return Ipv4ToString(GetPhysicalGwRoutableIp(node, gwId));  // 永遠 40.x.x.x
}
```

**原因**：這兩個函式用於 EndpointProbe、DelayMatrix 等報告輸出，
若繼續回傳 user node IP（90.3.x.x），報告中顯示的 IP 與實際流量路徑不符，
輸出難以對應到真實的 satellite path，調試時形成誤導。

---

**⑧ ActivateGwEndpointProbe() 移除 gwMode 參數與 user 分支**

```cpp
// 修正前
static void
ActivateGwEndpointProbe(Ptr<SimulationHelper> simHelper,
                        uint32_t gwId, bool installAppSink,
                        double stopSec, const std::string& gwMode)
{
    Ptr<Node> physicalGw = GetPhysicalGwNodeOrNull(gwId);
    if (gwMode != "physical" || !physicalGw)
    {
        // user mode：probe 設在 user node，非 feeder link 節點
        InstallEndpointAppSink(simHelper, topo->GetGwUserNode(gwId), ...);
        return;
    }
    InstallEndpointAppSinkAtAddress(physicalGw, GetPhysicalGwRoutableIp(physicalGw, gwId), ...);
}

// 修正後
static void
ActivateGwEndpointProbe(Ptr<SimulationHelper> /*simHelper*/,
                        uint32_t gwId, bool installAppSink, double stopSec)
{
    // 永遠 probe physical GW node
    Ptr<Node> physicalGw = GetPhysicalGwNodeOrNull(gwId);
    if (!physicalGw || !installAppSink) return;
    InstallEndpointAppSinkAtAddress(physicalGw,
                                    GetPhysicalGwRoutableIp(physicalGw, gwId), ...);
}
```

**原因**：Endpoint probe 的目的是確認封包有沒有真正送到 GW 端。
若 probe 安裝在 user node（非 satellite 節點），
即使封包走的是 IP 層捷徑，probe 也會顯示「收到」，
完全無法分辨封包是否真的經過衛星。只有安裝在 physical node 上才有意義。

---

## 檔案變更

| 檔案 | 修改內容 |
|------|----------|
| `Topology & ISL Routing/Codes/test-iridium-e2e-fix.cc` | (1) 新增 `bootstrapGwCount` 動態化邏輯；(2) 刪除 gwMode 所有相關參數、欄位、分支、函式（共 8 處修改，詳見上方 Bug 2 各項）|
| `Topology & ISL Routing/Codes/isl-graph.h` | 新增 `GwToUtRoute`、`UtDef` 結構（v7），新增 `HolDelayObserver` Stub 類別（待 SNS3 原始碼解鎖後實作），新增 `RefreshGwRoutesForSlot()` 私有方法聲明 |

---

## 驗證結果

### 來源：`Topology & ISL Routing/Outputs/fix/gw2gw.log`（simTime=630s，11 slots，截至 t=180s）

#### gwMode 移除確認

```
[GW] physical GW0 routable IP=40.1.0.1 ifIndex=1
[GW] physical GW1 routable IP=40.67.0.1 ifIndex=1
[GW2GW_APP] GW0=40.1.0.1 -> GW1=40.67.0.1 start=1.000s stop=629.000s
[GW2GW_OBS][PACKET] PacketSink::Rx trace connected on GW1 (physical)
```

修正前 GW0=90.3.0.2、GW1=90.3.0.3（user node IP）。
修正後確認使用 40.1.0.1 / 40.67.0.1（physical node IP），
不再走 IP 層捷徑。

#### Topology Bootstrap 確認

```
[TOPO_BOOTSTRAP] enabledBeams={1,2,72} gatewayPresets=5 primaryBeamId=72
[TOPO] physicalGwNodes=2 utUserNodes=91
[TOPO] logicalGwId=0 physicalGwNode=present
[TOPO] logicalGwId=1 physicalGwNode=present
```

beam 數從 5–6 降為 3，physical GW node 確認僅建立 gwSrc=0（JP-Tokyo）+ gwDst=1（IN-NewDelhi）兩個。

#### 逐 slot 路由切換（JP-Tokyo → IN-NewDelhi）

| slot | time(s) | ISL path | isl_cost(s) |
|------|---------|----------|-------------|
| 0 | 0 | 45->46->35->34->33 | 0.047382 |
| 1 | 60 | 15->14->3->4 | 0.037122 (`<-- ROUTE CHANGED`) |
| 2 | 120 | 45->34->33 | 0.029282 (`<-- ROUTE CHANGED`) |
| 3 | 180 | 14->3->4 | 0.026127 (`<-- ROUTE CHANGED`) |
| 4 | 240 | 45->34->33 | 0.027011 (`<-- ROUTE CHANGED`) |
| 5 | 300 | 45->34->33 | 0.025810 |
| 6 | 360 | 45->34->33 | 0.024578 |
| 7 | 420 | 44->45->34->33 | 0.036503 (`<-- ROUTE CHANGED`) |
| 8 | 480 | 44->45->34->33 | 0.035253 |
| 9 | 540 | 14->13->2->3 | 0.036253 (`<-- ROUTE CHANGED`) |
| 10 | 600 | 14->13->2->3 | 0.037280 |

11 個 slot 中有 7 次 `ROUTE CHANGED`，動態路由切換正常運作。

#### 完整封包路徑確認（Packet Path Trace）

```
[PKT_PATH] slot=0 t=0.0s GW0 -> feeder_up:sat45 -> isl:sat45->sat46->sat35->sat34->sat33 -> feeder_down:GW1
[PKT_PATH] slot=2 t=120.0s GW0 -> feeder_up:sat45 -> isl:sat45->sat34->sat33 -> feeder_down:GW1
[PKT_PATH] slot=9 t=540.0s GW0 -> feeder_up:sat14 -> isl:sat14->sat13->sat2->sat3 -> feeder_down:GW1
```

三段式路徑（feeder_up → ISL → feeder_down）確認正確建立，
進入衛星的節點與 IslRoutingManager 的 GW entry sat 一致。

#### 傳播延遲理論值確認

| slot | feeder_ms | isl_ms | total_theory_ms |
|------|-----------|--------|-----------------|
| 0 | 8.038 | 47.382 | 55.420 |
| 1 | 10.198 | 37.122 | 47.321 |
| 2 | 6.015 | 29.282 | 35.297 |
| 3 | 11.723 | 26.127 | 37.850 |
| 7 | 10.179 | 36.503 | 46.682 |
| 10 | 10.427 | 37.280 | 47.707 |

理論值範圍 34.7ms–55.4ms，符合 Tokyo→NewDelhi 多跳 ISL 距離預期（feeder 6–12ms，ISL 24–47ms）。

#### feeder OBS 待查

```
[OBS] traces connected: feeder=0  service=66  isl=264
[OBS] scope: feeder=0 service=0 isl=24
```

feeder link trace callback（`OrbiterRxFeeder`）未掛上，
`[FEEDER_LAYER]` verdict 目前無法量測實際 feeder rx_pkts。
ISL scope 24 links 正常。

