# 工作日誌 2026-04-15

## 目標
修正 `sat2gw` 情境下 E2E 可觀測性的統計來源，將 feeder link 觀測從衛星側 `RxFeeder` 改為 GW return-feeder Rx，並加入可驗證的 debug 機制與清楚的 case 輸出。

---

## 完成事項

### 1. `sat2gw` feeder 觀測來源切換（核心修正）

**現象**：`sat2gw` 情境下，feeder link 的封包統計掛在衛星端的 `RxFeeder` trace，但 `sat2gw` 的下行流量實際由 GW return-feeder Rx 接收，導致觀測來源與實際鏈路不符。

**原因**：`ConnectLinkObserverTraces()` 原本對所有 pathType 統一使用 `OrbiterRxFeederCb`，未區分 `sat2gw` 與其他路徑對 feeder 觀測點的差異。

**修正**：

檔案：`test-iridium-e2e.cc` 行 `547–668`

```cpp
// --- Link observer trace connection ---
// Must run AFTER ConnectIslDropTrace() so ISL callback can look up g_nodeToSatId.

static void
ConnectLinkObserverTraces(bool useGwReturnFeederObs)
{
    NodeContainer sats = Singleton<SatTopology>::Get()->GetOrbiterNodes();
    uint32_t      connFeeder{0}, connService{0}, connIsl{0};

    g_feederObsStats.clear();
    g_serviceObsStats.clear();
    g_islObsStats2.clear();
    g_gwDeviceRxHits.clear();
    g_gwDeviceTypes.clear();

    for (uint32_t i = 0; i < sats.GetN(); ++i)
    {
        Ptr<Node>   satNode = sats.Get(i);
        std::string satKey  = "sat" + std::to_string(i);

        for (uint32_t d = 0; d < satNode->GetNDevices(); ++d)
        {
            Ptr<SatOrbiterNetDevice> orbDev =
                DynamicCast<SatOrbiterNetDevice>(satNode->GetDevice(d));
            if (!orbDev) continue;

            // sat2gw 模式下不掛衛星側 feeder trace，改由 GW Rx 接收
            if (!useGwReturnFeederObs &&
                orbDev->TraceConnectWithoutContext(
                    "RxFeeder",
                    MakeBoundCallback(&OrbiterRxFeederCb, satKey)))
            {
                ++connFeeder;
            }
            if (!useGwReturnFeederObs)
            {
                orbDev->TraceConnectWithoutContext(
                    "RxFeederLinkDelay",
                    MakeBoundCallback(&OrbiterFeederDelayCb, satKey));
            }

            // service / ISL trace 兩種模式都掛
            if (orbDev->TraceConnectWithoutContext(
                    "RxUser",
                    MakeBoundCallback(&OrbiterRxUserCb, satKey)))
            {
                ++connService;
            }
            orbDev->TraceConnectWithoutContext(
                "RxUserLinkDelay",
                MakeBoundCallback(&OrbiterUserDelayCb, satKey));

            auto islDevices = orbDev->GetIslsNetDevices();
            for (auto& islDev : islDevices)
            {
                if (islDev->TraceConnectWithoutContext(
                        "PacketDropRateTrace",
                        MakeCallback(&IslObsCb)))
                {
                    ++connIsl;
                }
            }

            if (!useGwReturnFeederObs)
                g_feederObsStats[satKey].BeginWindow(0.0);
            g_serviceObsStats[satKey].BeginWindow(0.0);
            break;
        }
    }

    // sat2gw：對 GW 節點的 SatNetDevice 掛 GatewayRxFeederCb
    if (useGwReturnFeederObs)
    {
        for (uint32_t gwId = 0; gwId < 3; ++gwId)
        {
            NodeContainer gwNodes = GetGwNodesById(gwId);
            std::string   gwKey   = MakeGwKey(gwId);

            for (uint32_t n = 0; n < gwNodes.GetN(); ++n)
            {
                Ptr<Node> gwNode = gwNodes.Get(n);
                if (!gwNode) continue;

                for (uint32_t d = 0; d < gwNode->GetNDevices(); ++d)
                {
                    std::string devKey = gwKey + "/dev" + std::to_string(d);
                    g_gwDeviceTypes[devKey] =
                        gwNode->GetDevice(d)->GetInstanceTypeId().GetName();

                    if (g_obsDebug)
                    {
                        gwNode->GetDevice(d)->TraceConnectWithoutContext(
                            "Rx",
                            MakeBoundCallback(&GatewayDeviceRxDebugCb, devKey));
                    }

                    Ptr<SatNetDevice> satDev =
                        DynamicCast<SatNetDevice>(gwNode->GetDevice(d));
                    if (!satDev) continue;

                    satDev->SetAttribute("EnableStatisticsTags", BooleanValue(true));
                    if (satDev->TraceConnectWithoutContext(
                            "Rx",
                            MakeBoundCallback(&GatewayRxFeederCb, gwKey)))
                    {
                        ++connFeeder;
                    }
                }
            }
            g_feederObsStats[gwKey].BeginWindow(0.0);
        }
    }

    // 啟動時輸出觀測來源供確認
    std::cout << "[OBS] build=2026-04-15-rf-v2"
              << " feederSource=" << (useGwReturnFeederObs ? "gw_return_rx" : "orbiter_rxfeeder")
              << "\n";
}
```

進入點（行 `1865`）：

```cpp
// Connect feeder / service / ISL traces (must run after ConnectIslDropTrace).
ConnectLinkObserverTraces(e2eCfg.pathType == "sat2gw");
```

**驗證**：依據 repo 內 `Topology & ISL Routing/Outputs/E2E-ReturnFeeder/sat2gw_120s.md`，`pathType=sat2gw` 情境下模擬 120 秒，`TOTAL: drop_rate=0.000% [PASS]`，ISL drop rate 檢查通過。

---

### 2. 新增全域狀態與 GW Rx debug callback

**現象**：切換觀測來源後，難以確認 GW 各 device 是否實際有 Rx 命中，缺乏可見的 debug 路徑。

**原因**：原本系統無法在不修改 source 的情況下列出 GW 各 device 型別與個別 rx_hits。

**修正**：

全域狀態（行 `271–273`）：

```cpp
static std::map<std::string, uint64_t>     g_gwDeviceRxHits;  // key: "gw<N>/dev<D>"
static std::map<std::string, std::string>  g_gwDeviceTypes;   // key → device type name
static bool                                g_obsDebug{false};
```

Debug callback（行 `393–397`）：

```cpp
static void
GatewayDeviceRxDebugCb(std::string key, Ptr<const Packet> /*pkt*/, const Address& /*addr*/)
{
    g_gwDeviceRxHits[key]++;
}
```

**驗證**：啟用 `--obsDebug=1` 後，`PrintObsFinalSummary()` 會輸出各 GW device 的 rx_hits（見第 5 項）。

---

### 3. 新增 `GetGwNodesById()` 輔助函式

**修正**：

行 `923–933`：

```cpp
static NodeContainer
GetGwNodesById(uint32_t gwId)
{
    auto topo = Singleton<SatTopology>::Get();
    NodeContainer gwNodes = topo->GetGwNodes();
    if (gwId >= gwNodes.GetN())
    {
        return NodeContainer();  // out of range → 回傳空容器
    }
    return NodeContainer(gwNodes.Get(gwId));
}
```

---

### 4. 新增 CLI 參數 `--obsDebug` 及初始化

**修正**：

CLI 宣告（行 `1719–1721`）：

```cpp
cmd.AddValue("obsDebug",
             "Enable verbose OBS debug output (GW device list and rx_hits) (0/1)",
             obsDebug);
```

初始化（行 `1772`）：

```cpp
g_obsDebug = obsDebug;
```

---

### 5. `PrintObsFinalSummary()` 新增 GW device Rx debug 輸出

**修正**：

行 `724–741`：

```cpp
if (g_obsDebug && !g_gwDeviceTypes.empty())
{
    std::cout << std::string(84, '-') << "\n";
    std::cout << "GW device Rx debug hits\n";
    for (const auto& kv : g_gwDeviceTypes)
    {
        uint64_t hits = 0;
        auto hitIt = g_gwDeviceRxHits.find(kv.first);
        if (hitIt != g_gwDeviceRxHits.end())
        {
            hits = hitIt->second;
        }
        std::cout << "  " << kv.first
                  << " | " << kv.second
                  << " | rx_hits=" << hits << "\n";
    }
}
```

**預期輸出**：

```
GW device Rx debug hits
  gw0/dev0 | ns3::SatNetDevice | rx_hits=42
  gw0/dev1 | ns3::CsmaNetDevice | rx_hits=0
```

---

### 6. 新增 `sat2gw` case 標頭輸出

**修正**：

行 `1639–1640`：

```cpp
std::cout << "\n[CASE] sat2gw | gwId=" << cfg.gwId
          << " | feederlink_dn only | isl_cost=N/A\n";
```

---

### 7. `isl-graph.cc`：`PrintGwUtRouteReport()` isl_cost 語意修正

**現象**：`gw2ut_e2e` 報表中，「路徑有效但無 ISL hop」與「路徑無效」都顯示 `N/A`，語意不清。

**修正**：

`isl-graph.cc` 行 `1846–1852`：

```cpp
std::ostringstream costSs;
if (r.valid && HasIslTransitPath(r.satPath))
    costSs << std::fixed << std::setprecision(6) << r.islCost;  // 顯示數值
else if (r.valid)
    costSs << "no ISL hop";  // 有效路徑但無 ISL 中繼
else
    costSs << "-";           // 路徑無效
```

**驗證**：依據 repo 內 `Topology & ISL Routing/Outputs/E2E-PathType/gw2ut_e2e_ut0_120s.md`，entry=serving 情況已顯示 `no ISL hop`，符合語意修正目標。

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `Topology & ISL Routing/Codes/test-iridium-e2e.cc` | 全域狀態 `g_gwDeviceRxHits/Types/g_obsDebug`；`GatewayDeviceRxDebugCb`；`GetGwNodesById()`；`ConnectLinkObserverTraces()` 切換 feeder 觀測來源；`PrintObsFinalSummary()` GW debug 輸出；`--obsDebug` CLI 參數；`[CASE] sat2gw` 標頭 |
| `Topology & ISL Routing/Codes/isl-graph.cc` | `PrintGwUtRouteReport()` isl_cost 欄位區分數值 / `no ISL hop` / `-` |



## 明日計畫

- 認 `gw_return_rx` 觀測來源有 rx_hits 命中。
- 比對 `sat2gw_120s.md` 回歸結果，確認 `drop_rate` 數值一致。
- 確認 `gw2ut_e2e` 報表 `isl_cost` 欄位顯示格式正確（`no ISL hop` vs `-`）。
