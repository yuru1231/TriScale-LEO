# 2026-05-04 工作日誌
gw2gw_e2e 仰角測試 & 模擬效能診斷

---

## 今日目標

1. 嘗試將 `elevMinDeg` 提高至 35° 測試是否改善 `FEEDER_LAYER FAIL` 問題
2. 診斷模擬卡住的根本原因
3. 確認 `gw2gw_e2e` 的正確執行參數與時間預期

---

## 發現的問題與診斷結果

### 問題 1：`elevMinDeg=35` 導致模擬卡住 135+ 分鐘

**症狀**：
- `simTime=180 --elevMinDeg=35` 卡在 `slot=0 t=0.000s` 超過 135 分鐘
- CPU 持續 99.7%，無新 log 輸出

**根因推測**：
- `elevMinDeg=35` 選出恰好在 35° 邊緣的 entry satellite
- Iridium 衛星移動快速，幾分鐘後該衛星低於 35°
- SNS3 feeder channel 偵測到 link down，觸發大量 cascade 事件
- `SendGw2GwTaggedPacket` 每 100ms 持續送封包到已下線的 feeder channel
- 事件佇列爆炸

**驗證**：`simTime=10 --elevMinDeg=35` 在 < 1 秒完成（衛星在前 10 秒仍在 35° 以上）。

**結論**：`elevMinDeg=35` 不是有效的 debug 路徑。

---

### 問題 2：`gw2gw_e2e` 固有的執行時間長

**症狀**：
- `simTime=5 --elevMinDeg=5` 跑了 6+ 分鐘仍未完成
- `simTime=65 --elevMinDeg=5` 跑了 57+ 分鐘仍未完成

**根因（code 確認）**：

位於 [test-iridium-e2e-fix.cc](Topology%20&%20ISL%20Routing/Codes/test-iridium-e2e-fix.cc#L5688)：

```cpp
// Activating all 5 GW beams multiplies satellite MAC/DAMA/RBDC events
// by ~5x and inflates wall time from ~12 min to ~3 hours.
const bool includeAllFeederBeams = (pathType == "gw2gw_e2e");
```

`gw2gw_e2e` 強制啟用 `includeAllFeederBeams=true`（2 GW × 66 衛星 = **132 個 feeder beam**），SNS3 的 MAC/DAMA/RBDC 內部事件數量大幅增加。

**執行時間估算**：

| simTime | 預估 wall clock |
|---------|----------------|
| 5s | ~6 分鐘 |
| 65s | ~57–80 分鐘 |
| 180s | ~150–200 分鐘 |

這是 `gw2gw_e2e` 固有限制，非 bug。

---


## 修正 I：`SetGwFeederSats` linker error（undefined reference）

**症狀**：

```
undefined reference to 'ns3::IslRoutingManager::SetGwFeederSats'
scratch/test-iridium-e2e-fix.cc:5199
```

**根因**：`isl-graph.h` 已宣告 `SetGwFeederSats()`，`test-iridium-e2e-fix.cc` 也已呼叫，但 `isl-graph.cc` 從未提供實作。此外 `PrecomputeGwRoutes()` 有 `m_gwFeederSats` member，但從未在計算 entry/exit 候選時套用。

**修正位置**：[isl-graph.cc](Topology%20&%20ISL%20Routing/Codes/isl-graph.cc)

#### (1) 新增 `SetGwFeederSats` 實作

```cpp
void
IslRoutingManager::SetGwFeederSats(uint32_t gwId, const std::set<uint32_t>& validSats)
{
    // Records the set of satellites that have an active feeder channel to gwId.
    // PrecomputeGwRoutes() intersects this set with elevation-visible sats
    // to restrict entry/exit candidates. Empty set = no feeder filter applied.
    m_gwFeederSats[gwId] = validSats;
    CHKPT("SetGwFeederSats: gwId=" << gwId << " feederSats=" << validSats.size());
}
```

#### (2) `PrecomputeGwRoutes()` 新增 `feederFilter` lambda

```cpp
// feederFilter: intersects elevation-visible sats with gwId's feeder sat set.
// If m_gwFeederSats has no entry for gId (or it is empty), no filter is applied.
auto feederFilter = [&](const std::set<uint32_t>& vis,
                        uint32_t                  gId) -> std::vector<uint32_t>
{
    auto fit = m_gwFeederSats.find(gId);
    if (fit == m_gwFeederSats.end() || fit->second.empty())
        return std::vector<uint32_t>(vis.begin(), vis.end());
    std::vector<uint32_t> result;
    for (uint32_t s : vis)
        if (fit->second.count(s))
            result.push_back(s);
    return result;
};

std::vector<uint32_t> entryCandAB = feederFilter(srcSatsAB, gwA);
std::vector<uint32_t> exitCandAB  = feederFilter(dstSatsAB, gwB);
std::vector<uint32_t> entryCandBA = feederFilter(srcSatsBA, gwB);
std::vector<uint32_t> exitCandBA  = feederFilter(dstSatsBA, gwA);
```

AB loop 改用 `entryCandAB`/`exitCandAB`，BA loop 改用 `entryCandBA`/`exitCandBA`。

---

## 修正 J：feeder 候選衛星建構（IP scan → rtnConf）

**症狀（runtime log）**：

```
[GW_FEEDER] gwIdx=0 feederSats=255
[GW_FEEDER] gwIdx=1 feederSats=0
[GW_FEEDER] gwIdx=2 feederSats=0
[GW_FEEDER] gwIdx=3 feederSats=0
[GW_FEEDER] gwIdx=4 feederSats=0
[GW2GW_APP] slot0 entrySat=45 GW0 feeder ifIndex=46 gwIp=40.46.0.1
```

**根因**：

舊做法以 IP 掃描（`o1==40 && o2>=1`）判斷哪些衛星屬於哪個 GW。`includeAllFeederBeams=true` 時，所有 GW 的 feeder 介面集中在 GW0 的物理節點，`o2` 值 1–255 涵蓋全部 GW feeder 介面（含 Mumbai beam 46 = sat45）。

- `gwIdx=0 feederSats=255`：誤收全部 GW feeder 介面，sat45（Mumbai）錯誤進入 GW0（Tokyo）filter 集合
- `gwIdx=1–4 feederSats=0`：其餘 GW 節點無 feeder 介面，filter 退化為仰角可見集合（無限制）

**修正**：改用 `rtnConf.txt` 直接映射（`satId = beam - 1`，`gwIdx = gwIdFromFile - 1`）。

**修正位置**：[test-iridium-e2e-fix.cc](Topology%20&%20ISL%20Routing/Codes/test-iridium-e2e-fix.cc)，`ConfigureRoutingCase()` gw2gw_e2e 段落

```cpp
// IP-based scanning is unreliable with includeAllFeederBeams=true:
// all feeder interfaces collapse onto GW0's node (o2 up to 255 > numSats).
// rtnConf.txt directly encodes satId = beam-1, gwIdx = gwIdFromFile-1.
{
    std::map<uint32_t, std::set<uint32_t>> feederSatsMap;   // gwIdx → {satIds}
    std::ifstream rtnIn(cfg.rtnConfFilePath);
    if (rtnIn.is_open())
    {
        uint32_t beam, userChannel, gwIdFromFile, feederChannel;
        while (rtnIn >> beam >> userChannel >> gwIdFromFile >> feederChannel)
        {
            if (gwIdFromFile == 0 || beam == 0) continue;
            uint32_t satId = beam - 1;           // 0-indexed satellite ID
            uint32_t gwIdx = gwIdFromFile - 1;   // 0-indexed GW index
            if (satId < cfg.numSats)
                feederSatsMap[gwIdx].insert(satId);
        }
    }
    else
    {
        std::cout << "[GW_FEEDER] WARNING: cannot open " << cfg.rtnConfFilePath
                  << "; feeder filter disabled for all GWs\n";
    }

    uint32_t numGwNodes = Singleton<SatTopology>::Get()->GetGwNodes().GetN();
    for (uint32_t gwIdx = 0; gwIdx < numGwNodes; ++gwIdx)
    {
        auto it = feederSatsMap.find(gwIdx);
        if (it != feederSatsMap.end())
            routingMgr->SetGwFeederSats(gwIdx, it->second);
        else
            routingMgr->SetGwFeederSats(gwIdx, {});
        uint32_t count = (it != feederSatsMap.end()) ? it->second.size() : 0;
        std::cout << "[GW_FEEDER] gwIdx=" << gwIdx
                  << " feederSats=" << count << "\n";
    }
}
```

`E2EConfig::rtnConfFilePath` 欄位同步新增，並在 `main()` 設定。

**預期修正後 log**：

```
[GW_FEEDER] gwIdx=0 feederSats=~13    ← 只含 Tokyo 衛星
[GW_FEEDER] gwIdx=1 feederSats=~13    ← 只含 Delhi 衛星
[GW2GW_APP] slot0 entrySat=X          ← X ≠ 45（sat45 屬 Mumbai，已排除）
```

---

## 目前狀態

| 項目 | 狀態 |
|------|------|
| `elevMinDeg=35` 測試 | ✅ 確認無效，放棄此方向 |
| 模擬卡住根因 | ✅ 確認為 SNS3 132-beam MAC/DAMA/RBDC 事件量 |
| `SetGwFeederSats` linker error | ✅ 修正 I：新增實作 + feederFilter lambda |
| feeder 候選衛星誤判（gwIdx=0 feederSats=255）| ✅ 修正 J：IP scan → rtnConf 映射 |
| `gw2gw_e2e FEEDER_LAYER FAIL` | ⏳ 待實機驗證（feeder filter 已就位） |
| `sat2sat` NS_ABORT | ⏳ 待調查 |

---

## 驗證指令

```bash
cd $NS3_ROOT
./ns3 run "test-iridium --pathType=gw2gw_e2e --gwSrc=0 --gwDst=1" 2>&1 \
  | grep -E "GW_FEEDER|GW2GW_APP.*entrySat|FEEDER_LAYER|PACKET_LAYER|OrbiterRxFeeder" \
  | tee Logs/run_gw2gw_feederfix.log
```

確認項目：
- `gwIdx=0 feederSats` 在 10–20 之間（非 255）
- `gwIdx=1 feederSats` 在 10–20 之間（非 0）
- `entrySat` 不為 45
- `[FEEDER_LAYER]` 與 `[PACKET_LAYER]` verdict 是否由 FAIL 轉為 PASS

---

## 下一步

1. 實機跑驗證指令，確認 FEEDER_LAYER / PACKET_LAYER verdict
2. 若 FEEDER_LAYER PASS → 進入 PACKET_LAYER 完整驗證
3. 若 FEEDER_LAYER 仍 FAIL → 調查 `SatGwLlc::PacketTrace` / `SatGwMac::Tx` drop 路徑
4. 調查 `sat2sat` NS_ABORT（低優先）
