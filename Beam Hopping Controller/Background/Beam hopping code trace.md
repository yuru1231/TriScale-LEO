
**原生 BH 總覽**

原生 Beam Hopping example 是：

```cpp
contrib/satellite/examples/sat-fwd-link-beam-hopping-example.cc
```

做的是 **FWD link beam hopping**，原生機制的核心是：

```text
BSTP file
  -> SatBstpController
  -> SatNetDevice::ToggleState(bool)
  -> SatMac::Enable() / Disable()
```
---

**1. Example 層入口**

在 `sat-fwd-link-beam-hopping-example.cc` 一開始定義：

```cpp
uint32_t endUsersPerUt = 1;
Time simLength = Seconds(3.0);
bool scaleDown = true;
std::string simulationName = "sat-fwd-link-beam-hopping-example";
```

`endUsersPerUt` 表示每個 UT 底下有幾個 end user。若某 beam 有 30 個 UT，且 `endUsersPerUt = 1`，該 beam 就有 30 個 user。若改成 2，該 beam 就有 60 個 user。

`simLength` 是模擬時間，預設 3 秒。command line 的：

```text
--simTime
```

就是覆蓋這個變數。

`scaleDown` 控制 forward carrier bandwidth。原本 BH helper 會設 500 MHz；若 `scaleDown = true`，example 會改成 100 MHz，讓容量變小，traffic 比較容易產生擁塞與 beam 間差異。

`simulationName` 是 SimulationHelper 的名稱，影響 simulation metadata / output 命名。

---

**2. Command Line 參數**

example 裡有：

```cpp
cmd.AddValue("simTime", "Length of simulation", simLength);
cmd.AddValue("scaleDown", "...", scaleDown);
simulationHelper->AddDefaultUiArguments(cmd);
```

可從命令列改：

```text
--simTime
--scaleDown
--OutputPath
```

其中 `OutputPath` 來自 `SimulationHelper::AddDefaultUiArguments()`，用來指定統計輸出資料夾。

---

**3. 啟用 Beam Pool**

example 固定建立這些 beam：

```cpp
simulationHelper->SetBeams("1 2 3 4 11 12 13 14 25 26 27 28 40 41");
```

不是同時 transmitting，而是代表：

```text
這些 beam 會被建立
這些 beam 會有 UT / GW device / MAC / PHY / stats
這些 beam 會被註冊到 SatBstpController
這些 beam 變成 BSTP 可以控制的 beam pool
```

真正同時 active 的 beam 是由 BSTP 檔案每一行決定。

---

**4. 每個 Beam 的 UT 數**

example 對每個 beam 設定 UT 數：

```cpp
{1, 30}, {2, 9}, {3, 15}, {4, 30},
{11, 15}, {12, 30}, {13, 9}, {14, 18},
{25, 9}, {26, 15}, {27, 18}, {28, 30},
{40, 9}, {41, 15}
```

總 UT 數是全部加總：

```text
252 UT
```

因為：

```cpp
endUsersPerUt = 1;
```

所以總 end user 也是 252。

這些 UT 數會影響 offered load。beam 裡 UT 越多，通常該 beam 的 traffic demand 越高。example 刻意讓 beam 1、4、12、28 有 30 個 UT，形成高負載 beam。

---

**5. Traffic 設定**

example 安裝 FWD CBR traffic：

```cpp
AddCbrTraffic(
    SatTrafficHelper::FWD_LINK,
    SatTrafficHelper::UDP,
    MilliSeconds(1),
    512,
    gwUserNode,
    allUtUserNodes,
    MilliSeconds(1),
    simLength,
    MilliSeconds(1));
```

可讀成：

```text
link = FWD_LINK
protocol = UDP
interval = 1 ms
packetSize = 512 bytes
source = GW user node 0
destination = all UT user nodes
start = 1 ms
stop = simLength
startDelay = 1 ms
```

所以 beam hopping 影響的是 forward link 上 GW 到 UT 的資料傳輸。

---

**6. 原生 BH 開關**

example 會呼叫：

```cpp
simulationHelper->ConfigureFwdLinkBeamHopping();
```

這是原生 BH 最重要的入口。裡面設定：

```cpp
ns3::SatBeamHelper::EnableFwdLinkBeamHopping = true
ns3::SatBstpController::BeamHoppingMode = Static
ns3::SatBstpController::StaticBeamHoppingConfigFileName =
    m_scenarioPath + "/beamhopping/SatBstpConf_GW1.txt"
ns3::SatBstpController::SuperframeDuration = 1 ms
```

這代表：

```text
打開 FWD link beam hopping
使用 static BSTP
從 scenario/beamhopping/SatBstpConf_GW1.txt 讀 time plan
每個 superframe 當成 1 ms
```

注意：`BeamHoppingMode` 雖然有 `Dynamic` enum，但目前原生 code 裡沒有實作，打開會 fatal error。

---

**7. FWD Link 頻率與載波參數**

`ConfigureFwdLinkBeamHopping()` 也會設定 forward link 頻率：

```cpp
FwdFeederLinkBandwidth = 2e9
FwdFeederLinkBaseFrequency = 2.75e10

FwdUserLinkBandwidth = 5e8
FwdUserLinkBaseFrequency = 1.97e10

FwdUserLinkChannels = 1
FwdFeederLinkChannels = 4

FwdCarrierAllocatedBandwidth = 5e8
FwdCarrierRollOff = 0.2
FwdCarrierSpacing = 0.0
```

若 `scaleDown == true`，example 會覆蓋：

```cpp
FwdCarrierAllocatedBandwidth = 1e8
```

所以 `scaleDown` 的效果是：

```text
500 MHz -> 100 MHz
```

它不改 beam 數量，也不改 BSTP pattern。

---

**8. Scenario 檔案**

原生 example 使用：

```text
geo-33E-beam-hopping
```

主要檔案：

```text
standard/standard.txt
beams/fwdConf.txt
beams/rtnConf.txt
beamhopping/SatBstpConf_GW1.txt
positions/gw_positions.txt
positions/sat_positions.txt
positions/ut_positions.txt
waveforms/waveforms.txt
waveforms/default_waveform.txt
```

`standard/standard.txt` 是：

```text
DVB
```

表示 scenario 使用 DVB satellite stack，不是 LORA stack。

預計換成：

```text
constellation-starlink-1584-sats
```

原本沒有 `beamhopping/` 目錄，同時預計改成 dynamic provider。

---

**9. Beam Config：fwdConf / rtnConf**

`fwdConf.txt` 和 `rtnConf.txt` 每行 4 欄：

```text
beamId userFreqId gwId feederFreqId
```

對應 index 定義：

```cpp
BEAM_ID_INDEX = 0
U_FREQ_ID_INDEX = 1
GW_ID_INDEX = 2
F_FREQ_ID_INDEX = 3
BEAM_ELEM_COUNT = 4
```

原生 BH 的合法性檢查會用到：

```text
beamId 是否存在
beamId 對應哪個 GW
beamId 使用哪個 feeder frequency
同一個 GW 是否同時開了衝突的 feeder frequency
```

對 dynamic BHTP 來說，這些 mapping 必須保留。

---

**10. BSTP 檔案格式**

原生 static BSTP 位於：

```text
contrib/satellite/data/scenarios/geo-33E-beam-hopping/beamhopping/SatBstpConf_GW1.txt
```

內容：

```text
3, 1, 4, 12, 28
1, 2, 13, 25, 40
2, 11, 3, 26, 41
1, 1, 14, 12, 27
```

格式：

```text
validityInSuperframes, activeBeam1, activeBeam2, ...
```

第一行意思是：

```text
beam 1, 4, 12, 28 active
持續 3 個 superframe
```

因為原 example 設：

```cpp
SuperframeDuration = 1 ms
```

所以第一行持續：

```text
3 * 1 ms = 3 ms
```

---

**11. 原生 Runtime 流程**

建立 scenario 時，`SatBeamHelper` 會對每個 enabled beam 建 GW/UT net device。若開了 BH：

```cpp
EnableFwdLinkBeamHopping = true
```

建立：

```cpp
SatBstpController
```

每個 beam 註冊 callback：

```cpp
beamId -> SatNetDevice::ToggleState(bool)
```

初始化時：

```cpp
SatBstpController::Initialize()
```

呼叫：

```cpp
DoBstpConfiguration()
```

流程：

```text
讀下一行 BSTP
取得 validityInSuperframes
取得 activeBeams
遍歷所有已註冊 beam
    若 beamId 在 activeBeams 裡 -> ToggleState(true)
    否則 -> ToggleState(false)
排程下一次 DoBstpConfiguration()
```

下一次時間是：

```cpp
validityInSuperframes * m_superFrameDuration
```

---

**12. 真正的 Beam On/Off 做了什麼**

原生 callback 是：

```cpp
SatNetDevice::ToggleState(bool enabled)
```

內容：

```cpp
if (enabled)
{
    m_mac->Enable();
}
else
{
    m_mac->Disable();
}
```

`SatMac::Enable()` 設：

```cpp
m_txEnabled = true;
```

`SatMac::Disable()` 設：

```cpp
m_txEnabled = false;
```

所以原生 BH 不是移動 UT，不是換 satellite，不是改 routing，而是：

```text
開/關對應 beam 的 MAC 傳輸
```

---

**13. Runtime 狀態變數**

`SatBstpController` 內部：

```cpp
m_gwNdCallbacks
m_bhMode
m_configFileName
m_superFrameDuration
m_staticBstp
```

`m_gwNdCallbacks` 是：

```text
beamId -> ToggleCallback
```

`SatStaticBstp` 內部：

```cpp
m_bstp
m_currentIterator
m_beamGwMap
m_beamFeederFreqIdMap
m_enabledBeams
```

其中：

```text
m_bstp = BSTP file 讀進來的所有 lines
m_currentIterator = 下一次要讀哪一行
m_enabledBeams = SetBeams 建立的 beam pool
m_beamGwMap = beamId -> gwId
m_beamFeederFreqIdMap = beamId -> feederFreqId
```

這些就是 dynamic BHTP provider 需要模仿或沿用的狀態。

---

**14. 原生合法性檢查**

`SatStaticBstp::CheckValidity()` 會檢查：

```text
BSTP 不可為空
每行 beamId 不可重複
BSTP 裡出現的 beam 應該是 enabled beam
同一 GW 同一 feederFreqId 不可在同一 BSTP line 重複
所有 enabled beams 至少要在 BSTP 中出現過一次
```

對 static file 來說，這是在初始化時檢查整份檔案。

對 dynamic BHTP 來說，應該改成：

```text
每次產生 activeBeams 後，立即檢查這一個 window 是否合法
```

保留：

```text
beamId / userFreqId / gwId / feederFreqId
```


---

**15. 環境與輸出參數**

`SimulationHelper` 支援：

```text
OutputPath
InputXml
```

`SimulationHelperConf` 常用 attributes：

```text
SimTime
BeamsIDs
UserCountPerUt
UserCountPerMobileUt
UtCountPerBeam
ActivateStatistics
ActivateProgressLogs
MobileUtsFolder
```

`SatEnvVariables` 常用 attributes：

```text
DataPath = contrib/satellite/data
SimulationCampaignName
SimulationTag = default
EnableSimulationOutputOverwrite = true
EnableSimInfoOutput = true
EnableSimInfoDiffOutput = true
ExcludeSatelliteDataFolderFromSimInfoDiff = true
```
 BH pattern，但影響：

```text
scenario 從哪裡讀
output 寫到哪裡
是否覆蓋舊結果
simulation metadata 是否輸出
```

---

**16. 原生 BH 統計輸出**

example 啟用：

```cpp
GlobalFwdAppThroughput
PerBeamFwdAppThroughput
PerBeamBeamServiceTime
GlobalFwdAppDelay
GlobalFwdCompositeSinr
```

最重要的是：

```text
PerBeamBeamServiceTime
PerBeamFwdAppThroughput
```

`PerBeamBeamServiceTime` 用來看每個 beam 被 active 多久。

`PerBeamFwdAppThroughput` 用來看每個 beam 實際拿到多少 forward throughput。

dynamic BHTP 成功，輸出應隨 dynamic active beam selection 改變。

---

**17. 改成 Dynamic BHTP 要保留什麼**

原生 static 的核心輸入是：

```text
validityInSuperframes + activeBeams
```

dynamic provider 最小也要產生同樣格式：

```cpp
struct DynamicBstpConf
{
    uint32_t validityInSuperframes;
    std::vector<uint32_t> activeBeams;
};
```

然後在每次 `DoBstpConfiguration()` 時取代：

```cpp
m_staticBstp->GetNextConf()
```

變成：

```cpp
m_dynamicProvider->GetNextConf(Simulator::Now())
```

但是 dynamic provider 仍然要知道：

```text
enabled beams
beamId -> gwId
beamId -> feederFreqId
beamId -> userFreqId
SuperframeDuration
最大同時 active beam 數
目前或歷史 demand
```

---

**18. Dynamic BHTP 的最小設計**

最小可行版：

```text
保留原生 SatBstpController
新增 SatDynamicBstpProvider
AddNetDeviceCallback 時也把 beam info 給 provider
DoBstpConfiguration 時從 provider 拿下一組 activeBeams
沿用原生 ToggleState callback
輸出 dynamic-bstp.csv
```

最小 policy 可以先做：

```text
round-robin top-K beams
```

下一步再改成：

```text
根據 demand/backlog 選 top-K beams
```

再下一步才加入：

```text
fairness
timeSinceLastServed
interference constraint
LEO visibility
handover / UT association
power allocation
```

---

**19. Starlink Scenario 需要特別注意**

你預計從：

```text
geo-33E-beam-hopping
```

換到：

```text
constellation-starlink-1584-sats
```

確認：



- [X] 有 standard/standard.txt = DVB
- [X] 有 beams/fwdConf.txt 和 rtnConf.txt
- [ ] 改成 dynamic provider
- [ ] beamId 是否和你的 activeBeams 對得上
- [ ] gwId / feederFreqId 是否滿足同時 active 約束
- [ ] LEO constellation 下 SatBstpController 的 GW-device toggle 是否仍符合你的研究假設
```

原生 BH 是 GEO forward beam hopping 設計。搬到 LEO constellation 時，dynamic BHTP 可能還需要處理：

```text
satId
beamId
visible cells
UT-satellite association
GW feeder visibility
handover
```

原生 static BSTP 只有 `beamId`，沒有 `satId` 維度。這會是 Starlink dynamic 化時的模型差異。

---

**20. core**

原生 BH 的本質是：

```text
從 BSTP 取得 active beam list
依照 list 開關每個 beam 的 MAC transmission
用 service time / throughput 驗證效果
```

改 dynamic BHTP 應該保留：

```text
SatBstpController -> ToggleState -> SatMac Enable/Disable
```

替換：

```text
SatStaticBstp::GetNextConf()
```
