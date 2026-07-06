# sat-bh-example 參數整理

這份文件整理 `scratch/bh_dynamic/Codes/sat-bh-example.cc` 相關的輸入參數、預設值、衍生行為、輸出檔，以及重要 hardcode。

主要來源：

- `scratch/bh_dynamic/Codes/sat-bh-example.cc`
- `contrib/satellite/helper/sat-bh-helper.h`

## 輸入設定檔

`sat-bh-example` 現在只接受一個命令列入口：

```bash
./ns3 run "sat-bh-example --configFile=scratch/bh_dynamic/Codes/sat-bh-example.txt"
```

若沒有指定 `--configFile`，預設會讀：

```text
scratch/bh_dynamic/Codes/sat-bh-example.txt
```

設定檔格式為 `key=value`，空行會略過，註解可使用 `#` 或 `//`。以下 key 由 `ParseConfig()` 讀入。

### 預設設定檔內容

預設檔案位置：

```text
scratch/bh_dynamic/Codes/sat-bh-example.txt
```

目前內容如下：

```ini
# sat-bh-example input config
# Format: key=value. Comments may use # or //.

# Scenario / topology
scenario=constellation-starlink-1584-sats
satId=0
helperSatList=
satIdStart=490
maxHelperSats=10

# Time
simTime=300
numPeriods=0
warmUp=10
slotMs=10
periodMs=80
switchMs=2
propMs=10

# Beam hopping / scheduling
numBeams=7
maxActiveBeams=2
numHotspot=3
alpha=2
kappa=0.08
enableScheduler=0
enableObc=0
enableDynamicBstp=0
bhDemandWeight=1
bhFairnessWeight=0.5
bhValiditySF=1
bhStarvationThr=5

# Traffic / demand
enableFwdOfferedDemand=0
fwdOfferedDemandKbps=0

# Phase switches
enableCacheQueue=0
enablePrecoder=0
enableResourceManager=0
enableUserAssociation=1
enablePowerAllocation=0
enablePhaseE=0
enablePhaseF=0

# Resource manager
schedulingMode=0
maxReassignment=5

# RF / power
totalPowerDbm=43
noisePowerDbw=-126.47
powerMaxIter=30
powerEps=0.001
interferenceFactor=0.01

# Location / ROI / geometry
roiLat=35.676
roiLon=139.650
roiRadius=5
minElevDeg=37
altitudeKm=550
beamHalfAngle=2

# Output
metricsFile=bh-metrics.csv
timePlanFile=bh-timeplan.csv
trafficTraceFile=sat-bh-traffic.tr
outputDir=
```

### Scenario / 拓樸

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `scenario` | local `scenario` / `scenarioFolder` | `constellation-starlink-1584-sats` | SNS3 scenario folder。程式會由 folder 名稱推導 scenario kind 與 `numSats`；不再接受舊 alias。 |
| `satId` | `cfg.satId` | `0` | `BhExperimentConfig` 裡的基本 satellite index；實際安裝 helper 時可能改用 `helperSatList` 或 `satIdStart/maxHelperSats`。 |
| `helperSatList` | local `helperSatList` | 空字串 | 逗號分隔的 satellite ID 清單。設定後會覆蓋 `satIdStart/maxHelperSats`。 |
| `satIdStart` | `cfg.satIdStart` | `490` | 連續 helper window 的第一個 satellite ID。 |
| `maxHelperSats` | `cfg.maxHelperSats` | `10` | 最多安裝幾個 BH helper。`0` 表示全部；但 `starlink25` 若解析後為空，會 fallback 到 peak sat。 |

### 時間控制

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `simTime` | `cfg.simTimeSec` | `300.0` s | 當 `numPeriods > 0` 時會被忽略。 |
| `numPeriods` | `cfg.numPeriods` | `0` | 若 > 0，會用 `simTime = warmUp + numPeriods * periodMs / 1000` 計算模擬時間。 |
| `warmUp` | `cfg.warmUpSec` | `10.0` s | 主 traffic 在 warmup 後開始。 |
| `slotMs` | `cfg.slotDurationMs` | `10.0` ms | BH slot 長度。 |
| `periodMs` | `cfg.bhtpPeriodMs` | `80.0` ms | BHTP period；預設等於 8 個 slot。 |
| `switchMs` | `cfg.switchingTimeMs` | `2.0` ms | Beam switching dead-time。 |
| `propMs` | `cfg.propagationDelayMs` | `10.0` ms | NCC 到 OBC 的 propagation delay。 |

### Beam hopping / 排程

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `numBeams` | `cfg.numBeams` | `7` | `constellation-starlink-1584-sats` 會強制改成 `25`；Iridium scenario 會做範圍限制。 |
| `maxActiveBeams` | `cfg.maxActiveBeams` | `2` | K，每個 slot 最多同時啟用 beam 數。會被限制在 scenario beam count 內。 |
| `numHotspot` | `cfg.numHotspotBeams` | `3` | 目前 `starlink25` 會強制改成 `0`。 |
| `alpha` | `cfg.alphaDelaySensitivity` | `2.0` | Virtual traffic delay sensitivity。 |
| `kappa` | `cfg.interferenceKappa` | `0.08` | Interference cluster merge threshold。 |
| `enableScheduler` | `cfg.enableScheduler` | `false` | Phase 2 EM-based scheduler。 |
| `enableObc` | `cfg.enableObc` | `false` | Phase 2 real slot switching。 |
| `enableDynamicBstp` | `cfg.enableDynamicBstp` | `false` | Phase G greedy top-K provider。若同時開 `enableScheduler`，Scheduler 優先。 |
| `bhDemandWeight` | `cfg.bhDemandBacklogWeight` | `1.0` | Phase G demand score 權重。 |
| `bhFairnessWeight` | `cfg.bhFairnessWeight` | `0.5` | Phase G fairness score 權重。 |
| `bhValiditySF` | `cfg.bhValiditySuperframes` | `1` | Plan validity，單位是 superframe。 |
| `bhStarvationThr` | `cfg.bhStarvationThreshold` | `5` | 連續 N 次 skipped 後強制納入。 |

### Traffic / Demand

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `enableFwdOfferedDemand` | `cfg.enableFwdOfferedDemand` | `false` | `starlink25` 會強制改成 `true`。 |
| `fwdOfferedDemandKbps` | `cfg.fwdOfferedDemandKbps` | `0.0` kbps | `starlink25` 若 <= 0，會強制改成 `20000.0` kbps per beam。 |

### Phase 功能開關

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `enableCacheQueue` | `cfg.enableCacheQueue` | `false` | Phase 3 cache queue。 |
| `enablePrecoder` | `cfg.enablePrecoder` | `false` | Phase 3 MMSE precoder。 |
| `enableResourceManager` | `cfg.enableResourceManager` | `false` | Phase C resource manager loop。 |
| `enableUserAssociation` | `cfg.enableUserAssociation` | `true` | Phase C user association。 |
| `enablePowerAllocation` | `cfg.enablePowerAllocation` | `false` | Phase D IWFA power allocation。 |
| `enablePhaseE` | `cfg.enablePhaseE` | `false` | 真實 SNS3 callback wiring。 |
| `enablePhaseF` | `cfg.enablePhaseF` | `false` | DAMA/RBDC demand trace wiring。 |

### Resource / Association

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `schedulingMode` | `cfg.schedulingMode` | `0` | `0=WFQ`，`1=Priority`，`2=RoundRobin`。程式先用 `uint32_t` 解析，再轉成 `uint8_t`。 |
| `maxReassignment` | `cfg.maxReassignmentPerFrame` | `5` | 每 frame 最多呼叫幾次 `MoveUtBetweenBeams`。 |

### RF / Power

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `totalPowerDbm` | `cfg.totalPowerBudgetDbm` | `43.0` dBm | Phase D 總 TX power budget。 |
| `noisePowerDbw` | `cfg.noisePowerDbw` | `-126.47` dBW | Phase D thermal noise floor。 |
| `powerMaxIter` | `cfg.powerMaxIterations` | `30` | IWFA max iterations。 |
| `powerEps` | `cfg.powerConvergenceEps` | `0.001` W | IWFA convergence threshold。 |
| `interferenceFactor` | `cfg.interferenceFactor` | `0.01` | Cross-beam ICI leakage fraction。 |

### Location / ROI / Geometry

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `roiLat` | `cfg.roiLat` | `35.676` | ROI 中心緯度；預設東京。 |
| `roiLon` | `cfg.roiLon` | `139.650` | ROI 中心經度；預設東京。 |
| `roiRadius` | `cfg.roiRadiusDeg` | `5.0` deg | 近似 ROI 半徑 / half-width。 |
| `minElevDeg` | `cfg.minElevDeg` | `37.0` deg | 參考用 minimum elevation。 |
| `altitudeKm` | `cfg.altitudeKm` | `550.0` km | Beam geometry 與 interference model 使用。 |
| `beamHalfAngle` | `cfg.beamHalfAngleDeg` | `2.0` deg | Beam half-power half-angle。 |

### Output

| 設定檔 key | 對應變數 | 預設值 | 說明 |
| --- | --- | --- | --- |
| `metricsFile` | `cfg.metricsOutputFile` | `bh-metrics.csv` | runtime 會加上 timestamp。 |
| `timePlanFile` | `cfg.timePlanCsvFile` | `bh-timeplan.csv` | runtime 會加上 timestamp。 |
| `trafficTraceFile` | `cfg.trafficTraceFile` | `sat-bh-traffic.tr` | runtime 會加上 timestamp。 |
| `outputDir` | local `outputDir` | 空字串 | 空字串表示目前工作目錄。若設定，會在底下建立 `sns3-stats/`。 |

## BhExperimentConfig 其他預設值

以下目前不是 `sat-bh-example.txt` 直接暴露的 key，但會透過 `SatBhHelper::Configure(cfg)` 影響 helper 行為。

| 欄位 | 預設值 | 說明 |
| --- | --- | --- |
| `hotCellIndices` | empty | 固定 hot cell 清單；目前 `starlink25` 保持空。 |
| `emConvergenceEps` | `0.001` | EM convergence threshold。 |
| `emMaxIterations` | `50` | EM max iterations。 |
| `observationWindowPeriods` | `5` | Demand observation window。 |
| `nonHotspotPercentile` | `0.25` | Hotspot split threshold。 |
| `demandChangeThreshold` | `0.20` | Early trigger threshold。 |
| `maxQueueSizeMB` | `40.0` MB | Per-beam cache queue capacity。 |
| `noisePowerDb` | `-110.0` dB | MMSE precoder default noise power。 |
| `enablePatternSelection` | `false` | Optional Phase E pattern selection。 |
| `nominalKbpsPerSlot` | `50000.0` kbps | WFQ capacity hint。 |
| `maxDelayMs` | `80.0` ms | HOL deadline。 |
| `bhFwdHotspotBeamIds` | empty | 額外 FWD demand hotspot beam 清單。 |
| `bhFwdHotspotBoostKbps` | `0.0` kbps | 額外 hotspot demand boost。 |

## 衍生行為

- `numPeriods > 0` 會覆蓋 `simTime`：

```text
simTimeSec = warmUpSec + numPeriods * bhtpPeriodMs / 1000.0
```

- `scenario` 必須輸入 SNS3 scenario folder，例如 `constellation-starlink-1584-sats`。
- `helperSatList` 會覆蓋 `satIdStart/maxHelperSats`。
- 輸出檔 runtime 會加上 tag：

```text
{stem}_{scenario}_{YYYYMMDD_HHMMSS}{extension}
```

- `actualDemandPath` 從 `trafficTraceFile` 衍生：

```text
sat-bh-traffic_<tag>.tr -> bh-demand-actual_<tag>.csv
其他                    -> <traffic-stem>-actual-demand.csv
```

## Hardcode 設定

### SNS3 global config

Scenario 建立前會設定：

| Setting | Value |
| --- | --- |
| `ns3::SatConf::ForwardLinkRegenerationMode` | `REGENERATION_NETWORK` |
| `ns3::SatConf::ReturnLinkRegenerationMode` | `REGENERATION_NETWORK` |
| `ns3::SatHelper::HandoversEnabled` | `true`，但 `starlink25` 會強制改成 `false` |
| `ns3::SatHandoverModule::NumberClosestSats` | `2` |
| `ns3::SatGwMac::DisableSchedulingIfNoDeviceConnected` | `true` |
| `ns3::SatOrbiterMac::DisableSchedulingIfNoDeviceConnected` | `true` |
| `ns3::SatOrbiterFeederPhy::QueueSize` | `100000` |
| `ns3::SatEnvVariables::EnableSimulationOutputOverwrite` | `true` |
| `ns3::SatHelper::PacketTraceEnabled` | `true` |

### Starlink25 scenario

| 項目 | Hardcode 值 |
| --- | --- |
| 預設 scenario folder | `constellation-starlink-1584-sats` |
| 載入 scenario folder | `constellation-starlink-1584-sats` |
| `numSats` | `1584` |
| `numBeams` | `25` |
| Peak satellite fallback | `498` |
| Constellation time offset | `4168 s` |
| Custom gateway ID | `1` |
| Handovers | disabled |
| ISLs | disabled |
| Beam grid | 5 x 5，beam ID = cell index + 1 |
| Flat conversion 使用 Earth radius | `6371.0 km` |
| Custom `BeamUserInfo` 每 beam UT 數 | `1` |
| Custom `BeamUserInfo` 每 UT user 數 | `1` |
| `numHotspotBeams` | 強制改成 `0` |
| `enableFwdOfferedDemand` | 強制改成 `true` |
| `fwdOfferedDemandKbps <= 0` 時的預設值 | `20000.0 kbps` |

### Iridium scenario

| 項目 | Hardcode / derived 值 |
| --- | --- |
| 載入 scenario folder | `constellation-iridium-next-66-sats` |
| `numSats` | `66` |
| Max total beams | `72` |
| Per-satellite beam count | `ceil(numBeams / 66)` |
| Beam-to-satellite assignment | `(beamId - 1) % numSats` |

### LEO2SAT fallback scenario

| 項目 | Hardcode 值 |
| --- | --- |
| 載入 scenario folder | `constellation-leo-2-satellites` |
| `numSats` | `1` |
| Beam set | `1..numBeams` |

### Traffic

| Scenario / mode | Link | Interval | Packet size | 約略 offered rate |
| --- | --- | --- | --- | --- |
| `starlink25` main traffic | FWD | `600 us` | `1500 B` | `20 Mbps/UT` |
| `starlink25` baseline | RTN | `100 ms` | `512 B` | 約 `41 kbps/UT` |
| 非 `starlink25` main traffic | FWD | `20 ms` | `1500 B` | `600 kbps/UT` |
| 非 `starlink25` baseline | RTN | `100 ms` | `512 B` | 約 `41 kbps/UT` |
| Phase F 額外 CR traffic | RTN | `80 ms` | `1000 B` | `100 kbps/UT` |

### Phase F RBDC

只有 `enablePhaseF=1` 時啟用。

| Setting | Value |
| --- | --- |
| `DaService3_ConstantAssignmentProvided` | `false` |
| `DaService3_RbdcAllowed` | `true` |
| `DaService3_MinimumServiceRate` | `10` kbps |
| `DaService3_MaximumServiceRate` | `500` kbps |
| `DaService3_VolumeAllowed` | `false` |
| `SatBeamScheduler::ControlSlotsEnabled` | `true` |
| `SatBeamScheduler::ControlSlotInterval` | `1 s` |

### OBC real-toggle conflict guard

當 `enableObc` 為 true，且 `enableDynamicBstp` 或 `enableScheduler` 任一為 true 時，程式會關掉內建 static BSTP controller：

```text
ns3::SatBeamHelper::EnableFwdLinkBeamHopping = false
```

## 輸出檔

### BH output files

| 檔案 | 來源 / 說明 |
| --- | --- |
| `bh-metrics_<scenario>_<timestamp>.csv` | 來自 `cfg.metricsOutputFile`，所有 helpers 共用同一檔，用 `sat_id` 區分 rows。 |
| `bh-timeplan_<scenario>_<timestamp>.csv` | 來自 `cfg.timePlanCsvFile`。 |
| `sat-bh-traffic_<scenario>_<timestamp>.tr` | 來自 `cfg.trafficTraceFile`。 |
| `bh-demand-actual_<scenario>_<timestamp>.csv` | `starlink25` actual remaining demand，由 traffic trace path 衍生。 |
| `sat-bh-phaseF-fwd.tr` | Starlink FWD PDR trace；若有 `outputDir` 則寫到該資料夾。 |
| `bh-attributes.xml` | ConfigStore attributes snapshot；若有 `outputDir` 則寫到該資料夾。 |
| `sns3-stats/` | SNS3 built-in statistics 目錄；若有 `outputDir` 則建立在該資料夾下。 |

### 啟用的 SNS3 statistics

程式會啟用：

- per-beam FWD app throughput
- per-beam FWD user device throughput
- per-beam RTN app throughput
- per-beam beam service time
- per-sat FWD app throughput
- per-sat RTN app throughput
- per-UT FWD app throughput
- per-beam FWD user DA packet error

## 實驗檢查清單

- 要重現實驗時，同時記錄使用的 config file 和 timestamped output file name。
- `numSats` 不是設定檔 key，會由 `scenario` folder 推導。
- `constellation-starlink-1584-sats` 會在 parse 後修改 `numBeams`、`numHotspot`、`enableFwdOfferedDemand`。
- 想指定非連續 helper satellites 時，用 `helperSatList`。
- 想讓 run length 對齊 BHTP frames 時，用 `numPeriods`。
- `sat-bh-phaseF-fwd.tr` 不會自動加 timestamp；建議用 `outputDir` 隔離不同 run。
