# ISL & LEO Candidate
## 1. ISL Routing Lock
`UT → Sat-A → Sat-B → FT`

## candidate : candidate if satellite can serve Taiwan

## 2. Satellite Candidate Selection
透過`constellation-telesat-351-sats`作為基礎 ，新增條件`latitude ∈ [21.5°, 25.7°]`；
`longitude ∈ [119°, 122.5°]`，過濾出2個sat作為節點

## 3. Functions
### 3.1 Functions
```
UT
 │
Sat-A
 │
Sat-B
 │
FT
```
### 3.2 Taiwan Satellite Candidate Detection
透過衛星 SGP4 mobility model 取得每顆衛星位置 `lat ` `lon`，並判斷是否位於`latitude  : 21.5° – 25.7°
Longitude : 119° – 122.5°`

## 4 Basic Configuration
environment:
```
Simulator: ns-3 + SNS3 satellite module
Scenario : constellation-telesat-351-sats
Orbit model : SGP4
```
satellites:\
完整星座被載入，但 routing 與 scheduling 將限制於候選集合
`351 satellites`
節點:
```
UT : User Terminal
GW : Gateway
SAT : Orbiter satellite
FT : feeder terminal
```
ISL:
```
Config::SetDefault("ns3::PointToPointIslHelper::IslDataRate",
                   DataRateValue(DataRate("100Mb/s")));        //DataRate : 100 Mbps
```
Mobility :
使用 SGP4：`SatSGP4MobilityModel`設定`UpdatePositionPeriod = 1s`

## 5 Execution Flow
```
scratch/isl_lock_min.cc
sat-constellation-example.cc
```
## 5.1 isl_lock_min.cc
建立ROUTING PATH
```
NodeContainer
```
建立
```
UT
Sat-A
Sat-B
FT
```
建立 link
```
PointToPointHelper
```
建立：
```
UT ↔ Sat-A
Sat-A ↔ Sat-B
Sat-B ↔ FT
```
配置 IP
```
Ipv4AddressHelper
```
網段：
```
10.0.0.0/24
10.0.1.0/24
10.0.2.0/24
```
建立 static routing
```
Ipv4StaticRoutingHelper
```
Example：
```
UT → Sat-A
Sat-A → Sat-B
Sat-B → FT
```
建立應用層
```
UdpClient
UdpServer
```
封包路徑：
```
UT → FT
```
Trace forwarding\
透過：
```
Ipv4L3Protocol Tx / Rx trace
```
輸出：
```
forward_path.log
```
生成 PCAP
```
EnablePcap()
```
輸出：
```
pcap_ut_link
pcap_isl_link
```
## 5.2 sat-constellation-example.cc
篩選台灣區域衛星
載入星座
```
SimulationHelper::LoadScenario()
```
scenario：
```
constellation-telesat-351-sats
```
建立衛星拓樸
```
SimulationHelper::CreateSatScenario()
```
建立：
```
orbiter nodes
UT
GW
ISL links
```

取得衛星集合
```
SatTopology::GetOrbiterNodes()
Step 4
```
取得衛星位置
```
SatMobilityModel::GetGeoPosition()
```
輸出：

`latitude`、`longitude`

區域篩選
```
InTw(lat, lon)
```
判斷：
```
lat ∈ [21.5 , 25.7]
lon ∈ [119 , 122.5]
```

寫入 CSV
```
candidate_sats.csv
```
格式：
```
time,satId,in_tw
```
Example：
```
0,144,1
0,145,0
1,144,1
```
5. 輸出與驗證 (Outputs and Verification)
D1-1 輸出
route_dump.txt
```
UT → Sat-A
Sat-A → Sat-B
Sat-B → FT
```
forward_path.log

Example：
```
UT → Sat-A → Sat-B → FT

```
- 封包經過正確路徑。

PCAP

Example：
```
pcap_ut_link
pcap_isl_link
```
- 可用 Wireshark 驗證。

candidate_sats.csv

Example：
```
time,satId,in_tw
0,0,0
0,1,0
0,144,1
```
- 星座已成功載入
- SGP4 mobility 正常
- satellite position 可取得

# 6. Proves

- [X] ISL routing path 可鎖定
- [X] 星座 mobility 正常
- [X] 可建立區域衛星集合

# 7. Issues
1. `sa-constellation-example `不支援`--simTime`
> 因此 simulation 時間只能在程式中設定：`SetSimulationTime()`

2. 候選衛星偵測結果：
```
in_tw = 0
```

- 模擬時間過短 
- bounding box 範圍設定
- longitude coordinate system

3.`sat-constellation-example`不易擴充 CLI 參數。


# 8.
```
Consolidate compiler generated dependencies of target scratch_leo-bh-sim
 [Program Options] [General Arguments]

Program Options:
    --packetSize:      Size of constant packet (bytes) [512]
    --interval:        Interval to sent packets in seconds (e.g. (1s)) [100ms]
    --scenarioFolder:  Scenario folder (e.g. constellation-eutelsat-geo-2-sats-isls) [constellation-telesat-351-sats]
    --mode:            Optional mode: tw_candidates (dump candidate_sats.csv while keeping normal run)
    --tStart:          D1-2 sampling start time (seconds) [0]
    --tEnd:            D1-2 sampling end time (seconds) [300]
    --dt:              D1-2 sampling period (seconds) [1]
    --outCsv:          D1-2 output CSV path (default: OutputPath/candidate_sats.csv)
    --twLatMin:        TW bbox lat min [21.5]
    --twLatMax:        TW bbox lat max [25.7]
    --twLonMin:        TW bbox lon min [119]
    --twLonMax:        TW bbox lon max [122.5]
    --OutputPath:      Output path for storing the simulation statistics

General Arguments:
    --PrintGlobals:              Print the list of globals.
    --PrintGroups:               Print the list of groups.
    --PrintGroup=[group]:        Print all TypeIds of group.
    --PrintTypeIds:              Print all TypeIds.
    --PrintAttributes=[typeid]:  Print all attributes of typeid.
    --PrintVersion:              Print the ns-3 version.
    --PrintHelp:                 Print this help message.
```
