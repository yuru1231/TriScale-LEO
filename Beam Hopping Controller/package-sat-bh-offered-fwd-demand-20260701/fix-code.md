# Starlink25 BH Code Notes - 2026-07-01

## 修改範圍

主要涉及三個檔案：

- `scratch/bh_dynamic/Codes/sat-bh-example.cc`
- `contrib/satellite/helper/sat-bh-helper.h`
- `contrib/satellite/helper/sat-bh-helper.cc`

目前重點是讓 `sat-bh-example --scenario=starlink25` 可以接上：

- Starlink 1584 constellation snapshot
- Tokyo ROI 5x5 beam grid
- 多顆 helper satellite 的 beam/device mapping
- FWD offered demand 作為 Dynamic BSTP scheduler input
- `.tr` 輸出 `DEMAND / PLAN / EVENT`
- OBC beam activate/deactivate event trace
- Phase F FWD packet Rx trace 作為 packet validation

ISL routing 目前先暫緩。

## Starlink25 ROI Grid

`starlink25` 使用 Tokyo ROI：

```text
ROI center = Tokyo
lat = cfg.roiLat
lon = cfg.roiLon
grid = 5 x 5
beam count = 25
```

程式會用：

```cpp
auto gridXY = cp.GridPositions(5, 5, cfg.beamHalfAngleDeg);
```

產生 25 個 grid center，接著轉成經緯度：

```cpp
kCellLat[i] = ...
kCellLon[i] = ...
```

每個 grid cell 對應一個 beam：

```text
grid cell 0  -> beamId 1
grid cell 1  -> beamId 2
...
grid cell 24 -> beamId 25
```

並且每個 `(satId, beamId)` 會放 1 個 UT：

```cpp
info.SetPositions({std::make_pair(GeoCoordinate(kCellLat[i], kCellLon[i], 0.0), 0)});
info.AppendUt(1);
beamInfo.emplace(std::make_pair(satId, beamId), info);
```

所以目前 user position 與 grid/beam 有 mapping：

```text
UT position -> ROI grid cell -> beamId
```

但目前 traffic demand 還沒有依照 user position 做 hotspot 差異。

## Helper Satellite List

新增/使用 `--helperSatList` 來指定哪些 satellite 要套用 Tokyo ROI 25-beam grid，例如：

```bash
--helperSatList=382,404,448,883,927,948
```

在 `starlink25` 裡，這些 satellite 會各自建立 25 個 beam/UT mapping。

例如：

```text
helperSatList=382,404
=> 2 sats x 25 beams = 50 UTs
```

平行候選 helper satellites，不是同一時間可見 serving satellite。

目前 `.tr` 會同時寫出 382、404 helper sat 的排程事件。若要變成真實時間序列，需要再加入 visibility gating / serving-sat gating。

## 20 Mbps FWD Offered Demand

目前 `starlink25` 預設每個 beam 的 FWD offered demand 是 20 Mbps：

```cpp
cfg.enableFwdOfferedDemand = true;
cfg.fwdOfferedDemandKbps = 20000.0;
```

換算：

```text
20000 kbps = 20 Mbps
```

packet traffic 端也有對應的 FWD CBR：

```cpp
AddCbrTraffic(FWD_LINK, UDP, MicroSeconds(600), 1500, ...)
```

換算：

```text
1500 bytes / 600 us = 20 Mbps
```

## Demand Source Cleanup

排程輸入改成：

```text
FWD offered demand -> Dynamic BSTP -> PLAN -> OBC EVENT
```

在 `enableFwdOfferedDemand=true` 時，`SatBhHelper::InjectFwdDemand()` 每個 BHTP period 會把每個 beam 的 offered demand 餵進 Dynamic BSTP：

```cpp
m_dynamicProvider->UpdateBeamDemand(bid, demandKbps);
```

同時 `.tr` 會寫出：

```text
DEMAND,...,beam_id,OFFERED_FWD,...,demand_kbps
```



意思是：

```text
OFFERED_FWD = scheduler input
PacketSink Rx = packet output / validation
```

## Beam Selection Logic

目前 `starlink25 + enableDynamicBstp=1` 走的是 Greedy Dynamic BSTP provider。

beam 選擇流程：

```text
InjectFwdDemand()
  -> UpdateBeamDemand()
  -> SatGreedyBstpProvider::GetNextConf()
  -> conf.activeBeams
  -> ConfToTimePlan()
  -> slot.beamIds
  -> .tr PLAN.active_beams
  -> OBC ToggleState()
```

Greedy score 主要是：

```text
score = backlogWeight * demandKbps + fairnessWeight * waiting_time
```

但目前每個 beam 都是 20 Mbps，因此 demand 都一樣。實際 beam selection 主要會變成公平輪流：

```text
uniform demand -> fairness / starvation rotation
```

如果 command 有：

```bash
--maxActiveBeams=2
```

則每個 slot 最多 active 2 個 beams。

## active_beams 寫在哪裡

`.tr` 的 header 是：

```text
record_type,time_s,sat_id,beam_id,event,plan_id,slot_idx,demand_kbps,duration_ms,active_beams,mapped,toggled
```

`active_beams` 只在 `PLAN` row 有實際意義。

Dynamic BSTP 產生 `conf.activeBeams` 後，`ConfToTimePlan()` 會轉成 `slot.beamIds`。接著在 `RunDynamicBstpCycle()` 裡把 `slot.beamIds` 組成 `|` 分隔字串：

```text
1|2
13|25
```

並寫進：

```text
PLAN,...,active_beams,...
```

例如：

```text
PLAN,0.000000,382,0,SLOT,1,0,0.000,10,13|25,0,0
```

其中 `13|25` 就是該 slot 的 active beams。

`1|2` 和 `2|1` 代表同一組 active beam set；順序只反映程式輸出的排列。

## .tr 三種 row 的意義

### DEMAND

代表 scheduler input。


```text
DEMAND,...,OFFERED_FWD,...,20000.000
```

意思：

```text
此 beam 在此 period 被feed 20 Mbps offered FWD demand
```

### PLAN

代表 Dynamic BSTP scheduler decision。

`PLAN` row 中：

- `beam_id=0`：因為 PLAN 是 slot-level，不是單一 beam event
- `event=SLOT`
- `slot_idx`：第幾個 slot
- `duration_ms`：slot 長度
- `active_beams`：該 slot 排到的 beams
- `demand_kbps=0` 是正常的，因為 demand 已經寫在 DEMAND row
- `mapped=0/toggled=0` 是正常的，因為 PLAN 還不是 OBC 執行結果

### EVENT

代表 OBC execution result。

OBC 收到 plan 後，會對每個 active beam 做 activate/deactivate，並寫出：

```text
EVENT,...,ACTIVATE,...,mapped=1,toggled=1
EVENT,...,DEACTIVATE,...,mapped=1,toggled=1
```

欄位意義：

- `mapped=1`：找到該 sat/beam 對應的 real SatNetDevice
- `toggled=1`：已呼叫 `ToggleState(true/false)` 成功

## timeplan.csv 與 .tr 的差別

`bh-timeplan_*.csv` 是 BHTP table/template，欄位是：

```text
slotIdx,startMs,durationMs,beamIds,beamPatterns,modcod,clusterIds
```

`.tr` 的 `PLAN.active_beams` 則是 Dynamic BSTP 實際每個 period 產生的 active beams。

目前在 dynamic mode 下，真正要看排程變化應以 `.tr` 的 `PLAN` row 為主；`timeplan.csv` 是初始建立的表格。

## metrics.csv 的目前意義

目前 `bh-metrics_*.csv` 是 BH/OBC control KPI，不是 packet throughput KPI。

主要反映：

- dwell time
- slot utilization
- drop ratio
- Jain fairness
- OBC/beam hopping control behavior



## Packet Trace

`sat-bh-phaseF-fwd.tr` 是 PacketSink Rx trace。

它不是 scheduler input，而是用來驗證 packet 是否真的收到：

```text
time r /NodeList/UT.../App/PacketSink/Rx bytes=N beam=B
```

目前 `FwdRxTrace()` 會用 UT index 對回 beam：

```text
utIdx % 25 -> beamId
```

這依賴 starlink25 建立 UT 時，每顆 helper sat 都是 25 個 UT 一組。

## 建議驗證 Command

短時間 smoke test：

```bash
./ns3 run "sat-bh-example --scenario=starlink25 --enableObc=1 --enableDynamicBstp=1 --enablePhaseF=1 --maxActiveBeams=2 --simTime=0.5 --warmUp=0.1 --helperSatList=382,404 --metricsFile=/tmp/offered2-metrics-test.csv --timePlanFile=/tmp/offered2-timeplan-test.csv --trafficTraceFile=/tmp/offered2-traffic-test.tr"
```

較完整輸出：

```bash
mkdir -p results/sat-bh-offered-fwd-demand-full
./ns3 run "sat-bh-example --scenario=starlink25 --enableObc=1 --enableDynamicBstp=1 --enablePhaseF=1 --maxActiveBeams=2 --simTime=120 --warmUp=1 --helperSatList=382,404,448,883,927,948 --metricsFile=results/sat-bh-offered-fwd-demand-full/bh-metrics.csv --timePlanFile=results/sat-bh-offered-fwd-demand-full/bh-timeplan.csv --trafficTraceFile=results/sat-bh-offered-fwd-demand-full/sat-bh-traffic.tr"
```

## 目前限制

目前已經有：

```text
Starlink constellation + ROI grid + UT position mapping + BH helper + Dynamic BSTP + OBC event trace
```

沒有完成：

```text
真實時間變化的 serving satellite gating
ISL routing path integration
根據 user distribution / grid hotspot 產生不同 beam demand
```

目前結果可以觀測：

```text
多 helper satellite 候選下，每顆 satellite 的 BH scheduler 如何把 25 beams 排成 active_beams，並且 OBC 是否成功 toggle real beam devices。
```

但不能解讀成：

```text
每個時間點只有唯一一顆真實可見 satellite 正在服務 Tokyo ROI。
```

下一步：

```text
visibility/serving-sat schedule -> 只有當 sat 是當下 serving sat 時，才 inject demand / emit plan / toggle beam。
```
