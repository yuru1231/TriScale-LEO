# gw2sat — GW0 (TW-Taipei) feederlink 上行驗證

**日期**: 2026-04-14 | **工具**: test-iridium-e2e | **時長**: 120s / slotInterval=60s / 3 slots
**段配置**: feederlink=on, isl=off, servicelink=off

## 結果摘要

| 項目 | 結果 |
|------|------|
| ISL trace 連接 | 264 介面 / 132 unique links [OK] |
| feeder link 活躍衛星 | sat15（32087pkts, 4.62ms）、sat1（13082pkts, 32.81ms）等 13 顆有流量 |
| feeder drop rate | 0.00%（全衛星無丟棄）|
| OBS 事件 | sat61 service throughput=0（預期：gw2sat 無 service link）|
| 動態路由重算 | slot1: 3/66, slot2: 16/66 |

## 觀察

- 主要接入衛星 SAT15（delay=4.62ms）為 TW-Taipei 主覆蓋衛星，與 gw2gw baseline 一致
- sat61 service link 事件為 false alarm：gw2sat 本來就無 service 流量，非鏈路故障
- feeder link 無 drop，上行鏈路品質正常

---

```
./ns3 run "test-iridium-e2e \
  --mode=gw2sat \
  --gwId=0 \
  --simTime=120 \
  --slotInterval=60 \
  --beamId=1 \
  --islMaxDistKm=5000 \
  --islRateMbps=10 \
  --emaAlpha=0.3 \
  --changeThresh=0.1 \
  --cooldownRatio=0.5 \
  --elevMinDeg=5 \
  --enableFeederlink=1 \
  --obsLogPath=Logs/0414_gw2sat_link_obs.csv \
  --obsInterval=10 \
  --obsDropAlertPct=50" | tee "Topology & ISL Routing/Outputs/E2E-PathType/gw2sat_120s.log"
```
```
[CFG] pathType=gw2sat trafficProfile=none simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[OBS] log opened: Logs/0414_gw2sat_link_obs.csv
[OBS] traces connected:  feeder=66  service=66  isl=264
[OBS] snapshot interval=10s  dropAlertThresh=50%
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)

[E2E] pathType=gw2sat includesIsl=no segments={feederlink=on, isl=off, servicelink=off} traffic={sharedEdge=on, islBg=off, gw2gwBg=off, gw2gwDirect=off}
[E2E][feederlink] enabled
[TRAFFIC][feederlink] gwId=0 gwUsers=1 utUsers=91 start=1s stop=119s
[TRAFFIC][feederlink] FWD installed: interval=100ms pktSize=1500B rate~120 kbps/flow
[TRAFFIC][feederlink] RTN installed: interval=500ms pktSize=512B rate~8.192 kbps/flow
[E2E][isl] disabled
[E2E][servicelink] disabled
[CHKPT] 0s | Initialize: start
[CHKPT] 0s | LoadISLDefs: start | path=/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: start
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | Initialize: done | satellites=66 isls=132
[CHKPT] 0s | PrecomputeAllTables: start | slots=3
[CHKPT] PrecomputeAllTables: slot=0 t=0s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=1 t=60s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=2 t=120s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=1ms

[CASE] gw2sat | gwId=0 | feederlink_up only | isl_cost=N/A
[CHKPT] 0s | ScheduleRoutingUpdates: 3 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=1ms recompute=0ms recomputedSrc=3
[OBS][EVENT] t=80.0s  [service] sat61  window_throughput=0 kbps  => POSSIBLE LINK FAILURE
[OBS][EVENT] t=90.0s  [service] sat61  window_throughput=0 kbps  => POSSIBLE LINK FAILURE
[OBS][EVENT] t=100.0s  [service] sat61  window_throughput=0 kbps  => POSSIBLE LINK FAILURE
[OBS][EVENT] t=110.0s  [service] sat61  window_throughput=0 kbps  => POSSIBLE LINK FAILURE
[CHKPT] 120.0s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120.0s | RecomputeAffectedRoutes: slot=2 recomputed=16/66 wall=3ms
[CHKPT] 120.0s | ApplyRoutingTable: slot=2 t=120.0s | apply=4ms recompute=3ms recomputedSrc=16

=== E2E Link Observability Final Summary ===
link                    rx_pkts   rx_bytes     drop_pkts  drop_rate(%)  avg_delay(ms)
------------------------------------------------------------------------------------
feeder:sat0             0         0            0          0.00          0.00
feeder:sat1             13082     20415814     0          0.00          32.81
feeder:sat10            0         0            0          0.00          0.00
feeder:sat11            0         0            0          0.00          0.00
feeder:sat12            0         0            0          0.00          0.00
feeder:sat13            0         0            0          0.00          0.00
feeder:sat14            0         0            0          0.00          0.00
feeder:sat15            32087     49235488     0          0.00          4.62
feeder:sat16            0         0            0          0.00          0.00
feeder:sat17            3575      4815477      0          0.00          25.51
feeder:sat18            0         0            0          0.00          0.00
feeder:sat19            0         0            0          0.00          0.00
feeder:sat2             0         0            0          0.00          0.00
feeder:sat20            2387      2975372      0          0.00          44.00
feeder:sat21            4763      6732917      0          0.00          44.00
feeder:sat22            4763      6735233      0          0.00          37.25
feeder:sat23            9515      14532458     0          0.00          31.56
feeder:sat24            3575      4972209      0          0.00          24.22
feeder:sat25            0         0            0          0.00          0.00
feeder:sat26            0         0            0          0.00          0.00
feeder:sat27            0         0            0          0.00          0.00
feeder:sat28            0         0            0          0.00          0.00
feeder:sat29            0         0            0          0.00          0.00
feeder:sat3             0         0            0          0.00          0.00
feeder:sat30            0         0            0          0.00          0.00
feeder:sat31            0         0            0          0.00          0.00
feeder:sat32            0         0            0          0.00          0.00
feeder:sat33            9515      14636714     0          0.00          20.93
feeder:sat34            0         0            0          0.00          0.00
feeder:sat35            0         0            0          0.00          0.00
feeder:sat36            0         0            0          0.00          0.00
feeder:sat37            4763      6730613      0          0.00          32.25
feeder:sat38            3575      4972209      0          0.00          37.83
feeder:sat39            0         0            0          0.00          0.00
feeder:sat4             3575      4972209      0          0.00          18.00
feeder:sat40            0         0            0          0.00          0.00
feeder:sat41            0         0            0          0.00          0.00
feeder:sat42            0         0            0          0.00          0.00
feeder:sat43            0         0            0          0.00          0.00
feeder:sat44            0         0            0          0.00          0.00
feeder:sat45            0         0            0          0.00          0.00
feeder:sat46            0         0            0          0.00          0.00
feeder:sat47            0         0            0          0.00          0.00
feeder:sat48            3575      4765329      0          0.00          33.50
feeder:sat49            0         0            0          0.00          0.00
feeder:sat5             10703     16087894     0          0.00          22.17
feeder:sat50            2387      3028652      0          0.00          44.00
feeder:sat51            0         0            0          0.00          0.00
feeder:sat52            0         0            0          0.00          0.00
feeder:sat53            0         0            0          0.00          0.00
feeder:sat54            0         0            0          0.00          0.00
feeder:sat55            0         0            0          0.00          0.00
feeder:sat56            0         0            0          0.00          0.00
feeder:sat57            0         0            0          0.00          0.00
feeder:sat58            3575      4972209      0          0.00          26.50
feeder:sat59            3575      4765329      0          0.00          32.50
feeder:sat6             2387      3028652      0          0.00          27.50
feeder:sat60            3575      4972209      0          0.00          37.83
feeder:sat61            2387      2977676      0          0.00          40.00
feeder:sat62            0         0            0          0.00          0.00
feeder:sat63            0         0            0          0.00          0.00
feeder:sat64            0         0            0          0.00          0.00
feeder:sat65            0         0            0          0.00          0.00
feeder:sat7             0         0            0          0.00          0.00
feeder:sat8             0         0            0          0.00          0.00
feeder:sat9             0         0            0          0.00          0.00
service:sat0            0         0            0          0.00          0.00
service:sat1            492       129962       0          0.00          47.83
service:sat10           0         0            0          0.00          0.00
service:sat11           0         0            0          0.00          0.00
service:sat12           0         0            0          0.00          0.00
service:sat13           0         0            0          0.00          0.00
service:sat14           0         0            0          0.00          0.00
service:sat15           10713     2911180      0          0.00          1396.82
service:sat16           0         0            0          0.00          0.00
service:sat17           878       259286       0          0.00          4828.47
service:sat18           0         0            0          0.00          0.00
service:sat19           0         0            0          0.00          0.00
service:sat2            0         0            0          0.00          0.00
service:sat20           387       129330       0          0.00          10854.38
service:sat21           1592      453678       0          0.00          2701.35
service:sat22           1457      389858       0          0.00          92.96
service:sat23           1014      323648       0          0.00          8299.01
service:sat24           0         0            0          0.00          0.00
service:sat25           0         0            0          0.00          0.00
service:sat26           0         0            0          0.00          0.00
service:sat27           0         0            0          0.00          0.00
service:sat28           0         0            0          0.00          0.00
service:sat29           0         0            0          0.00          0.00
service:sat3            0         0            0          0.00          0.00
service:sat30           0         0            0          0.00          0.00
service:sat31           0         0            0          0.00          0.00
service:sat32           0         0            0          0.00          0.00
service:sat33           387       129330       0          0.00          10856.10
service:sat34           0         0            0          0.00          0.00
service:sat35           0         0            0          0.00          0.00
service:sat36           0         0            0          0.00          0.00
service:sat37           1739      518020       0          0.00          4880.70
service:sat38           0         0            0          0.00          0.00
service:sat39           0         0            0          0.00          0.00
service:sat4            0         0            0          0.00          0.00
service:sat40           0         0            0          0.00          0.00
service:sat41           0         0            0          0.00          0.00
service:sat42           0         0            0          0.00          0.00
service:sat43           0         0            0          0.00          0.00
service:sat44           0         0            0          0.00          0.00
service:sat45           0         0            0          0.00          0.00
service:sat46           0         0            0          0.00          0.00
service:sat47           0         0            0          0.00          0.00
service:sat48           980       259900       0          0.00          100.77
service:sat49           0         0            0          0.00          0.00
service:sat5            3313      908496       0          0.00          1340.39
service:sat50           0         0            0          0.00          0.00
service:sat51           0         0            0          0.00          0.00
service:sat52           0         0            0          0.00          0.00
service:sat53           0         0            0          0.00          0.00
service:sat54           0         0            0          0.00          0.00
service:sat55           0         0            0          0.00          0.00
service:sat56           0         0            0          0.00          0.00
service:sat57           0         0            0          0.00          0.00
service:sat58           0         0            0          0.00          0.00
service:sat59           980       259900       0          0.00          101.05
service:sat6            0         0            0          0.00          0.00
service:sat60           0         0            0          0.00          0.00
service:sat61           239       64982        0          0.00          54.03
service:sat62           0         0            0          0.00          0.00
service:sat63           0         0            0          0.00          0.00
service:sat64           0         0            0          0.00          0.00
service:sat65           0         0            0          0.00          0.00
service:sat7            0         0            0          0.00          0.00
service:sat8            0         0            0          0.00          0.00
service:sat9            0         0            0          0.00          0.00
isl:0-1                 23973     23973        0          0.00          0.00
isl:0-10                23973     23973        0          0.00          0.00
isl:0-11                23973     23973        0          0.00          0.00
isl:0-55                23973     23973        0          0.00          0.00
isl:1-0                 23974     23974        0          0.00          0.00
isl:1-12                23974     23974        0          0.00          0.00
isl:1-2                 24466     24466        0          0.00          0.00
isl:1-56                23974     23974        0          0.00          0.00
isl:10-0                23972     23972        0          0.00          0.00
isl:10-21               23972     23972        0          0.00          0.00
isl:10-65               23972     23972        0          0.00          0.00
isl:10-9                23972     23972        0          0.00          0.00
isl:11-0                23971     23971        0          0.00          0.00
isl:11-12               25563     25563        0          0.00          0.00
isl:11-21               23971     23971        0          0.00          0.00
isl:11-22               23971     23971        0          0.00          0.00
isl:12-1                23974     23974        0          0.00          0.00
isl:12-11               23974     23974        0          0.00          0.00
isl:12-13               25566     25566        0          0.00          0.00
isl:12-23               23974     23974        0          0.00          0.00
isl:13-12               23977     23977        0          0.00          0.00
isl:13-14               28919     28919        0          0.00          0.00
isl:13-2                23977     23977        0          0.00          0.00
isl:13-24               23977     23977        0          0.00          0.00
isl:14-13               23976     23976        0          0.00          0.00
isl:14-15               36169     36169        0          0.00          0.00
isl:14-25               23976     23976        0          0.00          0.00
isl:14-3                23976     23976        0          0.00          0.00
isl:15-14               23975     23975        0          0.00          0.00
isl:15-16               23975     23975        0          0.00          0.00
isl:15-26               23975     23975        0          0.00          0.00
isl:15-4                23975     23975        0          0.00          0.00
isl:16-15               25238     25238        0          0.00          0.00
isl:16-17               23973     23973        0          0.00          0.00
isl:16-27               23973     23973        0          0.00          0.00
isl:16-5                23973     23973        0          0.00          0.00
isl:17-16               25236     25236        0          0.00          0.00
isl:17-18               23971     23971        0          0.00          0.00
isl:17-28               23971     23971        0          0.00          0.00
isl:17-6                23971     23971        0          0.00          0.00
isl:18-17               24360     24360        0          0.00          0.00
isl:18-19               23973     23973        0          0.00          0.00
isl:18-29               23973     23973        0          0.00          0.00
isl:18-7                23973     23973        0          0.00          0.00
isl:19-18               24363     24363        0          0.00          0.00
isl:19-20               23976     23976        0          0.00          0.00
isl:19-30               23976     23976        0          0.00          0.00
isl:19-8                23976     23976        0          0.00          0.00
isl:2-1                 23976     23976        0          0.00          0.00
isl:2-13                24468     24468        0          0.00          0.00
isl:2-3                 23976     23976        0          0.00          0.00
isl:2-57                23976     23976        0          0.00          0.00
isl:20-19               24360     24360        0          0.00          0.00
isl:20-21               23973     23973        0          0.00          0.00
isl:20-31               23973     23973        0          0.00          0.00
isl:20-9                23973     23973        0          0.00          0.00
isl:21-10               23971     23971        0          0.00          0.00
isl:21-11               25563     25563        0          0.00          0.00
isl:21-20               23971     23971        0          0.00          0.00
isl:21-32               23971     23971        0          0.00          0.00
isl:22-11               23971     23971        0          0.00          0.00
isl:22-23               25428     25428        0          0.00          0.00
isl:22-32               23971     23971        0          0.00          0.00
isl:22-33               23971     23971        0          0.00          0.00
isl:23-12               23973     23973        0          0.00          0.00
isl:23-22               23973     23973        0          0.00          0.00
isl:23-24               26444     26444        0          0.00          0.00
isl:23-34               23973     23973        0          0.00          0.00
isl:24-13               26833     26833        0          0.00          0.00
isl:24-23               23975     23975        0          0.00          0.00
isl:24-25               23975     23975        0          0.00          0.00
isl:24-35               23975     23975        0          0.00          0.00
isl:25-14               26695     26695        0          0.00          0.00
isl:25-24               23976     23976        0          0.00          0.00
isl:25-26               23976     23976        0          0.00          0.00
isl:25-36               23976     23976        0          0.00          0.00
isl:26-15               23975     23975        0          0.00          0.00
isl:26-25               23975     23975        0          0.00          0.00
isl:26-27               23975     23975        0          0.00          0.00
isl:26-37               23975     23975        0          0.00          0.00
isl:27-16               23971     23971        0          0.00          0.00
isl:27-26               23971     23971        0          0.00          0.00
isl:27-28               23971     23971        0          0.00          0.00
isl:27-38               23971     23971        0          0.00          0.00
isl:28-17               23970     23970        0          0.00          0.00
isl:28-27               23970     23970        0          0.00          0.00
isl:28-29               23970     23970        0          0.00          0.00
isl:28-39               23970     23970        0          0.00          0.00
isl:29-18               23974     23974        0          0.00          0.00
isl:29-28               23974     23974        0          0.00          0.00
isl:29-30               23974     23974        0          0.00          0.00
isl:29-40               23974     23974        0          0.00          0.00
isl:3-14                28509     28509        0          0.00          0.00
isl:3-2                 23977     23977        0          0.00          0.00
isl:3-4                 23977     23977        0          0.00          0.00
isl:3-58                23977     23977        0          0.00          0.00
isl:30-19               23976     23976        0          0.00          0.00
isl:30-29               23976     23976        0          0.00          0.00
isl:30-31               23976     23976        0          0.00          0.00
isl:30-41               23976     23976        0          0.00          0.00
isl:31-20               23970     23970        0          0.00          0.00
isl:31-30               23970     23970        0          0.00          0.00
isl:31-32               23970     23970        0          0.00          0.00
isl:31-42               23970     23970        0          0.00          0.00
isl:32-21               23970     23970        0          0.00          0.00
isl:32-22               23970     23970        0          0.00          0.00
isl:32-31               23970     23970        0          0.00          0.00
isl:32-43               23970     23970        0          0.00          0.00
isl:33-22               23968     23968        0          0.00          0.00
isl:33-34               24355     24355        0          0.00          0.00
isl:33-43               23968     23968        0          0.00          0.00
isl:33-44               23968     23968        0          0.00          0.00
isl:34-23               23972     23972        0          0.00          0.00
isl:34-33               23972     23972        0          0.00          0.00
isl:34-35               24359     24359        0          0.00          0.00
isl:34-45               23972     23972        0          0.00          0.00
isl:35-24               24361     24361        0          0.00          0.00
isl:35-34               23974     23974        0          0.00          0.00
isl:35-36               23974     23974        0          0.00          0.00
isl:35-46               23974     23974        0          0.00          0.00
isl:36-25               26695     26695        0          0.00          0.00
isl:36-35               23976     23976        0          0.00          0.00
isl:36-37               23976     23976        0          0.00          0.00
isl:36-47               23976     23976        0          0.00          0.00
isl:37-26               23974     23974        0          0.00          0.00
isl:37-36               25713     25713        0          0.00          0.00
isl:37-38               23974     23974        0          0.00          0.00
isl:37-48               23974     23974        0          0.00          0.00
isl:38-27               23975     23975        0          0.00          0.00
isl:38-37               23975     23975        0          0.00          0.00
isl:38-39               23975     23975        0          0.00          0.00
isl:38-49               23975     23975        0          0.00          0.00
isl:39-28               23973     23973        0          0.00          0.00
isl:39-38               23973     23973        0          0.00          0.00
isl:39-40               23973     23973        0          0.00          0.00
isl:39-50               23973     23973        0          0.00          0.00
isl:4-15                23974     23974        0          0.00          0.00
isl:4-3                 27287     27287        0          0.00          0.00
isl:4-5                 23974     23974        0          0.00          0.00
isl:4-59                23974     23974        0          0.00          0.00
isl:40-29               23972     23972        0          0.00          0.00
isl:40-39               23972     23972        0          0.00          0.00
isl:40-41               23972     23972        0          0.00          0.00
isl:40-51               23972     23972        0          0.00          0.00
isl:41-30               23976     23976        0          0.00          0.00
isl:41-40               23976     23976        0          0.00          0.00
isl:41-42               23976     23976        0          0.00          0.00
isl:41-52               23976     23976        0          0.00          0.00
isl:42-31               23973     23973        0          0.00          0.00
isl:42-41               23973     23973        0          0.00          0.00
isl:42-43               23973     23973        0          0.00          0.00
isl:42-53               23973     23973        0          0.00          0.00
isl:43-32               23968     23968        0          0.00          0.00
isl:43-33               23968     23968        0          0.00          0.00
isl:43-42               23968     23968        0          0.00          0.00
isl:43-54               23968     23968        0          0.00          0.00
isl:44-33               23971     23971        0          0.00          0.00
isl:44-45               23971     23971        0          0.00          0.00
isl:44-54               23971     23971        0          0.00          0.00
isl:44-55               23971     23971        0          0.00          0.00
isl:45-34               23972     23972        0          0.00          0.00
isl:45-44               23972     23972        0          0.00          0.00
isl:45-46               23972     23972        0          0.00          0.00
isl:45-56               23972     23972        0          0.00          0.00
isl:46-35               23974     23974        0          0.00          0.00
isl:46-45               23974     23974        0          0.00          0.00
isl:46-47               23974     23974        0          0.00          0.00
isl:46-57               23974     23974        0          0.00          0.00
isl:47-36               24957     24957        0          0.00          0.00
isl:47-46               23977     23977        0          0.00          0.00
isl:47-48               23977     23977        0          0.00          0.00
isl:47-58               23977     23977        0          0.00          0.00
isl:48-37               23976     23976        0          0.00          0.00
isl:48-47               24956     24956        0          0.00          0.00
isl:48-49               23976     23976        0          0.00          0.00
isl:48-59               23976     23976        0          0.00          0.00
isl:49-38               23973     23973        0          0.00          0.00
isl:49-48               23973     23973        0          0.00          0.00
isl:49-50               23973     23973        0          0.00          0.00
isl:49-60               23973     23973        0          0.00          0.00
isl:5-16                23972     23972        0          0.00          0.00
isl:5-4                 27285     27285        0          0.00          0.00
isl:5-6                 23972     23972        0          0.00          0.00
isl:5-60                23972     23972        0          0.00          0.00
isl:50-39               23975     23975        0          0.00          0.00
isl:50-49               23975     23975        0          0.00          0.00
isl:50-51               23975     23975        0          0.00          0.00
isl:50-61               23975     23975        0          0.00          0.00
isl:51-40               23975     23975        0          0.00          0.00
isl:51-50               23975     23975        0          0.00          0.00
isl:51-52               23975     23975        0          0.00          0.00
isl:51-62               23975     23975        0          0.00          0.00
isl:52-41               23976     23976        0          0.00          0.00
isl:52-51               23976     23976        0          0.00          0.00
isl:52-53               23976     23976        0          0.00          0.00
isl:52-63               23976     23976        0          0.00          0.00
isl:53-42               23970     23970        0          0.00          0.00
isl:53-52               23970     23970        0          0.00          0.00
isl:53-54               23970     23970        0          0.00          0.00
isl:53-64               23970     23970        0          0.00          0.00
isl:54-43               23967     23967        0          0.00          0.00
isl:54-44               23967     23967        0          0.00          0.00
isl:54-53               23967     23967        0          0.00          0.00
isl:54-65               23967     23967        0          0.00          0.00
isl:55-0                23967     23967        0          0.00          0.00
isl:55-44               23967     23967        0          0.00          0.00
isl:55-56               23967     23967        0          0.00          0.00
isl:55-65               23967     23967        0          0.00          0.00
isl:56-1                23972     23972        0          0.00          0.00
isl:56-45               23972     23972        0          0.00          0.00
isl:56-55               23972     23972        0          0.00          0.00
isl:56-57               23972     23972        0          0.00          0.00
isl:57-2                23976     23976        0          0.00          0.00
isl:57-46               23976     23976        0          0.00          0.00
isl:57-56               23976     23976        0          0.00          0.00
isl:57-58               23976     23976        0          0.00          0.00
isl:58-3                25196     25196        0          0.00          0.00
isl:58-47               23977     23977        0          0.00          0.00
isl:58-57               23977     23977        0          0.00          0.00
isl:58-59               23977     23977        0          0.00          0.00
isl:59-4                23976     23976        0          0.00          0.00
isl:59-48               23976     23976        0          0.00          0.00
isl:59-58               25195     25195        0          0.00          0.00
isl:59-60               23976     23976        0          0.00          0.00
isl:6-17                23972     23972        0          0.00          0.00
isl:6-5                 23972     23972        0          0.00          0.00
isl:6-61                23972     23972        0          0.00          0.00
isl:6-7                 23972     23972        0          0.00          0.00
isl:60-49               23975     23975        0          0.00          0.00
isl:60-5                23975     23975        0          0.00          0.00
isl:60-59               24214     24214        0          0.00          0.00
isl:60-61               23975     23975        0          0.00          0.00
isl:61-50               23972     23972        0          0.00          0.00
isl:61-6                23972     23972        0          0.00          0.00
isl:61-60               24211     24211        0          0.00          0.00
isl:61-62               23972     23972        0          0.00          0.00
isl:62-51               23975     23975        0          0.00          0.00
isl:62-61               23975     23975        0          0.00          0.00
isl:62-63               23975     23975        0          0.00          0.00
isl:62-7                23975     23975        0          0.00          0.00
isl:63-52               23977     23977        0          0.00          0.00
isl:63-62               23977     23977        0          0.00          0.00
isl:63-64               23977     23977        0          0.00          0.00
isl:63-8                23977     23977        0          0.00          0.00
isl:64-53               23973     23973        0          0.00          0.00
isl:64-63               23973     23973        0          0.00          0.00
isl:64-65               23973     23973        0          0.00          0.00
isl:64-9                23973     23973        0          0.00          0.00
isl:65-10               23972     23972        0          0.00          0.00
isl:65-54               23972     23972        0          0.00          0.00
isl:65-55               23972     23972        0          0.00          0.00
isl:65-64               23972     23972        0          0.00          0.00
isl:7-18                23973     23973        0          0.00          0.00
isl:7-6                 23973     23973        0          0.00          0.00
isl:7-62                23973     23973        0          0.00          0.00
isl:7-8                 23973     23973        0          0.00          0.00
isl:8-19                23975     23975        0          0.00          0.00
isl:8-63                23975     23975        0          0.00          0.00
isl:8-7                 23975     23975        0          0.00          0.00
isl:8-9                 23975     23975        0          0.00          0.00
isl:9-10                23973     23973        0          0.00          0.00
isl:9-20                23973     23973        0          0.00          0.00
isl:9-64                23973     23973        0          0.00          0.00
isl:9-8                 23973     23973        0          0.00          0.00
------------------------------------------------------------------------------------
Log: Logs/0414_gw2sat_link_obs.csv
=============================================


=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.00      0           0               0               NO          
1     60.00     1           0               3               YES         
2     120.00    4           3               16              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
16      8       9       0.0000          0.2690          
17      8       19      0.0000          0.2690          
18      9       10      0.2690          0.0000          
19      9       20      0.2690          0.0000          
36      18      19      0.0000          0.2690          
38      19      20      0.2690          0.0000          
39      19      30      0.2690          0.0000          
55      27      38      0.0000          0.2434          
74      37      38      0.0000          0.2434          
76      38      39      0.2434          0.0000          
77      38      49      0.2434          0.0000          
87      43      54      0.0000          0.4517          
106     53      54      0.0000          0.4517          
108     44      54      0.0000          0.4517          
109     54      65      0.4517          0.1883          
128     64      65      0.0000          0.1883          
129     9       64      0.2690          0.0000          
130     55      65      0.0000          0.1883          
131     10      65      0.0000          0.1883          
Loaded ISL links: 19 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
  (all ISLs: 0 drops)
TOTAL: 6386708 pkts, 0 dropped | drop_rate=0.000% | success_rate=100.000%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 921.713 s
Event count:     0
```