# gw2gw_e2e — GW0 (TW-Taipei) → GW2 (US-SanFrancisco)

**日期**: 2026-04-14 | **工具**: test-iridium-e2e | **時長**: 120s / slotInterval=60s / 3 slots

## 結果摘要

| 項目 | 結果 |
|------|------|
| ISL trace 連接 | 264 介面 / 132 unique links [OK] |
| GW2GW 資料平面交付 | 603,648 bytes (~1179 pkts) [PASS] |
| ISL drop rate | 0.000%（6,340,555 pkts）[PASS] |
| 動態路由重算 | slot1: 3/66 src, slot2: 17/66 src |
| wall time | 617.876 s |

## ISL 路由路徑（TW-Taipei → US-SanFrancisco）

| slot | time(s) | entry | ISL path | exit | isl_cost(s) | 備注 |
|------|---------|-------|----------|------|-------------|------|
| 0 | 0 | SAT15 | 15→14→25→36→37 | SAT37 | 0.043969 | |
| 1 | 60 | SAT15 | 15→14→25→36→37 | SAT37 | 0.046214 | |
| 2 | 120 | SAT15 | 15→14→13→2→1 | SAT1 | 0.047600 | ROUTE CHANGED |

## 觀察

- `feeder:*` / `service:*` obs 全為 0：`gw2gw_direct` 使用 `OnOffHelper`+`PacketSinkHelper`，不透過 `SatTrafficHelper`，不觸發 feeder/service trace
- 所有 264 個 ISL 介面均有流量通過（isl obs 全非零）
- 21/132 條 ISL 有非零 EMA load cost，負載感知機制已啟動
- Slot 2 路由切換：SF 側可見衛星由 SAT37 換為 SAT1，exit sat 改變，整條路徑重算

---

```
./ns3 run "test-iridium-e2e \
  --mode=gw2gw_e2e \
  --gwSrc=0 \
  --gwDst=2 \
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
  --enableIsl=1 \
  --enableServicelink=1 \
  --obsLogPath=Logs/0414_gw2gw_e2e_link_obs.csv \
  --obsInterval=10 \
  --obsDropAlertPct=50" | tee "Topology & ISL Routing/Outputs/E2E-PathType/gw2gw_e2e_g0g2_120s.log"
```
```
[CFG] pathType=gw2gw_e2e trafficProfile=none simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[OBS] log opened: Logs/0414_gw2gw_e2e_link_obs.csv
[OBS] traces connected:  feeder=66  service=66  isl=264
[OBS] snapshot interval=10s  dropAlertThresh=50%
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)

[E2E] pathType=gw2gw_e2e includesIsl=yes segments={feederlink=on, isl=on, servicelink=on} traffic={sharedEdge=off, islBg=off, gw2gwBg=off, gw2gwDirect=on}
[E2E][feederlink] enabled
[GW2GW_DIRECT] GW0_user=90.2.0.2 -> GW2_user=90.2.0.4 start=1s stop=119s
[E2E][isl] enabled
[E2E][isl] routing/transit enabled without extra ISL-only load generator
[E2E][servicelink] enabled
[E2E][servicelink] reusing upstream edge traffic installation
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

[CASE] gw2gw_e2e | gwSrc=0 gwDst=2
[CHKPT] 0s | PrecomputeGwRoutes: start | gws=2 pairs=1 slots=3
[GwRouting] slot=0 t=0s GW0[TW-Taipei]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=1 t=60s GW0[TW-Taipei]=1sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=2 t=120s GW0[TW-Taipei]=1sats GW2[US-SanFrancisco]=1sats
[CHKPT] 0s | PrecomputeGwRoutes: done | wall=0ms

=== GW-to-GW Route Report (v6) ===

  [TW-Taipei → US-SanFrancisco]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15->14->25->36->37                                37      0.043969        
1     60        15      15->14->25->36->37                                37      0.046214        
2     120       15      15->14->13->2->1                                  1       0.047600          <-- ROUTE CHANGED

  [US-SanFrancisco → TW-Taipei]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         37      37->36->25->14->15                                15      0.043969        
1     60        37      37->36->25->14->15                                15      0.046214        
2     120       1       1->2->13->14->15                                  15      0.047600          <-- ROUTE CHANGED

===================================

[CHKPT] 0s | ScheduleRoutingUpdates: 3 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=1ms recompute=0ms recomputedSrc=3

[GW2GW_DIRECT] === Delivery summary ===
  src: GW0 (90.2.0.2)
  dst: GW2 (90.2.0.4)
  received: 603648 bytes (~1179 pkts)
  [PASS] received>0, gateway-to-gateway path is active
======================================

[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=17

=== E2E Link Observability Final Summary ===
link                    rx_pkts   rx_bytes     drop_pkts  drop_rate(%)  avg_delay(ms)
------------------------------------------------------------------------------------
feeder:sat0             0         0            0          0.00          0.00
feeder:sat1             0         0            0          0.00          0.00
feeder:sat10            0         0            0          0.00          0.00
feeder:sat11            0         0            0          0.00          0.00
feeder:sat12            0         0            0          0.00          0.00
feeder:sat13            0         0            0          0.00          0.00
feeder:sat14            0         0            0          0.00          0.00
feeder:sat15            0         0            0          0.00          0.00
feeder:sat16            0         0            0          0.00          0.00
feeder:sat17            0         0            0          0.00          0.00
feeder:sat18            0         0            0          0.00          0.00
feeder:sat19            0         0            0          0.00          0.00
feeder:sat2             0         0            0          0.00          0.00
feeder:sat20            0         0            0          0.00          0.00
feeder:sat21            0         0            0          0.00          0.00
feeder:sat22            0         0            0          0.00          0.00
feeder:sat23            0         0            0          0.00          0.00
feeder:sat24            0         0            0          0.00          0.00
feeder:sat25            0         0            0          0.00          0.00
feeder:sat26            0         0            0          0.00          0.00
feeder:sat27            0         0            0          0.00          0.00
feeder:sat28            0         0            0          0.00          0.00
feeder:sat29            0         0            0          0.00          0.00
feeder:sat3             0         0            0          0.00          0.00
feeder:sat30            0         0            0          0.00          0.00
feeder:sat31            0         0            0          0.00          0.00
feeder:sat32            0         0            0          0.00          0.00
feeder:sat33            0         0            0          0.00          0.00
feeder:sat34            0         0            0          0.00          0.00
feeder:sat35            0         0            0          0.00          0.00
feeder:sat36            0         0            0          0.00          0.00
feeder:sat37            0         0            0          0.00          0.00
feeder:sat38            0         0            0          0.00          0.00
feeder:sat39            0         0            0          0.00          0.00
feeder:sat4             0         0            0          0.00          0.00
feeder:sat40            0         0            0          0.00          0.00
feeder:sat41            0         0            0          0.00          0.00
feeder:sat42            0         0            0          0.00          0.00
feeder:sat43            0         0            0          0.00          0.00
feeder:sat44            0         0            0          0.00          0.00
feeder:sat45            0         0            0          0.00          0.00
feeder:sat46            0         0            0          0.00          0.00
feeder:sat47            0         0            0          0.00          0.00
feeder:sat48            0         0            0          0.00          0.00
feeder:sat49            0         0            0          0.00          0.00
feeder:sat5             0         0            0          0.00          0.00
feeder:sat50            0         0            0          0.00          0.00
feeder:sat51            0         0            0          0.00          0.00
feeder:sat52            0         0            0          0.00          0.00
feeder:sat53            0         0            0          0.00          0.00
feeder:sat54            0         0            0          0.00          0.00
feeder:sat55            0         0            0          0.00          0.00
feeder:sat56            0         0            0          0.00          0.00
feeder:sat57            0         0            0          0.00          0.00
feeder:sat58            0         0            0          0.00          0.00
feeder:sat59            0         0            0          0.00          0.00
feeder:sat6             0         0            0          0.00          0.00
feeder:sat60            0         0            0          0.00          0.00
feeder:sat61            0         0            0          0.00          0.00
feeder:sat62            0         0            0          0.00          0.00
feeder:sat63            0         0            0          0.00          0.00
feeder:sat64            0         0            0          0.00          0.00
feeder:sat65            0         0            0          0.00          0.00
feeder:sat7             0         0            0          0.00          0.00
feeder:sat8             0         0            0          0.00          0.00
feeder:sat9             0         0            0          0.00          0.00
service:sat0            0         0            0          0.00          0.00
service:sat1            0         0            0          0.00          0.00
service:sat10           0         0            0          0.00          0.00
service:sat11           0         0            0          0.00          0.00
service:sat12           0         0            0          0.00          0.00
service:sat13           0         0            0          0.00          0.00
service:sat14           0         0            0          0.00          0.00
service:sat15           0         0            0          0.00          0.00
service:sat16           0         0            0          0.00          0.00
service:sat17           0         0            0          0.00          0.00
service:sat18           0         0            0          0.00          0.00
service:sat19           0         0            0          0.00          0.00
service:sat2            0         0            0          0.00          0.00
service:sat20           0         0            0          0.00          0.00
service:sat21           0         0            0          0.00          0.00
service:sat22           0         0            0          0.00          0.00
service:sat23           0         0            0          0.00          0.00
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
service:sat33           0         0            0          0.00          0.00
service:sat34           0         0            0          0.00          0.00
service:sat35           0         0            0          0.00          0.00
service:sat36           0         0            0          0.00          0.00
service:sat37           0         0            0          0.00          0.00
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
service:sat48           0         0            0          0.00          0.00
service:sat49           0         0            0          0.00          0.00
service:sat5            0         0            0          0.00          0.00
service:sat50           0         0            0          0.00          0.00
service:sat51           0         0            0          0.00          0.00
service:sat52           0         0            0          0.00          0.00
service:sat53           0         0            0          0.00          0.00
service:sat54           0         0            0          0.00          0.00
service:sat55           0         0            0          0.00          0.00
service:sat56           0         0            0          0.00          0.00
service:sat57           0         0            0          0.00          0.00
service:sat58           0         0            0          0.00          0.00
service:sat59           0         0            0          0.00          0.00
service:sat6            0         0            0          0.00          0.00
service:sat60           0         0            0          0.00          0.00
service:sat61           0         0            0          0.00          0.00
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
isl:1-0                 23973     23973        0          0.00          0.00
isl:1-12                23973     23973        0          0.00          0.00
isl:1-2                 24081     24081        0          0.00          0.00
isl:1-56                23973     23973        0          0.00          0.00
isl:10-0                23972     23972        0          0.00          0.00
isl:10-21               23972     23972        0          0.00          0.00
isl:10-65               23972     23972        0          0.00          0.00
isl:10-9                23972     23972        0          0.00          0.00
isl:11-0                23971     23971        0          0.00          0.00
isl:11-12               24296     24296        0          0.00          0.00
isl:11-21               23971     23971        0          0.00          0.00
isl:11-22               23971     23971        0          0.00          0.00
isl:12-1                23974     23974        0          0.00          0.00
isl:12-11               23974     23974        0          0.00          0.00
isl:12-13               24299     24299        0          0.00          0.00
isl:12-23               23974     23974        0          0.00          0.00
isl:13-12               23977     23977        0          0.00          0.00
isl:13-14               24953     24953        0          0.00          0.00
isl:13-2                23977     23977        0          0.00          0.00
isl:13-24               23977     23977        0          0.00          0.00
isl:14-13               23976     23976        0          0.00          0.00
isl:14-15               26465     26465        0          0.00          0.00
isl:14-25               23976     23976        0          0.00          0.00
isl:14-3                23976     23976        0          0.00          0.00
isl:15-14               23975     23975        0          0.00          0.00
isl:15-16               23975     23975        0          0.00          0.00
isl:15-26               23975     23975        0          0.00          0.00
isl:15-4                23975     23975        0          0.00          0.00
isl:16-15               24189     24189        0          0.00          0.00
isl:16-17               23973     23973        0          0.00          0.00
isl:16-27               23973     23973        0          0.00          0.00
isl:16-5                23973     23973        0          0.00          0.00
isl:17-16               24187     24187        0          0.00          0.00
isl:17-18               23971     23971        0          0.00          0.00
isl:17-28               23971     23971        0          0.00          0.00
isl:17-6                23971     23971        0          0.00          0.00
isl:18-17               24027     24027        0          0.00          0.00
isl:18-19               23973     23973        0          0.00          0.00
isl:18-29               23973     23973        0          0.00          0.00
isl:18-7                23973     23973        0          0.00          0.00
isl:19-18               24029     24029        0          0.00          0.00
isl:19-20               23975     23975        0          0.00          0.00
isl:19-30               23975     23975        0          0.00          0.00
isl:19-8                23975     23975        0          0.00          0.00
isl:2-1                 23976     23976        0          0.00          0.00
isl:2-13                24084     24084        0          0.00          0.00
isl:2-3                 23976     23976        0          0.00          0.00
isl:2-57                23976     23976        0          0.00          0.00
isl:20-19               24027     24027        0          0.00          0.00
isl:20-21               23973     23973        0          0.00          0.00
isl:20-31               23973     23973        0          0.00          0.00
isl:20-9                23973     23973        0          0.00          0.00
isl:21-10               23970     23970        0          0.00          0.00
isl:21-11               24295     24295        0          0.00          0.00
isl:21-20               23970     23970        0          0.00          0.00
isl:21-32               23970     23970        0          0.00          0.00
isl:22-11               23971     23971        0          0.00          0.00
isl:22-23               24297     24297        0          0.00          0.00
isl:22-32               23971     23971        0          0.00          0.00
isl:22-33               23971     23971        0          0.00          0.00
isl:23-12               23973     23973        0          0.00          0.00
isl:23-22               23973     23973        0          0.00          0.00
isl:23-24               24462     24462        0          0.00          0.00
isl:23-34               23973     23973        0          0.00          0.00
isl:24-13               24518     24518        0          0.00          0.00
isl:24-23               23975     23975        0          0.00          0.00
isl:24-25               23975     23975        0          0.00          0.00
isl:24-35               23975     23975        0          0.00          0.00
isl:25-14               24516     24516        0          0.00          0.00
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
isl:3-14                24950     24950        0          0.00          0.00
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
isl:32-21               23968     23968        0          0.00          0.00
isl:32-22               23968     23968        0          0.00          0.00
isl:32-31               23968     23968        0          0.00          0.00
isl:32-43               23968     23968        0          0.00          0.00
isl:33-22               23968     23968        0          0.00          0.00
isl:33-34               24022     24022        0          0.00          0.00
isl:33-43               23968     23968        0          0.00          0.00
isl:33-44               23968     23968        0          0.00          0.00
isl:34-23               23972     23972        0          0.00          0.00
isl:34-33               23972     23972        0          0.00          0.00
isl:34-35               24026     24026        0          0.00          0.00
isl:34-45               23972     23972        0          0.00          0.00
isl:35-24               24028     24028        0          0.00          0.00
isl:35-34               23974     23974        0          0.00          0.00
isl:35-36               23974     23974        0          0.00          0.00
isl:35-46               23974     23974        0          0.00          0.00
isl:36-25               24516     24516        0          0.00          0.00
isl:36-35               23976     23976        0          0.00          0.00
isl:36-37               23976     23976        0          0.00          0.00
isl:36-47               23976     23976        0          0.00          0.00
isl:37-26               23974     23974        0          0.00          0.00
isl:37-36               24298     24298        0          0.00          0.00
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
isl:4-3                 24676     24676        0          0.00          0.00
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
isl:43-32               23967     23967        0          0.00          0.00
isl:43-33               23967     23967        0          0.00          0.00
isl:43-42               23967     23967        0          0.00          0.00
isl:43-54               23967     23967        0          0.00          0.00
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
isl:47-36               24193     24193        0          0.00          0.00
isl:47-46               23977     23977        0          0.00          0.00
isl:47-48               23977     23977        0          0.00          0.00
isl:47-58               23977     23977        0          0.00          0.00
isl:48-37               23976     23976        0          0.00          0.00
isl:48-47               24192     24192        0          0.00          0.00
isl:48-49               23976     23976        0          0.00          0.00
isl:48-59               23976     23976        0          0.00          0.00
isl:49-38               23973     23973        0          0.00          0.00
isl:49-48               23973     23973        0          0.00          0.00
isl:49-50               23973     23973        0          0.00          0.00
isl:49-60               23973     23973        0          0.00          0.00
isl:5-16                23972     23972        0          0.00          0.00
isl:5-4                 24674     24674        0          0.00          0.00
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
isl:53-42               23969     23969        0          0.00          0.00
isl:53-52               23969     23969        0          0.00          0.00
isl:53-54               23969     23969        0          0.00          0.00
isl:53-64               23969     23969        0          0.00          0.00
isl:54-43               23966     23966        0          0.00          0.00
isl:54-44               23966     23966        0          0.00          0.00
isl:54-53               23966     23966        0          0.00          0.00
isl:54-65               23966     23966        0          0.00          0.00
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
isl:58-3                24248     24248        0          0.00          0.00
isl:58-47               23977     23977        0          0.00          0.00
isl:58-57               23977     23977        0          0.00          0.00
isl:58-59               23977     23977        0          0.00          0.00
isl:59-4                23976     23976        0          0.00          0.00
isl:59-48               23976     23976        0          0.00          0.00
isl:59-58               24247     24247        0          0.00          0.00
isl:59-60               23976     23976        0          0.00          0.00
isl:6-17                23972     23972        0          0.00          0.00
isl:6-5                 23972     23972        0          0.00          0.00
isl:6-61                23972     23972        0          0.00          0.00
isl:6-7                 23972     23972        0          0.00          0.00
isl:60-49               23974     23974        0          0.00          0.00
isl:60-5                23974     23974        0          0.00          0.00
isl:60-59               24029     24029        0          0.00          0.00
isl:60-61               23974     23974        0          0.00          0.00
isl:61-50               23972     23972        0          0.00          0.00
isl:61-6                23972     23972        0          0.00          0.00
isl:61-60               24027     24027        0          0.00          0.00
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
isl:65-10               23970     23970        0          0.00          0.00
isl:65-54               23970     23970        0          0.00          0.00
isl:65-55               23970     23970        0          0.00          0.00
isl:65-64               23970     23970        0          0.00          0.00
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
Log: Logs/0414_gw2gw_e2e_link_obs.csv
=============================================


=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.00      0           0               0               NO          
1     60.00     1           0               3               YES         
2     120.00    0           0               17              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
16      8       9       0.0000          0.2690          
18      9       10      0.2690          0.0000          
19      9       20      0.2690          0.0000          
55      27      38      0.0000          0.3542          
74      37      38      0.0000          0.3542          
76      38      39      0.3542          0.0000          
77      38      49      0.3542          0.0000          
79      39      50      0.0000          0.2690          
98      49      50      0.0000          0.2690          
100     50      51      0.2690          0.0000          
101     50      61      0.2690          0.0000          
105     52      63      0.0000          0.2258          
107     53      64      0.0000          0.2690          
109     54      65      0.0000          0.2234          
124     62      63      0.0000          0.2258          
126     63      64      0.2258          0.2690          
127     8       63      0.0000          0.2258          
128     64      65      0.2690          0.2234          
129     9       64      0.2690          0.2690          
130     55      65      0.0000          0.2234          
131     10      65      0.0000          0.2234          
Loaded ISL links: 21 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
  (all ISLs: 0 drops)
TOTAL: 6340555 pkts, 0 dropped | drop_rate=0.000% | success_rate=100.000%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 617.876 s
Event count:     0
```