## 1.GW_TW-GW_JP
```
./ns3 run "scratch/test-iridium \
  --mode=gw2gw \
  --gwSrc=0 --gwDst=1 \
  --trafficProfile=gw2gw_direct \
  --simTime=120 --slotInterval=60" 2>&1 | tee ~/outputs/gw2gw_direct_tw2jp.log
```
```
[CFG] mode=gw2gw trafficProfile=gw2gw_direct simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[RBDC] trace connected: SatLlc/SatRequestManager/RbdcTrace
[TRAFFIC] profile=gw2gw_direct (GW_user→GW_user 端到端資料平面驗證)
[GW2GW_DIRECT] GW0_user=90.2.0.2  ->  GW1_user=90.2.0.3  start=1s  stop=119s
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

[CASE] gw2gw | gwSrc=0 gwDst=1
[CHKPT] 0s | PrecomputeGwRoutes: start | gws=2 pairs=1 slots=3
[GwRouting] slot=0 t=0s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=1 t=60s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=2 t=120s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats
[CHKPT] 0s | PrecomputeGwRoutes: done | wall=0ms

=== GW-to-GW Route Report (v6) ===

  [TW-Taipei → JP-Tokyo]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15                                                15      0.000000        
1     60        15      15                                                15      0.000000        
2     120       15      15                                                15      0.000000        

  [JP-Tokyo → TW-Taipei]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15                                                15      0.000000        
1     60        15      15                                                15      0.000000        
2     120       15      15                                                15      0.000000        

===================================

[CHKPT] 0s | ScheduleRoutingUpdates: 3 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3

[GW2GW_DIRECT] === 資料平面驗證結果 ===
  src: GW0 (90.2.0.2)
  dst: GW1 (90.2.0.3)
  received: 603648 bytes (~1179 pkts)
  [PASS] received>0 → 端到端 ISL 路徑驗證成功！
==========================================

[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=17

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       0           0               17              YES         
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

Total wall time: 468.3030 s
Event count:     0

```

## 2.GW_TW-GW_USA
```
./ns3 run "scratch/test-iridium \
  --mode=gw2gw \
  --gwSrc=0 --gwDst=2 \
  --trafficProfile=gw2gw_direct \
  --simTime=120 --slotInterval=60" 2>&1 | tee ~/outputs/gw2gw_direct_tw2usa.log
```
```
[CFG] mode=gw2gw trafficProfile=gw2gw_direct simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[RBDC] trace connected: SatLlc/SatRequestManager/RbdcTrace
[TRAFFIC] profile=gw2gw_direct (GW_user→GW_user 端到端資料平面驗證)
[GW2GW_DIRECT] GW0_user=90.2.0.2  ->  GW2_user=90.2.0.4  start=1s  stop=119s
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

[CASE] gw2gw | gwSrc=0 gwDst=2
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
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3

[GW2GW_DIRECT] === 資料平面驗證結果 ===
  src: GW0 (90.2.0.2)
  dst: GW2 (90.2.0.4)
  received: 603648 bytes (~1179 pkts)
  [PASS] received>0 → 端到端 ISL 路徑驗證成功！
==========================================

[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=1ms recompute=0ms recomputedSrc=17

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       1           0               17              YES         
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

Total wall time: 495.6190 s
Event count:     0
```