```
[CFG] mode=gw2gw trafficProfile=gw2gw simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)
[TRAFFIC] profile=gw2gw (GW-side queue/request background-load mode)
[TRAFFIC][gw2gw] install GW-side background load for gwSrc=0 and gwDst=2
[TRAFFIC] gwId=0 gwUsers=1 utUsers=91 start=1s stop=119s
[TRAFFIC] FWD installed: interval=60ms pktSize=1500B rate~200 kbps/flow
[TRAFFIC] RTN installed: interval=60ms pktSize=1024B rate~136.533 kbps/flow
[TRAFFIC] gwId=2 gwUsers=1 utUsers=91 start=1s stop=119s
[TRAFFIC] FWD installed: interval=60ms pktSize=1500B rate~200 kbps/flow
[TRAFFIC] RTN installed: interval=60ms pktSize=1024B rate~136.533 kbps/flow
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
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=9/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=9
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=17

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               9               YES         
2     120       0           0               17              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
17      8       19      0.0000          0.2690          
28      14      15      2.4446          0.0000          
36      18      19      0.0000          0.2690          
38      19      20      0.2690          0.0000          
39      19      30      0.2690          0.0000          
85      42      53      0.0000          0.1883          
87      43      54      0.0000          0.4517          
104     52      53      0.0000          0.1883          
105     52      63      0.0000          0.2772          
106     53      54      0.1883          0.4517          
107     53      64      0.1883          0.0000          
108     44      54      0.0000          0.4517          
109     54      65      0.4517          0.4360          
124     62      63      0.0000          0.2772          
126     63      64      0.2772          0.0000          
127     8       63      0.0000          0.2772          
128     64      65      0.0000          0.4360          
130     55      65      0.0000          0.4360          
131     10      65      0.0000          0.4360          
Loaded ISL links: 19 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
ISL           total_pkts  dropped   drop_rate(%)  success_rate(%)
--------------------------------------------------------------
14-15         105448      272       0.2570        99.742
--------------------------------------------------------------
TOTAL: 6706864 pkts, 272 dropped | drop_rate=0.004% | success_rate=99.996%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 932.257 s
Event count:     0
```