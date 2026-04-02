[CFG] mode=gw2gw simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
[CHKPT] 0s | Initialize: start
[CHKPT] 0s | LoadISLDefs: start | path=/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: start
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | Initialize: done | satellites=66 isls=132
[CHKPT] 0s | PrecomputeAllTables: start | slots=11
[CHKPT] PrecomputeAllTables: slot=0 t=0s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=1 t=60s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=2 t=120s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=3 t=180s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=4 t=240s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=5 t=300s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=6 t=360s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=7 t=420s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=8 t=480s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=9 t=540s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=10 t=600s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=5ms

[CASE] gw2gw | gwSrc=0 gwDst=1
[CHKPT] 0s | PrecomputeGwRoutes: start | gws=2 pairs=1 slots=11
[GwRouting] slot=0 t=0s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=1 t=60s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=2 t=120s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=3 t=180s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats
[GwRouting] slot=4 t=240s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats
[GwRouting] slot=5 t=300s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=3sats
[GwRouting] slot=6 t=360s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats
[GwRouting] slot=7 t=420s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=8 t=480s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=9 t=540s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=10 t=600s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats
[CHKPT] 0s | PrecomputeGwRoutes: done | wall=1ms

=== GW-to-GW Route Report (v6) ===

  [TW-Taipei → JP-Tokyo]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15                                                15      0.000000        
1     60        15      15                                                15      0.000000        
2     120       15      15                                                15      0.000000        
3     180       15      15                                                15      0.000000        
4     240       15      15                                                15      0.000000        
5     300       44      44                                                44      0.000000         <-- ROUTE CHANGED
6     360       44      44                                                44      0.000000        
7     420       14      14                                                14      0.000000         <-- ROUTE CHANGED
8     480       14      14                                                14      0.000000        
9     540       14      14                                                14      0.000000        
10    600       14      14                                                14      0.000000        

  [JP-Tokyo → TW-Taipei]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15                                                15      0.000000        
1     60        15      15                                                15      0.000000        
2     120       15      15                                                15      0.000000        
3     180       15      15                                                15      0.000000        
4     240       15      15                                                15      0.000000        
5     300       44      44                                                44      0.000000         <-- ROUTE CHANGED
6     360       44      44                                                44      0.000000        
7     420       14      14                                                14      0.000000         <-- ROUTE CHANGED
8     480       14      14                                                14      0.000000        
9     540       14      14                                                14      0.000000        
10    600       14      14                                                14      0.000000        

===================================

[CHKPT] 0s | ScheduleRoutingUpdates: 11 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=13/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=13
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=20/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=0ms recompute=0ms recomputedSrc=20
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=25/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=0ms recompute=0ms recomputedSrc=25
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=26/66 wall=2ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=2ms recompute=2ms recomputedSrc=26
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=36/66 wall=0ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=0ms recompute=0ms recomputedSrc=36
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=41/66 wall=0ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=0ms recompute=0ms recomputedSrc=41
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=44/66 wall=0ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=0ms recompute=0ms recomputedSrc=44
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=46/66 wall=0ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=0ms recompute=0ms recomputedSrc=46
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=50/66 wall=0ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=0ms recompute=0ms recomputedSrc=50

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       0           0               13              YES         
3     180       0           0               20              YES         
4     240       0           0               25              YES         
5     300       2           2               26              YES         
6     360       0           0               36              YES         
7     420       0           0               41              YES         
8     480       0           0               44              YES         
9     540       0           0               46              YES         
10    600       0           0               50              YES         
==============================

Total wall time: 2403.38 s
Event count:     0
