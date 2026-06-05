wenj@ubuntu-sns32:~/workspace/ns-3.43$ ./ns3 run "scratch/test-iridium --mode=gw2ut --gwId=0 --utId=0 --utLatDeg=25.0330 --utLonDeg=121.5654 --utName=UT-Taipei"
[CFG] mode=gw2ut simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
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
[CHKPT] 0s | PrecomputeAllTables: complete | wall=4ms

[CASE] gw2ut | gwId=0 utId=0 utLatDeg=25.033 utLonDeg=121.565
[CHKPT] 0s | PrecomputeGwUtRoutes: start | gws=1 uts=1 pairs=1 slots=11
[GwUtRouting] slot=0 t=0s UT0[UT-Taipei]=2sats
[GwUtRouting] slot=1 t=60s UT0[UT-Taipei]=1sats
[GwUtRouting] slot=2 t=120s UT0[UT-Taipei]=1sats
[GwUtRouting] slot=3 t=180s UT0[UT-Taipei]=1sats
[GwUtRouting] slot=4 t=240s UT0[UT-Taipei]=1sats
[GwUtRouting] slot=5 t=300s UT0[UT-Taipei]=2sats
[GwUtRouting] slot=6 t=360s UT0[UT-Taipei]=1sats
[GwUtRouting] slot=7 t=420s UT0[UT-Taipei]=2sats
[GwUtRouting] slot=8 t=480s UT0[UT-Taipei]=2sats
[GwUtRouting] slot=9 t=540s UT0[UT-Taipei]=2sats
[GwUtRouting] slot=10 t=600s UT0[UT-Taipei]=2sats
[CHKPT] 0s | PrecomputeGwUtRoutes: done | wall=0ms

=== GW-to-UT Route Report (v7) ===

  [TW-Taipei → UT-Taipei]
slot  time(s)   entry   ISL_path                                          serving   isl_cost(s)     
----------------------------------------------------------------------------------------------------
0     0         15      15                                                15        0.000000        
1     60        15      15                                                15        0.000000        
2     120       15      15                                                15        0.000000        
3     180       15      15                                                15        0.000000        
4     240       15      15                                                15        0.000000        
5     300       15      15                                                15        0.000000        
6     360       44      44                                                44        0.000000          <-- ROUTE CHANGED
7     420       14      14                                                14        0.000000          <-- ROUTE CHANGED
8     480       14      14                                                14        0.000000        
9     540       14      14                                                14        0.000000        
10    600       14      14                                                14        0.000000        

===================================

[CHKPT] 0s | ScheduleRoutingUpdates: 11 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=17
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=26/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=1ms recompute=0ms recomputedSrc=26
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=30/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=1ms recompute=0ms recomputedSrc=30
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=31/66 wall=2ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=3ms recompute=2ms recomputedSrc=31
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=39/66 wall=0ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=1ms recompute=0ms recomputedSrc=39
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=42/66 wall=4ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=4ms recompute=4ms recomputedSrc=42
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=46/66 wall=0ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=1ms recompute=0ms recomputedSrc=46
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=44/66 wall=1ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=1ms recompute=1ms recomputedSrc=44
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=50/66 wall=0ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=1ms recompute=0ms recomputedSrc=50

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       0           0               17              YES         
3     180       1           0               26              YES         
4     240       1           0               30              YES         
5     300       3           2               31              YES         
6     360       1           0               39              YES         
7     420       4           4               42              YES         
8     480       1           0               46              YES         
9     540       1           1               44              YES         
10    600       1           0               50              YES         
==============================

Total wall time: 2765.45 s
Event count:     0
