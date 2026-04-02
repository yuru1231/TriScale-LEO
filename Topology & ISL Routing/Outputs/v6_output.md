[CHKPT] 0s | Initialize: start
[CHKPT] 0s | LoadISLDefs: start | path=/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: start
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | Initialize: done | satellites=66 isls=132
[CHKPT] 0s | PrecomputeAllTables: start | slots=12
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
[CHKPT] PrecomputeAllTables: slot=11 t=660s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=5ms

=== Route Report: Full Paths Across Slots ===
time(s)   src   dst   full_path                                   route_cost    slot  
--------------------------------------------------------------------------------------
0         0     32    0->10->9->20->31->32                        0.067286      0     
60.000000 0     32    0->10->9->20->31->32                        0.069555      1     
120.0000000     32    0->10->9->20->31->32                        0.071747      2     
180.0000000     32    0->1->12->23->22->32                        0.070587      3       <-- PATH CHANGED
240.0000000     32    0->1->12->23->22->32                        0.068352      4     
300.0000000     32    0->1->12->23->22->32                        0.066052      5     
360.0000000     32    0->1->12->23->22->32                        0.063711      6     
420.0000000     32    0->1->12->23->22->32                        0.061362      7     
480.0000000     32    0->1->12->23->22->32                        0.059045      8     
540.0000000     32    0->1->12->23->22->32                        0.056818      9     
600.0000000     32    0->1->12->23->22->32                        0.054764      10    
660.0000000     32    0->1->12->23->22->32                        0.053003      11    

0.000000  0     11    0->1->2->13->12->11                         0.062453      0     
60.000000 0     11    0->1->2->13->12->11                         0.061538      1     
120.0000000     11    0->1->2->13->12->11                         0.060773      2     
180.0000000     11    0->1->12->11                                0.042422      3       <-- PATH CHANGED
240.0000000     11    0->1->12->11                                0.041346      4     
300.0000000     11    0->1->12->11                                0.040245      5     
360.0000000     11    0->1->12->11                                0.039135      6     
420.0000000     11    0->1->12->11                                0.038035      7     
480.0000000     11    0->1->12->11                                0.036966      8     
540.0000000     11    0->1->12->11                                0.035963      9     
600.0000000     11    0->1->12->11                                0.035066      10    
660.0000000     11    0->1->12->11                                0.034328      11    

0.000000  10    55    10->9->64->65->55                           0.052788      0     
60.000000 10    55    10->9->64->65->55                           0.053969      1     
120.00000010    55    10->9->64->65->55                           0.055104      2     
180.00000010    55    10->0->1->56->55                            0.054513      3       <-- PATH CHANGED
240.00000010    55    10->0->1->56->55                            0.053354      4     
300.00000010    55    10->0->1->56->55                            0.052154      5     
360.00000010    55    10->0->1->56->55                            0.050923      6     
420.00000010    55    10->0->1->56->55                            0.049674      7     
480.00000010    55    10->0->1->56->55                            0.048425      8     
540.00000010    55    10->0->1->56->55                            0.047202      9     
600.00000010    55    10->0->1->56->55                            0.046044      10    
660.00000010    55    10->0->55                                   0.029119      11      <-- PATH CHANGED

==============================================

=== ISL Avoidance Test =========================
Pair      : 0 -> 32  (slot 0, t=0.000000s)
Baseline  : 0->10->9->20->31->32  cost=0.067286
Block ISL : 0 <-> 10
Post-block: 0->1->2->13->24->23->22->32  cost=0.083479
Check (a) ISL absent from path : PASS
Check (b) No stale next-hop    : PASS
Routing table restored.
================================================
[CHKPT] 0.000000s | PrecomputeGwRoutes: start | gws=3 pairs=2 slots=12
[GwRouting] slot=0 t=0.000000s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=1 t=60.000000s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=2 t=120.000000s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=3 t=180.000000s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=4 t=240.000000s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=5 t=300.000000s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=3sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=6 t=360.000000s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=7 t=420.000000s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=8 t=480.000000s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=9 t=540.000000s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=10 t=600.000000s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=11 t=660.000000s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats GW2[US-SanFrancisco]=1sats
[CHKPT] 0.000000s | PrecomputeGwRoutes: done | wall=1ms

=== GW-to-GW Route Report (v6) ===

  [TW-Taipei → JP-Tokyo]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0.000000  15      15                                                15      0.000000        
1     60.000000 15      15                                                15      0.000000        
2     120.00000015      15                                                15      0.000000        
3     180.00000015      15                                                15      0.000000        
4     240.00000015      15                                                15      0.000000        
5     300.00000044      44                                                44      0.000000          <-- ROUTE CHANGED
6     360.00000044      44                                                44      0.000000        
7     420.00000014      14                                                14      0.000000          <-- ROUTE CHANGED
8     480.00000014      14                                                14      0.000000        
9     540.00000014      14                                                14      0.000000        
10    600.00000014      14                                                14      0.000000        
11    660.00000014      14                                                14      0.000000        

  [JP-Tokyo → TW-Taipei]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0.000000  15      15                                                15      0.000000        
1     60.000000 15      15                                                15      0.000000        
2     120.00000015      15                                                15      0.000000        
3     180.00000015      15                                                15      0.000000        
4     240.00000015      15                                                15      0.000000        
5     300.00000044      44                                                44      0.000000          <-- ROUTE CHANGED
6     360.00000044      44                                                44      0.000000        
7     420.00000014      14                                                14      0.000000          <-- ROUTE CHANGED
8     480.00000014      14                                                14      0.000000        
9     540.00000014      14                                                14      0.000000        
10    600.00000014      14                                                14      0.000000        
11    660.00000014      14                                                14      0.000000        

  [TW-Taipei → US-SanFrancisco]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0.000000  15      15->14->25->36->37                                37      0.043969        
1     60.000000 15      15->14->25->36->37                                37      0.046214        
2     120.00000015      15->14->13->2->1                                  1       0.047600          <-- ROUTE CHANGED
3     180.00000015      15->14->25->36                                    36      0.037717          <-- ROUTE CHANGED
4     240.00000015      15->14->25->36                                    36      0.040054        
5     300.00000015      15->14->25->36                                    36      0.042347        
6     360.00000044      44->45->46->35->36                                36      0.044439          <-- ROUTE CHANGED
7     420.00000014      14->13->24->35->36                                36      0.040314          <-- ROUTE CHANGED
8     480.00000014      14->13->24->35->36                                36      0.042179        
9     540.00000044      44->45->56->1->0                                  0       0.043645          <-- ROUTE CHANGED
10    600.00000044      44->45->56->1->0                                  0       0.041591        
11    660.00000014      14->13->12->1->0                                  0       0.047502          <-- ROUTE CHANGED

  [US-SanFrancisco → TW-Taipei]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0.000000  37      37->36->25->14->15                                15      0.043969        
1     60.000000 37      37->36->25->14->15                                15      0.046214        
2     120.0000001       1->2->13->14->15                                  15      0.047600          <-- ROUTE CHANGED
3     180.00000036      36->25->14->15                                    15      0.037717          <-- ROUTE CHANGED
4     240.00000036      36->25->14->15                                    15      0.040054        
5     300.00000036      36->25->14->15                                    15      0.042347        
6     360.00000036      36->35->46->45->44                                44      0.044439          <-- ROUTE CHANGED
7     420.00000036      36->35->24->13->14                                14      0.040314          <-- ROUTE CHANGED
8     480.00000036      36->35->24->13->14                                14      0.042179        
9     540.0000000       0->1->56->45->44                                  44      0.043645          <-- ROUTE CHANGED
10    600.0000000       0->1->56->45->44                                  44      0.041591        
11    660.0000000       0->1->12->13->14                                  14      0.047502          <-- ROUTE CHANGED

===================================

[CHKPT] 0.000000s | ScheduleRoutingUpdates: 12 events scheduled
[BeamHoppingManager] LoadOrbiterNodes: 66 nodes loaded
[BeamHoppingManager] ComputeBhSchedule: 64 events across 12 slots
[BeamHoppingManager] ScheduleBhUpdates: 64 events registered
[CHKPT] 0.000000s | ApplyRoutingTable: slot=0 t=0.000000s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60.000000s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60.000000s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60.000000s | ApplyRoutingTable: slot=1 t=60.000000s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120.000000s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120.000000s | RecomputeAffectedRoutes: slot=2 recomputed=13/66 wall=0ms
[CHKPT] 120.000000s | ApplyRoutingTable: slot=2 t=120.000000s | apply=0ms recompute=0ms recomputedSrc=13
[CHKPT] 180.000000s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180.000000s | RecomputeAffectedRoutes: slot=3 recomputed=20/66 wall=0ms
[CHKPT] 180.000000s | ApplyRoutingTable: slot=3 t=180.000000s | apply=0ms recompute=0ms recomputedSrc=20
[CHKPT] 240.000000s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240.000000s | RecomputeAffectedRoutes: slot=4 recomputed=25/66 wall=0ms
[CHKPT] 240.000000s | ApplyRoutingTable: slot=4 t=240.000000s | apply=0ms recompute=0ms recomputedSrc=25
[CHKPT] 300.000000s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300.000000s | RecomputeAffectedRoutes: slot=5 recomputed=26/66 wall=0ms
[CHKPT] 300.000000s | ApplyRoutingTable: slot=5 t=300.000000s | apply=0ms recompute=0ms recomputedSrc=26
[CHKPT] 360.000000s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360.000000s | RecomputeAffectedRoutes: slot=6 recomputed=36/66 wall=0ms
[CHKPT] 360.000000s | ApplyRoutingTable: slot=6 t=360.000000s | apply=0ms recompute=0ms recomputedSrc=36
[CHKPT] 420.000000s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420.000000s | RecomputeAffectedRoutes: slot=7 recomputed=41/66 wall=0ms
[CHKPT] 420.000000s | ApplyRoutingTable: slot=7 t=420.000000s | apply=0ms recompute=0ms recomputedSrc=41
[CHKPT] 480.000000s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480.000000s | RecomputeAffectedRoutes: slot=8 recomputed=44/66 wall=0ms
[CHKPT] 480.000000s | ApplyRoutingTable: slot=8 t=480.000000s | apply=0ms recompute=0ms recomputedSrc=44
[CHKPT] 540.000000s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540.000000s | RecomputeAffectedRoutes: slot=9 recomputed=46/66 wall=0ms
[CHKPT] 540.000000s | ApplyRoutingTable: slot=9 t=540.000000s | apply=1ms recompute=0ms recomputedSrc=46
[CHKPT] 600.000000s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600.000000s | RecomputeAffectedRoutes: slot=10 recomputed=50/66 wall=0ms
[CHKPT] 600.000000s | ApplyRoutingTable: slot=10 t=600.000000s | apply=0ms recompute=0ms recomputedSrc=50

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.000000  0           0               0               NO          
1     60.000000 0           0               3               YES         
2     120.0000000           0               13              YES         
3     180.0000000           0               20              YES         
4     240.0000000           0               25              YES         
5     300.0000000           0               26              YES         
6     360.0000000           0               36              YES         
7     420.0000000           0               41              YES         
8     480.0000000           0               44              YES         
9     540.0000001           0               46              YES         
10    600.0000000           0               50              YES         
==============================

Total wall time: 2534.374000 s
Event count:     0
