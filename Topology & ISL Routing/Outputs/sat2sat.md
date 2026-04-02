wenj@ubuntu-sns32:~/workspace/ns-3.43$ ./ns3 run "scratch/test-iridium --mode=sat2sat --satSrc=0 --satDst=33"
Consolidate compiler generated dependencies of target satellite
Consolidate compiler generated dependencies of target scratch_test-iridium
[CFG] mode=sat2sat simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
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

[CASE] sat2sat | src=0 dst=33

=== Route Report: Full Paths Across Slots ===
time(s)   src   dst   full_path                                   route_cost    slot  
--------------------------------------------------------------------------------------
0         0     33    0->1->2->57->46->35->34->33                 0.078176      0     
60.000000 0     33    0->1->2->57->46->35->34->33                 0.074919      1     
120.0000000     33    0->1->2->57->46->35->34->33                 0.072055      2     
180.0000000     33    0->1->2->57->46->35->34->33                 0.069849      3     
240.0000000     33    0->1->2->57->46->35->34->33                 0.068654      4     
300.0000000     33    0->1->56->45->34->33                        0.065518      5       <-- PATH CHANGED
360.0000000     33    0->1->56->45->34->33                        0.061945      6     
420.0000000     33    0->1->56->45->34->33                        0.058345      7     
480.0000000     33    0->1->56->45->34->33                        0.054778      8     
540.0000000     33    0->1->56->45->34->33                        0.051327      9     
600.0000000     33    0->1->56->45->34->33                        0.048115      10    

==============================================
[CHKPT] 0.000000s | ScheduleRoutingUpdates: 11 events scheduled
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
[CHKPT] 420.000000s | RecomputeAffectedRoutes: slot=7 recomputed=41/66 wall=2ms
[CHKPT] 420.000000s | ApplyRoutingTable: slot=7 t=420.000000s | apply=2ms recompute=2ms recomputedSrc=41
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
7     420.0000002           2               41              YES         
8     480.0000000           0               44              YES         
9     540.0000001           0               46              YES         
10    600.0000000           0               50              YES         
==============================

Total wall time: 2722.786000 s
Event count:     0
