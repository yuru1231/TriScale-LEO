```
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
[CHKPT] 0s | ScheduleRoutingUpdates: 12 events scheduled
[FtFilter] slot=0 t=0s FT0=2sats FT1=2sats FT2=1sats
[FtFilter] slot=1 t=60s FT0=1sats FT1=2sats FT2=2sats
[FtFilter] slot=2 t=120s FT0=1sats FT1=2sats FT2=1sats
[FtFilter] slot=3 t=180s FT0=1sats FT1=3sats FT2=2sats
[FtFilter] slot=4 t=240s FT0=1sats FT1=3sats FT2=1sats
[FtFilter] slot=5 t=300s FT0=2sats FT1=3sats FT2=1sats
[FtFilter] slot=6 t=360s FT0=1sats FT1=3sats FT2=1sats
[FtFilter] slot=7 t=420s FT0=2sats FT1=2sats FT2=1sats
[FtFilter] slot=8 t=480s FT0=2sats FT1=2sats FT2=1sats
[FtFilter] slot=9 t=540s FT0=2sats FT1=2sats FT2=2sats
[FtFilter] slot=10 t=600s FT0=2sats FT1=2sats FT2=2sats
[FtFilter] slot=11 t=660s FT0=1sats FT1=2sats FT2=1sats

=== FtVisibilityFilter Report ===
Slot 0 (0s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 2 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats
Slot 1 (60s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 1 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 2 visible sats
Slot 2 (120s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 1 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats
Slot 3 (180s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 1 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 2 visible sats
Slot 4 (240s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 1 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats
Slot 5 (300s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 2 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats
Slot 6 (360s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 1 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats
Slot 7 (420s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 2 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats
Slot 8 (480s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 2 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats
Slot 9 (540s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 2 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 2 visible sats
Slot 10 (600s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 2 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 2 visible sats
Slot 11 (660s):
  FT0[TW-Taipei] lat=25 lon=121.5 → 1 visible sats
  FT1[JP-Tokyo] lat=35.7 lon=139.7 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.8 lon=-122.4 → 1 visible sats

Best transit routes (contracted pairs, slot 0):
  FT0 → FT1: entry=sat15 exit=sat15 cost=0.0000s
  FT0 → FT2: entry=sat15 exit=sat37 cost=0.0440s
=================================

[BeamHoppingManager] LoadOrbiterNodes: 66 nodes loaded
[BeamHoppingManager] ComputeBhSchedule: 64 events across 12 slots
[BeamHoppingManager] ScheduleBhUpdates: 64 events registered
[CHKPT] 0.0000s | ApplyRoutingTable: slot=0 t=0.0000s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60.0000s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60.0000s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60.0000s | ApplyRoutingTable: slot=1 t=60.0000s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120.0000s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120.0000s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120.0000s | ApplyRoutingTable: slot=2 t=120.0000s | apply=0ms recompute=0ms recomputedSrc=17
[CHKPT] 180.0000s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180.0000s | RecomputeAffectedRoutes: slot=3 recomputed=23/66 wall=0ms
[CHKPT] 180.0000s | ApplyRoutingTable: slot=3 t=180.0000s | apply=0ms recompute=0ms recomputedSrc=23
[CHKPT] 240.0000s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240.0000s | RecomputeAffectedRoutes: slot=4 recomputed=30/66 wall=0ms
[CHKPT] 240.0000s | ApplyRoutingTable: slot=4 t=240.0000s | apply=0ms recompute=0ms recomputedSrc=30
[CHKPT] 300.0000s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300.0000s | RecomputeAffectedRoutes: slot=5 recomputed=33/66 wall=3ms
[CHKPT] 300.0000s | ApplyRoutingTable: slot=5 t=300.0000s | apply=3ms recompute=3ms recomputedSrc=33
[CHKPT] 360.0000s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360.0000s | RecomputeAffectedRoutes: slot=6 recomputed=41/66 wall=0ms
[CHKPT] 360.0000s | ApplyRoutingTable: slot=6 t=360.0000s | apply=0ms recompute=0ms recomputedSrc=41
[CHKPT] 420.0000s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420.0000s | RecomputeAffectedRoutes: slot=7 recomputed=43/66 wall=0ms
[CHKPT] 420.0000s | ApplyRoutingTable: slot=7 t=420.0000s | apply=0ms recompute=0ms recomputedSrc=43
[CHKPT] 480.0000s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480.0000s | RecomputeAffectedRoutes: slot=8 recomputed=48/66 wall=3ms
[CHKPT] 480.0000s | ApplyRoutingTable: slot=8 t=480.0000s | apply=3ms recompute=3ms recomputedSrc=48
[CHKPT] 540.0000s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540.0000s | RecomputeAffectedRoutes: slot=9 recomputed=47/66 wall=5ms
[CHKPT] 540.0000s | ApplyRoutingTable: slot=9 t=540.0000s | apply=5ms recompute=5ms recomputedSrc=47
[CHKPT] 600.0000s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600.0000s | RecomputeAffectedRoutes: slot=10 recomputed=51/66 wall=0ms
[CHKPT] 600.0000s | ApplyRoutingTable: slot=10 t=600.0000s | apply=0ms recompute=0ms recomputedSrc=51

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.0000    0           0               0               NO          
1     60.0000   0           0               3               YES         
2     120.0000  0           0               17              YES         
3     180.0000  0           0               23              YES         
4     240.0000  0           0               30              YES         
5     300.0000  3           3               33              YES         
6     360.0000  0           0               41              YES         
7     420.0000  0           0               43              YES         
8     480.0000  3           3               48              YES         
9     540.0000  5           5               47              YES         
10    600.0000  0           0               51              YES         
==============================

Total wall time: 2775.7940 s
Event count:     0
```