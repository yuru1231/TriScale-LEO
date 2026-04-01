## Config
`./ns3 run "scratch/test-iridium --simTime=630" 2>&1`

## Result
```
[CHKPT] 0s | Initialize: start
[CHKPT] 0s | LoadISLDefs: start | path=/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: start
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | Initialize: done | satellites=66 isls=132
[CHKPT] 0s | PrecomputeAllTables: start | slots=12
[CHKPT] PrecomputeAllTables: slot=0 t=0s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PeAllTables: slot=1 t=60s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputrecomputeAllTables: slot=2 t=120s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=3 t=180s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=4 t=240s | SAT0_routes=65 dijkstra=0ms total=1ms
[CHKPT] PrecomputeAllTables: slot=5 t=300s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=6 t=360s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=7 t=420s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=8 t=480s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=9 t=540s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=10 t=600s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=11 t=660s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=8ms

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
[CHKPT] 0.000000s | ScheduleRoutingUpdates: 12 events scheduled
[FtFilter] slot=0 t=0.000000s FT0=2sats FT1=2sats FT2=1sats
[FtFilter] slot=1 t=60.000000s FT0=1sats FT1=2sats FT2=2sats
[FtFilter] slot=2 t=120.000000s FT0=1sats FT1=2sats FT2=1sats
[FtFilter] slot=3 t=180.000000s FT0=1sats FT1=3sats FT2=2sats
[FtFilter] slot=4 t=240.000000s FT0=1sats FT1=3sats FT2=1sats
[FtFilter] slot=5 t=300.000000s FT0=2sats FT1=3sats FT2=1sats
[FtFilter] slot=6 t=360.000000s FT0=1sats FT1=3sats FT2=1sats
[FtFilter] slot=7 t=420.000000s FT0=2sats FT1=2sats FT2=1sats
[FtFilter] slot=8 t=480.000000s FT0=2sats FT1=2sats FT2=1sats
[FtFilter] slot=9 t=540.000000s FT0=2sats FT1=2sats FT2=2sats
[FtFilter] slot=10 t=600.000000s FT0=2sats FT1=2sats FT2=2sats
[FtFilter] slot=11 t=660.000000s FT0=1sats FT1=2sats FT2=1sats

=== FtVisibilityFilter Report ===
Slot 0 (0.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 2 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats
Slot 1 (60.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 1 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 2 visible sats
Slot 2 (120.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 1 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats
Slot 3 (180.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 1 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 2 visible sats
Slot 4 (240.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 1 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats
Slot 5 (300.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 2 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats
Slot 6 (360.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 1 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 3 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats
Slot 7 (420.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 2 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats
Slot 8 (480.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 2 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats
Slot 9 (540.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 2 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 2 visible sats
Slot 10 (600.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 2 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 2 visible sats
Slot 11 (660.000000s):
  FT0[TW-Taipei] lat=25.000000 lon=121.500000 → 1 visible sats
  FT1[JP-Tokyo] lat=35.700000 lon=139.700000 → 2 visible sats
  FT2[US-SanFrancisco] lat=37.800000 lon=-122.400000 → 1 visible sats

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
[CHKPT] 120.0000s | ApplyRoutingTable: slot=2 t=120.0000s | apply=1ms recompute=0ms recomputedSrc=17
[CHKPT] 180.0000s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180.0000s | RecomputeAffectedRoutes: slot=3 recomputed=23/66 wall=3ms
[CHKPT] 180.0000s | ApplyRoutingTable: slot=3 t=180.0000s | apply=3ms recompute=3ms recomputedSrc=23
[CHKPT] 240.0000s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240.0000s | RecomputeAffectedRoutes: slot=4 recomputed=30/66 wall=0ms
[CHKPT] 240.0000s | ApplyRoutingTable: slot=4 t=240.0000s | apply=0ms recompute=0ms recomputedSrc=30
[CHKPT] 300.0000s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300.0000s | RecomputeAffectedRoutes: slot=5 recomputed=33/66 wall=0ms
[CHKPT] 300.0000s | ApplyRoutingTable: slot=5 t=300.0000s | apply=0ms recompute=0ms recomputedSrc=33
[CHKPT] 360.0000s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360.0000s | RecomputeAffectedRoutes: slot=6 recomputed=41/66 wall=2ms
[CHKPT] 360.0000s | ApplyRoutingTable: slot=6 t=360.0000s | apply=3ms recompute=2ms recomputedSrc=41
[CHKPT] 420.0000s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420.0000s | RecomputeAffectedRoutes: slot=7 recomputed=43/66 wall=0ms
[CHKPT] 420.0000s | ApplyRoutingTable: slot=7 t=420.0000s | apply=0ms recompute=0ms recomputedSrc=43
[CHKPT] 480.0000s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480.0000s | RecomputeAffectedRoutes: slot=8 recomputed=48/66 wall=0ms
[CHKPT] 480.0000s | ApplyRoutingTable: slot=8 t=480.0000s | apply=0ms recompute=0ms recomputedSrc=48
[CHKPT] 540.0000s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540.0000s | RecomputeAffectedRoutes: slot=9 recomputed=47/66 wall=1ms
[CHKPT] 540.0000s | ApplyRoutingTable: slot=9 t=540.0000s | apply=1ms recompute=1ms recomputedSrc=47
[CHKPT] 600.0000s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600.0000s | RecomputeAffectedRoutes: slot=10 recomputed=51/66 wall=0ms
[CHKPT] 600.0000s | ApplyRoutingTable: slot=10 t=600.0000s | apply=0ms recompute=0ms recomputedSrc=51

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.0000    0           0               0               NO          
1     60.0000   0           0               3               YES         
2     120.0000  1           0               17              YES         
3     180.0000  3           3               23              YES         
4     240.0000  0           0               30              YES         
5     300.0000  0           0               33              YES         
6     360.0000  3           2               41              YES         
7     420.0000  0           0               43              YES         
8     480.0000  0           0               48              YES         
9     540.0000  1           1               47              YES         
10    600.0000  0           0               51              YES         
==============================

Total wall time: 2735.2720 s
Event count:     0
```

## Explanation

> **Predictive ISL Routing + Runtime Adjustment + FT/BH integration complete pipeline runed**

### 1.預測式 Routing Table 正確建立

```
PrecomputeAllTables()
ComputeBaseRoutes()
```

```
slot=0~11 均成功產生 routing table
SAT0_routes=65
```

* 每個時間點皆完成 Dijkstra 計算，代表 graph 建構正常
* routing table 已成功「離線預先建立」，非 runtime 才計算

---

### 2.Routing 會隨時間改變（Topology-driven）

```
PrintRouteReport()
TracePath()
```

```
slot 0~2: 0->10->9->20->31->32  
slot 3 之後: 0->1->12->23->22->32  (PATH CHANGED)
```

* 路徑會隨時間變動，符合 LEO 拓樸動態特性
* 證明 routing 是基於 G(t) 而非固定路徑

---

### 3.Routing Table 定時套用（Runtime Apply）

```
ScheduleRoutingUpdates()
ApplyRoutingTable()
```

```
12 events scheduled  
每 60s ApplyRoutingTable(slot)
```

* routing table 有成功在模擬時間中被切換
* 符合「offline compute + runtime apply」架構

---

### 4.Load-aware 動態重計算

```
HasSignificantChange()
RecomputeAffectedRoutes()
```

```
slot=3 → recomputed=23/66  
slot=6 → recomputed=41/66
```

* 當 load 改變時會觸發局部 Dijkstra
* 非全域重算，具備效率與動態適應能力

---

### 5.ISL failure 避開能力

```
BlockISL()
RunAvoidanceTest()
```

```
Baseline: 0->10->9->20->31->32  
Block: 0<->10  
New: 0->1->2->13->24->23->22->32  

PASS / PASS
```

* 路徑成功避開被封鎖 ISL
* 無錯誤 next-hop（無 stale routing）

---

### 6.FT 可視衛星篩選

```
FtFilter
```

```
TW → 1~2 sats  
JP → 2~3 sats  
US → 1~2 sats
```

* 每個時間點 FT 對應可見衛星正確變動
* 可支援 access / handover

---

### 7.端到端 FT Routing

```
Best transit routes
```

```
FT0 → FT2: entry=sat15 exit=sat37
```

* 已完成 FT → SAT → SAT → FT routing
* 不只是 sat-level routing，而是完整 network routing

---

### 8.Beam Hopping 已接入事件系統

```
ComputeBhSchedule()
ScheduleBhUpdates()
```

```
64 events across 12 slots
```

* BH scheduler 已成功運作並註冊事件
* 目前尚未影響 routing（僅整合，未耦合）

---

### 9.代表性測試對
|Pair	|說明	|特性|
|-|-|-|
|0 → 32	|SAT 0 在軌道面 0，SAT 32 在軌道面 2 的對側	|長距離、多跳（跨面 + 同面混合），可看到路徑切換
|0 → 11	|SAT 0 到同面最後一顆（SAT 11）	|跨面近距，1–2 跳，短路徑驗證
|10 → 55	|SAT 10 到 SAT 55	|同時橫跨同面 ISL + 跨面 ISL 的混合路徑