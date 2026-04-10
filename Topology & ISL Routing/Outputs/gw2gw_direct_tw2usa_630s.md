```
[CFG] mode=gw2gw trafficProfile=gw2gw_direct simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
[RBDC] trace connected: SatLlc/SatRequestManager/RbdcTrace
[TRAFFIC] profile=gw2gw_direct (GW_user→GW_user 端到端資料平面驗證)
[GW2GW_DIRECT] GW0_user=90.2.0.2  ->  GW2_user=90.2.0.4  start=1s  stop=629s
[CHKPT] 0s | Initialize: start
[CHKPT] 0s | LoadISLDefs: start | path=/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: start
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | Initialize: done | satellites=66 isls=132
[CHKPT] 0s | PrecomputeAllTables: start | slots=11
[CHKPT] PrecomputeAllTables: slot=0 t=0s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=1 t=60s | SAT0_routes=65 dijkstra=0ms total=1ms
[CHKPT] PrecomputeAllTables: slot=2 t=120s | SAT0_routes=65 dijkstra=0ms total=1ms
[CHKPT] PrecomputeAllTables: slot=3 t=180s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=4 t=240s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=5 t=300s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=6 t=360s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=7 t=420s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=8 t=480s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=9 t=540s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=10 t=600s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=7ms

[CASE] gw2gw | gwSrc=0 gwDst=2
[CHKPT] 0s | PrecomputeGwRoutes: start | gws=2 pairs=1 slots=11
[GwRouting] slot=0 t=0s GW0[TW-Taipei]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=1 t=60s GW0[TW-Taipei]=1sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=2 t=120s GW0[TW-Taipei]=1sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=3 t=180s GW0[TW-Taipei]=1sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=4 t=240s GW0[TW-Taipei]=1sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=5 t=300s GW0[TW-Taipei]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=6 t=360s GW0[TW-Taipei]=1sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=7 t=420s GW0[TW-Taipei]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=8 t=480s GW0[TW-Taipei]=2sats GW2[US-SanFrancisco]=1sats
[GwRouting] slot=9 t=540s GW0[TW-Taipei]=2sats GW2[US-SanFrancisco]=2sats
[GwRouting] slot=10 t=600s GW0[TW-Taipei]=2sats GW2[US-SanFrancisco]=2sats
[CHKPT] 0s | PrecomputeGwRoutes: done | wall=0ms

=== GW-to-GW Route Report (v6) ===

  [TW-Taipei → US-SanFrancisco]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15->14->25->36->37                                37      0.043969        
1     60        15      15->14->25->36->37                                37      0.046214        
2     120       15      15->14->13->2->1                                  1       0.047600          <-- ROUTE CHANGED
3     180       15      15->14->25->36                                    36      0.037717          <-- ROUTE CHANGED
4     240       15      15->14->25->36                                    36      0.040054        
5     300       15      15->14->25->36                                    36      0.042347        
6     360       44      44->45->46->35->36                                36      0.044439          <-- ROUTE CHANGED
7     420       14      14->13->24->35->36                                36      0.040314          <-- ROUTE CHANGED
8     480       14      14->13->24->35->36                                36      0.042179        
9     540       44      44->45->56->1->0                                  0       0.043645          <-- ROUTE CHANGED
10    600       44      44->45->56->1->0                                  0       0.041591        

  [US-SanFrancisco → TW-Taipei]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         37      37->36->25->14->15                                15      0.043969        
1     60        37      37->36->25->14->15                                15      0.046214        
2     120       1       1->2->13->14->15                                  15      0.047600          <-- ROUTE CHANGED
3     180       36      36->25->14->15                                    15      0.037717          <-- ROUTE CHANGED
4     240       36      36->25->14->15                                    15      0.040054        
5     300       36      36->25->14->15                                    15      0.042347        
6     360       36      36->35->46->45->44                                44      0.044439          <-- ROUTE CHANGED
7     420       36      36->35->24->13->14                                14      0.040314          <-- ROUTE CHANGED
8     480       36      36->35->24->13->14                                14      0.042179        
9     540       0       0->1->56->45->44                                  44      0.043645          <-- ROUTE CHANGED
10    600       0       0->1->56->45->44                                  44      0.041591        

===================================

[CHKPT] 0s | ScheduleRoutingUpdates: 11 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=1ms recompute=0ms recomputedSrc=17
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=23/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=0ms recompute=0ms recomputedSrc=23
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=30/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=0ms recompute=0ms recomputedSrc=30
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=33/66 wall=0ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=0ms recompute=0ms recomputedSrc=33
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=41/66 wall=2ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=2ms recompute=2ms recomputedSrc=41
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=43/66 wall=1ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=1ms recompute=1ms recomputedSrc=43
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=48/66 wall=1ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=1ms recompute=1ms recomputedSrc=48
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=47/66 wall=5ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=5ms recompute=5ms recomputedSrc=47
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=51/66 wall=1ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=2ms recompute=1ms recomputedSrc=51

[GW2GW_DIRECT] === 資料平面驗證結果 ===
  src: GW0 (90.2.0.2)
  dst: GW2 (90.2.0.4)
  received: 3214848 bytes (~6279 pkts)
  [PASS] received>0 → 端到端 ISL 路徑驗證成功！
==========================================


=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       1           0               17              YES         
3     180       0           0               23              YES         
4     240       0           0               30              YES         
5     300       0           0               33              YES         
6     360       2           2               41              YES         
7     420       1           1               43              YES         
8     480       1           1               48              YES         
9     540       5           5               47              YES         
10    600       2           1               51              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
4       2       3       0.0000          0.1099          
5       2       13      0.0000          0.1318          
6       3       4       0.1099          0.0000          
7       3       14      0.1099          0.1260          
9       4       15      0.0000          0.0452          
11      5       16      0.0000          0.1732          
13      6       17      0.0000          0.0466          
14      7       8       0.0000          0.1724          
15      7       18      0.0000          0.0317          
16      8       9       0.1724          0.0902          
17      8       19      0.1724          0.2768          
18      9       10      0.0902          0.2579          
19      9       20      0.0902          0.0000          
20      0       10      0.0000          0.2579          
21      10      21      0.2579          0.0542          
24      12      13      0.0000          0.1318          
26      13      14      0.1318          0.1260          
27      13      24      0.1318          0.0000          
28      14      15      0.1260          0.0452          
29      14      25      0.1260          0.0000          
30      15      16      0.0452          0.1732          
31      15      26      0.0452          0.0000          
32      16      17      0.1732          0.0466          
33      16      27      0.1732          0.0000          
34      17      18      0.0466          0.0317          
35      17      28      0.0466          0.7463          
36      18      19      0.0317          0.2768          
37      18      29      0.0317          0.0000          
38      19      20      0.2768          0.0000          
39      19      30      0.2768          0.0000          
40      20      21      0.0000          0.0542          
42      11      21      0.0000          0.0542          
43      21      32      0.0542          0.0000          
54      27      28      0.0000          0.7463          
55      27      38      0.0000          0.6463          
56      28      29      0.7463          0.0000          
57      28      39      0.7463          0.0000          
59      29      40      0.0000          0.2690          
61      30      41      0.0000          1.1719          
67      33      44      0.0000          0.0317          
74      37      38      0.0000          0.6463          
76      38      39      0.6463          0.0000          
77      38      49      0.6463          0.1393          
78      39      40      0.0000          0.2690          
79      39      50      0.0000          0.0155          
80      40      41      0.2690          1.1719          
81      40      51      0.2690          0.0000          
82      41      42      1.1719          0.0000          
83      41      52      1.1719          0.0879          
85      42      53      0.0000          0.0747          
88      44      45      0.0317          0.0000          
89      44      55      0.0317          0.0000          
93      46      57      0.0000          0.2434          
96      48      49      0.0000          0.1393          
98      49      50      0.1393          0.0155          
99      49      60      0.1393          0.0000          
100     50      51      0.0155          0.0000          
101     50      61      0.0155          0.1704          
102     51      52      0.0000          0.0879          
104     52      53      0.0879          0.0747          
105     52      63      0.0879          0.6341          
106     53      54      0.0747          0.0000          
107     53      64      0.0747          0.2238          
108     44      54      0.0317          0.0000          
109     54      65      0.0000          0.0333          
112     56      57      0.0000          0.2434          
114     57      58      0.2434          0.0000          
115     2       57      0.0000          0.2434          
117     3       58      0.1099          0.0000          
120     60      61      0.0000          0.1704          
122     61      62      0.1704          0.0000          
123     6       61      0.0000          0.1704          
124     62      63      0.0000          0.6341          
126     63      64      0.6341          0.2238          
127     8       63      0.1724          0.6341          
128     64      65      0.2238          0.0333          
129     9       64      0.0902          0.2238          
130     55      65      0.0000          0.0333          
131     10      65      0.2579          0.0333          
Loaded ISL links: 79 / 132 total ISL edges
================================================

Total wall time: 2803.0140 s
Event count:     0
```