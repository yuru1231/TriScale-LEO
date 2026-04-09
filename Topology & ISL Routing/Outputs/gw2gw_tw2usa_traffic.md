./ns3 run "scratch/test-iridium \
  --mode=gw2gw \
  --gwSrc=0 --gwDst=2 \
  --elevMinDeg=5 \ 
  --simTime=630 \
  --slotInterval=60"
Consolidate compiler generated dependencies of target scratch_test-iridium
[CFG] mode=gw2gw trafficProfile=none simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
[TRAFFIC] profile=none (baseline, no traffic)
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
[CHKPT] 0s | PrecomputeAllTables: complete | wall=6ms

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
[CHKPT] 0s | PrecomputeGwRoutes: done | wall=1ms

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
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=17
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=26/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=0ms recompute=0ms recomputedSrc=26
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=30/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=0ms recompute=0ms recomputedSrc=30
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=31/66 wall=0ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=1ms recompute=0ms recomputedSrc=31
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=39/66 wall=1ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=2ms recompute=1ms recomputedSrc=39
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=42/66 wall=4ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=4ms recompute=4ms recomputedSrc=42
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=46/66 wall=1ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=1ms recompute=1ms recomputedSrc=46
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=44/66 wall=7ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=7ms recompute=7ms recomputedSrc=44
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=50/66 wall=0ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=0ms recompute=0ms recomputedSrc=50

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       0           0               17              YES         
3     180       0           0               26              YES         
4     240       0           0               30              YES         
5     300       1           0               31              YES         
6     360       2           1               39              YES         
7     420       4           4               42              YES         
8     480       1           1               46              YES         
9     540       7           7               44              YES         
10    600       0           0               50              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
4       2       3       0.0000          0.1235          
5       2       13      0.0000          0.1764          
6       3       4       0.1235          0.0000          
7       3       14      0.1235          0.1729          
9       4       15      0.0000          0.1452          
11      5       16      0.0000          0.2520          
13      6       17      0.0000          0.0605          
14      7       8       0.0000          0.2520          
16      8       9       0.2520          0.1072          
17      8       19      0.2520          0.4197          
18      9       10      0.1072          0.2523          
19      9       20      0.1072          0.0000          
20      0       10      0.0000          0.2523          
21      10      21      0.2523          0.0000          
24      12      13      0.0000          0.1764          
26      13      14      0.1764          0.1729          
27      13      24      0.1764          0.0000          
28      14      15      0.1729          0.1452          
29      14      25      0.1729          0.0000          
30      15      16      0.1452          0.2520          
31      15      26      0.1452          0.0000          
32      16      17      0.2520          0.0605          
33      16      27      0.2520          0.0000          
34      17      18      0.0605          0.0000          
35      17      28      0.0605          0.8064          
36      18      19      0.0000          0.4197          
38      19      20      0.4197          0.0000          
39      19      30      0.4197          0.0000          
45      22      33      0.0000          0.0296          
54      27      28      0.0000          0.8064          
55      27      38      0.0000          0.8092          
56      28      29      0.8064          0.0000          
57      28      39      0.8064          0.0000          
59      29      40      0.0000          0.3600          
61      30      41      0.0000          1.8379          
66      33      34      0.0296          0.0000          
67      33      44      0.0296          0.0000          
74      37      38      0.0000          0.8092          
76      38      39      0.8092          0.0000          
77      38      49      0.8092          0.1729          
78      39      40      0.0000          0.3600          
79      39      50      0.0000          0.0415          
80      40      41      0.3600          1.8379          
81      40      51      0.3600          0.0000          
82      41      42      1.8379          0.0000          
83      41      52      1.8379          0.1029          
85      42      53      0.0000          0.0864          
86      33      43      0.0296          0.0000          
93      46      57      0.0000          0.3600          
96      48      49      0.0000          0.1729          
98      49      50      0.1729          0.0415          
99      49      60      0.1729          0.0000          
100     50      51      0.0415          0.0000          
101     50      61      0.0415          0.2520          
102     51      52      0.0000          0.1029          
104     52      53      0.1029          0.0864          
105     52      63      0.1029          0.8039          
106     53      54      0.0864          0.0000          
107     53      64      0.0864          0.3024          
109     54      65      0.0000          0.0353          
112     56      57      0.0000          0.3600          
114     57      58      0.3600          0.0000          
115     2       57      0.0000          0.3600          
117     3       58      0.1235          0.0000          
120     60      61      0.0000          0.2520          
122     61      62      0.2520          0.0000          
123     6       61      0.0000          0.2520          
124     62      63      0.0000          0.8039          
126     63      64      0.8039          0.3024          
127     8       63      0.2520          0.8039          
128     64      65      0.3024          0.0353          
129     9       64      0.1072          0.3024          
130     55      65      0.0000          0.0353          
131     10      65      0.2523          0.0353          
Loaded ISL links: 74 / 132 total ISL edges
================================================

Total wall time: 2886.8770 s
Event count:     0
