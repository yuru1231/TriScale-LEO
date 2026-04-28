```
./ns3 run "scratch/test-iridium \
  --mode=gw2ut \
  --trafficProfile=gw2ut \
  --simTime=630 \
  --gwId=0 \
  --utId=1 \
  --utLatDeg=37.8 \
  --utLonDeg=-122.4 \
  --utName=UT-SanFrancisco" \
  2>&1 | tee output_gw2ut_630s.txt
```
```
[  0%] Building CXX object scratch/CMakeFiles/scratch_test-iridium.dir/test-iridium.cc.o
[  0%] Linking CXX executable ../../build/scratch/ns3.43-test-iridium-default
[CFG] mode=gw2ut trafficProfile=gw2ut simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)
[TRAFFIC] profile=gw2ut (normal service traffic)
[TRAFFIC] gwId=0 gwUsers=1 utUsers=91 start=1s stop=629s
[TRAFFIC] FWD installed: interval=100ms pktSize=1500B rate~120 kbps/flow
[TRAFFIC] RTN installed: interval=500ms pktSize=512B rate~8.192 kbps/flow
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
[CHKPT] 0s | PrecomputeAllTables: complete | wall=3ms

[CASE] gw2ut | gwId=0 utId=1 utLatDeg=37.8 utLonDeg=-122.4
[CHKPT] 0s | PrecomputeGwUtRoutes: start | gws=1 uts=1 pairs=1 slots=11
[GwUtRouting] slot=0 t=0s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=1 t=60s UT1[UT-SanFrancisco]=2sats
[GwUtRouting] slot=2 t=120s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=3 t=180s UT1[UT-SanFrancisco]=2sats
[GwUtRouting] slot=4 t=240s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=5 t=300s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=6 t=360s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=7 t=420s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=8 t=480s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=9 t=540s UT1[UT-SanFrancisco]=2sats
[GwUtRouting] slot=10 t=600s UT1[UT-SanFrancisco]=2sats
[CHKPT] 0s | PrecomputeGwUtRoutes: done | wall=1ms

=== GW-to-UT Route Report (v7) ===

  [TW-Taipei → UT-SanFrancisco]
slot  time(s)   entry   ISL_path                                          serving   isl_cost(s)     
----------------------------------------------------------------------------------------------------
0     0         15      15->14->25->36->37                                37        0.043969        
1     60        15      15->14->25->36->37                                37        0.046214        
2     120       15      15->14->13->2->1                                  1         0.047600          <-- ROUTE CHANGED
3     180       15      15->14->25->36                                    36        0.037717          <-- ROUTE CHANGED
4     240       15      15->14->25->36                                    36        0.040054        
5     300       15      15->14->25->36                                    36        0.042347        
6     360       44      44->45->46->35->36                                36        0.044439          <-- ROUTE CHANGED
7     420       14      14->13->24->35->36                                36        0.040314          <-- ROUTE CHANGED
8     480       14      14->13->24->35->36                                36        0.042179        
9     540       44      44->45->56->1->0                                  0         0.043645          <-- ROUTE CHANGED
10    600       44      44->45->56->1->0                                  0         0.041591        

===================================

[CHKPT] 0s | ScheduleRoutingUpdates: 11 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=16/66 wall=4ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=4ms recompute=4ms recomputedSrc=16
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=22/66 wall=3ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=3ms recompute=3ms recomputedSrc=22
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=27/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=1ms recompute=0ms recomputedSrc=27
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=30/66 wall=2ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=2ms recompute=2ms recomputedSrc=30
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=42/66 wall=0ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=0ms recompute=0ms recomputedSrc=42
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=46/66 wall=11ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=12ms recompute=11ms recomputedSrc=46
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=50/66 wall=8ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=8ms recompute=8ms recomputedSrc=50
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=50/66 wall=1ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=2ms recompute=1ms recomputedSrc=50
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=54/66 wall=2ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=3ms recompute=2ms recomputedSrc=54

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       4           4               16              YES         
3     180       3           3               22              YES         
4     240       1           0               27              YES         
5     300       2           2               30              YES         
6     360       0           0               42              YES         
7     420       12          11              46              YES         
8     480       8           8               50              YES         
9     540       2           1               50              YES         
10    600       3           2               54              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
0       0       1       0.0646          0.0000          
1       0       11      0.0646          0.0000          
4       2       3       0.0000          0.1099          
5       2       13      0.0000          0.1318          
6       3       4       0.1099          0.0000          
7       3       14      0.1099          0.1260          
9       4       15      0.0000          0.0452          
11      5       16      0.0000          0.1732          
12      6       7       0.0000          0.3204          
13      6       17      0.0000          0.0466          
14      7       8       0.3204          0.1724          
15      7       18      0.3204          0.0627          
16      8       9       0.1724          0.0902          
17      8       19      0.1724          0.2455          
18      9       10      0.0902          0.1569          
19      9       20      0.0902          0.8650          
20      0       10      0.0646          0.1569          
21      10      21      0.1569          0.3192          
24      12      13      0.0000          0.1318          
26      13      14      0.1318          0.1260          
27      13      24      0.1318          0.0000          
28      14      15      0.1260          0.0452          
29      14      25      0.1260          0.0000          
30      15      16      0.0452          0.1732          
31      15      26      0.0452          0.0000          
32      16      17      0.1732          0.0466          
33      16      27      0.1732          0.0000          
34      17      18      0.0466          0.0627          
35      17      28      0.0466          0.0799          
36      18      19      0.0627          0.2455          
37      18      29      0.0627          0.0000          
38      19      20      0.2455          0.8650          
39      19      30      0.2455          0.0000          
40      20      21      0.8650          0.3192          
41      20      31      0.8650          0.0000          
42      11      21      0.0000          0.3192          
43      21      32      0.3192          0.0000          
45      22      33      0.0000          0.0186          
54      27      28      0.0000          0.0799          
55      27      38      0.0000          0.3039          
56      28      29      0.0799          0.0000          
57      28      39      0.0799          0.0000          
59      29      40      0.0000          0.3122          
61      30      41      0.0000          1.6273          
63      31      42      0.0000          0.0286          
66      33      34      0.0186          0.0000          
67      33      44      0.0186          0.0000          
69      34      45      0.0000          0.1584          
73      36      47      0.0000          0.4234          
74      37      38      0.0000          0.3039          
75      37      48      0.0000          0.1584          
76      38      39      0.3039          0.0000          
77      38      49      0.3039          0.1393          
78      39      40      0.0000          0.3122          
79      39      50      0.0000          0.7445          
80      40      41      0.3122          1.6273          
81      40      51      0.3122          0.0000          
82      41      42      1.6273          0.0286          
83      41      52      1.6273          0.0502          
84      42      43      0.0286          0.0000          
85      42      53      0.0286          0.0747          
86      33      43      0.0186          0.0000          
87      43      54      0.0000          0.0260          
88      44      45      0.0000          0.1584          
90      45      46      0.1584          0.0000          
91      45      56      0.1584          0.0000          
92      46      47      0.0000          0.4234          
94      47      48      0.4234          0.1584          
95      47      58      0.4234          0.0000          
96      48      49      0.1584          0.1393          
97      48      59      0.1584          0.0000          
98      49      50      0.1393          0.7445          
99      49      60      0.1393          0.0000          
100     50      51      0.7445          0.0000          
101     50      61      0.7445          0.1704          
102     51      52      0.0000          0.0502          
104     52      53      0.0502          0.0747          
105     52      63      0.0502          1.1599          
106     53      54      0.0747          0.0260          
107     53      64      0.0747          0.1638          
108     44      54      0.0000          0.0260          
109     54      65      0.0260          0.0109          
111     0       55      0.0646          0.0000          
117     3       58      0.1099          0.0000          
120     60      61      0.0000          0.1704          
122     61      62      0.1704          0.0000          
123     6       61      0.0000          0.1704          
124     62      63      0.0000          1.1599          
125     7       62      0.3204          0.0000          
126     63      64      1.1599          0.1638          
127     8       63      0.1724          1.1599          
128     64      65      0.1638          0.0109          
129     9       64      0.0902          0.1638          
130     55      65      0.0000          0.0109          
131     10      65      0.1569          0.0109          
Loaded ISL links: 95 / 132 total ISL edges
================================================

Total wall time: 3874.4740 s
Event count:     0
```