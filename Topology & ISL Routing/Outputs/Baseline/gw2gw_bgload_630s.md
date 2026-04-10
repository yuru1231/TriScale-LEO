```
# PLACEHOLDER — 尚未執行
# 執行指令：
# ./ns3 run "scratch/test-iridium_baseline \
#   --mode=gw2gw \
#   --gwSrc=0 --gwDst=2 \
#   --trafficProfile=gw2gw \
#   --simTime=630 --slotInterval=60" \
#   2>&1 | tee output_gw2gw_bgload_630s.log
#
# 比對目的：與 gw2gw_bgload_120s.md 形成時間維度對比（120s vs 630s，background load）
# 預期觀察：11 slots / route change 累積次數 / drop rate 是否上升 / Loaded ISL links 是否增加
```
```
   --mode=gw2gw \
   --gwSrc=0 --gwDst=2 \
   --trafficProfile=gw2gw \
   --simTime=630 --slotInterval=60" \
   2>&1 | tee output_gw2gw_630s.log
[CFG] mode=gw2gw trafficProfile=gw2gw simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)
[TRAFFIC] profile=gw2gw (GW-side queue/request background-load mode)
[TRAFFIC][gw2gw] install GW-side background load for gwSrc=0 and gwDst=2
[TRAFFIC] gwId=0 gwUsers=1 utUsers=91 start=1s stop=629s
[TRAFFIC] FWD installed: interval=60ms pktSize=1500B rate~200 kbps/flow
[TRAFFIC] RTN installed: interval=60ms pktSize=1024B rate~136.533 kbps/flow
[TRAFFIC] gwId=2 gwUsers=1 utUsers=91 start=1s stop=629s
[TRAFFIC] FWD installed: interval=60ms pktSize=1500B rate~200 kbps/flow
[TRAFFIC] RTN installed: interval=60ms pktSize=1024B rate~136.533 kbps/flow
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
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=9/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=9
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=17
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=25/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=0ms recompute=0ms recomputedSrc=25
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=31/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=0ms recompute=0ms recomputedSrc=31
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=33/66 wall=0ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=0ms recompute=0ms recomputedSrc=33
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=46/66 wall=0ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=1ms recompute=0ms recomputedSrc=46
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=50/66 wall=0ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=0ms recompute=0ms recomputedSrc=50
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=54/66 wall=0ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=0ms recompute=0ms recomputedSrc=54
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=52/66 wall=0ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=0ms recompute=0ms recomputedSrc=52
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=62/66 wall=0ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=0ms recompute=0ms recomputedSrc=62

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               9               YES         
2     120       0           0               17              YES         
3     180       0           0               25              YES         
4     240       0           0               31              YES         
5     300       0           0               33              YES         
6     360       1           0               46              YES         
7     420       0           0               50              YES         
8     480       0           0               54              YES         
9     540       0           0               52              YES         
10    600       0           0               62              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
0       0       1       0.0646          1.0838          
1       0       11      0.0646          0.0000          
2       1       2       1.0838          0.7025          
3       1       12      1.3368          0.0000          
4       2       3       0.7025          0.1099          
5       2       13      0.7025          0.1318          
6       3       4       0.1099          0.0000          
7       3       14      0.1099          0.1260          
9       4       15      0.0000          0.0452          
11      5       16      0.0000          0.1732          
12      6       7       0.0000          0.8952          
13      6       17      0.0000          0.0466          
14      7       8       0.8952          1.8264          
15      7       18      0.8952          0.0767          
16      8       9       1.8264          1.1564          
17      8       19      1.8264          1.4028          
18      9       10      1.1564          0.1569          
19      9       20      1.1564          0.0000          
20      0       10      0.0646          0.1569          
21      10      21      0.1569          0.0542          
22      11      12      0.2530          0.0000          
24      12      13      25.1546         0.1318          
25      12      23      0.0000          0.1140          
26      13      14      24.5909         0.1260          
27      13      24      0.1318          0.0000          
28      14      15      27.2499         0.0452          
29      14      25      0.1260          0.0000          
30      15      16      0.0452          1.1850          
31      15      26      0.0452          1.2461          
32      16      17      0.1732          0.0466          
33      16      27      0.1732          0.0000          
34      17      18      0.0466          0.0767          
35      17      28      0.0466          0.5923          
36      18      19      0.0767          1.4028          
37      18      29      0.0767          2.5690          
38      19      20      1.4028          0.0000          
39      19      30      1.4028          1.6282          
40      20      21      0.0000          0.0542          
42      11      21      0.0000          0.0542          
43      21      32      0.0542          0.0000          
45      22      33      0.0000          0.0186          
49      24      35      0.0000          0.0416          
50      25      26      0.0000          1.2461          
51      25      36      0.0000          1.3529          
52      26      27      1.2461          0.0000          
53      26      37      1.2461          0.5381          
54      27      28      0.0000          0.0799          
55      27      38      0.0000          1.3269          
56      28      29      0.0799          2.5690          
57      28      39      0.0799          1.7477          
58      29      30      2.5690          1.6282          
59      29      40      2.5690          0.0000          
60      30      31      1.6282          0.0000          
61      30      41      1.6282          0.4385          
63      31      42      0.0000          0.0286          
65      32      43      0.0000          0.1140          
66      33      34      0.0186          0.0000          
67      33      44      0.0186          0.0000          
68      34      35      0.0000          0.0416          
70      35      36      0.0416          1.3529          
71      35      46      0.0416          0.0000          
72      36      37      1.3529          0.6109          
73      36      47      1.3529          0.0000          
74      37      38      0.5381          1.3269          
75      37      48      0.5381          0.9698          
76      38      39      1.3269          1.7477          
77      38      49      1.3269          0.1393          
78      39      40      1.7477          0.0000          
79      39      50      1.7477          0.0374          
80      40      41      0.0000          0.4385          
82      41      42      0.4385          0.0286          
83      41      52      0.4385          0.3193          
84      42      43      0.0286          0.1140          
85      42      53      0.0286          0.0855          
86      33      43      0.0186          0.1140          
87      43      54      0.1140          0.0260          
89      44      55      0.0000          0.1140          
94      47      48      0.0000          0.9698          
95      47      58      0.0000          2.7005          
96      48      49      0.9698          0.1393          
97      48      59      0.9698          0.9530          
98      49      50      0.1393          0.0374          
99      49      60      0.1393          0.8952          
100     50      51      0.0374          0.0000          
101     50      61      0.0374          0.4908          
102     51      52      0.0000          0.3193          
103     51      62      0.0000          0.3204          
104     52      53      0.3193          0.0855          
105     52      63      0.3193          0.0160          
106     53      54      0.0855          0.0260          
107     53      64      0.0855          0.1638          
108     44      54      0.0000          0.0260          
109     54      65      0.0260          1.9831          
110     55      56      0.1140          0.0000          
111     0       55      0.0646          0.1140          
113     1       56      1.0838          0.0000          
114     57      58      0.0000          2.9534          
115     2       57      0.7025          0.0000          
116     58      59      2.7005          1.2060          
117     3       58      0.1099          2.7306          
118     59      60      0.9530          0.8952          
119     4       59      0.0000          0.9530          
120     60      61      0.8952          0.4908          
121     5       60      0.0000          0.8952          
122     61      62      0.4908          0.3204          
123     6       61      0.0000          0.4908          
124     62      63      0.3204          0.0160          
125     7       62      0.8952          0.3204          
126     63      64      0.0160          0.1638          
127     8       63      1.8264          0.0160          
128     64      65      0.1638          1.9831          
129     9       64      1.1564          0.1638          
130     55      65      0.1140          1.9831          
131     10      65      0.1569          1.9831          
Loaded ISL links: 114 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
ISL           total_pkts  dropped   drop_rate(%)  success_rate(%)
--------------------------------------------------------------
12-13         227179      16364     7.2030        92.797
13-14         401694      17436     4.3400        95.659
14-15         474595      10766     2.2680        97.732
16-15         212767      3990      1.8750        98.125
17-16         222182      10952     4.9290        95.071
23-12         169118      11        0.0060        99.993
--------------------------------------------------------------
TOTAL: 37231437 pkts, 59519 dropped | drop_rate=0.160% | success_rate=99.840%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 4902.212 s
Event count:     0
```