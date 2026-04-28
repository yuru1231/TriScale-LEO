```
./ns3 run "scratch/test-iridium \
  --mode=gw2gw_e2e \
  --gwSrc=0 --gwDst=1 \
  --simTime=300 --slotInterval=60 \
  --enableFeederlink=1 --enableIsl=1" \
  2>&1 | tee "Topology & ISL Routing/Outputs/gw2gw_e2e_TW_JP_300s.log"
```
```
[CFG] pathType=gw2gw_e2e trafficProfile=none simTime=300 slotInterval=60 numSlots=6 lastSlotTime=300
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[OBS] log opened: e2e_link_obs.csv
[OBS] build=2026-04-15-rf-v2 feederSource=orbiter_rxfeeder
[OBS] traces connected:  feeder=66  service=66  isl=264
[OBS] snapshot interval=10s  dropAlertThresh=50%
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)
[CHKPT] 0s | Initialize: start
[CHKPT] 0s | LoadISLDefs: start | path=/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: start
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | Initialize: done | satellites=66 isls=132
[CHKPT] 0s | PrecomputeAllTables: start | slots=6
[CHKPT] PrecomputeAllTables: slot=0 t=0s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=1 t=60s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=2 t=120s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=3 t=180s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=4 t=240s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=5 t=300s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=1ms

[CASE] gw2gw_e2e | gwSrc=0 gwDst=1
[CHKPT] 0s | PrecomputeGwRoutes: start | gws=2 pairs=1 slots=6
[GwRouting] slot=0 t=0s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=1 t=60s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=2 t=120s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=2sats
[GwRouting] slot=3 t=180s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats
[GwRouting] slot=4 t=240s GW0[TW-Taipei]=1sats GW1[JP-Tokyo]=3sats
[GwRouting] slot=5 t=300s GW0[TW-Taipei]=2sats GW1[JP-Tokyo]=3sats
[CHKPT] 0s | PrecomputeGwRoutes: done | wall=0ms

=== GW-to-GW Route Report (v6) ===

  [TW-Taipei → JP-Tokyo]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15                                                15      N/A             
1     60        15      15                                                15      N/A             
2     120       15      15                                                15      N/A             
3     180       15      15                                                15      N/A             
4     240       15      15                                                15      N/A             
5     300       44      44                                                44      N/A               <-- ROUTE CHANGED

  [JP-Tokyo → TW-Taipei]
slot  time(s)   entry   ISL_path                                          exit    isl_cost(s)     
--------------------------------------------------------------------------------------------------
0     0         15      15                                                15      N/A             
1     60        15      15                                                15      N/A             
2     120       15      15                                                15      N/A             
3     180       15      15                                                15      N/A             
4     240       15      15                                                15      N/A             
5     300       44      44                                                44      N/A               <-- ROUTE CHANGED

===================================

[OBS] scope: feeder=2 service=0 isl=0
[OBS] WARNING: active segment has empty scope; related OBS output may be suppressed

[E2E] pathType=gw2gw_e2e includesIsl=yes segments={feederlink=on, isl=on, servicelink=off} traffic={sharedEdge=off, islBg=off, gw2gwBg=off, gw2gwDirect=on}
[E2E][feederlink] enabled
[GW2GW_DIRECT] GW0_user=90.2.0.2 -> GW1_user=90.2.0.3 start=1s stop=299s
[E2E][isl] enabled
[E2E][isl] routing/transit enabled without extra ISL-only load generator
[E2E][servicelink] disabled
[CHKPT] 0s | ScheduleRoutingUpdates: 6 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=1ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=1ms recompute=1ms recomputedSrc=17
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=26/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=1ms recompute=0ms recomputedSrc=26
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=30/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=0ms recompute=0ms recomputedSrc=30

[GW2GW_DIRECT] === Delivery summary ===
  src: GW0 (90.2.0.2)
  dst: GW1 (90.2.0.3)
  received: 1525248 bytes (~2979 pkts)
  [PASS] received>0, gateway-to-gateway path is active
======================================

[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=31/66 wall=0ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=1ms recompute=0ms recomputedSrc=31

=== E2E Link Observability Final Summary ===
link                    rx_pkts   rx_bytes     drop_pkts  drop_rate(%)  avg_delay(ms)
------------------------------------------------------------------------------------
feeder:sat15            0         0            0          0.00          0.00
feeder:sat44            0         0            0          0.00          0.00
------------------------------------------------------------------------------------
Log: e2e_link_obs.csv
=============================================


=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.00      0           0               0               NO          
1     60.00     0           0               3               YES         
2     120.00    1           1               17              YES         
3     180.00    1           0               26              YES         
4     240.00    0           0               30              YES         
5     300.00    1           0               31              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
9       4       15      0.0000          0.6816          
13      6       17      0.0000          0.2772          
16      8       9       0.0000          0.0923          
17      8       19      0.0000          0.5070          
18      9       10      0.0923          0.2243          
19      9       20      0.0923          0.0000          
20      0       10      0.0000          0.2243          
21      10      21      0.2243          0.0000          
28      14      15      0.0000          0.6816          
30      15      16      0.6816          0.0000          
31      15      26      0.6816          0.0000          
32      16      17      0.0000          0.2772          
34      17      18      0.2772          0.0000          
35      17      28      0.2772          0.0000          
36      18      19      0.0000          0.5070          
38      19      20      0.5070          0.0000          
39      19      30      0.5070          0.0000          
45      22      33      0.0000          0.1107          
55      27      38      0.0000          0.1215          
61      30      41      0.0000          0.1318          
66      33      34      0.1107          0.0000          
67      33      44      0.1107          0.0000          
74      37      38      0.0000          0.1215          
76      38      39      0.1215          0.0000          
77      38      49      0.1215          0.0000          
79      39      50      0.0000          0.2212          
80      40      41      0.0000          0.1318          
82      41      42      0.1318          0.0000          
83      41      52      0.1318          0.5231          
86      33      43      0.1107          0.0000          
98      49      50      0.0000          0.2212          
100     50      51      0.2212          0.0000          
101     50      61      0.2212          0.0000          
102     51      52      0.0000          0.5231          
104     52      53      0.5231          0.0000          
105     52      63      0.5231          0.3609          
107     53      64      0.0000          0.2115          
109     54      65      0.0000          0.2844          
124     62      63      0.0000          0.3609          
126     63      64      0.3609          0.2115          
127     8       63      0.0000          0.3609          
128     64      65      0.2115          0.2844          
129     9       64      0.0923          0.2115          
130     55      65      0.0000          0.2844          
131     10      65      0.2243          0.2844          
Loaded ISL links: 45 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
  (all ISLs: 0 drops)
TOTAL: 15859375 pkts, 0 dropped | drop_rate=0.000% | success_rate=100.000%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 1659.143 s
Event count:     0
```