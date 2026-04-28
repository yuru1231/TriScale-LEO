```
./ns3 run "scratch/test-iridium \
  --mode=gw2gw_e2e \
  --trafficProfile=gw2gw \
  --gwSrc=0 --gwDst=1 \
  --simTime=300 --slotInterval=60" \
  2>&1 | tee "Topology & ISL Routing/Outputs/gw2gw_bg_300s.log"
```
```
[CFG] pathType=gw2gw_e2e trafficProfile=gw2gw simTime=300 slotInterval=60 numSlots=6 lastSlotTime=300
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

[E2E] pathType=gw2gw_e2e includesIsl=yes segments={feederlink=on, isl=on, servicelink=on} traffic={sharedEdge=off, islBg=off, gw2gwBg=on, gw2gwDirect=off}
[E2E][feederlink] enabled
[TRAFFIC][gw2gw] install GW-side background load for gwSrc=0 and gwDst=1
[TRAFFIC][feederlink] gwId=0 gwUsers=1 utUsers=91 start=1s stop=299s
[TRAFFIC][feederlink] FWD installed: interval=60ms pktSize=1500B rate~200 kbps/flow
[TRAFFIC][feederlink] RTN installed: interval=60ms pktSize=1024B rate~136.533 kbps/flow
[TRAFFIC][servicelink] gwId=1 gwUsers=1 utUsers=91 start=1s stop=299s
[TRAFFIC][servicelink] FWD installed: interval=60ms pktSize=1500B rate~200 kbps/flow
[TRAFFIC][servicelink] RTN installed: interval=60ms pktSize=1024B rate~136.533 kbps/flow
[E2E][isl] enabled
[E2E][isl] routing/transit enabled without extra ISL-only load generator
[E2E][servicelink] enabled
[E2E][servicelink] reusing upstream edge traffic installation
[CHKPT] 0s | ScheduleRoutingUpdates: 6 events scheduled
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

=== E2E Link Observability Final Summary ===
link                    rx_pkts   rx_bytes     drop_pkts  drop_rate(%)  avg_delay(ms)
------------------------------------------------------------------------------------
feeder:sat15            262011    402212546    0          0.00          7.71
feeder:sat44            0         0            0          0.00          0.00
------------------------------------------------------------------------------------
Log: e2e_link_obs.csv
=============================================


=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.00      0           0               0               NO          
1     60.00     0           0               9               YES         
2     120.00    0           0               17              YES         
3     180.00    0           0               25              YES         
4     240.00    0           0               31              YES         
5     300.00    0           0               33              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
9       4       15      0.0000          0.2690          
13      6       17      0.0000          0.2772          
15      7       18      0.0000          0.4561          
17      8       19      0.0000          0.8474          
28      14      15      0.8385          0.2690          
30      15      16      0.2690          0.0000          
31      15      26      0.2690          0.0000          
32      16      17      0.0000          0.2772          
34      17      18      0.2772          0.4561          
35      17      28      0.2772          0.0000          
36      18      19      0.4561          0.8474          
37      18      29      0.4561          0.0000          
38      19      20      0.8474          0.0000          
39      19      30      0.8474          0.0000          
45      22      33      0.0000          0.1107          
49      24      35      0.0000          0.2474          
63      31      42      0.0000          0.1704          
66      33      34      0.1107          0.0000          
67      33      44      0.1107          0.0000          
68      34      35      0.0000          0.2474          
70      35      36      0.2474          0.0000          
71      35      46      0.2474          0.0000          
72      36      37      0.0000          0.4333          
79      39      50      0.0000          0.2227          
82      41      42      0.0000          0.1704          
83      41      52      0.0000          0.2988          
84      42      43      0.1704          0.0000          
85      42      53      0.1704          0.0646          
86      33      43      0.1107          0.0000          
87      43      54      0.0000          0.1549          
98      49      50      0.0000          0.2227          
100     50      51      0.2227          0.0000          
101     50      61      0.2227          0.0000          
102     51      52      0.0000          0.2988          
104     52      53      0.2988          0.0646          
105     52      63      0.2988          0.0951          
106     53      54      0.0646          0.1549          
107     53      64      0.0646          0.0000          
108     44      54      0.0000          0.1549          
109     54      65      0.1549          0.1495          
117     3       58      0.0000          0.1794          
124     62      63      0.0000          0.0951          
126     63      64      0.0951          0.0000          
127     8       63      0.0000          0.0951          
128     64      65      0.0000          0.1495          
130     55      65      0.0000          0.1495          
131     10      65      0.0000          0.1495          
Loaded ISL links: 47 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
ISL           total_pkts  dropped   drop_rate(%)  success_rate(%)
--------------------------------------------------------------
14-15         203319      272       0.1330        99.866
--------------------------------------------------------------
TOTAL: 16490288 pkts, 272 dropped | drop_rate=0.002% | success_rate=99.998%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 2220.604 s
Event count:     0
```