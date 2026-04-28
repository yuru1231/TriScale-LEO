```
./ns3 run "test-iridium \
  --mode=sat2ut \
  --gwId=0 \
  --utId=0 \
  --utLatDeg=25.0330 \
  --utLonDeg=121.5654 \
  --utName=UT-Taipei \
  --simTime=120 \
  --slotInterval=60 \
  --beamId=1 \
  --islMaxDistKm=5000 \
  --islRateMbps=10 \
  --emaAlpha=0.3 \
  --changeThresh=0.1 \
  --cooldownRatio=0.5 \
  --elevMinDeg=5 \
  --enableServicelink=1 \
  --obsLogPath=Logs/0415_sat2ut_refactor_link_obs.csv \
  --obsInterval=10 \
  --obsDropAlertPct=50" | tee "Topology & ISL Routing/Outputs/E2E-Refactor/sat2ut_ut0_120s_re.md" 
```
```
Consolidate compiler generated dependencies of target satellite
Consolidate compiler generated dependencies of target scratch_test-iridium
[CFG] pathType=sat2ut trafficProfile=none simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[OBS] log opened: Logs/0415_sat2ut_refactor_link_obs.csv
[OBS] traces connected:  feeder=66  service=66  isl=264
[OBS] snapshot interval=10s  dropAlertThresh=50%
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)

[E2E] pathType=sat2ut includesIsl=no segments={feederlink=off, isl=off, servicelink=on} traffic={sharedEdge=on, islBg=off, gw2gwBg=off, gw2gwDirect=off}
[E2E][feederlink] disabled
[E2E][isl] disabled
[E2E][servicelink] enabled
[TRAFFIC][servicelink] gwId=0 gwUsers=1 utUsers=91 start=1s stop=119s
[TRAFFIC][servicelink] FWD installed: interval=100ms pktSize=1500B rate~120 kbps/flow
[TRAFFIC][servicelink] RTN installed: interval=500ms pktSize=512B rate~8.192 kbps/flow
[CHKPT] 0s | Initialize: start
[CHKPT] 0s | LoadISLDefs: start | path=/home/wenj/workspace/ns-3.43/contrib/satellite/data/scenarios/constellation-iridium-66-sats-fixed/positions/isls.txt
[CHKPT] 0s | LoadISLDefs: done | loaded=132 ISLs
[CHKPT] 0s | InitOrbiterDevices: start
[CHKPT] 0s | InitOrbiterDevices: done | satellites=66
[CHKPT] 0s | Initialize: done | satellites=66 isls=132
[CHKPT] 0s | PrecomputeAllTables: start | slots=3
[CHKPT] PrecomputeAllTables: slot=0 t=0s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=1 t=60s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=2 t=120s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=1ms
[CHKPT] 0s | PrecomputeGwUtRoutes: start | gws=1 uts=1 pairs=1 slots=3
[GwUtRouting] slot=0 t=0s UT0[UT-Taipei]=2sats
[GwUtRouting] slot=1 t=60s UT0[UT-Taipei]=1sats
[GwUtRouting] slot=2 t=120s UT0[UT-Taipei]=1sats
[CHKPT] 0s | PrecomputeGwUtRoutes: done | wall=0ms

[CASE] sat2ut | utId=0 | servicelink only | isl_cost=N/A
[OBS] scope: feeder=0 service=2 isl=0
[CHKPT] 0s | ScheduleRoutingUpdates: 3 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=1ms recompute=0ms recomputedSrc=3
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=16/66 wall=6ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=7ms recompute=6ms recomputedSrc=16

=== E2E Link Observability Final Summary ===
link                    rx_pkts   rx_bytes     drop_pkts  drop_rate(%)  avg_delay(ms)
------------------------------------------------------------------------------------
service:sat15           10713     2911180      0          0.00          1396.82
service:sat45           0         0            0          0.00          0.00
------------------------------------------------------------------------------------
Log: Logs/0415_sat2ut_refactor_link_obs.csv
=============================================


=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.00      0           0               0               NO          
1     60.00     1           0               3               YES         
2     120.00    7           6               16              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
16      8       9       0.0000          0.2690          
17      8       19      0.0000          0.2690          
18      9       10      0.2690          0.0000          
19      9       20      0.2690          0.0000          
36      18      19      0.0000          0.2690          
38      19      20      0.2690          0.0000          
39      19      30      0.2690          0.0000          
55      27      38      0.0000          0.2434          
74      37      38      0.0000          0.2434          
76      38      39      0.2434          0.0000          
77      38      49      0.2434          0.0000          
87      43      54      0.0000          0.4517          
106     53      54      0.0000          0.4517          
108     44      54      0.0000          0.4517          
109     54      65      0.4517          0.1883          
128     64      65      0.0000          0.1883          
129     9       64      0.2690          0.0000          
130     55      65      0.0000          0.1883          
131     10      65      0.0000          0.1883          
Loaded ISL links: 19 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
  (all ISLs: 0 drops)
TOTAL: 6386708 pkts, 0 dropped | drop_rate=0.000% | success_rate=100.000%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 962.880 s
Event count:     0
```