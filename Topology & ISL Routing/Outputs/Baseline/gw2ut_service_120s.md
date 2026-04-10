```
# PLACEHOLDER — 尚未執行
# 執行指令：
# ./ns3 run "scratch/test-iridium_baseline \
#   --mode=gw2ut \
#   --trafficProfile=gw2ut \
#   --gwId=0 \
#   --utId=1 \
#   --utLatDeg=37.8 \
#   --utLonDeg=-122.4 \
#   --utName=UT-SanFrancisco \
#   --simTime=120 --slotInterval=60" \
#   2>&1 | tee output_gw2ut_service_120s.log
#
# 比對目的：與 gw2ut_service_630s.md 形成時間維度對比（120s vs 630s）
# 預期觀察：3 slots / route change 次數 / ISL drop rate / Loaded ISL links
```

```
   --mode=gw2ut \
   --trafficProfile=gw2ut \
   --gwId=0 \
   --utId=1 \
   --utLatDeg=37.8 \
   --utLonDeg=-122.4 \
   --utName=UT-SanFrancisco \
   --simTime=120 --slotInterval=60" \
   2>&1 | tee output_gw2ut_service_120s.log
[CFG] mode=gw2ut trafficProfile=gw2ut simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)
[TRAFFIC] profile=gw2ut (normal service traffic)
[TRAFFIC] gwId=0 gwUsers=1 utUsers=91 start=1s stop=119s
[TRAFFIC] FWD installed: interval=100ms pktSize=1500B rate~120 kbps/flow
[TRAFFIC] RTN installed: interval=500ms pktSize=512B rate~8.192 kbps/flow
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

[CASE] gw2ut | gwId=0 utId=1 utLatDeg=37.8 utLonDeg=-122.4
[CHKPT] 0s | PrecomputeGwUtRoutes: start | gws=1 uts=1 pairs=1 slots=3
[GwUtRouting] slot=0 t=0s UT1[UT-SanFrancisco]=1sats
[GwUtRouting] slot=1 t=60s UT1[UT-SanFrancisco]=2sats
[GwUtRouting] slot=2 t=120s UT1[UT-SanFrancisco]=1sats
[CHKPT] 0s | PrecomputeGwUtRoutes: done | wall=0ms

=== GW-to-UT Route Report (v7) ===

  [TW-Taipei → UT-SanFrancisco]
slot  time(s)   entry   ISL_path                                          serving   isl_cost(s)     
----------------------------------------------------------------------------------------------------
0     0         15      15->14->25->36->37                                37        0.043969        
1     60        15      15->14->25->36->37                                37        0.046214        
2     120       15      15->14->13->2->1                                  1         0.047600          <-- ROUTE CHANGED

===================================

[CHKPT] 0s | ScheduleRoutingUpdates: 3 events scheduled
[CHKPT] 0s | ApplyRoutingTable: slot=0 t=0s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60s | ApplyRoutingTable: slot=1 t=60s | apply=0ms recompute=0ms recomputedSrc=3
[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=16/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=1ms recompute=0ms recomputedSrc=16

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       1           0               16              YES         
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

Total wall time: 719.305 s
Event count:     0
```