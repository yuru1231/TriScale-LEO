```
[CFG] mode=sat2sat trafficProfile=none simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
[ISL_DROP] trace connected: 264 ISL interfaces (132 unique links)
[RBDC] trace skipped (rbdcVerbose=0, use --rbdcVerbose=1 to enable)
[TRAFFIC] profile=none (baseline, no traffic)
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

[CASE] sat2sat | src=0 dst=33

=== Route Report: Full Paths Across Slots ===
time(s)   src   dst   full_path                                   route_cost    slot  
--------------------------------------------------------------------------------------
0         0     33    0->1->2->57->46->35->34->33                 0.078176      0     
60.000000 0     33    0->1->2->57->46->35->34->33                 0.074919      1     
120.0000000     33    0->1->2->57->46->35->34->33                 0.072055      2     

==============================================
[CHKPT] 0.000000s | ScheduleRoutingUpdates: 3 events scheduled
[CHKPT] 0.000000s | ApplyRoutingTable: slot=0 t=0.000000s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60.000000s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60.000000s | RecomputeAffectedRoutes: slot=1 recomputed=3/66 wall=0ms
[CHKPT] 60.000000s | ApplyRoutingTable: slot=1 t=60.000000s | apply=1ms recompute=0ms recomputedSrc=3
[CHKPT] 120.000000s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120.000000s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120.000000s | ApplyRoutingTable: slot=2 t=120.000000s | apply=0ms recompute=0ms recomputedSrc=17

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.000000  0           0               0               NO          
1     60.000000 1           0               3               YES         
2     120.0000000           0               17              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
16      8       9       0.0000          0.2690          
18      9       10      0.2690          0.0000          
19      9       20      0.2690          0.0000          
55      27      38      0.0000          0.3542          
74      37      38      0.0000          0.3542          
76      38      39      0.3542          0.0000          
77      38      49      0.3542          0.0000          
79      39      50      0.0000          0.6449          
98      49      50      0.0000          0.6449          
100     50      51      0.6449          0.0000          
101     50      61      0.6449          0.0000          
105     52      63      0.0000          0.5030          
107     53      64      0.0000          0.2690          
109     54      65      0.0000          0.8292          
124     62      63      0.0000          0.5030          
126     63      64      0.5030          0.2690          
127     8       63      0.0000          0.5030          
128     64      65      0.2690          0.8292          
129     9       64      0.2690          0.2690          
130     55      65      0.0000          0.8292          
131     10      65      0.0000          0.8292          
Loaded ISL links: 21 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
  (all ISLs: 0 drops)
TOTAL: 6340563 pkts, 0 dropped | drop_rate=0.000% | success_rate=100.000%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 544.236 s
Event count:     0
```