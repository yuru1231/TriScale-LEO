```
./ns3 run "scratch/test-iridium \
  --mode=sat2sat \
  --satSrc=0 --satDst=10 \
  --simTime=120 --slotInterval=60 \
  --enableIsl=1" \
  2>&1 | tee "Topology & ISL Routing/Outputs/sat2sat.log"
```
```
[CFG] pathType=sat2sat trafficProfile=none simTime=120 slotInterval=60 numSlots=3 lastSlotTime=120
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
[CHKPT] 0s | PrecomputeAllTables: start | slots=3
[CHKPT] PrecomputeAllTables: slot=0 t=0s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=1 t=60s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] PrecomputeAllTables: slot=2 t=120s | SAT0_routes=65 dijkstra=0ms total=0ms
[CHKPT] 0s | PrecomputeAllTables: complete | wall=1ms

[CASE] sat2sat | src=0 dst=10

=== Route Report: Full Paths Across Slots ===
time(s)   src   dst   full_path                                   route_cost    slot  
--------------------------------------------------------------------------------------
0         0     10    0->10                                       0.013165      0     
60.000000 0     10    0->10                                       0.013166      1     
120.0000000     10    0->10                                       0.013167      2     

==============================================
[OBS] scope: feeder=0 service=0 isl=1

[E2E] pathType=sat2sat includesIsl=yes segments={feederlink=off, isl=on, servicelink=off} traffic={sharedEdge=off, islBg=on, gw2gwBg=off, gw2gwDirect=off}
[E2E][feederlink] disabled
[E2E][isl] enabled
[TRAFFIC][isl] install aggressive background load via GW=0 <-> all UTs
[TRAFFIC][isl] gwId=0 gwUsers=1 utUsers=91 start=1.000000s stop=119.000000s
[TRAFFIC][isl] FWD installed: interval=30ms pktSize=1500B rate~400.000000 kbps/flow
[TRAFFIC][isl] RTN installed: interval=30ms pktSize=1500B rate~400.000000 kbps/flow
[E2E][servicelink] disabled
[CHKPT] 0.000000s | ScheduleRoutingUpdates: 3 events scheduled
[CHKPT] 0.000000s | ApplyRoutingTable: slot=0 t=0.000000s | apply=0ms recompute=0ms recomputedSrc=0
[CHKPT] 60.000000s | ApplyRoutingTable: slot=1 HasSignificantChange=YES
[CHKPT] 60.000000s | RecomputeAffectedRoutes: slot=1 recomputed=13/66 wall=0ms
[CHKPT] 60.000000s | ApplyRoutingTable: slot=1 t=60.000000s | apply=0ms recompute=0ms recomputedSrc=13
[CHKPT] 120.000000s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120.000000s | RecomputeAffectedRoutes: slot=2 recomputed=21/66 wall=0ms
[CHKPT] 120.000000s | ApplyRoutingTable: slot=2 t=120.000000s | apply=1ms recompute=0ms recomputedSrc=21

=== E2E Link Observability Final Summary ===
link                    rx_pkts   rx_bytes     drop_pkts  drop_rate(%)  avg_delay(ms)
------------------------------------------------------------------------------------
isl:0-10                23974     23974        0          0.00          0.00
------------------------------------------------------------------------------------
Log: e2e_link_obs.csv
=============================================


=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0.00      0           0               0               NO          
1     60.00     0           0               13              YES         
2     120.00    1           0               21              YES         
==============================


=== ISL Load Cost Summary (EMA queue delay) ===
edgeIdx satA    satB    loadAB(ms)      loadBA(ms)      
13      6       17      0.0000          0.2102          
17      8       19      0.0000          0.2690          
28      14      15      10.1784         0.0000          
32      16      17      0.0000          0.2102          
34      17      18      0.2102          0.0000          
35      17      28      0.2102          0.0000          
36      18      19      0.0000          0.2690          
38      19      20      0.2690          0.0000          
39      19      30      0.2690          0.0000          
55      27      38      0.0000          0.2434          
74      37      38      0.0000          0.2434          
76      38      39      0.2434          0.0000          
77      38      49      0.2434          0.0000          
87      43      54      0.0000          0.4517          
105     52      63      0.0000          0.1961          
106     53      54      0.0000          0.4517          
107     53      64      0.0000          0.1883          
108     44      54      0.0000          0.4517          
109     54      65      0.4517          0.0000          
124     62      63      0.0000          0.1961          
126     63      64      0.1961          0.1883          
127     8       63      0.0000          0.1961          
128     64      65      0.1883          0.0000          
129     9       64      0.0000          0.1883          
Loaded ISL links: 24 / 132 total ISL edges
================================================


=== ISL Packet Drop Rate Summary ===
ISL           total_pkts  dropped   drop_rate(%)  success_rate(%)
--------------------------------------------------------------
13-14         75746       3         0.0030        99.996
14-15         152989      16646     10.880        89.119
3-14          75855       40        0.0520        99.947
--------------------------------------------------------------
TOTAL: 6926030 pkts, 16689 dropped | drop_rate=0.241% | success_rate=99.759%
[PASS] overall ISL drop rate < 1.000%
=====================================

Total wall time: 988.962 s
Event count:     0
```