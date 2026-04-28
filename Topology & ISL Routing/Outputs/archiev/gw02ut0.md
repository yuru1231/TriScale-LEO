[CHKPT] 120s | ApplyRoutingTable: slot=2 HasSignificantChange=YES
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=16/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=16

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       0           0               16              YES         
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

Total wall time: 618.6980 s
Event count:     0
