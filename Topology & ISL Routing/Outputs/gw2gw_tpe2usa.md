# gw2gw 驗證：TW-Taipei → US-SanFrancisco（長距離跨太平洋）

## 測試指令

```bash
./ns3 run "scratch/test-iridium --mode=gw2gw --gwSrc=0 --gwDst=2"
```

| 參數 | 值 |
|------|-----|
| mode | gw2gw |
| gwSrc | 0 (TW-Taipei, 25.0°N 121.5°E) |
| gwDst | 2 (US-SanFrancisco, 37.8°N 122.4°W) |
| simTime | 630s |
| slotInterval | 60s |
| numSlots | 11 (slot 0–10) |
| islMaxDistKm | 5000 km |

## 預期現象

與 TW→JP（約 2100 km，同顆衛星覆蓋）不同，TW→USA 跨太平洋距離約 9000 km：
- **entry ≠ exit**：起終點由不同衛星覆蓋，中間需要多跳 ISL 路由
- **isl_cost > 0**：多跳累積傳播延遲
- **路徑切換更頻繁**：長弧路由中間節點更容易因星座移動而換路

---

## 實驗輸出

<!-- 請將 SNS3 執行結果貼於此處 -->

```
(待填入)
```

---

## 結果分析
| slot | time(s) | entry | ISL_path       | exit | isl_cost(s) | 路徑變化 |
| ---- | ------- | ----- | -------------- | ---- | ----------- | ---- |
| 0    | 0       | 15    | 15→14→25→36→37 | 37   | 0.043969    | -    |
| 1    | 60      | 15    | 15→14→25→36→37 | 37   | 0.046214    | -    |
| 2    | 120     | 15    | 15→14→13→2→1   | 1    | 0.047600    | ✔    |
| 3    | 180     | 15    | 15→14→25→36    | 36   | 0.037717    | ✔    |
| 4    | 240     | 15    | 15→14→25→36    | 36   | 0.040054    | -    |
| 5    | 300     | 15    | 15→14→25→36    | 36   | 0.042347    | -    |
| 6    | 360     | 44    | 44→45→46→35→36 | 36   | 0.044439    | ✔    |
| 7    | 420     | 14    | 14→13→24→35→36 | 36   | 0.040314    | ✔    |
| 8    | 480     | 14    | 14→13→24→35→36 | 36   | 0.042179    | -    |
| 9    | 540     | 44    | 44→45→56→1→0   | 0    | 0.043645    | ✔    |
| 10   | 600     | 44    | 44→45→56→1→0   | 0    | 0.041591    | -    |

### ISL 路徑變化觀察

- 總路徑切換次數（`<-- ROUTE CHANGED`）：5次
- isl_cost 範圍：0.0377 ~ 0.0476 s
- ISL 跳數範圍：4 ~ 5 跳

```
[CFG] mode=gw2gw simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
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
[CHKPT] 0s | PrecomputeAllTables: complete | wall=5ms

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
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=13/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=13
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=20/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=0ms recompute=0ms recomputedSrc=20
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=25/66 wall=1ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=1ms recompute=1ms recomputedSrc=25
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=26/66 wall=0ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=0ms recompute=0ms recomputedSrc=26
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=36/66 wall=1ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=2ms recompute=1ms recomputedSrc=36
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=41/66 wall=0ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=0ms recompute=0ms recomputedSrc=41
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=44/66 wall=0ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=0ms recompute=0ms recomputedSrc=44
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=46/66 wall=0ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=0ms recompute=0ms recomputedSrc=46
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=50/66 wall=0ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=0ms recompute=0ms recomputedSrc=50

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       0           0               13              YES         
3     180       0           0               20              YES         
4     240       1           1               25              YES         
5     300       0           0               26              YES         
6     360       2           1               36              YES         
7     420       0           0               41              YES         
8     480       0           0               44              YES         
9     540       0           0               46              YES         
10    600       0           0               50              YES         
==============================

Total wall time: 2636.79 s
Event count:     0
```