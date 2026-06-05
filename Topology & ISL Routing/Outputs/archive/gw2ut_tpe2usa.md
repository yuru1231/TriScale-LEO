# gw2ut 驗證：GW@TW-Taipei → UT@US-SanFrancisco（長距離）

## 測試指令

```bash
./ns3 run "scratch/test-iridium --mode=gw2ut --gwId=0 --utId=1 --utLatDeg=37.8 --utLonDeg=-122.4 --utName=UT-SanFrancisco"
```

| 參數 | 值 |
|------|-----|
| mode | gw2ut |
| gwId | 0 (TW-Taipei, 25.0°N 121.5°E) |
| utId | 1 |
| utLatDeg | 37.8°N |
| utLonDeg | -122.4°W (San Francisco) |
| utName | UT-SanFrancisco |
| simTime | 630s |
| slotInterval | 60s |
| numSlots | 11 (slot 0–10) |
| islMaxDistKm | 5000 km |

## 預期現象

gw2ut 相較 gw2gw 多了一段 UT 可見衛星選擇邏輯（`servingSatId` 為 UT 最佳仰角衛星）：
- **entry ≠ serving**：GW 接入衛星與 UT 服務衛星不同，需 ISL 橋接
- **isl_cost > 0**：多跳跨太平洋傳播延遲
- **serving 衛星切換**：UT@SF 可見衛星隨星座移動而切換，帶動整條路徑重算
- 比較 gw2gw 結果：entry 側（GW@Taipei）行為應相似，差異出現在 serving 側

---

## 實驗輸出

<!-- 請將 SNS3 執行結果貼於此處 -->

```
(待填入)
```

---

## 結果分析

| slot | time(s) | entry | ISL_path       | serving | isl_cost(s) | 路徑變化 |
| ---- | ------- | ----- | -------------- | ------- | ----------- | ---- |
| 0    | 0       | 15    | 15→14→25→36→37 | 37      | 0.043969    | -    |
| 1    | 60      | 15    | 15→14→25→36→37 | 37      | 0.046214    | -    |
| 2    | 120     | 15    | 15→14→13→2→1   | 1       | 0.047600    | ✔    |
| 3    | 180     | 15    | 15→14→25→36    | 36      | 0.037717    | ✔    |
| 4    | 240     | 15    | 15→14→25→36    | 36      | 0.040054    | -    |
| 5    | 300     | 15    | 15→14→25→36    | 36      | 0.042347    | -    |
| 6    | 360     | 44    | 44→45→46→35→36 | 36      | 0.044439    | ✔    |
| 7    | 420     | 14    | 14→13→24→35→36 | 36      | 0.040314    | ✔    |
| 8    | 480     | 14    | 14→13→24→35→36 | 36      | 0.042179    | -    |
| 9    | 540     | 44    | 44→45→56→1→0   | 0       | 0.043645    | ✔    |
| 10   | 600     | 44    | 44→45→56→1→0   | 0       | 0.041591    | -    |


### ISL 路徑變化觀察

總路徑切換次數（<-- ROUTE CHANGED）：5 次
isl_cost 範圍：0.0377 ~ 0.0476 s
ISL 跳數範圍：4 ~ 5 跳
UT 可見衛星數量變化：
→ 1 ~ 2 顆衛星（動態變化）

（依據 log：slot 1 / 3 / 9 / 10 出現 2 sats，其餘為 1 sats）

### 與 gw2gw_tpe2usa 比較

| 指標             | gw2gw                 | gw2ut                  |
| -------------- | --------------------- | ---------------------- |
| 路徑切換次數         | 較少（通常只因 ISL topology） | **較多（+UT serving 切換）** |
| isl_cost 範圍(s) | 穩定                    | **略波動（受 serving 影響）**  |
| ISL 跳數範圍       | 穩定                    | **4–5 跳（仍穩定）**         |



```
[CFG] mode=gw2ut simTime=630 slotInterval=60 numSlots=11 lastSlotTime=600
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
[CHKPT] 120s | RecomputeAffectedRoutes: slot=2 recomputed=17/66 wall=0ms
[CHKPT] 120s | ApplyRoutingTable: slot=2 t=120s | apply=0ms recompute=0ms recomputedSrc=17
[CHKPT] 180s | ApplyRoutingTable: slot=3 HasSignificantChange=YES
[CHKPT] 180s | RecomputeAffectedRoutes: slot=3 recomputed=26/66 wall=0ms
[CHKPT] 180s | ApplyRoutingTable: slot=3 t=180s | apply=0ms recompute=0ms recomputedSrc=26
[CHKPT] 240s | ApplyRoutingTable: slot=4 HasSignificantChange=YES
[CHKPT] 240s | RecomputeAffectedRoutes: slot=4 recomputed=30/66 wall=0ms
[CHKPT] 240s | ApplyRoutingTable: slot=4 t=240s | apply=1ms recompute=0ms recomputedSrc=30
[CHKPT] 300s | ApplyRoutingTable: slot=5 HasSignificantChange=YES
[CHKPT] 300s | RecomputeAffectedRoutes: slot=5 recomputed=31/66 wall=0ms
[CHKPT] 300s | ApplyRoutingTable: slot=5 t=300s | apply=1ms recompute=0ms recomputedSrc=31
[CHKPT] 360s | ApplyRoutingTable: slot=6 HasSignificantChange=YES
[CHKPT] 360s | RecomputeAffectedRoutes: slot=6 recomputed=39/66 wall=0ms
[CHKPT] 360s | ApplyRoutingTable: slot=6 t=360s | apply=0ms recompute=0ms recomputedSrc=39
[CHKPT] 420s | ApplyRoutingTable: slot=7 HasSignificantChange=YES
[CHKPT] 420s | RecomputeAffectedRoutes: slot=7 recomputed=42/66 wall=3ms
[CHKPT] 420s | ApplyRoutingTable: slot=7 t=420s | apply=4ms recompute=3ms recomputedSrc=42
[CHKPT] 480s | ApplyRoutingTable: slot=8 HasSignificantChange=YES
[CHKPT] 480s | RecomputeAffectedRoutes: slot=8 recomputed=46/66 wall=0ms
[CHKPT] 480s | ApplyRoutingTable: slot=8 t=480s | apply=1ms recompute=0ms recomputedSrc=46
[CHKPT] 540s | ApplyRoutingTable: slot=9 HasSignificantChange=YES
[CHKPT] 540s | RecomputeAffectedRoutes: slot=9 recomputed=44/66 wall=0ms
[CHKPT] 540s | ApplyRoutingTable: slot=9 t=540s | apply=0ms recompute=0ms recomputedSrc=44
[CHKPT] 600s | ApplyRoutingTable: slot=10 HasSignificantChange=YES
[CHKPT] 600s | RecomputeAffectedRoutes: slot=10 recomputed=50/66 wall=2ms
[CHKPT] 600s | ApplyRoutingTable: slot=10 t=600s | apply=2ms recompute=2ms recomputedSrc=50

=== IslRoutingManager Stats ===
slot  simTime   apply(ms)   recompute(ms)   recompSrc       changed     
0     0         0           0               0               NO          
1     60        0           0               3               YES         
2     120       0           0               17              YES         
3     180       0           0               26              YES         
4     240       1           0               30              YES         
5     300       1           0               31              YES         
6     360       0           0               39              YES         
7     420       4           3               42              YES         
8     480       1           0               46              YES         
9     540       0           0               44              YES         
10    600       2           2               50              YES         
==============================

Total wall time: 2921.66 s
Event count:     0
```