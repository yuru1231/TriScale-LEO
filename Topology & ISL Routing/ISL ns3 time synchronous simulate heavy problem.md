原[isl-leo-candidate.cc](https://github.com/bmw-ntust-internship/Lucy/blob/73f30da4b3d6699b68d371a98047c712dd8dc61e/TriScale-LEO/Topology%20%26%20ISL%20Routing/codes/original_isl-leo-candidate.cc)
現[isl-leo-candidate.cc](https://github.com/bmw-ntust-internship/Lucy/blob/2cace6f20ed26d6b0477c01cd319563973663e91/TriScale-LEO/Topology%20%26%20ISL%20Routing/codes/new_isl-leo-candidate.cc)
03/12[isl-leo-candidate.cc](https://github.com/bmw-ntust-internship/Lucy/blob/3c1b6506234cf76c51a10c72e97867ec2b0ac73b/TriScale-LEO/Topology%20%26%20ISL%20Routing/codes/0312_isl-leo-candidate.cc)
## Problems of synchronous
### 1.AdvanceSimulationTo 設計錯誤
```
void AdvanceSimulationTo(double tSec)
{
  if (Simulator::Now().GetSeconds() >= tSec)
    {
      return;  // 直接 return，不等待事件處理完
    }

  Simulator::Stop(Seconds(tSec));
  Simulator::Run();  // 每次都重新 Run，造成多次 Run/Stop 循環
}
```
`Simulator::Run() `被反覆呼叫，每個時間步都重啟一次模擬器。ns3 的 `Simulator::Run() `並非設計來被多次呼叫，
導致：
- 事件佇列狀態不一致
- 時間推進不穩定
- mobility model 更新可能未及時反映

### 2.GraphCache 只建立一次，但拓樸持續變化
```
// main() 中，只在模擬開始前建立一次 cache
GraphCache cache = BuildGraphCache(cfg);

// 之後所有時間步都複用同一個 cache ← ISL adjacency 不會更新！
candidates = RunCandidateScan(cfg, cache);
```
`BuildRealIslAdjacency() `抓取的是當前瞬間的 ISL 連接，但 LEO 衛星軌道移動會讓 ISL 鏈路在仿真過程中改變，cache.adj 卻從未刷新。

### 3.RunCandidateScan 與 d1_final 中時間步重複推進
```
// d1_final 模式：
candidates = RunCandidateScan(cfg, cache);  // 內部推進到 tEnd

// 然後又再跑一次相同的時間迴圈：
for (double t = cfg.tStart; t <= cfg.tEnd + 1e-9; t += cfg.dt)
  {
    AdvanceSimulationTo(t);  // 時間已超過，全部直接 return
    auto rows = EvaluateIslConnectivity(cfg, cache, t, satIds);
    // 結果：全部 rows 都是同一個時間點的數據
  }
```
`isl_connectivity `的資料實際上全是 `tEnd` 時刻的快照，而非各時間步的真實狀態

修改後再次遇到

# routing plan `NO_PATH` 
## 1.islMaxDistanceKm=3000 把太多 ISL 邊切掉，dynamic graph 碎裂
## 2.GetClosestSat() 只選一顆 anchor，只要那顆 anchor 在 disconnected component 裡，全部 NO_PATH

透過參數比對測試
1.
```
./ns3 run "isl-leo-candidate \
--mode=candidate_scan \
--scenarioFolder=constellation-telesat-351-sats \
--simTime=6 --tStart=0 --tEnd=5 --dt=1 \
--refLat=25.0330 --refLon=121.5654 --elevDeg=20 \
--islMaxDistanceKm=3000 --gwAnchorCount=3 \
--gwIndex=0 --statsLevel=min \
--outDir=/tmp/test_3000_a1" 2>&1 | tee /tmp/test_3000_a3.log
```

2.
```
./ns3 run "isl-leo-candidate \
--mode=candidate_scan \
--scenarioFolder=constellation-telesat-351-sats \
--simTime=6 --tStart=0 --tEnd=5 --dt=1 \
--refLat=25.0330 --refLon=121.5654 --elevDeg=20 \
--islMaxDistanceKm=3000 --gwAnchorCount=3 \
--gwIndex=0 --statsLevel=min \
--outDir=/tmp/test_3000_a3" 2>&1 | tee /tmp/test_3000_a3.log
```
3.
```
./ns3 run "isl-leo-candidate \
--mode=candidate_scan \
--scenarioFolder=constellation-telesat-351-sats \
--simTime=6 --tStart=0 --tEnd=5 --dt=1 \
--refLat=25.0330 --refLon=121.5654 --elevDeg=20 \
--islMaxDistanceKm=5000 --gwAnchorCount=3 \
--gwIndex=0 --statsLevel=min \
--outDir=/tmp/test_5000_a3" 2>&1 | tee /tmp/test_5000_a3.log
```
|組合|結果|
|-|-|
|threshold=3000 + anchor=1|❌ NO_PATH|
|threshold=3000 + anchor=3|❌ NO_PATH|
|threshold=5000 + anchor=3|✅ 有 candidate|

# 更新以銜接L2 BH Contorller
## 1.Base ISL adjacency 改為只初始化一次
只在第一次呼叫時建立 g_baseAdj，之後每個時間步直接重用靜態 base adjacency，每步只做距離門檻過濾，形成 dynamic adjacency
> 現在以固定衛星連接為主，判斷距離門檻->dynamic adjacency

## 輸出完整路徑節點序列
後續若要做 path-aware BH policy 或 handover 預備分析會更方便

## visible_duration_sec 改為最長連續可視區段
- 先記錄 serving satellite 在該 window 中成為 candidate 的所有時間點
- 再計算其中 最長連續可視區段
> 可作為 BH dwell/switch 規劃的依據

## ISL connectivity 從 O(N²) 改為 O(E)
新增：`topology_pruned.csv`
