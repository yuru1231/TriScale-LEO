# 工作日誌 2026-04-07

## 目標

延伸 ISL Routing 驗證範圍，從短距離（TW-Taipei ↔ JP-Tokyo，~2100km）擴展至長距離跨太平洋路由（TW-Taipei ↔ US-SanFrancisco，~9000km），驗證以下假設：

1. 長距離路由下 entry ≠ exit（多顆衛星覆蓋兩端）
2. ISL hop 數 > 0，isl_cost > 0
3. 路徑切換更頻繁（長弧中間節點對星座移動更敏感）

---

## 完成事項

### 1. gw2gw 長距離驗證（TW-Taipei → US-SanFrancisco）

**測試指令**：
```bash
./ns3 run "scratch/test-iridium --mode=gw2gw --gwSrc=0 --gwDst=2"
```

**現象**：11 個時槽中共發生 **5 次 ROUTE CHANGED**，且每槽 isl_cost > 0，entry ≠ exit。

| slot | time(s) | entry | ISL_path               | exit | isl_cost(s) | 變化 |
|------|---------|-------|------------------------|------|-------------|------|
| 0    | 0       | 15    | 15→14→25→36→37         | 37   | 0.043969    | -    |
| 1    | 60      | 15    | 15→14→25→36→37         | 37   | 0.046214    | -    |
| 2    | 120     | 15    | 15→14→13→2→1           | 1    | 0.047600    | ✔    |
| 3    | 180     | 15    | 15→14→25→36            | 36   | 0.037717    | ✔    |
| 4    | 240     | 15    | 15→14→25→36            | 36   | 0.040054    | -    |
| 5    | 300     | 15    | 15→14→25→36            | 36   | 0.042347    | -    |
| 6    | 360     | 44    | 44→45→46→35→36         | 36   | 0.044439    | ✔    |
| 7    | 420     | 14    | 14→13→24→35→36         | 36   | 0.040314    | ✔    |
| 8    | 480     | 14    | 14→13→24→35→36         | 36   | 0.042179    | -    |
| 9    | 540     | 44    | 44→45→56→1→0           | 0    | 0.043645    | ✔    |
| 10   | 600     | 44    | 44→45→56→1→0           | 0    | 0.041591    | -    |

**觀察**：
- ISL 跳數：**4～5 跳**（與 TW→JP 的 0 跳形成對比）
- isl_cost 範圍：**0.0377～0.0476 s**（純傳播延遲）
- 路徑切換次數：**5/10**（TW→JP 為 2/10）
- 雙向路徑完全對稱（SF→TPE 路徑為 TPE→SF 的逆序）✅
- exit 衛星在 slot 9 由 sat=36 大幅跳換至 sat=0，代表 SF 側可見衛星集合切換到完全不同的星群

**原因**：距離長達 ~9000km，Dijkstra 必須跨越多顆衛星，中間節點集合隨星座移動而更頻繁改變最短路徑。GW0（Taipei）入口依然是 sat=15→44→14 的週期切換（與 TW→JP 一致），但 GW2（SF）出口切換獨立於 GW0，造成整條路徑更多樣的組合切換。

**結論**：長距離路由下 ISL 路徑確實更頻繁切換，且 isl_cost 非零，驗證假設正確 ✅

---

### 2. gw2ut 長距離驗證（TW-Taipei → UT-SanFrancisco）

**測試指令**：
```bash
./ns3 run "scratch/test-iridium --mode=gw2ut --gwId=0 --utId=1 --utLatDeg=37.8 --utLonDeg=-122.4 --utName=UT-SanFrancisco"
```

**現象**：路徑與切換時機和 gw2gw TPE→SF **完全一致**（5 次 ROUTE CHANGED 在相同槽，isl_cost 每槽相同）。serving 衛星在各槽等於 gw2gw 的 exit 衛星。

| slot | time(s) | entry / ISL_path / serving | isl_cost(s) |
|------|---------|---------------------------|-------------|
| 0–1 | 0–60 | SAT15 / `15->14->25->36->37` / SAT37 | 0.0440–0.0462 |
| 2 | 120 | SAT15 / `15->14->13->2->1` / SAT1 **← ROUTE CHANGED** | 0.0476 |
| 3–5 | 180–300 | SAT15 / `15->14->25->36` / SAT36 **← ROUTE CHANGED @ slot=3** | 0.0377–0.0423 |
| 6 | 360 | SAT44 / `44->45->46->35->36` / SAT36 **← ROUTE CHANGED** | 0.0444 |
| 7–8 | 420–480 | SAT14 / `14->13->24->35->36` / SAT36 **← ROUTE CHANGED @ slot=7** | 0.0403–0.0422 |
| 9–10 | 540–600 | SAT44 / `44->45->56->1->0` / SAT0 **← ROUTE CHANGED @ slot=9** | 0.0412–0.0436 |

Wall time: 2921.66s

**原因**：UT-SanFrancisco（37.8°N/122.4°W）與 GW2-SanFrancisco（37.8°N/122.4°W）座標完全相同。`ComputeElevationDeg()` 對兩者計算結果一致，UT 的 serving 候選集 = GW2 的 exit 候選集，Dijkstra 路徑選擇完全相同。這與短距離 TW→JP 的情況相反——短距離時 UT-Taipei 與 GW-Taipei 有 ~10km 差距，邊界槽可見性不同，導致切換晚一槽。

**結論**：**當 UT 與 GW 座標重合時，gw2ut = gw2gw（路徑與時機完全一致）**；座標差距才是 gw2ut 與 gw2gw 產生切換時機差異的根本原因 ✅

---

### 3. 文件更新

今日更新（含本次補齊）：

- [X] 更新 [Layer1.md](https://github.com/bmw-ntust-internship/Lucy/blob/3a4e19a4811f8fe2d0805da11cf9d13928dcdee4/TriScale-LEO/Topology%20%26%20ISL%20Routing/Predict/Readme.md)：新增 gw2gw TPE→SF 長距離驗證段落（路由切換表、說明）
- [X] 更新 [Readme.md](https://github.com/bmw-ntust-internship/Lucy/blob/3a4e19a4811f8fe2d0805da11cf9d13928dcdee4/TriScale-LEO/Readme.md)：新增 mode=gw2gw（TW→USA）驗證摘要
- [X] 建立 [gw2gw_tpe2usa.md](https://github.com/bmw-ntust-internship/Lucy/blob/3a4e19a4811f8fe2d0805da11cf9d13928dcdee4/TriScale-LEO/Topology%20%26%20ISL%20Routing/Predict/Result/gw2gw_tpe2usa.md)：已回填完整執行輸出與結果分析
- [X] 建立 [gw2ut_tpe2usa.md](https://github.com/bmw-ntust-internship/Lucy/blob/3a4e19a4811f8fe2d0805da11cf9d13928dcdee4/TriScale-LEO/Topology%20%26%20ISL%20Routing/Predict/Result/gw2ut_tpe2usa.md)：已回填完整執行輸出與結果分析
- [X] 建立 [Architecture_and_Native_Comparison.md](https://github.com/bmw-ntust-internship/Lucy/blob/3a4e19a4811f8fe2d0805da11cf9d13928dcdee4/TriScale-LEO/Topology%20%26%20ISL%20Routing/Predict/Architecture_and_Native_Comparison.md)：分析原生模組差異


---

## 驗證結果總表

| 驗證項目 | 觀察輸出 | 結論 |
|----------|----------|------|
| gw2gw TPE→SF：entry ≠ exit | entry 與 exit 每槽均為不同衛星 | 長距離多衛星覆蓋 ✅ |
| gw2gw TPE→SF：isl_cost > 0 | 0.0377～0.0476 s，4～5 ISL 跳 | 跨 ISL 傳播延遲正確 ✅ |
| gw2gw TPE→SF：路徑切換頻率 | 5 次 / 10 槽（TW→JP 僅 2 次） | 長弧路由切換更頻繁 ✅ |
| gw2gw TPE→SF：雙向對稱 | SF→TPE 路徑為 TPE→SF 逆序，cost 完全一致 | Dijkstra 對稱正確 ✅ |
| gw2ut TPE→UT@SF：路徑與 gw2gw 一致 | 5 次切換相同槽，isl_cost 每槽相同，serving=exit | UT 座標=GW2 座標，結果完全對齊 ✅ |
| gw2ut vs gw2gw 切換差異原因 | TW→JP：UT 與 GW 有 ~10km 差異 → 切換晚一槽；TW→SF：UT 與 GW 座標相同 → 完全一致 | 座標差距是 serving/exit 切換時機分離的根本原因 ✅ |

---

## 修改的檔案

| 檔案 | 修改內容 |
|------|----------|
| `Topology & ISL Routing/Outputs/gw2gw_tpe2usa.md` | 新建，回填 gw2gw TPE→SF 完整執行輸出與結果分析 |
| `Topology & ISL Routing/Outputs/gw2ut_tpe2usa.md` | 新建，已回填 gw2ut TPE→SF 完整執行輸出與結果分析 |
| `Topology & ISL Routing/Layer1.md` | 新增 gw2gw 與 gw2ut TPE→SF 長距離驗證段落；修正 recompSrc 趨勢說明與 HasSignificantChange 解釋（兩處事實錯誤） |
| `Readme.md` | 新增 mode=gw2gw（TW→USA）驗證摘要；更新 Section 7 資料結構定義 |

> 今日無程式碼修改，僅執行驗證實驗並記錄輸出結果。

---




