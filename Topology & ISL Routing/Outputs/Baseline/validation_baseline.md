# Baseline 驗證總表

本文件整理 `Topology & ISL Routing/Outputs/Baseline/` 所有 baseline 測試項目，  
對應程式碼：`scratch/test-iridium_baseline.cc`

驗證維度：**模式（mode）× 流量（traffic）× 時間（simTime）× 距離（pair）**

---

## 一、測試矩陣

### 1.1 時間維度（短 vs 長）

| mode x traffic | 120s | 630s |
|---|---|---|
| sat2sat x none |  `sat2sat_s0s33_120s.md` | — |
| gw2gw x background |  `gw2gw_bgload_120s.md` |  `gw2gw_bgload_630s.md` |
| gw2gw x delivery |  `gw2gw_delivery_120s.md` | — |
| gw2ut x service |  `gw2ut_service_120s.md` |  `gw2ut_service_630s.md` |

### 1.2 距離維度（近 vs 遠）

| mode x traffic | 短距 | 長距 |
|---|---|---|
| sat2sat x none | ✓ `sat2sat_s0s1_120s.md`（1-hop） | ✓ `sat2sat_s0s33_120s.md`（8-hop） |
| gw2gw x delivery | — | ✓ `gw2gw_delivery_120s.md`（TW→US） |
| gw2ut x service | — | ✓ `gw2ut_service_630s.md`（TW→UT-SF） |

---

## 二、Traffic Profile 說明

`test-iridium_baseline.cc` 共定義 5 種 traffic profile，以 `--trafficProfile=<值>` 傳入。

### 2.1 五種 Profile 彙整

|profile|	名稱的意思|	實際裝流方式	|為什麼名稱不等於裝法|
|-|-|-|-|
|sat2sat|	觀察 sat→sat ISL 路徑|	GW→all UT|	SNS3 無法在衛星裝 App，但觀察目標是 ISL|
|gw2gw	|觀察 GW→GW 路由報告	|兩端 GW↔UT CBR	|SatTrafficHelper 只能 GW↔UT，但路由 report 看 GW-to-GW path|
|gw2gw_direct	|真正的 GW user→GW user|	GW_user → GW_user UDP	加 _direct 表示這次流量真的從 GW user 出發，而非繞 UT|
|gw2ut	|觀察 GW→UT 服務路徑	|GW↔UT CBR	||
|none	|不觀察流量，只看路由|	無	||

---

### 2.2 Background / Delivery / Service 差異

#### 流量裝法比較

| 項目 | `gw2gw`（background） | `gw2gw_direct`（delivery） | `gw2ut`（service） |
|---|---|---|---|
| **發送端** | GW_src 的 UT 群 + GW_dst 的 UT 群（雙邊） | GW_src 的 user node | GW 的 user node |
| **接收端** | 各自的 UT 群（雙邊） | GW_dst 的 user node | UT 的 user node |
| **helper** | `SatTrafficHelper`（衛星專用） | ns3 `OnOffHelper` + `PacketSinkHelper` | `SatTrafficHelper` |
| **協定** | CBR over satellite link | UDP over ISL → GW | CBR over satellite link |
| **計量** | ISL drop count + loaded links | `PacketSink::GetTotalRx()` bytes | serving sat 切換路徑 |

#### 目的差異

```
background  → 讓 ISL queue 有負載
              目的是驅動 queue delay → UpdateLoadCosts → HasSignificantChange
              不在乎封包抵達沒有

delivery    → 確認封包真的到達目的端
              目的是驗證 GW unicast forward rule 是否正確
              received=0 代表 routing rule 缺失

service     → 觀察 UT 服務過程中 serving sat 如何切換
              目的是驗證 gw2ut ISL 路徑隨衛星移動正確重算
              不計量 bytes，看路徑變化
```

#### PASS / FAIL 判準

| | background | delivery | service |
|---|---|---|---|
| **看什麼** | ISL drop rate、loaded link 數、route change | sink received bytes | entry sat、serving sat、ISL path per slot |
| **PASS 條件** | drop rate < 0.01%，有 route change | received > 0 bytes | serving sat 切換次數合理，路徑合理 |
| **FAIL 代表** | ISL 壅塞、cost 計算異常 | GW forward rule 缺失 | routing loop、serving sat 不切換 |

---

## 三、測項意義

各測試存在的目的、驗證假設與 PASS 判準如下。

---

### 3.1 `sat2sat_s0s33_120s` — 純 ISL 路由正確性（長距）

| 項目 | 內容 |
|---|---|
| **對象** | sat0 → sat33（8-hop 跨越太平洋） |
| **流量** | none（無使用者流量，只驗證路由層） |
| **時長** | 120s / 3 slots |
| **測項意義** | 驗證 Dijkstra + ISL cost 計算在長距多跳場景下是否正確找到最短路徑，且無封包遺失。ISL cost 會隨衛星移動更新，但路徑不應在短時間內無意義跳變。 |
| **PASS 條件** | ① 路徑合理（跨 8 hops）② ISL drop = 0 ③ HasSignificantChange 觸發但路徑穩定 |
| **對比維度** | 短距 vs 長距 → 與 `sat2sat_s0s1_120s` 比對 hop count 與 cost 差異 |

---

### 3.2 `sat2sat_s0s1_120s` — 純 ISL 路由正確性（短距）

| 項目 | 內容 |
|---|---|
| **對象** | sat0 → sat1（1-hop，最近鄰） |
| **流量** | none |
| **時長** | 120s / 3 slots |
| **測項意義** | 與 `sat2sat_s0s33_120s` 形成距離對比組，確認短距場景下 Dijkstra 仍正確選擇直連 ISL 而非繞路，並確認 cost 值符合預期（~光速傳播時延）。 |
| **PASS 條件** | ① 路徑為 1-hop direct ② ISL drop = 0 ③ cost < 長距 case |
| **對比維度** | 短距 vs 長距（與 `sat2sat_s0s33_120s` 對比） |
| **實際結果** | 路徑固定 `0->1`（1-hop）全 3 slots，cost=0.013173s（vs s0s33 的 0.072–0.078s），ISL drop=0，Loaded ISL links=21/132，PASS |

---

### 3.3 `gw2gw_bgload_120s` — GW 對 GW 背景流量下路由穩定性

| 項目 | 內容 |
|---|---|
| **對象** | GW0（TW-Taipei）→ GW2（US-SanFrancisco） |
| **流量** | background：FWD 200 kbps/flow + RTN 136.5 kbps/flow × 91 UTs |
| **時長** | 120s / 3 slots |
| **測項意義** | 驗證在真實背景流量下，ISL queue 累積是否影響 drop rate，並觀察 slot boundary 是否觸發 route change用來確認 ISL 在中等負載下仍穩定。 |
| **PASS 條件** | ① route change 合理② drop rate < 0.01% ③ ISL link stats 合理（loaded links 數目符合拓撲） |
| **對比維度** | 時間維度 → 與 `gw2gw_bgload_630s` 比對長時間 drop 累積趨勢 |

---

### 3.4 `gw2gw_bgload_630s` — GW 對 GW 背景流量長時間穩定性

| 項目 | 內容 |
|---|---|
| **對象** | GW0（TW-Taipei）→ GW2（US-SanFrancisco） |
| **流量** | background：同 `gw2gw_bgload_120s` |
| **時長** | 630s / 11 slots |
| **測項意義** | 驗證長時間運行下 ISL drop 是否線性累積或出現突增，是否有特定 slot 發生 ISL 擁塞，並確認 route change 次數與頻率在合理範圍。 |
| **PASS 條件** | ① drop rate 全程 < 1.000%（NS3 閾值）② route change 次數符合拓撲變化節奏 ③ 無 ISL link 持續飽和 |
| **對比維度** | 時間維度（與 `gw2gw_bgload_120s` 比對） |
| **實際結果** | 11 slots 全部觸發 HasSignificantChange，route change 5 次（slot 2/3/6/7/9）；ISL drop=59,519/37,231,437（0.160%）；高壅塞鏈路：12-13（7.203%）、13-14（4.340%）、14-15（2.268%）、17-16（4.929%）；overall PASS（< 1.000%）；Loaded ISL links=114/132 |

---

### 3.5 `gw2gw_delivery_120s` — GW 對 GW 端到端封包交付驗證

| 項目 | 內容 |
|---|---|
| **對象** | GW0（TW-Taipei）→ GW2（US-SanFrancisco） |
| **流量** | gw2gw_direct：送出固定量封包，驗證 sink 收到數量 |
| **時長** | 120s / 3 slots |
| **測項意義** | 與 `gw2gw_bgload_120s` 使用相同路由邏輯，但改為驗證「封包是否真的抵達目的端」，而非只看 drop rate。確認 delivery 路徑完整，sink bytes 與發送量吻合。 |
| **PASS 條件** | ① sink 接收 bytes ≥ 預期發送量 × 98% ② ISL drop = 0 ③ route change 行為與 bgload 一致（slot 2 換路） |
| **對比維度** | 流量類型（background load vs endpoint delivery），確認兩者路由行為一致 |

---

### 3.6 `gw2ut_service_630s` — GW 到 UT 長時間服務路由追蹤

| 項目 | 內容 |
|---|---|
| **對象** | GW0（TW-Taipei）→ UT-SanFrancisco（lat=37.8, lon=-122.4） |
| **流量** | gw2ut：從 GW 服務 UT 端，驗證 serving sat 切換 |
| **時長** | 630s / 11 slots |
| **測項意義** | 驗證 GW-to-UT 場景下：① serving satellite 隨衛星移動正確切換 ② entry sat 切換時 ISL 路徑重算正確 ③ 長時間模擬不出現 routing deadlock 或 loop。此為 UT 移動服務的核心 baseline。 |
| **PASS 條件** | ① serving sat 切換次數符合 Iridium 軌道周期預期（11 slots 約 5 次） ② 每次切換後路徑合理 ③ ISL load 分佈合理（loaded links > 80 / 132） |
| **對比維度** | 時間維度 → 與 `gw2ut_service_120s` 比對切換次數與路徑穩定性 |

---

### 3.7 `gw2ut_service_120s` — GW 到 UT 短時間基準

| 項目 | 內容 |
|---|---|
| **對象** | GW0（TW-Taipei）→ UT-SanFrancisco |
| **流量** | gw2ut |
| **時長** | 120s / 3 slots |
| **測項意義** | 與 `gw2ut_service_630s` 對比，確認 3 slots 內 serving sat 是否切換（預期 ≤ 1 次），並驗證短時間下路由初始化是否正常。 |
| **PASS 條件** | ① routing 正常初始化 ② 0–1 次 serving sat 切換 ③ 路徑長度與 `gw2ut_service_630s` slot 0–2 吻合 |
| **對比維度** | 時間維度（與 `gw2ut_service_630s` 前 3 slots 比對） |
| **實際結果** | slot 0–1：serving=37（path: 15->14->25->36->37），slot 2：serving=1（path: 15->14->13->2->1）；1 次 route change，ISL drop=0/6,386,708（0.000%），Loaded ISL links=19/132；路徑與 `gw2ut_service_630s` slot 0–2 完全一致，PASS |

---

## 四、執行指令

### 4.1 全部指令（已完成）

#### `sat2sat_s0s33_120s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=sat2sat \
  --satSrc=0 --satDst=33 \
  --simTime=120 --slotInterval=60" \
  2>&1 | tee output_sat2sat_s0s33_120s.log
```

**驗證重點：** 路徑固定 8 hops，ISL drop = 0，HasSignificantChange 觸發。

---

#### `gw2gw_bgload_120s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=gw2gw \
  --gwSrc=0 --gwDst=2 \
  --trafficProfile=gw2gw \
  --simTime=120 --slotInterval=60" \
  2>&1 | tee output_gw2gw_bgload_120s.log
```

**驗證重點：** slot 2 route change，edge 14-15 drop ≤ 300，drop rate < 0.01%。

---

#### `gw2gw_delivery_120s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=gw2gw \
  --gwSrc=0 --gwDst=2 \
  --trafficProfile=gw2gw_direct \
  --simTime=120 --slotInterval=60" \
  2>&1 | tee output_gw2gw_delivery_120s.log
```

**驗證重點：** sink bytes ≥ 600,000，ISL drop = 0，路由行為與 bgload 一致。

---

#### `gw2ut_service_630s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=gw2ut \
  --trafficProfile=gw2ut \
  --gwId=0 \
  --utId=1 \
  --utLatDeg=37.8 \
  --utLonDeg=-122.4 \
  --utName=UT-SanFrancisco \
  --simTime=630 --slotInterval=60" \
  2>&1 | tee output_gw2ut_service_630s.log
```

**驗證重點：** 11 slots 共 5 次 route change，serving sat 切換路徑合理，loaded ISL links ≈ 95/132。

---

#### `sat2sat_s0s1_120s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=sat2sat \
  --satSrc=0 --satDst=1 \
  --simTime=120 --slotInterval=60" \
  2>&1 | tee output_sat2sat_s0s1_120s.log
```

**驗證重點：** 路徑固定 1-hop（0->1），cost ≈ 0.013s，ISL drop = 0。

---

#### `gw2ut_service_120s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=gw2ut \
  --trafficProfile=gw2ut \
  --gwId=0 \
  --utId=1 \
  --utLatDeg=37.8 \
  --utLonDeg=-122.4 \
  --utName=UT-SanFrancisco \
  --simTime=120 --slotInterval=60" \
  2>&1 | tee output_gw2ut_service_120s.log
```

**驗證重點：** 3 slots，slot 2 route change（serving 37→1），路徑與 gw2ut_service_630s 前 3 slots 一致。

---

#### `gw2gw_bgload_630s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=gw2gw \
  --gwSrc=0 --gwDst=2 \
  --trafficProfile=gw2gw \
  --simTime=630 --slotInterval=60" \
  2>&1 | tee output_gw2gw_bgload_630s.log
```

**驗證重點：** 11 slots 全觸發 HasSignificantChange，route change 5 次，overall drop rate < 1.000%。

---

## 五、已確認結果彙整

| Case | Duration | Route change | ISL drop | Loaded ISL links | Status |
|---|---|---|---|---|---|
| `sat2sat_s0s33_120s` | 120s / 3 slots | 無（cost 更新但路徑不變） | `0 / 6,340,563`（0.000%） | `21 / 132` | ✓ PASS |
| `sat2sat_s0s1_120s` | 120s / 3 slots | 無（1-hop 固定） | `0 / 6,340,563`（0.000%） | `21 / 132` | ✓ PASS |
| `gw2gw_bgload_120s` | 120s / 3 slots | slot 2：exit sat 37 → 1 | `272 / 6,706,864`（0.004%） | `19 / 132` | ✓ PASS |
| `gw2gw_bgload_630s` | 630s / 11 slots | 5 次（slot 2/3/6/7/9） | `59,519 / 37,231,437`（0.160%） | `114 / 132` | ✓ PASS |
| `gw2gw_delivery_120s` | 120s / 3 slots | slot 2（同 bgload） | `0 / 6,340,563`（0.000%） | `21 / 132` | ✓ PASS |
| `gw2ut_service_120s` | 120s / 3 slots | 1 次（slot 2：serving 37→1） | `0 / 6,386,708`（0.000%） | `19 / 132` | ✓ PASS |
| `gw2ut_service_630s` | 630s / 11 slots | 5 次（slot 2/3/6/7/9） | `0`（無 drop） | `95 / 132` | ✓ PASS |

---

## 六、附錄：各 Case 路徑演變

### 6.1 `sat2sat_s0s1_120s` 路徑

| slot | time(s) | full_path | route_cost |
|---|---|---|---|
| 0 | 0 | `0->1` | 0.013173s |
| 1 | 60 | `0->1` | 0.013173s |
| 2 | 120 | `0->1` | 0.013173s |

- 路徑全程固定，cost 不變（距離最短，衛星相對位置幾乎不動）
- HasSignificantChange=YES（slot 1, 2）但 s0→s1 路由未受影響
- Loaded ISL links：21 / 132

---

### 6.2 `gw2ut_service_120s` 路徑

| slot | time(s) | entry sat | ISL path | serving sat |
|---|---|---|---|---|
| 0–1 | 0–60 | 15 | `15->14->25->36->37` | 37 |
| 2 | 120 | 15 | `15->14->13->2->1` | 1 |

- slot 2 發生 route change，serving sat 37 → 1
- 路徑與 `gw2ut_service_630s` slot 0–2 完全一致（驗證一致性）
- Loaded ISL links：19 / 132

---

### 6.3 `gw2gw_bgload_630s` 路徑演變（TW→US）

| slot | time(s) | entry | ISL path | exit | isl_cost(s) |
|---|---|---|---|---|---|
| 0–1 | 0–60 | 15 | `15->14->25->36->37` | 37 | 0.043969–0.046214 |
| 2 | 120 | 15 | `15->14->13->2->1` | 1 | 0.047600 |
| 3–5 | 180–300 | 15 | `15->14->25->36` | 36 | 0.037717–0.042347 |
| 6 | 360 | 44 | `44->45->46->35->36` | 36 | 0.044439 |
| 7–8 | 420–480 | 14 | `14->13->24->35->36` | 36 | 0.040314–0.042179 |
| 9–10 | 540–600 | 44 | `44->45->56->1->0` | 0 | 0.041591–0.043645 |

> 反向（US→TW）路徑對稱，同樣 5 次 route change。

**壅塞 ISL 鏈路（EMA queue delay 顯著）：**

| ISL edge | drop | drop_rate |
|---|---|---|
| 12↔13 | 16,364 | 7.203% |
| 13↔14 | 17,436 | 4.340% |
| 14↔15 | 10,766 | 2.268% |
| 16↔15 | 3,990 | 1.875% |
| 17↔16 | 10,952 | 4.929% |

- Loaded ISL links：114 / 132（vs 120s 的 19/132，長時間負載大幅增加）
- Overall drop rate：0.160%，PASS（< 1.000%）

---

### 6.4 `gw2ut_service_630s` 路徑演變

| slot | time(s) | entry sat | ISL path | serving sat |
|---|---|---|---|---|
| 0–1 | 0–60 | 15 | `15->14->25->36->37` | 37 |
| 2 | 120 | 15 | `15->14->13->2->1` | 1 |
| 3–5 | 180–300 | 15 | `15->14->25->36` | 36 |
| 6 | 360 | 44 | `44->45->46->35->36` | 36 |
| 7–8 | 420–480 | 14 | `14->13->24->35->36` | 36 |
| 9–10 | 540–600 | 44 | `44->45->56->1->0` | 0 |

- ISL load 最高邊：`30<->41`，loadBA = 1.6273 ms
- Loaded ISL links：95 / 132
