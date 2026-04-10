# Baseline 驗�?總表

?��?件整??`Topology & ISL Routing/Outputs/Baseline/` ?�??baseline 測試?�目�? 
對�?程�?碼�?`scratch/test-iridium_baseline.cc`

驗�?維度�?*模�?（mode）�?流�?（traffic）�??��?（simTime）�?距離（pair�?*

---

## 一?�測試矩??
### 1.1 ?��?維度（短 vs ?��?

| mode / traffic | 120s | 630s |
|---|---|---|
| sat2sat / none | ??`sat2sat_s0s33_120s.md` | ??|
| gw2gw / background | ??`gw2gw_bgload_120s.md` | ??`gw2gw_bgload_630s.md` |
| gw2gw / delivery | ??`gw2gw_delivery_120s.md` | ??|
| gw2ut / service | ??`gw2ut_service_120s.md` | ??`gw2ut_service_630s.md` |

### 1.2 距離維度（�? vs ?��?

| mode / traffic | ?��? | ?��? |
|---|---|---|
| sat2sat / none | ??`sat2sat_s0s1_120s.md`�?-hop�?| ??`sat2sat_s0s33_120s.md`�?-hop�?|
| gw2gw / delivery | ??| ??`gw2gw_delivery_120s.md`（TW?�US�?|
| gw2ut / service | ??| ??`gw2ut_service_630s.md`（TW?�UT-SF�?|

---

## 二、Traffic Profile 說�?

`test-iridium_baseline.cc` ?��?�?5 �?traffic profile，以 `--trafficProfile=<??` ?�入??

### 2.0 `mode` �� `trafficProfile` ��e��

�����ӰѼƤ��}�ݡG

- `mode`�G�M�w�n���ҭ��@����ѳ���
- `trafficProfile`�G�M�w�ӳ����U�O�_�[�y�q�A�H�Υ[���@�جy�q

�]���A`none` �M `sat2sat` �ä��Ĭ�A�ӬO�N����P���סG

- `mode=sat2sat, trafficProfile=none`�G�� ISL baseline�A�u�� route path�Bhop count�Bcost
- `mode=sat2sat, trafficProfile=sat2sat`�G�b sat2sat ��ѳ����W�[�I���y�q�A�[�� ISL queue �P load-aware reroute

�P�˦a�A��L profile �]�u�O�u�[�b mode �W���y�q����v�A���O�t�@�� mode�C

| mode \\ trafficProfile | `none` | `sat2sat` | `gw2gw` | `gw2gw_direct` | `gw2ut` |
|---|---|---|---|---|---|
| `sat2sat` | �¸�� baseline�A�嫬 | ISL �I���t�������A�嫬 | �i�f�t�A���D�嫬 | �i�f�t�A���D�嫬 | �i�f�t�A���D�嫬 |
| `gw2gw` | �¸�� baseline����Ӳ� | �i�f�t�A���D�嫬 | GW �� GW �I���t���A�嫬 | GW_user �� GW_user �ݨ�����ҡA�嫬 | �i�f�t�A���D�嫬 |
| `gw2ut` | �¸�� baseline����Ӳ� | �i�f�t�A���D�嫬 | �i�f�t�A���D�嫬 | �i�f�t�A���D�嫬 | GW �� UT �A�Ȭy�q�A�嫬 |

�\Ū�覡�G

- ���� `mode`�A�M�w�A�n�ݪ���ѳ���
- �A�� `trafficProfile`�A�M�w�o�ӳ����O�b�L�y�q baseline �U���ҡA�٬O�b�S�w�y�q����U����
- �����ثe������ baseline case �H�嫬�զX���D

### 2.1 五種 Profile 彙整

| profile ??| 流�?裝�? | 觀察�?�?| ?��? mode |
|---|---|---|---|
| `none` | ??| routing path, cost | `sat2sat` |
| `gw2ut` | GW?�UT CBR（正常�?度�? | serving sat ?��?, ISL path | `gw2ut` |
| `sat2sat` | GW?�all UT（�?密度，interval=30ms�?| ISL queue, load-aware reroute | `sat2sat` |
| `gw2gw` | ?�端 GW?�UT CBR（中密度�?| ISL drop, route change, loaded links | `gw2gw` |
| `gw2gw_direct` | GW_user ??GW_user UDP OnOff | sink received bytes | `gw2gw` |

---

### 2.2 Background / Delivery / Service 差異

#### 流�?裝�?比�?

| ?�目 | `gw2gw`（background�?| `gw2gw_direct`（delivery�?| `gw2ut`（service�?|
|---|---|---|---|
| **?�送端** | GW_src ??UT �?+ GW_dst ??UT 群�??��?�?| GW_src ??user node | GW ??user node |
| **?�收�?* | ?�自??UT 群�??��?�?| GW_dst ??user node | UT ??user node |
| **helper** | `SatTrafficHelper`（�??��??��? | ns3 `OnOffHelper` + `PacketSinkHelper` | `SatTrafficHelper` |
| **?��?** | CBR over satellite link | UDP over ISL ??GW | CBR over satellite link |
| **計�?** | ISL drop count + loaded links | `PacketSink::GetTotalRx()` bytes | serving sat ?��?路�? |

#### ?��?差異

```
background  ?? ?��? ISL queue ?��?載�?              ?��??��???queue delay ??UpdateLoadCosts ??HasSignificantChange
              不在乎�??�抵?��???
delivery    ?? ?�確認�??��??�到?�目?�端??              ?��??��?�?GW unicast forward rule ?�否�?��
              received=0 �?�� routing rule 缺失

service     ?? ?��?�?UT ?��??��?�?serving sat 如�??��???              ?��??��?�?gw2ut ISL 路�??��??�移?�正確�?�?              不�???bytes，�?路�?變�?
```

#### PASS / FAIL ?��?

| | background | delivery | service |
|---|---|---|---|
| **?��?�?* | ISL drop rate?�loaded link ?�、route change | sink received bytes | entry sat?�serving sat?�ISL path per slot |
| **PASS 條件** | drop rate < 0.01%，�? route change | received > 0 bytes | serving sat ?��?次數?��?，路徑�???|
| **FAIL �?��** | ISL 壅�??�cost 計�??�常 | GW forward rule 缺失 | routing loop?�serving sat 不�???|

---

## 三、測?��?�?
?�測試�??��??��??��?證�?設�? PASS ?��?如�???
---

### 3.1 `sat2sat_s0s33_120s` ??�?ISL 路由�?��?��??��?�?
| ?�目 | ?�容 |
|---|---|
| **對象** | sat0 ??sat33�?-hop 跨�?太平洋�? |
| **流�?** | none（無使用?��??��??��?證路?�層�?|
| **?�長** | 120s / 3 slots |
| **測�??�義** | 驗�? Dijkstra + ISL cost 計�??�長距�?跳場?��??�否�?��?�到?�?�路徑�?且無封�??�失?�ISL cost ?�隨衛�?移�??�新，�?路�?不�??�短?��??�無?�義跳�???|
| **PASS 條件** | ??路�??��?（跨 8 hops）② ISL drop = 0 ??HasSignificantChange 觸發但路徑穩�?|
| **對�?維度** | ?��? vs ?��? ????`sat2sat_s0s1_120s` 比�? hop count ??cost 差異 |

---

### 3.2 `sat2sat_s0s1_120s` ??�?ISL 路由�?��?��??��?�?
| ?�目 | ?�容 |
|---|---|
| **對象** | sat0 ??sat1�?-hop，�?近鄰�?|
| **流�?** | none |
| **?�長** | 120s / 3 slots |
| **測�??�義** | ??`sat2sat_s0s33_120s` 形�?距離對�?組�?確�??��??�景�?Dijkstra 仍正確選?�直??ISL ?��?繞路，並確�? cost ?�符?��??��?~?�速傳?��?延�???|
| **PASS 條件** | ??路�???1-hop direct ??ISL drop = 0 ??cost < ?��? case |
| **對�?維度** | ?��? vs ?��?（�? `sat2sat_s0s33_120s` 對�?�?|
| **實�?結�?** | 路�??��? `0->1`�?-hop）全 3 slots，cost=0.013173s（vs s0s33 ??0.072??.078s）�?ISL drop=0，Loaded ISL links=21/132，PASS |

---

### 3.3 `gw2gw_bgload_120s` ??GW �?GW ?�景流�?下路?�穩定�?
| ?�目 | ?�容 |
|---|---|
| **對象** | GW0（TW-Taipei）�? GW2（US-SanFrancisco�?|
| **流�?** | background：FWD 200 kbps/flow + RTN 136.5 kbps/flow ? 91 UTs |
| **?�長** | 120s / 3 slots |
| **測�??�義** | 驗�??��?實�??��??��?，ISL queue 累�??�否影響 drop rate，並觀�?slot boundary ?�否觸發 route change?�此?�「ISL queue 壓�??�測試�??�小�??��??��?確�? ISL ?�中等�?載�?仍�?失穩??|
| **PASS 條件** | ??route change 行為?��?（slot 2 ?��? exit sat）② drop rate < 0.01% ??ISL link stats ?��?（loaded links ?�目符�??�撲�?|
| **對�?維度** | ?��?維度 ????`gw2gw_bgload_630s` 比�??��???drop 累�?趨勢 |

---

### 3.4 `gw2gw_bgload_630s` ??GW �?GW ?�景流�??��??�穩定�?
| ?�目 | ?�容 |
|---|---|
| **對象** | GW0（TW-Taipei）�? GW2（US-SanFrancisco�?|
| **流�?** | background：�? `gw2gw_bgload_120s` |
| **?�長** | 630s / 11 slots |
| **測�??�義** | 驗�??��??��?行�? ISL drop ?�否線性累積�??�現突�?，是?��??��? slot ?��? ISL ?��?，並確�? route change 次數?�頻?�在?��?範�???|
| **PASS 條件** | ??drop rate ?��? < 1.000%（NS3 ?�值�???route change 次數符�??�撲變�?節�?????ISL link ?��?飽�? |
| **對�?維度** | ?��?維度（�? `gw2gw_bgload_120s` 比�?�?|
| **實�?結�?** | 11 slots ?�部觸發 HasSignificantChange，route change 5 次�?slot 2/3/6/7/9）�?ISL drop=59,519/37,231,437�?.160%）�?高�?塞�?路�?12-13�?.203%）�?3-14�?.340%）�?4-15�?.268%）�?7-16�?.929%）�?overall PASS�? 1.000%）�?Loaded ISL links=114/132 |

---

### 3.5 `gw2gw_delivery_120s` ??GW �?GW 端到端�??�交付�?�?
| ?�目 | ?�容 |
|---|---|
| **對象** | GW0（TW-Taipei）�? GW2（US-SanFrancisco�?|
| **流�?** | gw2gw_direct：送出?��??��??��?驗�? sink ?�到?��? |
| **?�長** | 120s / 3 slots |
| **測�??�義** | ??`gw2gw_bgload_120s` 使用?��?路由?�輯，�??�為驗�??��??�是?��??�抵?�目?�端?��??��??��? drop rate?�確�?delivery 路�?完整，sink bytes ?�發?��??��???|
| **PASS 條件** | ??sink ?�收 bytes ???��??�送�? ? 98% ??ISL drop = 0 ??route change 行為??bgload 一?��?slot 2 ?�路�?|
| **對�?維度** | 流�?類�?（background load vs endpoint delivery）�?確�??�者路?��??��???|

---

### 3.6 `gw2ut_service_630s` ??GW ??UT ?��??��??�路?�追�?
| ?�目 | ?�容 |
|---|---|
| **對象** | GW0（TW-Taipei）�? UT-SanFrancisco（lat=37.8, lon=-122.4�?|
| **流�?** | gw2ut：�? GW ?��? UT 端�?驗�? serving sat ?��? |
| **?�長** | 630s / 11 slots |
| **測�??�義** | 驗�? GW-to-UT ?�景下�???serving satellite ?��??�移?�正確�?????entry sat ?��???ISL 路�??��?�?�� ???��??�模?��??�現 routing deadlock ??loop?�此??UT 移�??��??�核�?baseline??|
| **PASS 條件** | ??serving sat ?��?次數符�? Iridium 軌�??��??��?�?1 slots �?5 次�? ??每次?��?後路徑�?????ISL load ?��??��?（loaded links > 80 / 132�?|
| **對�?維度** | ?��?維度 ????`gw2ut_service_120s` 比�??��?次數?�路徑穩定�?|

---

### 3.7 `gw2ut_service_120s` ??GW ??UT ?��??�基�?
| ?�目 | ?�容 |
|---|---|
| **對象** | GW0（TW-Taipei）�? UT-SanFrancisco |
| **流�?** | gw2ut |
| **?�長** | 120s / 3 slots |
| **測�??�義** | ??`gw2ut_service_630s` 對�?，確�?3 slots ??serving sat ?�否?��?（�?????1 次�?，並驗�??��??��?路由?��??�是?�正常�?|
| **PASS 條件** | ??routing �?��?��?????0?? �?serving sat ?��? ??路�??�度??`gw2ut_service_630s` slot 0?? ?��? |
| **對�?維度** | ?��?維度（�? `gw2ut_service_630s` ??3 slots 比�?�?|
| **實�?結�?** | slot 0??：serving=37（path: 15->14->25->36->37）�?slot 2：serving=1（path: 15->14->13->2->1）�?1 �?route change，ISL drop=0/6,386,708�?.000%）�?Loaded ISL links=19/132；路徑�? `gw2ut_service_630s` slot 0?? 完全一?��?PASS |

---

## ?�、執行�?�?
### 4.1 ?�部?�令（已完�?�?
#### `sat2sat_s0s33_120s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=sat2sat \
  --satSrc=0 --satDst=33 \
  --simTime=120 --slotInterval=60" \
  2>&1 | tee output_sat2sat_s0s33_120s.log
```

**驗�??��?�?* 路�??��? 8 hops，ISL drop = 0，HasSignificantChange 觸發??
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

**驗�??��?�?* slot 2 route change，edge 14-15 drop ??300，drop rate < 0.01%??
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

**驗�??��?�?* sink bytes ??600,000，ISL drop = 0，路?��??��? bgload 一?��?
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

**驗�??��?�?* 11 slots ??5 �?route change，serving sat ?��?路�??��?，loaded ISL links ??95/132??
---

#### `sat2sat_s0s1_120s`
```bash
./ns3 run "scratch/test-iridium_baseline \
  --mode=sat2sat \
  --satSrc=0 --satDst=1 \
  --simTime=120 --slotInterval=60" \
  2>&1 | tee output_sat2sat_s0s1_120s.log
```

**驗�??��?�?* 路�??��? 1-hop�?->1）�?cost ??0.013s，ISL drop = 0??
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

**驗�??��?�?* 3 slots，slot 2 route change（serving 37??）�?路�???gw2ut_service_630s ??3 slots 一?��?
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

**驗�??��?�?* 11 slots ?�觸??HasSignificantChange，route change 5 次�?overall drop rate < 1.000%??
---

## 五、已確�?結�?彙整

| Case | Duration | Route change | ISL drop | Loaded ISL links | Status |
|---|---|---|---|---|---|
| `sat2sat_s0s33_120s` | 120s / 3 slots | ?��?cost ?�新但路徑�?變�? | `0 / 6,340,563`�?.000%�?| `21 / 132` | ??PASS |
| `sat2sat_s0s1_120s` | 120s / 3 slots | ?��?1-hop ?��?�?| `0 / 6,340,563`�?.000%�?| `21 / 132` | ??PASS |
| `gw2gw_bgload_120s` | 120s / 3 slots | slot 2：exit sat 37 ??1 | `272 / 6,706,864`�?.004%�?| `19 / 132` | ??PASS |
| `gw2gw_bgload_630s` | 630s / 11 slots | 5 次�?slot 2/3/6/7/9�?| `59,519 / 37,231,437`�?.160%�?| `114 / 132` | ??PASS |
| `gw2gw_delivery_120s` | 120s / 3 slots | slot 2（�? bgload�?| `0 / 6,340,563`�?.000%�?| `21 / 132` | ??PASS |
| `gw2ut_service_120s` | 120s / 3 slots | 1 次�?slot 2：serving 37??�?| `0 / 6,386,708`�?.000%�?| `19 / 132` | ??PASS |
| `gw2ut_service_630s` | 630s / 11 slots | 5 次�?slot 2/3/6/7/9�?| `0`（無 drop�?| `95 / 132` | ??PASS |

---

## ?�、�??��???Case 路�?演�?

### 6.1 `sat2sat_s0s1_120s` 路�?

| slot | time(s) | full_path | route_cost |
|---|---|---|---|
| 0 | 0 | `0->1` | 0.013173s |
| 1 | 60 | `0->1` | 0.013173s |
| 2 | 120 | `0->1` | 0.013173s |

- 路�??��??��?，cost 不�?（�??��??��?衛�??��?位置幾�?不�?�?- HasSignificantChange=YES（slot 1, 2）�? s0?�s1 路由?��?影響
- Loaded ISL links�?1 / 132

---

### 6.2 `gw2ut_service_120s` 路�?

| slot | time(s) | entry sat | ISL path | serving sat |
|---|---|---|---|---|
| 0?? | 0??0 | 15 | `15->14->25->36->37` | 37 |
| 2 | 120 | 15 | `15->14->13->2->1` | 1 |

- slot 2 ?��? route change，serving sat 37 ??1
- 路�???`gw2ut_service_630s` slot 0?? 完全一?��?驗�?一?�性�?
- Loaded ISL links�?9 / 132

---

### 6.3 `gw2gw_bgload_630s` 路�?演�?（TW?�US�?
| slot | time(s) | entry | ISL path | exit | isl_cost(s) |
|---|---|---|---|---|---|
| 0?? | 0??0 | 15 | `15->14->25->36->37` | 37 | 0.043969??.046214 |
| 2 | 120 | 15 | `15->14->13->2->1` | 1 | 0.047600 |
| 3?? | 180??00 | 15 | `15->14->25->36` | 36 | 0.037717??.042347 |
| 6 | 360 | 44 | `44->45->46->35->36` | 36 | 0.044439 |
| 7?? | 420??80 | 14 | `14->13->24->35->36` | 36 | 0.040314??.042179 |
| 9??0 | 540??00 | 44 | `44->45->56->1->0` | 0 | 0.041591??.043645 |

> ?��?（US?�TW）路徑�?稱�??�樣 5 �?route change??
**壅�? ISL ?�路（EMA queue delay 顯�?）�?**

| ISL edge | drop | drop_rate |
|---|---|---|
| 12??3 | 16,364 | 7.203% |
| 13??4 | 17,436 | 4.340% |
| 14??5 | 10,766 | 2.268% |
| 16??5 | 3,990 | 1.875% |
| 17??6 | 10,952 | 4.929% |

- Loaded ISL links�?14 / 132（vs 120s ??19/132，長?��?負�?大�?增�?�?- Overall drop rate�?.160%，PASS�? 1.000%�?
---

### 6.4 `gw2ut_service_630s` 路�?演�?

| slot | time(s) | entry sat | ISL path | serving sat |
|---|---|---|---|---|
| 0?? | 0??0 | 15 | `15->14->25->36->37` | 37 |
| 2 | 120 | 15 | `15->14->13->2->1` | 1 |
| 3?? | 180??00 | 15 | `15->14->25->36` | 36 |
| 6 | 360 | 44 | `44->45->46->35->36` | 36 |
| 7?? | 420??80 | 14 | `14->13->24->35->36` | 36 |
| 9??0 | 540??00 | 44 | `44->45->56->1->0` | 0 |

- ISL load ?�高�?：`30<->41`，loadBA = 1.6273 ms
- Loaded ISL links�?5 / 132
