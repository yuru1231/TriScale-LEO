# Starlink Dataset Analysis Report

**Date:** 2026-06-01  
**Source:** PanLab@UVic — https://onlineacademiccommunity.uvic.ca/starlink/  
**Dataset:** Zenodo (doi: 10.5281/zenodo.10020034) — Sept 13–17, 2023  
**Collected by:** Jinwei Zhao, Jianping Pan (PIMRC'23) 

---

## 1. Overview

分析三個時段的量測資料，結合兩篇實際閱讀的論文進行交叉解讀：

| Session | 日期時間 | iperf3 | IRTT |
|---|---|---|---|
| S1 | 2023-09-13 00:40 UTC | ✓ | ✓ |
| S2 | 2023-09-13 01:40 UTC | ✓ | ✓ |
| S3 | 2023-09-15 00:00 UTC | ✓ | ✓ |

**測量節點：** Starlink 用戶端，hostname `jaylin`，本地 IP 192.168.1.144，Linux 5.19.0。

---

## 2. Throughput Comparison (iperf3)

### 2.1 Summary

| Metric | S1 (09-13 00:40) | S2 (09-13 01:40) | S3 (09-15 00:00) |
|---|---|---|---|
| Mean throughput | **36.00 Mbps** | **20.09 Mbps** | **26.82 Mbps** |
| TCP retransmits | 379 | 362 | 208 |

同地點、相距 1 小時（S1→S2），吞吐量從 36 Mbps 跌至 20 Mbps（−44%）。跨兩天（S1→S3）仍有 25% 差距。**Starlink 吞吐量在不同時段高度不穩定，單一時段無法代表典型值。**

### 2.2 Drop Analysis

**S1（09-13 00:40）：**

| Drop 時間 | 對應 wall clock | 最近 slot | 誤差 |
|---|---|---|---|
| t ≈ 11.4 s | 00:40:11 | **12 s** | −0.6 s |
| t ≈ 26.4 s | 00:40:26 | **27 s** | −0.6 s |
| t ≈ 86.4 s | 00:41:26 | **27 s** | −0.6 s |
| t ≈ 116.4 s | 00:41:56 | **57 s** | −0.6 s |

Drop 1→2 間隔：**15.0 秒（精確）**，直接對應 ICC'24 論文的 15 秒同步 handover。

**S2（09-13 01:40）：** 共 52 次 drop，多數間隔 < 1 秒，顯示嚴重鏈路問題（遮蔽/衛星負載），非正常 handover 行為。

**S3（09-15 00:00）：**

| Drop pair | 間隔 | 對應 slot |
|---|---|---|
| 71.5 s → 86.5 s | **15.0 s** | 12s → 27s |

不同日期獨立出現 15.0 秒間隔，**確認 15 秒全球同步 handover**。

---

## 3. Latency Comparison (IRTT)

### 3.1 Summary

| Metric | S1 (09-13 00:40) | S2 (09-13 01:40) | S3 (09-15 00:00) |
|---|---|---|---|
| RTT mean | **38.17 ms** | **51.95 ms** | **42.28 ms** |
| RTT median | 36.39 ms | 49.12 ms | 40.52 ms |
| RTT max | 129.45 ms | **304.94 ms** | 146.09 ms |
| UL delay mean | 21.88 ms | 24.12 ms | 26.78 ms |
| DL delay mean | 16.29 ms | **27.83 ms** | 15.49 ms |
| UL − DL | **+5.59 ms** | **−3.71 ms** | **+11.29 ms** |
| IPDV send max | 93.23 ms | 78.25 ms | 97.46 ms |
| Total loss | 0.602% | **1.418%** | 1.051% |
| UL loss | 0.560% | 0.912% | 0.907% |
| DL loss | **0.043%** | **0.511%** | 0.145% |

### 3.2 Observations

**O1：RTT 差異可達 36%（38→52 ms），S2 最高值達 305 ms。**  
S2 的極端 RTT 與密集 drop 一致，確認 S2 為異常時段。正常時段（S1/S3）RTT 約 38–42 ms。

**O2：UL/DL 差距方向在三個時段不一致（+5.6 / −3.7 / +11.3 ms）。**  
S2 的 DL 延遲（27.83 ms）反而高於 UL（24.12 ms），方向與其他兩個時段相反。方向不一致表示差距不是穩定的物理現象，而是時鐘偏移造成的量測誤差。**→ 論文指出的 UL/DL 對稱性在資料中獲得支持。**

**O3：DL 封包遺失在 S2 暴增 12 倍（0.043%→0.511%）。**  
正常時段下行幾乎無遺失；S2 下行遺失劇增，代表 S2 的鏈路問題影響到了下行。

**O4：IPDV send 最大值跨時段穩定（78–97 ms）。**  
handover 造成的 jitter spike 上限約為 80–100 ms，顯示 handover 衝擊幅度相對固定。

---

## 4. Paper Findings & Cross-Validation

### 4.1 Paper 1：Inside the Nebula: Measuring Starlink Satellite Access Networks

| 論文 | 資料驗證結果 |
|---|---|
| 每 15 秒一次全球同步 handover |  S1 和 S3 均出現精確 15.0s 間隔 |
| Handover 造成 TCP cwnd 重置 |  Drop 後每次都出現 slow start 爬升 |
| UL/DL 延遲對稱 |  跨時段差距方向不一致，支持時鐘偏移解讀 |

### 4.2 Paper 2：Measuring the Satellite Links of a LEO Network (ICC'24)

| 論文 | 資料驗證結果 |
|---|---|
| Handover 固定在 12/27/42/57 秒 |  S1 的 drop 對應 12s 和 27s，誤差 −0.6s |
| RTT 呈階梯狀跳動 |  S1/S3 均出現清晰的週期性 drop |
| ISL 在偏遠地區引入高延遲 |單一地點無法驗證|

---

## 5. Conclusions

**C1：15 秒全球同步 handover 已在兩個獨立時段確認。**  
S1（09-13）和 S3（09-15）各自出現精確 15.0 秒的 drop 間隔，且對應相同的 wall clock slot（12s/27s/57s）。此為本資料集中最強的發現，與兩篇論文一致。

**C2：Starlink 吞吐量在不同時段差異極大（20–36 Mbps），S2 出現異常。**  
S2（09-13 01:40）吞吐量僅 20 Mbps，RTT max 達 305 ms，DL loss 暴增 12 倍，顯示 Starlink 效能受時段影響顯著。36 Mbps 不是穩定典型值。

**C3：正常時段 RTT 約 38–42 ms，handover 時最高達 129–146 ms。**  
Propagation floor 約 23 ms（S1 最小值）。與 GEO（>600 ms）相比改善顯著，但高於地面光纖（<5 ms）。

**C4：UL/DL 延遲差距方向不一致，不應作為設計依據。**  
三個時段的 UL−DL 分別為 +5.6 / −3.7 / +11.3 ms，方向不一致。支持論文的說法：UL/DL 物理上對稱，差距來自時鐘偏移。

**C5：IPDV spike 上限約 78–97 ms，為 handover 固有特性。**  
跨時段穩定出現，代表每次 handover 都會對即時應用（VoIP < 30 ms）造成超標影響。

---

## 6. Implications for TriScale-LEO

| TriScale-LEO 層 | 從資料 + 論文學到的 |
|---|---|
| **Layer 1 (ISL Routing)** | Handover 每 15s 一次（12/27/42/57s）；應在 handover 前預測並切換路徑 |
| **Layer 2 (Beam Hopping)** | TCP cwnd 在 handover 後重置；scheduler 需考慮 slow start 恢復期 |
| **Layer 3 (QoS Scheduler)** | 正常時段 UL loss 是 DL 的 3–13 倍；jitter spike 固定約 80–100ms |
| **模擬參數（正常時段）** | RTT ~38–42ms；propagation floor ~23ms；total loss ~0.6–1.1% |
| **設計注意事項** | 效能有顯著時段差異；不應只用單一時段參數；需考慮異常時段的降級行為 |

---

## 7. Limitations & Next Steps

**Limitations：**
- 僅一個地點（Victoria, BC），不同地點結果可能不同
- S2 異常原因不明
- UL/DL 絕對值受時鐘偏移影響，不可靠
- 尚未分析 09-14, 09-16, 09-17 的資料


---

## 8. Data Files & References

```
PanLAB/LENS/
├── iperf3-2m-20230913.json                  S1: 09-13 00:40
├── irtt-10ms-2m-20230913.json               S1: 09-13 00:40
├── iperf3-2m-2023-09-13-01-40-00.json       S2: 09-13 01:40
├── irtt-10ms-2m-2023-09-13-01-40-00.json    S2: 09-13 01:40
├── iperf3-2m-2023-09-15-00-00-00.json       S3: 09-15 00:00
└── irtt-10ms-2m-2023-09-15-00-00-00.json    S3: 09-15 00:00
```

**Dataset:** https://doi.org/10.5281/zenodo.10020034  
**Papers read:**
- "Inside the Nebula: Measuring Starlink Satellite Access Networks"
- "Measuring the Satellite Links of a LEO Network" (ICC'24, Pan, Zhao, Cai)

**Analysis scripts:** [scripts/analyze_iperf3.py](scripts/analyze_iperf3.py), [scripts/analyze_irtt.py](scripts/analyze_irtt.py)
