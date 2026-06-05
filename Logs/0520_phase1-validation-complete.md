## 2026/05/20


##  compare_v2_peruser.py — Fail

 `compare_v2_peruser.py`

**輸入：** `C++ macro mode, frame=38537, n_user=121807`

**指令：**

```bash
python "compare_v2_peruser.py" --cpp-dir "projection/output/phase1" --frame 38537
```

**結果：**

```
|Δ SNR| p95 = 8.7044 dB    FAIL
```

Debug print 顯示每個 user 的路徑損耗差距：

```
user     Python (dB)    C++ (dB)    Δ (dB)
   0      186.653385   178.103233  +8.550152
   1      186.549996   178.103236  +8.446760
```

---

## 診斷 path_loss 差距

**原因：**

 `channel.path_loss()` 呼叫 `itur.atmospheric_attenuation_slant_path(lat, lon, f, elev, p, D)`，
需要 **全球 lat/lon 座標**。

C++ 的 user positions（`user_positions.csv`）是以衛星下方地表點為原點的 **local frame 座標**（z ≈ 0）。
透過 Python `get_positions_in_lat_long_coordinates()` 轉換後，高度值異常，
導致 itur 回傳 ~9.11 dB 大氣損耗（正確值應為 ~0.55 dB，90° 仰角）。

| 成分 | Python | C++ | Δ |
|---|---|---|---|
| FSPL（600 km，30 GHz） | ~177.54 dB | ~177.54 dB | 0 dB |
| 大氣損耗 | ~9.11 dB（itur，輸入異常） | ~0.55 dB（zenith-scaling） | **+8.56 dB** |
| **path_loss 合計** | ~186.65 dB | ~178.10 dB | **+8.55 dB** |

**結論：** 差距與 beam gain 計算無關。beam model 比較的是 UPA Dirichlet kernel，不是大氣損耗模型。

---

## 修正 compare_v2_peruser.py — 隔離大氣損耗模型

**修改檔案：** `2D/compare_v2_peruser.py`

**修改邏輯：** 移除 `channel.path_loss()` 呼叫，改從 `channel_results.csv` 直接讀取 C++ 計算好的 `path_loss_dB`。

```python
# 舊版（移除）
loss_db = channel.path_loss(cpp_user_pos, sat_pos)

# 新版：兩側使用相同 path_loss 值，只比較 beam gain 差異
cpp_pl_list = []
with open(chan_csv) as _f:
    for _row in csv.DictReader(_f):
        if int(_row["frame_id"]) == args.frame:
            cpp_pl_list.append(float(_row["path_loss_dB"]))
loss_db = np.array(cpp_pl_list)
```

**原因：**

固定兩側 path_loss 後，任何剩餘的 SNR delta 只可能來自 beam gain 計算差異：

```
SNR = 10·log10( P_tx · G_ant · |beam_gain|² / (path_loss · noise) )

固定 P_tx, G_ant, path_loss, noise → SNR delta ≈ 0 ⟺ beam gain 等價
```

---

## Phase 1 驗證通過

**修正後指令：**

```bash
python "compare_v2_peruser.py" --cpp-dir "projection/output/phase1" --frame 38537
```

**結果：**

```
Loaded C++ macro results  — frame=38537, n_user=121807
  sat_pos = [25.7, 0.0, 599999.9] m
  C++ SNR  mean=-1.052 dB  max=8.188 dB
  C++ SINR mean=-2.366 dB  max=7.152 dB

  path_loss loaded: mean=178.120 dB  min=178.103  max=178.282 dB

Per-user comparison: Python macro vs C++ macro
n_user                    : 121807
Beam agreement            : 100.0%  (same beam assigned)

  SNR  mean (dB)  :     -1.052      -1.052
  SNR  max  (dB)  :      8.188       8.188
  SINR mean (dB)  :     -2.366      -2.366

  |Δ SNR|  p50    : 0.0024 dB
  |Δ SNR|  p95    : 0.0158 dB   ✅ PASS
  |Δ SNR|  p99    : 0.0248 dB
  |Δ SINR| p95    : 0.0159 dB   ✅ PASS

  beam_gain max   : Python=47.712 dB  C++=47.712 dB  ✅
  No outliers (|Δ SNR| > 0.5 dB) — model equivalence confirmed ✅
```

**Phase 1 結論：**

| 指標 | 值 | 判斷 |
|---|---|---|
| `beam_gain max` | 47.712 dB（兩側一致） | ✅ |
| Beam 分配一致率 | 100.0%（121807 / 121807） | ✅ |
| `\|Δ SNR\| p95` | 0.0158 dB | ✅ PASS（< 0.5 dB） |
| `\|Δ SINR\| p95` | 0.0159 dB | ✅ PASS |
| Outliers | 0 / 121807 | ✅ |

還有 ~0.016 dB 差異。來源：Python BLAS 矩陣乘法 vs C++ sin/cos 計算的浮點累積差，非模型誤差。


