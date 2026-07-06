# Sat BH 剩餘 demand 計算說明

這份文件說明 `sat-bh-example` 如何輸出剩餘 demand，以及哪些檔案應該用來判斷「實際沒有被服務完」的需求量。

目前有兩種相關輸出：

- `sat-bh-traffic_*.tr`：排程層的 accounting。
- `bh-demand-actual_*.csv`：由實際 FWD 接收 throughput 計算出的剩餘 demand。

如果要看真正的剩餘 demand，請使用 `bh-demand-actual_*.csv`。

## 驗證指令

```bash
./ns3 run "scratch/bh_dynamic/Codes/sat-bh-example --enableObc=1 --enableDynamicBstp=1 --simTime=11 --warmUp=1 --fwdOfferedDemandKbps=1000 --maxHelperSats=1 --satIdStart=498 --outputDir=/tmp/bh-example-full-test"
```

檢查輸出檔：

```bash
find /tmp/bh-example-full-test -maxdepth 2 -type f | sort
```

查看實際剩餘 demand：

```bash
sed -n '1,40p' /tmp/bh-example-full-test/bh-demand-actual_*.csv
```

彙總每個 beam 的平均 offered / served / remaining：

```bash
awk -F, 'NR>1 {off[$4]+=$5; served[$4]+=$7; rem[$4]+=$8; n[$4]++} END {printf "beam,avg_offered_kbps,avg_served_kbps,avg_remaining_kbps\n"; for (b=1;b<=25;b++) printf "%d,%.3f,%.3f,%.3f\n", b, off[b]/n[b], served[b]/n[b], rem[b]/n[b]}' /tmp/bh-example-full-test/bh-demand-actual_*.csv
```

## 實際剩餘 demand 的定義

實際剩餘 demand 是在模擬結束後，根據 UT user 端 `PacketSink` 實際收到的 FWD packets 計算。

也就是說，它不是只看 beam 有沒有被排程，而是用真正收到的 bytes 換算 throughput，再從 offered demand 裡扣掉。

接收端 trace 接在這裡：

```cpp
// scratch/bh_dynamic/Codes/sat-bh-example.cc
sink->TraceConnectWithoutContext(
    "Rx", MakeBoundCallback(&FwdRxTrace, i));
```

`FwdRxTrace()` 會把每個 beam、每個 BHTP period 收到的 bytes 記錄起來：

```cpp
static void
FwdRxTrace(uint32_t utIdx, Ptr<const Packet> pkt, const Address& /*from*/)
{
    const uint32_t beamIdx = g_beamPdr.empty() ? 0 : utIdx % g_beamPdr.size();

    if (!g_beamPdr.empty())
    {
        g_beamPdr[beamIdx].rxBytes += pkt->GetSize();
    }

    const double nowSec = Simulator::Now().GetSeconds();
    if (!g_beamPdr.empty() &&
        g_actualDemandPeriodSec > 0.0 &&
        nowSec >= g_actualDemandWarmUpSec)
    {
        const uint32_t periodIdx =
            static_cast<uint32_t>((nowSec - g_actualDemandWarmUpSec) /
                                  g_actualDemandPeriodSec);
        if (g_beamRxBytesByPeriod.size() <= periodIdx)
        {
            g_beamRxBytesByPeriod.resize(periodIdx + 1,
                                         std::vector<uint64_t>(g_beamPdr.size(), 0));
        }
        g_beamRxBytesByPeriod[periodIdx][beamIdx] += pkt->GetSize();
    }
}
```

模擬結束後會輸出 `bh-demand-actual_*.csv`：

```cpp
actualDemand << "period_idx,period_start_s,period_end_s,beam_id,"
             << "offered_kbps,rx_bytes,served_kbps,"
             << "remaining_kbps,remaining_pct\n";

const double servedKbps =
    static_cast<double>(rxBytes) * 8.0 / 1000.0 /
    g_actualDemandPeriodSec;
const double remainingKbps = std::max(0.0, offeredKbps - servedKbps);
const double remainingPct =
    offeredKbps > 0.0 ? remainingKbps / offeredKbps * 100.0 : 0.0;
```

公式如下：

```text
served_kbps = rx_bytes * 8 / 1000 / period_seconds
remaining_kbps = max(0, offered_kbps - served_kbps)
remaining_pct = remaining_kbps / offered_kbps * 100
```

Starlink FWD offered-demand 模式下：

```text
offered_kbps = fwdOfferedDemandKbps + optional hotspot boost
```

如果同時使用多個 ROI helper satellites，程式會把 offered demand 乘上 helper satellite 數量：

```cpp
offeredKbps *= roiSatFactor;
```

## 排程層剩餘 demand

`sat-bh-traffic_*.tr` 裡也有 `DEMAND,SERVICE_ACCOUNTING` rows，但這是排程層 accounting，不是實際 throughput。

對應 code：

```cpp
// contrib/satellite/helper/sat-bh-helper.cc
const bool served = active.find(bid) != active.end();
const double servedKbps = served ? demandKbps : 0.0;
const double remainingKbps = served ? 0.0 : demandKbps;
```

意思是：

- 如果 beam 在本輪 BHTP cycle 被選進 active set，就視為排程層剩餘 demand 是 `0`。
- 如果 beam 沒有被選進 active set，就視為本輪完全沒服務，剩餘 demand 等於完整 offered demand。

這個輸出適合用來檢查 beam selection 的決策，但不適合拿來代表實際 throughput。

## 輸出欄位解讀

`bh-demand-actual_*.csv` 欄位：

```text
period_idx,period_start_s,period_end_s,beam_id,
offered_kbps,rx_bytes,served_kbps,remaining_kbps,remaining_pct
```

範例：

```text
0,1.000000,1.080000,13,1000.000,93000,9300.000,0.000,0.000
```

代表：

- 第 0 個 period 是 `1.000s` 到 `1.080s`。
- beam 13 的 offered demand 是 `1000 kbps`。
- 該 period 內 UT user `PacketSink` 實際收到 `93000 bytes`。
- 換算後實際 served throughput 是 `9300 kbps`。
- 因為 served throughput 大於 offered demand，所以剩餘 demand 被 clamp 成 `0 kbps`。

另一個範例：

```text
0,1.000000,1.080000,1,1000.000,0,0.000,1000.000,100.000
```

代表 beam 1 在該 period 沒有收到任何 FWD packets，所以完整 `1000 kbps` demand 都沒有被服務完。

## PDR 表格的注意事項

console 最後印出的 PDR 表格只是粗略 summary：

```text
PDR = total_rx_bytes / total_offered_bytes
```

而且 PDR 會被 clamp 到 `100%`。因此如果某些 beam 收到的 bytes 遠大於 offered baseline，整體 PDR 可能顯示 `100%`，但個別 80 ms periods 仍然可能有剩餘 demand。

要看細節，請看：

```text
bh-demand-actual_*.csv
```

## 完整測試結果範例

10 秒量測測試產生：

```text
/tmp/bh-example-full-test/bh-demand-actual_starlink25_20260706_130128.csv
```

該次測試的部分 beam 平均值：

```text
beam,avg_offered_kbps,avg_served_kbps,avg_remaining_kbps
1,1000.000,6462.000,824.000
2,1000.000,6259.200,832.000
3,1000.000,6069.600,840.000
12,1000.000,2402.400,936.000
13,1000.000,6265.200,824.000
25,1000.000,5791.200,840.000
```

這裡 `avg_served_kbps` 可能大於 `1000 kbps`，原因是封包可能集中在某些 80 ms periods 內 burst 到達。

但 `avg_remaining_kbps` 仍然不是 0，因為很多 individual periods 沒有收到該 beam 的 packets。程式是先對每個 period 計算 remaining，再把各 period 平均起來。
