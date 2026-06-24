# SNS3 Beam `userChannelId` Code Trace 

## 1. 研究問題


同一個 BHTP slot 同時開啟多個 beams，
這些 beams 如何使用不同 userChannelId / carrier / SatChannel，
以降低同頻干擾並服務更多 cells？


在 SNS3 中，追蹤實際的：

```text
beamId -> userChannelId -> SatChannel -> carrierId -> PHY interference
```

---

# 2. 概念

## 2.1 Beam

`beamId` 代表 spot beam 編號。

例如：

```text
beam 1
beam 2
beam 3
...
```
---


## 2.2 userChannelId

`userChannelId` 是 beam 在 user link 使用的 channel ID。

原生 beam configuration 格式是：

```text
beamId userChannelId gwId feederChannelId
```

也就是：

```text
beamId
決定是哪一個 beam

userChannelId
決定 user link 使用哪一個 user channel / frequency band

gwId
決定連到哪個 gateway

feederChannelId
決定 feeder link 使用哪一個 feeder channel
```

做 frequency reuse，應該透過：

```text
beamId -> userChannelId
```

---

# 3. Code Trace 主線

## Step 1：beam config 被讀進來

`satellite-conf.cc`

讀取 beam config：

```text
beamId userChannelId gwId feederChannelId
```

建立：

```text
beamId -> userChannelId
beamId -> feederChannelId
beamId -> gwId
```

---

## Step 2：beam 被綁定到對應的 channel

當 SNS3 建立 satellite topology / helper / beam 時，會根據 beam config 把 beam 指到對應 channel。

`**frequency reuse**`
```text
beam 1 -> userChannelId 0
beam 2 -> userChannelId 1
beam 3 -> userChannelId 2
beam 4 -> userChannelId 3
beam 5 -> userChannelId 0
```

 

如果：

```text
beam 1 -> userChannelId 0
beam 5 -> userChannelId 0
```

代表 beam 1 和 beam 5 共用同一個 user channel。

也就是：

```text
co-frequency reuse
```

---

## Step 3：SatChannel 表示實際 frequency band

`satellite-channel.h`


```text
SatChannel 代表一個 frequency band
```

所以：

```text
相同 userChannelId
通常會對應相同 SatChannel
```

結果：

```text
beam A 和 beam B 使用同一個 SatChannel
=> receiver 端可能看得到彼此
=> 可能進入 interference calculation
```

反之：

```text
beam A 和 beam B 使用不同 SatChannel
=> 代表不同 frequency band
=> 理論上不會形成 co-channel interference
```

---

## Step 4：封包進入 PHY 時帶有 beamId 和 carrierId

`satellite-phy.cc`


概念程式碼：

```cpp
txParams->m_beamId = m_beamId;
txParams->m_carrierId = carrierId;
```

這表示封包送進 PHY 時，會記錄：

```text
這個 packet 來自哪一個 beam
這個 packet 使用哪一個 carrier
```

所以 receiver / interference model 可以根據：

```text
beamId
carrierId
channel
```

判斷是否有干擾。

---

# 4. userChannelId 與干擾的關係

## Case A：多個 beam 使用同一個 userChannelId

例如：

```text
slot 0:
  beam 1 -> userChannelId 0
  beam 2 -> userChannelId 0
  beam 3 -> userChannelId 0
```

代表：

```text
三個 beams 同時開啟
而且使用相同 user channel
```

這是：

```text
co-channel multi-beam transmission
```

可能造成：

```text
interference 增加
SINR 下降
throughput 下降
```

---

## Case B：多個 beam 使用不同 userChannelId

例如：

```text
slot 0:
  beam 1 -> userChannelId 0
  beam 2 -> userChannelId 1
  beam 3 -> userChannelId 2
```

代表：

```text
三個 beams 同時開啟
但使用不同 user channel
```

這是：

```text
different-frequency multi-beam transmission
```

理論上可以：

```text
降低 co-channel interference
讓同一個 BHTP slot 服務更多 cells
提升 service opportunity
```

---

# 5. 對 Beam Hopping 的設計意義

BHTP 本身應該只負責：

```text
哪個時間 slot
開啟哪些 beams
```

例如：

```text
slot 0:
  active beams = 1, 2, 3

slot 1:
  active beams = 4, 5, 6
```

應該檢查：

```text
active beams 的 userChannelId 是否相同
```

也就是：

```text
同一個 slot 裡，同時 active 的 beams
如果 userChannelId 相同
=> 可能同頻干擾

如果 userChannelId 不同
=> 可視為不同頻率
```

---

# 6. Scheduler 檢查邏輯

可以建立一個 mapping：

```cpp
std::map<uint32_t, uint32_t> beamToUserChannel;
```

例如：

```cpp
beamToUserChannel[1] = 0;
beamToUserChannel[2] = 1;
beamToUserChannel[3] = 2;
beamToUserChannel[4] = 3;
beamToUserChannel[5] = 0;
```

然後在排 BHTP slot 時檢查：

```cpp
bool CanCoSchedule(
    uint32_t newBeam,
    const std::vector<uint32_t>& slotBeams,
    const std::map<uint32_t, uint32_t>& beamToUserChannel)
{
    uint32_t newChannel = beamToUserChannel.at(newBeam);

    for (uint32_t existingBeam : slotBeams)
    {
        uint32_t existingChannel = beamToUserChannel.at(existingBeam);

        if (existingChannel == newChannel)
        {
            return false;
        }
    }

    return true;
}
```

這代表：

```text
同一個 BHTP slot 內
不要同時排到相同 userChannelId 的 beams
```

---

這樣可以避免相鄰 cell 過度集中在同一個 channel。

---


# 7. Trace 輸出欄位

為了證明 userChannelId 有真的進入排程與 PHY，可輸出：

```text
time_s
slot_id
beam_id
cell_id
active
user_channel_id
carrier_id
sat_channel_id
snr_db
sinr_db
interference_db
throughput_bps
```

尤其：

```text
beam_id
user_channel_id
carrier_id
sinr_db
interference_db
```

以證明 SNS3 的實際干擾模型有被改變。

---

# 11. 結論

在 SNS3 中，影響干擾的是 beam 實際綁定的 `userChannelId`、`carrierId` 與 `SatChannel`。因此做 Beam Hopping 時，BHTP 負責決定同一個 slot 啟用哪些 beams，而 `userChannelId` 負責決定這些 beams 是否使用相同頻率、是否可能互相干擾。
