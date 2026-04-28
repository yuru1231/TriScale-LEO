# gw2gw_e2e — gwMode 比較測試

比較兩種端點模式在相同路徑（TW-Taipei → US-Seattle，120s）下的行為差異。

---

## Mode 1：user node（預設，完整 E2E）

流量路徑：`GW_user0 → CSMA → GW_phys0 → feeder → SAT → ISL → SAT → feeder → GW_phys2 → CSMA → GW_user2`

```bash
./ns3 run "scratch/test-iridium-e2e-fix \
  --pathType=gw2gw_e2e \
  --gwSrc=0 --gwDst=2 \
  --simTime=120 --slotInterval=60 \
  --gwMode=user" \
  2>&1 | tee "Topology & ISL Routing/Outputs/E2E-ReturnFeeder/gw2gw_gwmode_user_120s.log"
```

預期觀察點：
- `[GW2GW_APP] gwMode=user`
- `GW0=90.2.0.2 -> GW2=90.2.0.x`（user subnet IP）
- PacketSink 安裝在 GW user node

---

## Mode 2：physical node（feeder+ISL 路徑）

流量路徑：`GW_phys0 → feeder → SAT → ISL → SAT → feeder → GW_phys2`

```bash
./ns3 run "scratch/test-iridium-e2e-fix \
  --pathType=gw2gw_e2e \
  --gwSrc=0 --gwDst=2 \
  --simTime=120 --slotInterval=60 \
  --gwMode=physical" \
  2>&1 | tee "Topology & ISL Routing/Outputs/E2E-ReturnFeeder/gw2gw_gwmode_physical_120s.log"
```

預期觀察點：
- `[GW2GW_APP] gwMode=physical`
- `[GW2GW_APP] physical GW0 routable IP: <addr> (ifIndex=<i>)`（確認解析到的 IP）
- 若 IP 解析到 CSMA 介面（90.x.x.x），路徑含 GW user 到 physical 的 CSMA hop
- 若 IP 解析到 feeder 介面（10.x.x.x），路徑為純 feeder+ISL

---

## 比較重點

| 觀察項目 | user mode | physical mode |
|---|---|---|
| srcAddr / dstAddr | 90.2.0.x（user subnet）| 視介面順序決定 |
| PacketSink 節點 | GW user node | physical GW node |
| 包含 CSMA hop | 是 | 視 IP 解析結果 |
| 路由涵蓋範圍 | 完整 E2E | feeder+ISL（或部分）|
| 預期 rxBytes | > 0（已驗證） | 需驗證 |

---

## 注意事項

- physical mode 的 IP 解析結果需從 log 中
  `[GW2GW_APP] physical GW<id> routable IP` 確認
- 若 physical mode 的 `rxBytes=0`，可能原因：
  1. physical GW node 的 IP 不在衛星路由表的 return path 內
  2. feeder link IP 屬於不同子網路，缺少跨子網路路由
  3. 需確認 SNS3 regeneration mode 下 physical GW node 的 IP stack routing 設定
