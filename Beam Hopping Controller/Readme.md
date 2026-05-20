# Layer 2：Beam Hopping Controller

> 完整規格與架構設計見：[Layer2.md](../Layer2.md)

## 模組一覽

| 模組 | 檔案 | Phase | 職責 |
|------|------|:-----:|------|
| SatBhHelper | `sat-bh-helper.h/.cc` | 1–E | 統一安裝入口、所有模組接線 |
| SatBhTimePlan | `sat-bh-time-plan.h/.cc` | 1 | BHTP 資料模型（BhSlotEntry per-beam） |
| SatBhScheduler | `sat-bh-scheduler.h/.cc` | 2 | NCC 端 EM 演算法 → BHTP 產生 |
| SatBhObc | `sat-bh-obc.h/.cc` | 2 | 衛星端 BHTP 執行器（狀態機） |
| SatGwCacheQueue | `sat-gw-cache-queue.h/.cc` | 3 | Beam inactive 時封包緩衝 |
| SatBhPrecoder | `sat-bh-precoder.h/.cc` | 3 | cluster ≥ 2 beam 時 MMSE 預編碼 |
| SatBhMetrics | `sat-bh-metrics.h/.cc` | 1 | Packet-level beam KPI CSV |
| SatUserAssociator | `sat-bh-user-associator.h/.cc` | C | WFQ/Priority/RR UT-beam 分配 |
| SatResourceManager | `sat-bh-resource-manager.h/.cc` | C | 503 ms self-scheduling frame 主控 |
| SatPowerAllocator | `sat-power-allocator.h/.cc` | D | IWFA water-filling TX 功率最佳化 |
| SatL1RoutingInterface | `sat-l1-routing-interface.h/.cc` | B | Layer 1 ISL Routing 預留介面（stub） |

## 執行指令

```bash
# Phase 1（預設，無 traffic）
./ns3 run "sat-bh-example"

# Phase 1 with traffic（SNS3 CBR FWD+RTN，iridium66 全星座）
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=60" \
  2>&1 | tee Outputs/66_bhtp/traffic/run.log

# Phase 2（Scheduler + OBC stubs）
./ns3 run "sat-bh-example --enableScheduler=1 --enableObc=1"

# Phase C（ResourceManager + UserAssociation）
./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1"

# Phase D（+ Power Allocation）
./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1"

# Phase E（+ 真實 SNS3 API 接線）
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --satId=1 --simTime=60" \
  2>&1 | tee bh_phasee.log

# Phase F（+ 真實 DAMA demand trace）
NS_LOG="SatBhHelper=info:SatResourceManager=info" \
./ns3 run "sat-bh-example --scenario=iridium66 --simTime=120 \
           --enableResourceManager=1 --enableUserAssociation=1 \
           --enablePowerAllocation=1 --enablePhaseE=1 --enablePhaseF=1 --satId=1" \
  2>&1 | tee bh_phasef.log
```

## 輸出檔案

| 檔案 | 內容 |
|------|------|
| `bh-metrics_{scenario}_{timestamp}.csv` | Packet-level beam KPI（每 503 ms，含 sat_id 欄位） |
| `bh-timeplan_{scenario}_{timestamp}.csv` | BHTP slot table（per-beam pattern 格式） |
| `bh-attributes.xml` | ns-3 ConfigStore attribute snapshot |

### 已驗證輸出結果一覽

| 資料夾 | Phase | 說明 | 狀態 |
|--------|:-----:|------|:----:|
| `Outputs/66_bhtp/phase1/` | 1 | 靜態 BHTP，無 traffic，合成 demand | ✅ |
| `Outputs/66_bhtp/phase2/` | 2 | Scheduler+OBC stub，輸出與 Phase 1 相同 | ✅ |
| `Outputs/66_bhtp/phasee/` | E | ResourceManager loop 驗證，無 live traffic | ✅ |
| `Outputs/66_bhtp/phasef/` | F | RBDC trace 接線，DAMA CR 觸發測試 | ✅ |
| `Outputs/66_bhtp/rbdc/` | F | RBDC demand 驗證輸出 | ✅ |
| `Outputs/66_bhtp/traffic/` | 1+traffic | 66 衛星 iridium66，SNS3 CBR FWD+RTN 真實 traffic | ✅ |

#### `traffic/` 結果摘要（2026-05-17，iridium66，simTime=60s）

```
BHTP：2-beam 交替（slotDuration=26ms，19 slots × 26ms ≈ 503ms）
beam 1: throughput=0.057 Mbps，avg_delay=10ms，slot_util=79.17%，drop_rate=1.96%
beam 2: throughput=0.051 Mbps，avg_delay=10ms，slot_util=79.17%，drop_rate=0%
jain_fairness=0.9972（全 66 顆衛星一致）
```

- slot_util ~79% = K=2/M=19×2 beams → 符合靜態交替 BHTP 預期
- throughput 非零 → 真實 SNS3 FWD CBR traffic 已流通
- drop_rate beam1=1.96% → 封包在 beam inactive 時稍有掉包（無 CacheQueue 緩衝）

## 實作進度

| Phase | 說明 | 狀態 |
|-------|------|:----:|
| Phase 1 | SatBhTimePlan + SatBhMetrics | ✅ |
| Phase 2 | SatBhScheduler + SatBhObc | ✅ |
| Phase 3 | SatGwCacheQueue + SatBhPrecoder | ✅ |
| Phase B | BhSlotEntry per-beam 擴充 + SatL1RoutingInterface stub | ✅ |
| Phase C | SatUserAssociator + SatResourceManager | ✅ |
| Phase D | SatPowerAllocator（IWFA） | ✅ |
| Phase E | 真實 SNS3 API 接線（MoveUtBetweenBeams / SetTxMaxPowerDbw） | ✅ |
| Phase F | 真實 DAMA demand trace（BacklogRequestsTrace → ResourceManager） | ✅ |
| Traffic | SNS3 CBR FWD+RTN 真實 traffic 接線（sat-bh-example Step 3） | ✅ |
