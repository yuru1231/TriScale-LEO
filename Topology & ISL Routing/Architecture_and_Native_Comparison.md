## ISL Routing 模組替換對照表
### 模組替換關係
| 類別          | 原生模組                               | 本研究模組                                  | 替換程度    | 精準說明                                |
| ----------- | ---------------------------------- | -------------------------------------- | ------- | ----------------------------------- |
| Routing 控制層 | `SatIslArbiterUnicastHelper`       | `IslRoutingManager`                    | 🔴 部分替換 | 取代「routing 計算與安裝流程」，但不取代 forwarding |
| Routing 初始化 | `SatOrbiterHelper::SetIslRoutes()` | `Initialize()` + `ApplyRoutingTable()` | 🔴 部分替換 | 原生一次性設定 → 改為可多次更新                   |
| Routing 計算  | Floyd-Warshall                     | Dijkstra + cost                        | 🔴 完全替換 | metric 與時間維度皆不同                     |
| Routing 更新  | `UpdateArbiters()`                 | `ScheduleRoutingUpdates()`             | 🔴 功能替換 | 原生只是重新 install                      |
| Forwarding  | `SatIslArbiterUnicast`             | 相同                                     | 🟢 保留   | 僅使用其 lookup                         |


### 功能對照
| 功能             | 原生 SNS3 | IslRoutingManager | 判斷       |
| -------------- | ------- | ----------------- | -------- |
| 靜態 routing     | ✔       | ✔                 | baseline |
| 動態 routing     | ❌       | ✔                 | 新增       |
| hop-count      | ✔       | ✔                 | 共用       |
| delay cost     | ❌       | ✔                 | 新增       |
| queue cost     | ❌       | ✔                 | 新增       |
| multi-time     | ❌       | ✔                 | 新增       |
| routing update | ❌       | ✔                 | 新增       |
| forwarding     | ✔       | ✔                 | 共用       |

### API 使用層級
| API / 函式                | 來源 | 使用方式 | 是否重寫  |
| ----------------------- | -- | ---- | ----- |
| `AddNextHopEntry()`     | 原生 | ✔ 使用 | ❌     |
| `SetArbiter()`          | 原生 | ✔ 使用 | ❌     |
| `ClearNextHopEntries()` | 原生 | ✔ 使用 | ❌     |
| `InstallArbiters()`     | 原生 | ❌ 未用 | ✔ 被取代 |
| `UpdateArbiters()`      | 原生 | ❌ 未用 | ✔ 被取代 |
| routing algorithm       | 原生 | ❌    | ✔ 自建  |
| scheduling              | 原生 | ❌    | ✔ 自建  |

### 限制
| 原生設計              | 限制                  |
| ----------------- | ------------------- |
| Floyd-Warshall    | 無法 scale + 無 metric |
| hop-count only    | 不符合真實 delay         |
| init-only routing | 不適用 LEO             |
| 無 queue model     | 無法反映 congestion     |
| 無 UT/GW path      | 無法做 E2E             |

因此
| 模組                          | 必須保留               |
| --------------------------| -------------------- |
| `PrecomputeAllTables()`    | 原生無時間維度              |
| `ScheduleRoutingUpdates()` | 原生無 runtime update   |
| `ApplyRoutingTable()`      | 原生無動態安裝              |
| queue-aware cost          | 原生無 congestion model |
| GW/UT routing             | 原生無 E2E              |
