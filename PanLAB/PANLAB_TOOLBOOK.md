# PanLAB 工具書

本文件整理[PanLab](https://onlineacademiccommunity.uvic.ca/starlink/)中的主要工具、資料流與研究用途。

## 1. PanLAB 中 Github tools

1. `LEOViz`
   Starlink/OneWeb 觀測、服務衛星推估、視覺化、驗證報告。
2. `python-skyfield`
   天文與軌道計算核心函式庫，負責時間、TLE、星曆、座標與觀測幾何。
3. `starlink-lens`
   Starlink gateway 延遲量測代理工具，可排程蒐集 `ping` / `irtt` / obstruction map。
4. `starlink_exporter`
   Prometheus exporter，將 Starlink dish 的 gRPC 狀態轉成監控指標。
5. `Unofficial Starlink GeoIP Map`
   Starlink GeoIP / PoP / BGP / RIPE Atlas / availability 的資料管線與地圖輸出。

流程：

1. `starlink-lens` 或 `LEOViz` 蒐集原始觀測資料
2. `LEOViz` 對 obstruction / status / location / TLE 做融合與服務衛星推估
3. `python-skyfield` 提供衛星軌道與天體位置計算
4. `starlink_exporter` 提供即時監控視角
5. `starlink-geoip` 提供 PoP / GeoIP / backbone / 地理背景資料
6. `validation_tool.py` 生成可重現性報告，支撐論文 artifact

---

## 2. LEOViz

### 2.1 工具定位

`LEOViz` :研究成果展示與論文驗證的主工具。

提供：

- Starlink dish 資料收集
- obstruction map 蒐集
- dish status / GPS diagnostics 蒐集
- TLE 載入
- 服務衛星推估
- 視覺化影片輸出
- sample data 測試
- 中文驗證報告工具

### 2.2 主要入口

- starlink/main.py : 主程式入口，負責 CLI 參數、排程與工作啟動。
- starlink/jobs.py : 資料蒐集工作管理器，處理 gRPC status、GPS、obstruction map。
- starlink/data_feature_extraction.py : 
  obstruction / status / location 特徵融合與前處理。
- starlink/satellite_matching_estimation.py : 服務衛星推估核心演算法。
- starlink/plot.py : 輸出視覺化影片。
- starlink/validation_tool.py : 驗證單次實驗輸出是否完整、合理，產生中文報告。


### 2.3 子模組用途

- `config.py`
  儲存 gRPC 位址、gateway、資料目錄、量測時間、TLE URL 等設定。
- `grpc_command.py`
  包裝 Starlink gRPC 呼叫。
- `latency.py`
  執行 ICMP ping 量測。
- `obstruction.py`
  obstruction map 資料解析與處理。
- `timeslot_manager.py`
  15 秒 timeslot 對齊邏輯。
- `location_provider.py`
  提供 stationary 或 mobile 模式下的 observer location。
- `pop.py`
  與 PoP 資訊相關的輔助邏輯。
- `gs.py`
  地面站或座標處理輔助。
- `util.py`
  共用工具函式，例如資料目錄建立、TLE 載入。

### 2.4 主要輸入

- Starlink dish gRPC 回傳
- obstruction map parquet
- latency ping log
- TLE 快照
- stationary 模式下的 `lat/lon/alt`
- mobile 模式下的 GPS diagnostics

### 2.5 主要輸出

- `GRPC_STATUS-*.csv`
- `GRPC_LOCATION-*.csv`
- `obstruction_map-*.parquet`
- `obstruction-data-*.csv`
- `processed_obstruction-data-*.csv`
- `serving_satellite_data-*.csv`
- `starlink-*.mp4`



---

## 3. python-skyfield

### 3.1 工具定位

`python-skyfield` 是軌道計算與天文座標轉換核心。  
在 PanLAB 中，它的主要研究用途是：

- 載入 Starlink TLE
- 計算衛星於不同時刻的方位角/仰角
- 根據 observer location 建立 topocentric 視角
- 支撐 LEOViz 的服務衛星匹配

### 3.2 主要入口

- skyfield/api.py : 對外 API 入口。
- skyfield/timelib.py : 時間核心。
- skyfield/positionlib.py : 位置與觀測核心。
- skyfield/vectorlib.py : 向量函式與組合。
- skyfield/sgp4lib.py : TLE/SGP4 衛星推進。
- skyfield/jpllib.py : JPL 星曆。
- skyfield/toposlib.py : 地面站模型。
- skyfield/framelib.py : 座標框架。
- skyfield/almanac.py : 事件型天文函式。

### 3.3 在 PanLAB 中的角色

LEOViz把觀測到的 obstruction trajectory 和 Skyfield 算出的實際衛星角度軌跡匹配。  

> Skyfield 是真實幾何引擎。

### 3.4 研究注意事項

- 版本要固定，避免不同 Skyfield / sgp4 版本造成小幅軌道差異。
- TLE 必須和實驗時間相近。
- location timestamp 與 observation timestamp 必須對齊。

---

## 4. starlink-lens

### 4.1 工具定位

`starlink-lens` :量測代理與資料採集器，偏長時間收資料，而不是直接做視覺化。

用途：

- 定時執行 `ping`
- 執行 `irtt`
- 取得 obstruction map
- 上傳資料到 Swift object storage
- 配合 cron/scheduler 長時間蒐集 LENS dataset

### 4.2 主要入口

- cmd/lens/main.go : 主程式入口。
- cmd/lens/config.go : 環境變數與設定讀取。
- cmd/lens/ping.go :  `ping` / `irtt` 執行與檔案輸出。
- cmd/lens/sync.go : Swift 上傳邏輯。
- cmd/lens/grpc.go : obstruction map 相關 gRPC 功能。

### 4.3 用途

LENS 更適合：

- 長時間跑資料收集
- 多節點部署
- 自動化上傳
- 建立 latency dataset

### 4.4 和 LEOViz 的關係

- `starlink-lens` 偏蒐集器
- `LEOViz` 分析器 + 視覺化器 + 驗證器

---

## 5. starlink_exporter

### 5.1 工具定位

`starlink_exporter` 的角色是監控，不是論文推估主引擎。

它會：

- 透過 Starlink gRPC 抓設備狀態
- 轉成 Prometheus metrics
- 提供 `/metrics`、`/health`
- 支援 Grafana 監控

### 5.2 主要入口

- cmd/starlink_exporter/main.go : 
啟動 HTTP server、註冊 exporter。
- internal/exporter/exporter.go : 核心採集邏輯。
- [internal/exporter/metric.go : metric schema 定義。
- internal/publicIP/exporter.go : public IP / PTR / PoP 補充資訊。

### 5.3 適合使用場景

- 觀察 dish 即時健康狀況
- throughput、latency、GPS、alert 等資料接進 Grafana
- 和 LEOViz / LENS 的 batch 結果做對照

### 5.4 不適合代替

不能取代：

- obstruction-based serving satellite estimation
- 視覺化影片輸出
- 論文驗證報告

---

## 6. Unofficial Starlink GeoIP Map

### 6.1 工具定位

資料管線與地圖背景系統，用來回答：

- 某 subnet 對應哪個 PoP
- 某 PoP 在哪個城市/國家
- Starlink ASN 宣告了哪些前綴
- RIPE Atlas probes 如何分布
- availability zones 長什麼樣子

### 6.2 主要入口

- starlink-geoip-master/run.py : 總控排程入口。
- geoip_pop.py : feed / pops / PTR 合併主流程。
- map/process_map.py : map JSON 產生器。
- bgp.py : BGP 前綴蒐集。
- atlas.py : RIPE Atlas probes 蒐集。
- availability.py : availability cells 轉換。
- monthly_latency_snapshot.py : 每月 latency 快照。



### 6.4 和 LEOViz 的關係

LEOViz 主要回答可能在連哪顆衛星。  
GeoIP Map 這塊主要回答可能出在哪個地面 PoP / 網路區域。
併後，會形成：

- 空間層：哪顆衛星
- 網路層：哪個 PoP / 哪段 backbone

---



### Skyfield

- 結果對 TLE 時效性敏感
- 版本差異可能造成微小幾何偏移

### starlink-lens

- 偏 Linux / shell / scheduler 環境
- 需要額外依賴例如 `ping`、`irtt`、Swift 連線

### starlink_exporter

- 適合監控，不等於論文級 serving satellite 推估器

### GeoIP Map

- 大量依賴外部 feed、BGP、DNS PTR、PeeringDB、ArcGIS geocoder
- 資料具有時間性，不同日期輸出可能不同

---

## 12. 

- `LEOViz`：實驗主軸
- `Skyfield`：軌道物理引擎
- `LENS`：長時段量測器
- `Exporter`：即時監控器
- `GeoIP Map`：PoP / backbone 背景資料系統

