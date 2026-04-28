# 2026-04-13

## 目標

將 test-iridium_baseline.cc 重構為具備三段 E2E 架構的 test-iridium-e2e.cc，以 `feederlink / isl / servicelink` 鏈路名稱作為執行邏輯，並觀測個鏈路間的行為。

---

## 已完成工作

### 1. 預計新增 test-iridium-e2e.cc（E2E 重構版）

- 保留 baseline 中已驗證的 RBDC trace、ISL drop rate trace、GW preset、TrafficConfig 等功能模組
- 新增 `E2ESegment` enum（FEEDERLINK / ISL / SERVICELINK），作為流量段的語意標籤
- 新增 `E2EConfig` struct，統一收納所有模式參數（mode、legacyProfile、explicitSegments、simTimeSec、各段 TrafficConfig、GW/UT ID 等）
- 新增 `E2ESegmentConfig` struct（enabled flag + 各段獨立 TrafficConfig）
- 新增 `E2EExecutionPlan` struct，收納四種流量安裝旗標（sharedEdge / islBg / gw2gwBg / gw2gwDirect）
- 新增 `BuildE2EPlan()` 函式：先執行 `ValidateE2EConfig()` 驗證參數合法性，再依 legacyProfile 填充執行計畫
- 新增 `ValidateE2EConfig()` 函式：對 mode、gwSrc/gwDst 一致性、gwPreset 存在性、utName 非空等進行前置斷言
- 新增 `ApplyLegacySegmentDefaults()` 函式：當使用者未手動指定 `--enableFeederlink` 等旗標時，依 legacyProfile 自動推斷啟用哪些段
- 新增 `PrintE2ERunBanner()` 函式：在模擬開始前列印 mode、各段 on/off、各流量旗標的設定摘要
- 新增 `InstallGwUtSegmentTrafficBase()` 函式：對原 `InstallGwUtCbrTraffic()` 重構，加入 segmentLabel 參數（"feederlink" / "isl" / "servicelink"），供日誌識別各段來源
- 新增 `InstallFeederlinkTraffic()`、`InstallIslTraffic()`、`InstallServicelinkTraffic()` 三個段安裝函式，各自呼叫 `PrintSegmentBanner()` 並依 plan 旗標決定裝哪種流量
- 新增 `InstallE2ETraffic()` 組合函式：依序呼叫三段安裝函式，並實作 feederlink 安裝成功後 servicelink 共用流量的邏輯
- 新增 `ConfigureRoutingCase()` 函式：將 main() 中原本的 if/else routing case 邏輯抽出為獨立函式，以 `E2EConfig` 為參數
- `main()` 中新增 `--enableFeederlink`、`--enableIsl`、`--enableServicelink` CLI 參數，支援逐段獨立開關
- `main()` 中將散落的流量安裝呼叫替換為 `PrintE2ERunBanner()` + `InstallE2ETraffic()` 兩行組合呼叫
- `SimulationHelper` 模擬名稱由 `"test-iridium-3modes"` 改為 `"test-iridium-3segment-e2e"`

---

## Bug 修復 / 結構改動

### 流量安裝邏輯：從 switch-case profile 改為 Plan-driven 段架構

- 檔案：`C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\test-iridium-e2e.cc`
- 改動：原本 `InstallTrafficByProfile()` 以 switch-case 依 profile 安裝，無法表達「哪個段在跑」；改為以 `E2EExecutionPlan` 旗標驅動，每個段有明確的安裝函式與 banner 輸出
- 原因：為下一步分段驗證（只測 feederlink 或只測 isl）提供明確控制點，並讓日誌輸出能清楚顯示每個 E2E 段的啟用狀態

### gw2gw background traffic：安裝函式段標籤修正

- 檔案：`C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes\test-iridium-e2e.cc`
- 改動：`InstallGw2GwBackgroundLoad()` 由呼叫舊的 `InstallGwUtCbrTraffic()` 改為分別以 `"feederlink"` 和 `"servicelink"` 為標籤呼叫 `InstallGwUtSegmentTrafficBase()`
- 原因：確保背景流量輸出與段架構語意一致，便於日誌追蹤

---


## 下一步

- [ ] 在 SNS3 環境中編譯 test-iridium-e2e.cc，確認無建置錯誤
- [ ] 以 `--mode=gw2gw --enableFeederlink=1 --enableIsl=1 --enableServicelink=1` 執行完整 E2E 測試，確認三段 banner 均正確輸出
- [ ] 以 `--enableIsl=1`（其餘段關閉）單獨測試 ISL 段，確認 `InstallIslTraffic()` 的 `installIslBackgroundTraffic` 旗標正確觸發
- [ ] 確認 `[E2E]` run banner 格式在 log 中可供快速識別段設定

---

## 備註

- test-iridium_baseline.cc 保留不動，作為對照基準。
- E2E 新架構向下相容：不傳 `--enableFeederlink/Isl/Servicelink` 時，`ApplyLegacySegmentDefaults()` 會依 `--trafficProfile` 自動推斷啟用段，行為與 baseline 一致。
- `ValidateE2EConfig()` 加入前置斷言，防止 gwSrc == gwDst 或非法 gwId 在模擬啟動後才觸發 ABORT。
