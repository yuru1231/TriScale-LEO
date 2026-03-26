# CLAUDE.md — Claude Code 行為規範

這份文件定義 Claude Code 在此專案中的行為規則，必須嚴格遵守。

---

## 執行規則

**不執行任何指令。**
使用者在 Windows 上用 VS Code + Claude Code 開發。Claude Code 可以直接覆蓋 VS Code 工作區內的檔案。
實際執行由使用者手動複製到 VMware 上的 SNS3 環境，Claude Code 不呼叫任何 shell 指令、不執行編譯、不跑測試。

**路徑規則**
- 跟routing isl layer1相關的要寫在Layer1，code要改在C:\Users\wenj\Desktop\TriScale-LEO\Topology & ISL Routing\Codes
- 跟Beam hopping 相關的要寫在Layer2，code要改在C:\Users\wenj\Desktop\TriScale-LEO\Beam Hopping Controller\Codes
- **跟qos layer3相關的要寫在Layer1，code要改在C:\Users\wenj\Desktop\TriScale-LEO\QoS-Aware Packet Scheduler\Codes

如果關連到end to end(E2E)相關無法完全歸類為某一個層面，以在哪裡實作作為路徑選擇

Guide是整個架構的說明 要做為一個可以讓人立刻了解並跟著實作的
---

## 改 code 的規則
**要寫註解**
**改動前先說明範圍與預期輸出。**
每次提供程式碼前，先用一段文字說明：
- 改了哪個檔案的哪個函式或區塊
- 預期執行後會看到什麼輸出或行為變化



**改動範圍要明確。**
如果一次改動涉及多個檔案，逐一列出，不要把多個檔案的內容混在一起。

---

## 設計規則
**OOP**
參數皆須遵循OOP，避免hard code
**有不確定的地方先問，不要自己發明。**
遇到接口不確定、設計有歧義、或需要在兩個方案之間選擇時，先提出問題，等確認後再寫程式碼。

**不動 SNS3 原始碼，除非明確討論過。**
SNS3 原始碼（`contrib/satellite/` 下的原生檔案）不得修改，除非使用者明確提出並討論過。目前已確認可以改動的 SNS3 檔案：
- `satellite-sgp4-mobility-model.h/.cc`（已加入 `GetGeoPositionAt`）
- `satellite-isl-arbiter-unicast.h`（已加入 `ClearNextHopEntries`）

新的 SNS3 原始碼改動必須先說明原因，等確認後再動。

---