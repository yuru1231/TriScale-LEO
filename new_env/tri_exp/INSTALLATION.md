# tri_exp 在 ns-3.43 的安裝說明

這份文件是將 `tri_exp` 部署到 ns-3.43 目錄的分階段 SOP。。

## 本機基準


```text
tri_exp path:        /home/lucy/tri_exp
ns-3.43 path:        /home/lucy/ns-allinone-3.43/ns-3.43
gcc:                 11.4.0
cmake:               3.22.1
python3:             3.10.12
```

preflight 檢查：

```text
[PASS] /home/lucy/ns-allinone-3.43/ns-3.43 exists
[PASS] /home/lucy/ns-allinone-3.43/ns-3.43/ns3 is executable
[PASS] /home/lucy/ns-allinone-3.43/ns-3.43/contrib/satellite exists
[PASS] /home/lucy/ns-allinone-3.43/ns-3.43/contrib/satellite/data exists
[PASS] /home/lucy/tri_exp exists
[PASS] /home/lucy/tri_exp/model exists
[PASS] /home/lucy/tri_exp/helper exists
[PASS] /home/lucy/tri_exp/contribsatellite/CMakeLists.txt exists
[PASS] /home/lucy/tri_exp/scratch/bh_dynamic/CMakeLists.txt exists
[PASS] /home/lucy/tri_exp/scratch/bh_dynamic/Codes/CMakeLists.txt exists
[PASS] /home/lucy/tri_exp/scratch/bh_dynamic/Codes/sat-bh-example.cc exists
[PASS] /home/lucy/tri_exp/scratch/bh_dynamic/Codes/sat-bh-2d-footprint.cc exists
[PASS] satellite data contains scenarios/constellation-starlink-1584-sats
```

本機 clean-room 驗證，使用：

```text
test ns-3.43 path: /home/lucy/ns-allinone-3.43-tri-exp-test/ns-3.43
install method:    scripts/install_tri_exp_minimal.sh --apply
```

已驗證指令：

```bash
cd /home/lucy/ns-allinone-3.43-tri-exp-test/ns-3.43
rm -rf build cmake-cache
./ns3 configure --enable-examples --enable-tests
./ns3 build
./ns3 build bh_dynamic/Codes/sat-bh-example
./ns3 run "bh_dynamic/Codes/sat-bh-example --scenario=starlink25"
```

結果：

```text
[PASS] configure completed
[PASS] full build completed
[PASS] bh_dynamic/Codes/sat-bh-example target built
[PASS] starlink25 smoke test ran successfully
[PASS] bh_dynamic/Codes/sat-bh-2d-footprint geometry-only smoke test ran successfully
```

## 重要部署模型

`tri_exp` 不是單純的 scratch 程式。它會修改 ns-3 的 satellite contrib module，
並將 beam-hopping 支援加入 satellite library。

預期檔案映射：

```text
tri_exp/model/<changed files only>      -> ns-3.43/contrib/satellite/model/
tri_exp/helper/<changed/new files only> -> ns-3.43/contrib/satellite/helper/
tri_exp/contribsatellite/CMakeLists.txt
                                        -> ns-3.43/contrib/satellite/CMakeLists.txt
tri_exp/scratch/bh_dynamic/*            -> ns-3.43/scratch/bh_dynamic/
```

`tri_exp/contribsatellite/CMakeLists.txt` 會安裝成
`contrib/satellite/CMakeLists.txt`，讓 ns-3 使用更新後的 satellite build
definition。

最小安裝清單記錄於：

```text
/home/lucy/tri_exp/INSTALL_MANIFEST.txt
```

可用以下 script 檢查或套用 manifest：

```text
/home/lucy/tri_exp/scripts/install_tri_exp_minimal.sh
```

除非有明確原因，不要複製整個 `model/` 與 `helper/` 目錄。本機比對結果中，
有 385 個檔案與 ns-3.43 完全相同，不需要安裝。

## Clean-room 安裝原則

部署到實際工作用 ns-3 之前，必須先在複製出來的測試目錄中驗證：

```bash
cd /home/lucy
cp -a ns-allinone-3.43 ns-allinone-3.43-tri-exp-test
```

使用複製出的目錄作為安裝目標：

```text
/home/lucy/ns-allinone-3.43-tri-exp-test/ns-3.43
```

重要：如果複製出的目錄中含有 `build/` 或 `cmake-cache/`，在執行
`./ns3 configure` 前必須先刪除它們。CMake cache 會保存原始 source tree 的絕對
路徑，因此複製過來的 cache 會造成 source-directory mismatch。

```bash
cd /home/lucy/ns-allinone-3.43-tri-exp-test/ns-3.43
rm -rf build cmake-cache
```

只有在複製目錄中的 configure、build、smoke test 都通過後，才能將相同流程套用
到主要 ns-3.43 工作目錄。

## 最小 clean-room 部署流程

本流程應先用在複製出來的 ns-3 tree。若目標平台路徑不同，請替換下列路徑。

```bash
TRI_EXP=/home/lucy/tri_exp
NS3=/home/lucy/ns-allinone-3.43-tri-exp-test/ns-3.43
```

先以 dry-run 模式執行 installer：

```bash
"$TRI_EXP/scripts/install_tri_exp_minimal.sh" --ns3 "$NS3"
```

安裝前預期 dry-run 摘要：

```text
replace-existing entries: 15
existing entries differ:  15
add-new entries:          32
scratch entries:          6
Dry-run only. No files were modified.
```

如果 dry-run 回報 source file 缺失、ns-3 目錄缺失，或檔案數量不符合預期，請停止
並先修正該條件，再執行 apply。

若要安裝到 clean-room copy，重新執行並加上 `--apply`：

```bash
"$TRI_EXP/scripts/install_tri_exp_minimal.sh" --ns3 "$NS3" --apply
```

以下保留 installer 的手動等價流程，方便審查與除錯。

安裝 `INSTALL_MANIFEST.txt` 中列出的 15 個既有 satellite 差異檔，以及 32 個新
增 satellite helper 檔：

```bash
cd "$TRI_EXP"

for f in \
  helper/satellite-conf.cc \
  helper/satellite-conf.h \
  helper/satellite-helper.cc \
  helper/satellite-helper.h \
  helper/simulation-helper.cc \
  helper/simulation-helper.h \
  model/satellite-isl-arbiter-unicast.h \
  model/satellite-isl-arbiter.cc \
  model/satellite-orbiter-net-device.cc \
  model/satellite-point-to-point-isl-channel.cc \
  model/satellite-point-to-point-isl-channel.h \
  model/satellite-sgp4-mobility-model.cc \
  model/satellite-sgp4-mobility-model.h \
  model/satellite-topology.cc \
  model/satellite-ut-mac.cc \
  helper/beam-hopping-manager.cc \
  helper/beam-hopping-manager.h \
  helper/ft-filter.cc \
  helper/ft-filter.h \
  helper/isl-graph.cc \
  helper/isl-graph.h \
  helper/sat-bh-helper.cc \
  helper/sat-bh-helper.h \
  helper/sat-bh-metrics.cc \
  helper/sat-bh-metrics.h \
  helper/sat-bh-obc.cc \
  helper/sat-bh-obc.h \
  helper/sat-bh-precoder.cc \
  helper/sat-bh-precoder.h \
  helper/sat-bh-resource-manager.cc \
  helper/sat-bh-resource-manager.h \
  helper/sat-bh-scheduler.cc \
  helper/sat-bh-scheduler.h \
  helper/sat-bh-time-plan.cc \
  helper/sat-bh-time-plan.h \
  helper/sat-bh-user-associator.cc \
  helper/sat-bh-user-associator.h \
  helper/sat-constellation-params.h \
  helper/sat-dynamic-bstp-provider.h \
  helper/sat-greedy-bstp-provider.cc \
  helper/sat-greedy-bstp-provider.h \
  helper/sat-gw-cache-queue.cc \
  helper/sat-gw-cache-queue.h \
  helper/sat-l1-routing-interface.cc \
  helper/sat-l1-routing-interface.h \
  helper/sat-power-allocator.cc \
  helper/sat-power-allocator.h
do
  install -D "$TRI_EXP/$f" "$NS3/contrib/satellite/$f"
done
```

安裝 satellite CMakeLists replacement：

```bash
install -D \
  "$TRI_EXP/contribsatellite/CMakeLists.txt" \
  "$NS3/contrib/satellite/CMakeLists.txt"
```

安裝 scratch 實驗：

```bash
for f in \
  scratch/bh_dynamic/CMakeLists.txt \
  scratch/bh_dynamic/Codes/CMakeLists.txt \
  scratch/bh_dynamic/Codes/README.md \
  scratch/bh_dynamic/Codes/sat-bh-2d-footprint.cc \
  scratch/bh_dynamic/Codes/sat-bh-example.cc \
  scratch/bh_dynamic/Codes/sat-constellation-params.h
do
  install -D "$TRI_EXP/$f" "$NS3/$f"
done
```

安裝檔案後，在複製目錄中強制重新 configure/build：

```bash
cd "$NS3"
rm -rf build cmake-cache
./ns3 configure --enable-examples --enable-tests
./ns3 build
```

## Dry-run 部署結果

本機 ns-3.43 tree 的 dry-run 比對結果：

```text
model/helper files with the same relative path:             400
model/helper files with identical content:                  385
model/helper files with different content:                  15
model/helper files that would be newly added to satellite:  32
scratch files to install under scratch/bh_dynamic:          6
satellite CMakeLists replacement:                           1
```

與 `tri_exp` 內容不同的既有 satellite 檔案：

```text
helper/satellite-conf.cc
helper/satellite-conf.h
helper/satellite-helper.cc
helper/satellite-helper.h
helper/simulation-helper.cc
helper/simulation-helper.h
model/satellite-isl-arbiter-unicast.h
model/satellite-isl-arbiter.cc
model/satellite-orbiter-net-device.cc
model/satellite-point-to-point-isl-channel.cc
model/satellite-point-to-point-isl-channel.h
model/satellite-sgp4-mobility-model.cc
model/satellite-sgp4-mobility-model.h
model/satellite-topology.cc
model/satellite-ut-mac.cc
```

15 個檔案有內容差異，其餘 385 個同相對路徑
檔案與目前本機 ns-3.43 satellite 檔案 byte-for-byte 完全相同。

完整差異清單在：

```text
tri_exp_ns343_different_existing_files.txt
```

檔案功能分類：

```text
Gateway/beam override API:
  helper/satellite-conf.cc
  helper/satellite-conf.h
  helper/satellite-helper.cc
  helper/satellite-helper.h

Custom constellation snapshot support:
  helper/simulation-helper.cc
  helper/simulation-helper.h

ISL routing and sparse constellation behavior:
  model/satellite-isl-arbiter-unicast.h
  model/satellite-isl-arbiter.cc
  model/satellite-orbiter-net-device.cc
  model/satellite-point-to-point-isl-channel.cc
  model/satellite-point-to-point-isl-channel.h
  model/satellite-topology.cc

SGP4 position/time helper:
  model/satellite-sgp4-mobility-model.cc
  model/satellite-sgp4-mobility-model.h

UT MAC stale-control-message tolerance:
  model/satellite-ut-mac.cc
```

修改新增的重要行為：

1. `SatConf` / `SatHelper` 可以覆寫某個 beam 使用的 gateway。
2. `SatHelper::IslsEnabled` 可以針對 sparse fixed snapshot 停用 ISL 安裝。
3. `SimulationHelper` 可以安裝自訂 constellation beam map、平移 SGP4 start time，
   並替 custom constellation beams 指定 gateway。
4. ISL 無路由情況會 log 並 drop，而不是讓整個 simulation abort。
5. Sparse constellation 的 topology printing / routing 不再假設每顆 orbiter 都具
   有所有 beam layer。
6. `SatSGP4MobilityModel` 可以查詢任意 simulation time 的衛星地理位置。
7. `SatUtMac` 會丟棄 stale TBTP messages，而不是終止整個 run。

預期新增的 satellite helper 檔案：

```text
helper/beam-hopping-manager.cc
helper/beam-hopping-manager.h
helper/ft-filter.cc
helper/ft-filter.h
helper/isl-graph.cc
helper/isl-graph.h
helper/sat-bh-helper.cc
helper/sat-bh-helper.h
helper/sat-bh-metrics.cc
helper/sat-bh-metrics.h
helper/sat-bh-obc.cc
helper/sat-bh-obc.h
helper/sat-bh-precoder.cc
helper/sat-bh-precoder.h
helper/sat-bh-resource-manager.cc
helper/sat-bh-resource-manager.h
helper/sat-bh-scheduler.cc
helper/sat-bh-scheduler.h
helper/sat-bh-time-plan.cc
helper/sat-bh-time-plan.h
helper/sat-bh-user-associator.cc
helper/sat-bh-user-associator.h
helper/sat-constellation-params.h
helper/sat-dynamic-bstp-provider.h
helper/sat-greedy-bstp-provider.cc
helper/sat-greedy-bstp-provider.h
helper/sat-gw-cache-queue.cc
helper/sat-gw-cache-queue.h
helper/sat-l1-routing-interface.cc
helper/sat-l1-routing-interface.h
helper/sat-power-allocator.cc
helper/sat-power-allocator.h
```

32 個新的 satellite helper 檔案都已列在
`tri_exp/contribsatellite/CMakeLists.txt` 中，因此當該 CMakeLists 被安裝成
`contrib/satellite/CMakeLists.txt` 後，這些檔案預期會被編進 `satellite` contrib
library。

預期安裝的 scratch 檔案：

```text
scratch/bh_dynamic/CMakeLists.txt
scratch/bh_dynamic/Codes/CMakeLists.txt
scratch/bh_dynamic/Codes/README.md
scratch/bh_dynamic/Codes/sat-bh-2d-footprint.cc
scratch/bh_dynamic/Codes/sat-bh-example.cc
scratch/bh_dynamic/Codes/sat-constellation-params.h
```

## preflight checklist

複製檔案前先執行以下檢查：

```bash
test -d /path/to/ns-3.43
test -x /path/to/ns-3.43/ns3
test -d /path/to/ns-3.43/contrib/satellite
test -d /path/to/ns-3.43/contrib/satellite/data
test -d /path/to/ns-3.43/contrib/satellite/data/scenarios/constellation-starlink-1584-sats

test -d /path/to/tri_exp/model
test -d /path/to/tri_exp/helper
test -f /path/to/tri_exp/contribsatellite/CMakeLists.txt
test -f /path/to/tri_exp/scratch/bh_dynamic/CMakeLists.txt
test -f /path/to/tri_exp/scratch/bh_dynamic/Codes/CMakeLists.txt
test -f /path/to/tri_exp/scratch/bh_dynamic/Codes/sat-bh-example.cc
test -f /path/to/tri_exp/scratch/bh_dynamic/Codes/sat-bh-2d-footprint.cc

gcc --version
cmake --version
python3 --version
```

如果任何 `test` 指令失敗，請停止並先修正缺失路徑，再繼續。

## Build 驗證目標

部署後，最小驗證流程：

```bash
cd /path/to/ns-3.43
./ns3 configure --enable-examples --enable-tests
./ns3 build
./ns3 build bh_dynamic/Codes/sat-bh-example
./ns3 run "bh_dynamic/Codes/sat-bh-example --scenario=starlink25"
```

第二個可選 smoke test：

```bash
./ns3 build bh_dynamic/Codes/sat-bh-2d-footprint
./ns3 run "bh_dynamic/Codes/sat-bh-2d-footprint --simTime=0"
```

2D footprint smoke test 依賴 ns-3.43 satellite scenario
`constellation-iridium-next-66-sats`。較舊的名稱 `constellation-iridium-66-sats`
不存在於已確認的本機 `contrib/satellite/data/scenarios` 目錄中，會在 run time
abort。

installation 驗證時，請用 `--simTime=0` 以 geometry-only mode 執行 2D footprint
example。完整模擬，例如 `--simTime=60`，目前需要額外調整 Iridium scenario 的
frame/bandwidth 配置；否則 SNS3 可能以
`Bandwidth of super frame exceeds allocated bandwidth` abort。
## 已知 portability risks

最終 installation SOP 必須明確處理以下事項：

1. 必須將 `tri_exp/contribsatellite/CMakeLists.txt` 安裝到
   `contrib/satellite/CMakeLists.txt`。
2. 既有 `build/` 與 `cmake-cache/` 可能含有 stale absolute paths。使用 `cp -a`
   複製 ns-3 tree 後，這點尤其重要。
3. run time 需要 `contrib/satellite/data`，而 `tri_exp` 不會取代該資料目錄。
4. `sat-bh-example.cc` 的 `starlink25` option 會映射到 satellite data scenario
   `constellation-starlink-1584-sats`。
5. 2D footprint example 必須使用 `constellation-iridium-next-66-sats`，不是
   `constellation-iridium-66-sats`。它的 installation smoke test 應使用
   `--simTime=0`；完整 simulation mode 需要另外調整 bandwidth/frame。
6. `scratch/bh_dynamic/Codes/README.md` 有提到 `run-starlink25.sh`，但該 script
   目前沒有包含在 `tri_exp` 中。
7. beam-hopping helper implementation 的 source of truth 位於
   `contrib/satellite/helper/`；不要把 scratch copy 當成主要修改來源。
8. 更新後的 satellite `CMakeLists.txt` 包含 beam-hopping helper source 與 header
   files。如果仍安裝舊 CMake file，預期會出現 build 或 link failure。

## 目前狀態

這份文件目前包含已確認的 preflight 條件、最小部署形狀、manifest-driven
installation，以及成功的 clean-room build/run 驗證。正式套用到非測試用 ns-3.43
工作目錄前，rollback procedure 仍需要最後定稿。
