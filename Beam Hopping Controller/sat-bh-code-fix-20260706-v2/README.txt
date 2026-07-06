sat-bh-code-fix-20260706-v2

This package contains the current BH example code changes:

1. scratch/bh_dynamic/Codes/sat-bh-example.cc
   - reads config from sat-bh-example.txt by default
   - supports .conf -> .txt fallback for old --configFile paths
   - supports outputDir
   - writes actual remaining demand CSV from PacketSink::Rx throughput
   - prints FWD PDR summary

2. scratch/bh_dynamic/Codes/sat-bh-example.txt
   - key=value input config file replacing sat-bh-example.conf

3. contrib/satellite/helper/sat-bh-helper.cc
4. contrib/satellite/helper/sat-bh-helper.h
   - writes scheduler-level SERVICE_ACCOUNTING rows
   - adds served_kbps, remaining_kbps, remaining_pct columns to BH traffic trace

5. doc/sat-bh-demand-remaining.md
   - Chinese documentation for actual remaining demand calculation

6. doc/sat-bh-example-parameters.md
   - parameter documentation updated to .txt config

Run example:

./ns3 run "scratch/bh_dynamic/Codes/sat-bh-example --configFile=scratch/bh_dynamic/Codes/sat-bh-example.txt"

Suggested validation config values:

simTime=11
warmUp=1
enableObc=1
enableDynamicBstp=1
fwdOfferedDemandKbps=1000
maxHelperSats=1
satIdStart=498
outputDir=/tmp/bh-example-full-test
