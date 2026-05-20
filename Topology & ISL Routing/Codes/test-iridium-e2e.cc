// Canonical entry point for the current Iridium E2E test implementation.
// The validated gw2gw_e2e fixes live in test-iridium-e2e-fix.cc, including:
// - tagged UDP sender for delay measurement (avoids OnOffHelper PacketTag limitation)
// - feeder-satellite selection from rtnConf.txt
// - per-slot GW host-route / ARP refresh when entry/exit satellites change
//
// Keep this wrapper so existing build/run commands that target test-iridium-e2e
// continue to work without reviving the deprecated OnOff-based path.
#include "test-iridium-e2e-fix.cc"
