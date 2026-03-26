---
name: Layer 1 Frequently Modified Files and Bug Patterns
description: Common files changed in Layer 1 (ISL Routing / Topology) and recurring 現象→原因 patterns
type: project
---

Frequently modified files in Layer 1 (`Topology & ISL Routing/Codes/`):
- `v5_isl-graph.h` / `v5_isl-graph.cc` — core ISL routing logic, OOP refactors, arbiter lifecycle
- `v5_test-iridium.cc` — verification scripts using OOP API

Recurring 現象→原因 patterns:
- ISL eligible count = 0 at t=0 → distance threshold too strict (resolved at 5000 km, DEC-002)
- `HasSignificantChange` always NO → no UDP traffic flowing; `UpdateLoadCosts()` reads zero queue delay
- `ApplyRoutingTable` slow / memory pressure → `CreateObject` called per slot; fix: pre-create arbiter in `InitOrbiterDevices()` and use `ClearNextHopEntries()` (DEC-003)
- `ifIndex` mismatch → device mount order differs from `isls.txt` order; fix: use `peerNodeIdToIfIdx`

Active DEC records (as of 2026-03-25):
- DEC-001: Arbiter mechanism replaces IP layer routing
- DEC-002: ISL Distance Threshold (5000 km)
- DEC-003: Arbiter lifecycle management (pre-creation vs. scheduled creation)
- DEC-004: Wall time attributed to SNS3 DVB MAC beam scheduler, not ISL logic

**Why:** Tracking these prevents re-investigating known root causes in future logs.
**How to apply:** When a new 現象 resembles a known pattern, reference the existing DEC and note the fix is already applied.
