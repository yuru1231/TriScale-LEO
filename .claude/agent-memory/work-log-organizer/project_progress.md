---
name: Project Progress State as of 2026-03-26
description: Current development state per layer and known next steps
type: project
---

Layer 1 (ISL Routing / Topology) — Status: v5 complete, archived, and documented
- v5 OOP (`IslRoutingManager`) fully implemented and passing all verification items
- Dynamic routing with UDP traffic confirmed working
- FtVisibilityFilter integrated and correct across 12 time slots
- Pre-computed routing tables: 12 slots, SAT0_routes=65, PrecomputeAllTables wall=5ms
- ApplyRoutingTable: ≤5ms per slot using pre-created arbiters (DEC-003)
- All v3/v4/v5 code versions archived to `Topology & ISL Routing/Codes/`
- ft-filter.cc/.h archived to Layer 1 codes path
- Layer1.md complete architecture document created (2026-03-26)
- Active DEC records: DEC-001, DEC-002, DEC-003, DEC-004

Layer 2 (Beam Hopping Controller) — Status: module built, standalone validation planned
- BeamHoppingManager background run confirmed (64 BH events, 66 satellites, no ISL interference)
- beam-hopping-manager.cc/.h archived to both Layer 1 (static tools) and Layer 2 (main impl)
- Layer2.md created as draft (content to be filled)
- Next step (2026-03-27): confirm SNS3 BH inject API (TODO SNS3_BH_INJECT), build standalone BH_test.cc

Layer 3 (QoS-Aware Packet Scheduler) — Status: not yet started
- Layer3.md placeholder created (2026-03-26)
- Will depend on BeamHoppingManager::GetCurrentCell() for beam-aware scheduling

E2E — Status: v1_e2e-iridium.cc drafted and archived, NOT finalized or integrated
- Architecture kept strictly layered: Layer 1 and Layer 2 developed and verified independently
- v1 output archived to E2E/Outputs/v1_output.md

**Why:** Tracking layer status avoids accidentally treating unfinished work as complete.
**How to apply:** When writing future logs, check this memory to set correct 目標 framing and 明日計畫 continuity.
