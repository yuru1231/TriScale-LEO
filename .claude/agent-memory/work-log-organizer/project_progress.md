---
name: Project Progress State as of 2026-03-27
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

Layer 2 (Beam Hopping Controller) — Status: Phase 1 fully implemented and verified (2026-03-27)
- Phase 1 core modules all implemented and passing e2e simulation (300s, no crash):
  - sat-bh-time-plan.h/.cc: BHTP data model, Validate/PrettyPrint/ToCsv
  - sat-bh-metrics.h/.cc: KPI collection, T_p=503ms periodic CSV flush, FinalFlush
  - sat-bh-helper.h/.cc: BhExperimentConfig (OOP, no hard-code), feature flag arch
- Phase 2 stubs complete (interface ready, impl pending):
  - sat-bh-scheduler.h/.cc: EM algorithm skeleton (E-step/M-step, cluster grouping)
  - sat-bh-obc.h/.cc: OBC state machine (IDLE/ACTIVE/SWITCHING/WAIT_PLAN)
- Phase 3 stubs complete (interface ready, impl pending):
  - sat-gw-cache-queue.h/.cc: Q_max=40MB, TAIL_DROP policy
  - sat-bh-precoder.h/.cc: MMSE W=H^H(HH^H+σ²I)^{-1}, Cholesky
- sat-bh-example.cc: unified example script, verified Phase 1 default run
- Static BHTP output: 19 slots, 503ms period, K=2, beam dwell correct
- Outputs: bh-metrics.csv, bh-timeplan.csv, bh-attributes.xml confirmed generated
- Layer2.md: draft exists, content fill still needed
- Next step (2026-03-27 afternoon): Phase 2 SatBhScheduler EM implementation; confirm SNS3 hook availability (GwMac::Tx trace, ChannelEstimation trace)

Layer 3 (QoS-Aware Packet Scheduler) — Status: not yet started
- Layer3.md placeholder created (2026-03-26)
- Will depend on BeamHoppingManager::GetCurrentCell() for beam-aware scheduling

E2E — Status: v1_e2e-iridium.cc drafted and archived, NOT finalized or integrated
- Architecture kept strictly layered: Layer 1 and Layer 2 developed and verified independently
- v1 output archived to E2E/Outputs/v1_output.md

**Why:** Tracking layer status avoids accidentally treating unfinished work as complete.
**How to apply:** When writing future logs, check this memory to set correct 目標 framing and 明日計畫 continuity.
