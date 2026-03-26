---
name: Work Log Style Preferences
description: User's preferred verbosity and structure for work logs
type: feedback
---

Log header format must be `# 工作日誌 YYYY-MM-DD` (not a title like "# Verify Dynamic Routing ...").

Each completed item (完成事項) requires all four sub-fields: 現象, 原因, 修正, 驗證 — even for pure verification items with no code change (write "無需修正，驗證既有實作" in 修正).

修正 field: include key function names and parameter names; no shell commands or execution steps.

驗證 field: include observable output lines (log lines, numeric values, table rows) — not just "works correctly".

修改的檔案 table: always use full absolute Windows paths (`C:\Users\wenj\Desktop\TriScale-LEO\...`).

A 驗證結果總表 section (separate from individual items) is appropriate when there are 5+ verification items — use it as a summary checklist.

**Why:** The user provided a draft with these structural gaps and the completed log filled them in without pushback.
**How to apply:** Always self-check all four sub-fields are non-empty before writing the final log.
