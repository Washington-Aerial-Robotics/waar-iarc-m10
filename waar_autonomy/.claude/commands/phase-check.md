---
name: phase-check
description: Audit codebase against current phase deliverables and DoD. Use when checking completion status, before ending a session, or when asked about project progress.
---

Read `docs/dev_roadmap.md` and `docs/system_design.md`, then scan the codebase to produce a structured completion report for the **current active phase**.

## Steps

1. Read `docs/dev_roadmap.md` to identify the active phase and its deliverables.
2. Read `docs/system_design.md` for the canonical spec of each deliverable.
3. Scan the relevant source files under `src/` and `tests/`.
4. Output the report below. Only mark DONE if the implementation matches the spec.

## Output Format

```
## Phase Check Report — Phase <N>: <Phase Title>

### Deliverables
| Item | Status | File | Notes |
|------|--------|------|-------|
| <entity/UC name> | ✅ DONE / ⚠️ PARTIAL / ❌ MISSING | <file> | <gap> |

### Test Coverage
| Test | Status | File |
|------|--------|------|
| <test from roadmap> | ✅ EXISTS / ❌ MISSING | <file or —> |

### Metrics Instrumentation
| Metric | Logged? | Where |
|--------|---------|-------|
| time_to_first_corridor | ✅ / ❌ | |
| corridor_width_m | ✅ / ❌ | |
| corridor_length_m | ✅ / ❌ | |
| corridor_confidence | ✅ / ❌ | |
| redundancy_ratio | ✅ / ❌ | |

### TBD Violations
Any deferred items (from CLAUDE.md "Known TBDs") that have been accidentally implemented:
- <item> in <file> — recommend: revert or flag as Design Delta

### Design Delta Flags
Implementations that deviate from system_design.md spec:
- <description> — severity: LOW / MEDIUM / HIGH

### Summary
- Completion: <N>/<total> deliverables done
- Blocking issues: <list or "none">
- Recommended next action: <one sentence>
```

Report only — do not fix issues unless explicitly instructed.
