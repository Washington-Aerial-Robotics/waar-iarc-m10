---
name: design-delta
description: Output a formal Design Delta note before making cross-layer changes or resolving TBD items. Use when a change affects multiple layers or deviates from system_design.md.
---

Output a formal Design Delta note for a change that crosses layer boundaries, deviates from `docs/system_design.md`, or resolves a TBD item.

**Do not write any code until the user confirms the delta** (unless it's a clearly local fix with no cross-layer impact).

## Steps

1. Read the relevant section of `docs/system_design.md`.
2. Identify what is changing and why.
3. Output the delta in the format below.
4. Ask: "Shall I proceed with implementation?"

## Output Format

```markdown
## Design Delta: <short title>

**Trigger:** <bug / new requirement / TBD resolution / performance>

### What Changed
- Files: <list>
- Interfaces: <port IDs, UC IDs, entity names>
- Behaviour: <before → after>

### Why
<Engineering justification — reference metrics or constraints>

### Layer Impact
| Layer | Impact |
|-------|--------|
| domain/ | none / modified: <what> |
| use_cases/ | none / modified: <what> |
| ports/ | none / modified: <what> |
| adapters/ | none / modified: <what> |
| application/ | none / modified: <what> |

### TBD Status
- Resolves deferred item? yes / no
- Which TBD from dev_roadmap.md: <description or N/A>
- Update CLAUDE.md "Known TBDs" after approval: yes / no

### Risks & Mitigations
- <risk>: <mitigation>

### Required Follow-up
- [ ] Update docs/system_design.md section <N>
- [ ] Update CLAUDE.md Current Development State
- [ ] Add/update tests: <which>
- [ ] Update docs/dev_roadmap.md if phase scope changed
```
