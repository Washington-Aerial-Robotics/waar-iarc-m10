---
name: new-entity
description: Create a new domain entity in src/domain/ following Clean Architecture. Use when adding entities specified in system_design.md section 4.1.
---

Create a new domain entity in `src/domain/`. Check `docs/system_design.md` section 4.1 for the canonical spec first.

## Steps

1. Read the entity spec from `docs/system_design.md` (fields, invariants, queries).
2. Create `src/domain/<entity_name>.py`:
   - `@dataclass` for plain data, class for rich behaviour
   - Full type annotations
   - Docstring referencing the entity ID (e.g., "E1 Block")
   - **Zero imports** from `use_cases/`, `application/`, `adapters/`, `infrastructure/`
   - No I/O, no side effects, no framework dependencies
   - TBD fields → implement baseline only, add `# TBD: <description>` comment
3. Create `tests/domain/test_<entity_name>.py`:
   - ≥3 unit tests: instantiation, invariants, key methods
   - Deterministic toy inputs only
4. Run `pytest tests/domain/test_<entity_name>.py -v` — fix until green.
5. Run `black` and `isort` on the new file.

## Output

```
## New Entity: <Name>
- File: src/domain/<n>.py
- Test: tests/domain/test_<n>.py
- Tests passing: <N>/<N>
- system_design.md ref: <entity ID>
- TBDs left as comments: <list or none>
- Deviations from spec: <none / description>
```

## Hard Constraints
- ZERO upward imports (domain is the bottom layer)
- No serialization methods (to_dict, to_json) — those belong in adapters
- Do not implement TBD fields unless explicitly requested
