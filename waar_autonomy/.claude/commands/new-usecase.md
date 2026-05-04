---
name: new-usecase
description: Create a new Use Case in src/use_cases/ following Clean Architecture. Use when implementing UC1–UC6 from system_design.md or adding new business rules.
---

Create a new Use Case in `src/use_cases/`. Read `docs/system_design.md` section 4.2 for the canonical spec first.

## Steps

1. Read the UC spec (inputs, outputs, baseline algorithm, TBDs).
2. Create `src/use_cases/<snake_case_name>.py`:
   - Callable class with `__call__` or `execute()` signature
   - Input/output types: domain entities or DTOs from `use_cases/types.py` only
   - Docstring with UC ID (e.g., "UC2 ComputeBestCorridor")
   - Implement baseline algorithm; mark deferred items `# TBD: <description>`
   - **No ROS2, no asyncio, no threading, no I/O**
   - External state (e.g., clock) → accept as parameter, never import directly
3. Create `tests/use_cases/test_<snake_case_name>.py`:
   - Deterministic toy inputs (small crafted maps/states)
   - Cover: happy path, edge cases, boundary conditions
   - Use `pytest.mark.parametrize` for multiple scenarios
4. Run `pytest tests/use_cases/test_<snake_case_name>.py -v` — fix until green.
5. Wire into `src/application/baseline_loop.py` if it belongs in the tick loop.
6. Run `black` and `isort`.

## Output

```
## New Use Case: <n>
- File: src/use_cases/<n>.py
- Test: tests/use_cases/test_<n>.py
- Tests passing: <N>/<N>
- system_design.md ref: <UC ID>
- Wired into baseline_loop: yes / no / N/A
- TBDs left as comments: <list or none>
- Deviations from spec: <none / description + Design Delta if significant>
```

## Hard Constraints
- Only import from `domain/` and `use_cases/types.py`
- Never import from `adapters/`, `application/`, `infrastructure/`
- Do not implement TBD items unless explicitly requested
