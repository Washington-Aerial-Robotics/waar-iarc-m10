---
name: new-adapter
description: Create a new Adapter (concrete port implementation) in src/adapters/. Use when implementing a Port interface for sim, ROS2, or test contexts.
---

Create a new Adapter in `src/adapters/`. Read the target Port interface from `src/ports/` and `docs/system_design.md` section 4.3 first.

## Steps

1. Read `src/ports/<port_name>.py` for the abstract contract.
2. Create `src/adapters/<context>_<port_name>_adapter.py`:
   - Inherit from the Port ABC, implement all abstract methods
   - External system coupling (ROS2, sim, I/O) lives **only here**
   - No business logic — pure translation between external format ↔ domain types
   - Stub methods → `raise NotImplementedError("...")` with clear message
3. Create `tests/adapters/test_<context>_<port_name>_adapter.py`:
   - Use lightweight fakes/fixtures, not real external systems
   - Test: external data → domain types (correct translation)
   - Test: domain commands → external format (correct translation)
4. Run `pytest tests/adapters/...` — fix until green.
5. Register in `src/experiments/default_registry.py` if applicable.

## Output

```
## New Adapter: <n>
- File: src/adapters/<n>.py
- Port implemented: <I1–I4>
- Test: tests/adapters/test_<n>.py
- Tests passing: <N>/<N>
- Registered in registry: yes / no / N/A
- Deviations from port contract: <none / description>
```

## Hard Constraints
- May import from `domain/`, `use_cases/`, `ports/` — never from other `adapters/`
- Business logic found in an adapter → flag as Design Delta, move to a Use Case
- Stub methods must `raise NotImplementedError`, never `pass` silently
