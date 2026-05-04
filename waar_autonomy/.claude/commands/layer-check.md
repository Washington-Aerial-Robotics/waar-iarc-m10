---
name: layer-check
description: Scan for cross-layer import violations in Clean Architecture. Use when suspecting architectural drift, after a large refactor, or before a PR review.
---

Scan `src/` for cross-layer import violations per the Hard Constraints in `CLAUDE.md`.

## Allowed Import Matrix

```
domain/       → stdlib only (no project imports)
use_cases/    → domain/ only
ports/        → domain/ only
adapters/     → domain/, use_cases/, ports/ only
application/  → all layers (orchestration)
infrastructure/ → domain/ only
experiments/  → all layers (entry point)
analysis/     → domain/, use_cases/ only
```

## What to flag

For each `.py` file under `src/`:
1. Extract all `import` and `from ... import` statements
2. Map each to its layer by path
3. Check against the matrix above
4. Also flag:
   - Any import from `src/_archive/`
   - `ros`, `rclpy`, `asyncio`, `threading` in `domain/` or `use_cases/`
   - `open()`, `print()`, direct `logging` calls in `domain/`

## Output Format

```
## Layer Check Report
Scanned: <N> files

### Violations
| File | Import | Violation | Severity |
|------|--------|-----------|----------|
| src/use_cases/foo.py | from adapters.bar import X | use_cases→adapters (forbidden) | HIGH |

### Warnings
| File | Note |
|------|------|

### Clean Layers
<list of layers with zero violations>

### Summary
- Total violations: <N>
- HIGH severity: <N>
- Action: <fix priority order or "none">
```

Do not fix violations — report only.
