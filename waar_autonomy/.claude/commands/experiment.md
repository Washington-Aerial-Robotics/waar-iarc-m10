---
name: experiment
description: Set up a parameter sweep experiment for Phase 6 tuning. Use when running systematic sweeps over UC4 weights, inflation margins, or certification thresholds.
---

Set up a structured parameter sweep. Ask the user for parameters, ranges, seeds, and maps if not specified.

## Steps

1. Read `src/experiments/config.py` and `src/experiments/runner.py`.
2. Create `experiments/sweep_<parameter_name>.py`:
   - Generate all config combinations with `itertools.product`
   - Run each config with N seeds via `ExperimentRunner`
   - Collect per-run: `time_to_first_corridor`, `corridor_width_m`, `corridor_length_m`, `corridor_confidence`
   - Aggregate: mean ± std per config
   - Save to `results/sweep_<parameter_name>_<timestamp>.json`
   - Use `pathlib.Path` for all paths (no hardcoded strings)
   - Always log seed and config for reproducibility
3. Create `experiments/analyze_sweep_<parameter_name>.py`:
   - Load results JSON
   - Output: ranked configs by primary metric, sensitivity table, best/worst summary
   - Save summary PNG to `results/`
4. Run a smoke test (2 seeds × 2 configs) to verify the pipeline.

## Output

```
## Experiment Setup: <n>
- Sweep script: experiments/sweep_<n>.py
- Analysis script: experiments/analyze_sweep_<n>.py
- Parameters swept: <list with ranges>
- Total configurations: <N>
- Seeds per config: <N>
- Total runs: <N>
- Primary metric: <metric>
- Smoke test: PASSED / FAILED
```

## Constraints
- Experiments are entry points — may import from all layers
- Never modify `src/` from experiment scripts (read-only)
- Results must be reproducible: always persist seed + full config
