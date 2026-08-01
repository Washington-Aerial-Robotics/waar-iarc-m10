"""
sim.py — entry point for the single-drone corridor simulator.

Delegates to src/experiments/run_sim.py (clean architecture).

Run:
  python sim.py                   # animated
  python sim.py --seed 7
  python sim.py --no-anim         # headless
  python sim.py --delay 0.02      # slow down animation
  python sim.py --hazards 40      # denser hazard field
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from experiments.run_sim import main  # noqa: E402

if __name__ == "__main__":
    main()
