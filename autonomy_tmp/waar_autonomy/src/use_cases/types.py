from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CertificationResult:
    certified: bool
    clearance: float
    coverage:  float
