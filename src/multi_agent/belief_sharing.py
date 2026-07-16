from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from application.simulator import Simulator

simulators: list["Simulator"]

class BeliefSharing:
    """
    Shared-belief synchronization.

    Current model:
        perfect communication
        instantaneous synchronization
    """

    def synchronize(
        self,
        simulators: list[Simulator],
    ) -> None:

        if len(simulators) == 0:
            return

        shape = (
            simulators[0]
            .world
            .detected
            .shape
        )

        fused = np.zeros(
            shape,
            dtype=np.int8,
        )

        for sim in simulators:

            detected = sim.world.detected

            known = detected != 0

            fused[known] = detected[known]

        for sim in simulators:

            sim.world.detected[:, :] = fused