# WAAR IARC M10 / Vicon

ESP32 flight control, Vicon lab tools, a single-drone autonomy simulation,
and experimental ROS2 team coordination.

**Start here for one physical drone with an onboard Raspberry Pi 4/5:**
[Single-drone bring-up and Pi image guide](deploy/raspberry-pi/README.md).

The ESP32 runs the flight controller. The Pi companion receives Vicon poses
relayed from the lab PC and forwards them to the ESP32. This does not yet run
the simulated corridor planner on real hardware or implement a complete
autonomous mission.

Software checks (no drone, Vicon, or ROS2 needed):

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-test.txt
.\.venv\Scripts\python.exe scripts/check_software.py
```

See also [Vicon setup](ESP32/KAF_Drone/tests/vicon/README.md),
[flight-controller setup](ESP32/KAF_Drone/README.md),
[planner simulation](Autonomy/waar_autonomy/README.md), and
[ROS2 coordinator](Autonomy/mas_coordinator/README.md).
