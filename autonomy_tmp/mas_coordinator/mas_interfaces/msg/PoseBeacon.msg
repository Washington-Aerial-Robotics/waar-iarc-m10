# PoseBeacon.msg
# Broadcast by every drone at ~5 Hz so neighbors can maintain a neighbor graph

string  drone_id
float64 x
float64 y
float64 z
float32 heading_deg     # yaw, degrees
string  state           # current mission state: BOOT|SURVEY|VERIFY_TAG|PATH_VERIFY|CONVERGE|FINISH
float32 battery_pct     # [0.0, 100.0]
builtin_interfaces/Time stamp
