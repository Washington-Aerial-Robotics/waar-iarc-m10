#include "../core/kaf_drone.h"

//How long a motion-capture position stays trusted after it was received (this device's millis(), not the
//sender's clock). Vicon streams at ~100 Hz, so this tolerates ~15 dropped/occluded frames of WiFi jitter
//before estimation_positionValid() goes false and commander_step()'s existing failsafe lands the drone.
#define MOCAP_STALE_MS 150UL
//If no packet has arrived for this long, the next packet is accepted regardless of its sequence number -
//lets vicon_link.py be restarted (sequence back to 0) without power-cycling the drone.
#define MOCAP_RESTART_MS 1000UL

//Registers the COM_SET_MOCAP handler and marks motion capture as the active position source - only call
//this from a sketch that also passes mocap_step to flight_positionEstimator() (see firmware_vicon.ino).
peripheral mocap_reset();
//Position estimator for flight_positionEstimator(): returns the latest motion-capture position while it is
//fresh (<= MOCAP_STALE_MS old), false otherwise so flight.cpp's positionEstimate() keeps dead-reckoning.
bool mocap_step( coordinate* estimate );
//True once mocap_reset() has run - estimation_positionValid() then defers to mocap_positionValid().
bool mocap_isActive();
//True when a motion-capture position was received within MOCAP_STALE_MS.
bool mocap_positionValid();
//Latest received yaw (rad, kafenv.state.q.z convention) - NAN if never received or not sent. Stored for
//telemetry/future fusion only; the magnetometer still drives kafenv.state.q.z.
float mocap_yaw();
