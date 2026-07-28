#ifndef INTER_DRONE_AVOIDANCE_H
#define INTER_DRONE_AVOIDANCE_H

/*
 * Inter-drone horizontal separation (firmware).
 *
 * INTER_DRONE_HARD_SEP_M (R_hard): physical safety floor — mid-air collision limit.
 * R_soft (MissionSimConfig.min_separation_soft_m) is RL shaping only; not used here.
 *
 * Peer positions: kafenv.n.devices[].position (x,y) from UWB ranging (COM_RANGING_* in
 * firmware.cpp) and/or COM_REPLY_ST_EST / COM_REPLY_POS / COM_RET_STATE (command_task.cpp).
 * Requires fresh lastSeen within NETWORK_DEVICE_TIMEOUT.
 *
 * Priority: COM_KILL / NULL_MODE (motors off) > safety pilot (recent COM_SET_MOTOR_CMD)
 * > this repulsion > autonomous setpoints.
 */

#define INTER_DRONE_HARD_SEP_M 1.5f
#define PILOT_MOTOR_OVERRIDE_MS 400UL

void applyInterDroneHardSeparation( void );

#endif
