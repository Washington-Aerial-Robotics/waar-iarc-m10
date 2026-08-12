#include "../core/kaf_drone.h"

#define CMD_MODE_MASK    0b11111000
#define CMD_IDLE_MODE             0 //FLASH
#define CMD_NOMINAL_MODE          8 //FLASH STABILITY DISCONNECT BATTERY 
#define CMD_NULL_MODE            16 //
#define CMD_DESCENT_MODE         24 //FLASH STABILITY                    DESCENT

#define FLIGHTPATH_NONE           0
#define FLIGHTPATH_LAUNCH         1
#define FLIGHTPATH_LAND           2
#define FLIGHTPATH_POSLOCK        3
#define FLIGHTPATH_GLIDEPOINT     4
#define FLIGHTPATH_RETURNHOME     5
#define FLIGHTPATH_CIRCLE         6

//How long a POS_SETPOINT_MODE/TRAJECTORY_MODE setpoint (kafenv.cmd.setpointMillis, set by COM_SET_TRAJSETPT)
//may go without a fresh update before commander_step() treats it as lost communication with the Pi and
//falls back to FLIGHTPATH_LAND - the same emergency-descent path already used for ground-station disconnect
//and low battery. Deliberately short relative to those (Pi setpoints are expected at well over 1Hz).
#define SETPOINT_STALE_MS 500UL

//kafenv.info.autonomyMode - which autonomy behavior commander_step() runs. Orthogonal to flightMode.
#define AUTONOMY_MANUAL        0
#define AUTONOMY_QUALIFICATION 1
#define AUTONOMY_MINE_SEARCH   2

//kafenv.info.qualState - Qualification state machine states (commander.cpp's commander_qualificationStep()).
//Runs entirely onboard the ESP32: launch/hover/orbit/hold/land all reuse the existing FLIGHTPATH_* trajectory
//primitives above, driven by explicit QUALCMD_* commands rather than a continuously-streamed Pi trajectory -
//see docs/qualification-uwb-positioning.md's sibling design note for why this differs from Mine Search,
//which still needs the Pi's SLAM/planning stack and COM_SET_TRAJSETPT.
#define QUAL_BOOT                0 //waiting for a valid position estimate + formation slot before LAUNCH is accepted
#define QUAL_CLIMB_TO_FORMATION  1 //FLIGHTPATH_LAUNCH in progress, climbing to QUAL_HOVER_ALTITUDE_M
#define QUAL_HOVER_HOLD          2 //FLIGHTPATH_POSLOCK, holding formation position, waiting for BEGIN_ORBIT
#define QUAL_ORBIT               3 //FLIGHTPATH_CIRCLE, periodically re-issued, counting revolutions
#define QUAL_POST_ORBIT_HOLD     4 //FLIGHTPATH_POSLOCK after 10 revolutions or an early HOLD command
#define QUAL_LANDING             5 //FLIGHTPATH_LAND in progress
#define QUAL_FINISH              6 //landed, actuation disabled - terminal

//High-level commands accepted only while kafenv.info.autonomyMode == AUTONOMY_QUALIFICATION - deliberately a
//small, fixed vocabulary (COM_SET_QUALCOMMAND carries one of these as a single byte, no fuzzy text matching
//on this firmware side; any natural-language recognition belongs on the phone, upstream of this message).
#define QUALCMD_LAUNCH       1
#define QUALCMD_BEGIN_ORBIT  2
#define QUALCMD_HOLD         3
#define QUALCMD_LAND         4 //higher priority than the above - accepted from any QUAL_* state
#define QUALCMD_ABORT        5 //same priority as QUALCMD_LAND, immediate FLIGHTPATH_LAND

#define QUAL_HOVER_ALTITUDE_M     6.0F  //20ft
#define QUAL_ORBIT_RADIUS_M       5.0F  //~10m diameter
#define QUAL_ORBIT_ANGULAR_RATE   0.3F  //rad/s - ~21s per revolution, conservative for a first attempt
#define QUAL_ORBIT_REVOLUTIONS_TARGET 10
#define QUAL_CIRCLE_REISSUE_MS    1000UL //how often FLIGHTPATH_CIRCLE is re-issued during ORBIT to keep the
                                          //4th-order polynomial approximation from drifting off the true
                                          //circle - the trajectory math is only accurate near the point it
                                          //was last centered on (current position at time of issue), not
                                          //over a full multi-revolution duration.
#define QUAL_FORMATION_SPACING_M  3.0F  //10ft between adjacent drones' hover positions along the line
#define QUAL_ALTITUDE_TOLERANCE_M 0.3F
#define QUAL_ALTITUDE_DWELL_MS    1000UL //time within tolerance before CLIMB_TO_FORMATION is considered done

void commander_setTrajectories( STDBYTE mode, const float args[4] );
void commander_qualificationCommand( STDBYTE cmd );

peripheral commander_reset();
void commander_step( const unsigned long currentTime );