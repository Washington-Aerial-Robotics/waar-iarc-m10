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

//kafenv.info.sensorStatus bits - pre-flight sensor health, recomputed every commander_step() tick and
//exposed via the existing COM_REQUEST_INFO telemetry (see kaf_drone.h). Independent of estimation_positionValid()/
//gps_isFixGood()'s use in mode-entry guards - this is purely a display aid so an operator can see WHICH
//sensor is the problem before ever attempting a launch, rather than a bare "position invalid" rejection.
#define SENSOR_STATUS_GPS  0b0001
#define SENSOR_STATUS_BARO 0b0010
#define SENSOR_STATUS_IMU  0b0100
#define SENSOR_STATUS_MAG  0b1000

//kafenv.info.autonomyMode - which autonomy behavior commander_step() runs. Orthogonal to flightMode.
#define AUTONOMY_MANUAL        0
#define AUTONOMY_QUALIFICATION 1
#define AUTONOMY_MINE_SEARCH   2
#define AUTONOMY_SQUARE_TEST   3 //GPS+BME position-hold validation maneuver - NOT part of the rules-mandated
                                  //qualification behavior (which requires a circle, not a square) - a
                                  //separate, simpler pattern for exercising GPS x/y + BME z position control
                                  //before ever attempting the real orbit.

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

//kafenv.info.squareState - Square Test state machine (commander.cpp's commander_squareStep()). x/y come
//from GPS (via kafenv.state.x, fed by estimate_position() - see estimation.cpp), z from BME280 (same
//estimator, preferred over GPS's own noisier altitude). Entirely onboard, same reasoning as Qualification.
#define SQUARE_BOOT      0 //waiting for a valid position estimate before START is accepted
#define SQUARE_CLIMB     1 //FLIGHTPATH_LAUNCH in progress, climbing to SQUARE_ALTITUDE_M
#define SQUARE_LEG1      2 //FLIGHTPATH_GLIDEPOINT to corner (S, 0)
#define SQUARE_LEG2      3 //FLIGHTPATH_GLIDEPOINT to corner (S, S)
#define SQUARE_LEG3      4 //FLIGHTPATH_GLIDEPOINT to corner (0, S)
#define SQUARE_LEG4      5 //FLIGHTPATH_GLIDEPOINT back to the launch origin (0, 0)
#define SQUARE_LANDING   6 //FLIGHTPATH_LAND in progress
#define SQUARE_FINISH    7 //landed, actuation disabled - terminal

//High-level commands accepted only while autonomyMode == AUTONOMY_SQUARE_TEST.
#define SQUARECMD_START  1
#define SQUARECMD_LAND   2 //higher priority than START - accepted from any in-progress SQUARE_* state
#define SQUARECMD_ABORT  3 //same priority as SQUARECMD_LAND, immediate FLIGHTPATH_LAND

#define SQUARE_ALTITUDE_M          6.0F  //20ft, same rounding convention as QUAL_HOVER_ALTITUDE_M
#define SQUARE_SIDE_LENGTH_M       5.0F  //adjust to taste - keep well within your test area's GPS-clear space
#define SQUARE_GLIDE_VELOCITY_M_S  1.0F  //conservative first-attempt speed
#define SQUARE_POSITION_TOLERANCE_M 0.5F
#define SQUARE_POSITION_DWELL_MS  500UL  //time within tolerance before a leg/climb is considered arrived
#define SQUARE_LEG_TIMEOUT_MS     30000UL //safety: if a leg doesn't complete in this long (bad tuning, GPS
                                           //drift, physical obstruction), land rather than hold forever -
                                           //a gap the current Qualification CLIMB_TO_FORMATION state does
                                           //NOT yet have; worth retrofitting there too, not done here.

void commander_setTrajectories( STDBYTE mode, const float args[4] );
void commander_qualificationCommand( STDBYTE cmd );
void commander_squareCommand( STDBYTE cmd );

peripheral commander_reset();
void commander_step( const unsigned long currentTime );