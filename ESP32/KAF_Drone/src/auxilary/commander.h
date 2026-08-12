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

void commander_setTrajectories( STDBYTE mode, const float args[4] );

peripheral commander_reset();
void commander_step( const unsigned long currentTime );