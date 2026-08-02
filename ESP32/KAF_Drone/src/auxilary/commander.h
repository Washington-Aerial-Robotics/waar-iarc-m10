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

void commander_setTrajectories( STDBYTE mode, const float args[4] );

peripheral commander_reset();
void commander_step( const unsigned long currentTime );