#include "../core/firmware.h"
#include "../core/communication.h"
#include "../core/flight.h"
#include "commander.h"
#include "common_data.h"

#if ALT_DEFINE
#define NAN 0.0F
#define isfinite( arg ) true
static void* memset( void* dest, int ch, size_t count );
static void* memcpy( void* dest, const void* src, size_t count );
static float sqrtf( float arg );
#else
#include <string.h>
#include <math.h>
#endif

#define GROUND_STATION_ID 'G'
#define COMMANDER_COM_METHOD 5
#define STORAGE_CAPACITY 0x5000
#define STARTUP_CMD_COUNT 3
#define ATTITUDE_THRESHOLD_TIME 3000 //ms

extern COMS_BUFFERTYPE;

struct commanding {
  struct storage {
    struct startupcmd {
      unsigned char length;
      union {
        packet_header header;
        STDBYTE bytes[127];
      };
    } startupcmds[STARTUP_CMD_COUNT];
    union {
      struct {
        float ascentRate;
        float hoverHeight;
        float descentRate;
        float groundHeight;
        float glideV;
        float glideX;
        float glideY;
        float glideZ;
        float homeX;
        float homeY;
        float circleR;
        float circleW;
      } s;
      float a[ sizeof( s ) / sizeof( float ) ];
    } traj;
  } store;
  bool doReadStorage;
  unsigned long lastTime;
  unsigned long attitudeLimit;
} commander;

struct startupcommand {
  unsigned char index;
  commanding::storage::startupcmd content;
};

void commander_setTrajectories( STDBYTE mode, const float args[4] ) {
  kafenv.info.flightMode = ( kafenv.info.flightMode & ~DEFAULT_MODES_MASK ) | TRAJECTORY_MODE;
  kafenv.info.actuation = true;
  switch( mode ) {
    case FLIGHTPATH_LAUNCH : {
      if( args != NULLPTR ) {
        commander.store.traj.s.hoverHeight = args[0];
        commander.store.traj.s.ascentRate = args[1];
      }
      DPRINTF( "[H] Launch Flight Path: H=%.3f, V=%.3f\n", commander.store.traj.s.hoverHeight, commander.store.traj.s.ascentRate );
      kafenv.cmd.setpoints[ 1] = ( commander.store.traj.s.hoverHeight - kafenv.state.x.z ) / commander.store.traj.s.ascentRate;
      kafenv.cmd.setpoints[ 6] = kafenv.state.x.x;
      kafenv.cmd.setpoints[11] = kafenv.state.x.y;
      kafenv.cmd.setpoints[16] = kafenv.state.x.z;
      kafenv.cmd.setpoints[21] = kafenv.state.q.z;
      kafenv.cmd.setpoints[15] = commander.store.traj.s.ascentRate;
      break;
    }
    case FLIGHTPATH_LAND : {
      if( args != NULLPTR ) {
        commander.store.traj.s.groundHeight = args[0];
        commander.store.traj.s.descentRate = args[1];
      }
      DPRINTF( "[H] Landing Flight Path: H=%.3f, V=%.3f\n", commander.store.traj.s.groundHeight, commander.store.traj.s.descentRate );
      kafenv.cmd.setpoints[ 1] = ( kafenv.state.x.z - commander.store.traj.s.groundHeight ) / commander.store.traj.s.descentRate;
      kafenv.cmd.setpoints[ 6] = kafenv.state.x.x;
      kafenv.cmd.setpoints[11] = kafenv.state.x.y;
      kafenv.cmd.setpoints[16] = kafenv.state.x.z;
      kafenv.cmd.setpoints[21] = kafenv.state.q.z;
      kafenv.cmd.setpoints[15] = -commander.store.traj.s.descentRate;
      break;
    }
    case FLIGHTPATH_POSLOCK : {
      DPRINTF( "[H] Position Lock Flight Path: X=[ %.3f, %.3f, %.3f ]\n", kafenv.state.x.x, kafenv.state.x.y, kafenv.state.x.z );
      kafenv.cmd.setpoints[ 6] = kafenv.state.x.x;
      kafenv.cmd.setpoints[11] = kafenv.state.x.y;
      kafenv.cmd.setpoints[16] = kafenv.state.x.z;
      kafenv.cmd.setpoints[21] = kafenv.state.q.z;
      break;
    }
    case FLIGHTPATH_RETURNHOME : {
      if( args != NULLPTR ) {
        commander.store.traj.s.homeX = args[0];
        commander.store.traj.s.homeY = args[1];
      }
      DPRINTF( "[H] Return Home Flight Path: X=%.3f, Y=%.3f\n", commander.store.traj.s.homeX, commander.store.traj.s.homeY );
      const float dx = commander.store.traj.s.homeX - kafenv.state.x.x;
      const float dy = commander.store.traj.s.homeY - kafenv.state.x.y;
      const float distance = sqrtf( dx * dx + dy * dy );
      kafenv.cmd.setpoints[ 1] =  distance / commander.store.traj.s.glideV;
      kafenv.cmd.setpoints[ 6] = kafenv.state.x.x;
      kafenv.cmd.setpoints[11] = kafenv.state.x.y;
      kafenv.cmd.setpoints[16] = kafenv.state.x.z;
      kafenv.cmd.setpoints[21] = kafenv.state.q.z;
      kafenv.cmd.setpoints[ 5] = dx / distance * commander.store.traj.s.glideV;
      kafenv.cmd.setpoints[10] = dy / distance * commander.store.traj.s.glideV;
      break;
    }
    case FLIGHTPATH_GLIDEPOINT : {
      if( args != NULLPTR ) {
        commander.store.traj.s.glideV = args[0];
        commander.store.traj.s.glideX = args[1];
        commander.store.traj.s.glideY = args[2];
        commander.store.traj.s.glideZ = args[3];
      }
      DPRINTF( "[H] Glide Point Flight Path: V=%.3f, X=[ %.3f, %.3f, %.3f ]\n", commander.store.traj.s.glideV,
          commander.store.traj.s.glideX, commander.store.traj.s.glideY, commander.store.traj.s.glideZ );
      const coordinate diff = { commander.store.traj.s.glideX - kafenv.state.x.x, 
          commander.store.traj.s.glideY - kafenv.state.x.y, commander.store.traj.s.glideZ - kafenv.state.x.z };
      const float distance = sqrtf( diff.x * diff.x + diff.y * diff.y + diff.z * diff.z );
      kafenv.cmd.setpoints[ 1] =  distance / commander.store.traj.s.glideV;
      kafenv.cmd.setpoints[ 6] = kafenv.state.x.x;
      kafenv.cmd.setpoints[11] = kafenv.state.x.y;
      kafenv.cmd.setpoints[16] = kafenv.state.x.z;
      kafenv.cmd.setpoints[21] = kafenv.state.q.z;
      kafenv.cmd.setpoints[ 5] = diff.x / distance * commander.store.traj.s.glideV;
      kafenv.cmd.setpoints[10] = diff.y / distance * commander.store.traj.s.glideV;
      kafenv.cmd.setpoints[15] = diff.z / distance * commander.store.traj.s.glideV;
      break;
    }
    case FLIGHTPATH_CIRCLE : { https://www.desmos.com/calculator/pyhj1cangn
      if( args != NULLPTR ) {
        commander.store.traj.s.circleR = args[0];
        commander.store.traj.s.circleW = args[1];
      }
      DPRINTF( "[H] Circular Flight Path: R=%.3f, W=%.3f\n", commander.store.traj.s.circleR, commander.store.traj.s.circleW );
      const float a   = commander.store.traj.s.circleR;
      const float u   = commander.store.traj.s.circleW / 3.1415926535F;
      const float u1a = u * a;
      const float u2a = u * u1a;
      const float u3a = u * u2a;
      const float u4a = u * u3a;
      kafenv.cmd.setpoints[ 1] = 2 / u;
      kafenv.cmd.setpoints[ 2] = u4a * ( -2.00000000000F );
      kafenv.cmd.setpoints[ 3] = u3a * (  8.00000000000F );
      kafenv.cmd.setpoints[ 4] = u2a * ( -8.00000000000F );
      kafenv.cmd.setpoints[ 6] =   a                      + kafenv.state.x.x;
      kafenv.cmd.setpoints[ 8] = u3a * (  2.59807621135F );
      kafenv.cmd.setpoints[ 0] = u2a * ( -7.79422863406F );
      kafenv.cmd.setpoints[10] = u1a * (  5.19615242271F );
      kafenv.cmd.setpoints[11] =                            kafenv.state.x.y;
      kafenv.cmd.setpoints[16] =                            kafenv.state.x.z;
      kafenv.cmd.setpoints[20] = commander.store.traj.s.circleW;
      kafenv.cmd.setpoints[21] = 0;
      break;
    }
    default : { }
  }
}

peripheral commander_reset() {
  DPRINTF( "[H] Resetting Commander\n" );
  commander.doReadStorage = true;
  commander.lastTime = 0;
  commander.attitudeLimit = 0;
  for( unsigned char i = 0; i < STARTUP_CMD_COUNT; i++ ) {
    commander.store.startupcmds[i].length = 0;
    memset( commander.store.startupcmds[i].bytes, 0, sizeof( commander.store.startupcmds[i].bytes ) );
  }
  commander.store.traj.s.ascentRate = 0.9F;
  commander.store.traj.s.hoverHeight = 20.0F;
  commander.store.traj.s.descentRate = 0.1F;
  commander.store.traj.s.groundHeight = 0;
  commander.store.traj.s.glideV = 1.5F;
  commander.store.traj.s.glideX = 0;
  commander.store.traj.s.glideY = 0;
  commander.store.traj.s.glideZ = 0;
  commander.store.traj.s.homeX = 0;
  commander.store.traj.s.homeY = 0;
  commander.store.traj.s.circleR = 5.0F;
  commander.store.traj.s.circleW = 0.10472F;
  com_receiveMessage( COM_SET_STARTUP, 2, []( void** response, const void* content, const unsigned short len ) {
    startupcommand* comContent = ( startupcommand* )content;
    DPRINTF( "[H] Replying Set Startup Command: Index=%u Length=%u\n", comContent->index, comContent->content.length );
    if( comContent->index >= STARTUP_CMD_COUNT || comContent->content.length > 127 ) {
      *response = NULLPTR;
    }
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[H] Executing Set Startup Command\n" );
    startupcommand* comContent = ( startupcommand* )content;
    commander.store.startupcmds[ comContent->index ].length = comContent->content.length;
    memcpy( commander.store.startupcmds[ comContent->index ].bytes, comContent->content.bytes, comContent->content.length );
  } );
  com_receiveMessage( COM_SET_STORAGE, sizeof( char ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[H] Replying Set Storage Command\n" );
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    char* action = ( char* )content;
    DPRINTF( "[H] Executing Set Storage Command: Action='%c'\n", *action );
    firmware_handlePersistents( COMS_BUFFER, sizeof( COMS_BUFFER ), 0, *action );
  } );
  com_receiveMessage( COM_SET_TRAJECTORY, sizeof( STDBYTE ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[H] Replying Set Trajectory Command\n" );
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    STDBYTE* trajMode = ( STDBYTE* )content;
    DPRINTF( "[H] Executing Set Trajectory Command: Trajectory=%u\n", *trajMode );
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    commander_setTrajectories( *trajMode, NULLPTR );
    kafenv.info.triggerLock = 0;
  } );
  com_receiveMessage( COM_SET_TRAJCONFIG, sizeof( commander.store.traj.a ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[H] Replying Set Trajectory Configuration Command\n" );
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[H] Executing Set Trajectory Configuration Command\n" );
    float* array = ( float* )content;
    for( unsigned char i = 0; i < FPARLEN( commander.store.traj.a ); i++ ) {
      if( isfinite( array[i] ) ) {
        commander.store.traj.a[i] = array[i];
      }
    }
  } );
  return { "commander", sizeof( commander.store ), sizeof( commander ), &commander, [](){ commander_reset(); }, NULLPTR };
}

void commander_step( const unsigned long currentTime ) {
  const STDBYTE commandMode = CMD_MODE_MASK & kafenv.info.flightMode;
  //read and write configuration saved on flash
  if( commandMode != CMD_NULL_MODE ) {
    if( commander.doReadStorage ) {
      DPRINTF( "[H] Attempting Storage Read\n" );
      if( firmware_handlePersistents( COMS_BUFFER, sizeof( COMS_BUFFER ), 0, 'R' ) > 0 ) {
        DPRINTF( "[H] Running Startup Commands\n" );
        radio com_radio = { currentTime, COMMANDER_COM_METHOD, false, false, NULLPTR, 
            []() { return ( unsigned short )0; }, []( void* ptr, unsigned short len ) {}, []( void* ptr, unsigned short len ) {} };
        entity* gs = com_registerEntity( GROUND_STATION_ID );
        gs->flightMode = NULL_MODE;
        gs->liason = COMMANDER_COM_METHOD;
        gs->nodeOrder = 0;
        gs->lastSeen = currentTime;
        gs->position = { 0, 0, 0 };
        gs->distance = NAN;
        for( unsigned char i = 0; i < STARTUP_CMD_COUNT; i++ ) {
          if( commander.store.startupcmds[i].length > 0 ) {
            commander.store.startupcmds[i].header.fromID = GROUND_STATION_ID;
            commander.store.startupcmds[i].header.toID = kafenv.info.deviceID;
            com_radio.packet = &commander.store.startupcmds[i].bytes;
            com_step( &com_radio );
          }
        }
      } else {
        DPRINTF( "[H] Invalid Storage Read Signature\n" );
        firmware_handlePersistents( COMS_BUFFER, sizeof( COMS_BUFFER ), 0, 'W' );
      }
      commander.doReadStorage = false;
    }
  }
  //check for inverted attitude orientation
  if( commandMode == CMD_NOMINAL_MODE || commandMode == CMD_DESCENT_MODE ) {
    commander.attitudeLimit = kafenv.state.q.x * kafenv.state.q.x + kafenv.state.q.y * kafenv.state.q.y > 1 ? 
        commander.attitudeLimit + ( currentTime - commander.lastTime ) : 0;
    if( commander.attitudeLimit > ATTITUDE_THRESHOLD_TIME ) {
      DPRINTF( "[H] Quadcopter Crash Detected, Terminating Flight: Time=%lu\n", currentTime );
      kafenv.info.actuation = false;
      kafenv.info.flightMode = CMD_IDLE_MODE | ( DEFAULT_MODES_MASK & kafenv.info.flightMode );
    }
  }
  //check for disconnects and low battery
  if( commandMode == CMD_NOMINAL_MODE ) {
    if( kafenv.info.actuation && ( com_getEntityById( GROUND_STATION_ID ) == NULLPTR || kafenv.info.battery < 5.0F ) ) {
      DPRINTF( "[H] Quadcopter Critical Error, Initiating Landing: Time=%lu\n", currentTime );
      kafenv.info.triggerLock = 1;
      FLTSYNC;
      kafenv.info.flightMode = CMD_DESCENT_MODE | NULL_MODE;
      commander_setTrajectories( FLIGHTPATH_LAND, NULLPTR );
      kafenv.info.triggerLock = 0;
    }
  }
  //check for controlled emergency descent
  if( commandMode == CMD_DESCENT_MODE ) {
    if( kafenv.state.x.z < 0.1F ) {
      kafenv.info.actuation = false;
      kafenv.info.flightMode = CMD_IDLE_MODE | ( DEFAULT_MODES_MASK & kafenv.info.flightMode );
    } else {
      kafenv.cmd.setpoints[ 6] = kafenv.state.x.x;
      kafenv.cmd.setpoints[11] = kafenv.state.x.y;
    }
  }
  commander.lastTime = currentTime;
}