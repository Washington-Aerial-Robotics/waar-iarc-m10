#include "../core/firmware.h"
#include "../core/communication.h"
#include "../core/flight.h"
#include "commander.h"
#include "common_data.h"
#include "estimation.h"

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

#define COMMANDER_COM_METHOD 5
#define STORAGE_CAPACITY 0x5000
#define STARTUP_CMD_COUNT 3
#define ATTITUDE_THRESHOLD_TIME 3000 //ms
#define TARGET_NONE             0
#define TARGET_LEGACY           1
#define TARGET_PI_STREAM        2

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
  STDBYTE targetKind;
  STDBYTE targetSender;
  unsigned char targetLength;
  bool sequenceInitialized;
} commander;

struct startupcommand {
  unsigned char index;
  commanding::storage::startupcmd content;
};

//The legacy polynomial generator and the streamed Pi controller deliberately have distinct target kinds:
//only the latter owns a 500ms heartbeat. This preserves one-shot ground-station trajectories while making
//loss of a live Pi stream fail safe.
static bool finiteStatePosition() {
  return isfinite( kafenv.state.x.x ) && isfinite( kafenv.state.x.y ) && isfinite( kafenv.state.x.z )
      && isfinite( kafenv.state.q.z );
}

static bool trajectoryRequestValid( const STDBYTE mode, const float args[4], const bool emergency ) {
  if( !finiteStatePosition() || ( !emergency && !estimation_positionValid() ) ) return false;
  switch( mode ) {
    case FLIGHTPATH_LAUNCH : {
      const float height = args == NULLPTR ? commander.store.traj.s.hoverHeight : args[0];
      const float rate = args == NULLPTR ? commander.store.traj.s.ascentRate : args[1];
      return isfinite( height ) && isfinite( rate ) && rate > 0 && height >= kafenv.state.x.z;
    }
    case FLIGHTPATH_LAND : {
      const float height = args == NULLPTR ? commander.store.traj.s.groundHeight : args[0];
      const float rate = args == NULLPTR ? commander.store.traj.s.descentRate : args[1];
      return isfinite( height ) && isfinite( rate ) && rate > 0 && height <= kafenv.state.x.z;
    }
    case FLIGHTPATH_POSLOCK : return true;
    case FLIGHTPATH_GLIDEPOINT : {
      const float speed = args == NULLPTR ? commander.store.traj.s.glideV : args[0];
      const float x = args == NULLPTR ? commander.store.traj.s.glideX : args[1];
      const float y = args == NULLPTR ? commander.store.traj.s.glideY : args[2];
      const float z = args == NULLPTR ? commander.store.traj.s.glideZ : args[3];
      return isfinite( speed ) && speed > 0 && isfinite( x ) && isfinite( y ) && isfinite( z );
    }
    case FLIGHTPATH_RETURNHOME : {
      const float speed = commander.store.traj.s.glideV;
      const float x = args == NULLPTR ? commander.store.traj.s.homeX : args[0];
      const float y = args == NULLPTR ? commander.store.traj.s.homeY : args[1];
      return isfinite( speed ) && speed > 0 && isfinite( x ) && isfinite( y );
    }
    case FLIGHTPATH_CIRCLE : {
      const float radius = args == NULLPTR ? commander.store.traj.s.circleR : args[0];
      const float rate = args == NULLPTR ? commander.store.traj.s.circleW : args[1];
      return isfinite( radius ) && radius > 0 && isfinite( rate ) && rate > 0;
    }
    default : return false;
  }
}

static bool setTrajectories( const STDBYTE mode, const float args[4], const bool emergency ) {
  if( !trajectoryRequestValid( mode, args, emergency ) ) {
    DPRINTF( "[H] Rejected Flight Path: Mode=%u, PositionValid=%u, Emergency=%u\n",
        mode, estimation_positionValid(), emergency );
    return false;
  }
  //Trajectory generation never arms by itself. Callers must explicitly arm through COM_SET_ACTUATION;
  //when already flying, a requested/emergency landing naturally preserves the existing actuated state.
  const bool wasActuating = kafenv.info.actuation;
  const STDBYTE commandMode = mode == FLIGHTPATH_LAND && wasActuating ?
      CMD_DESCENT_MODE : ( kafenv.info.flightMode & CMD_MODE_MASK );
  kafenv.info.flightMode = commandMode | TRAJECTORY_MODE;
  kafenv.info.actuation = wasActuating;
  //Every path owns the complete polynomial array. Clearing it first prevents a previous path's untouched
  //coefficients from leaking into the next one; index 0 is always the elapsed-time counter.
  for( unsigned char i = 0; i < FPARLEN( kafenv.cmd.setpoints ); i++ ) {
    kafenv.cmd.setpoints[i] = 0;
  }
  kafenv.cmd.setpointVelocity = { 0, 0, 0 };
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
      if( distance > 0 ) {
        kafenv.cmd.setpoints[ 5] = dx / distance * commander.store.traj.s.glideV;
        kafenv.cmd.setpoints[10] = dy / distance * commander.store.traj.s.glideV;
      }
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
      if( distance > 0 ) {
        kafenv.cmd.setpoints[ 5] = diff.x / distance * commander.store.traj.s.glideV;
        kafenv.cmd.setpoints[10] = diff.y / distance * commander.store.traj.s.glideV;
        kafenv.cmd.setpoints[15] = diff.z / distance * commander.store.traj.s.glideV;
      }
      break;
    }
    case FLIGHTPATH_CIRCLE : {
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
      //Y's t^2 coefficient belongs at index 9. Index 0 is exclusively TRAJECTORY_MODE's elapsed-time
      //counter; writing the coefficient there made a circle start at an arbitrary (often negative) time.
      kafenv.cmd.setpoints[ 9] = u2a * ( -7.79422863406F );
      kafenv.cmd.setpoints[10] = u1a * (  5.19615242271F );
      kafenv.cmd.setpoints[11] =                            kafenv.state.x.y;
      kafenv.cmd.setpoints[16] =                            kafenv.state.x.z;
      kafenv.cmd.setpoints[20] = commander.store.traj.s.circleW;
      kafenv.cmd.setpoints[21] = 0;
      break;
    }
    default : return false;
  }
  commander.targetKind = TARGET_LEGACY;
  commander.targetSender = GROUND_STATION_ID;
  commander.targetLength = FPARLEN( kafenv.cmd.setpoints );
  return true;
}

bool commander_setTrajectories( const STDBYTE mode, const float args[4] ) {
  return setTrajectories( mode, args, false );
}

static bool commander_setEmergencyLanding() {
  if( setTrajectories( FLIGHTPATH_LAND, NULLPTR, true ) ) return true;
  DPRINTF( "[H] Emergency landing path unavailable; disarming because a finite position/trajectory could not be formed\n" );
  kafenv.info.actuation = false;
  kafenv.info.flightMode = CMD_IDLE_MODE | NULL_MODE;
  FPFILL0( i, kafenv.cmd.motors );
  return false;
}

bool commander_piStreamActive() {
  return commander.targetKind == TARGET_PI_STREAM && commander.targetSender == PI_CONTROLLER_ID;
}

bool commander_piSetpointFresh( const unsigned long currentTime ) {
  return commander_piStreamActive() && commander.sequenceInitialized
      && ( currentTime - kafenv.cmd.setpointMillis ) <= SETPOINT_STALE_MS;
}

void commander_resetPiSequence() {
  commander.sequenceInitialized = false;
  kafenv.cmd.setpointSeq = 0;
  kafenv.cmd.setpointMillis = 0;
  if( commander.targetKind == TARGET_PI_STREAM ) {
    commander.targetKind = TARGET_NONE;
    commander.targetSender = 0;
    commander.targetLength = 0;
    kafenv.cmd.setpointVelocity = { 0, 0, 0 };
  }
}

bool commander_validateFlightModeCommand( const STDBYTE mode, const unsigned char length, const float* values,
    const STDBYTE sender ) {
  for( unsigned char i = 0; i < length; i++ ) {
    if( !isfinite( values[i] ) ) return false;
  }
  const STDBYTE flightMode = DEFAULT_MODES_MASK & mode;
  if( sender == PI_CONTROLLER_ID ) {
    //The Pi owns exactly one live control mode. Landing uses the separately constrained
    //COM_SET_TRAJECTORY(FLIGHTPATH_LAND) path; all manual/calibration/motor modes remain G-only.
    if( mode != ( CMD_NOMINAL_MODE | POS_SETPOINT_MODE ) || length != 0 ) return false;
  } else if( sender != GROUND_STATION_ID ) {
    return false;
  }
  if( flightMode != POS_SETPOINT_MODE && flightMode != TRAJECTORY_MODE ) return true;
  if( !estimation_positionValid() ) return false;
  const unsigned char requiredLength = flightMode == POS_SETPOINT_MODE ? 4 : FPARLEN( kafenv.cmd.setpoints );
  if( length == requiredLength ) return true;
  if( length != 0 || commander.targetLength < requiredLength || commander.targetSender != sender ) return false;
  return commander.targetKind == TARGET_PI_STREAM ? commander_piSetpointFresh( millis() ) : commander.targetKind == TARGET_LEGACY;
}

void commander_acceptFlightModeCommand( const STDBYTE mode, const unsigned char length, const STDBYTE sender ) {
  const STDBYTE flightMode = DEFAULT_MODES_MASK & mode;
  if( flightMode == POS_SETPOINT_MODE || flightMode == TRAJECTORY_MODE ) {
    if( length > 0 ) {
      commander.targetKind = TARGET_LEGACY;
      commander.targetSender = sender;
      commander.targetLength = length;
      kafenv.cmd.setpointVelocity = { 0, 0, 0 };
    } else if( commander.targetKind != TARGET_PI_STREAM ) {
      kafenv.cmd.setpointVelocity = { 0, 0, 0 };
    }
  } else {
    commander.targetKind = TARGET_NONE;
    commander.targetSender = 0;
    commander.targetLength = 0;
    kafenv.cmd.setpointVelocity = { 0, 0, 0 };
  }
}

void commander_acceptLegacySetpoint( const STDBYTE sender, const unsigned char length ) {
  commander.targetKind = TARGET_LEGACY;
  commander.targetSender = sender;
  commander.targetLength = length;
  kafenv.cmd.setpointVelocity = { 0, 0, 0 };
}

bool commander_canArm( const STDBYTE sender ) {
  const STDBYTE flightMode = DEFAULT_MODES_MASK & kafenv.info.flightMode;
  if( sender == PI_CONTROLLER_ID ) {
    return commander_controlCalibrated()
        && kafenv.info.flightMode == ( CMD_NOMINAL_MODE | POS_SETPOINT_MODE )
        && commander_validateFlightModeCommand( kafenv.info.flightMode, 0, kafenv.cmd.setpoints, sender );
  }
  if( sender != GROUND_STATION_ID ) return false;
  if( flightMode != POS_SETPOINT_MODE && flightMode != TRAJECTORY_MODE ) return true;
  return commander_validateFlightModeCommand( kafenv.info.flightMode, 0, kafenv.cmd.setpoints, sender );
}

static bool controllerGainsValid( const coordinate gain ) {
  return isfinite( gain.Kp ) && gain.Kp > 0
      && isfinite( gain.Ki ) && gain.Ki >= 0
      && isfinite( gain.Kd ) && gain.Kd >= 0;
}

bool commander_controlCalibrated() {
  bool valid = isfinite( kafenv.cal.hoverThrust )
      && kafenv.cal.hoverThrust > CONTROL_HOVER_THRUST_MIN
      && kafenv.cal.hoverThrust < CONTROL_HOVER_THRUST_MAX
      && isfinite( kafenv.cal.gravitation )
      && kafenv.cal.gravitation > CONTROL_GRAVITY_MIN
      && kafenv.cal.gravitation < CONTROL_GRAVITY_MAX
      && isfinite( kafenv.cal.anglealpha ) && kafenv.cal.anglealpha > 0 && kafenv.cal.anglealpha <= 1
      && isfinite( kafenv.cal.positionalpha ) && kafenv.cal.positionalpha > 0
      && controllerGainsValid( kafenv.cal.xpid ) && controllerGainsValid( kafenv.cal.vpid )
      && controllerGainsValid( kafenv.cal.apid ) && controllerGainsValid( kafenv.cal.qpid );
  ITRVEC3( i ) valid = valid && controllerGainsValid( kafenv.cal.wpid[i] );
  return valid;
}

bool commander_batteryValid() {
  //No battery ADC/monitor currently updates kafenv.info.battery; kaf_reset()'s 100 is a placeholder.
  //Keep this fail-closed until a range-checked, timestamped hardware measurement path is implemented.
  return false;
}

static bool piSetpointValid( const trajsetpoint* setpt, const unsigned short len, const packet_header header ) {
  if( len != sizeof( trajsetpoint ) || header.fromID != PI_CONTROLLER_ID || !estimation_positionValid() ) {
    DPRINTF( "[H] Rejected Pi Setpoint: Length=%u, Sender='%c', PositionValid=%u\n",
        len, header.fromID, estimation_positionValid() );
    return false;
  }
  const STDBYTE flightMode = DEFAULT_MODES_MASK & kafenv.info.flightMode;
  if( kafenv.info.actuation && flightMode != POS_SETPOINT_MODE ) {
    DPRINTF( "[H] Rejected Pi Setpoint: Armed flight mode %u is not POS_SETPOINT_MODE\n", flightMode );
    return false;
  }
  if( commander.sequenceInitialized && setpt->sequence <= kafenv.cmd.setpointSeq ) {
    DPRINTF( "[H] Rejected Pi Setpoint: Sequence=%lu <= Last=%lu\n",
        ( unsigned long )setpt->sequence, kafenv.cmd.setpointSeq );
    return false;
  }
  return isfinite( setpt->x ) && isfinite( setpt->y ) && isfinite( setpt->z ) && isfinite( setpt->yaw )
      && isfinite( setpt->vx ) && isfinite( setpt->vy ) && isfinite( setpt->vz );
}

peripheral commander_reset() {
  DPRINTF( "[H] Resetting Commander\n" );
  commander.doReadStorage = true;
  commander.lastTime = 0;
  commander.attitudeLimit = 0;
  commander.targetKind = TARGET_NONE;
  commander.targetSender = 0;
  commander.targetLength = 0;
  commander.sequenceInitialized = false;
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
  com_receiveValidatedMessage( COM_SET_TRAJECTORY, sizeof( STDBYTE ), []( void** response, const void* content,
      const unsigned short len, const packet_header header ) {
    DPRINTF( "[H] Replying Set Trajectory Command\n" );
    const STDBYTE trajectory = *( ( const STDBYTE* )content );
    const bool authorized = header.fromID == GROUND_STATION_ID
        || ( header.fromID == PI_CONTROLLER_ID && trajectory == FLIGHTPATH_LAND );
    //A Pi LAND is safe while disarmed: it prepares a descent trajectory but cannot start the motors.
    //While airborne it preserves actuation and changes the target to descent. Other Pi legacy paths remain
    //forbidden; normal target generation belongs in COM_SET_TRAJSETPT.
    if( len != sizeof( STDBYTE ) || !authorized || !trajectoryRequestValid( trajectory, NULLPTR, false ) ) {
      *response = NULLPTR;
    }
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    STDBYTE* trajMode = ( STDBYTE* )content;
    DPRINTF( "[H] Executing Set Trajectory Command: Trajectory=%u\n", *trajMode );
    //A disarmed Pi LAND is an acknowledged, idempotent no-op. An airborne LAND is explicitly moved into
    //the descent commander state so its ground-height gate disarms the motors when the descent completes.
    if( header.fromID == PI_CONTROLLER_ID && *trajMode == FLIGHTPATH_LAND && !kafenv.info.actuation ) return;
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    if( *trajMode == FLIGHTPATH_LAND ) {
      kafenv.info.flightMode = CMD_DESCENT_MODE | ( kafenv.info.flightMode & DEFAULT_MODES_MASK );
    }
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
  //Generic Pi-to-ESP32 position/velocity/yaw setpoint, shared by both Qualification and Mine Search - the
  //Pi decides WHERE the drone should be (formation slot, orbit point, survey waypoint); this ESP32 only
  //tracks it via the existing POS_SETPOINT_MODE cascade, validates freshness/sequence, and never itself
  //decides where to go. Payload is fixed-size ( trajsetpoint below), not the raw variable-length float
  //array COM_SET_SETPT uses, so it can carry a sequence number and be rejected out-of-order.
  com_receiveValidatedMessage( COM_SET_TRAJSETPT, sizeof( trajsetpoint ), []( void** response, const void* content,
      const unsigned short len, const packet_header header ) {
    if( !piSetpointValid( ( const trajsetpoint* )content, len, header ) ) *response = NULLPTR;
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    const trajsetpoint* setpt = ( const trajsetpoint* )content;
    DPRINTF( "[H] Accepted Trajectory Setpoint: Sequence=%lu, X=[ %.3f, %.3f, %.3f ], Yaw=%.3f, V=[ %.3f, %.3f, %.3f ]\n",
        ( unsigned long )setpt->sequence, setpt->x, setpt->y, setpt->z, setpt->yaw, setpt->vx, setpt->vy, setpt->vz );
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    kafenv.cmd.setpoints[0] = setpt->x;
    kafenv.cmd.setpoints[1] = setpt->y;
    kafenv.cmd.setpoints[2] = setpt->z;
    kafenv.cmd.setpoints[3] = setpt->yaw;
    kafenv.cmd.setpointVelocity = { setpt->vx, setpt->vy, setpt->vz };
    kafenv.cmd.setpointSeq = setpt->sequence;
    kafenv.cmd.setpointMillis = millis();
    commander.targetKind = TARGET_PI_STREAM;
    commander.targetSender = header.fromID;
    commander.targetLength = 4;
    commander.sequenceInitialized = true;
    //Deliberately does NOT change kafenv.info.flightMode - receiving a setpoint updates the TARGET only.
    //Entering POS_SETPOINT_MODE (i.e. actually arming/moving toward it) is a separate, explicit
    //COM_SET_FLIGHTMODE decision, gated on estimation_positionValid() - see communication.cpp.
    kafenv.info.triggerLock = 0;
  } );
  //Origin latching is explicit: the Pi retries this zero-payload command until a fresh, quality-gated GPS
  //fix exists. It is forbidden while armed. Relatching while disarmed also resets the stream sequence so a
  //restarted Pi can begin again without weakening replay protection during flight.
  com_receiveValidatedMessage( COM_SET_GPSORIGIN, 0, []( void** response, const void* content,
      const unsigned short len, const packet_header header ) {
    if( len != 0 || header.fromID != PI_CONTROLLER_ID || kafenv.info.actuation ) {
      *response = NULLPTR;
      return ( unsigned short )0;
    }
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    const bool originLatched = gps_setOrigin();
    if( originLatched ) {
      estimation_latchLocalOrigin();
      kafenv.state.x = { 0, 0, 0 };
      kafenv.state.v = { 0, 0, 0 };
      commander_resetPiSequence();
    }
    kafenv.info.triggerLock = 0;
    if( !originLatched ) *response = NULLPTR;
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) { } );
  com_receiveValidatedMessage( COM_REQUEST_TELEMETRY, 0, []( void** response, const void* content,
      const unsigned short len, const packet_header header ) {
    if( len != 0 ) {
      *response = NULLPTR;
      return ( unsigned short )0;
    }
    autonomy_telemetry* telemetry = ( autonomy_telemetry* )*response;
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    telemetry->protocolVersion = AUTONOMY_TELEMETRY_VERSION;
    telemetry->flightMode = kafenv.info.flightMode;
    telemetry->flags = ( estimation_positionValid() ? AUTONOMY_FLAG_POSITION_VALID : 0 )
        | ( estimation_originSet() ? AUTONOMY_FLAG_ORIGIN_SET : 0 )
        | ( kafenv.info.actuation ? AUTONOMY_FLAG_ACTUATION : 0 )
        | ( commander_piStreamActive() ? AUTONOMY_FLAG_PI_STREAM : 0 )
        | ( commander_piSetpointFresh( millis() ) ? AUTONOMY_FLAG_SETPOINT_FRESH : 0 )
        | ( commander_controlCalibrated() ? AUTONOMY_FLAG_CONTROL_CALIBRATED : 0 )
        | ( commander_batteryValid() ? AUTONOMY_FLAG_BATTERY_VALID : 0 );
    telemetry->reserved = 0;
    telemetry->setpointSequence = ( uint32_t )kafenv.cmd.setpointSeq;
    ITRVEC3( i ) telemetry->position[i] = kafenv.state.x.f[i];
    ITRVEC3( i ) telemetry->velocity[i] = kafenv.state.v.f[i];
    if( flight_rotationQuaternion( telemetry->quaternion ) ) telemetry->flags |= AUTONOMY_FLAG_ATTITUDE_VALID;
    ITRVEC3( i ) telemetry->angularVelocity[i] = kafenv.state.w.f[i];
    telemetry->battery = kafenv.info.battery;
    kafenv.info.triggerLock = 0;
    return ( unsigned short )sizeof( autonomy_telemetry );
  }, []( const void* content, const packet_header header ) { }, COM_REPLY_TELEMETRY );
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
  //Manual/legacy nominal control remains tied to the ground station. Streamed Pi control is instead tied
  //to the authenticated setpoint stream below; requiring a synthetic 'G' entity would make a healthy Pi
  //mission land as soon as the phone disconnected.
  if( commandMode == CMD_NOMINAL_MODE ) {
    const bool controllerMissing = commander_piStreamActive() ?
        com_getEntityById( PI_CONTROLLER_ID ) == NULLPTR : com_getEntityById( GROUND_STATION_ID ) == NULLPTR;
    const bool lowBattery = commander_batteryValid() && kafenv.info.battery < 5.0F;
    if( kafenv.info.actuation && ( controllerMissing || lowBattery ) ) {
      DPRINTF( "[H] Quadcopter Critical Error, Initiating Landing: Time=%lu\n", currentTime );
      kafenv.info.triggerLock = 1;
      FLTSYNC;
      kafenv.info.flightMode = CMD_DESCENT_MODE | NULL_MODE;
      commander_setEmergencyLanding();
      kafenv.info.triggerLock = 0;
    }
  }
  //check for lost Pi setpoint stream or an invalid position estimate while autonomously flying by
  //position - same emergency-descent response as the ground-station-disconnect/low-battery check above,
  //reused rather than duplicated. Only applies to the two flight modes that depend on kafenv.state.x/
  //kafenv.cmd.setpoints being trustworthy; ACCEL_SETPOINT_MODE (attitude-only) doesn't need this.
  if( commandMode == CMD_NOMINAL_MODE && kafenv.info.actuation ) {
    const STDBYTE flightModeBits = DEFAULT_MODES_MASK & kafenv.info.flightMode;
    if( flightModeBits == POS_SETPOINT_MODE || flightModeBits == TRAJECTORY_MODE ) {
      //Only Pi streaming targets have a 500ms heartbeat. Legacy COM_SET_SETPT/inline polynomial paths are
      //one-shot by design and remain guarded by position validity plus their controller's 8s entity timeout.
      const bool setpointStale = commander_piStreamActive() && !commander_piSetpointFresh( currentTime );
      if( !estimation_positionValid() || setpointStale ) {
        DPRINTF( "[H] Position/Setpoint Invalid During Autonomous Flight, Initiating Landing: "
            "PositionValid=%u, SetpointStale=%u, Time=%lu\n", estimation_positionValid(), setpointStale, currentTime );
        kafenv.info.triggerLock = 1;
        FLTSYNC;
        kafenv.info.flightMode = CMD_DESCENT_MODE | NULL_MODE;
        commander_setEmergencyLanding();
        kafenv.info.triggerLock = 0;
      }
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
