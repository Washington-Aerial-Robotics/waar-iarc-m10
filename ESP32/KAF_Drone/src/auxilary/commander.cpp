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

//Qualification state machine bookkeeping - internal to commander.cpp, not persisted. kafenv.info.qualState/
//qualRevolutions are the externally-visible subset (telemetry only, see kaf_drone.h).
static struct {
  unsigned long orbitEntryMillis;       //millis() QUAL_ORBIT was entered - baseline for this drone's
                                          //formationSlot phase-stagger delay before its first circle issue
  unsigned long lastCircleReissueMillis; //0 = no circle issued yet this orbit
  float accumulatedAngle;                //radians traversed since the first circle issue (post phase-delay)
  unsigned long altitudeGoodSinceMillis; //0 = not currently within QUAL_ALTITUDE_TOLERANCE_M
} qual;

#define QUAL_TWO_PI 6.28318530718F

//Square Test state machine bookkeeping - internal, not persisted. kafenv.info.squareState is the externally-
//visible subset (telemetry only). x/y targets are in the GPS-relative local frame latched by gps_setOrigin()
//at START; z is SQUARE_ALTITUDE_M throughout (BME-derived, via the shared estimator - see estimation.cpp).
static struct {
  float targetX, targetY, targetZ;      //current leg's destination in the local frame
  unsigned long positionGoodSinceMillis; //0 = not currently within SQUARE_POSITION_TOLERANCE_M of the target
  unsigned long legStartMillis;          //millis() the current leg/climb began - baseline for SQUARE_LEG_TIMEOUT_MS
} square;

struct startupcommand {
  unsigned char index;
  commanding::storage::startupcmd content;
};

//COM_SET_TRAJSETPT payload - one generic Pi-to-ESP32 setpoint shape reused by Qualification and Mine
//Search alike. `sequence` must strictly increase for a setpoint to be accepted (see the handler in
//commander_reset()); the ESP32 timestamps its own receipt (kafenv.cmd.setpointMillis) rather than trusting
//a Pi-supplied clock, since only locally-measured arrival time is meaningful for staleness detection.
struct trajsetpoint {
  unsigned long sequence;
  float x, y, z, yaw;
  float vx, vy, vz;
};

void commander_setTrajectories( STDBYTE mode, const float args[4] ) {
  kafenv.info.flightMode = ( kafenv.info.flightMode & ~DEFAULT_MODES_MASK ) | TRAJECTORY_MODE;
  kafenv.info.actuation = true;
  //Previously nothing here cleared kafenv.cmd.setpoints before writing the new mode's fields, so any
  //per-axis polynomial coefficient NOT touched by the new mode (e.g. LAUNCH/LAND/POSLOCK never set indices
  //2-5, 7-10, 12-15, 17-20 at all) kept whatever a PRIOR trajectory call had left there - a LAUNCH issued
  //right after a CIRCLE would fly with the circle's stale X/Y polynomial terms still active. Index 0 is
  //deliberately left alone here (reset to 0 for every mode below, individual cases may still override it -
  //see FLIGHTPATH_CIRCLE, which uses it for a coefficient rather than TRAJECTORY_MODE's elapsed-time role;
  //that specific conflict is flagged separately and not resolved by this clear).
  for( unsigned char i = 2; i < FPARLEN( kafenv.cmd.setpoints ); i++ ) {
    kafenv.cmd.setpoints[i] = 0;
  }
  kafenv.cmd.setpoints[0] = 0;
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
      //setpoints[10]/[8] give Y (sin-based) its two legitimate odd-power terms (t^1, t^3) - Y needs no
      //t^0/t^2/t^4 term (sin(0)=0, and sin's series has no even powers), so Y is already fully specified
      //without this slot. X (cos-based, indices 2/4/6 above) is likewise already fully specified from its
      //own even-power terms alone. This line used to write a fifth coefficient into setpoints[0] - which
      //TRAJECTORY_MODE (flight.cpp) also uses as its elapsed-time accumulator - corrupting the very first
      //tick's time base every time a circle trajectory was issued. Removed rather than relocated: nothing
      //in either axis's Taylor expansion actually needs a fifth term at this order.
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

//Discrete, phone-issued transitions of the qualification state machine. Runs on the com_task core, same as
//commander_step() (both driven from the same loop in firmware_full.ino), so no cross-core race between this
//and commander_qualificationStep() below - the triggerLock/FLTSYNC pairs here are only for the flight_task
//core's benefit, same convention as every other commander_setTrajectories() call in this file.
void commander_qualificationCommand( STDBYTE cmd ) {
  if( kafenv.info.autonomyMode != AUTONOMY_QUALIFICATION ) {
    DPRINTF( "[H] Qualification Command Ignored: autonomyMode is not AUTONOMY_QUALIFICATION\n" );
    return;
  }
  //LAND/ABORT have priority over every other qualification command - accepted from any in-progress state.
  if( ( cmd == QUALCMD_LAND || cmd == QUALCMD_ABORT ) && kafenv.info.qualState != QUAL_LANDING && kafenv.info.qualState != QUAL_FINISH ) {
    DPRINTF( "[H] Qualification: %s commanded from state %u\n", cmd == QUALCMD_ABORT ? "ABORT" : "LAND", kafenv.info.qualState );
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    kafenv.info.flightMode = CMD_NOMINAL_MODE | TRAJECTORY_MODE;
    kafenv.info.actuation = true; //LAND must still be able to actuate even if commanded early from QUAL_BOOT
    commander_setTrajectories( FLIGHTPATH_LAND, NULLPTR );
    kafenv.info.triggerLock = 0;
    kafenv.info.qualState = QUAL_LANDING;
    return;
  }
  switch( cmd ) {
    case QUALCMD_LAUNCH : {
      if( kafenv.info.qualState != QUAL_BOOT ) {
        DPRINTF( "[H] Qualification: LAUNCH rejected, qualState=%u (expected QUAL_BOOT)\n", kafenv.info.qualState );
        return;
      }
      //Latches the current GPS fix as this launch's local-frame origin. Safe to call every LAUNCH attempt:
      //if the current fix isn't good enough, gps_setOrigin() leaves any previously-latched origin alone
      //and returns false, rather than clearing it - so this doesn't fail forward. Without this call
      //nothing in the firmware ever set originSet, so estimation_positionValid() (which checks it) was
      //always false and LAUNCH could never succeed.
      gps_setOrigin();
      if( !estimation_positionValid() ) {
        DPRINTF( "[H] Qualification: LAUNCH rejected, no valid position estimate\n" );
        return;
      }
      DPRINTF( "[H] Qualification: LAUNCH, Slot=%u\n", kafenv.info.formationSlot );
      kafenv.info.triggerLock = 1;
      FLTSYNC;
      kafenv.info.flightMode = CMD_NOMINAL_MODE | TRAJECTORY_MODE;
      kafenv.info.actuation = true;
      const float args[4] = { QUAL_HOVER_ALTITUDE_M, 0.9F, 0, 0 };
      commander_setTrajectories( FLIGHTPATH_LAUNCH, args );
      kafenv.info.triggerLock = 0;
      kafenv.info.qualState = QUAL_CLIMB_TO_FORMATION;
      qual.altitudeGoodSinceMillis = 0;
      break;
    }
    case QUALCMD_BEGIN_ORBIT : {
      if( kafenv.info.qualState != QUAL_HOVER_HOLD ) {
        DPRINTF( "[H] Qualification: BEGIN_ORBIT rejected, qualState=%u (expected QUAL_HOVER_HOLD)\n", kafenv.info.qualState );
        return;
      }
      DPRINTF( "[H] Qualification: BEGIN_ORBIT, Slot=%u\n", kafenv.info.formationSlot );
      //Doesn't issue FLIGHTPATH_CIRCLE immediately - commander_qualificationStep() does, once this drone's
      //formationSlot-based phase-stagger delay has elapsed (still holding via the existing POSLOCK from
      //QUAL_HOVER_HOLD in the meantime). This spreads out when each drone crosses its neighbours' orbit
      //paths instead of all four doing so at once.
      kafenv.info.qualState = QUAL_ORBIT;
      qual.orbitEntryMillis = millis();
      qual.lastCircleReissueMillis = 0;
      qual.accumulatedAngle = 0;
      kafenv.info.qualRevolutions = 0;
      break;
    }
    case QUALCMD_HOLD : {
      if( kafenv.info.qualState != QUAL_ORBIT ) {
        DPRINTF( "[H] Qualification: HOLD rejected, qualState=%u (expected QUAL_ORBIT)\n", kafenv.info.qualState );
        return;
      }
      DPRINTF( "[H] Qualification: HOLD, Revolutions=%u\n", kafenv.info.qualRevolutions );
      kafenv.info.triggerLock = 1;
      FLTSYNC;
      commander_setTrajectories( FLIGHTPATH_POSLOCK, NULLPTR );
      kafenv.info.triggerLock = 0;
      kafenv.info.qualState = QUAL_POST_ORBIT_HOLD;
      break;
    }
    default : {
      DPRINTF( "[H] Qualification: Unknown command %u\n", cmd );
    }
  }
}

//Time-driven qualification progression - called every commander_step() tick regardless of mode; no-ops
//immediately if not in Qualification mode or if the current qualState has nothing time-driven to do
//(QUAL_BOOT/QUAL_HOVER_HOLD/QUAL_POST_ORBIT_HOLD/QUAL_FINISH all only advance via
//commander_qualificationCommand() above, or the position/setpoint-invalid failsafe in commander_step()).
static void commander_qualificationStep( const unsigned long currentTime ) {
  if( kafenv.info.autonomyMode != AUTONOMY_QUALIFICATION ) {
    return;
  }
  switch( kafenv.info.qualState ) {
    case QUAL_CLIMB_TO_FORMATION : {
      const float altError = kafenv.state.x.z - QUAL_HOVER_ALTITUDE_M;
      const bool withinTolerance = ( altError < QUAL_ALTITUDE_TOLERANCE_M ) && ( altError > -QUAL_ALTITUDE_TOLERANCE_M );
      if( !withinTolerance ) {
        qual.altitudeGoodSinceMillis = 0;
      } else if( qual.altitudeGoodSinceMillis == 0 ) {
        qual.altitudeGoodSinceMillis = currentTime;
      } else if( currentTime - qual.altitudeGoodSinceMillis >= QUAL_ALTITUDE_DWELL_MS ) {
        DPRINTF( "[H] Qualification: Formation altitude reached (%.2fm), holding\n", kafenv.state.x.z );
        kafenv.info.triggerLock = 1;
        FLTSYNC;
        commander_setTrajectories( FLIGHTPATH_POSLOCK, NULLPTR );
        kafenv.info.triggerLock = 0;
        kafenv.info.qualState = QUAL_HOVER_HOLD;
      }
      break;
    }
    case QUAL_ORBIT : {
      const unsigned long periodMs = ( unsigned long )( QUAL_TWO_PI / QUAL_ORBIT_ANGULAR_RATE * 1000.0F );
      const unsigned long startDelayMs = ( ( unsigned long )kafenv.info.formationSlot ) * ( periodMs / 4UL );
      if( currentTime - qual.orbitEntryMillis < startDelayMs ) {
        break; //still within this drone's phase-stagger delay - keep holding (still POSLOCK'd from HOVER_HOLD)
      }
      if( qual.lastCircleReissueMillis == 0 || currentTime - qual.lastCircleReissueMillis >= QUAL_CIRCLE_REISSUE_MS ) {
        if( qual.lastCircleReissueMillis != 0 ) {
          qual.accumulatedAngle += QUAL_ORBIT_ANGULAR_RATE * ( ( currentTime - qual.lastCircleReissueMillis ) / 1000.0F );
          kafenv.info.qualRevolutions = ( unsigned char )( qual.accumulatedAngle / QUAL_TWO_PI );
        }
        qual.lastCircleReissueMillis = currentTime;
        const float args[4] = { QUAL_ORBIT_RADIUS_M, QUAL_ORBIT_ANGULAR_RATE, 0, 0 };
        kafenv.info.triggerLock = 1;
        FLTSYNC;
        commander_setTrajectories( FLIGHTPATH_CIRCLE, args );
        kafenv.info.triggerLock = 0;
        DPRINTF( "[H] Qualification: Circle re-issued, Revolutions=%u\n", kafenv.info.qualRevolutions );
      }
      if( kafenv.info.qualRevolutions >= QUAL_ORBIT_REVOLUTIONS_TARGET ) {
        DPRINTF( "[H] Qualification: Target revolutions reached, holding\n" );
        kafenv.info.triggerLock = 1;
        FLTSYNC;
        commander_setTrajectories( FLIGHTPATH_POSLOCK, NULLPTR );
        kafenv.info.triggerLock = 0;
        kafenv.info.qualState = QUAL_POST_ORBIT_HOLD;
      }
      break;
    }
    case QUAL_LANDING : {
      if( kafenv.state.x.z < 0.1F ) {
        kafenv.info.actuation = false;
        kafenv.info.flightMode = CMD_IDLE_MODE | ( DEFAULT_MODES_MASK & kafenv.info.flightMode );
        kafenv.info.qualState = QUAL_FINISH;
        DPRINTF( "[H] Qualification: Landed, FINISH\n" );
      }
      break;
    }
    default : { }
  }
}

//Starts (or fast-forwards to landing from) a Square Test leg by directly issuing FLIGHTPATH_LAND -
//shared by commander_squareCommand()'s LAND/ABORT priority handling.
static void squareBeginLanding() {
  kafenv.info.triggerLock = 1;
  FLTSYNC;
  kafenv.info.flightMode = CMD_NOMINAL_MODE | TRAJECTORY_MODE;
  kafenv.info.actuation = true;
  commander_setTrajectories( FLIGHTPATH_LAND, NULLPTR );
  kafenv.info.triggerLock = 0;
  kafenv.info.squareState = SQUARE_LANDING;
}

//Discrete, phone-issued transitions of the Square Test state machine - same com_task-only-caller reasoning
//as commander_qualificationCommand() above, no cross-core race with commander_squareStep() below.
void commander_squareCommand( STDBYTE cmd ) {
  if( kafenv.info.autonomyMode != AUTONOMY_SQUARE_TEST ) {
    DPRINTF( "[H] Square Command Ignored: autonomyMode is not AUTONOMY_SQUARE_TEST\n" );
    return;
  }
  //LAND/ABORT have priority - accepted from any in-progress state.
  if( ( cmd == SQUARECMD_LAND || cmd == SQUARECMD_ABORT ) && kafenv.info.squareState != SQUARE_LANDING && kafenv.info.squareState != SQUARE_FINISH ) {
    DPRINTF( "[H] Square: %s commanded from state %u\n", cmd == SQUARECMD_ABORT ? "ABORT" : "LAND", kafenv.info.squareState );
    squareBeginLanding();
    return;
  }
  if( cmd == SQUARECMD_START ) {
    if( kafenv.info.squareState != SQUARE_BOOT ) {
      DPRINTF( "[H] Square: START rejected, squareState=%u (expected SQUARE_BOOT)\n", kafenv.info.squareState );
      return;
    }
    //Same reasoning as QUALCMD_LAUNCH: latches the current GPS fix as this run's local-frame origin -
    //without this, estimation_positionValid() would never become true and START could never succeed.
    gps_setOrigin();
    if( !estimation_positionValid() ) {
      DPRINTF( "[H] Square: START rejected, no valid position estimate\n" );
      return;
    }
    DPRINTF( "[H] Square: START\n" );
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    kafenv.info.flightMode = CMD_NOMINAL_MODE | TRAJECTORY_MODE;
    kafenv.info.actuation = true;
    const float args[4] = { SQUARE_ALTITUDE_M, 0.9F, 0, 0 };
    commander_setTrajectories( FLIGHTPATH_LAUNCH, args );
    kafenv.info.triggerLock = 0;
    kafenv.info.squareState = SQUARE_CLIMB;
    square.positionGoodSinceMillis = 0;
    square.legStartMillis = millis();
  } else {
    DPRINTF( "[H] Square: Unknown command %u\n", cmd );
  }
}

//Advances to the next leg (or landing, after LEG4) with a fresh FLIGHTPATH_GLIDEPOINT target.
static void squareAdvanceLeg( STDBYTE nextState, float x, float y, float z, const unsigned long currentTime ) {
  square.targetX = x;
  square.targetY = y;
  square.targetZ = z;
  square.positionGoodSinceMillis = 0;
  square.legStartMillis = currentTime;
  kafenv.info.squareState = nextState;
  const float args[4] = { SQUARE_GLIDE_VELOCITY_M_S, x, y, z };
  kafenv.info.triggerLock = 1;
  FLTSYNC;
  commander_setTrajectories( FLIGHTPATH_GLIDEPOINT, args );
  kafenv.info.triggerLock = 0;
  DPRINTF( "[H] Square: Advancing to state %u, target=[ %.2f, %.2f, %.2f ]\n", nextState, x, y, z );
}

//Time-driven Square Test progression - called every commander_step() tick regardless of mode; no-ops
//immediately if not in Square Test mode. Mirrors commander_qualificationStep()'s structure.
static void commander_squareStep( const unsigned long currentTime ) {
  if( kafenv.info.autonomyMode != AUTONOMY_SQUARE_TEST ) {
    return;
  }
  const float S = SQUARE_SIDE_LENGTH_M;

  if( kafenv.info.squareState == SQUARE_LANDING ) {
    if( kafenv.state.x.z < 0.1F ) {
      kafenv.info.actuation = false;
      kafenv.info.flightMode = CMD_IDLE_MODE | ( DEFAULT_MODES_MASK & kafenv.info.flightMode );
      kafenv.info.squareState = SQUARE_FINISH;
      DPRINTF( "[H] Square: Landed, FINISH\n" );
    }
    return;
  }
  if( kafenv.info.squareState != SQUARE_CLIMB && kafenv.info.squareState != SQUARE_LEG1 &&
      kafenv.info.squareState != SQUARE_LEG2 && kafenv.info.squareState != SQUARE_LEG3 && kafenv.info.squareState != SQUARE_LEG4 ) {
    return; //SQUARE_BOOT/SQUARE_FINISH - nothing time-driven to do
  }

  //Safety: land rather than hold forever if a leg doesn't complete in time (bad tuning, GPS drift,
  //physical obstruction) - see SQUARE_LEG_TIMEOUT_MS's comment in commander.h.
  if( currentTime - square.legStartMillis >= SQUARE_LEG_TIMEOUT_MS ) {
    DPRINTF( "[H] Square: Leg timed out in state %u, landing\n", kafenv.info.squareState );
    squareBeginLanding();
    return;
  }

  if( kafenv.info.squareState == SQUARE_CLIMB ) {
    const float altError = kafenv.state.x.z - SQUARE_ALTITUDE_M;
    const bool withinTolerance = ( altError < QUAL_ALTITUDE_TOLERANCE_M ) && ( altError > -QUAL_ALTITUDE_TOLERANCE_M );
    if( !withinTolerance ) {
      square.positionGoodSinceMillis = 0;
    } else if( square.positionGoodSinceMillis == 0 ) {
      square.positionGoodSinceMillis = currentTime;
    } else if( currentTime - square.positionGoodSinceMillis >= QUAL_ALTITUDE_DWELL_MS ) {
      DPRINTF( "[H] Square: Formation altitude reached (%.2fm)\n", kafenv.state.x.z );
      squareAdvanceLeg( SQUARE_LEG1, S, 0, SQUARE_ALTITUDE_M, currentTime );
    }
    return;
  }

  //SQUARE_LEG1..LEG4: check 3D distance to the current leg's target.
  const float dx = kafenv.state.x.x - square.targetX;
  const float dy = kafenv.state.x.y - square.targetY;
  const float dz = kafenv.state.x.z - square.targetZ;
  const bool arrived = ( dx * dx + dy * dy + dz * dz ) < ( SQUARE_POSITION_TOLERANCE_M * SQUARE_POSITION_TOLERANCE_M );
  if( !arrived ) {
    square.positionGoodSinceMillis = 0;
    return;
  }
  if( square.positionGoodSinceMillis == 0 ) {
    square.positionGoodSinceMillis = currentTime;
    return;
  }
  if( currentTime - square.positionGoodSinceMillis < SQUARE_POSITION_DWELL_MS ) {
    return;
  }

  switch( kafenv.info.squareState ) {
    case SQUARE_LEG1 : squareAdvanceLeg( SQUARE_LEG2, S, S, SQUARE_ALTITUDE_M, currentTime ); break;
    case SQUARE_LEG2 : squareAdvanceLeg( SQUARE_LEG3, 0, S, SQUARE_ALTITUDE_M, currentTime ); break;
    case SQUARE_LEG3 : squareAdvanceLeg( SQUARE_LEG4, 0, 0, SQUARE_ALTITUDE_M, currentTime ); break;
    case SQUARE_LEG4 : {
      DPRINTF( "[H] Square: Back at origin, landing\n" );
      squareBeginLanding();
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
  //Generic Pi-to-ESP32 position/velocity/yaw setpoint, shared by both Qualification and Mine Search - the
  //Pi decides WHERE the drone should be (formation slot, orbit point, survey waypoint); this ESP32 only
  //tracks it via the existing POS_SETPOINT_MODE cascade, validates freshness/sequence, and never itself
  //decides where to go. Payload is fixed-size ( trajsetpoint below), not the raw variable-length float
  //array COM_SET_SETPT uses, so it can carry a sequence number and be rejected out-of-order.
  com_receiveMessage( COM_SET_TRAJSETPT, sizeof( trajsetpoint ), []( void** response, const void* content, const unsigned short len ) {
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    const trajsetpoint* setpt = ( const trajsetpoint* )content;
    if( setpt->sequence <= kafenv.cmd.setpointSeq && kafenv.cmd.setpointMillis != 0 ) {
      DPRINTF( "[H] Rejected Trajectory Setpoint: Sequence=%lu <= Last=%lu\n", setpt->sequence, kafenv.cmd.setpointSeq );
      return;
    }
    if( !isfinite( setpt->x ) || !isfinite( setpt->y ) || !isfinite( setpt->z ) || !isfinite( setpt->yaw ) ||
        !isfinite( setpt->vx ) || !isfinite( setpt->vy ) || !isfinite( setpt->vz ) ) {
      DPRINTF( "[H] Rejected Trajectory Setpoint: Non-finite value, Sequence=%lu\n", setpt->sequence );
      return;
    }
    DPRINTF( "[H] Accepted Trajectory Setpoint: Sequence=%lu, X=[ %.3f, %.3f, %.3f ], Yaw=%.3f, V=[ %.3f, %.3f, %.3f ]\n",
        setpt->sequence, setpt->x, setpt->y, setpt->z, setpt->yaw, setpt->vx, setpt->vy, setpt->vz );
    kafenv.info.triggerLock = 1;
    FLTSYNC;
    kafenv.cmd.setpoints[0] = setpt->x;
    kafenv.cmd.setpoints[1] = setpt->y;
    kafenv.cmd.setpoints[2] = setpt->z;
    kafenv.cmd.setpoints[3] = setpt->yaw;
    kafenv.cmd.setpointVelocity = { setpt->vx, setpt->vy, setpt->vz };
    kafenv.cmd.setpointSeq = setpt->sequence;
    kafenv.cmd.setpointMillis = millis();
    //Deliberately does NOT change kafenv.info.flightMode - receiving a setpoint updates the TARGET only.
    //Entering POS_SETPOINT_MODE (i.e. actually arming/moving toward it) is a separate, explicit
    //COM_SET_FLIGHTMODE decision, gated on estimation_positionValid() - see communication.cpp.
    kafenv.info.triggerLock = 0;
  } );
  //Phone-selected autonomy mode. Deliberately does nothing but set the field - no flightMode/actuation
  //change here, matching "mode selection must never automatically arm or launch a drone". Rejected while
  //already armed, so a mode swap can't happen mid-flight into a behavior the current flight state doesn't
  //expect.
  com_receiveMessage( COM_SET_AUTONOMYMODE, sizeof( STDBYTE ), []( void** response, const void* content, const unsigned short len ) {
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    const STDBYTE mode = *( const STDBYTE* )content;
    if( kafenv.info.actuation ) {
      DPRINTF( "[H] Rejected Set Autonomy Mode: Already armed\n" );
      return;
    }
    if( mode != AUTONOMY_MANUAL && mode != AUTONOMY_QUALIFICATION && mode != AUTONOMY_MINE_SEARCH && mode != AUTONOMY_SQUARE_TEST ) {
      DPRINTF( "[H] Rejected Set Autonomy Mode: Unknown mode %u\n", mode );
      return;
    }
    DPRINTF( "[H] Set Autonomy Mode: %u\n", mode );
    kafenv.info.autonomyMode = mode;
    if( mode == AUTONOMY_QUALIFICATION ) {
      kafenv.info.qualState = QUAL_BOOT;
      kafenv.info.qualRevolutions = 0;
    } else if( mode == AUTONOMY_SQUARE_TEST ) {
      kafenv.info.squareState = SQUARE_BOOT;
    }
  } );
  //Phone-set formation slot (0-3) - which position along the hover line / orbit phase-stagger this drone
  //uses. Rejected while armed, same reasoning as autonomy mode above.
  com_receiveMessage( COM_SET_FORMATIONSLOT, sizeof( STDBYTE ), []( void** response, const void* content, const unsigned short len ) {
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    const STDBYTE slot = *( const STDBYTE* )content;
    if( kafenv.info.actuation ) {
      DPRINTF( "[H] Rejected Set Formation Slot: Already armed\n" );
      return;
    }
    if( slot > 3 ) {
      DPRINTF( "[H] Rejected Set Formation Slot: %u out of range (0-3)\n", slot );
      return;
    }
    DPRINTF( "[H] Set Formation Slot: %u\n", slot );
    kafenv.info.formationSlot = slot;
  } );
  //High-level qualification command (QUALCMD_*) - the only way the phone actually moves a qualification
  //flight forward; see commander_qualificationCommand() for the state machine itself.
  com_receiveMessage( COM_SET_QUALCOMMAND, sizeof( STDBYTE ), []( void** response, const void* content, const unsigned short len ) {
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    commander_qualificationCommand( *( const STDBYTE* )content );
  } );
  //High-level Square Test command (SQUARECMD_*) - see commander_squareCommand().
  com_receiveMessage( COM_SET_SQUARECOMMAND, sizeof( STDBYTE ), []( void** response, const void* content, const unsigned short len ) {
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    commander_squareCommand( *( const STDBYTE* )content );
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
  //check for lost Pi setpoint stream or an invalid position estimate while autonomously flying by
  //position - same emergency-descent response as the ground-station-disconnect/low-battery check above,
  //reused rather than duplicated. Only applies to the two flight modes that depend on kafenv.state.x/
  //kafenv.cmd.setpoints being trustworthy; ACCEL_SETPOINT_MODE (attitude-only) doesn't need this. The
  //setpointMillis staleness check only applies outside Qualification: Qualification's trajectories come
  //from this ESP32's own FLIGHTPATH_* calls (commander_qualificationStep() below), never from
  //COM_SET_TRAJSETPT, so setpointMillis never advances during a qualification flight and would otherwise
  //make this fire immediately, every time, regardless of anything actually being wrong.
  if( commandMode == CMD_NOMINAL_MODE && kafenv.info.actuation ) {
    const STDBYTE flightModeBits = DEFAULT_MODES_MASK & kafenv.info.flightMode;
    if( flightModeBits == POS_SETPOINT_MODE || flightModeBits == TRAJECTORY_MODE ) {
      const bool checkSetpointStaleness = kafenv.info.autonomyMode != AUTONOMY_QUALIFICATION;
      const bool setpointStale = checkSetpointStaleness &&
          ( kafenv.cmd.setpointMillis == 0 || ( currentTime - kafenv.cmd.setpointMillis ) > SETPOINT_STALE_MS );
      if( !estimation_positionValid() || setpointStale ) {
        DPRINTF( "[H] Position/Setpoint Invalid During Autonomous Flight, Initiating Landing: "
            "PositionValid=%u, SetpointStale=%u, Time=%lu\n", estimation_positionValid(), setpointStale, currentTime );
        //Reuses each mode's own LAND command rather than a third copy of landing logic - this used to
        //hardcode kafenv.info.qualState = QUAL_LANDING unconditionally (via a CMD_DESCENT_MODE path
        //distinct from qualification/square's own CMD_NOMINAL_MODE|TRAJECTORY_MODE LAND), which left
        //squareState untouched and stale during a Square Test failsafe, and didn't match either state
        //machine's own idea of what "landing" looks like.
        if( kafenv.info.autonomyMode == AUTONOMY_QUALIFICATION ) {
          commander_qualificationCommand( QUALCMD_LAND );
        } else if( kafenv.info.autonomyMode == AUTONOMY_SQUARE_TEST ) {
          commander_squareCommand( SQUARECMD_LAND );
        } else {
          kafenv.info.triggerLock = 1;
          FLTSYNC;
          kafenv.info.flightMode = CMD_DESCENT_MODE | NULL_MODE;
          commander_setTrajectories( FLIGHTPATH_LAND, NULLPTR );
          kafenv.info.triggerLock = 0;
        }
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
  commander_qualificationStep( currentTime );
  commander_squareStep( currentTime );
  commander.lastTime = currentTime;
}