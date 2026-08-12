#include "pid_tuner.h"
#include "commander.h"
#include "../core/flight.h"
#include "../core/communication.h"

#if ALT_DEFINE
#define NAN 0.0F
#define isfinite( arg ) true
static float abs( float num );
static double abs( double num );
static void* memset( void* dest, int ch, size_t count );
static void* memcpy( void* dest, const void* src, size_t count );
static double sqrt( double arg );
static float sqrtf( float arg );
static float expf( float arg );
#else
#include <string.h>
#include <math.h>
#endif

#define TUNING_MODE_NONE       0
#define TUNING_MODE_START_SIM  1
#define TUNING_MODE_SIM        2
#define TUNING_MODE_START_PHYS 3
#define TUNING_MODE_PHYS       6
#define TUNING_MODE_START_SIML 4
#define TUNING_MODE_SIML       8
#define TUNING_MODE_EXIT       9

#define TUNER_SET0( D, N ) for( unsigned char N = 0; N < sizeof( D ) / sizeof( double ); N++ ) D[N] = 0
#define TUNER_ITRVEC13( N ) for( unsigned char N = 0; N < 13; N++ )
#define TUNER_TOSENSOR( S, C ) ITRVEC3( i ) S.f[i] = S.f[i] * C[i].gain + C[i].ofst
#define TUNER_ADJUSTVAL( A, R, M ) ( M * ( 1 - expf( R - A ) ) )
#define TUNER_GRAVITY 9.806

struct {
  struct {
    struct {
      float maxDX;
      float maxT;
      float overshootDX;
      float overshootT;
      float steadyDX;
      float steadyT;
      unsigned int iterateCount;
    } resp[6];
    struct {
      double mass;
      double moment[3];
      double mixing[16];
      double controlStep;
      double timeStep;
    } sim;
  } persist;
  STDBYTE tuningMode;
  STDBYTE tuningStep;
  STDBYTE tuningCount;
  bool( *previousEstimator )( coordinate* );
  struct {
    double state[13];
    double k[4][13];
    double accel[3];
    bool constrained[13];
  } sim;
  struct {
  } phys;
  struct {
    float overshootDX;
    float overshootT;
    float steadyDX;
    float steadyT;
    struct {
      float currTime;
      float initialDX;
      float initialErrVar;
      float peakPos;
      float peakNeg;
      float totalError;
      float totalVar;
    } utils;
  } resp;
} pidtuner;

static void resetResponseMonitor() {
  DPRINTF( "[T] Response Monitor Reset\n" );
  pidtuner.resp.overshootDX              = 0;
  pidtuner.resp.overshootT               = 0;
  pidtuner.resp.steadyDX                 = 0;
  pidtuner.resp.steadyT                  = 0;
  pidtuner.resp.utils.currTime           = 0;
  pidtuner.resp.utils.initialDX          = NAN;
  pidtuner.resp.utils.initialErrVar      = 0;
  pidtuner.resp.utils.peakPos            = -1e9;
  pidtuner.resp.utils.peakNeg            = 1e9;
  pidtuner.resp.utils.totalError         = 0;
  pidtuner.resp.utils.totalVar           = 0;
}

static void doResponseMonitor( const float measured, const float deltaT, coordinate* pid, unsigned char id ) {
  if( !isfinite( pidtuner.resp.utils.initialDX ) ) {
    pidtuner.resp.utils.initialDX = measured;
  }
  const float normalmeas = measured / pidtuner.resp.utils.initialDX;
  if( normalmeas > pidtuner.resp.utils.peakPos ) {
    pidtuner.resp.utils.peakPos = normalmeas;
    pidtuner.resp.overshootT = pidtuner.resp.utils.currTime;
  }
  if( normalmeas < pidtuner.resp.utils.peakNeg ) {
    pidtuner.resp.utils.peakNeg = normalmeas;
    pidtuner.resp.overshootT = pidtuner.resp.utils.currTime;
  }
  pidtuner.resp.utils.totalError += deltaT * normalmeas;
  const float averageError = pidtuner.resp.utils.totalError / ( pidtuner.resp.utils.currTime + deltaT );
  pidtuner.resp.steadyDX = abs( averageError );
  pidtuner.resp.utils.totalVar += deltaT * abs( normalmeas - averageError );
  if( pidtuner.resp.utils.totalVar / pidtuner.resp.utils.currTime > pidtuner.persist.resp[id].maxDX ) {
    pidtuner.resp.steadyT = pidtuner.resp.utils.currTime;
  }
  if( averageError < 1 ) {
    const float c1 = pidtuner.resp.utils.peakPos - 1;
    const float c2 = pidtuner.resp.steadyDX - pidtuner.resp.utils.peakNeg;
    pidtuner.resp.overshootDX = c1 > c2 ? c1 : c2;
  } else {
    const float c1 = pidtuner.resp.utils.peakPos - pidtuner.resp.steadyDX;
    const float c2 = 1 - pidtuner.resp.utils.peakNeg;
    pidtuner.resp.overshootDX = c1 > c2 ? c1 : c2;
  }
  DPRINTF( "[T] Response Monitor: T=%.3f Y=%.3f X=%.3f\n", pidtuner.resp.utils.currTime, measured, normalmeas );
  pidtuner.resp.utils.currTime += deltaT;
  if( pidtuner.resp.utils.currTime > pidtuner.persist.resp[id].maxT ) {
    DPRINTF( "[T] Response Measured: Overshoot=%.3f, Peak Time=%.3f, Error=%.3f, Steady Time=%.3f\n",
        pidtuner.resp.overshootDX, pidtuner.resp.overshootT, pidtuner.resp.steadyDX, pidtuner.resp.steadyT );
    DPRINTF( "[T] Response Required: Overshoot=%.3f, Peak Time=%.3f, Error=%.3f, Steady Time=%.3f\n", 
        pidtuner.persist.resp[id].overshootDX, pidtuner.persist.resp[id].overshootT, 
        pidtuner.persist.resp[id].steadyDX, pidtuner.persist.resp[id].steadyT );
    DPRINTF( "[T] Current PID Values: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid->Kp, pid->Ki, pid->Kd );
    const float Kp = 0.1F * pid->Kp;
    if( pidtuner.resp.overshootDX > pidtuner.persist.resp[id].overshootDX ) {
      const float delQ = TUNER_ADJUSTVAL( pidtuner.resp.overshootDX, pidtuner.persist.resp[id].overshootDX, Kp );
      pid->Ki -= delQ;
      pid->Kd += 0.1 * delQ;
      if( pid->Ki < 0 ) {
        pid->Ki = 0;
        pid->Kp -= delQ;
      }
    } else if( pidtuner.resp.overshootT > pidtuner.persist.resp[id].overshootT ) {
      const float delQ = TUNER_ADJUSTVAL( pidtuner.resp.overshootT, pidtuner.persist.resp[id].overshootT, Kp );
      pid->Kp += delQ;
      pid->Kd -= 0.1 * delQ;
      if( pid->Kd < 0 ) {
        pid->Kd = 0;
        pid->Ki += delQ;
      }
    } else if( pidtuner.resp.steadyT > pidtuner.persist.resp[id].steadyT ) {
      pid->Kd += 0.1 * TUNER_ADJUSTVAL( pidtuner.resp.steadyT, pidtuner.persist.resp[id].steadyT, Kp );
    } else if( pidtuner.resp.steadyDX > pidtuner.persist.resp[id].steadyDX ) {
      pid->Ki += TUNER_ADJUSTVAL( pidtuner.resp.steadyDX, pidtuner.persist.resp[id].steadyDX, Kp );
    } else {
      pidtuner.tuningCount = pidtuner.persist.resp[id].iterateCount;
    }
    DPRINTF( "[T] Adjusted PID Values: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid->Kp, pid->Ki, pid->Kd );
    if( ++pidtuner.tuningCount >= pidtuner.persist.resp[id].iterateCount ) {
      pidtuner.tuningStep++;
      pidtuner.tuningCount = 0;
    } else {
      pidtuner.tuningStep--;
    }
  }
}

static void vec3mult( coordinate* out, const double mat[9], const double in[3] ) {
  out->x = ( float )( mat[0] * in[0] + mat[3] * in[1] + mat[6] * in[2] );
  out->y = ( float )( mat[1] * in[0] + mat[4] * in[1] + mat[7] * in[2] );
  out->z = ( float )( mat[2] * in[0] + mat[5] * in[1] + mat[8] * in[2] );
}

static void simF( double dx[13], const double x[13] ) {
  const double dt = pidtuner.persist.sim.timeStep;
  double taut[4] = { 0, 0, 0, 0 };
  for( unsigned char i = 0; i < 4; i++ ) {
    for( unsigned char j = 0; j < 4; j++ ) {
      taut[i] += pidtuner.persist.sim.mixing[ i * 4 + j ] * kafenv.cmd.motors[j] * kafenv.cmd.motors[j];
    }
  }
  dx[ 0] = dt * ( x[ 3] );
  dx[ 1] = dt * ( x[ 4] );
  dx[ 2] = dt * ( x[ 5] );
  dx[ 3] = dt * (       2 * ( x[ 7] * x[ 9] + x[ 6] * x[ 8] )   * taut[3] / pidtuner.persist.sim.mass );
  dx[ 4] = dt * (       2 * ( x[ 8] * x[ 9] - x[ 6] * x[ 7] )   * taut[3] / pidtuner.persist.sim.mass );
  dx[ 5] = dt * ( ( 1 - 2 * ( x[ 7] * x[ 7] + x[ 8] * x[ 8] ) ) * taut[3] / pidtuner.persist.sim.mass - TUNER_GRAVITY );
  dx[ 6] = dt * ( 0.5F * ( -x[10] * x[ 7] - x[11] * x[ 8] - x[12] * x[ 9] ) );
  dx[ 7] = dt * ( 0.5F * (  x[10] * x[ 6] + x[12] * x[ 8] - x[11] * x[ 9] ) );
  dx[ 8] = dt * ( 0.5F * (  x[11] * x[ 6] - x[12] * x[ 7] + x[10] * x[ 9] ) );
  dx[ 9] = dt * ( 0.5F * (  x[12] * x[ 6] + x[11] * x[ 7] - x[10] * x[ 8] ) );
  dx[10] = dt * ( ( ( pidtuner.persist.sim.moment[1] - pidtuner.persist.sim.moment[2] ) * x[11] * x[12] + taut[0] ) / pidtuner.persist.sim.moment[0] );
  dx[11] = dt * ( ( ( pidtuner.persist.sim.moment[2] - pidtuner.persist.sim.moment[0] ) * x[10] * x[12] + taut[1] ) / pidtuner.persist.sim.moment[1] );
  dx[12] = dt * ( ( ( pidtuner.persist.sim.moment[0] - pidtuner.persist.sim.moment[1] ) * x[10] * x[11] + taut[2] ) / pidtuner.persist.sim.moment[2] );
}

static void simulationReset( imu* mpu ) {
  DPRINTF( "[T] Simulator Environment Reset\n" );
  TUNER_SET0( pidtuner.sim.state, i );
  pidtuner.sim.state[6] = 1;
  TUNER_SET0( pidtuner.sim.k[0], i );
  TUNER_SET0( pidtuner.sim.k[1], i );
  TUNER_SET0( pidtuner.sim.k[2], i );
  TUNER_SET0( pidtuner.sim.k[3], i );
  TUNER_SET0( pidtuner.sim.accel, i );
  TUNER_ITRVEC13( i ) pidtuner.sim.constrained[i] = false;
  resetResponseMonitor();
  if( mpu != NULLPTR ) {
    mpu->accelUpdate = false;
    mpu->gyroUpdate = false;
    mpu->magUpdate = false;
    mpu->timeStep = pidtuner.persist.sim.controlStep;
    FPFILL0( i, kafenv.cmd.setpoints );
    FPFILL0( i, kafenv.cmd.motors );
    ITRVEC3( i ) kafenv.state.x.f[i] = 0;
    ITRVEC3( i ) kafenv.state.v.f[i] = 0;
    ITRVEC3( i ) kafenv.state.q.f[i] = 0;
    ITRVEC3( i ) kafenv.state.w.f[i] = 0;
    flight_reset();
    flight_positionEstimator( []( coordinate* coord ) {
      ITRVEC3( i ) coord->f[i] = (float)pidtuner.sim.state[i];
      return true;
    } );
  }
}

static void simulationStep( imu* mpu ) {
  double dt = 0;
  ITRVEC3( i ) pidtuner.sim.accel[i] = pidtuner.sim.state[ 3 + i ];
  const float f = pidtuner.sim.state[12];
  for( ; dt < pidtuner.persist.sim.controlStep; dt += pidtuner.persist.sim.timeStep ) {
    simF( pidtuner.sim.k[0], pidtuner.sim.state );
    double x[13];
    TUNER_ITRVEC13( i ) x[i] = pidtuner.sim.state[i] + 0.5 * pidtuner.sim.k[0][i];
    simF( pidtuner.sim.k[1], x );
    TUNER_ITRVEC13( i ) x[i] = pidtuner.sim.state[i] + 0.5 * pidtuner.sim.k[1][i];
    simF( pidtuner.sim.k[2], x );
    TUNER_ITRVEC13( i ) x[i] = pidtuner.sim.state[i] + pidtuner.sim.k[2][i];
    simF( pidtuner.sim.k[3], x );
    TUNER_ITRVEC13( i ) {
      if( !pidtuner.sim.constrained[i] ) {
        pidtuner.sim.state[i] += ( pidtuner.sim.k[0][i] + 2 * pidtuner.sim.k[1][i] + 2 * pidtuner.sim.k[2][i] + pidtuner.sim.k[3][i] ) / 6;
      }
    }
  }
  const double qsum = sqrt( pidtuner.sim.state[6] * pidtuner.sim.state[6] + 
    pidtuner.sim.state[7] * pidtuner.sim.state[7] + pidtuner.sim.state[8] * pidtuner.sim.state[8] +
    pidtuner.sim.state[9] * pidtuner.sim.state[9] );
  for( unsigned char i = 6; i < 10; i++ ) {
    pidtuner.sim.state[i] = pidtuner.sim.state[i] / qsum;
  }
  ITRVEC3( i ) pidtuner.sim.accel[i] = ( pidtuner.sim.state[ 3 + i ] - pidtuner.sim.accel[i] ) / dt;
  const double r = pidtuner.sim.state[6];
  const double x = pidtuner.sim.state[7];
  const double y = pidtuner.sim.state[8];
  const double z = pidtuner.sim.state[9];
  const double R[] = { 2 * ( 0.5 - y * y - z * z ), 2 * (       x * y - r * z ), 2 * (       x * z + r * y ),
                       2 * (       x * y + r * z ), 2 * ( 0.5 - x * x - z * z ), 2 * (       y * z - r * x ),
                       2 * (       x * z - r * y ), 2 * (       y * z + r * x ), 2 * ( 0.5 - x * x - y * y ) };
  vec3mult( &mpu->accelInput, R, pidtuner.sim.accel );
  const double northVector[3] = { 0, 1, 0 };
  vec3mult( &mpu->magInput, R, northVector );
  mpu->gyroInput = { ( float )pidtuner.sim.state[10], ( float )pidtuner.sim.state[11], ( float )pidtuner.sim.state[12] };
  TUNER_TOSENSOR( mpu->gyroInput, kafenv.cal.gyrofilt );
  TUNER_TOSENSOR( mpu->accelInput, kafenv.cal.accelfilt );
  TUNER_TOSENSOR( mpu->magInput, kafenv.cal.gyrofilt );
  mpu->gyroUpdate = true;
  mpu->accelUpdate = true;
  mpu->magUpdate = false;
  mpu->timeStep = ( float )dt;
  DPRINTF( "[T] Simulation Step: DT=%.3f, SimT=%.3f\n", dt, pidtuner.persist.sim.timeStep );
  DPRINTF( "[T] Simulation Step: X=[ %.3f, %.3f, %.3f ]\n", pidtuner.sim.state[0], pidtuner.sim.state[1], pidtuner.sim.state[2] );
  DPRINTF( "[T] Simulation Step: V=[ %.3f, %.3f, %.3f ]\n", pidtuner.sim.state[3], pidtuner.sim.state[4], pidtuner.sim.state[5] );
  DPRINTF( "[T] Simulation Step: A=[ %.3f, %.3f, %.3f ]\n", pidtuner.sim.accel[0], pidtuner.sim.accel[1], pidtuner.sim.accel[2] );
  DPRINTF( "[T] Simulation Step: Q=[ %.3f, %.3f, %.3f, %.3f ]\n", 
      pidtuner.sim.state[6], pidtuner.sim.state[7], pidtuner.sim.state[8], pidtuner.sim.state[9] );
  DPRINTF( "[T] Simulation Step: W=[ %.3f, %.3f, %.3f ]\n", pidtuner.sim.state[10], pidtuner.sim.state[11], pidtuner.sim.state[12] );
}

void pindtuner_simulationState( double matrix[13] ) {
  for( unsigned char i = 0; i < 13; i++ ) {
    matrix[i] = pidtuner.sim.state[i];
  }
}

peripheral pidtuner_reset() {
  DPRINTF( "[T] Resetting PID Tuner\n" );
  pidtuner.tuningMode = TUNING_MODE_NONE;
  pidtuner.tuningStep = MAXBYTE;
  pidtuner.tuningCount = 0;
  pidtuner.previousEstimator = NULLPTR;
  pidtuner.persist.sim.mass = 0.37129;
  pidtuner.persist.sim.moment[0] = 0.00080338097;
  pidtuner.persist.sim.moment[1] = 0.0001414958;
  pidtuner.persist.sim.moment[2] = 0.00109898711;
  pidtuner.persist.sim.mixing[ 0] =  0.085; //pitch
  pidtuner.persist.sim.mixing[ 1] =  0.085; //pitch
  pidtuner.persist.sim.mixing[ 2] = -0.085; //pitch
  pidtuner.persist.sim.mixing[ 3] = -0.085; //pitch
  pidtuner.persist.sim.mixing[ 4] = -0.374; //roll
  pidtuner.persist.sim.mixing[ 5] =  0.374; //roll
  pidtuner.persist.sim.mixing[ 6] = -0.374; //roll
  pidtuner.persist.sim.mixing[ 7] =  0.374; //roll
  pidtuner.persist.sim.mixing[ 8] = -0.002; //yaw
  pidtuner.persist.sim.mixing[ 9] =  0.002; //yaw
  pidtuner.persist.sim.mixing[10] =  0.002; //yaw
  pidtuner.persist.sim.mixing[11] = -0.002; //yaw
  pidtuner.persist.sim.mixing[12] =  3.198; //thrust
  pidtuner.persist.sim.mixing[13] =  3.198; //thrust
  pidtuner.persist.sim.mixing[14] =  3.198; //thrust
  pidtuner.persist.sim.mixing[15] =  3.198; //thrust
  pidtuner.persist.sim.controlStep = 0.01;
  pidtuner.persist.sim.timeStep = 0.0005;
  //                   i      mx     mt     ox     ot      sx     st      l
  pidtuner.persist.resp[0] = { 0.05F, 10.0F, 0.10F, 0.100F, 0.01F, 0.500F, 10 }; //v
  pidtuner.persist.resp[1] = { 0.05F, 10.0F, 0.15F, 0.150F, 0.03F, 1.200F, 30 }; //x
  pidtuner.persist.resp[2] = { 0.05F, 10.0F, 0.15F, 0.130F, 0.05F, 0.900F, 10 }; //w2
  pidtuner.persist.resp[3] = { 0.05F, 10.0F, 0.04F, 0.080F, 0.01F, 0.300F, 20 }; //w0
  pidtuner.persist.resp[4] = { 0.05F, 10.0F, 0.04F, 0.080F, 0.01F, 0.300F, 20 }; //w1
  pidtuner.persist.resp[5] = { 0.05F, 10.0F, 0.08F, 0.160F, 0.02F, 0.600F, 10 }; //q
  simulationReset( NULLPTR );
  com_receiveMessage( COM_SET_PIDTUNING, 1, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[T] Replying Set PID Tuning Command\n" );
    STDBYTE mode = *( ( STDBYTE* )content );
    if( mode != TUNING_MODE_START_SIM && mode != TUNING_MODE_START_PHYS && mode != TUNING_MODE_START_SIML && mode != TUNING_MODE_EXIT ) {
      *response = NULLPTR;
    }
    pidtuner.tuningMode = mode;
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) { } );
  com_receiveMessage( COM_SET_SIM_VARS, sizeof( pidtuner.persist.sim ), 
      []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[T] Replying Set PID Tuning Simulation Variables\n" );
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[T] Executing Set Phyical PID Tuning Command\n" );
    memcpy( &pidtuner.persist.sim, content, sizeof( pidtuner.persist.sim ) );
  } ); 
  com_receiveMessage( COM_SET_RESP_VARS, sizeof( pidtuner.persist.resp ), 
      []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[T] Replying Set PID Tuning Response Configuration\n" );
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[T] Executing Set PID Tuning Response Configuration\n" );
    memcpy( &pidtuner.persist.resp, content, sizeof( pidtuner.persist.resp ) );
  } );
  return { "pidtuner", sizeof( pidtuner.persist ), sizeof( pidtuner ), &pidtuner, [](){ pidtuner_reset(); }, NULLPTR };
}

void pidtuner_step( void* imuData ) {
  imu* mpu = ( imu* )imuData;
  switch( pidtuner.tuningMode ) {
    case TUNING_MODE_START_SIM : case TUNING_MODE_START_PHYS : case TUNING_MODE_START_SIML : {
      DPRINTF( "[T] Starting PID Tuning: Mode=%u\n", pidtuner.tuningMode );
      pidtuner.tuningMode = pidtuner.tuningMode * 2;
      pidtuner.tuningStep = 0;
      pidtuner.tuningCount = 0;
      pidtuner.previousEstimator = ( bool( * )( coordinate* ) )flight_positionEstimator( NULLPTR );
      break;
    }
    case TUNING_MODE_SIML : {
      DPRINTF( "[T] Simulation Mode: Step=%u\n", pidtuner.tuningStep );
      switch( pidtuner.tuningStep ) {
        case 1 : {
          DPRINTF( "[T] Stepping Simulation\n" );
          simulationStep( mpu );
          break;
        }
        default : {
          DPRINTF( "[T] Reset Simulation\n" );
          simulationReset( mpu );
          pidtuner.tuningStep = 1;
        }
      }
      break;
    }
    case TUNING_MODE_SIM : {
      DPRINTF( "[T] Simulation PID Tuning Mode: Step=%u\n", pidtuner.tuningStep );
      switch( pidtuner.tuningStep ) {
        case 0 : {
          DPRINTF( "[T] Resetting PID Values\n" );
          kafenv.cal.xpid = { 1, 0, 0 };
          kafenv.cal.vpid = { 1, 0, 0 };
          kafenv.cal.apid = { 1, 0, 0 };
          kafenv.cal.qpid = { 0, 0, 0 };
          ITRVEC3( i ) kafenv.cal.wpid[i] = { 0, 0, 0 };
          pidtuner.tuningStep = 1;
          break;
        }
        case 1 : {
          DPRINTF( "[T] Reset Simulation For Acceleration PID Tuner\n" );
          simulationReset( mpu );
          for( unsigned char i = 6; i < 13; i++ ) pidtuner.sim.constrained[i] = true;
          pidtuner.tuningStep = 2;
          break;
        }
        case 2 : {
          DPRINTF( "[T] Stepping Simulation Acceleration PID Tuner\n" );
          simulationStep( mpu );
          kafenv.info.flightMode = CMD_NULL_MODE | ACCEL_SETPOINT_MODE;
          doResponseMonitor( pidtuner.sim.accel[2], ( float )pidtuner.persist.sim.controlStep, &kafenv.cal.apid, 0 );
          break;
        }
        case 3 : {
          DPRINTF( "[T] Reset Simulation For Velocity PID Tuner\n" );
          simulationReset( mpu );
          for( unsigned char i = 6; i < 13; i++ ) pidtuner.sim.constrained[i] = true;
          pidtuner.tuningStep = 4;
          break;
        }
        case 4 : {
          DPRINTF( "[T] Stepping Simulation Velocity PID Tuner\n" );
          simulationStep( mpu );
          kafenv.info.flightMode = CMD_NULL_MODE | POS_SETPOINT_MODE;
          kafenv.cmd.setpoints[2] = kafenv.state.x.z + kafenv.state.v.z * mpu->timeStep + 1;
          doResponseMonitor( 1 - pidtuner.sim.state[5], ( float )pidtuner.persist.sim.controlStep, &kafenv.cal.vpid, 0 );
          break;
        }
        case 5 : {
          DPRINTF( "[T] Reset Simulation for Position PID Tuner\n" );
          simulationReset( mpu );
          for( unsigned char i = 6; i < 13; i++ ) pidtuner.sim.constrained[i] = true;
          kafenv.cmd.setpoints[2] = 1;
          pidtuner.tuningStep = 6;
          break;
        }
        case 6 : {
          DPRINTF( "[T] Stepping Simulation Position PID Tuner\n" );
          simulationStep( mpu );
          kafenv.info.flightMode = CMD_NULL_MODE | POS_SETPOINT_MODE;
          doResponseMonitor( 1 - pidtuner.sim.state[2], ( float )pidtuner.persist.sim.controlStep, &kafenv.cal.xpid, 1 );
          break;
        }
        case 7 : {
          DPRINTF( "[T] Reset Simulation for Yaw Rate PID Tuner\n" );
          simulationReset( mpu );
          for( unsigned char i = 10; i < 12; i++ ) pidtuner.sim.constrained[i] = true;
          pidtuner.sim.state[12] = 1;
          kafenv.cal.qpid = { 1, 0, 0 };
          kafenv.cal.wpid[2] = { 1, 0, 0 };
          pidtuner.tuningStep = 8;
          break;
        }
        case 8 : {
          DPRINTF( "[T] Stepping Simulation Yaw Rate PID Tuner\n" );
          simulationStep( mpu );
          kafenv.info.flightMode = CMD_NULL_MODE | ACCEL_SETPOINT_MODE;
          doResponseMonitor( pidtuner.sim.state[12], ( float )pidtuner.persist.sim.controlStep, &kafenv.cal.wpid[2], 2 );
          break;
        }
        case 9 : {
          DPRINTF( "[T] Reset Simulation for Roll Rate PID Tuner\n" );
          simulationReset( mpu );
          for( unsigned char i = 11; i < 13; i++ ) pidtuner.sim.constrained[i] = true;
          pidtuner.sim.state[10] = 1;
          kafenv.cal.qpid = { 1, 0, 0 };
          kafenv.cal.wpid[0] = { 1, 0, 0 };
          pidtuner.tuningStep = 10;
          break;
        }
        case 10 : {
          DPRINTF( "[T] Stepping Simulation Roll Rate PID Tuner\n" );
          simulationStep( mpu );
          kafenv.info.flightMode = CMD_NULL_MODE | ACCEL_SETPOINT_MODE;
          doResponseMonitor( pidtuner.sim.state[10], ( float )pidtuner.persist.sim.controlStep, &kafenv.cal.wpid[0], 3 );
          break;
        }
        case 11 : {
          DPRINTF( "[T] Reset Simulation for Pitch Rate PID Tuner\n" );
          simulationReset( mpu );
          for( unsigned char i = 10; i < 13; i += 2 ) pidtuner.sim.constrained[i] = true;
          pidtuner.sim.state[11] = 1;
          kafenv.cal.qpid = { 1, 0, 0 };
          kafenv.cal.wpid[1] = { 1, 0, 0 };
          pidtuner.tuningStep = 12;
          break;
        }
        case 12 : {
          DPRINTF( "[T] Stepping Simulation Pitch Rate PID Tuner\n" );
          simulationStep( mpu );
          kafenv.info.flightMode = CMD_NULL_MODE | ACCEL_SETPOINT_MODE;
          doResponseMonitor( pidtuner.sim.state[11], ( float )pidtuner.persist.sim.controlStep, &kafenv.cal.wpid[1], 4 );
          break;
        }
        case 13 : {
          DPRINTF( "[T] Reset Simulation for Angle PID Tuner\n" );
          simulationReset( mpu );
          for( unsigned char i = 11; i < 13; i++ ) pidtuner.sim.constrained[i] = true;
          pidtuner.sim.state[6] = 0;
          pidtuner.sim.state[9] = 1;
          kafenv.cal.qpid = { 1, 0, 0 };
          pidtuner.tuningStep = 14; 
          break;         
        }
        case 14 : {
          DPRINTF( "[T] Stepping Simulation Angle PID Tuner\n" );
          simulationStep( mpu );
          kafenv.info.flightMode = CMD_NULL_MODE | ACCEL_SETPOINT_MODE;
          doResponseMonitor( pidtuner.sim.state[9], ( float )pidtuner.persist.sim.controlStep, &kafenv.cal.qpid, 5 );
          break;
        }
        default : {
          DPRINTF( "[T] Exiting Simulation PID Tuner\n" );
          kafenv.info.flightMode = CMD_NULL_MODE | NULL_MODE;
          pidtuner.tuningMode = TUNING_MODE_EXIT;
        }
      }
      kafenv.info.actuation = false;
      break;
    }
    case TUNING_MODE_PHYS : {
      DPRINTF( "[T] Physical PID Tuning Mode: Step=%u\n", pidtuner.tuningStep );
      switch( pidtuner.tuningStep ) {
        default : {
          DPRINTF( "[T] Exiting Physical PID Tuner\n" );
          kafenv.info.flightMode = CMD_NULL_MODE | NULL_MODE;
          pidtuner.tuningMode = TUNING_MODE_EXIT;
          //Physical tuning exits fail closed. Selecting/preparing a landing path never grants authority to
          //arm; a later explicit arm command must pass the normal commander guard.
          commander_setTrajectories( FLIGHTPATH_LAND, NULLPTR );
          kafenv.info.actuation = false;
        }
      }
      break;
    }
    case TUNING_MODE_EXIT : {
      DPRINTF( "[T] PID Tuning Exit\n" );
      flight_positionEstimator( pidtuner.previousEstimator );
      pidtuner.tuningMode = TUNING_MODE_NONE;
      pidtuner.tuningStep = 0;
      pidtuner.previousEstimator = NULLPTR;
      kafenv.info.flightMode = CMD_NOMINAL_MODE | NULL_MODE;
      kafenv.info.actuation = false;
      break;
    }
    default : {
      //DEBUG logging throttled to ~1/s - see periph_freertos.cpp for why
      static unsigned long lastInactivePrint = 0;
      if( millis() - lastInactivePrint > 1000 ) {
        lastInactivePrint = millis();
        DPRINTF( "[T] PID Tuning Inactive\n" );
      }
    }
  }
}
