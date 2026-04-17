#include "flight.h"

#if ALT_DEFINE
#define NAN           0
#define sqrtf( N )    0
#define isfinite( N ) true
#define fmodf( N, M ) 0
#define atan2f( N )   0
#define cosf( N )     0
#define sinf( N )     0
#define expf( N )     0
#else
#include <math.h>
#endif

/*       ^ +y
     O1  |  O0  
       \___/
 -x <- |___| -> +x
       /   \
     O3  |  O2
         v -y    
 cosz, -sinz,     0
 sinz,  cosz,     0
   0,      0,     1
1 - qy * qy,     qx * qy,     qc * qy 
    qx * qy, 1 - qx * qx,    -qc * qx
   -qc * qy,     qc * qx, 1 - qx * qx - qy * qy
*/

#define BOUND( V, LB, UB ) V > UB ? UB : ( V < LB ? LB : V )
#define PI 3.14159265359F //pi
#define TU 6.28318530718F //2*pi
#define S2 1.41421356237F //sqrt(2)
#define S8 0.35355339059F //sqrt(2)/4
#define P2 9.86960440109F //pi^2
#define AF 13.1000000000F //alpha
#define EF 1e-14F         //epsilon

//simple calibrated filter definition
struct measfilter {
  unsigned int count;
  float total;
  float deviation;
  float previous;
};

//low pass butterworth filter definition
struct lowpass {
  float internal;
  float lowpass;
};

//pid definition
struct pid {
  float intlimit;
  float outlimit;
  float integ;
  float prevmeas;
  lowpass deriv;
};

static struct {//DRONE CONTROL DATA
  float rotMat[9];
  unsigned int calibTimeCount = 0;
  float calibTimeTotal = 0;
  float posstep = 0;
  float yawOffset = 0;
  float estimateTime = 0;
  coordinate estimateGain;
  coordinate acceleration;
  pid positionPID[3];
  pid velocityPID[3];
  pid attitudePID[3];
  pid attiratePID[3];
  lowpass gyroLowpass[3];
  measfilter accelCalib[3];
  measfilter gyroCalib[3];
  measfilter magCalib[3];
  measfilter sensorCalib[ FPARLEN( kafenv.cal.sensefilt ) ];
  bool( *estimator )( coordinate* );
  void( *flightInvoker )();
} flight;

static void lowpassReset( lowpass* lowpass ) {
  lowpass->internal = NAN;
  lowpass->lowpass = NAN;
}

static float lowpassStep( lowpass* lowpass, const float measured ) {
  lowpass->internal -= S2 * ( lowpass->internal + S8 * ( lowpass->lowpass - measured ) );
  FPGUARD( lowpass->internal, 0 );
  lowpass->lowpass += lowpass->internal;
  FPGUARD( lowpass->lowpass, measured );
  return lowpass->lowpass;
}

static void pidReset( pid* pid ) {
  pid->integ    = NAN;
  pid->prevmeas = NAN;
  lowpassReset( &pid->deriv );
}

static float pidStep( const coordinate* pidval, pid* pid, const float desired, const float measured, const float dt ) {
  const float error = desired - measured;
  const float deriv = lowpassStep( &pid->deriv, ( pid->prevmeas - measured ) / dt );
  pid->prevmeas = measured;
  pid->integ = pid->integ + error * dt;
  FPGUARD( pid->integ, 0 );
  pid->integ = BOUND( pid->integ, -pid->intlimit, pid->intlimit );
  const float output = pidval->Kp * error + pidval->Ki * pid->integ + pidval->Kd * deriv;
  return BOUND( output, -pid->outlimit, pid->outlimit );
}

static void filterReset( measfilter* filter ) {
  filter->count = 0;
  filter->total = 0;
  filter->deviation = 0;
  filter->previous = 0;
}

static float filterStep( const coordinate* filterval, measfilter* filter, const float measured ) {
  float corrected = measured - filterval->ofst;
  const float prevMeas = isfinite( filter->previous ) ? filter->previous : corrected;
  const float error = corrected - prevMeas;
  float sig = error * error;
  sig = sig * sig * sig;
  corrected = prevMeas + ( sig / ( sig + filterval->stdv + EF ) ) * error;
  filter->previous = corrected;
  return filterval->gain * corrected;
}

static float filterCalibrate( coordinate* filterval, measfilter* filter, const float measured ) {
  filter->count++;
  filter->total += measured;
  filterval->ofst = filter->total / filter->count;
  const float measDelta = measured - filterval->ofst;
  filter->deviation += measDelta * measDelta;
  const float sig2 = filter->deviation / filter->count;
  filterval->stdv = P2 * sig2 * sig2 * sig2;
  filter->previous = measured;
  return sig2;
}

static void positionEstimate( const bool estimate, const float dt ) {
  if( estimate ) {
    coordinate position;
    if( flight.estimator( &position ) ) {
      const float alpha2 = kafenv.cal.positionalpha * kafenv.cal.positionalpha;
      ITRVEC3( i ) flight.estimateGain.f[i] = alpha2 * position.f[i];
      flight.estimateTime = -0.5F * kafenv.cal.positionalpha * dt;
    }
  } else {
    flight.estimateTime -= kafenv.cal.positionalpha * dt;
    if( flight.estimateTime > 10 ) {
      flight.estimateGain = { 0, 0, 0 };
    }
  }
  const float accel = ( 1 + flight.estimateTime ) * expf( flight.estimateTime );
  ITRVEC3( i ) {
    kafenv.state.v.f[i] += dt * ( flight.acceleration.f[i] + flight.estimateGain.f[i] * accel );
    kafenv.state.x.f[i] += dt * kafenv.state.v.f[i];
  }
  DPRINTF( "[F] Updated Velocity: V=[ %.3f, %.3f, %.3f ]\n", kafenv.state.v.x, kafenv.state.v.y, kafenv.state.v.z );
  DPRINTF( "[F] Updated Position: X=[ %.3f, %.3f, %.3f ]\n", kafenv.state.x.x, kafenv.state.x.y, kafenv.state.x.z );
}

void flight_rotationMatrix( float matrix[9] ) {
  for( unsigned char i = 0; i < 9; i++ ) {
    matrix[i] = flight.rotMat[i];
  }
}

void flight_setPositionEstimator( bool( *estimator )( coordinate* ) ) {
  if( estimator != NULLPTR ) { 
    flight.estimator = estimator;
  }
}

void flight_runFunction( void( *function )() ) {
  flight.flightInvoker = function;
}

float flight_calibrateSensor( const STDBYTE id, const float value ) {
  return id < FPARLEN( kafenv.cal.sensefilt ) ? kafenv.cal.sensefilt[id].gain *
      sqrtf( filterCalibrate( &kafenv.cal.sensefilt[id], &flight.sensorCalib[id], value ) ) : NAN;
}

float flight_filterSensor( const STDBYTE id, const float value ) {
  return id < FPARLEN( kafenv.cal.sensefilt ) ? filterStep( &kafenv.cal.sensefilt[id], &flight.sensorCalib[id], value ) : NAN;
}

void flight_attitudeEstimate( const imu* sensor ) {
  const float dt = sensor->timeStep;
  const float qc2 = 2 - kafenv.state.q.x * kafenv.state.q.x - kafenv.state.q.y * kafenv.state.q.y;
  float qc = sqrtf( qc2 < 0 ? 0 : qc2 );
  //update gyro
  if( sensor->gyroUpdate ) {
    ITRVEC3( i ) kafenv.state.w.f[i] = lowpassStep( &flight.gyroLowpass[i], 
        filterStep( &kafenv.cal.gyrofilt[i], &flight.gyroCalib[i], sensor->gyroInput.f[i] ) );
    const float dt2 = 0.5F * dt;
    const float dt2qc = dt2 * qc;
    kafenv.state.q.x += dt2qc * kafenv.state.w.x;
    kafenv.state.q.y += dt2qc * kafenv.state.w.y;
    kafenv.state.q.z += dt * kafenv.state.w.z;
    qc -= dt2 * ( kafenv.state.q.x * kafenv.state.w.x + kafenv.state.q.y * kafenv.state.w.y );
  }
  //update accel
  coordinate a = { 0, 0, 0 };
  if( sensor->accelUpdate ) {
    ITRVEC3( i ) a.f[i] = filterStep( &kafenv.cal.accelfilt[i], &flight.accelCalib[i], sensor->accelInput.f[i] );
    const float g = kafenv.cal.accelfilt[2].ofst * kafenv.cal.accelfilt[2].gain;
    const float g2 = g * g;
    const float ax2ay2 = a.x * a.x + a.y * a.y;
    const float cos2 = ( a.z + g < 0 ? -1 : 1 ) * sqrtf( g2 < ax2ay2 ? 0 : g2 - ax2ay2 ) / ( g + EF ) + 1;
    const float cos = sqrtf( cos2 < 0 ? 0 : cos2 );
    qc += kafenv.cal.anglealpha * ( cos - qc );
    const float am = sqrtf( ( cos2 > 2 ? 0 : 2 - cos2 ) / ( ax2ay2 + EF ) );
    const float dqx = kafenv.cal.anglealpha * ( -a.y * am - kafenv.state.q.x );
    const float dqy = kafenv.cal.anglealpha * (  a.x * am - kafenv.state.q.y );
    kafenv.state.q.x += dqx;
    kafenv.state.q.y += dqy;
    const float invdt2qc = 2 / dt / qc;
    if( !sensor->gyroUpdate ) {
      kafenv.state.w.x = lowpassStep( &flight.gyroLowpass[0], kafenv.state.w.x + dqx * invdt2qc );
      kafenv.state.w.y = lowpassStep( &flight.gyroLowpass[1], kafenv.state.w.y + dqy * invdt2qc );
    }
  }
  //update magnetometer
  if( sensor->magUpdate ) {
    coordinate m;
    ITRVEC3( i ) m.f[i] = filterStep( &kafenv.cal.magfilt[i], &flight.magCalib[i], sensor->magInput.f[i] );
    const float mz2 = m.z * m.z;
    const float confidence = kafenv.cal.anglealpha * ( 1 - mz2 / ( m.x * m.x + m.y * m.y + mz2 + EF ) );
    const float dqz = fmodf( atan2f( m.y, m.x + EF ) - flight.yawOffset - kafenv.state.q.z, TU ) * confidence;
    kafenv.state.q.z += dqz;
    if( !sensor->gyroUpdate ) {
      kafenv.state.w.z = lowpassStep( &flight.gyroLowpass[2], kafenv.state.w.z + dqz / dt );
    }
  }
  //normalize quaternion aspects of rotation
  const float invqm = S2 / ( sqrtf( qc * qc + kafenv.state.q.x * kafenv.state.q.x + kafenv.state.q.y * kafenv.state.q.y ) + EF );
  kafenv.state.q.x *= invqm;
  kafenv.state.q.y *= invqm;
  DPRINTF( "[F] Updated Angular Rate: W=[ %.3f, %.3f, %.3f ]\n", kafenv.state.w.x, kafenv.state.w.y, kafenv.state.w.z );
  DPRINTF( "[F] Updated Attitude: Q=[ %.3f, %.3f, %.3f, 0 ], Yaw=%.3f\n", 
      qc * invqm / S2, kafenv.state.q.x / S2, kafenv.state.q.y / S2, kafenv.state.q.z );
  //create rotation matrix
  const float sinz = sinf( kafenv.state.q.z );
  const float cosz = cosf( kafenv.state.q.z );
  const float qx = kafenv.state.q.x * cosz - kafenv.state.q.y * sinz;
  const float qy = kafenv.state.q.x * sinz + kafenv.state.q.y * cosz;
  const float q1xx = 1 - qx * qx;
  const float qyy = qy * qy;
  const float qxy = qx * qy;
  const float qcx = qc * qx;
  const float qcy = qc * qy;
  const float q1yy = 1 - qyy;
  flight.rotMat[0] = cosz * q1yy - sinz * qxy;
  flight.rotMat[1] = sinz * q1yy + cosz * qxy;
  flight.rotMat[2] = qcy;
  flight.rotMat[3] = cosz * qxy - sinz * q1xx;
  flight.rotMat[4] = sinz * qxy + cosz * q1xx;
  flight.rotMat[5] = -qcx;
  flight.rotMat[6] = -cosz * qcy - sinz * qcx;
  flight.rotMat[7] = -sinz * qcy + cosz * qcx;
  flight.rotMat[8] = q1xx - qyy;
  DPRINTF( "[F] Updated Rotation Matrix: R=[ %.3f, %.3f, %.3f; %.3f, %.3f, %.3f; %.3f, %.3f, %.3f ]\n", flight.rotMat[0], flight.rotMat[1], 
      flight.rotMat[2], flight.rotMat[3], flight.rotMat[4], flight.rotMat[5], flight.rotMat[6], flight.rotMat[7], flight.rotMat[8] );
  //update acceleration
  ITRVEC3( i ) flight.acceleration.f[i] = flight.rotMat[ i ] * a.x + flight.rotMat[ i + 1 ] * a.y + flight.rotMat[ i + 2 ] * a.z;
  DPRINTF( "[F] Updated Acceleration: A=[ %.3f, %.3f, %.3f ]\n", flight.acceleration.x, flight.acceleration.y, flight.acceleration.z );
}

void flight_attitudeControl( const coordinate a, const float tk, float dt ) {
  // calculate acceleration vector magnitude
  DPRINTF( "[F] Acceleration Setpoint: A=[ %.3f, %.3f, %.3f ]\n", a.x, a.y, a.z );
  const float am = 1 / ( sqrtf( a.x * a.x + a.y * a.y + a.z * a.z ) + EF );
  // convert from world frame to body frame acceleration setpoint
  const float az = flight.rotMat[2] * a.x + flight.rotMat[5] * a.y + flight.rotMat[8] * a.z;
  const float st = az < 0 ? 0 : az;
  const float b0 = am / ( sqrtf( az * am * 0.5F + 0.5F ) + EF );
  float sp[] = { -b0 * ( flight.rotMat[1] * a.x + flight.rotMat[4] * a.y + flight.rotMat[7] * a.z ), 
                  b0 * ( flight.rotMat[0] * a.x + flight.rotMat[3] * a.y + flight.rotMat[6] * a.z ), 
                  tk - kafenv.state.q.z };
  DPRINTF( "[F] Control State Setpoint: Thrust=%.3f, QST=[ %.3f, %.3f ], Yaw=%.3f\n", st, sp[0], sp[1], sp[2] );
  // pid control step
  ITRVEC3( i ) sp[i] = pidStep( &( kafenv.cal.qpid ), &( flight.attitudePID[i] ), 0, -sp[i], dt );
  ITRVEC3( i ) sp[i] = pidStep( &( kafenv.cal.wpid[i] ), &( flight.attiratePID[i] ), sp[i], kafenv.state.w.f[i], dt );
  // set motor inputs
  kafenv.cmd.motors[0] = st + sp[0] - sp[1] - sp[2];
  kafenv.cmd.motors[1] = st + sp[0] + sp[1] + sp[2];
  kafenv.cmd.motors[2] = st - sp[0] - sp[1] + sp[2];
  kafenv.cmd.motors[3] = st - sp[0] + sp[1] - sp[2];
  for( unsigned char i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
    kafenv.cmd.motors[i] = BOUND( kafenv.cmd.motors[i], 0, 1 );
  }
  kafenv.info.actuation = true;
  DPRINTF( "[F] Attitude Control Motor Actuation: Motors=[ %.3f, %.3f, %.3f, %.3f ]\n",
      kafenv.cmd.motors[0], kafenv.cmd.motors[1], kafenv.cmd.motors[2], kafenv.cmd.motors[3] );
}

void flight_positionControl( const imu* sensor, const float position[4] ) {
  const float dt = sensor->timeStep;
  float setpoint[3];
  flight_attitudeEstimate( sensor );
  const bool doPosUpdate = ( flight.posstep += sensor->timeStep ) > 0.05F;
  positionEstimate( doPosUpdate, sensor->timeStep );
  if( doPosUpdate ) {
    ITRVEC3( i ) setpoint[i] = pidStep( &( kafenv.cal.xpid ), &( flight.positionPID[i] ), position[i], kafenv.state.x.f[i], flight.posstep );
    ITRVEC3( i ) setpoint[i] = pidStep( &( kafenv.cal.vpid ), &( flight.velocityPID[i] ), setpoint[i], kafenv.state.v.f[i], flight.posstep );
    flight.posstep = 0;
  }
  flight_attitudeControl( { setpoint[0], setpoint[1], setpoint[2] + kafenv.cal.gravitation }, position[3], dt );
}

memory flight_reset() {
  DPRINTF( "[F] Resetting Flight Software\n" );
  flight.calibTimeCount = 0;
  flight.calibTimeTotal = 0;
  flight.posstep = 0;
  flight.yawOffset = 0;
  flight.estimateTime = 0;
  flight.estimateGain = { 0, 0, 0 };
  flight.acceleration = { 0, 0, 0 };
  FPFILL0( i, flight.rotMat );
  ITRVEC3( i ) flight.positionPID[i].outlimit = 10;                // m/s max speed
  ITRVEC3( i ) flight.velocityPID[i].outlimit = 0.95F;             // %V  max accel
  ITRVEC3( i ) flight.attitudePID[i].outlimit = PI * 5;            // /s  max rot speed
  ITRVEC3( i ) flight.attiratePID[i].outlimit = 0.5F;              // %V  max volt change
  ITRVEC3( i ) flight.positionPID[i].intlimit = 10;                // m   * s
  ITRVEC3( i ) flight.velocityPID[i].intlimit = 25;                // m/s * s
  ITRVEC3( i ) flight.attitudePID[i].intlimit = PI / 3;            //     * s
  ITRVEC3( i ) flight.attiratePID[i].intlimit = PI / 3;            // /s  * s
  ITRVEC3( i ) pidReset( &( flight.positionPID[i] ) );
  ITRVEC3( i ) pidReset( &( flight.velocityPID[i] ) );
  ITRVEC3( i ) pidReset( &( flight.attitudePID[i] ) );
  ITRVEC3( i ) pidReset( &( flight.attiratePID[i] ) );
  ITRVEC3( i ) lowpassReset( &( flight.gyroLowpass[i] ) );
  ITRVEC3( i ) filterReset( &( flight.accelCalib[i] ) );
  ITRVEC3( i ) filterReset( &( flight.gyroCalib[i] ) );
  for( unsigned char i = 0; i < FPARLEN( kafenv.cal.sensefilt ); i++ ) {
    filterReset( &( flight.sensorCalib[i] ) );
  }
  flight.estimator = []( coordinate* estimator ) { return false; };
  flight.flightInvoker = NULLPTR;
  memory mempage = { "flight", sizeof( flight ), &flight };
  return mempage;
}

void flight_step( const imu* sensor ) {
  void ( *flightFunction )() = flight.flightInvoker;
  if( flightFunction != NULLPTR ) {
    DPRINTF( "[F] Flight Function: Pointer=%lu\n", flightFunction );
    flightFunction();
    flight.flightInvoker = NULLPTR;
  }
  switch( kafenv.info.flightMode & DEFAULT_MODES_MASK ) {
    case CALIBRATION_MODE : {
      DPRINTF( "[F] Running Calibration Mode Step\n" );
      if( sensor->accelUpdate ) {
        ITRVEC3( i ) filterCalibrate( &kafenv.cal.accelfilt[i], &flight.accelCalib[i], sensor->accelInput.f[i] );
        DPRINTF( "[F] Accelerometer Trim: Offset=[ %f, %f, %f ]\n", 
          kafenv.cal.accelfilt[0].ofst, kafenv.cal.accelfilt[1].ofst, kafenv.cal.accelfilt[2].ofst );
        DPRINTF( "[F] Accelerometer Variance: Stdev=[ %f, %f, %f ]\n", 
          kafenv.cal.accelfilt[0].stdv, kafenv.cal.accelfilt[1].stdv, kafenv.cal.accelfilt[2].stdv );
      }
      if( sensor->gyroUpdate ) {
        ITRVEC3( i ) filterCalibrate( &kafenv.cal.gyrofilt[i], &flight.gyroCalib[i], sensor->gyroInput.f[i] );
        DPRINTF( "[F] Gyroscope Trim: Offset=[ %f, %f, %f ]\n", 
          kafenv.cal.gyrofilt[0].ofst, kafenv.cal.gyrofilt[1].ofst, kafenv.cal.gyrofilt[2].ofst );
        DPRINTF( "[F] Gyroscope Variance: Stdev=[ %f, %f, %f ]\n", 
          kafenv.cal.gyrofilt[0].stdv, kafenv.cal.gyrofilt[1].stdv, kafenv.cal.gyrofilt[2].stdv );
      }
      if( sensor->magUpdate ) {
        ITRVEC3( i ) filterCalibrate( &kafenv.cal.magfilt[i], &flight.magCalib[i], sensor->magInput.f[i] );
        flight.yawOffset = atan2f( kafenv.cal.magfilt[1].ofst, kafenv.cal.magfilt[0].ofst );
        ITRVEC3( i )  kafenv.cal.magfilt[i].ofst = 0;
        DPRINTF( "[F] Magnetometer Trim: Yaw=%f\n", flight.yawOffset );
        DPRINTF( "[F] Magnetometer Variance: Stdev=[ %f, %f, %f ]\n", 
          kafenv.cal.magfilt[0].stdv, kafenv.cal.magfilt[1].stdv, kafenv.cal.magfilt[2].stdv );
      }
      kafenv.cal.positionalpha = 1 / ( AF * ( ( flight.calibTimeTotal += sensor->timeStep ) / ++flight.calibTimeCount ) );
      flight.estimator( NULLPTR );
      kafenv.info.actuation = false;
      break;
    }
    case INACTIVE_MODE : {
      DPRINTF( "[F] Running Inactive Mode Step\n" );
      flight_attitudeEstimate( sensor );
      positionEstimate( true, sensor->timeStep );
    }
    case NULL_MODE : {
      DPRINTF( "[F] Running Null Mode Step\n" );
      kafenv.info.actuation = false;
      FPFILL0( i, kafenv.cmd.motors );
      break;
    }
    case TRAJECTORY_MODE : {
      DPRINTF( "[F] Running Trajectory Mode Step\n" );
      const float t = kafenv.cmd.setpoints[0] > kafenv.cmd.setpoints[1] ? kafenv.cmd.setpoints[1] : kafenv.cmd.setpoints[0];
      const float t2 = t * t;
      const float t3 = t2 * t;
      const float t4 = t2 * t2;
      kafenv.cmd.setpoints[0] += sensor->timeStep;
      const float pos[4] = { kafenv.cmd.setpoints[ 2] * t4 + kafenv.cmd.setpoints[ 3] * t3 + kafenv.cmd.setpoints[ 4] * t2 + 
          kafenv.cmd.setpoints[ 5] * t + kafenv.cmd.setpoints[ 6], kafenv.cmd.setpoints[ 7] * t4 + kafenv.cmd.setpoints[ 8] * t3 + 
          kafenv.cmd.setpoints[ 9] * t2 + kafenv.cmd.setpoints[10] * t + kafenv.cmd.setpoints[11], kafenv.cmd.setpoints[12] * t4 + 
          kafenv.cmd.setpoints[13] * t3 + kafenv.cmd.setpoints[14] * t2 + kafenv.cmd.setpoints[15] * t + kafenv.cmd.setpoints[16],
          kafenv.cmd.setpoints[17] * t4 + kafenv.cmd.setpoints[18] * t3 + kafenv.cmd.setpoints[19] * t2 + kafenv.cmd.setpoints[20] * t + 
          kafenv.cmd.setpoints[21] };
      flight_positionControl( sensor, pos );
      break;
    }
    case POS_SETPOINT_MODE : {
      DPRINTF( "[F] Running Position Setpoint Step\n" );
      flight_positionControl( sensor, kafenv.cmd.setpoints );
      break;
    }
    case ACCEL_SETPOINT_MODE : {
      const float dt = sensor->timeStep;
      DPRINTF( "[F] Running Acceleration Setpoint Step: Step=%.3f\n", dt );
      flight_attitudeEstimate( sensor );
      flight_attitudeControl( { kafenv.cmd.setpoints[0], kafenv.cmd.setpoints[1], kafenv.cmd.setpoints[2] }, kafenv.cmd.setpoints[3], dt );
      break;
    }
    case MOTOR_SETPOINT_MODE : {
      DPRINTF( "[F] Running Motor Setpoint Step\n" );
      for( unsigned char i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
        kafenv.cmd.motors[i] = kafenv.cmd.setpoints[i];
      }
    }
    case ACTUATION_MODE : {
      DPRINTF( "[F] Running Actuation Step: Motors=[ %.3f, %.3f, %.3f, %.3f ]\n", 
          kafenv.cmd.motors[0], kafenv.cmd.motors[1], kafenv.cmd.motors[2], kafenv.cmd.motors[3] );
      kafenv.info.actuation = true;
      break;
    }
    default : {}
  }
}