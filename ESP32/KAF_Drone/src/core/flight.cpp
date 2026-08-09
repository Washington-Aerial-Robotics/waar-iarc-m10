#include "flight.h"

#if ALT_DEFINE
#define NAN 0.0F
#define isfinite( arg ) true
static float sqrtf( float arg );
static float fmodf( float arg0, float arg1 );
static float atan2f( float y, float x );
static float cosf( float arg );
static float sinf( float arg );
static float expf( float arg );
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

#define BOUND( V, LB, UB ) V < UB ? ( V > LB ? V : LB ) : UB
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
  float accelZ;
  unsigned int calibTimeCount;
  unsigned int posTimeCount;
  float calibTimeTotal;
  float yawOffset;
  coordinate estimate;
  coordinate acceleration;
  pid positionPID[3];
  pid velocityPID[3];
  pid attitudePID[3];
  pid attiratePID[3];
  pid acclratePID;
  lowpass gyroLowpass[3];
  measfilter accelCalib[3];
  measfilter gyroCalib[3];
  measfilter magCalib[3];
  measfilter sensorCalib[ FPARLEN( kafenv.cal.sensefilt ) ];
  bool( *estimator )( coordinate* );
} flight;

static void lowpassReset( lowpass* lowpass ) {
  lowpass->internal = NAN;
  lowpass->lowpass = NAN;
}

static float lowpassStep( lowpass* lowpass, const float measured ) {
  lowpass->internal -= S2 * ( lowpass->internal + S8 * ( lowpass->lowpass - measured ) );
  FPGUARD( lowpass->internal, 0 );
  lowpass->lowpass += lowpass->internal;
  FPGUARD( lowpass->lowpass, 0 );
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
  FPGUARD( filter->previous, corrected );
  const float prevMeas = filter->previous;
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
  FPGUARD( filter->total, 0 );
  filterval->ofst = filter->total / filter->count;
  const float measDelta = measured - filterval->ofst;
  filter->deviation += measDelta * measDelta;
  FPGUARD( filter->deviation, 0 );
  const float sig2 = filter->deviation / filter->count;
  filterval->stdv = P2 * sig2 * sig2 * sig2;
  filter->previous = measured;
  return sig2;
}

static void positionEstimate( const float dt ) {
  ITRVEC3( i ) kafenv.state.v.f[i] += dt * flight.acceleration.f[i];
  coordinate pos;
  if( flight.estimator( &pos ) ) {
    DPRINTF( "[F] Position Received: X=[ %.3f, %.3f, %.3f ]\n", pos.x, pos.y, pos.z );
    ITRVEC3( i ) flight.estimate.f[i] = kafenv.cal.positionalpha * ( ( pos.f[i] - kafenv.state.x.f[i] ) / dt - kafenv.state.v.f[i] );
  } else {
    const float dtalpha = dt * kafenv.cal.positionalpha;
    ITRVEC3( i ) flight.estimate.f[i] -= dtalpha * flight.estimate.f[i];
  }
  DPRINTF( "[F] Updated Gain Deltas: D=[ %.3f, %.3f, %.3f ]\n", flight.estimate.x, flight.estimate.y, flight.estimate.z );
  ITRVEC3( i ) {
    kafenv.state.v.f[i] += dt * flight.estimate.f[i];
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

void* flight_positionEstimator( bool( *estimator )( coordinate* ) ) {
  bool( *previous )( coordinate* ) = flight.estimator;
  if( estimator != NULLPTR ) {
    flight.estimator = estimator;
  }
  return ( void* )previous;
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
    const float g2 = kafenv.cal.gravitation * kafenv.cal.gravitation;
    const float ax2ay2 = a.x * a.x + a.y * a.y;
    const float cos2 = ( a.z + kafenv.cal.gravitation < 0 ? -1 : 1 ) * 
        sqrtf( g2 < ax2ay2 ? 0 : g2 - ax2ay2 ) / ( kafenv.cal.gravitation + EF ) + 1;
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
  flight.accelZ = a.z;
  DPRINTF( "[F] Updated Rotation Matrix: R=[ %.3f, %.3f, %.3f; %.3f, %.3f, %.3f; %.3f, %.3f, %.3f ]\n", flight.rotMat[0], flight.rotMat[1], 
      flight.rotMat[2], flight.rotMat[3], flight.rotMat[4], flight.rotMat[5], flight.rotMat[6], flight.rotMat[7], flight.rotMat[8] );
  //update acceleration
  ITRVEC3( i ) flight.acceleration.f[i] = flight.rotMat[ i ] * a.x + flight.rotMat[ i + 1 ] * a.y + flight.rotMat[ i + 2 ] * a.z;
  DPRINTF( "[F] Updated Acceleration: A=[ %.3f, %.3f, %.3f ]\n", flight.acceleration.x, flight.acceleration.y, flight.acceleration.z );
}

void flight_attitudeControl( const coordinate a, const float tk, const float dt ) {
  // calculate acceleration vector magnitude
  DPRINTF( "[F] Acceleration Setpoint: A=[ %.3f, %.3f, %.3f ], Theta=%.3f\n", a.x, a.y, a.z, tk );
  const float az = a.z + kafenv.cal.gravitation;
  const float am = 1 / ( sqrtf( a.x * a.x + a.y * a.y + az * az ) + EF );
  // convert from world frame to body frame acceleration setpoint
  const float ak = flight.rotMat[2] * a.x + flight.rotMat[5] * a.y;
  const float b0 = am / ( sqrtf( ( ak + flight.rotMat[8] * az ) * am * 0.5F + 0.5F ) + EF );
  float sp[] = { -b0 * ( flight.rotMat[1] * a.x + flight.rotMat[4] * a.y + flight.rotMat[7] * az ), 
                  b0 * ( flight.rotMat[0] * a.x + flight.rotMat[3] * a.y + flight.rotMat[6] * az ), 
                  tk - kafenv.state.q.z };
  // pid control step
  ITRVEC3( i ) sp[i] = pidStep( &kafenv.cal.qpid, &flight.attitudePID[i], 0, -sp[i], dt );
  ITRVEC3( i ) sp[i] = pidStep( &kafenv.cal.wpid[i], &flight.attiratePID[i], sp[i], kafenv.state.w.f[i], dt );
  // set motor inputs
  float st = pidStep( &kafenv.cal.apid, &flight.acclratePID, ak + flight.rotMat[8] * a.z, flight.accelZ, dt ) 
      + kafenv.cal.apid.gain * kafenv.cal.gravitation;
  st = st < 0 ? ( ( sp[0] < 0 ? -sp[0] : sp[0] ) + ( sp[1] < 0 ? 
      -sp[1] : sp[1] ) + ( sp[2] < 0 ? -sp[2] : sp[2] ) ) : ( st < 1 ? st : 1 );
  DPRINTF( "[F] Attitude Control Outputs: Thrust=%.3f, Pitch=%.3f, Roll=%.3f, Yaw=%.3f\n", st, sp[0], sp[1], sp[2] );
  kafenv.cmd.motors[0] = st + sp[0] - sp[1] - sp[2];
  kafenv.cmd.motors[1] = st + sp[0] + sp[1] + sp[2];
  kafenv.cmd.motors[2] = st - sp[0] - sp[1] + sp[2];
  kafenv.cmd.motors[3] = st - sp[0] + sp[1] - sp[2];
  for( unsigned char i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
    kafenv.cmd.motors[i] = sqrtf( BOUND( kafenv.cmd.motors[i], 0, 1 ) );
  }
  DPRINTF( "[F] Attitude Control Motor Actuation: Motors=[ %.3f, %.3f, %.3f, %.3f ]\n",
      kafenv.cmd.motors[0], kafenv.cmd.motors[1], kafenv.cmd.motors[2], kafenv.cmd.motors[3] );
}

void flight_positionControl( const imu* sensor, const float position[4] ) {
  const float dt = sensor->timeStep;
  flight_attitudeEstimate( sensor );
  positionEstimate( dt );
  float setpoint[3] = { 0, 0, 0 };
  DPRINTF( "[F] Position Setpoint: X=[ %.3f, %.3f, %.3f ]\n", position[0], position[1], position[2] );
  ITRVEC3( i ) setpoint[i] = pidStep( &kafenv.cal.xpid, &flight.positionPID[i], position[i], kafenv.state.x.f[i], dt );
  ITRVEC3( i ) setpoint[i] = pidStep( &kafenv.cal.vpid, &flight.velocityPID[i], setpoint[i], kafenv.state.v.f[i], dt );
  flight_attitudeControl( { setpoint[0], setpoint[1], setpoint[2] }, position[3], dt );
}

peripheral flight_reset() {
  DPRINTF( "[F] Resetting Flight Software\n" );
  FPFILL0( i, flight.rotMat );
  flight.accelZ = 0;
  flight.calibTimeCount = 0;
  flight.posTimeCount = 0;
  flight.calibTimeTotal = 0;
  flight.yawOffset = 0;
  flight.estimate = { 0, 0, 0 };
  flight.acceleration = { 0, 0, 0 };
  ITRVEC3( i ) flight.positionPID[i].outlimit = 10;                // m/s  max speed
  ITRVEC3( i ) flight.velocityPID[i].outlimit = 1;                 // %V   max accel
  ITRVEC3( i ) flight.attitudePID[i].outlimit = PI * 5;            // /s   max rot speed
  ITRVEC3( i ) flight.attiratePID[i].outlimit = 0.5F;              // %V   max volt
               flight.acclratePID   .outlimit = 0.9F;              // %V   max volt
  ITRVEC3( i ) flight.positionPID[i].intlimit = 10;                // m    * s
  ITRVEC3( i ) flight.velocityPID[i].intlimit = 25;                // m/s  * s
  ITRVEC3( i ) flight.attitudePID[i].intlimit = PI / 3;            //      * s
  ITRVEC3( i ) flight.attiratePID[i].intlimit = PI / 3;            // /s   * s
               flight.acclratePID   .intlimit = 19.6F;             // m/s2 * s
  ITRVEC3( i ) pidReset( &flight.positionPID[i] );
  ITRVEC3( i ) pidReset( &flight.velocityPID[i] );
  ITRVEC3( i ) pidReset( &flight.attitudePID[i] );
  ITRVEC3( i ) pidReset( &flight.attiratePID[i] );
               pidReset( &flight.acclratePID );
  ITRVEC3( i ) lowpassReset( &flight.gyroLowpass[i] );
  ITRVEC3( i ) filterReset( &flight.accelCalib[i] );
  ITRVEC3( i ) filterReset( &flight.gyroCalib[i] );
  for( unsigned char i = 0; i < FPARLEN( kafenv.cal.sensefilt ); i++ ) {
    filterReset( &( flight.sensorCalib[i] ) );
  }
  flight.estimator = []( coordinate* estimator ) { return false; };
  return { "flight", 0, sizeof( flight ), &flight, [](){ flight_reset(); }, NULLPTR };
}

void flight_step( const imu* sensor ) {
  switch( kafenv.info.flightMode & DEFAULT_MODES_MASK ) {
    case NULL_MODE : {
      //DEBUG logging throttled to ~1/s - see periph_freertos.cpp for why
      static unsigned long lastNullModePrint = 0;
      if( millis() - lastNullModePrint > 1000 ) {
        lastNullModePrint = millis();
        DPRINTF( "[F] Running Null Mode Step\n" );
      }
      FPFILL0( i, kafenv.cmd.motors );
      kafenv.info.actuation = false;
      break;
    }
    case INACTIVE_MODE : {
      DPRINTF( "[F] Running Inactive Mode Step\n" );
      flight_attitudeEstimate( sensor );
      positionEstimate( sensor->timeStep );
      break;
    }
    case CALIBRATION_MODE : {
      DPRINTF( "[F] Running Calibration Mode Step\n" );
      float accelStdevTotal = 1;
      if( sensor->accelUpdate ) {
        ITRVEC3( i ) accelStdevTotal += filterCalibrate( &kafenv.cal.accelfilt[i], &flight.accelCalib[i], sensor->accelInput.f[i] );
        DPRINTF( "[F] Accelerometer Trim: Offset=[ %f, %f, %f ]\n", 
          kafenv.cal.accelfilt[0].ofst, kafenv.cal.accelfilt[1].ofst, kafenv.cal.accelfilt[2].ofst );
        DPRINTF( "[F] Accelerometer Variance: Stdev=[ %f, %f, %f ]\n", 
          kafenv.cal.accelfilt[0].stdv, kafenv.cal.accelfilt[1].stdv, kafenv.cal.accelfilt[2].stdv );
      }
      float gyroStdevTotal = 1;
      if( sensor->gyroUpdate ) {
        ITRVEC3( i ) gyroStdevTotal += filterCalibrate( &kafenv.cal.gyrofilt[i], &flight.gyroCalib[i], sensor->gyroInput.f[i] );
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
      const float avgDT = ( flight.calibTimeTotal += sensor->timeStep ) / ++flight.calibTimeCount;
      kafenv.cal.anglealpha = sqrtf( sqrtf( gyroStdevTotal ) * avgDT / ( sqrtf( accelStdevTotal ) + EF ) );
      kafenv.cal.anglealpha = kafenv.cal.anglealpha < 1 ? kafenv.cal.anglealpha : 1;
      kafenv.cal.positionalpha = 0.5F * ( flight.posTimeCount += flight.estimator( NULLPTR ) ? 1 : 0 ) / flight.calibTimeTotal;
      kafenv.cal.gravitation = kafenv.cal.accelfilt[2].ofst * kafenv.cal.accelfilt[2].gain;
      DPRINTF( "[F] Calibration Constants: Angle=%f, Position=%f, Gravitation=%f\n", 
          kafenv.cal.anglealpha, kafenv.cal.positionalpha, kafenv.cal.gravitation );
      break;
    }
    case ACTUATION_MODE : {
      DPRINTF( "[F] Running External Control Step\n" );
      break;
    }
    case MOTOR_SETPOINT_MODE : {
      DPRINTF( "[F] Running Motor Setpoint Step\n" );
      for( unsigned char i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
        kafenv.cmd.motors[i] = kafenv.cmd.setpoints[i];
      }
      DPRINTF( "[F] Actuation Values: Motors=[ %.3f, %.3f, %.3f, %.3f ]\n", 
          kafenv.cmd.motors[0], kafenv.cmd.motors[1], kafenv.cmd.motors[2], kafenv.cmd.motors[3] );
      break;
    }
    case ACCEL_SETPOINT_MODE : {
      const float dt = sensor->timeStep;
      DPRINTF( "[F] Running Acceleration Setpoint Step: Step=%.3f\n", dt );
      flight_attitudeEstimate( sensor );
      flight_attitudeControl( { kafenv.cmd.setpoints[0], kafenv.cmd.setpoints[1], kafenv.cmd.setpoints[2] }, kafenv.cmd.setpoints[3], dt );
      break;
    }
    case POS_SETPOINT_MODE : {
      DPRINTF( "[F] Running Position Setpoint Step\n" );
      flight_positionControl( sensor, kafenv.cmd.setpoints );
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
    default : {}
  }
  if( kafenv.info.triggerLock != 0 ) {
    kafenv.info.triggerLock = MAXBYTE;
    DPRINTF( "[F] Flight Trigger Lock Held\n" );
    while( kafenv.info.triggerLock != 0 ) { }
    DPRINTF( "[F] Flight Trigger Lock Released\n" );
  }
}