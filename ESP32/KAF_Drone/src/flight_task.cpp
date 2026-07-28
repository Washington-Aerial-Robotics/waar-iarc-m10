#include "kaf_drone.h"
#include "communication.h"
#include "inter_drone_avoidance.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <math.h>
#include <cstring>
#endif

static void updateImuAttitudeEstimate( float dt ) {
  if( dt <= 0.0F ) {
    return;
  }
  float ax = kafenv.f.accelInput.x;
  float ay = kafenv.f.accelInput.y;
  float az = kafenv.f.accelInput.z;
  float gx = kafenv.f.gyroInput.x;
  float gy = kafenv.f.gyroInput.y;
  float gz = kafenv.f.gyroInput.z;

  if( kafenv.f.accelInput.stdev < 1e8F ) {
    float rollMeas = atan2f( ay, az );
    float pitchMeas = atan2f( -ax, sqrtf( ay * ay + az * az ) );
    const float alpha = 0.98F;
    kafenv.u.stateEstimate.t.z = alpha * ( kafenv.u.stateEstimate.t.z + gx * dt ) + ( 1.0F - alpha ) * rollMeas;
    kafenv.u.stateEstimate.t.y = alpha * ( kafenv.u.stateEstimate.t.y + gy * dt ) + ( 1.0F - alpha ) * pitchMeas;
    kafenv.u.stateEstimate.t.x += gz * dt;
    kafenv.u.stateEstimate.w.x = gx;
    kafenv.u.stateEstimate.w.y = gy;
    kafenv.u.stateEstimate.w.z = gz;
  }
}

void softwareInit() {
  kafenv.u.flightMode = NULL_MODE;
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.positionPID[i] ) ) );
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.velocityPID[i] ) ) );
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.attitudePID[i] ) ) );
  KF_VEC3( i, kaf_pidreset( &( kafenv.s.attiratePID[i] ) ) );
  KF_VEC3( i, kaf_filterreset( &( kafenv.f.accelCalib[i] ) ) );
  KF_VEC3( i, kaf_filterreset( &( kafenv.f.gyroCalib[i] ) ) );
  KF_VEC3( i, kaf_filterreset( &( kafenv.f.magCalib[i] ) ) );
  KF_VEC3( i, kaf_filterreset( &( kafenv.f.gpsCalib[i] ) ) );
  KF_VEC3( i, kaf_filterreset( &( kafenv.f.baroCalib[i] ) ) );
  memset( &kafenv.u.stateEstimate, 0, sizeof( kafenv.u.stateEstimate ) );
  kafenv.n.nextMessageID = (unsigned int)( rand() % 0x100 );
  memset( kafenv.n.deviceLookup, INVALID_DEVICE_IDX, sizeof( kafenv.n.deviceLookup ) );
  for( unsigned char i = 0; i < DEVICE_COUNT; i++ ) {
    kafenv.n.deviceIndices[i] = i;
  }
}

void controlsStep() {
  float timeStep = kafenv.f.timeStep * 1e-6F;
  updateImuAttitudeEstimate( timeStep );

  switch( kafenv.u.flightMode ) {
    case CALIBRATION_MODE : {
      memset( &kafenv.u.stateEstimate, 0, sizeof( kafenv.u.stateEstimate ) );
      if( kafenv.f.accelInput.stdev < 2 ) {
        KF_VEC3( i, kaf_filtercalib( &( kafenv.f.accelCalib[i] ), kafenv.f.accelInput.f[i] ) );
      }
      if( kafenv.f.gyroInput.stdev < 2 ) {
        KF_VEC3( i, kaf_filtercalib( &( kafenv.f.gyroCalib[i] ), kafenv.f.gyroInput.f[i] ) );
      }
    }
    case NULL_MODE : {
      kafenv.f.motorsEnabled = false;
      KF_VECN( i, MOTOR_COUNT, kafenv.f.motorOutput[i] = 0 );
      break;
    }
    case TRAJECTORY_MODE : {
      //wip
    }
    case POS_SETPOINT_MODE : {
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.positionPID[i] ), kafenv.s.posSetpoint[i],     kafenv.u.stateEstimate.x.f[i], timeStep ) );
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.velocityPID[i] ), kafenv.s.cascadeSetpoint[i], kafenv.u.stateEstimate.v.f[i], timeStep ) );
      //wip convert to attitude
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.attitudePID[i] ), kafenv.s.cascadeSetpoint[i], kafenv.u.stateEstimate.t.f[i], timeStep ) );
      KF_VEC3( i, kafenv.s.cascadeSetpoint[i] = kaf_pidstep( &( kafenv.s.attiratePID[i] ), kafenv.s.cascadeSetpoint[i], kafenv.u.stateEstimate.w.f[i], timeStep ) );
      //wip convert to motor cmd
    }
    case MOTOR_SETPOINT_MODE : {
      applyInterDroneHardSeparation();
      KF_VECN( i, MOTOR_COUNT, kafenv.f.motorOutput[i] = kafenv.s.motorSetpoint[i] );
      break;
    }
    default : {}
  }
}