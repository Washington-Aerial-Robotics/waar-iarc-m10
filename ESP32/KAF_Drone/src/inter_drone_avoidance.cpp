#include "inter_drone_avoidance.h"
#include "kaf_drone.h"
#include "communication.h"

#if !ALT_DEFINE
#include <math.h>
#endif

static bool peerPositionFresh( unsigned char deviceIndex ) {
  if( deviceIndex == INVALID_DEVICE_IDX ) {
    return false;
  }
  unsigned long age = kafenv.n.currentTime - kafenv.n.devices[deviceIndex].lastSeen;
  return age <= NETWORK_DEVICE_TIMEOUT;
}

/*
 * If any teammate is within R_hard horizontally, nudge motor setpoints away from them.
 * Does not run when motors are disabled, when the GS safety pilot is active, or when no
 * peer position is available.
 *
 * TODO (if peers never appear): broadcast fused state on a timer via COM_SET_ST_EST
 * forwarded on the mesh, or complete UWB trilateration so devices[].position is valid.
 */
void applyInterDroneHardSeparation( void ) {
  if( !kafenv.f.motorsEnabled || kafenv.u.flightMode == NULL_MODE ) {
    return;
  }

  if( kafenv.n.currentTime - kafenv.s.lastPilotMotorCmdMs < PILOT_MOTOR_OVERRIDE_MS ) {
    return;
  }

  if( kafenv.u.flightMode != MOTOR_SETPOINT_MODE && kafenv.u.flightMode != POS_SETPOINT_MODE &&
      kafenv.u.flightMode != TRAJECTORY_MODE ) {
    return;
  }

  float selfX = kafenv.u.stateEstimate.x.x;
  float selfY = kafenv.u.stateEstimate.x.y;
  float nearestD = INTER_DRONE_HARD_SEP_M;
  float awayX = 0.0F;
  float awayY = 0.0F;
  bool threat = false;

  for( unsigned int i = 0; i < DEVICE_COUNT; i++ ) {
    unsigned char idx = kafenv.n.deviceIndices[i];
    if( !peerPositionFresh( idx ) ) {
      continue;
    }
    if( kafenv.n.devices[idx].deviceID == kafenv.u.deviceID ) {
      continue;
    }
    float dx = selfX - kafenv.n.devices[idx].position.x;
    float dy = selfY - kafenv.n.devices[idx].position.y;
    float d = sqrtf( dx * dx + dy * dy );
    if( d < 1e-3F ) {
      d = 1e-3F;
      dx = 1.0F;
      dy = 0.0F;
    }
    if( d < nearestD ) {
      nearestD = d;
      awayX = dx / d;
      awayY = dy / d;
      threat = true;
    }
  }

  if( !threat ) {
    return;
  }

  float strength = ( INTER_DRONE_HARD_SEP_M - nearestD ) / INTER_DRONE_HARD_SEP_M;
  if( strength < 0.0F ) {
    strength = 0.0F;
  }
  if( strength > 1.0F ) {
    strength = 1.0F;
  }

  float yaw = kafenv.u.stateEstimate.t.z;
  float c = cosf( yaw );
  float s = sinf( yaw );
  float bodyForward = c * awayX + s * awayY;
  float bodyRight = -s * awayX + c * awayY;
  float gain = 0.22F * strength;

  float m0 = kafenv.s.motorSetpoint[0] + gain * ( bodyForward + bodyRight );
  float m1 = kafenv.s.motorSetpoint[1] + gain * ( bodyForward - bodyRight );
  float m2 = kafenv.s.motorSetpoint[2] + gain * ( -bodyForward + bodyRight );
  float m3 = kafenv.s.motorSetpoint[3] + gain * ( -bodyForward - bodyRight );

  kafenv.s.motorSetpoint[0] = m0 > 1.0F ? 1.0F : ( m0 < 0.0F ? 0.0F : m0 );
  kafenv.s.motorSetpoint[1] = m1 > 1.0F ? 1.0F : ( m1 < 0.0F ? 0.0F : m1 );
  kafenv.s.motorSetpoint[2] = m2 > 1.0F ? 1.0F : ( m2 < 0.0F ? 0.0F : m2 );
  kafenv.s.motorSetpoint[3] = m3 > 1.0F ? 1.0F : ( m3 < 0.0F ? 0.0F : m3 );
}
