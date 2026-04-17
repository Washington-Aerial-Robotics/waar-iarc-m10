#include "kaf_drone.h"

#define DEFAULT_MODES_MASK   7
#define NULL_MODE            0
#define INACTIVE_MODE        1
#define CALIBRATION_MODE     2
#define ACTUATION_MODE       3
#define MOTOR_SETPOINT_MODE  4
#define ACCEL_SETPOINT_MODE  5
#define POS_SETPOINT_MODE    6
#define TRAJECTORY_MODE      7

struct imu {//FLIGHT DATA
  float timeStep = 0;
  coordinate accelInput = { 0, 0, 0 };
  coordinate gyroInput = { 0, 0, 0 };
  coordinate magInput = { 0, 0, 0 };
  bool accelUpdate = false;
  bool gyroUpdate = false;
  bool magUpdate = false;
};

void flight_rotationMatrix( float matrix[9] );
void flight_setPositionEstimator( bool( *estimator )( coordinate* ) );
void flight_runFunction( void( *function )() );
float flight_calibrateSensor( const STDBYTE id, const float value );
float flight_filterSensor( const STDBYTE id, const float value );

void flight_attitudeEstimate( const imu* sensor );
void flight_attitudeControl( const coordinate a, const float tk, const float dt );
void flight_positionControl( const imu* sensor, const float position[4] );

memory flight_reset();
void flight_step( const imu* sensor );