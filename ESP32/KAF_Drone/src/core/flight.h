#include "kaf_drone.h"

//ID numbers for default flight modes. First three bit of the flight mode are defined
//by the core flight software code, and signify
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
  float timeStep = 0.01F;
  coordinate accelInput = { 0, 0, 0 };
  coordinate gyroInput = { 0, 0, 0 };
  coordinate magInput = { 0, 0, 0 };
  bool accelUpdate = false;
  bool gyroUpdate = false;
  bool magUpdate = false;
};

void flight_rotationMatrix( float matrix[9] );
//Returns a normalized ROS-order quaternion [x,y,z,w] derived from the current rotation matrix. Returns
//false and writes identity when the matrix is not finite/normalizable (for example before attitude init).
bool flight_rotationQuaternion( float quaternion[4] );
void* flight_positionEstimator( bool( *estimator )( coordinate* ) );
float flight_calibrateSensor( const STDBYTE id, const float value );
float flight_filterSensor( const STDBYTE id, const float value );

void flight_attitudeEstimate( const imu* sensor );
void flight_attitudeControl( const coordinate a, const float tk, const float dt );
void flight_positionControl( const imu* sensor, const float position[4] );

peripheral flight_reset();
void flight_step( const imu* sensor );
