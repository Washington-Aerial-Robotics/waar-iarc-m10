#include "kaf_quadcopter_code.h"

extern void peripheral_commonInit();
extern void peripheral_mpu9250Init();
extern void peripheral_mpu9250Loop();
extern void peripheral_escsInit();
extern void peripheral_escsLoop();
extern void estimation_reset();
extern bool estimation_step( coordinate* write );
extern imu common_imu;

void setup() {
  //init
  firmware_reset();
  peripheral_commonInit();
  peripheral_mpu9250Init();
  peripheral_escsInit();
  estimation_reset();
  flight_setPositionEstimator( &estimation_step );
}

void loop() {
  static unsigned long prevTime;
  unsigned long currentTime = millis();
  common_imu.timeStep = ( currentTime - prevTime ) * 1e-3F;
  prevTime = currentTime;
  peripheral_mpu9250Loop();
  flight_step( &common_imu );
  peripheral_escsLoop();
}