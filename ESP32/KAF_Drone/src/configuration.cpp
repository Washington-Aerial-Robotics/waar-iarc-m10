#include "kaf_drone.h"
#include <string.h>

void setupDroneConfiguration() {
  kafenv.u.deviceID =                 'A';
  kafenv.s.rtos.comTaskPeriodOffset = 0;
  kafenv.u.stateEstimate.x.x        = 132.4;
  kafenv.u.stateEstimate.x.y        = 65.8;
  kafenv.u.stateEstimate.x.z        = -32;
  strcpy( kafenv.s.wifi.wifiName,     "wifi" );
  strcpy( kafenv.s.wifi.wifiPassword, "password" );
}