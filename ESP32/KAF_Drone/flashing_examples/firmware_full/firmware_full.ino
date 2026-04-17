#include "kaf_quadcopter_code.h"

extern memory estimation_reset();
extern bool estimation_step( coordinate* estimate );
extern memory commander_reset();
extern void commander_step( const unsigned long currentTime );
extern void peripheral_commonInit();
extern void peripheral_escsInit();
extern void peripheral_escsLoop();
extern void peripheral_mpu9250Init();
extern void peripheral_mpu9250Loop();
extern void peripheral_dw3000Init();
extern void peripheral_dw3000Loop();
extern void peripheral_wifiInit();
extern void peripheral_wifiLoop();
extern void peripheral_webserverInit();
extern void peripheral_webserverLoop();
extern void peripheral_esp32Init();
extern void peripheral_serialInit();
extern void peripheral_serialLoop();
extern void peripheral_serialStart( unsigned int baudrate );
extern void peripheral_freertosLoopComs();
extern void peripheral_freertosLoopFlight();
extern void peripheral_freertosSetup( void ( *flightTask )( void* ), void ( *comTask )( void* ) );
extern imu common_imu;

void setup() {
  peripheral_serialStart( 115200 );
  firmware_reset();
  peripheral_commonInit();
  peripheral_escsInit();
  peripheral_mpu9250Init();
  peripheral_dw3000Init();
  peripheral_wifiInit();
  peripheral_webserverInit();
  peripheral_esp32Init();
  peripheral_serialInit();
  firmware_registerMemoryPage( estimation_reset() );
  firmware_registerMemoryPage( commander_reset() );
  flight_setPositionEstimator( &estimation_step );
  peripheral_freertosSetup( []( void* pvParameters ) {
    for(;;) {
      peripheral_freertosLoopFlight();
      peripheral_mpu9250Loop();
      flight_step( &common_imu );
      peripheral_escsLoop();
    }
  }, []( void* pvParameters ) {
    for(;;) {
      peripheral_freertosLoopComs();
      peripheral_dw3000Loop();
      peripheral_serialLoop();
      peripheral_wifiLoop();
      peripheral_webserverLoop();
      commander_step( millis() );
    }
  } );
}

void loop() { }