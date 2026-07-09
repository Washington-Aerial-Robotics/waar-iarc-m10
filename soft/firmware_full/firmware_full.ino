#include "kaf_quadcopter_code.h"

extern peripheral estimation_reset();
extern bool estimation_step( coordinate* estimate );
extern peripheral commander_reset();
extern void commander_step( const unsigned long currentTime );
extern peripheral pidtuner_reset();
extern void pidtuner_step( void* imuData );
extern void peripheral_commonInit();
extern void peripheral_escsInit();
extern void peripheral_escsLoop();
extern void peripheral_mpu9250Init();
extern void peripheral_mpu9250Loop();
extern void peripheral_dw3000Init();
extern void peripheral_dw3000Loop();
extern void peripheral_esp32Init();
extern void peripheral_wifiInit();
extern void peripheral_wifiLoop();
extern void peripheral_webserverInit();
extern void peripheral_webserverLoop();
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
  firmware_registerPeripheral( estimation_reset() );
  firmware_registerPeripheral( commander_reset() );
  firmware_registerPeripheral( pidtuner_reset() );
  flight_positionEstimator( &estimation_step );
  peripheral_freertosSetup( []( void* pvParameters ) {
    for(;;) {
      peripheral_freertosLoopFlight();
      peripheral_mpu9250Loop();
      pidtuner_step( &common_imu );
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