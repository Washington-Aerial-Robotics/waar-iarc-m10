#include "kaf_quadcopter_code.h"

extern void peripheral_commonInit();
extern void peripheral_serialInit();
extern void peripheral_serialLoop();
extern void peripheral_wifiInit();
extern void peripheral_wifiLoop();
extern void peripheral_webserverInit();
extern void peripheral_webserverLoop();
extern void peripheral_dw3000Init();
extern void peripheral_dw3000Loop();
extern void peripheral_serialStart( unsigned int baudrate );
extern void peripheral_wifiSetNetwork( char* networkName, char* networkPassword );

void setup() {
  peripheral_serialStart( 115200 );
  firmware_reset();
  peripheral_commonInit();
  peripheral_serialInit();
  peripheral_wifiInit();
  peripheral_webserverInit();
  peripheral_dw3000Init();
  kafenv.info.deviceID = 'A';
  peripheral_wifiSetNetwork( "iPhone", "password1234" );
}

void loop() {
  peripheral_serialLoop();
  peripheral_wifiLoop();
  peripheral_webserverLoop();
  peripheral_dw3000Loop();
  delay( 1000 );
}