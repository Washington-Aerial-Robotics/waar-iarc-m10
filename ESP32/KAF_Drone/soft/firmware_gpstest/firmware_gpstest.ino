/*
  GPS (SAM-M10Q) Bench Test
  --------------------------
  Exercises the real peripheral_samm10qInit()/peripheral_samm10qLoop() code
  path directly against live GPS hardware, without the rest of the flight
  stack. Reads raw NMEA over UART1 (GPIO 16 = RX from GPS TX, GPIO 17 = TX
  to GPS RX, 9600 baud) and prints parsed position once a fix is available.

  The existing peripheral_samm10qLoop() already DPRINTFs every byte it
  reads (very verbose - useful to confirm bytes are arriving at all) and
  each full NMEA line, plus a "[P] GPS Position" line once it successfully
  parses a $GNGGA sentence with a valid fix. This sketch adds one more
  summary line every second so you don't have to parse the raw spam by eye.

  SAFETY: no motors/ESCs touched by this sketch at all.
*/

#include "kaf_quadcopter_code.h"
#include "auxilary/estimation.h"

extern void peripheral_samm10qInit();
extern void peripheral_samm10qLoop();
extern void peripheral_serialStart( unsigned int baudrate );
extern sensors common_sensor;

void setup() {
  peripheral_serialStart( 115200 );
  firmware_reset();
  peripheral_samm10qInit();

  Serial.println( "GPS bench test starting." );
  Serial.println( "Watching for $GNGGA sentences on UART1 (GPIO16=RX, GPIO17=TX, 9600 baud)." );
  Serial.println( "No fix yet? Get the antenna near a window or outdoors - a cold GPS fix can take a minute or more." );
}

void loop() {
  static unsigned long lastSummary = 0;
  peripheral_samm10qLoop();

  unsigned long now = millis();
  if( now - lastSummary >= 1000 ) {
    lastSummary = now;
    Serial.printf( "[SUMMARY] Fix Update=%s  Lat=%.6f  Lon=%.6f  Alt=%.2f  Pos=[ %.3f, %.3f, %.3f ]\n",
        common_sensor.gps.update ? "YES" : "no",
        common_sensor.gps.latitude, common_sensor.gps.longitude, common_sensor.gps.altitude,
        common_sensor.gps.position.x, common_sensor.gps.position.y, common_sensor.gps.position.z );
  }
}
