#include "../core/firmware.h"
#include "../core/flight.h"
#include "../auxilary/common_data.h"
#include "../auxilary/estimation.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include <string.h>
#endif

#define RADIUS 111.11111111111111111111111111111111111111111111111111

extern SENSOR_BUFFERTYPE;

static struct {
  unsigned char dataLen;
  unsigned char readIndex;
  char readData[254];
  double coords[3];
  bool validoffset;
} sam;

static char* nextstring() {
  for( unsigned char i = sam.readIndex; i < sam.dataLen; i++ ) {
    if( sam.readData[i] == ',' || sam.readData[i] == '*' || sam.readData[i] == 0 ) {
      sam.readData[i] = 0;
      char* ref = &sam.readData[ sam.readIndex ];
      sam.readIndex = i + 1;
      return ref;
    }
  }
  sam.readIndex = sam.dataLen;
  const unsigned char fin = sam.readIndex;
  sam.readData[fin] = 0;
  return &sam.readData[fin];
}

void peripheral_samm10qLoop() {
  SENSOR_BUFFER.gps.update = false;
  bool isLineAvailable = false;
  for( ; Serial1.available() > 0; sam.dataLen++ ) {
    if( sam.dataLen >= sizeof( sam.readData ) ) {
      sam.readData[0] = Serial1.read();
      sam.dataLen = 0;
    } else if( ( sam.readData[sam.dataLen] = Serial1.read() ) == '\n' ) {
      sam.readData[sam.dataLen] = 0;
      isLineAvailable = true;
      break;
    }
    DPRINTF( "SREAD: i=%u, c=%c\n", sam.dataLen, sam.readData[sam.dataLen] );
  }
  DPRINTF( "LINE: %s, available=%u\n", sam.readData, isLineAvailable );
  if( isLineAvailable ) {
    sam.readIndex = 0;
    char* chk1 = nextstring();
    nextstring();
    char* lat = nextstring();
    char* latdir = nextstring();
    char* lng = nextstring();
    char* lngdir = nextstring();
    char* chk2 = nextstring();
    nextstring();
    nextstring();
    char* alt = nextstring();
    const char chk1cmp[] = "$GNGGA";
    if( memcmp( chk1, chk1cmp, sizeof( chk1cmp ) - 1 ) == 0 && strtol( chk2, NULLPTR, 10 ) != 0 ) {
      char upperlat[] = { lat[0], lat[1], 0 };
      SENSOR_BUFFER.gps.latitude = ( strtod( upperlat, NULLPTR ) + strtod( &lat[2], NULLPTR ) / 60 ) * ( latdir[0] == 'S' ? -1 : 1 );
      char upperlng[] = { lng[0], lng[1], lng[2], 0 };
      SENSOR_BUFFER.gps.longitude = ( strtod( upperlng, NULLPTR ) + strtod( &lng[3], NULLPTR ) / 60 ) * ( lngdir[0] == 'W' ? -1 : 1 );
      SENSOR_BUFFER.gps.altitude = strtod( alt, NULLPTR );
      DPRINTF( "[P] GPS Position: Latitude=%.3f, Longitude=%.3f, Altitude=%.3f", 
          SENSOR_BUFFER.gps.latitude, SENSOR_BUFFER.gps.longitude, SENSOR_BUFFER.gps.altitude );
      if( sam.validoffset ) {
        SENSOR_BUFFER.gps.position.x = ( float )( RADIUS * ( SENSOR_BUFFER.gps.latitude - sam.coords[0] ) );
        SENSOR_BUFFER.gps.position.y = ( float )( RADIUS * ( SENSOR_BUFFER.gps.longitude - sam.coords[1] ) * cos( SENSOR_BUFFER.gps.latitude ) );
        SENSOR_BUFFER.gps.position.z = ( float )( SENSOR_BUFFER.gps.altitude - sam.coords[2] );
        SENSOR_BUFFER.gps.update = true;
        DPRINTF( "[P] GPS Positioning Data: Value=[ %.3f, %.3f, %.3f ]", 
            SENSOR_BUFFER.gps.position.x, SENSOR_BUFFER.gps.position.y, SENSOR_BUFFER.gps.position.z );
      } else {
        sam.coords[0] = SENSOR_BUFFER.gps.latitude;
        sam.coords[1] = SENSOR_BUFFER.gps.longitude;
        sam.coords[2] = SENSOR_BUFFER.gps.altitude;
      }
    }
    sam.dataLen = 0;
  }
}

void peripheral_samm10qInit() {
  firmware_registerPeripheral( { "samm10q", 0, sizeof( sam ), &sam, &peripheral_samm10qInit, &peripheral_samm10qLoop } );
  DPRINTF( "[P] Initializing SAMM10Q\n" );
  Serial1.begin( 9600, SERIAL_8N1, UART1RX, UART1TX );
  sam.dataLen = 0;
  sam.readIndex = 0;
  memset( sam.readData, 0, sizeof( sam.readData ) );
  ITRVEC3( i ) sam.coords[i] = 0;
  sam.validoffset = false;
}