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

//Metres per degree of latitude (WGS84 mean, matches the longitude scaling's cos(latitude) approximation
//well enough at competition-arena scale - a few hundred metres). Previously this held the km/degree value
//(111.111) and was applied directly to position.x/y, which the rest of the firmware treats as metres -
//a 1000x scale error that (combined with the bugs below) meant GPS never produced a trustworthy position.
#define METERS_PER_DEGREE 111111.0
#define DEG2RAD 0.017453292519943295

extern SENSOR_BUFFERTYPE;

static struct {
  unsigned char dataLen;
  unsigned char readIndex;
  char readData[254];
  double originLat;
  double originLng;
  double originAlt;
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

//True when the most recently-parsed fix meets the minimum quality bar to be trusted at all (used both to
//gate ordinary position updates and to gate gps_setOrigin()). Does NOT check freshness - callers that care
//about staleness (estimation_positionValid()) additionally check lastFixMillis.
static bool isFixGoodQuality() {
  return SENSOR_BUFFER.gps.fixQuality != 0
      && SENSOR_BUFFER.gps.satellites >= GPS_MIN_SATELLITES
      && SENSOR_BUFFER.gps.hdop > 0.0F
      && SENSOR_BUFFER.gps.hdop <= GPS_MAX_HDOP;
}

//Pre-flight sensor status accessor (commander.cpp's sensorStatus telemetry bitfield): a quality-gated,
//fresh fix, WITHOUT requiring gps_setOrigin() to have latched yet - unlike estimation_positionValid(),
//which does require that, since this needs to report GPS health before a launch/start attempt has ever
//had the chance to set an origin.
bool gps_isFixGood() {
  return isFixGoodQuality() && SENSOR_BUFFER.gps.lastFixMillis != 0 &&
      ( millis() - SENSOR_BUFFER.gps.lastFixMillis ) <= GPS_STALE_MS;
}

bool gps_setOrigin() {
  if( !isFixGoodQuality() || millis() - SENSOR_BUFFER.gps.lastFixMillis > GPS_STALE_MS ) {
    DPRINTF( "[P] GPS Origin Latch Rejected: Quality=%u, Sats=%u, HDOP=%.2f\n",
        SENSOR_BUFFER.gps.fixQuality, SENSOR_BUFFER.gps.satellites, SENSOR_BUFFER.gps.hdop );
    return false;
  }
  sam.originLat = SENSOR_BUFFER.gps.latitude;
  sam.originLng = SENSOR_BUFFER.gps.longitude;
  sam.originAlt = SENSOR_BUFFER.gps.altitude;
  SENSOR_BUFFER.gps.originSet = true;
  SENSOR_BUFFER.gps.position = { 0, 0, 0 };
  DPRINTF( "[P] GPS Origin Latched: Lat=%.6f, Lng=%.6f, Alt=%.2f\n", sam.originLat, sam.originLng, sam.originAlt );
  return true;
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
    nextstring();                 //UTC time - unused
    char* lat = nextstring();
    char* latdir = nextstring();
    char* lng = nextstring();
    char* lngdir = nextstring();
    char* chk2 = nextstring();    //fix quality (0 = invalid)
    char* numsv = nextstring();   //satellite count
    char* hdop = nextstring();    //horizontal dilution of precision
    char* alt = nextstring();
    const char chk1cmp[] = "$GNGGA";
    if( memcmp( chk1, chk1cmp, sizeof( chk1cmp ) - 1 ) == 0 && strlen( lat ) > 0 ) {
      SENSOR_BUFFER.gps.fixQuality = ( unsigned char )strtol( chk2, NULLPTR, 10 );
      SENSOR_BUFFER.gps.satellites = ( unsigned char )strtol( numsv, NULLPTR, 10 );
      SENSOR_BUFFER.gps.hdop       = strtof( hdop, NULLPTR );
      char upperlat[] = { lat[0], lat[1], 0 };
      SENSOR_BUFFER.gps.latitude = ( strtod( upperlat, NULLPTR ) + strtod( &lat[2], NULLPTR ) / 60 ) * ( latdir[0] == 'S' ? -1 : 1 );
      char upperlng[] = { lng[0], lng[1], lng[2], 0 };
      SENSOR_BUFFER.gps.longitude = ( strtod( upperlng, NULLPTR ) + strtod( &lng[3], NULLPTR ) / 60 ) * ( lngdir[0] == 'W' ? -1 : 1 );
      SENSOR_BUFFER.gps.altitude = strtod( alt, NULLPTR );
      DPRINTF( "[P] GPS Position: Latitude=%.6f, Longitude=%.6f, Altitude=%.3f, Quality=%u, Sats=%u, HDOP=%.2f\n",
          SENSOR_BUFFER.gps.latitude, SENSOR_BUFFER.gps.longitude, SENSOR_BUFFER.gps.altitude,
          SENSOR_BUFFER.gps.fixQuality, SENSOR_BUFFER.gps.satellites, SENSOR_BUFFER.gps.hdop );
      if( isFixGoodQuality() ) {
        SENSOR_BUFFER.gps.lastFixMillis = millis();
        if( SENSOR_BUFFER.gps.originSet ) {
          //Local tangent-plane approximation about the latched origin - accurate to well under 1m of
          //error over the few-hundred-metre scale of this competition's arena; latitude must be in
          //radians for cos() (it is stored in degrees), which the previous implementation didn't convert.
          SENSOR_BUFFER.gps.position.x = ( float )( METERS_PER_DEGREE * ( SENSOR_BUFFER.gps.longitude - sam.originLng )
              * cos( SENSOR_BUFFER.gps.latitude * DEG2RAD ) );
          SENSOR_BUFFER.gps.position.y = ( float )( METERS_PER_DEGREE * ( SENSOR_BUFFER.gps.latitude - sam.originLat ) );
          SENSOR_BUFFER.gps.position.z = ( float )( SENSOR_BUFFER.gps.altitude - sam.originAlt );
          SENSOR_BUFFER.gps.update = true;
          DPRINTF( "[P] GPS Positioning Data: Value=[ %.3f, %.3f, %.3f ]\n",
              SENSOR_BUFFER.gps.position.x, SENSOR_BUFFER.gps.position.y, SENSOR_BUFFER.gps.position.z );
        }
      }
    }
    sam.dataLen = 0;
  }
}

void peripheral_samm10qInit() {
  firmware_registerPeripheral( { "samm10q", 0, sizeof( sam ), &sam, &peripheral_samm10qLoop, &peripheral_samm10qInit } );
  DPRINTF( "[P] Initializing SAMM10Q\n" );
  Serial1.begin( 9600, SERIAL_8N1, UART1RX, UART1TX );
  sam.dataLen = 0;
  sam.readIndex = 0;
  memset( sam.readData, 0, sizeof( sam.readData ) );
  sam.originLat = 0;
  sam.originLng = 0;
  sam.originAlt = 0;
  SENSOR_BUFFER.gps.originSet = false;
  SENSOR_BUFFER.gps.fixQuality = 0;
  SENSOR_BUFFER.gps.satellites = 0;
  SENSOR_BUFFER.gps.hdop = 0;
  SENSOR_BUFFER.gps.lastFixMillis = 0;
}
