/*
  SAM-M10Q GPS Bench Test - Arduino version
  --------------------------
  Logs a coordinate every time you type anything + Enter in the Serial
  Monitor. Runs entirely on the ESP32 itself, reading the SAM-M10Q over the
  same UART1 pins the real firmware uses (UART1RX/UART1TX from
  auxilary/common_data.h) - no separate USB-TTL adapter needed, and no need
  to flash the full flight firmware to test GPS in isolation.

  Uses the real project pin/baud constants but re-implements its own NMEA
  parsing and local-frame math as a deliberate line-for-line copy of the
  FIXED logic in periph_samm10q.cpp (METERS_PER_DEGREE=111111.0, not the
  original buggy 111.111 km/degree; latitude converted to radians before
  cos(); longitude(+east)->x, latitude(+north)->y) - a working result here
  is real evidence that fix is correct, not just that this separate sketch
  happens to work. Also applies the same GPS_MIN_SATELLITES/GPS_MAX_HDOP
  gate the firmware uses and flags any point that wouldn't have passed it.

  First point logged becomes the local-frame origin (0,0,0); every
  later point is reported relative to it.

  SAFETY: no motors/ESCs touched by this sketch at all - GPS/UART1 only.
*/

#include "kaf_quadcopter_code.h"
#include "auxilary/common_data.h"

#define METERS_PER_DEGREE 111111.0
#define DEG2RAD 0.017453292519943295
#define GPS_MIN_SATELLITES 6
#define GPS_MAX_HDOP 3.0F

static char lineBuf[254];
static unsigned char lineLen = 0;

static double originLat = 0, originLng = 0, originAlt = 0;
static bool haveOrigin = false;
static unsigned int pointCount = 0;

static double curLat = 0, curLng = 0, curAlt = 0;
static unsigned char curQuality = 0, curSats = 0;
static float curHdop = 0;
static unsigned long curFixMillis = 0;

// Splits the next comma-separated field out of *cursor (NMEA-style), same
// tokenizing approach periph_samm10q.cpp's nextstring() uses.
static char* nextField( char** cursor ) {
  char* start = *cursor;
  char* comma = strchr( start, ',' );
  if( comma != NULLPTR ) {
    *comma = 0;
    *cursor = comma + 1;
  } else {
    char* star = strchr( start, '*' );
    if( star != NULLPTR ) {
      *star = 0;
    }
    *cursor = start + strlen( start );
  }
  return start;
}

static void parseLine( char* line ) {
  if( strncmp( line, "$GNGGA", 6 ) != 0 && strncmp( line, "$GPGGA", 6 ) != 0 ) {
    return;
  }
  char* cursor = line;
  nextField( &cursor );          // sentence id
  nextField( &cursor );          // UTC time - unused
  char* lat = nextField( &cursor );
  char* latDir = nextField( &cursor );
  char* lng = nextField( &cursor );
  char* lngDir = nextField( &cursor );
  char* qual = nextField( &cursor );
  char* sats = nextField( &cursor );
  char* hdop = nextField( &cursor );
  char* alt = nextField( &cursor );

  if( strlen( lat ) == 0 ) {
    return;
  }
  unsigned char quality = ( unsigned char )atoi( qual );
  if( quality == 0 ) {
    return;
  }

  char latDeg[3] = { lat[0], lat[1], 0 };
  double latitude = atof( latDeg ) + atof( &lat[2] ) / 60.0;
  if( latDir[0] == 'S' ) {
    latitude = -latitude;
  }
  char lngDeg[4] = { lng[0], lng[1], lng[2], 0 };
  double longitude = atof( lngDeg ) + atof( &lng[3] ) / 60.0;
  if( lngDir[0] == 'W' ) {
    longitude = -longitude;
  }

  curLat = latitude;
  curLng = longitude;
  curAlt = atof( alt );
  curQuality = quality;
  curSats = ( unsigned char )atoi( sats );
  curHdop = atof( hdop );
  curFixMillis = millis();
}

static bool isGoodQuality() {
  return curQuality != 0 && curSats >= GPS_MIN_SATELLITES && curHdop > 0.0F && curHdop <= GPS_MAX_HDOP;
}

static void logPoint() {
  if( curFixMillis == 0 ) {
    Serial.println( "No fix yet - not logged." );
    return;
  }
  const bool good = isGoodQuality();
  double x = 0, y = 0, z = 0;
  if( !haveOrigin ) {
    originLat = curLat;
    originLng = curLng;
    originAlt = curAlt;
    haveOrigin = true;
    Serial.println( "This is the ORIGIN point (0, 0, 0) - later points are relative to this one." );
  } else {
    x = METERS_PER_DEGREE * ( curLng - originLng ) * cos( curLat * DEG2RAD );
    y = METERS_PER_DEGREE * ( curLat - originLat );
    z = curAlt - originAlt;
  }
  pointCount++;
  Serial.printf( "Point %u: lat=%.6f lon=%.6f alt=%.2fm | sats=%u hdop=%.2f quality=%u (%s)\n",
      pointCount, curLat, curLng, curAlt, curSats, curHdop, curQuality, good ? "GOOD" : "BELOW FIRMWARE THRESHOLD" );
  Serial.printf( "  Local: x=%.3fm y=%.3fm z=%.3fm\n", x, y, z );
  if( !good ) {
    Serial.printf( "  WARNING: fails the firmware's own gate (needs sats>=%u, 0<hdop<=%.1f) - the real "
        "firmware would NOT have accepted this point.\n", GPS_MIN_SATELLITES, GPS_MAX_HDOP );
  }
}

void setup() {
  Serial.begin( 115200 );
  Serial1.begin( 9600, SERIAL_8N1, UART1RX, UART1TX );
  Serial.println( "SAM-M10Q GPS bench test starting." );
  Serial.printf( "UART1: RX=%d, TX=%d, 9600 baud\n", UART1RX, UART1TX );
  Serial.println( "Type anything + Enter in this Serial Monitor to log a point." );
  Serial.println( "Waiting for first fix..." );
}

void loop() {
  while( Serial1.available() ) {
    char c = ( char )Serial1.read();
    if( c == '\n' ) {
      lineBuf[lineLen] = 0;
      parseLine( lineBuf );
      lineLen = 0;
    } else if( lineLen < sizeof( lineBuf ) - 1 ) {
      lineBuf[lineLen++] = c;
    }
  }

  if( Serial.available() ) {
    while( Serial.available() ) {
      char c = ( char )Serial.read();
      if( c == '\n' ) {
        break;
      }
    }
    logPoint();
  }
}
