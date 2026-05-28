#include "../core/communication.h"
#include "../core/firmware.h"
#include "../auxilary/common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#endif

#define SERIAL_COM_METHOD 1
#define DEBUG_PRINT       0

extern COMS_BUFFERTYPE;

static struct {
  unsigned int baudrate;
  unsigned int timeout;
  radio coms;
} serial;

static unsigned short serialReceiving() {
  unsigned short serialLength;
  bool messageAvailable = false;
  if( ( serialLength = (unsigned short)Serial.available() ) > 0 ) {
#if DEBUG_PRINT
    int activeCount = 0;
    unsigned char current = 0;
    unsigned short index = 0;
    for( unsigned short i = 0; i < serialLength; i++ ) {
      short b = Serial.read();
      short bn;
      unsigned char lower;
      if( ( bn = b - 'A' ) <= ( 'F' - 'A' ) && bn >= 0 ) {
        lower = (unsigned char)( bn + 0xA );
      } else if( ( bn = b - 'a' ) <= ( 'f' - 'a' ) && bn >= 0  ) {
        lower = (unsigned char)( bn + 0xA );
      } else if( ( bn = b - '0' ) <= ( '9' - '0' ) && bn >= 0  ) {
        lower = (unsigned char)bn;
      } else if( b == ' ' || b == '\n' ) {
        if( activeCount > 0 ) {
          COMS_BUFFER[index++] = current;
          current = 0;
          activeCount = 0;
          if( b == '\n' ) {
            break;
          }
        }
        continue;
      } else {
        break;
      }
      if( activeCount++ > 1 ) {
        break;
      }
      current = ( current << 4 ) + lower;
    }
    serialLength = index;
#else
    serialLength = serialLength < sizeof( COMS_BUFFER ) ? serialLength : sizeof( COMS_BUFFER );
    Serial.readBytes( COMS_BUFFER, serialLength );
#endif
  }
  return serialLength;
}

static void serialSending( void* buffer, unsigned short len ) {
  unsigned char* ptr = (unsigned char*)buffer;
#if DEBUG_PRINT
  Serial.printf( "[P] Serial Message: { " );
  for( unsigned short i = 0; i < len; i++ ) {
    Serial.printf( "%02x ", ptr[i] );
  }
  Serial.printf( "}\n" );
#else
  Serial.write( ( char* )buffer, len );
#endif
}

void peripheral_serialLoop() {
  serial.coms.currentTime = millis();
  com_step( &serial.coms );
}

void peripheral_serialInit() {
  firmware_registerPeripheral( { "serial", 0, sizeof( serial.coms ), &serial.coms, &peripheral_serialInit, &peripheral_serialLoop } );
  DPRINTF( "[P] Initializing Serial\n" );
  serial.coms = { 0, SERIAL_COM_METHOD, false, true, COMS_BUFFER, &serialReceiving, &serialSending, &serialSending };
}

void peripheral_serialStart( unsigned int baudrate ) {
  serial.baudrate = baudrate;
  serial.timeout = 250;
  Serial.begin( serial.baudrate );
  Serial.setTimeout( serial.timeout );
  delay( 1000 );
  Serial.printf( "Serial Started--------------------------------------------------------------------------------------------------\n" );
}