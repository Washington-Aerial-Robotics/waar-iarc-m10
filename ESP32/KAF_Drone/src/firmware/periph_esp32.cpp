#include "../core/firmware.h"
#include "../core/communication.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include <ESP.h>
#include <EEPROM.h>
#endif

#define EEPROM_LENGTH 2000

void peripheral_esp32KillCommand() {
  DPRINTF( "[P] Restarting ESP32\n" );
  delay( 1000 );
  ESP.restart();
}

void peripheral_esp32Init() {
  firmware_registerPeripheral( { "esp32", 0, 0, NULLPTR, &peripheral_esp32Init, NULLPTR } );
  DPRINTF( "[P] Initializing ESP32\n" );
  EEPROM.begin( EEPROM_LENGTH );
  firmware_registerStorage( [] ( void* ptr, unsigned short len, unsigned short ofst, char type ) {
    DPRINTF( "[P] ESP32 EEPROM Action: Type='%c', Location=%u, Length=%u\n", type, ofst, len );
    if( ofst + len < EEPROM_LENGTH ) {
      if( type == 'r' ) {
        DPRINTF( "[P] ESP32 EEPROM Read: Offset=%04x, Length=%04x\n", ofst, len );
#if PERSIST
        EEPROM.readBytes( ofst, ptr, len );
        return true;
#else
        return false;
#endif
      } else if( type == 'w' ) {
        DPRINTF( "[P] ESP32 EEPROM Write: Offset=%04x, Length=%04x\n", ofst, len );
#if PERSIST
        EEPROM.writeBytes( ofst, ptr, len );
        EEPROM.commit();
        Serial.printf( "GG\076G[ofst=%u,len=%u]\n", ofst, len );
        return true;
#else
        return false;
#endif
      }
    }
  } );
  com_receiveMessage( COM_SET_KILL, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[P] Replying Set Kill Command\n" );
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[P] Executing Set Kill Command\n" );
    ESP.restart();
  } );
}