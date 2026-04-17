#include "../core/firmware.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include <ESP.h>
#include <EEPROM.h>
#endif

#define EEPROM_LENGTH 2000

void peripheral_esp32Init() {
  memory mempage = { "esp32", 0, NULL };
  firmware_registerPeripheral( { 0, true, &mempage, &peripheral_esp32Init, NULL } );
  DPRINTF( "[P] Initializing ESP32\n" );
  EEPROM.begin( EEPROM_LENGTH );
  firmware_registerEnvironment( [] () {
    DPRINTF( "[P] Shutting Down ESP32\n" );
    ESP.restart();
  }, [] ( void* ptr, unsigned short ofst, unsigned short len, char type ) {
    DPRINTF( "[P] ESP32 EEPROM Action: Type='%c', Location=%u, Length=%u\n", type, ofst, len );
    if( ofst + len < EEPROM_LENGTH ) {
      if( type == 'r' ) {
        DPRINTF( "[P] ESP32 EEPROM Read: Offset=%04x, Length=%04x\n", ofst, len );
#if PERSIST
        EEPROM.readBytes( ofst, ptr, len );
#endif
      } else if( type == 'w' ) {
        DPRINTF( "[P] ESP32 EEPROM Write: Offset=%04x, Length=%04x\n", ofst, len );
#if PERSIST
        EEPROM.writeBytes( ofst, ptr, len );
        EEPROM.commit();
        Serial.printf( "GG\076G[ofst=%u,len=%u]\n", ofst, len );
#endif
      }
    }
  } );
}
