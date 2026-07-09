#include "kaf_drone.h"

#ifndef KAF_DRONE_FIRMWARE
#define KAF_DRONE_FIRMWARE

#define PERIP_CHARACTERISTIC_ID 0xB0BA5EED

void firmware_registerPeripheral( const peripheral periph );
void firmware_registerStorage( bool( *store )( void*, unsigned short, unsigned short, char ) );
const peripheral* firmware_getPeripheral( unsigned char idx );
unsigned short firmware_handlePersistents( char* buffer, const unsigned short length, const unsigned short address, const unsigned char action );

peripheral firmware_reset();

#endif