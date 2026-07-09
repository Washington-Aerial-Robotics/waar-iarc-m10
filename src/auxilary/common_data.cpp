#include "../core/firmware.h"
#include "../core/flight.h"
#include "../core/communication.h"
#include "common_data.h"

#if ALT_DEFINE
static void* memset( void* dest, int ch, size_t count );
#else
#include <string.h>
#include <math.h>
#endif

COMS_BUFFERTYPE;
FLIGHT_BUFFERTYPE;

void peripheral_commonInit() {
  firmware_registerPeripheral( { "com_comms", 0, sizeof( COMS_BUFFER ), &COMS_BUFFER, peripheral_commonInit, NULLPTR } );
  firmware_registerPeripheral( { "com_imu", 0, sizeof( FLIGHT_BUFFER ), &FLIGHT_BUFFER, peripheral_commonInit, NULLPTR } );
  DPRINTF( "[W] Initializing Common Buffers\n" );
  memset( &COMS_BUFFER, 0, sizeof( COMS_BUFFER ) );
  memset( &FLIGHT_BUFFER, 0, sizeof( FLIGHT_BUFFER ) );
}