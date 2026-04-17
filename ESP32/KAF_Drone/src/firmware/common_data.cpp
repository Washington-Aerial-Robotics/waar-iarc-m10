#include "../core/firmware.h"
#include "../core/flight.h"
#include "../core/communication.h"
#include "common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <string.h>
#include <math.h>
#endif

COMS_BUFFERTYPE;
FLIGHT_BUFFERTYPE;

void peripheral_commonInit() {
  firmware_registerMemoryPage( { "common_coms", sizeof( COMS_BUFFER ), &COMS_BUFFER } );
  firmware_registerMemoryPage( { "common_imu", sizeof( FLIGHT_BUFFER ), &FLIGHT_BUFFER } );
  DPRINTF( "[W] Initializing Common Buffers\n" );
  memset( &COMS_BUFFER, 0, sizeof( COMS_BUFFER ) );
  memset( &FLIGHT_BUFFER, 0, sizeof( FLIGHT_BUFFER ) );
}