#include "../core/firmware.h"
#include "../core/flight.h"
#include "common_data.h"
#include "estimation.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#endif

extern SENSOR_BUFFERTYPE;

//DO NOT MODIFY ANYTHING ABOVE THIS LINE_________________________________________________________________________________

static struct {
  bool working = false;
  //PUT ALL GLOBAL VARIABLES HERE
} sam;

void peripheral_samm10qLoop() {
  //PUT RUNTIME LOOP CODE HERE
}

void peripheral_samm10qInit() {
  memory mempage = { "samm10q", sizeof( sam ), &sam };
  firmware_registerPeripheral( { 0, true, &mempage, &peripheral_samm10qLoop, &peripheral_samm10qInit } );
  DPRINTF( "[P] Initializing SAMM10Q\n" );
  //PUT INITIALIZATION CODE HERE
}