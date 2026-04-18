#include "kaf_drone.h"
#include "string.h"

//DEBUG PRINT_________________________________________________________________________________________________
static void loopDebugFlight() {
  
}

static void loopDebugCommand() {
  
}

static void initDebug( int id ) {
  strcpy( kafenv.p.peripheral[id].name, "DEBUG" );
  kafenv.p.peripheral[id].enabled = true;
  kafenv.p.peripheral[id].loopType = 3;
  kafenv.p.peripheral[id].dataSize = 0;
  kafenv.p.peripheral[id].initFunction = &initDebug;
  kafenv.p.peripheral[id].loopFunction = &loopDebugFlight;
  kafenv.p.peripheral[id].auxLoopFunction = &loopDebugCommand;
  kafenv.p.peripheral[id].data = 0;
}

void setup() {
  kafenv.u.deviceID                 = 'A';
  kafenv.n.comTaskDelay             = 0;
  strcpy( kafenv.n.networkName,       "iPhone" );
  strcpy( kafenv.n.networkPassword,   "password" );
  initDebug( 7 );
  firmwareSetup();
}
void loop() {}