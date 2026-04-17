#include "kaf_drone.h"

#ifndef KAF_DRONE_FIRMWARE
#define KAF_DRONE_FIRMWARE

struct peripheral {
  STDBYTE type;                 //
  bool isenabled;               //
  memory* memorypage;           //
  void( *initFunction )();      //
  void( *loopFunction )();      //
};

void firmware_registerMemoryPage( const memory memory );
void firmware_registerPeripheral( const peripheral peripheral );
void firmware_registerEnvironment( void( *kill )(), void( *store )( void*, unsigned short, unsigned short, char ) );
void firmware_registerStorage( void* ptr, unsigned short ofst, unsigned short len, char type );

memory firmware_reset();

#endif