#include "firmware.h"
#include "communication.h"
#include "flight.h"

#if ALT_DEFINE
#define memset( A, B, C )
#define memcmp( A, B, C )    false
#define strcpy( A, B )
#define memcpy( A, B, C )
#else
#include <string.h>
#endif

#define PERIPHERAL_COUNT         16
#define PAGE_COUNT               24

#pragma pack( push, 1 )
union extendedCom {
  struct {
    bool firmware;
    unsigned char index;
  } idx;
  char memname[ sizeof( memory::name ) ];
  struct {
    void* address;
    unsigned short length;
  } mempage;
  struct {
    STDBYTE type;
    bool isenabled;
    void* init;
    void* loop;
  } firmware;
  void ( *invokefunc )();
  struct {
    unsigned short position;
    char action;
  } calibstore;
  float rotMat[9];
  struct {
    coordinate position;
    float rotMat[9];
    float motorStates[ FPARLEN( kafenv.cmd.motors ) ];
    float batteryLevel;
    STDBYTE flightMode;
  } statelogger;
};
#pragma pack( pop )

static struct {
  unsigned char peripheralCount;
  unsigned char mempageCount;
  peripheral peripherals[PERIPHERAL_COUNT];
  memory mempages[PAGE_COUNT];
  void( *killFunction )();
  void( *storageFunction )( void*, unsigned short, unsigned short, char );
} firmware;

void firmware_registerMemoryPage( const memory mempage ) {
  if( mempage.address != NULLPTR && mempage.length > 0 && firmware.mempageCount < PAGE_COUNT ) {
    DPRINTF( "[W] Registering Memory: Name=\"%s\", Address=%016x, Length=%u\n", mempage.name, mempage.address, mempage.length );
    for( unsigned char i = 0; i < firmware.mempageCount; i++ ) {
      if( memcmp( firmware.mempages[i].name, mempage.name, sizeof( memory::name ) ) == 0 ) {
        return;
      }
    }
    firmware.mempages[ firmware.mempageCount++ ] = mempage;
  }
}

void firmware_registerStorage( void* ptr, unsigned short ofst, unsigned short len, char type ) {
  firmware.storageFunction( ptr, ofst, len, type );
}

void firmware_registerPeripheral( const peripheral periph ) {
  firmware_registerMemoryPage( *( periph.memorypage ) );
  if( firmware.peripheralCount < PERIPHERAL_COUNT ) {
    DPRINTF( "[W] Registering Peripheral: Name=\"%s\", Type=%02x\n", periph.memorypage->name, periph.type );
    for( unsigned char i = 0; i < firmware.peripheralCount; i++ ) {
      if( memcmp( firmware.peripherals[i].memorypage->name, periph.memorypage->name, sizeof( memory::name ) ) == 0 ) {
        return;
      }
    }
    firmware.peripherals[ firmware.peripheralCount++ ] = periph;
  }
}

void firmware_registerEnvironment( void( *kill )(), void( *store )( void*, unsigned short, unsigned short, char ) ) {
  if( kill != NULLPTR ) {
    firmware.killFunction = kill;
  }
  if( store != NULLPTR ) {
    firmware.storageFunction = store;
  }
}

memory firmware_reset() {
  const memory memDrone    = kaf_reset();
  const memory memCom      = com_reset();
  const memory memFlight   = flight_reset();
  const memory memPages    = { "firmware", sizeof( firmware ), &firmware };
  DPRINTF( "[W] Resetting Firmware\n" );
  firmware.killFunction = []() {};
  firmware.storageFunction = []( void* ptr, unsigned short size, unsigned short address, char action ) {};
  firmware.peripheralCount = 0;
  firmware.mempageCount = 0;
  memset( &firmware.peripherals, 0, sizeof( firmware.peripherals ) );
  memset( &firmware.mempages, 0, sizeof( firmware.mempages ) );
  DPRINTF( "[W] Registering Core Memory Pages\n" );
  firmware_registerMemoryPage( memDrone );
  firmware_registerMemoryPage( memCom );
  firmware_registerMemoryPage( memFlight );
  firmware_registerMemoryPage( memPages );
  DPRINTF( "[W] Registering Extended Communication Protocol\n" );
  com_receiveMessage( COM_REQUEST_MEMPAGES, sizeof( extendedCom::idx ), []( void** response, const void* content, const unsigned short len ) {
    extendedCom* recv = ( extendedCom* )content;
    extendedCom* respData = ( extendedCom* )*response;
    DPRINTF( "[W] Replying Request Memory Page List Command: Firmware=%u, Index=%u\n", recv->idx.firmware, recv->idx.index );
    if( recv->idx.firmware && recv->idx.index < firmware.peripheralCount ) {
      memcpy( respData->memname, firmware.peripherals[ recv->idx.index ].memorypage->address, sizeof( memory::address ) );
    } else if( !recv->idx.firmware && recv->idx.index < firmware.mempageCount ) {
      memcpy( respData->memname, firmware.mempages[ recv->idx.index ].address, sizeof( memory::address ) );
    } else {
      *response = NULLPTR;
      return (unsigned short)0;
    }
    return (unsigned short)sizeof( respData->memname );
  }, []( const void* content, const packet_header header ) {} );
  com_receiveMessage( COM_REQUEST_MEMPAGE, sizeof( extendedCom::memname ), []( void** response, const void* content, const unsigned short len ) {
    char* pageName = ( ( extendedCom* )content )->memname;
    DPRINTF( "[W] Replying Request Memory Page Command: Page Name=%s\n", pageName );
    for( unsigned char i = 0; i < firmware.mempageCount; i++ ) {
      if( memcmp( firmware.mempages[i].name, pageName, sizeof( memory::name ) ) == 0 ) {
        extendedCom* respData = ( extendedCom* )*response;
        respData->mempage.address = firmware.mempages[i].address;
        respData->mempage.length = firmware.mempages[i].length;
        return (unsigned short)sizeof( respData->mempage );
      }
    }
    *response = NULLPTR;
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {} );
  com_receiveMessage( COM_REQUEST_PERIPH, sizeof( extendedCom::memname ), []( void** response, const void* content, const unsigned short len ) {
    char* periphName = ( ( extendedCom* )content )->memname;
    DPRINTF( "[W] Replying Request Peripheral Command: Page Name=%s\n", periphName );
    for( unsigned char i = 0; i < firmware.peripheralCount; i++ ) {
      if( memcmp( firmware.peripherals[i].memorypage->name, periphName, sizeof( memory::name ) ) == 0 ) {
        extendedCom* respData = ( extendedCom* )*response;
        respData->firmware.type = firmware.peripherals[i].type;
        respData->firmware.isenabled = firmware.peripherals[i].isenabled;
        respData->firmware.init = ( void* )firmware.peripherals[i].initFunction;
        respData->firmware.loop = ( void* )firmware.peripherals[i].loopFunction;
        return (unsigned short)sizeof( respData->firmware );
      }
    }
    *response = NULLPTR;
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {} );
  com_receiveMessage( COM_SET_INVOKEFUNC, sizeof( extendedCom::invokefunc ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[W] Replying Set Invoke Function Command\n" );
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[W] Executing Set Invoke Function Command\n" );
    ( ( extendedCom* )content )->invokefunc();
  } );
  com_receiveMessage( COM_SET_KILL, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[W] Replying Set Kill Command\n" );
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) { 
    DPRINTF( "[W] Executing Set Kill Command\n" );
    firmware.killFunction();
  } );
  com_receiveMessage( COM_SET_CALIBSTORE, sizeof( extendedCom::calibstore ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[W] Replying Set Calibration Command\n" );
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) { 
    extendedCom* recv = ( extendedCom* )content;
    DPRINTF( "[W] Executing Set Calibration Command: Position=%u, Action='%c'\n", recv->calibstore.position, recv->calibstore.action );
    firmware.storageFunction( &kafenv.cal, sizeof( kafenv.cal ), recv->calibstore.position, recv->calibstore.action );
  } );
  com_receiveMessage( COM_REQUEST_ROTMAT, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[W] Replying Request Rotation Matrix Commmand\n" );
    extendedCom* respData = ( extendedCom* )*response;
    flight_rotationMatrix( respData->rotMat );
    return (unsigned short)sizeof( extendedCom::rotMat );
  }, []( const void* content, const packet_header header ) { } );
  com_receiveMessage( COM_REQUEST_STATELOG, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[W] Replying Request State Logger Commmand\n" );
    extendedCom* respData = ( extendedCom* )*response;
    respData->statelogger.position = kafenv.state.x;
    flight_rotationMatrix( respData->statelogger.rotMat );
    memcpy( respData->statelogger.motorStates, kafenv.cmd.motors, sizeof( kafenv.cmd.motors ) );
    respData->statelogger.batteryLevel = kafenv.info.battery;
    respData->statelogger.flightMode = kafenv.info.flightMode; 
    return (unsigned short)sizeof( extendedCom::statelogger );
  }, []( const void* content, const packet_header header ) { } );
  DPRINTF( "[W] Finalizing Firmware Setup\n" );
  return memPages;
}