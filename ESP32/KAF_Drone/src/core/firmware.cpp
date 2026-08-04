#include "firmware.h"
#include "communication.h"
#include "flight.h"

#if ALT_DEFINE
static void* memset( void* dest, int ch, size_t count );
static int strncmp( const char* lhs, const char* rhs, size_t count );
static void* memcpy( void* dest, const void* src, size_t count );
#else
#include <string.h>
#endif

#define PAGE_COUNT        25

struct persistentheader {
  unsigned int characteristicID;
  unsigned int versionID;
};

static struct {
  unsigned char peripheralCount;
  peripheral peripherals[PAGE_COUNT];
  unsigned short persistLength;
  unsigned char persistentCount;
  unsigned char persistents[PAGE_COUNT];
  bool( *storageFunction )( void*, unsigned short, unsigned short, char );
} firmware;

void firmware_registerPeripheral( const peripheral periph ) {
  if( firmware.peripheralCount < PAGE_COUNT ) {
    for( unsigned char i = 0; i < firmware.peripheralCount; i++ ) {
      if( strncmp( firmware.peripherals[i].name, periph.name, sizeof( peripheral::name ) ) == 0 ) {
        return;
      }
    }
    DPRINTF( "[W] Registering Peripheral: Count=%u/%u, Name=\"%s\", Length=%u, Memory=%08x, Initializer=%08x, Looping=%08x\n",
        firmware.peripheralCount + 1, PAGE_COUNT, periph.name, periph.length, periph.memory, periph.init, periph.loop );
    if( periph.persist > 0 && periph.memory != NULLPTR ) {
      firmware.persistents[ firmware.persistentCount++ ] = firmware.peripheralCount;
      firmware.persistLength += periph.persist;
      DPRINTF( "[W] Peripheral Persistent: Count=%u, Length=%u, Total Length=%u\n", 
          firmware.persistentCount, periph.persist, firmware.persistLength );
    }
    firmware.peripherals[ firmware.peripheralCount++ ] = periph;
  }
}

void firmware_registerStorage( bool( *store )( void*, unsigned short, unsigned short, char ) ) {
  if( store != NULLPTR ) {
    firmware.storageFunction = store;
  }
}

const peripheral* firmware_getPeripheral( unsigned char idx ) {
  return idx < PAGE_COUNT ? &firmware.peripherals[idx] : NULLPTR;
}

unsigned short firmware_handlePersistents( char* buffer, const unsigned short length, const unsigned short address, const unsigned char action ) {
  switch( action ) {
    case 'R' : case 'L' : {
      DPRINTF( "[W] Reading Persistent Memory: Length=%u, Address=%u\n", firmware.persistLength, address );
      if( length >= firmware.persistLength && ( action == 'L' || firmware.storageFunction( buffer, firmware.persistLength, address, 'r' ) ) ) {
        persistentheader* header = ( persistentheader* )( ( void* )buffer );
        DPRINTF( "[W] Characteristic ID: ID=%08x, Version=%08x\n", header->characteristicID, header->versionID );
        if( header->characteristicID == PERIP_CHARACTERISTIC_ID && header->versionID == kafenv.info.version ) {
          unsigned short index = sizeof( persistentheader );
          for( unsigned char i = 0; i < firmware.persistentCount; i++ ) {
            const unsigned char idx = firmware.persistents[i];
            DPRINTF( "[W] Persistent Read: Name=\"%s\", Location=%04x->%04x, Length=%u\n", 
                firmware.peripherals[idx].name, &buffer[index], firmware.peripherals[idx].memory, firmware.peripherals[idx].persist  );
            memcpy( firmware.peripherals[idx].memory, &buffer[index], firmware.peripherals[idx].persist );
            index += firmware.peripherals[idx].persist;
          }
          return firmware.persistLength;
        }
      }
      return 0;
    }
    case 'W' : case 'S' : {
      DPRINTF( "[W] Writing Persistent Memory: Length=%u, Address=%u\n", firmware.persistLength, address );
      if( length >= firmware.persistLength ) {
        *( ( persistentheader* )( ( void* )buffer ) ) = { PERIP_CHARACTERISTIC_ID, kafenv.info.version };
        unsigned short index = sizeof( persistentheader );
        for( unsigned char i = 0; i < firmware.persistentCount; i++ ) {
          const unsigned char idx = firmware.persistents[i];
          DPRINTF( "[W] Persistent Write: Name=\"%s\", Location=%04x->%04x, Length=%u\n", 
              firmware.peripherals[idx].name, firmware.peripherals[idx].memory, &buffer[index], firmware.peripherals[idx].persist  );
          memcpy( &buffer[index], firmware.peripherals[idx].memory, firmware.peripherals[idx].persist );
          index += firmware.peripherals[idx].persist;
        }
        return action == 'S' || firmware.storageFunction( buffer, firmware.persistLength, address, 'w' ) ? firmware.persistLength : 0;
      }
      return 0;
    }
    case 'D' : {
      DPRINTF( "[W] Clearing Persistent Memory: Length=%u, Address=%u\n", firmware.persistLength, address );
      if( length >= firmware.persistLength ) {
        memset( buffer, 0, firmware.persistLength );
        return firmware.storageFunction( buffer, firmware.persistLength, address, 'w' ) ? sizeof( persistentheader ) : 0;
      }
      return 0;
    }
    default : {
      return firmware.storageFunction( buffer, length, address, action ) ? length : 0;
    }
  }
}

peripheral firmware_reset() {
  const peripheral memDrone    = kaf_reset();
  const peripheral memCom      = com_reset();
  const peripheral memFlight   = flight_reset();
  const peripheral memPages    = { "firmware", false, sizeof( firmware ), &firmware, [](){ firmware_reset(); }, NULLPTR };
  DPRINTF( "[W] Resetting Firmware\n" );
  firmware.peripheralCount = 0;
  memset( &firmware.peripherals, 0, sizeof( firmware.peripherals ) );
  firmware.persistLength = sizeof( persistentheader );
  firmware.persistentCount = 0;
  memset( &firmware.persistents, MAXBYTE, sizeof( firmware.persistents ) );
  firmware.storageFunction = []( void* ptr, unsigned short size, unsigned short address, char action ) { return false; };
  DPRINTF( "[W] Registering Core Memory Pages\n" );
  firmware_registerPeripheral( memDrone );
  firmware_registerPeripheral( memCom );
  firmware_registerPeripheral( memFlight );
  firmware_registerPeripheral( memPages );
  DPRINTF( "[W] Registering Extended Communication Protocol\n" );
  com_receiveMessage( COM_REQUEST_PERIPHID, sizeof( unsigned char ), []( void** response, const void* content, const unsigned short len ) {
    const unsigned char index = *( ( const unsigned char* )content );
    DPRINTF( "[W] Replying Request Peripheral by ID Command: Index=%u\n", index );
    if( index < firmware.peripheralCount ) {
      *( ( peripheral* )*response ) = firmware.peripherals[index];
      return (unsigned short)sizeof( peripheral );
    } else {
      *response = NULLPTR;
      return (unsigned short)0;
    }
  }, []( const void* content, const packet_header header ) {} );
  com_receiveMessage( COM_REQUEST_PERIPH, sizeof( char ), []( void** response, const void* content, const unsigned short len ) {
    const char* name = ( const char* )content;
    DPRINTF( "[W] Replying Request Peripheral by Name Command: Page Name=%s\n", name );
    for( unsigned char i = 0; i < firmware.peripheralCount; i++ ) {
      if( strncmp( firmware.peripherals[i].name, name, sizeof( peripheral::name ) ) == 0 ) {
        *( ( peripheral* )*response ) = firmware.peripherals[i];
        return (unsigned short)sizeof( peripheral );
      }
    }
    *response = NULLPTR;
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {} );
  com_receiveMessage( COM_SET_INVOKEFUNC, sizeof( void* ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[W] Replying Set Invoke Function Command\n" );
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[W] Executing Set Invoke Function Command: Address=%08x\n", content );
    ( ( void(*)( ) )content )();
  } );
  com_receiveMessage( COM_REQUEST_ROTMAT, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[W] Replying Request Rotation Matrix Commmand\n" );
    flight_rotationMatrix( ( float* )*response );
    return (unsigned short)sizeof( float[9] );
  }, []( const void* content, const packet_header header ) { } );
  DPRINTF( "[W] Finalizing Firmware Setup\n" );
  return memPages;
}