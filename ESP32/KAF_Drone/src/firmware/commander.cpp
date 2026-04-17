#include "../core/firmware.h"
#include "../core/communication.h"
#include "../core/flight.h"

#if ALT_DEFINE
#define memset( A, B, C )
#define memcpy( A, B, C )
#define rand()            0
#define NAN               0
#else
#include <string.h>
#include <math.h>
#endif

#define GROUND_STATION_ID 'G'
#define CHARACTERISTIC_ID 0x123C0FFEEBABE321
#define COMMANDER_COM_METHOD 5
#define STORAGE_CAPACITY 0x5000
#define STARTUP_CMD_COUNT 3
#define ATTITUDE_THRESHOLD_TIME 3000 //ms
#define AUTO_DESCENT_RATE 0.25F // m/s

struct commander_struct {
  bool doReadStorage;
  bool doSaveStorage;
  unsigned long lastTime;
  unsigned long attitudeLimit;
  struct storageformat {
    struct {
      unsigned long long characteristicID;
      unsigned int versionID;
    } header;
    drone_state storedstate;
    struct startupcmd {
      unsigned char length;
      union {
        packet_header header;
        STDBYTE bytes[127];
      };
    } startupcmds[STARTUP_CMD_COUNT];
  } storage;
} commander;

struct startupcommand_packet {
  unsigned char index;
  commander_struct::storageformat::startupcmd content;
};

memory commander_reset() {
  DPRINTF( "[H] Resetting Commander\n" );
  commander.doReadStorage = true;
  commander.lastTime = 0;
  commander.doSaveStorage = false;
  commander.attitudeLimit = 0;
  for( unsigned char i = 0; i < STARTUP_CMD_COUNT; i++ ) {
    commander.storage.startupcmds[i].length = 0;
    memset( commander.storage.startupcmds[i].bytes, 0, sizeof( commander.storage.startupcmds[i].bytes ) );
  }
  com_receiveMessage( COM_SET_STARTUP, 1, []( void** response, const void* content, const unsigned short len ) {
    startupcommand_packet* comContent = ( startupcommand_packet* )content;
    DPRINTF( "[H] Replying Set Startup Command: Index=%u Length=%u\n", comContent->index, comContent->content.length );
    if( comContent->index >= STARTUP_CMD_COUNT || comContent->content.length > 127 ) {
      *response = NULLPTR;
    }
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    DPRINTF( "[H] Executing Set Startup Command\n" );
    startupcommand_packet* comContent = ( startupcommand_packet* )content;
    commander.storage.startupcmds[ comContent->index ].length = comContent->content.length;
    memcpy( commander.storage.startupcmds[ comContent->index ].bytes, comContent->content.bytes, comContent->content.length );
    commander.doSaveStorage = true;
  } );
  com_receiveMessage( COM_SET_SAVESTORAGE, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[H] Replying Set Save Storage Command\n" );
    commander.doSaveStorage = true;
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) { } );
  memory mempage = { "commander", sizeof( commander ), &commander };
  return mempage;
}

void commander_step( const unsigned long currentTime ) {
  if( commander.doReadStorage ) {
    firmware_registerStorage( &commander.storage, 0, sizeof( commander.storage ), 'r' );
    if( commander.storage.header.characteristicID == CHARACTERISTIC_ID && commander.storage.header.versionID == kafenv.info.version ) {
      DPRINTF( "[H] Valid Storage Read: Version=%08x\n", commander.storage.header.versionID );
      commander.doSaveStorage = false;
      kafenv = commander.storage.storedstate;
      radio com_radio = { currentTime, COMMANDER_COM_METHOD, false, false, NULLPTR, 
          []() { return ( unsigned short )0; }, []( void* ptr, unsigned short len ) {}, []( void* ptr, unsigned short len ) {} };
      entity* gs = com_registerEntity( GROUND_STATION_ID );
      gs->flightMode = NULL_MODE;
      gs->liason = COMMANDER_COM_METHOD;
      gs->nodeOrder = 0;
      gs->lastSeen = currentTime;
      gs->position = { 0, 0, 0 };
      gs->distance = NAN;
      for( unsigned char i = 0; i < STARTUP_CMD_COUNT; i++ ) {
        if( commander.storage.startupcmds[i].length > 0 ) {
          commander.storage.startupcmds[i].header.fromID = GROUND_STATION_ID;
          commander.storage.startupcmds[i].header.toID = kafenv.info.deviceID;
          com_radio.packet = &commander.storage.startupcmds[i].bytes;
          com_step( &com_radio );
        }
      }
    } else {
      DPRINTF( "[H] Invalid Storage Read Signature\n" );
      commander.doSaveStorage = true;
    }
    commander.doReadStorage = false;
  }
  if( kafenv.info.actuation ) {
    commander.attitudeLimit = kafenv.state.q.x * kafenv.state.q.x + kafenv.state.q.y * kafenv.state.q.y > 1 ? 
        commander.attitudeLimit + ( currentTime - commander.lastTime ) : 0;
    if( com_getEntityById( GROUND_STATION_ID ) == NULLPTR || kafenv.info.battery < 5.0F ) {
      DPRINTF( "[H] Quadcopter Critical Error, Initiating Landing: Time=%lu\n", currentTime );
      flight_runFunction( []() {
        kafenv.info.flightMode = TRAJECTORY_MODE;
        FPFILL0( q, kafenv.cmd.setpoints );
        kafenv.cmd.setpoints[ 1] = kafenv.state.x.z / AUTO_DESCENT_RATE;
        kafenv.cmd.setpoints[ 6] = kafenv.state.x.x;
        kafenv.cmd.setpoints[11] = kafenv.state.x.y;
        kafenv.cmd.setpoints[15] = -AUTO_DESCENT_RATE;
        kafenv.cmd.setpoints[16] = kafenv.state.x.z;
        kafenv.cmd.setpoints[21] = kafenv.state.q.z;
      } );
      commander.doSaveStorage = true;
    } else if( commander.attitudeLimit > ATTITUDE_THRESHOLD_TIME ) {
      DPRINTF( "[H] Quadcopter Crash Detected, Terminating Flight: Time=%lu\n", currentTime );
      flight_runFunction( []() {
        const STDBYTE mode = kafenv.info.flightMode & DEFAULT_MODES_MASK;
        kafenv.info.flightMode = ( kafenv.info.flightMode & ~DEFAULT_MODES_MASK ) | ( mode == NULL_MODE || mode == CALIBRATION_MODE ? 
            mode : ( mode == ACTUATION_MODE || mode == MOTOR_SETPOINT_MODE ? NULL_MODE : INACTIVE_MODE ) );
        kafenv.info.actuation = false;
      } );
    }
  }
  if( commander.doSaveStorage ) {
    commander.doSaveStorage = false;
    commander.storage.header.characteristicID = CHARACTERISTIC_ID;
    commander.storage.header.versionID = kafenv.info.version;
    commander.storage.storedstate = kafenv;
    DPRINTF( "[H] Storage Write: Time=%lu, Length=%04x\n", currentTime, sizeof( commander.storage ) );
    firmware_registerStorage( &commander.storage, 0, sizeof( commander.storage ), 'w' );
  }
  commander.lastTime = currentTime;
}