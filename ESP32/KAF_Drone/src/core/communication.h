#include "kaf_drone.h"

#define COM_ID               0b00111111//actual message type id of the packet
#define COM_CMD              0b01000000//message type is command that requires a response (SET, REQUEST)
#define COM_FWD              0b10000000//forwarding bit indicating if packet should be forwarded
//60-63 status update communication signals
#define COM_SUCCESS          (       0 | 60 )//,
#define COM_FAILURE          (       0 | 61 )//,
#define COM_ACKNOWLEDGED     (       0 | 62 )//,
#define COM_INVALID          (       0 | 63 )//,
//00-19 default communication methods already implemented
#define COM_REQUEST_PING     ( COM_CMD |  0 )//,COM_PONG
#define COM_REPLY_PING       (       0 |  0 )//,
#define COM_SET_TRIGGER      ( COM_CMD | 10 )//triggervalue,COM_ACKNOWLEDGED
#define COM_REQUEST_DEVICES  ( COM_CMD |  1 )//entitylistoffset,COM_REPLY_DEVICES
#define COM_REPLY_DEVICES    (       0 |  1 )//entitylist,
#define COM_SET_SENDMSG      ( COM_CMD | 11 )//sendmsg,COM_ACKNOWLEDGED
#define COM_REQUEST_STATE    ( COM_CMD |  2 )//,COM_REPLY_STATE
#define COM_REPLY_STATE      (       0 |  2 )//dronestate,
#define COM_SET_FLIGHTMODE   ( COM_CMD | 12 )//flightmode,COM_SUCCESS,
#define COM_REQUEST_INFO     ( COM_CMD |  3 )//,COM_REPLY_INFO
#define COM_REPLY_INFO       (       0 |  3 )//droneinfo,
#define COM_SET_INFO         ( COM_CMD | 13 )//droneinfo,COM_SUCCESS
#define COM_REQUEST_STEST    ( COM_CMD |  4 )//,COM_REPLY_ST_EST
#define COM_REPLY_STEST      (       0 |  4 )//stateestimate,
#define COM_SET_STEST        ( COM_CMD | 14 )//stateestimate,COM_SUCCESS
#define COM_REQUEST_SETPT    ( COM_CMD |  5 )//,COM_REPLY_SETPT
#define COM_REPLY_SETPT      (       0 |  5 )//setpoint,
#define COM_SET_SETPT        ( COM_CMD | 15 )//setpoint,COM_SUCCESS
#define COM_REQUEST_MOTORS   ( COM_CMD |  6 )//,COM_REPLY_MOTORS
#define COM_REPLY_MOTORS     (       0 |  6 )//motorvalue,
#define COM_SET_MOTORS       ( COM_CMD | 16 )//motorvalue,COM_SUCCESS
#define COM_REQUEST_CALIB    ( COM_CMD |  7 )//,COM_REPLY_CALIB
#define COM_REPLY_CALIB      (       0 |  7 )//calibration,
#define COM_SET_CALIB        ( COM_CMD | 17 )//calibration,COM_SUCCESS
#define COM_REQUEST_KAFENV   ( COM_CMD |  8 )//,COM_REPLY_KAFENV
#define COM_REPLY_KAFENV     (       0 |  8 )//drone,
#define COM_SET_KAFENV       ( COM_CMD | 18 )//drone,COM_SUCCESS
#define COM_REQUEST_MEMORY   ( COM_CMD |  9 )//memtransfer,COM_REPLY_MEMORY
#define COM_REPLY_MEMORY     (       0 |  9 )//memtransfer,
#define COM_SET_MEMORY       ( COM_CMD | 19 )//memtransfer,COM_SUCCESS
//20-49 com protocols implements outside of the default com algo
#define COM_REQUEST_MEMPAGES ( COM_CMD | 20 )//firmware
#define COM_REPLY_MEMPAGES   (       0 | 20 )//firmware
#define COM_REQUEST_MEMPAGE  ( COM_CMD | 21 )//firmware
#define COM_REPLY_MEMPAGE    (       0 | 21 )//firmware
#define COM_REQUEST_PERIPH   ( COM_CMD | 22 )//firmware
#define COM_REPLY_PERIPH     (       0 | 22 )//firmware
#define COM_REQUEST_WIFI     ( COM_CMD | 23 )//wifi
#define COM_REPLY_WIFI       (       0 | 23 )//wifi
#define COM_REQUEST_ROTMAT   ( COM_CMD | 25 )//firmware
#define COM_REPLY_ROTMAT     (       0 | 25 )//firmware
#define COM_REQUEST_STATELOG ( COM_CMD | 26 )//firmware
#define COM_REPLY_STATELOG   ( COM_CMD | 26 )//firmware
#define COM_SET_WIFI         ( COM_CMD | 27 )//wifi
#define COM_SET_KILL         ( COM_CMD | 28 )//firmware
#define COM_SET_INVOKEFUNC   ( COM_CMD | 29 )//firmware
#define COM_SET_CALIBSTORE   ( COM_CMD | 30 )//firmware
#define COM_SET_STARTUP      ( COM_CMD | 31 )//commander
#define COM_SET_SAVESTORAGE  ( COM_CMD | 32 )//commander
#define COM_SET_WIFIAP       ( COM_CMD | 33 )//wifi
//50-59 communication method specific messages
#define COM_RANGING_1        ( COM_CMD | 50 )//dw3000
#define COM_RANGING_2        ( COM_CMD | 51 )//dw3000
#define COM_RANGING_3        (       0 | 52 )//dw3000

struct radio {
  unsigned long currentTime = 0;
  STDBYTE method = 0;
  bool allowBroadcast = true;
  bool fwdReply = true;
  void* packet = 0;
  unsigned short( *receiving )() = []() { return ( unsigned short )0; };
  void( *replying )( void*, unsigned short ) = []( void* ptr, unsigned short len ) {};
  void( *sending )( void*, unsigned short ) = []( void* ptr, unsigned short len ) {};
};

struct entity {
  STDBYTE entityID;                             //Unique id of peer drone
  STDBYTE flightMode;                           //
  STDBYTE liason;                               //
  unsigned char nodeOrder;                      //
  unsigned long lastSeen;                       //Last observation of peer drone in ms
  coordinate position;                          //Position estimate in m of peer drone
  float distance;                               //Distance in m between peer drone and this drone
};

struct packet_header {
  STDBYTE toID;                          //ID of the intended receiver of the packet
  STDBYTE fromID;                        //ID of the sender of the communication packet
  STDBYTE messageType;                   //Message type of the communication packet
  STDBYTE messageID;                     //unique identifier for message of the same subject
};

entity* com_registerEntity( const STDBYTE entityID );
entity* com_getEntityById( const STDBYTE entityID );
unsigned char com_getEntity( entity* entity, unsigned char index );

void* com_sendMessage( const STDBYTE method, const unsigned char attempts, packet_header header, const void* content, 
    const unsigned short size, void( *handler )( const void*, const unsigned short, const packet_header ) );
void com_receiveMessage( const STDBYTE type, const unsigned char minContentSize, 
      unsigned short( *reply )( void**, const void*, const unsigned short ), void( *process )( const void*, const packet_header ) );

memory com_reset();
void com_step( const radio* radio );