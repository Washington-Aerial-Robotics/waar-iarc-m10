#include "kaf_drone.h"

#define COM_ID               0b00111111//actual message type id of the packet
#define COM_CMD              0b01000000//message type is command that requires a response (SET, REQUEST)
#define COM_FWD              0b10000000//forwarding bit indicating if packet should be forwarded
//60-63 status update communication signals
#define COM_SUCCESS          (       0 | 60 )//communication,,
#define COM_FAILURE          (       0 | 61 )//communication,,
#define COM_ACKNOWLEDGED     (       0 | 62 )//communication,,
#define COM_INVALID          (       0 | 63 )//communication,,
//00-19 default communication methods already implemented
#define COM_REQUEST_PING     ( COM_CMD |  0 )//communication,,COM_PONG
#define COM_REPLY_PING       (       0 |  0 )//communication,,
#define COM_SET_ACTUATION    ( COM_CMD | 10 )//communication,actuation,COM_ACKNOWLEDGED
#define COM_REQUEST_DEVICES  ( COM_CMD |  1 )//communication,entitylistoffset,COM_REPLY_DEVICES
#define COM_REPLY_DEVICES    (       0 |  1 )//communication,entitylist,
#define COM_SET_SENDMSG      ( COM_CMD | 11 )//communication,sendmsg,COM_ACKNOWLEDGED
#define COM_REQUEST_STATE    ( COM_CMD |  2 )//communication,,COM_REPLY_STATE
#define COM_REPLY_STATE      (       0 |  2 )//communication,dronestate,
#define COM_SET_FLIGHTMODE   ( COM_CMD | 12 )//communication,flightmode,COM_SUCCESS,
#define COM_REQUEST_INFO     ( COM_CMD |  3 )//communication,,COM_REPLY_INFO
#define COM_REPLY_INFO       (       0 |  3 )//communication,droneinfo,
#define COM_SET_INFO         ( COM_CMD | 13 )//communication,droneinfo,COM_SUCCESS
#define COM_REQUEST_STEST    ( COM_CMD |  4 )//communication,,COM_REPLY_ST_EST
#define COM_REPLY_STEST      (       0 |  4 )//communication,stateestimate,
#define COM_SET_STEST        ( COM_CMD | 14 )//communication,stateestimate,COM_SUCCESS
#define COM_REQUEST_SETPT    ( COM_CMD |  5 )//communication,,COM_REPLY_SETPT
#define COM_REPLY_SETPT      (       0 |  5 )//communication,setpoint,
#define COM_SET_SETPT        ( COM_CMD | 15 )//communication,setpoint,COM_SUCCESS
#define COM_REQUEST_MOTORS   ( COM_CMD |  6 )//communication,,COM_REPLY_MOTORS
#define COM_REPLY_MOTORS     (       0 |  6 )//communication,motorvalue,
#define COM_SET_MOTORS       ( COM_CMD | 16 )//communication,motorvalue,COM_SUCCESS
#define COM_REQUEST_CALIB    ( COM_CMD |  7 )//communication,,COM_REPLY_CALIB
#define COM_REPLY_CALIB      (       0 |  7 )//communication,calibration,
#define COM_SET_CALIB        ( COM_CMD | 17 )//communication,calibration,COM_SUCCESS
#define COM_REQUEST_KAFENV   ( COM_CMD |  8 )//communication,,COM_REPLY_KAFENV
#define COM_REPLY_KAFENV     (       0 |  8 )//communication,drone,
#define COM_SET_KAFENV       ( COM_CMD | 18 )//communication,drone,COM_SUCCESS
#define COM_REQUEST_MEMORY   ( COM_CMD |  9 )//communication,memtransfer,COM_REPLY_MEMORY
#define COM_REPLY_MEMORY     (       0 |  9 )//communication,memtransfer,
#define COM_SET_MEMORY       ( COM_CMD | 19 )//communication,memtransfer,COM_SUCCESS
//20-49 com protocols implements outside of the default com algo
#define COM_REQUEST_PERIPHID ( COM_CMD | 20 )//firmware
#define COM_REPLY_PERIPHID   (       0 | 20 )//firmware
#define COM_REQUEST_PERIPH   ( COM_CMD | 21 )//firmware
#define COM_REPLY_PERIPH     (       0 | 21 )//firmware
#define COM_REQUEST_ROTMAT   ( COM_CMD | 23 )//firmware
#define COM_REPLY_ROTMAT     (       0 | 23 )//firmware
#define COM_SET_INVOKEFUNC   ( COM_CMD | 24 )//firmware
#define COM_SET_STARTUP      ( COM_CMD | 25 )//commander
#define COM_SET_STORAGE      ( COM_CMD | 26 )//commander
#define COM_SET_TRAJECTORY   ( COM_CMD | 27 )//commander
#define COM_SET_TRAJCONFIG   ( COM_CMD | 28 )//commander
#define COM_SET_PIDTUNING    ( COM_CMD | 30 )//pid_tuner
#define COM_SET_SIM_VARS     ( COM_CMD | 31 )//pid_tuner
#define COM_SET_RESP_VARS    ( COM_CMD | 32 )//pid_tuner
#define COM_REQUEST_WIFI     ( COM_CMD | 33 )//wifi
#define COM_REPLY_WIFI       (       0 | 33 )//wifi
#define COM_SET_WIFI         ( COM_CMD | 34 )//wifi
#define COM_SET_KILL         ( COM_CMD | 35 )//esp32
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

peripheral com_reset();
void com_step( const radio* radio );