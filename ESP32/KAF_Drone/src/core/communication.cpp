#include "communication.h"
#include "../auxilary/estimation.h"
#include "../auxilary/commander.h"

#if ALT_DEFINE
#define NAN 0.0F
#define isfinite( N ) true
static void* memset( void* dest, int ch, size_t count );
static void* memcpy( void* dest, const void* src, size_t count );
static int rand();
#else
#include <string.h>
#include <math.h>
#endif

//communication and network
#define NETWORK_DEVICE_TIMEOUT 8000
#define MAX_ENTITY_COUNT          6
#define LIST_BROADCAST_COUNT      6
#define SEND_QUEUE_COUNT          3
#define RECEIVE_FUNC_OFFSET      20
#define RECEIVE_FUNC_COUNT       30

#pragma pack( push, 1 )
union comContent {
  STDBYTE buffer[MAXBYTE];
  STDBYTE actuation;
  unsigned char entitylistoffset;
  struct {
    unsigned char length;
    unsigned char deviceID[LIST_BROADCAST_COUNT];
    unsigned char nodeOrder[LIST_BROADCAST_COUNT];
    unsigned int timeLastSeen[LIST_BROADCAST_COUNT];
  } entitylist;
  struct {
    struct {
      STDBYTE method;
      unsigned char length;
      unsigned char attempts;
      packet_header header;
    } h;
    STDBYTE msgstart;
  } sendmsg;
  struct {
    coordinate position;
    STDBYTE status;
  } dronestate;
  struct {
    struct {
      unsigned char flightMode;
      unsigned char commandLength;
    } h;
    float value[ FPARLEN( kafenv.cmd.setpoints ) ];
  } flightmode;
  drone_state::droneinfo droneinfo;
  drone_state::stateestimate stateestimate;
  struct {
    unsigned char length;
    float value[ FPARLEN( kafenv.cmd.setpoints ) ];
  } setpoint;
  float motorvalue[ FPARLEN( kafenv.cmd.motors ) ];
  drone_state::calibrations calibration;
  drone_state drone;
  struct {
    struct {
      void* pointer;
      unsigned short length;
    } h;
    STDBYTE msgstart;
  } memtransfer;
};
#pragma pack( pop )

struct fwd {
  STDBYTE originID;
  STDBYTE targetID;
};

static struct {//NETWORK DATA
  unsigned long currentTime;
  STDBYTE nextMessageID;
  struct {
    unsigned char remainingAttempts;
    STDBYTE deliveryMethod;
    unsigned short packetSize;
    void* packetBuffer;
    STDBYTE recvFromID;
    STDBYTE recvMessageID;
    char fwdStatus;
    void( *handlingFunction )( const void*, const unsigned short, const packet_header );
  } sendPackets[SEND_QUEUE_COUNT];
  unsigned short( *replyHandlers[ RECEIVE_FUNC_COUNT ] )( void**, const void*, const unsigned short );
  unsigned short( *validatedReplyHandlers[ RECEIVE_FUNC_COUNT ] )( void**, const void*, const unsigned short, const packet_header );
  STDBYTE validatedSuccessTypes[ RECEIVE_FUNC_COUNT ];
  void( *processingHandlers[ RECEIVE_FUNC_COUNT * 2 ] )( const void*, const packet_header );
  struct {
    char headerBuffer[ sizeof( packet_header ) + sizeof( fwd ) ];
    comContent content;
  } tx, sx[SEND_QUEUE_COUNT];
  unsigned char contentSizes[MAXBYTE];
  unsigned char broadcastType;
  unsigned char entitySendOffset;
  unsigned char entityCount;                     //Number of peer drones observed by this drone
  unsigned char entityLookup[MAXBYTE];           //Drone ID to devices struct array index lookup
  unsigned char entityIndices[MAX_ENTITY_COUNT]; //
  entity entities[MAX_ENTITY_COUNT];             //Structure for each peer that the drone can observe
} coms;

static unsigned char queueMessage( unsigned char attempts ) {
  for( unsigned char i = 0; i < SEND_QUEUE_COUNT; i++ ) {
    if( coms.sendPackets[i].remainingAttempts == 0 ) {
      coms.sendPackets[i].remainingAttempts = attempts;
      return i;
    }
  }
  return SEND_QUEUE_COUNT;
}

static void broadcastDeviceList() {
  unsigned long disconnectTime = coms.currentTime - NETWORK_DEVICE_TIMEOUT;
  for( unsigned char index = coms.entitySendOffset; coms.entityCount > index; ) {
    unsigned char listIndex = coms.entityIndices[index];
    if( coms.entities[listIndex].lastSeen < disconnectTime ) {
      coms.entityIndices[index] = coms.entityIndices[ --coms.entityCount ];
      coms.entityIndices[ coms.entityCount ] = listIndex;
      coms.entityLookup[ coms.entities[listIndex].entityID ] = MAX_ENTITY_COUNT;
    } else {
      unsigned char i = index - coms.entitySendOffset;
      coms.tx.content.entitylist.deviceID[i] = coms.entities[listIndex].entityID;
      coms.tx.content.entitylist.nodeOrder[i] = coms.entities[listIndex].nodeOrder;
      coms.tx.content.entitylist.timeLastSeen[i] = coms.entities[listIndex].lastSeen;
      index++;
    }
  }
  coms.entitySendOffset += LIST_BROADCAST_COUNT;
  coms.entitySendOffset = coms.entityCount == 0 ? 0 : coms.entitySendOffset % coms.entityCount;
}

static packet_header* prepareSendBuffer( void* buffer, const packet_header rxheader, const STDBYTE messageType ) {
  packet_header* txheader = ( ( packet_header* )buffer ) - 1;
  txheader->toID = rxheader.fromID;
  txheader->fromID = kafenv.info.deviceID;
  txheader->messageType = messageType;
  txheader->messageID = rxheader.messageID;
  return txheader;
}

entity* com_registerEntity( const STDBYTE entityID ) {
  unsigned char listIndex = coms.entityLookup[entityID];
  if( listIndex >= MAX_ENTITY_COUNT || coms.entities[listIndex].entityID != entityID ) {
    if( coms.entityCount < MAX_ENTITY_COUNT ) {
      listIndex = coms.entityIndices[coms.entityCount++];
    } else {
      unsigned long disconnectTime = coms.currentTime - NETWORK_DEVICE_TIMEOUT;
      for( listIndex = 0; listIndex < MAX_ENTITY_COUNT - 1; listIndex++ ) {
        if( coms.entities[listIndex].lastSeen < disconnectTime ) {
          break;
        }
      }
    }
    coms.entityLookup[ coms.entities[listIndex].entityID ] = MAX_ENTITY_COUNT;
    coms.entityLookup[entityID] = listIndex;
    coms.entities[listIndex].entityID = entityID;
    DPRINTF( "[C] Registering Entity ID='%c', Index=%u\n", entityID, listIndex );
  }
  return &coms.entities[listIndex];
}

entity* com_getEntityById( const STDBYTE entityID ) {
  unsigned char listIndex = coms.entityLookup[entityID];
  if( listIndex >= MAX_ENTITY_COUNT || coms.entities[listIndex].entityID != entityID ||
      ( coms.currentTime - coms.entities[listIndex].lastSeen ) > NETWORK_DEVICE_TIMEOUT ) {
    return NULLPTR;
  }
  return &coms.entities[listIndex];
}

unsigned char com_getEntity( entity* entity, unsigned char index ) {
  unsigned long disconnectTime = coms.currentTime - NETWORK_DEVICE_TIMEOUT;
  for( unsigned char i = index; i < coms.entityCount; i++ ) {
    *entity = coms.entities[ coms.entityIndices[i] ];
    if( entity->lastSeen > disconnectTime ) {
      return i + 1;
    }
  }
  return MAXBYTE;
}

void* com_sendMessage( const STDBYTE method, const unsigned char attempts, packet_header header, const void* content, 
    const unsigned short size, void( *handler )( const void*, const unsigned short, const packet_header ) ) {
  unsigned char idx;
  const bool fitsSend = size <= sizeof( coms.tx.content );
  if( ( ( content != NULLPTR && size >= sizeof( packet_header ) ) || fitsSend ) && ( idx = queueMessage( attempts ) ) < SEND_QUEUE_COUNT ) {
    void* contentBuffer = fitsSend ? &coms.sx[idx].content : ( void* )( ( packet_header* )content + 1 );
    coms.sendPackets[idx].packetBuffer = contentBuffer;
    coms.sendPackets[idx].packetSize = size;
    coms.sendPackets[idx].recvFromID = header.toID;
    coms.sendPackets[idx].recvMessageID = header.messageType;
    coms.sendPackets[idx].deliveryMethod = method;
    coms.sendPackets[idx].fwdStatus = ( header.messageType & COM_FWD ) != 0 ? 1 : 0;
    coms.sendPackets[idx].handlingFunction = handler;
    *( ( packet_header* )contentBuffer - 1 ) = header;
    return contentBuffer;
  }
  return NULLPTR;
}

void com_receiveMessage( const STDBYTE type, const unsigned char minContentSize,
      unsigned short( *reply )( void**, const void*, const unsigned short ), void( *process )( const void*, const packet_header ) ) {
  unsigned char i = ( type & ~COM_CMD ) - RECEIVE_FUNC_OFFSET;
  if( i < RECEIVE_FUNC_COUNT ) {
    DPRINTF( "[C] Registering Receive Type=%02x, Size=%u, Reply=%08x, Process=%08x\n", type, minContentSize, reply, process );
    coms.contentSizes[type] = minContentSize + sizeof( packet_header );
    coms.contentSizes[ COM_FWD | type ] = coms.contentSizes[type] + sizeof( fwd );
    if( ( type & COM_CMD ) == 0 ) {
      coms.processingHandlers[ i + RECEIVE_FUNC_COUNT ] = process;
    } else {
      coms.replyHandlers[i] = reply;
      coms.processingHandlers[i] = process;
    }
  }
}

void com_receiveValidatedMessage( const STDBYTE type, const unsigned char minContentSize,
      unsigned short( *reply )( void**, const void*, const unsigned short, const packet_header ),
      void( *process )( const void*, const packet_header ), const STDBYTE successReplyType ) {
  unsigned char i = ( type & ~COM_CMD ) - RECEIVE_FUNC_OFFSET;
  if( i < RECEIVE_FUNC_COUNT ) {
    DPRINTF( "[C] Registering Validated Receive Type=%02x, Size=%u, Reply=%08x, Process=%08x\n", type, minContentSize, reply, process );
    coms.contentSizes[type] = minContentSize + sizeof( packet_header );
    coms.contentSizes[ COM_FWD | type ] = coms.contentSizes[type] + sizeof( fwd );
    coms.validatedReplyHandlers[i] = reply;
    coms.validatedSuccessTypes[i] = successReplyType;
    coms.processingHandlers[i] = process;
  }
}

//Sender IDs are not cryptographic authentication, but a fail-closed allowlist prevents an accidental or
//unknown AP client from invoking legacy mutation commands. The dedicated KILL command intentionally remains
//universally reachable as the emergency/reset path; its policy is kept explicit here rather than emerging
//from an omitted check. Header-aware validators below further narrow each Pi command's payload and state.
static bool commandSenderAuthorized( const STDBYTE messageType, const STDBYTE sender ) {
  switch( messageType ) {
    case COM_SET_ACTUATION : case COM_SET_FLIGHTMODE : case COM_SET_TRAJECTORY :
      return sender == GROUND_STATION_ID || sender == PI_CONTROLLER_ID;
    case COM_SET_TRAJSETPT : case COM_SET_GPSORIGIN :
      return sender == PI_CONTROLLER_ID;
    case COM_SET_SENDMSG : case COM_SET_INFO : case COM_SET_STEST : case COM_SET_SETPT :
    case COM_SET_MOTORS : case COM_SET_CALIB : case COM_SET_KAFENV : case COM_SET_MEMORY :
    case COM_SET_INVOKEFUNC : case COM_SET_STARTUP : case COM_SET_STORAGE : case COM_SET_TRAJCONFIG :
    case COM_SET_PIDTUNING : case COM_SET_SIM_VARS : case COM_SET_RESP_VARS : case COM_SET_WIFI :
      return sender == GROUND_STATION_ID;
    case COM_SET_KILL :
      return true;
    default :
      return true;
  }
}

static unsigned short fixedMutationSize( const STDBYTE messageType ) {
  switch( messageType ) {
    case COM_SET_INFO : return sizeof( coms.tx.content.droneinfo );
    case COM_SET_STEST : return sizeof( coms.tx.content.stateestimate );
    case COM_SET_MOTORS : return sizeof( coms.tx.content.motorvalue );
    case COM_SET_CALIB : return sizeof( coms.tx.content.calibration );
    case COM_SET_KAFENV : return sizeof( coms.tx.content.drone );
    default : return 0;
  }
}

static void* replyMessage( const comContent* rx, STDBYTE* messageType, unsigned short* messageSize,
    const packet_header rxheader ) {
  if( !commandSenderAuthorized( *messageType, rxheader.fromID ) ) {
    DPRINTF( "[C] Rejected Unauthorized Command: Type=%02x, Sender='%c'\n", *messageType, rxheader.fromID );
    *messageType = COM_FAILURE;
    *messageSize = 0;
    return &coms.tx.content;
  }
  switch( *messageType ) {
    case COM_REQUEST_PING : {
      *messageType = COM_REPLY_PING;
      *messageSize = 0;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_PING, Reply=REPLY_PING\n" );
      break;
    }
    case COM_SET_ACTUATION : {
      const bool requestedActuation = rx->actuation == MAXBYTE;
      const bool validValue = rx->actuation == 0 || requestedActuation;
      if( *messageSize != sizeof( rx->actuation ) || !validValue
          || ( requestedActuation && !commander_canArm( rxheader.fromID ) ) ) {
        *messageType = COM_FAILURE;
        DPRINTF( "[C] Handling Reply: Received=SET_ACTUATION, Reply=FAILURE (value/sender/arming guard)\n" );
      } else {
        kafenv.info.actuation = requestedActuation;
        *messageType = COM_SUCCESS;
        DPRINTF( "[C] Handling Reply: Received=SET_ACTUATION, Reply=SUCCESS\n" );
      }
      *messageSize = 0;
      break;
    }
    case COM_REQUEST_DEVICES : {
      coms.entitySendOffset = rx->entitylistoffset;
      broadcastDeviceList();
      *messageSize = sizeof( coms.tx.content.entitylist );
      *messageType = COM_REPLY_DEVICES;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_DEVICES, Reply=REPLY_DEVICES\n" );
      break;
    }
    case COM_SET_SENDMSG : {
      const unsigned short contentSize = *messageSize - sizeof( rx->sendmsg.h );
      if( rx->sendmsg.h.length != contentSize ) {
        *messageType = COM_FAILURE;
        DPRINTF( "[C] Handling Reply: Received=SET_SENDMSG, Reply=FAILURE\n" );
      } else {
        *messageType = COM_SUCCESS;
        DPRINTF( "[C] Handling Reply: Received=SET_SENDMSG, Reply=SUCCESS\n" );
      }
      *messageSize = 0;
      break;
    }
    case COM_REQUEST_STATE : {
      coms.tx.content.dronestate.position = kafenv.state.x;
      coms.tx.content.dronestate.status = kafenv.info.flightMode;
      *messageSize = sizeof( coms.tx.content.dronestate );
      *messageType = COM_REPLY_STATE;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_STATE, Reply=REPLY_STATE\n" );
      break;
    }
    case COM_REQUEST_INFO : {
      coms.tx.content.droneinfo = kafenv.info;
      *messageSize = sizeof( coms.tx.content.droneinfo );
      *messageType = COM_REPLY_INFO;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_INFO, Reply=REPLY_INFO\n" );
      break;
    }
    case COM_REQUEST_STEST : {
      coms.tx.content.stateestimate = kafenv.state;
      *messageSize = sizeof( coms.tx.content.stateestimate );
      *messageType = COM_REPLY_STEST;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_STEST, Reply=REPLY_STEST\n" );
      break;
    }
    case COM_REQUEST_SETPT : {
      if( rx->setpoint.length <= FPARLEN( kafenv.cmd.setpoints ) ) {
        unsigned short size = ( (unsigned short)rx->setpoint.length ) * sizeof( float );
        coms.tx.content.setpoint.length = rx->setpoint.length;
        memcpy( coms.tx.content.setpoint.value, kafenv.cmd.setpoints, size );
        *messageSize = sizeof( coms.tx.content.setpoint.length ) + size;
        *messageType = COM_REPLY_SETPT;
        DPRINTF( "[C] Handling Reply: Received=REQUEST_SETPT, Reply=SUCCESS\n" );
      } else {
        *messageSize = 0;
        *messageType = COM_FAILURE;
        DPRINTF( "[C] Handling Reply: Received=REQUEST_SETPT, Reply=FAILURE\n" );
      }
      break;
    }
    case COM_SET_FLIGHTMODE :  {
      const unsigned short valuesSize = ( (unsigned short)rx->flightmode.h.commandLength ) * sizeof( float );
      if( rx->flightmode.h.commandLength <= FPARLEN( kafenv.cmd.setpoints ) &&
          sizeof( rx->flightmode.h ) + valuesSize == *messageSize &&
          commander_validateFlightModeCommand( rx->flightmode.h.flightMode, rx->flightmode.h.commandLength,
              rx->flightmode.value, rxheader.fromID ) ) {
        *messageType = COM_SUCCESS;
        DPRINTF( "[C] Handling Reply: Received=SET_FLIGHTMODE, Reply=SUCCESS\n" );
      } else {
        *messageType = COM_FAILURE;
        DPRINTF( "[C] Handling Reply: Received=SET_FLIGHTMODE, Reply=FAILURE\n" );
      }
      *messageSize = 0;
      break;
    }
    case COM_SET_SETPT : {
      const unsigned short valuesSize = ( (unsigned short)rx->setpoint.length ) * sizeof( float );
      bool valuesFinite = true;
      for( unsigned char i = 0; i < rx->setpoint.length && i < FPARLEN( kafenv.cmd.setpoints ); i++ ) {
        valuesFinite = valuesFinite && isfinite( rx->setpoint.value[i] );
      }
      if( rx->setpoint.length <= FPARLEN( kafenv.cmd.setpoints ) &&
          sizeof( rx->setpoint.length ) + valuesSize == *messageSize && valuesFinite
          && rxheader.fromID == GROUND_STATION_ID ) {
        *messageType = COM_SUCCESS;
        DPRINTF( "[C] Handling Reply: Received=SET_SETPT, Reply=SUCCESS\n" );
      } else {
        *messageType = COM_FAILURE;
        DPRINTF( "[C] Handling Reply: Received=SET_SETPT, Reply=FAILURE\n" );
      }
      *messageSize = 0;
      break;
    }
    case COM_REQUEST_MOTORS : {
      memcpy( coms.tx.content.motorvalue, kafenv.cmd.motors, sizeof( kafenv.cmd.motors ) );
      *messageSize = sizeof( coms.tx.content.motorvalue );
      *messageType = COM_REPLY_MOTORS;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_MOTORS, Reply=REPLY_MOTORS\n" );
      break;
    }
    case COM_REQUEST_CALIB : {
      coms.tx.content.calibration = kafenv.cal;
      *messageSize = sizeof( coms.tx.content.calibration );
      *messageType = COM_REPLY_CALIB;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_CALIB, Reply=REPLY_CALIB\n" );
      break;
    }
    case COM_REQUEST_KAFENV : {
      coms.tx.content.drone = kafenv;
      *messageSize = sizeof( coms.tx.content.drone );
      *messageType = COM_REPLY_KAFENV;
      DPRINTF( "[C] Handling Reply: Received=REQUEST_KAFENV, Reply=REPLY_KAFENV\n" );
      break;
    }
    case COM_REQUEST_MEMORY : {
      if( rx->memtransfer.h.length > sizeof( coms.tx.content ) - sizeof( coms.tx.content.memtransfer.h ) ) {
        *messageSize = 0;
        *messageType = COM_FAILURE;
        DPRINTF( "[C] Handling Reply: Received=REQUEST_MEMORY, Reply=FAILURE\n" );
      } else {
        memcpy( &coms.tx.content.memtransfer.msgstart, rx->memtransfer.h.pointer, rx->memtransfer.h.length );
        *messageSize = sizeof( coms.tx.content.memtransfer.h ) + rx->memtransfer.h.length;
        *messageType = COM_REPLY_MEMORY;
        DPRINTF( "[C] Handling Reply: Received=REQUEST_MEMORY, Reply=REPLY_MEMORY\n" );
      }
      break;
    }
    case COM_SET_MEMORY : {
      const unsigned short contentSize = *messageSize - sizeof( rx->memtransfer.h );
      if( rx->memtransfer.h.length != contentSize ) {
        *messageType = COM_FAILURE;
        DPRINTF( "[C] Handling Reply: Received=SET_MEMORY, Reply=FAILURE\n" );
      } else {
        *messageType = COM_SUCCESS;
        DPRINTF( "[C] Handling Reply: Received=SET_MEMORY, Reply=SUCCESS\n" );
      }
      *messageSize = 0;
      break;
    }
    case COM_SET_INFO : case COM_SET_STEST : case COM_SET_MOTORS : case COM_SET_CALIB : case COM_SET_KAFENV : {
      const bool validLength = *messageSize == fixedMutationSize( *messageType );
      *messageSize = 0;
      *messageType = validLength ? COM_SUCCESS : COM_FAILURE;
      DPRINTF( "[C] Handling Reply: Received=SET_COMMAND, Reply=%s\n", validLength ? "SUCCESS" : "FAILURE" );
      break;
    }
    default : {
      const STDBYTE typeUnmasked = *messageType & COM_ID;
      if( typeUnmasked >= RECEIVE_FUNC_OFFSET && typeUnmasked - RECEIVE_FUNC_OFFSET < RECEIVE_FUNC_COUNT ) {
        const unsigned char handlerIndex = typeUnmasked - RECEIVE_FUNC_OFFSET;
        unsigned short( *reply )( void**, const void*, const unsigned short ) = coms.replyHandlers[handlerIndex];
        unsigned short( *validatedReply )( void**, const void*, const unsigned short, const packet_header ) =
            coms.validatedReplyHandlers[handlerIndex];
        if( reply != NULLPTR || validatedReply != NULLPTR ) {
          DPRINTF( "[C] Invoking Custom Reply Command: Type=%02x, Unmasked=%02x\n", *messageType, typeUnmasked );
          //Custom payloads start at content. headerBuffer reserves enough space immediately before it for
          //normal and forwarded headers; using &coms.tx here wrote payloads into that reserve and made
          //prepareSendBuffer() step outside the object.
          void* buffer = &coms.tx.content;
          unsigned short sendLength = validatedReply != NULLPTR ?
              validatedReply( &buffer, rx, *messageSize, rxheader ) : reply( &buffer, rx, *messageSize );
          if( buffer != NULLPTR ) {
            *messageType = validatedReply != NULLPTR ? coms.validatedSuccessTypes[handlerIndex] : typeUnmasked;
            *messageSize = sendLength;
            DPRINTF( "[C] Handling Reply: Received=CUSTOM_COMMAND, Reply=CUSTOM_REPLY\n" );
            return buffer;
          }
        }
      } 
      *messageType = COM_FAILURE;
      *messageSize = 0;
      DPRINTF( "[C] Handling Reply: Received=CUSTOM_COMMAND, Reply=FAILURE\n" );
    }
  }
  return &coms.tx.content;
}

peripheral com_reset() {
  DPRINTF( "[C] Resetting Communication Protocol Handler\n" );
  memset( &coms, 0, sizeof( coms ) );
  coms.currentTime = NETWORK_DEVICE_TIMEOUT;
  coms.nextMessageID = ( unsigned int )( rand() % 0x100 );
  for( unsigned char i = 0; i < SEND_QUEUE_COUNT; i++ ) {
    coms.sendPackets[i].remainingAttempts = 0;
    coms.sendPackets[i].deliveryMethod = 0;
    coms.sendPackets[i].packetSize = 0;
    coms.sendPackets[i].packetBuffer = &coms.sx[i].content;
    coms.sendPackets[i].recvFromID = 0;
    coms.sendPackets[i].recvMessageID = 0;
    coms.sendPackets[i].fwdStatus = 0;
    coms.sendPackets[i].handlingFunction = NULLPTR;
  }
  for( unsigned char i = 0; i < RECEIVE_FUNC_COUNT; i++ ) {
    coms.replyHandlers[i] = NULLPTR;
    coms.validatedReplyHandlers[i] = NULLPTR;
    coms.validatedSuccessTypes[i] = COM_SUCCESS;
    coms.processingHandlers[i] = NULLPTR;
    coms.processingHandlers[ i + RECEIVE_FUNC_COUNT ] = NULLPTR;
  }
  memset( coms.contentSizes, sizeof( packet_header ), sizeof( coms.contentSizes ) );
  coms.contentSizes[COM_SET_ACTUATION]   += sizeof( coms.tx.content.actuation );
  coms.contentSizes[COM_REQUEST_DEVICES] += sizeof( coms.tx.content.entitylistoffset );
  coms.contentSizes[COM_REPLY_DEVICES]   += sizeof( coms.tx.content.entitylist );
  coms.contentSizes[COM_SET_SENDMSG]     += sizeof( coms.tx.content.sendmsg.h );
  coms.contentSizes[COM_REPLY_STATE]     += sizeof( coms.tx.content.dronestate );
  coms.contentSizes[COM_SET_FLIGHTMODE]  += sizeof( coms.tx.content.flightmode.h );
  coms.contentSizes[COM_REPLY_INFO]      += sizeof( coms.tx.content.droneinfo );
  coms.contentSizes[COM_SET_INFO]        += sizeof( coms.tx.content.droneinfo );
  coms.contentSizes[COM_REPLY_STEST]     += sizeof( coms.tx.content.stateestimate );
  coms.contentSizes[COM_SET_STEST]       += sizeof( coms.tx.content.stateestimate );
  coms.contentSizes[COM_REPLY_SETPT]     += sizeof( coms.tx.content.setpoint.length );
  coms.contentSizes[COM_SET_SETPT]       += sizeof( coms.tx.content.setpoint.length );
  coms.contentSizes[COM_REPLY_MOTORS]    += sizeof( coms.tx.content.motorvalue );
  coms.contentSizes[COM_SET_MOTORS]      += sizeof( coms.tx.content.motorvalue );
  coms.contentSizes[COM_REPLY_CALIB]     += sizeof( coms.tx.content.calibration );
  coms.contentSizes[COM_SET_CALIB]       += sizeof( coms.tx.content.calibration );
  coms.contentSizes[COM_REPLY_KAFENV]    += sizeof( coms.tx.content.drone );
  coms.contentSizes[COM_SET_KAFENV]      += sizeof( coms.tx.content.drone );
  coms.contentSizes[COM_REQUEST_MEMORY]  += sizeof( coms.tx.content.memtransfer.h );
  coms.contentSizes[COM_REPLY_MEMORY]    += sizeof( coms.tx.content.memtransfer.h );
  coms.contentSizes[COM_SET_MEMORY]      += sizeof( coms.tx.content.memtransfer.h );
  for( unsigned char i = 0; i < ( ~COM_FWD ); i++ ) {
    coms.contentSizes[ i | COM_FWD ] = coms.contentSizes[i] + sizeof( fwd );
  }
  coms.broadcastType = 0;
  coms.entitySendOffset = 0;
  coms.entityCount = 0;
  memset( coms.entityLookup, MAX_ENTITY_COUNT, sizeof( coms.entityLookup ) );
  for( unsigned char i = 0; i < MAX_ENTITY_COUNT; i++ ) {
    coms.entityIndices[i] = i;
    coms.entities[i].entityID = 0;
    coms.entities[i].flightMode = 0;
    coms.entities[i].liason = 0;
    coms.entities[i].nodeOrder = 0;
    coms.entities[i].lastSeen = 0;
    coms.entities[i].position = { NAN, NAN, NAN };
    coms.entities[i].distance = NAN;
  }
  return { "comms", false, sizeof( coms ), &coms, [](){ com_reset(); }, NULLPTR };
}

void com_step( const radio* radio ) {
  coms.currentTime = coms.currentTime < radio->currentTime ? radio->currentTime : coms.currentTime;
  const unsigned short receiveSize = radio->receiving(); // receive packet
  packet_header* packetptr = ( packet_header* )radio->packet;
  //DEBUG logging throttled to ~1/s - com_step() runs once per comms-loop iteration (~100+/s), so this
  //was by far the largest unthrottled source of Serial traffic once the other per-loop prints were fixed
  static unsigned long lastInvokedPrint = 0;
  if( millis() - lastInvokedPrint > 1000 ) {
    lastInvokedPrint = millis();
    DPRINTF( "[C] Invoked Step: Method=%02x, Time=%lu\n", radio->method, coms.currentTime );
  }
  if( receiveSize >= sizeof( packet_header ) && packetptr->fromID != kafenv.info.deviceID ) { // check for valid packet
    packet_header rxheader = *packetptr;
    const STDBYTE fromID = rxheader.fromID;
    const unsigned short contentSize = receiveSize - sizeof( packet_header );
    const comContent* content = ( comContent* )( packetptr + 1 );
    DPRINTF( "[C] Received Packet: Total Size=%u, To='%c', From='%c', Type=%02x, ID=%02x\n", 
        receiveSize, rxheader.toID, rxheader.fromID, rxheader.messageType, rxheader.messageID );
    if( kafenv.info.deviceID == rxheader.toID ) { // check for recepient
      if( receiveSize < coms.contentSizes[ rxheader.messageType ] ) { // check for minimum valid packet size
        DPRINTF( "[C] Invalid Packet\n" );
        radio->replying( prepareSendBuffer( &coms.tx.content, rxheader, COM_FAILURE ), sizeof( packet_header ) );
        rxheader.messageType = COM_INVALID;
      } else if( ( rxheader.messageType & COM_FWD ) == COM_FWD ) { // check for forwarding message
        fwd rxFwd = *( fwd* )( packetptr + 1 );
        if( rxFwd.originID != kafenv.info.deviceID ) { // if bad packet discard
          DPRINTF( "[C] Invalid Forwarding Packet\n" );
          radio->replying( prepareSendBuffer( &coms.tx.content, rxheader, COM_FAILURE ), sizeof( packet_header ) );
          rxheader.messageType = COM_INVALID;
        } else { // forwarding protocol on good packets
          DPRINTF( "[C] Forwarding Packet: Origin='%c', Target='%c'\n", rxFwd.originID, rxFwd.targetID );
          const bool isRecipient = rxFwd.targetID == kafenv.info.deviceID;
          const bool requiresReply = isRecipient && ( rxheader.messageID & COM_CMD ) != 0;
          if( requiresReply && !radio->fwdReply ) {
            DPRINTF( "[C] Forwarding Acknowledging Sender\n" );
            radio->replying( prepareSendBuffer( &coms.tx.content, rxheader, COM_ACKNOWLEDGED ), sizeof( packet_header ) );
          }
          unsigned short sendSize;
          STDBYTE targetID;
          bool doForwarding = false;
          if( requiresReply ) { // check for forwarding recepient
            STDBYTE messageType = rxheader.messageType ^ COM_FWD;
            sendSize = contentSize - sizeof( fwd );
            packet_header logicalHeader = rxheader;
            logicalHeader.fromID = rxFwd.originID;
            logicalHeader.toID = rxFwd.targetID;
            void* tx = replyMessage( ( comContent* )( ( fwd* )content + 1 ), &messageType, &sendSize, logicalHeader );
            sendSize += sizeof( packet_header ) + sizeof( fwd );
            fwd* txFwd = ( fwd* )tx - 1;
            txFwd->originID = kafenv.info.deviceID;
            txFwd->targetID = rxFwd.originID;
            targetID = rxFwd.originID;
            packetptr = prepareSendBuffer( txFwd, rxheader, messageType | COM_FWD );
            if( !radio->fwdReply ) {
              radio->replying( packetptr, sendSize );
            } else {
              doForwarding = true;
            }
            rxheader.messageType = messageType == COM_FAILURE ? COM_INVALID : rxheader.messageType;
          } else if( !isRecipient ) {
            sendSize = receiveSize;
            targetID = rxFwd.targetID;
            doForwarding = true;
          }
          if( doForwarding ) { // initiate sending of reply or forwarding packet
            STDBYTE forwardID = targetID;
            for( unsigned char i = 0; i < 4; i++ ) { // lookup for valid forwarding entity
              entity* forwarding = com_getEntityById( forwardID );
              if( forwarding == NULLPTR || forwarding->entityID == kafenv.info.deviceID ||
                  forwarding->entityID == rxFwd.originID || forwarding->entityID == fromID ) {
                break;
              } else if( forwarding->nodeOrder == 0 ) { // detection of valid forwarding entity
                packetptr->toID = forwarding->entityID;
                packetptr->fromID = kafenv.info.deviceID;
                unsigned char index;
                if( forwarding->liason == radio->method ) { //send if same send method
                  radio->sending( packetptr, sendSize );
                } else if( sendSize <= sizeof( coms.tx ) && ( index = queueMessage( 11 ) ) != SEND_QUEUE_COUNT ) { // queue if diff method
                  memcpy( &coms.sx[index], packetptr, sendSize );
                  coms.sendPackets[index].deliveryMethod = forwarding->liason;
                  coms.sendPackets[index].packetBuffer = &coms.sx[index];
                  coms.sendPackets[index].packetSize = ( unsigned char )sendSize;
                  coms.sendPackets[index].fwdStatus = 0;
                  coms.sendPackets[index].handlingFunction = NULLPTR;
                }
                break;
              } else { // nesting of forwarding
                forwardID = forwarding->liason;
              }
            }
          }
          entity* entity = com_registerEntity( rxFwd.originID ); // forwarding entity logging
          if( rxFwd.originID == fromID ) {
            entity->nodeOrder = 0;
            entity->liason = radio->method;
          } else {
            entity->nodeOrder = entity->nodeOrder < MAXBYTE ? entity->nodeOrder : MAXBYTE;
            entity->liason = fromID;
          }
          entity->lastSeen = coms.currentTime + NETWORK_DEVICE_TIMEOUT / 2;
          rxheader.fromID = rxFwd.originID; // set rxheader for processMessage handling
          rxheader.toID = rxFwd.targetID;
        }
      } else if( ( rxheader.messageType & COM_CMD ) != 0 ) { // non-forwarding normal message handling for commands
        DPRINTF( "[C] Command Packet\n" );
        STDBYTE messageType = rxheader.messageType;
        unsigned short sendSize = contentSize;
        void* tx = replyMessage( content, &messageType, &sendSize, rxheader );
        radio->replying( prepareSendBuffer( tx, rxheader, messageType ), sendSize + sizeof( packet_header ) );
        //Only successful commands are processed. COM_FAILURE is a validation failure, not a request to
        //mutate state; COM_INVALID prevents the command switch below from running.
        rxheader.messageType = messageType == COM_FAILURE ? COM_INVALID : rxheader.messageType;
      }
      for( int i = 0; i < SEND_QUEUE_COUNT; i++ ) { // for all messaged addressed as recepient, check for queue listeners
        void( *func )( const void*, const unsigned short, const packet_header ) = coms.sendPackets[i].handlingFunction;
        if( coms.sendPackets[i].remainingAttempts > 0 && func != NULLPTR &&
            coms.sendPackets[i].recvFromID == rxheader.fromID && coms.sendPackets[i].recvMessageID == rxheader.messageID ) {
          if( coms.sendPackets[i].fwdStatus == 1 && rxheader.messageID == COM_ACKNOWLEDGED ) {
            coms.sendPackets[i].fwdStatus = -1;
            coms.sendPackets[i].remainingAttempts = 5;
          } else {
            func( content, contentSize, rxheader );
            coms.sendPackets[i].remainingAttempts = 0;
            coms.sendPackets[i].handlingFunction = NULLPTR;
          }
        }
      }
    }
    entity* from = com_registerEntity( fromID ); // received message entity logging
    from->nodeOrder = 0;
    from->liason = radio->method;
    from->lastSeen = coms.currentTime;
    // process message and log, record information about sender
    switch( rxheader.messageType ) {
      case COM_REPLY_DEVICES : {
        unsigned char len = sizeof( content->entitylist.deviceID );
        len = len > content->entitylist.length ? content->entitylist.length : len;
        for( unsigned char i = 0; i < len; i++ ) {
          if( content->entitylist.deviceID[i] != kafenv.info.deviceID && content->entitylist.timeLastSeen[i] < NETWORK_DEVICE_TIMEOUT ) {
            entity* entity = com_getEntityById( content->entitylist.deviceID[i] );
            const unsigned char currOrder = content->entitylist.nodeOrder[i] + 1;
            bool noEntityData = entity == NULLPTR;
            entity = noEntityData ? com_registerEntity( content->entitylist.deviceID[i] ) : entity;
            if( noEntityData || entity->nodeOrder > currOrder ) {
              entity->liason = from->entityID;
              entity->nodeOrder = currOrder;
              entity->lastSeen = coms.currentTime - content->entitylist.timeLastSeen[i];
            }
          }
        }
        break;
      }
      case COM_REPLY_STATE : {
        from->position = content->dronestate.position;
        from->flightMode = content->dronestate.status;
        break;
      }
      case COM_REPLY_STEST : {
        from->position = content->stateestimate.x;
        break;
      }
      case COM_REPLY_KAFENV : {
        from->position = content->drone.state.x;
        break;
      }
      default : { }
    }
    //respond to commands from sender
    if( rxheader.toID == kafenv.info.deviceID ) {
      switch( rxheader.messageType ) {
        case COM_SET_SENDMSG : {
          const STDBYTE retMethod = from->liason;
          void* ret = com_sendMessage( content->sendmsg.h.method, content->sendmsg.h.attempts, 
              content->sendmsg.h.header, NULLPTR, content->sendmsg.h.length, NULLPTR );
          if( ret != NULLPTR ) {
            memcpy( ret, &content->sendmsg.msgstart, content->sendmsg.h.length );
          }
          break;
        }
        case COM_SET_FLIGHTMODE : {
          kafenv.info.triggerLock = 1;
          FLTSYNC;
          kafenv.info.flightMode = content->flightmode.h.flightMode;
          memcpy( kafenv.cmd.setpoints, content->flightmode.value, content->flightmode.h.commandLength * sizeof( float ) );
          commander_acceptFlightModeCommand( content->flightmode.h.flightMode, content->flightmode.h.commandLength,
              rxheader.fromID );
          kafenv.info.triggerLock = 0;
          break;
        }
        case COM_SET_INFO : {
          kafenv.info = content->droneinfo;
          break;
        }
        case COM_SET_STEST : {
          kafenv.state = content->stateestimate;
          break;
        }
        case COM_SET_SETPT : {
          kafenv.info.triggerLock = 1;
          FLTSYNC;
          memcpy( kafenv.cmd.setpoints, content->setpoint.value, content->setpoint.length * sizeof( float ) );
          commander_acceptLegacySetpoint( rxheader.fromID, content->setpoint.length );
          kafenv.info.triggerLock = 0;
          break;
        }
        case COM_SET_MOTORS : {
          memcpy( kafenv.cmd.motors, content->motorvalue, sizeof( kafenv.cmd.motors ) );
          break;
        }
        case COM_SET_CALIB : {
          kafenv.info.triggerLock = 1;
          FLTSYNC;
          kafenv.cal = content->calibration;
          kafenv.info.triggerLock = 0;
          break;
        }
        case COM_SET_KAFENV : {
          kafenv = content->drone;
          break;
        }
        case COM_SET_MEMORY : {
          memcpy( content->memtransfer.h.pointer, &content->memtransfer.msgstart, content->memtransfer.h.length );
          break;
        }
        default : {
          unsigned char receiveIdx = rxheader.messageType & COM_ID;
          if( receiveIdx >= RECEIVE_FUNC_OFFSET && receiveIdx - RECEIVE_FUNC_OFFSET < RECEIVE_FUNC_COUNT ) {
            receiveIdx = receiveIdx - RECEIVE_FUNC_OFFSET;
            receiveIdx = ( rxheader.messageType | COM_CMD ) == 0 ? receiveIdx + RECEIVE_FUNC_COUNT : receiveIdx;
            void( *process )( const void*, const packet_header ) = coms.processingHandlers[ receiveIdx ];
            if( process != NULLPTR ) {
              process( content, rxheader );
            }
          }
        }
      }
    }
  }
  if( radio->allowBroadcast ) { //broadcast queued send messages
    switch( coms.broadcastType = ( coms.broadcastType + 1 ) % 6 ) {
      case 0 : {
        coms.tx.content.dronestate.position = kafenv.state.x;
        coms.tx.content.dronestate.status = kafenv.info.flightMode;
        unsigned short len = sizeof( coms.tx.content.dronestate ) + sizeof( packet_header );
        radio->sending( prepareSendBuffer( &coms.tx, { 0, 0, 0, 0 }, COM_REPLY_STATE ), len );
      }
      case 2 : case 4 : {
        broadcastDeviceList();
        unsigned short len = sizeof( coms.tx.content.entitylist ) + sizeof( packet_header );
        radio->sending( prepareSendBuffer( &coms.tx, { 0, 0, 0, 0 }, COM_REPLY_DEVICES ), len );
      }
      default : { }
    }
  }
  for( int i = 0; i < SEND_QUEUE_COUNT; i++ ) { // attempt sending messages if in queue and matches send method
    unsigned char attempts = coms.sendPackets[i].remainingAttempts;
    if( attempts > 0 ) {
      if( coms.sendPackets[i].fwdStatus != -1 && coms.sendPackets[i].deliveryMethod == radio->method ) {
        radio->sending( coms.sendPackets[i].packetBuffer, coms.sendPackets[i].packetSize );
        attempts = coms.sendPackets[i].handlingFunction == NULLPTR ? 0 : ( attempts <= 3 ? 0 : attempts - 3 );
      } else {
        attempts--;
      }
      if( attempts == 0 ) {
        void( *func )( const void*, const unsigned short, const packet_header ) = coms.sendPackets[i].handlingFunction;
        if( func != NULLPTR ) {
          packet_header header;
          func( NULLPTR, 0, header );
        }
      }
      coms.sendPackets[i].remainingAttempts = attempts;
    }
  }
}
