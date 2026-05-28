#include "kaf_drone.h"
#include "communication.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <string.h>
#endif

void communicationRespond() {
  bool isFwdMsg = ( kafenv.c.rx.data.messageType & COM_FWD ) != 0;
  unsigned char originID = 0;
  if( isFwdMsg ) {
    fwd* rxFwd;
    if( kafenv.n.immediateForwardingReturn && ( rxFwd = KF_FWD( rx, kafenv.n.receiveSize ) )->targetID == kafenv.u.deviceID ) {
      originID = rxFwd->originID;
      kafenv.c.rx.data.messageType = kafenv.c.rx.data.messageType ^ COM_FWD;
    } else {
      kafenv.c.tx.data.messageType = COM_ACKNOWLEDGED;
      kafenv.n.replySize = 0;
      return;
    }
  }
  switch( kafenv.c.rx.data.messageType ) {
    case COM_PING : {
      kafenv.c.tx.data.messageType = COM_PONG;
      kafenv.n.replySize = 0;
      break;
    }
    case COM_TRIGGER : {
      kafenv.p.trigger = kafenv.c.rx.data.byte;
      kafenv.c.tx.data.messageType = COM_ACKNOWLEDGED;
      kafenv.n.replySize = 0;
      break;
    }
    case COM_GET_DEVICES : {
      int offset = kafenv.c.rx.data.byte;
      kafenv.c.tx.data.messageType = COM_RET_DEVICES;
      memset( kafenv.c.tx.data.deviceList.deviceID, PLACEHOLDER_ID, sizeof( kafenv.c.tx.data.deviceList.deviceID ) );
      for( int i = 0; i + offset < kafenv.n.deviceCount && i < sizeof( kafenv.c.tx.data.deviceList.deviceID ); i++ ) {
        unsigned char index = kaf_getnextvalid( i + offset );
        if( index == INVALID_DEVICE_IDX ) {
          break;
        }
        kafenv.c.tx.data.deviceList.deviceID[i] = kafenv.n.devices[index].deviceID;
        kafenv.c.tx.data.deviceList.nodeOrder[i] = kafenv.n.devices[index].nodeOrder;
        kafenv.c.tx.data.deviceList.timeLastSeen[i] = ( unsigned int )( kafenv.n.currentTime - kafenv.n.devices[index].lastSeen );
      }
      kafenv.n.replySize = sizeof( kafenv.c.tx.data.deviceList );
      break;
    }
    case COM_GET_STATE : {
      kafenv.c.tx.data.messageType = COM_RET_STATE;
      kafenv.c.tx.data.coord = kafenv.u.stateEstimate.x;
      kafenv.n.replySize = sizeof( kafenv.c.tx.data.coord );
      break;
    }
    case COM_REQUEST_WIFI : {
      kafenv.c.tx.data.messageType = COM_REPLY_WIFI;
      memcpy( kafenv.c.tx.data.bytes, kafenv.n.networkAddress, strlen( kafenv.n.networkAddress ) );
      kafenv.n.replySize = strlen( kafenv.n.networkAddress );
      break;
    }
    case COM_REQUEST_POS : {
      kafenv.c.tx.data.messageType = COM_REPLY_POS;
      kafenv.c.tx.data.coord = kafenv.u.stateEstimate.x;
      kafenv.n.replySize = sizeof( kafenv.c.tx.data.coord );
      break;
    }
    case COM_REQUEST_ATT : {
      kafenv.c.tx.data.messageType = COM_REPLY_ATT;
      kafenv.c.tx.data.coord = kafenv.u.stateEstimate.t;
      kafenv.n.replySize = sizeof( kafenv.c.tx.data.coord );
      break;
    }
    case COM_REQUEST_GLOBAL : {
      unsigned int startPos = kafenv.c.rx.data.globalVarTransfer.startPos;
      int length = kafenv.c.rx.data.globalVarTransfer.length;
      if( startPos >= 0 && startPos + length < sizeof( kafenv ) && length > 0 && length <= sizeof( kafenv.c.tx.data.bytes ) ) {
        kafenv.c.tx.data.messageType = COM_REQUEST_GLOBAL;
        memcpy( kafenv.c.tx.data.bytes, ( (char*)(void*)( &kafenv ) ) + startPos, length );
        kafenv.n.replySize = length;
      } else {
        kafenv.c.tx.data.messageType = COM_FAILURE;
        kafenv.n.replySize = 0;
      }
      break;
    }
    case COM_REQUEST_ST_EST : {
      kafenv.c.tx.data.messageType = COM_REPLY_ST_EST;
      kafenv.c.tx.data.stateEst = kafenv.u.stateEstimate;
      kafenv.n.replySize = sizeof( kafenv.c.tx.data.stateEst );
      break;
    }
    case COM_SET_GLOBAL : {
      unsigned int startPos = kafenv.c.rx.data.globalVarTransfer.startPos;
      int length = kafenv.c.rx.data.globalVarTransfer.length;
      if( startPos >= 0 && startPos + length < sizeof( kafenv ) && length > 0 && length <= sizeof( kafenv.c.tx.data.globalVarTransfer.bytes ) ) {
        kafenv.c.tx.data.messageType = COM_SUCCESS;
      } else {
        kafenv.c.tx.data.messageType = COM_FAILURE;
      }
      kafenv.n.replySize = 0;
      break;
    }
    case COM_SET_CTRL_MODE : {
      if( kafenv.c.rx.data.byte < FLIGHT_MODE_BOUNDS ) {
        kafenv.c.tx.data.messageType = COM_SUCCESS;
      } else{
        kafenv.c.tx.data.messageType = COM_FAILURE;
      }
      kafenv.n.replySize = 0;
      break;
    }
    case COM_SET_FWDMSG : {
      if( kafenv.c.rx.data.setFwdSend.length < COM_HEADER_LEN ||
          kafenv.c.rx.data.setFwdSend.length > sizeof( kafenv.c.rx.data.setFwdSend.message ) ) {
        kafenv.c.tx.data.messageType = COM_FAILURE;
      } else {
        kafenv.c.tx.data.messageType = COM_ACKNOWLEDGED;
      }
      kafenv.n.replySize = 0;
      break;
    }
    case COM_SET_WIFI : case COM_SET_POS : case COM_SET_ATT : case COM_SET_ST_EST : case COM_SET_MOTOR_CMD : case COM_SET_POS_CMD : {
      kafenv.c.tx.data.messageType = COM_SUCCESS;
      kafenv.n.replySize = 0;
      break;
    }
    default : {
      if( ( kafenv.c.rx.data.messageType & COM_CMD ) != 0 && ( kafenv.c.rx.data.messageType & COM_ID ) >= 0x10 ) {
        kafenv.c.tx.data.messageType = COM_FAILURE;
        kafenv.n.replySize = 0;
      }
    }
    if( isFwdMsg ) {//2 reply directly to sender
      if( kafenv.n.replySize < sizeof( kafenv.c.tx.data.bytes ) ) {
        kafenv.c.tx.data.messageType = kafenv.c.tx.data.messageType | COM_FWD;
        fwd* txFwd = KF_FWD( tx, kafenv.n.replySize + sizeof( fwd ) );
        txFwd->originID = kafenv.u.deviceID;
        txFwd->targetID = originID;
        kafenv.n.replySize += sizeof( fwd );
      } else {
        kafenv.c.tx.data.messageType = COM_ACKNOWLEDGED;
        kafenv.n.replySize = 0;
      }
    }
  }
}

void communicationRecord() {
  unsigned char index = kaf_getdevice( kafenv.c.rx.data.fromID );
  kafenv.n.devices[index].nodeOrder = 0;
  kafenv.n.devices[index].lastSeen = kafenv.n.currentTime;
  //forwarded message processing
  if( ( kafenv.c.rx.data.messageType & COM_FWD ) != 0 ) {
    unsigned char sendIdx, packet;
    fwd* rxFwd = KF_FWD( rx, kafenv.n.receiveSize );
    if( rxFwd->targetID == kafenv.u.deviceID ) {
      kafenv.c.rx.data.messageType = kafenv.c.rx.data.messageType ^ COM_FWD;
      communicationRespond();
      if( kafenv.n.replySize < sizeof( kafenv.c.tx.data.bytes ) ) {
        sendIdx = kaf_getforwarding( rxFwd->originID );
        sendIdx = sendIdx == INVALID_DEVICE_IDX ? index : sendIdx;
        if( kafenv.n.devices[sendIdx].liason != INVALID_NETWORK_METHOD ) {
          packet = kaf_addcoms( 10, kafenv.n.devices[sendIdx].liason, kafenv.n.replySize + sizeof( fwd ), NULL );
          memcpy( kafenv.c.sx[packet].raw, kafenv.c.rx.raw, kafenv.n.replySize + COM_HEADER_LEN );
          kafenv.c.sx[packet].data.toID = kafenv.n.devices[sendIdx].deviceID;
          fwd* sxFwd = KF_FWD( sx[packet], kafenv.n.replySize + sizeof( fwd ) );
          sxFwd->originID = kafenv.u.deviceID;
          sxFwd->targetID = rxFwd->originID;
        }
      }
    } else if( rxFwd->originID != kafenv.u.deviceID && ( sendIdx = kaf_getforwarding( rxFwd->targetID ) ) != INVALID_DEVICE_IDX && 
          kafenv.n.devices[sendIdx].deviceID != kafenv.c.rx.data.fromID && kafenv.n.devices[sendIdx].deviceID != rxFwd->originID ) {
      packet = kaf_addcoms( 10, kafenv.n.devices[sendIdx].liason, kafenv.n.receiveSize, NULL );
      memcpy( kafenv.c.sx[packet].raw, kafenv.c.rx.raw, kafenv.n.receiveSize + COM_HEADER_LEN );
      kafenv.c.sx[packet].data.fromID = kafenv.u.deviceID;
      kafenv.c.sx[packet].data.toID = kafenv.n.devices[sendIdx].deviceID;
    }
  }
  //record information about sender
  switch( kafenv.c.rx.data.messageType ) {
    case COM_RET_DEVICES : {
      for( int i = 0; i < sizeof( kafenv.c.rx.data.deviceList.deviceID ); i++ ) {
        unsigned char currDevID = kafenv.c.rx.data.deviceList.deviceID[i];
        if( currDevID == PLACEHOLDER_ID ) {
          break;
        } else if( currDevID != kafenv.u.deviceID && kafenv.c.rx.data.deviceList.timeLastSeen[i] < NETWORK_DEVICE_TIMEOUT ) {
          unsigned char currOrder = kafenv.c.rx.data.deviceList.nodeOrder[i] + 1;
          unsigned char currIdx = kaf_hasdevice( currDevID );
          if( currIdx == INVALID_DEVICE_IDX || kafenv.n.devices[currIdx].nodeOrder > currOrder ) {
            currIdx =  currIdx == INVALID_DEVICE_IDX ? kaf_getdevice( currDevID ) : currIdx;
            kafenv.n.devices[currIdx].liason = kafenv.c.rx.data.fromID;
            kafenv.n.devices[currIdx].nodeOrder = currOrder;
            kafenv.n.devices[currIdx].lastSeen = kafenv.n.currentTime - kafenv.c.rx.data.deviceList.timeLastSeen[i];
          }
        }
      }
      break;
    }
    case COM_RET_STATE : {
      kafenv.n.devices[index].position = kafenv.c.rx.data.coord;
      break;
    }
    case COM_REPLY_ST_EST : {
      kafenv.n.devices[index].position = kafenv.c.rx.data.stateEst.x;
      break;
    }
    case COM_REPLY_POS : {
      kafenv.n.devices[index].position = kafenv.c.rx.data.coord;
      break;
    }
    default : { }
  }
  //respond to commands from sender
  if( kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
    switch( kafenv.c.rx.data.messageType ) {
      case COM_SET_GLOBAL : {
        if( kafenv.c.tx.data.messageType == COM_SUCCESS ) {
          unsigned int startPos = kafenv.c.rx.data.globalVarTransfer.startPos;
          int length = kafenv.c.rx.data.globalVarTransfer.length;
          memcpy( ( (char*)(void*)( &kafenv ) ) + startPos, kafenv.c.rx.data.globalVarTransfer.bytes, length );
        }
        break;
      }
      case COM_SET_WIFI : {
        memcpy( kafenv.n.networkName, kafenv.c.rx.data.setWiFi.networkName, 20 );
        memcpy( kafenv.n.networkPassword, kafenv.c.rx.data.setWiFi.networkPassword, 20 );
        break;
      }
      case COM_SET_POS : {
        kafenv.u.stateEstimate.x = kafenv.c.rx.data.coord;
        break;
      }
      case COM_SET_ATT : {
        kafenv.u.stateEstimate.t = kafenv.c.rx.data.coord;
        break;
      }
      case COM_SET_ST_EST : {
        kafenv.u.stateEstimate = kafenv.c.rx.data.stateEst;
        break;
      }
      case COM_SET_CTRL_MODE : {
        if( kafenv.c.tx.data.messageType == COM_SUCCESS ) {
          kafenv.u.flightMode = kafenv.c.rx.data.byte;
        }
        break;
      }
      case COM_SET_MOTOR_CMD : {
        kafenv.u.flightMode = MOTOR_SETPOINT_MODE;
        KF_VECN( i, 4, kafenv.s.motorSetpoint[i] = kafenv.c.rx.data.coord.f[i] );
        break;
      }
      case COM_SET_POS_CMD : {
        kafenv.u.flightMode = POS_SETPOINT_MODE;
        KF_VEC3( i, kafenv.s.posSetpoint[i] = kafenv.c.rx.data.coord.f[i] );
        break;
      }
      case COM_SET_FWDMSG : { 
        if( kafenv.c.tx.data.messageType == COM_ACKNOWLEDGED ) {
          unsigned char packet = kaf_addcoms( 1, kafenv.c.rx.data.setFwdSend.method, kafenv.c.rx.data.setFwdSend.length, NULL );
          memcpy( kafenv.c.sx[packet].raw, kafenv.c.rx.data.setFwdSend.message, kafenv.c.rx.data.setFwdSend.length );
        }
        break;
      }
      default : { }
    }
  }
}

void communicationStep() {
  void( *comTaskQueue )() = kafenv.p.comTaskQueue;
  if( comTaskQueue != NULL ) {
    kafenv.p.comTaskQueue = NULL;
    comTaskQueue();
  }
  kafenv.n.comStepSkips++;
  if( kafenv.n.comStepSkips < 10 ) {
    return;
  }
  kafenv.n.comStepSkips = 0;




}