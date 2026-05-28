#include "kaf_drone.h"
#include "communication.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <math.h>
#include <stdio.h>
#endif

struct global_variables kafenv;

void kaf_dumpmemory() {
  //KF_REGVAR( 'c', "kafenv.u.deviceID", kafenv.u.deviceID );
}

void kaf_matmul( float* mat, float* in, float* out, unsigned char size ) {
  for( unsigned char y = 0; y < size; y++ ) {
    out[y] = 0;
    for( unsigned char x = 0; x < size; x++ ) {
      out[y] += mat[ x + y * size ] * in[x];
    }
  }
}

void kaf_filterreset( measfilter* filter ) {
  filter->stateX1 = 0;
  filter->stateX2 = 0;
  filter->measCount = 0;
  filter->measTotal = 0;
  filter->measDev = 0;
}

void kaf_filtercalib( measfilter* filter, float measured ) {
  filter->measCount++;
  filter->measTotal += measured;
  filter->offset = filter->measTotal / filter->measCount;
  filter->measDev += (  measured - filter->offset ) * ( measured - filter->offset );
  filter->uncertainity = sqrt( filter->measDev / filter->measCount );
}

float kaf_filterstep( measfilter* filter, float measured, float dt ) {
  float dX2 = ( filter->frequency * filter->frequency * filter->stateX1 ) * dt;
  float corrected = filter->gain *( measured + filter->offset );
  filter->stateX1 += ( -1.414213562373095 * filter->frequency * filter->stateX1 - filter->stateX2 + corrected ) * dt;
  filter->stateX2 += dX2;
  return filter->stateX2;
}

void kaf_pidreset( pid* pid ) {
  pid->integ = 0;
  pid->deriv = 0; 
  pid->error = 0; 
  pid->desired = 0; 
  pid->prevmeas = NAN;
  float r = tan( ( (float)M_PI ) / pid->derlimit );
  pid->lowpass[0] = r * r / ( 1 + sqrt( 2 ) * r + r * r );
  pid->lowpass[1] = pid->lowpass[0] * ( 2 - 2 / ( r * r ) );
  pid->lowpass[2] = pid->lowpass[0] * ( 1 / ( r * r ) - sqrt( 2 ) / r + 1  );
  pid->lowpass[3] = 0;
  pid->lowpass[4] = 0;
}

float kaf_pidstep( pid* pid, float desired, float measured, float dt ) {
  pid->desired = desired;
  pid->error = fmodf( pid->desired - measured, pid->modula );
  if( pid->error > pid->modula / 2 ) {
    pid->error = pid->error - pid->modula;
  } else if( pid->error < -pid->modula / 2 ) {
    pid->error = pid->error + pid->modula;
  }
  pid->deriv = ( pid->prevmeas - measured ) / dt;
  pid->prevmeas = measured;
  if( isfinite( pid->deriv ) ) {
    pid->deriv = 0;
  } else {
    float datalp0 = pid->deriv - pid->lowpass[1] * pid->lowpass[3] - pid->lowpass[2] * pid->lowpass[4];
    pid->deriv = pid->lowpass[0] * ( datalp0 + 2 * pid->lowpass[3] + pid->lowpass[4] );
    pid->lowpass[4] = pid->lowpass[3];
    pid->lowpass[3] = datalp0;
  }
  pid->integ = pid->integ + pid->error * dt;
  KF_BOUND( pid->integ, -pid->intlimit, pid->intlimit );
  float output = pid->Kp * pid->error + pid->Ki * pid->integ + pid->Kd * pid->deriv + pid->Kf * pid->desired;
  KF_BOUND( output, -pid->outlimit, pid->outlimit );
  return output;
}

unsigned char kaf_getnextvalid( unsigned char indicesIndex ) {
  unsigned long disconnectTime = kafenv.n.currentTime - NETWORK_DEVICE_TIMEOUT;
  for( ; kafenv.n.deviceCount >= indicesIndex; ) {  
    unsigned char index =  kafenv.n.deviceIndices[indicesIndex];
    if( kafenv.n.devices[index].lastSeen < disconnectTime ) {
      kafenv.n.deviceIndices[indicesIndex] = kafenv.n.deviceIndices[ --kafenv.n.deviceCount ];
      kafenv.n.deviceIndices[ kafenv.n.deviceCount ] = index;
      kafenv.n.deviceLookup[ kafenv.n.devices[index].deviceID ] = INVALID_DEVICE_IDX;
    } else {
      return index;
    }
  }
  return INVALID_DEVICE_IDX;
}

unsigned char kaf_getdevice( unsigned char deviceID ) {
  unsigned char index = kafenv.n.deviceLookup[deviceID];
  if( index == INVALID_DEVICE_IDX ) {
    if( kafenv.n.deviceCount < DEVICE_COUNT ) {
      index = kafenv.n.deviceIndices[kafenv.n.deviceCount++];
    } else {
      unsigned long disconnectTime = kafenv.n.currentTime - NETWORK_DEVICE_TIMEOUT;
      for( index = 0; index < DEVICE_COUNT - 1; index++ ) {
        if( kafenv.n.devices[index].lastSeen < disconnectTime ) {
          break;
        }
      }
    }
    kafenv.n.deviceLookup[ kafenv.n.devices[index].deviceID ] = INVALID_DEVICE_IDX;
    kafenv.n.deviceLookup[deviceID] = index;
    kafenv.n.devices[index].deviceID = deviceID;
  }
  kafenv.n.devices[index].liason = kafenv.n.receiveMethod;
  return index;
}

unsigned char kaf_hasdevice( unsigned char deviceID ) {
  unsigned char index = kafenv.n.deviceLookup[deviceID];
  if( index == INVALID_DEVICE_IDX || ( kafenv.n.currentTime - kafenv.n.devices[index].lastSeen ) > NETWORK_DEVICE_TIMEOUT ) {
    return INVALID_DEVICE_IDX;
  }
  return index;
}

unsigned char kaf_getforwarding( unsigned char deviceID ) {
  unsigned char forwardID = deviceID;
  for( unsigned char i = 0; i < 4; i++ ) {
    unsigned char toIdx = kaf_hasdevice( forwardID );
    if( toIdx == INVALID_DEVICE_IDX ) {
      return INVALID_DEVICE_IDX;
    } else if( kafenv.n.devices[toIdx].nodeOrder == 0 ) {
      return toIdx;
    } else {
      forwardID = kafenv.n.devices[toIdx].liason;
    }
  }
  return INVALID_DEVICE_IDX;
}

unsigned char kaf_addcoms( unsigned char attempts, unsigned char method, unsigned char size, void( *handler )( bool ) ) {
  for( unsigned char i = 0; i < SEND_QUEUE_COUNT; i++ ) {
    if( kafenv.n.sendPackets[i].remainingAttempts == 0 ) {
      kafenv.n.sendPackets[i].remainingAttempts = attempts;
      kafenv.n.sendPackets[i].deliveryMethod = method;
      kafenv.n.sendPackets[i].packetSize = size;
      kafenv.n.sendPackets[i].handlingFunction = handler;
      return i;
    }
  }
  return SEND_QUEUE_COUNT - 1;
}

void kaf_handlecoms( bool doSend, bool doReply, unsigned char method, unsigned char len, void( *sendFunc )( unsigned char*, unsigned char ) ) {
  bool isRecepient = false;
  if( doReply ) {
    kafenv.n.receiveMethod = method;
    kafenv.n.receiveSize = len - COM_HEADER_LEN;
    kafenv.n.replySize = sizeof( kafenv.c.tx.data.bytes ) + 1;
    if( isRecepient = kafenv.c.rx.data.toID == kafenv.u.deviceID ) {
      communicationRespond();
      if( kafenv.n.replySize < sizeof( kafenv.c.tx.data.all ) ) {
        kafenv.c.tx.data.fromID = kafenv.u.deviceID;
        kafenv.c.tx.data.toID = kafenv.c.rx.data.fromID;
        kafenv.c.tx.data.messageID = kafenv.n.nextMessageID;
        sendFunc( kafenv.c.tx.raw, kafenv.n.replySize );
        kafenv.n.nextMessageID = ( unsigned char )( rand() % 0x100 );
      }
    }
    communicationRecord();
  }
  for( int i = 0; i < SEND_QUEUE_COUNT; i++ ) {
    unsigned char attempts = kafenv.n.sendPackets[i].remainingAttempts;
    if( attempts > 0 ) {
      bool replySuccess;
      if( isRecepient && kafenv.c.sx[i].data.messageID == kafenv.c.rx.data.messageID ) {
        if( ( kafenv.c.sx[i].data.messageType & COM_FWD ) == 0 || kafenv.n.sendPackets[i].handlingFunction == NULL ) {
          replySuccess = kafenv.c.sx[i].data.fromID == kafenv.c.rx.data.toID && kafenv.c.sx[i].data.toID == kafenv.c.rx.data.fromID;
        } else {
          fwd* sf = KF_FWD( sx[i], kafenv.n.sendPackets[i].packetSize );
          fwd* rf = KF_FWD( rx, kafenv.n.receiveSize );
          replySuccess = ( kafenv.c.rx.data.messageType & COM_FWD ) != 0 && sf->originID == rf->targetID && sf->targetID == rf->originID;
        }
      }
      if( replySuccess ) {
        attempts = 0;
      } else if( kafenv.n.sendPackets[i].deliveryMethod == method ) {
        if( doSend ) {
          sendFunc( kafenv.c.sx[i].raw, kafenv.n.sendPackets[i].packetSize );
          attempts = attempts <= 2 ? 0 : attempts - 2;
        } else {
          attempts--;
        }
      }
      if( attempts == 0 ) {
        kafenv.n.sendPackets[i].deliveryMethod = 0;
        kafenv.n.sendPackets[i].packetSize = 0;
        void( *handlingFunction )( bool ) = kafenv.n.sendPackets[i].handlingFunction;
        if( handlingFunction != NULL ) {
          kafenv.n.sendPackets[i].handlingFunction = NULL;
          handlingFunction( replySuccess );
        }
      }
      kafenv.n.sendPackets[i].remainingAttempts = attempts;
    }
  }
}