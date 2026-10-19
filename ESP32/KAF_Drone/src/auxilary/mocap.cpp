#include "mocap.h"
#include "../core/communication.h"
#include "../core/firmware.h"

#if ALT_DEFINE
#define NAN 0.0F
#define isfinite( N ) true
#else
#include <math.h>
#include <Arduino.h>
#endif

//COM_SET_MOCAP payload - must match MOCAP_PAYLOAD in tests/vicon/kaf_link.py (little-endian, 20 bytes)
#pragma pack( push, 1 )
struct mocappose {
  unsigned int sequence;       //Sender-side frame counter - older/duplicate packets are dropped
  coordinate position;         //Position in m in the firmware world frame (x right/east, y front/north, z up)
  float yaw;                   //Yaw in rad, kafenv.state.q.z convention, NAN if not sent - stored, not fused
};
#pragma pack( pop )

//Written by the comms task (COM_SET_MOCAP handler), read by the flight task (mocap_step()). Double-buffered:
//the handler fills the slot the flight task isn't reading, then flips `latest`, so a read never sees a
//half-written position. A torn read would need two packets within one coordinate copy (~100 Hz vs ns).
static struct {
  bool active;
  unsigned int lastSequence;
  struct {
    coordinate position;
    float yaw;
    unsigned long receivedMillis;
  } sample[2];
  volatile unsigned char latest;
  unsigned long packetCount;
} mocap;

peripheral mocap_reset() {
  DPRINTF( "[M] Resetting Motion Capture\n" );
  mocap.active = true;
  mocap.lastSequence = 0;
  for( unsigned char i = 0; i < 2; i++ ) {
    mocap.sample[i].position = { 0, 0, 0 };
    mocap.sample[i].yaw = NAN;
    mocap.sample[i].receivedMillis = 0;
  }
  mocap.latest = 0;
  mocap.packetCount = 0;
  com_receiveMessage( COM_SET_MOCAP, sizeof( mocappose ), []( void** response, const void* content, const unsigned short len ) {
    const mocappose* pose = ( const mocappose* )content;
    if( !isfinite( pose->position.x ) || !isfinite( pose->position.y ) || !isfinite( pose->position.z ) ) {
      *response = NULLPTR; //replies COM_FAILURE and skips the processing handler below
    }
    return ( unsigned short )0;
  }, []( const void* content, const packet_header header ) {
    const mocappose* pose = ( const mocappose* )content;
    const unsigned long now = millis();
    const unsigned long lastReceived = mocap.sample[ mocap.latest ].receivedMillis;
    const bool streamRestarted = lastReceived == 0 || ( now - lastReceived ) > MOCAP_RESTART_MS;
    if( pose->sequence <= mocap.lastSequence && !streamRestarted ) {
      return;
    }
    const unsigned char next = mocap.latest ^ 1;
    mocap.sample[next].position = pose->position;
    mocap.sample[next].yaw = pose->yaw;
    mocap.sample[next].receivedMillis = now;
    mocap.latest = next;
    mocap.lastSequence = pose->sequence;
    mocap.packetCount++;
    //DEBUG logging throttled to ~1/s - this handler runs at the Vicon stream rate
    static unsigned long lastMocapPrint = 0;
    if( now - lastMocapPrint > 1000 ) {
      lastMocapPrint = now;
      DPRINTF( "[M] Mocap Position: X=[ %.3f, %.3f, %.3f ], Yaw=%.3f, Seq=%u, Count=%lu\n", pose->position.x,
          pose->position.y, pose->position.z, pose->yaw, pose->sequence, mocap.packetCount );
    }
  } );
  return { "mocap", 0, sizeof( mocap ), &mocap, [](){ mocap_reset(); }, NULLPTR };
}

bool mocap_step( coordinate* estimate ) {
  //NULLPTR is flight.cpp's calibration pass, which counts true returns to scale kafenv.cal.positionalpha -
  //report true like estimation_step() does so the fusion gain calibrates the same as with GPS
  if( estimate == NULLPTR ) {
    return true;
  }
  if( !mocap_positionValid() ) {
    return false;
  }
  *estimate = mocap.sample[ mocap.latest ].position;
  return true;
}

bool mocap_isActive() {
  return mocap.active;
}

bool mocap_positionValid() {
  const unsigned long receivedMillis = mocap.sample[ mocap.latest ].receivedMillis;
  return mocap.active && receivedMillis != 0 && ( millis() - receivedMillis ) <= MOCAP_STALE_MS;
}

float mocap_yaw() {
  return mocap.sample[ mocap.latest ].yaw;
}
