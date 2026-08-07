#include "../core/firmware.h"
#include "../auxilary/common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include <DShotRMT.h>
#endif

#define BOUND( V, LB, UB )    V = V > UB ? UB : ( V < LB ? LB : V )
#define ESC_DSHOT_MODE      DSHOT300
#define ESC_BIDIRECTIONAL      false
#define ESC_RAMP                0.1F    // max throttle percent change per loop iteration
#define ESC_ARM_FRAMES           200    // zero-throttle frames before ESCs are considered armed (~2s @ 100Hz flight loop, see FLIGHT_TASK_PERIOD_MS in periph_freertos.cpp)

static struct {
  bool motorEnabled = false;
  unsigned int armFrames = 0;
  unsigned int pins[ FPARLEN( kafenv.cmd.motors ) ] = { ESC_PINS };
  float setpoints[ FPARLEN( kafenv.cmd.motors ) ];
  DShotRMT* motors[ FPARLEN( kafenv.cmd.motors ) ];
} escs;

void peripheral_escsLoop() {
  if( kafenv.info.actuation ) {
    if( escs.motorEnabled ) {
      DPRINTF( "[P] Run ESCs: Status=ACTIVE\n" );
      for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
        float target = kafenv.cmd.motors[i] * 100.0F;
        BOUND( target, 0.0F, 100.0F );
        float value = target - escs.setpoints[i];
        BOUND( value, -ESC_RAMP, ESC_RAMP );
        value += escs.setpoints[i];
        BOUND( value, 0.0F, 100.0F );
        escs.setpoints[i] = value;
        escs.motors[i]->sendThrottlePercent( value );
      }
    } else {
      DPRINTF( "[P] Run ESCs: Status=ARMING\n" );
      for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
        escs.motors[i]->sendThrottlePercent( 0.0F );
      }
      if( ++escs.armFrames >= ESC_ARM_FRAMES ) {
        escs.motorEnabled = true;
      }
    }
    DPRINTF( "[P] ESC Setpoint: Motors=[ %.1f, %.1f, %.1f, %.1f ]\n", escs.setpoints[0], escs.setpoints[1], escs.setpoints[2], escs.setpoints[3] );
  } else {
    DPRINTF( "[P] Run ESCs: Status=DISABLED\n" );
    escs.motorEnabled = false;
    escs.armFrames = 0;
    for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
      escs.setpoints[i] = 0.0F;
      escs.motors[i]->sendThrottlePercent( 0.0F );
    }
  }
}

void peripheral_escsInit() {
  firmware_registerPeripheral( { "escs", 0, sizeof( escs ), &escs, &peripheral_escsInit, &peripheral_escsLoop } );
  DPRINTF( "[P] Initializing ESCs\n" );
  escs.motorEnabled = false;
  escs.armFrames = 0;
  for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
    escs.motors[i] = new DShotRMT( ( uint16_t )escs.pins[i], ESC_DSHOT_MODE, ESC_BIDIRECTIONAL );
    escs.motors[i]->begin();
    escs.motors[i]->sendThrottlePercent( 0.0F );
    escs.setpoints[i] = 0.0F;
  }
}