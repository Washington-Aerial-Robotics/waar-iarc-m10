#include "../core/firmware.h"
#include "../auxilary/common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include "../lib/ESP32Servo/ESP32Servo.h"
#endif

#define BOUND( V, LB, UB )    V = V > UB ? UB : ( V < LB ? LB : V )
#define ESC_MAX               2000
#define ESC_MIN               1000
#define ESC_RAMP                20
#define ESC_ARMEDRAMP           10

static struct {
  bool motorEnabled = false;
  unsigned int pins[ FPARLEN( kafenv.cmd.motors ) ] = { ESC_PINS };
  unsigned short setpoints[ FPARLEN( kafenv.cmd.motors ) ];
  Servo servos[ FPARLEN( kafenv.cmd.motors ) ];
} escs;

void peripheral_escsLoop() {
  if( kafenv.info.actuation ) {
    if( escs.motorEnabled ) {
      DPRINTF( "[P] Run ESCs: Status=ACTIVE\n" );
      for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
        unsigned short value = (unsigned short)( ( ESC_MAX - ESC_MIN ) * kafenv.cmd.motors[i] + ESC_MIN ) - escs.setpoints[i];
        BOUND( value, -ESC_RAMP, ESC_RAMP );
        value += escs.setpoints[i];
        BOUND( value, ESC_MIN, ESC_MAX );
        if( value != escs.setpoints[i] ) {
          escs.setpoints[i] = value;
          escs.servos[i].writeMicroseconds( value );
        }
      }
    } else {
      DPRINTF( "[P] Run ESCs: Status=RAMP_UP\n" );
      escs.motorEnabled = true;
      for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
        if( escs.setpoints[i] < ESC_MIN ) {
          escs.setpoints[i] += ESC_ARMEDRAMP;
          escs.servos[i].writeMicroseconds( escs.setpoints[i] );
          escs.motorEnabled = false;
        }
      }
    }
    DPRINTF( "[P] ESC Setpoint: Motors=[ %d, %d, %d, %d ]\n", escs.setpoints[0], escs.setpoints[1], escs.setpoints[2], escs.setpoints[3] );
  } else {
    DPRINTF( "[P] Run ESCs: Status=DISABLED\n" );
    escs.motorEnabled = false;
    for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
      escs.setpoints[i] = 0;
      escs.servos[i].writeMicroseconds( 0 );
    }
  }
}

void peripheral_escsInit() {
  firmware_registerPeripheral( { "escs", 0, sizeof( escs ), &escs, &peripheral_escsInit, &peripheral_escsLoop } );
  DPRINTF( "[P] Initializing ESCs\n" );
  escs.motorEnabled = false;
  //arm all the pins and set default values
  for( int i = 0; i < FPARLEN( kafenv.cmd.motors ); i++ ) {
    pinMode( escs.pins[i], OUTPUT );
    escs.servos[i].attach( escs.pins[i] );
    escs.servos[i].writeMicroseconds( 0 );
    escs.setpoints[i] = 0;
  }
}