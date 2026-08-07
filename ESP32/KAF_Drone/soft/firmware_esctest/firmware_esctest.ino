/*
  ESC DShot300 Firmware Bench Test
  ---------------------------------
  Exercises the real periph_esc.cpp / DShotRMT integration through the
  KAF_Drone library (arming hold, ramp, mixer interface), without the IMU,
  estimator, commander, or WiFi stack. Use this to bench-verify the DShot
  motor output independently of the web UI.

  REQUIRES: "DShotRMT" library by derdoktor667
    Arduino IDE > Sketch > Include Library > Manage Libraries... > search "DShotRMT" > Install

  SAFETY:
  - Remove propellers before running this test.
  - Make sure the battery is NOT connected while you upload code or wire things up.
  - Keep hands and loose objects clear of motors during testing.
  - This sketch calls peripheral_escsLoop() continuously at ~100Hz, the same
    rate the real flight loop runs at (see FLIGHT_TASK_PERIOD_MS in
    periph_freertos.cpp), so the DShot arm-hold and ramp behave exactly as
    they would in flight.
*/

#include "kaf_quadcopter_code.h"

extern void peripheral_commonInit();
extern void peripheral_escsInit();
extern void peripheral_escsLoop();

const float          TEST_THROTTLE   = 0.06F;  // ~6% - keep LOW for a bench test
const unsigned long  SPIN_MS         = 2000;
const unsigned long  GAP_MS          = 1000;
const unsigned long  CYCLE_GAP_MS    = 3000;
const unsigned long  LOOP_PERIOD_MS  = 10;      // matches FLIGHT_TASK_PERIOD_MS

void setup() {
  Serial.begin( 115200 );
  delay( 1000 );
  Serial.println( "ESC DShot300 Firmware Test starting..." );
  Serial.println( "!!! REMOVE PROPELLERS before continuing !!!" );
  Serial.println( ">>> Ensure the LiPo is UNPLUGGED right now. <<<" );

  firmware_reset();
  peripheral_commonInit();
  peripheral_escsInit();

  Serial.println( "Zero-throttle DShot stream is live (disarmed)." );
  Serial.println( "Plug in the LiPo now and listen for the ESC arm tone." );
  Serial.println( "Type 'y' + Enter once armed and ready to continue." );

  unsigned long lastLoop = millis();
  while( true ) {
    unsigned long now = millis();
    if( now - lastLoop >= LOOP_PERIOD_MS ) {
      peripheral_escsLoop();
      lastLoop = now;
    }
    if( Serial.available() ) {
      char c = Serial.read();
      if( c == 'y' || c == 'Y' ) break;
    }
  }

  Serial.println( "Enabling actuation. Motors will arm (~2s zero-throttle hold), then the test sequence begins." );
  kafenv.info.actuation = true;
}

void loop() {
  static unsigned long lastLoop = 0;
  static unsigned char phase = 0;
  static unsigned long phaseStart = 0;
  static bool phaseInit = true;
  unsigned long now = millis();

  if( now - lastLoop >= LOOP_PERIOD_MS ) {
    peripheral_escsLoop();
    lastLoop = now;
  }

  if( phaseInit ) {
    phaseStart = now;
    phaseInit = false;
    for( int i = 0; i < 4; i++ ) kafenv.cmd.motors[i] = 0;
    switch( phase ) {
      case 0: Serial.println( "Testing Motor 1 (GPIO 25)" ); kafenv.cmd.motors[0] = TEST_THROTTLE; break;
      case 2: Serial.println( "Testing Motor 2 (GPIO 26)" ); kafenv.cmd.motors[1] = TEST_THROTTLE; break;
      case 4: Serial.println( "Testing Motor 3 (GPIO 32)" ); kafenv.cmd.motors[2] = TEST_THROTTLE; break;
      case 6: Serial.println( "Testing Motor 4 (GPIO 33)" ); kafenv.cmd.motors[3] = TEST_THROTTLE; break;
      case 8: Serial.println( "Cycle complete. Pausing before repeat..." ); break;
      default: break; // odd phases are gaps - motors already zeroed above
    }
  }

  unsigned long duration;
  switch( phase ) {
    case 0: case 2: case 4: case 6: duration = SPIN_MS; break;
    case 1: case 3: case 5: case 7: duration = GAP_MS; break;
    default: duration = CYCLE_GAP_MS; break;
  }

  if( now - phaseStart >= duration ) {
    phase = ( phase + 1 ) % 9;
    phaseInit = true;
  }
}
