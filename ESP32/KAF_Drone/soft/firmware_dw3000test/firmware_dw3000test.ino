/*
  DW3000 UWB Bench Test - Minimal Device ID Read
  --------------------------
  Bypasses the whole dwt_checkidlerc()/dwt_initialise() state-machine path
  (which has been failing with DW_NOT_IDLE) and instead does the most
  minimal possible test: reset the chip, select it over SPI, and directly
  ask it to identify itself via dwt_check_dev_id(). This mirrors Makerfabs'
  own official ex_00a_reading_dev_id example for this exact board.

  dwt_check_dev_id() internally prints the raw DEVICE ID register value via
  printf("DEVICE ID: %x\r\n", ...) regardless of whether it matches a known
  DW3000 chip ID, so even a "failure" here tells us what the chip (or bus
  noise) is actually returning.

  Uses this project's real pin definitions (DW_IRQ/DW_RST/DW_SS from
  common_data.h, verified against Makerfabs' own dw3000_board_config.h:
  IRQ=34, RST=27, SS=4) rather than the vendor library's own defaults, to
  exercise the exact wiring this board actually uses.

  SAFETY: no motors/ESCs touched by this sketch at all.
*/

#include "kaf_quadcopter_code.h"
#include "auxilary/common_data.h"
#include "lib/Dw3000/dw3000.h"

extern void peripheral_serialStart( unsigned int baudrate );
extern void peripheral_commonInit();
extern void peripheral_escsInit();
extern void peripheral_mpu9250Init();

void setup() {
  peripheral_serialStart( 115200 );
  Serial.println( "DW3000 bench test starting." );
  Serial.printf( "Using pins: IRQ=%d, RST=%d, SS=%d\n", DW_IRQ, DW_RST, DW_SS );

  //DEBUG: replicating firmware_full.ino's exact setup() sequence up to the point dw3000Init runs,
  //to bisect whether one of these earlier peripherals is what breaks dwt_checkidlerc() there but not here.
  firmware_reset();
  //peripheral_commonInit();
  //peripheral_escsInit();
  //peripheral_mpu9250Init();
  Serial.println( "Bisecting: only firmware_reset() this round - now testing DW3000..." );

  spiBegin( DW_IRQ, DW_RST );
  spiSelect( DW_SS );
  delay( 2 ); // per Makerfabs' own example: time for DW3000 to transition INIT_RC -> IDLE_RC

  if( dwt_check_dev_id() == DWT_SUCCESS ) {
    Serial.println( "DEV ID OK - chip is responding correctly." );
  } else {
    Serial.println( "DEV ID FAILED - see raw DEVICE ID value printed above." );
  }

  Serial.println( "Now checking dwt_checkidlerc() in this same clean environment..." );
  for( unsigned char i = 0; i < 5; i++ ) {
    bool idle = dwt_checkidlerc();
    Serial.printf( "  Attempt %u: dwt_checkidlerc() = %s\n", i, idle ? "TRUE (idle)" : "false (not idle)" );
    delay( 50 );
  }
}

void loop() {
  delay( 2000 );
  Serial.println( "Re-checking device ID..." );
  if( dwt_check_dev_id() == DWT_SUCCESS ) {
    Serial.println( "DEV ID OK." );
  } else {
    Serial.println( "DEV ID FAILED." );
  }
}
