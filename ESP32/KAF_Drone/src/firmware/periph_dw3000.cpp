/*
 * KNOWN ISSUE: DW3000 bring-up (dwt_checkidlerc() never sees IDLE_RC, "DW_NOT_IDLE") consistently fails
 * on the Makerfabs ESP32-UWB-DW3000 board once peripheral_commonInit()/escsInit()/mpu9250Init() have run
 * first, despite the chip verified working in isolation (dwt_check_dev_id() reads the correct 0xdeca0302
 * every time when nothing else has run yet). Ruled out: wrong pins (SPI/CS/RST/IRQ all confirmed against
 * Makerfabs' own dw3000_board_config.h), SPI clock speed (tested 8MHz and 16MHz), and pure cold-boot
 * timing margin (retried up to 5x, 10s apart - see attemptDW3000Bringup()/DW_INIT_* below - all failed).
 * Root cause is a real conflict with one of commonInit/escsInit/mpu9250Init, not yet isolated further.
 * Parked as low priority; the retry logic below means it will self-recover automatically if the
 * underlying conflict is ever fixed, without needing any other code changes.
 */

#include "../core/communication.h"
#include "../core/firmware.h"
#include "../auxilary/common_data.h"
#include "../lib/Dw3000/dw3000.h"
#include <SPI.h>
#include <Arduino.h>

#define DW_COM_METHOD            3
#define DW_QUEUE_LENGTH          5
#define DW_MAX_BUFFER_LENGTH   127
#define DW_TX_ANT_DLY        16385 //TX antenna delay
#define DW_RX_ANT_DLY        16385 //RX antenna delay
#define DW_PRE_TIMEOUT           5
#define DW_RESPONSE_PERIOD    1000
#define DW_SCAN_PERIOD       30000
#define DW_PERIOD_R12          900
#define DW_PERIOD_R23          700
#define DW_RANGE_TOLERANCE     200
#define DW_SPIN_TIMEOUT        300
#define DW_INIT_IDLE_POLL_MS       300 //per-attempt cap on polling for IDLE_RC - kept short so a retry never stalls the comms task for long
#define DW_INIT_RETRY_INTERVAL_MS 10000 //minimum spacing between bring-up retries
#define DW_INIT_MAX_ATTEMPTS          5 //total attempts (including the initial one during setup()) before giving up

static struct {
  bool working = false;
  unsigned char frameLength = 0;
  unsigned char broadcastIndex = 0;
  unsigned char attemptCount = 0;
  unsigned long lastAttemptTime = 0;
  radio coms;
  dwt_config_t config;
  unsigned char queueLength;
  unsigned char queueSizes[DW_QUEUE_LENGTH];
#pragma pack( push, 1 )
  union {
    unsigned char raw[DW_MAX_BUFFER_LENGTH];
    struct {
      packet_header header;
      union {
        struct {
          coordinate position;
        } ranging1;
        struct {
          unsigned int processingTime;
          coordinate position;
        } ranging2;
        struct {
          unsigned int processingTime;
          unsigned int timeOfFlight;
        } ranging3;
      };
    };
  } rx, tx, range[DW_QUEUE_LENGTH];
#pragma pack( pop )
} dw;

static bool sendMessageDW( unsigned char* buffer, unsigned char msgLen, unsigned char sendMask ) {
  dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK );
  dwt_writetxdata( msgLen, buffer, 0 );
  dwt_writetxfctrl( msgLen + FCS_LEN, 0, 1 );
  if( dwt_starttx( sendMask ) == DWT_SUCCESS ) {
    if( !( sendMask & DWT_RESPONSE_EXPECTED ) ) {
      unsigned long endTime = micros() + 200;
      while( !( dwt_read32bitreg( SYS_STATUS_ID ) & SYS_STATUS_TXFRS_BIT_MASK ) && micros() < endTime );
      dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK );
    }
    return true;
  }
  return false;
}

static bool receiveMessageDW( unsigned int timeout ) {
  unsigned int status = 0;
  unsigned long endTime = micros() + timeout + DW_SPIN_TIMEOUT;
  while( !( ( status = dwt_read32bitreg( SYS_STATUS_ID ) ) & 
      ( SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR ) ) && micros() < endTime );
  if ( status & SYS_STATUS_RXFCG_BIT_MASK ) {
    dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK|SYS_STATUS_TXFRS_BIT_MASK );
    dw.frameLength = (unsigned char)( dwt_read32bitreg( RX_FINFO_ID ) & RXFLEN_MASK );
    dw.frameLength = dw.frameLength < DW_MAX_BUFFER_LENGTH ? dw.frameLength : DW_MAX_BUFFER_LENGTH;
    dwt_readrxdata( dw.rx.raw, dw.frameLength, 0 );
    return true;
  } else {
    dwt_write32bitreg( SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO|SYS_STATUS_ALL_RX_ERR );
    return false;
  }
}

static void processRangingData( unsigned char devID, unsigned int tof1, unsigned int tof2, const coordinate* position ) {
  entity* ranging = com_registerEntity( devID );
  ranging->nodeOrder = 0;
  ranging->lastSeen = millis();
  ranging->position = *position;
  ranging->distance = 0.5F * ( tof1 + tof2 ) * DWT_TIME_UNITS * SPEED_OF_LIGHT;
}

static void attemptDW3000Bringup() {
  test_run_info( (unsigned char*)"DS TWR RESP" );
  spiBegin( DW_IRQ, DW_RST );
  spiSelect( DW_SS );
  bool dwIdle = false;
  for( unsigned long start = millis(); !( dwIdle = dwt_checkidlerc() ) && millis() - start < DW_INIT_IDLE_POLL_MS; ) {
    delay( 20 );
  }
  if( !dwIdle ) {// Need to make sure DW IC is in IDLE_RC before proceeding
    DPRINTF( "[P] DW3000 Attempting Connection: Failure=DW_NOT_IDLE\n" );
    dw.working = false;
  } else if( dwt_initialise( DWT_DW_INIT ) == DWT_ERROR ) {
    DPRINTF( "[P] DW3000 Attempting Connection: Failure=DW_INITIALIZATION_ERROR\n" );
    dw.working = false;
  } else if( dwt_configure( &dw.config ) ) {
    DPRINTF( "[P] DW3000 Attempting Connection: Failure=DW_CONFIGURATION_ERROR\n" );
    dw.working = false;
  } else {
    DPRINTF( "[P] DW3000 Attempting Connection: Status=Success\n" );
    dw.working = true;
    extern dwt_txconfig_t txconfig_options;
    dwt_configuretxrf( &txconfig_options );// Configure the TX spectrum parameters (power, PG delay and PG count)
    dwt_setlnapamode( DWT_LNA_ENABLE|DWT_PA_ENABLE );
    dwt_setleds( DWT_LEDS_ENABLE|DWT_LEDS_INIT_BLINK );
    dwt_setrxantennadelay( DW_RX_ANT_DLY );// Apply default antenna delay value. See NOTE 2 below.
    dwt_settxantennadelay( DW_TX_ANT_DLY );
  }
}

void peripheral_dw3000Loop() {
  //Rate-limited retry: DW3000 bring-up has shown inconsistent cold-boot timing on this board. Only
  //retry while not yet working, spaced well apart so we never re-reset/reconfigure the chip or flood
  //serial output back into the instability this was built to avoid, and give up after a bounded number
  //of attempts rather than retrying forever.
  if( !dw.working && dw.attemptCount < DW_INIT_MAX_ATTEMPTS && millis() - dw.lastAttemptTime >= DW_INIT_RETRY_INTERVAL_MS ) {
    dw.attemptCount++;
    dw.lastAttemptTime = millis();
    attemptDW3000Bringup();
    DPRINTF( "[P] DW3000 Retry: Attempt=%u/%u, Success=%s\n", dw.attemptCount, DW_INIT_MAX_ATTEMPTS, dw.working ? "Yes" : "No" );
  }
  /*bool sendSuccess = false;
  if( dw.working ) {
    entity current;
    dw.broadcastIndex = com_getEntity( &current, dw.broadcastIndex );
    if( dw.broadcastIndex == MAXBYTE ) {
      dw.broadcastIndex = com_getEntity( &current, 0 );
    }
    dw.range[0].header.fromID = kafenv.info.deviceID;
    dw.range[0].header.toID = current.entityID;
    dw.range[0].header.messageType = COM_RANGING_1;
    dw.range[0].header.messageID = 0;
    dw.range[0].ranging1.position = kafenv.state.x;
    dw.queueSizes[0] = sizeof( packet_header ) + sizeof( dw.rx.ranging1 );
    dw.queueLength = 1;
    for( unsigned char count = 0; count < 5 && dw.queueLength > 0; count++ ) {
      dw.queueLength--;
      dw.coms.currentTime = millis();
      dwt_setrxaftertxdelay( DW_PERIOD_R12 - DW_RANGE_TOLERANCE );
      dwt_setrxtimeout( DW_SCAN_PERIOD );
      dwt_setpreambledetecttimeout( DW_PRE_TIMEOUT );
      if( sendMessageDW( dw.range[dw.queueLength].raw, dw.queueSizes[dw.queueLength], DWT_START_TX_IMMEDIATE|DWT_RESPONSE_EXPECTED ) 
          && receiveMessageDW( DW_SCAN_PERIOD ) ) {
        if( dw.rx.header.toID == kafenv.info.deviceID ) {
          if( dw.rx.header.messageType == COM_RANGING_2 ) {
            const uint64_t range1RxTime = get_rx_timestamp_u64();
            const uint32_t resp_tx_time = ( range1RxTime + ( DW_PERIOD_R12 * UUS_TO_DWT_TIME ) ) >> 8;
            uint64_t range2TxTime = ( ( (uint64_t)( resp_tx_time & 0xFFFFFFFEUL ) ) << 8 ) + DW_TX_ANT_DLY;
            dwt_setdelayedtrxtime( resp_tx_time );
            dwt_setrxaftertxdelay( DW_PERIOD_R23 - DW_RANGE_TOLERANCE );
            dwt_setrxtimeout( DW_RANGE_TOLERANCE * 2 );
            dwt_setpreambledetecttimeout( DW_PRE_TIMEOUT );
            const unsigned char fromID = dw.rx.header.fromID;
            dw.tx.header.toID = fromID;
            dw.tx.header.messageType = COM_RANGING_2;
            dw.tx.ranging2.processingTime = ( unsigned int )( range2TxTime - range1RxTime );
            if( sendMessageDW( dw.tx.raw, sizeof( packet_header ) + sizeof( dw.tx.ranging2 ), DWT_START_TX_DELAYED|DWT_RESPONSE_EXPECTED ) ) {
              const coordinate pos = dw.rx.ranging1.position;
              if( receiveMessageDW( DW_PERIOD_R23 + DW_RANGE_TOLERANCE ) && dw.rx.header.messageType == COM_RANGING_3 && 
                  dw.rx.header.fromID == fromID && dw.rx.header.toID == kafenv.info.deviceID ) {
                range2TxTime = get_tx_timestamp_u64();
                const uint64_t range3RxTime = get_rx_timestamp_u64();
                processRangingData( fromID, ( (unsigned int)( range3RxTime - range2TxTime ) ) - 
                    dw.rx.ranging3.processingTime, dw.rx.ranging3.timeOfFlight, &pos );
              }
            }
          } else if( dw.rx.header.messageType == COM_RANGING_1 ) {
            const unsigned char fromID = dw.rx.header.fromID;
            const uint64_t range1TxTime = get_tx_timestamp_u64();
            const uint64_t range2RxTime = get_rx_timestamp_u64();
            const uint32_t final_tx_time = ( range2RxTime + ( DW_PERIOD_R23 * UUS_TO_DWT_TIME ) ) >> 8;
            const uint64_t range3TxTime = ( ( (uint64_t)( final_tx_time & 0xFFFFFFFEUL ) ) << 8 ) + DW_TX_ANT_DLY;
            dwt_setdelayedtrxtime( final_tx_time );
            unsigned int timeOfFlight = ( (unsigned int)( range2RxTime - range1TxTime ) ) - dw.rx.ranging2.processingTime;
            dw.tx.header.toID = fromID;
            dw.tx.header.messageType = COM_RANGING_3;
            dw.tx.ranging3.processingTime = (unsigned int)( range3TxTime - range2RxTime );
            dw.tx.ranging3.timeOfFlight = timeOfFlight;
            if( sendMessageDW( dw.tx.raw, sizeof( packet_header ) + sizeof( dw.tx.ranging3 ), DWT_START_TX_DELAYED ) ) {
              processRangingData( fromID, timeOfFlight, timeOfFlight, &dw.rx.ranging2.position );
            }
          }
        }
        if( dw.rx.header.messageType != COM_RANGING_1 && dw.rx.header.messageType != COM_RANGING_2 && dw.rx.header.messageType != COM_RANGING_3 ) {
          com_step( &dw.coms );
        } else if( count == 0 ) {
          dw.frameLength = 0;
          com_step( &dw.coms );
        }
      }
    }
  }*/
}

void peripheral_dw3000Init() {
  firmware_registerPeripheral( { "dw3000", 0, sizeof( dw ), &dw, &peripheral_dw3000Init, &peripheral_dw3000Loop } );
  DPRINTF( "[P] Initializing DW3000\n" );
  dw.working = false;
  dw.frameLength = 0;
  dw.broadcastIndex = 0;
  dw.coms = { 0, DW_COM_METHOD, true, true, dw.rx.raw, 
    []() { return ( unsigned short )dw.frameLength; }, 
    []( void* buffer, unsigned short len ) { 
    dwt_setdelayedtrxtime( ( get_rx_timestamp_u64() + ( DW_RESPONSE_PERIOD * UUS_TO_DWT_TIME ) ) >> 8 );
    const unsigned char length = ( unsigned char )( len > DW_MAX_BUFFER_LENGTH ? DW_MAX_BUFFER_LENGTH : len );
    sendMessageDW( ( unsigned char* )buffer, length, DWT_START_TX_DELAYED ); 
  }, []( void* buffer, unsigned short len ) {
    if( dw.queueLength < DW_QUEUE_LENGTH ) {
      dw.queueSizes[dw.queueLength] = ( unsigned char )( len > DW_MAX_BUFFER_LENGTH ? DW_MAX_BUFFER_LENGTH : len );
      memcpy( dw.range[dw.queueLength].raw, buffer, dw.queueSizes[dw.queueLength] );
      dw.queueLength++;
    }
  } };
  dw.config = { 5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, 
    DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0 };
  dw.queueLength = 0;
  memset( &dw.range, DW_QUEUE_LENGTH * DW_MAX_BUFFER_LENGTH, 0 );
  dw.attemptCount = 1;
  dw.lastAttemptTime = millis();
  attemptDW3000Bringup();
  DPRINTF( "[P] DW 3000 Success Status: %s\n", dw.working ? "Yes" : "No" );
}