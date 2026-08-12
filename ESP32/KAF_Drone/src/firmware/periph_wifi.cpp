#include "../core/communication.h"
#include "../core/firmware.h"
#include "../auxilary/common_data.h"
#include "../auxilary/commander.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <string.h>
#include <WiFi.h>
#include <Arduino.h>
#endif

#define WIFI_COM_METHOD 2
#define WIFI_RX_CAPACITY 512

extern COMS_BUFFERTYPE;

#pragma pack( push, 1 )
union extendedCom {
  char networkAddress[20];
  struct {
    char networkName[20];
    char networkPassword[20];
    bool enableAP;
  } network;
};
#pragma pack( pop )

static struct {
  struct {
    unsigned int networkHash;
    bool wifiSetAP;
    char networkName[25];                           //Specifies the name of the WiFi network to connect
    char networkPassword[25];                       //Specifies the password of the WiFi network
    char networkAddress[25];                        //If connected, this records the IP address of the drone
  } p;
  unsigned int networkHash;
  bool wifiConnected = false;
  radio coms;
  WiFiServer server = WiFiServer( 70 );
  WiFiClient client;
  unsigned short receiveLength;
  unsigned char receiveBuffer[WIFI_RX_CAPACITY];
} wifi;

static void updateNetworkHash() {
  wifi.p.networkHash = 0;
  unsigned int hash = 0x84222325;
  const unsigned char* bytes = ( const unsigned char* )( ( void* )&wifi.p );
  for( unsigned char i = 0; i < sizeof( wifi.p ); i++ ) {
    hash = ( hash ^ bytes[i] ) * 0x1B3;
  }
  wifi.p.networkHash = hash;
  DPRINTF( "[P] Wifi Updated Network Hash: Hash=%08x, Previous=%08x\n", wifi.p.networkHash, wifi.networkHash );
}

static void wifiReadUpTo( const unsigned short targetLength ) {
  const int available = wifi.client.available();
  if( available <= 0 || wifi.receiveLength >= targetLength ) return;
  unsigned short readLength = targetLength - wifi.receiveLength;
  if( readLength > ( unsigned short )available ) readLength = ( unsigned short )available;
  const size_t received = wifi.client.readBytes( ( char* )&wifi.receiveBuffer[wifi.receiveLength], readLength );
  wifi.receiveLength += ( unsigned short )received;
}

//Returns zero for legacy/unframed commands. Autonomy requests have an exact size, so TCP fragmentation is
//reassembled and coalesced data is not consumed past the current request. COM_SET_FLIGHTMODE carries its
//own float count; the Pi is authorized only for count zero, but consuming the announced frame also keeps a
//malformed request from desynchronizing the following header.
static unsigned short wifiAutonomyFrameLength() {
  if( wifi.receiveLength < sizeof( packet_header ) ) return sizeof( packet_header );
  switch( wifi.receiveBuffer[2] ) {
    case COM_SET_GPSORIGIN : case COM_REQUEST_TELEMETRY : return sizeof( packet_header );
    case COM_SET_ACTUATION : case COM_SET_TRAJECTORY : return sizeof( packet_header ) + 1;
    case COM_SET_TRAJSETPT : return sizeof( packet_header ) + sizeof( trajsetpoint );
    case COM_SET_FLIGHTMODE : {
      const unsigned short headerLength = sizeof( packet_header ) + 2;
      if( wifi.receiveLength < headerLength ) return headerLength;
      return headerLength + ( ( unsigned short )wifi.receiveBuffer[5] ) * sizeof( float );
    }
    default : return 0;
  }
}

static unsigned short wifiReceiving() {
  wifi.client.setTimeout( 100 );
  //Read only enough to identify a header first. This is what prevents a second coalesced autonomy request
  //from being swallowed as payload for the first one.
  wifiReadUpTo( sizeof( packet_header ) );
  if( wifi.receiveLength < sizeof( packet_header ) ) return 0;

  unsigned short frameLength = wifiAutonomyFrameLength();
  if( frameLength != 0 ) {
    if( frameLength > WIFI_RX_CAPACITY ) {
      //Return the available prefix so the protocol layer emits COM_FAILURE for the impossible declared
      //length, then reset instead of overflowing the accumulator.
      frameLength = wifi.receiveLength;
    } else {
      wifiReadUpTo( frameLength );
      frameLength = wifiAutonomyFrameLength(); //flight-mode size becomes known after its two-byte header
      if( frameLength > WIFI_RX_CAPACITY ) frameLength = wifi.receiveLength;
      if( wifi.receiveLength < frameLength ) {
        wifiReadUpTo( frameLength );
        if( wifi.receiveLength < frameLength ) return 0;
      }
    }
  } else {
    //Legacy compatibility: once a non-autonomy header is identified, preserve the old behavior of handing
    //all currently available bytes to com_step(). Legacy TCP commands still require one-request-at-a-time
    //pacing; the autonomy path above is the fragmentation-safe interface used by Ubuntu.
    wifiReadUpTo( WIFI_RX_CAPACITY );
    frameLength = wifi.receiveLength;
  }

  memcpy( COMS_BUFFER, wifi.receiveBuffer, frameLength );
  wifi.receiveLength = 0;
  DPRINTF( "[P] WiFi Received Complete Frame: Length=%u\n", frameLength );
  return frameLength;
}

static void wifiSending( void* buffer, unsigned short len ) {
  wifi.client.write( (char*)buffer, len );
  wifi.client.flush();
}

void peripheral_wifiNetwork( const bool rwflag, char* name, char* password, char* address, bool* ap ) {
  if( rwflag ) {
    strncpy( wifi.p.networkName, name, sizeof( wifi.p.networkName ) - 1 );
    strncpy( wifi.p.networkPassword, password, sizeof( wifi.p.networkPassword ) - 1 );
    wifi.p.wifiSetAP = *ap;
    updateNetworkHash();
  } else {
    strncpy( name, wifi.p.networkName, sizeof( wifi.p.networkName ) - 1 );
    strncpy( password, wifi.p.networkPassword, sizeof( wifi.p.networkPassword ) - 1 );
    strncpy( address, wifi.p.networkAddress, sizeof( wifi.p.networkAddress ) - 1 );
    *ap = wifi.p.wifiSetAP;
  }
}

void peripheral_wifiLoop() {
  if( wifi.networkHash != wifi.p.networkHash ) {
    WiFi.disconnect();
    wifi.receiveLength = 0;
    DPRINTF( "[P] Attempting WiFi Connection: Network=%s, Password=%s\n", wifi.p.networkName, wifi.p.networkPassword );
    if( wifi.p.wifiSetAP ) {
      //The AP itself is the gateway; both addresses must be in the configured /24 subnet. The former
      //192.168.0.1 gateway with a 192.168.1.240 AP address caused clients to install an unusable route.
      WiFi.softAPConfig( IPAddress( 192, 168, 1, 240 ), IPAddress( 192, 168, 1, 240 ), IPAddress( 255, 255, 255, 0 ) );
      WiFi.mode( WIFI_AP );
      WiFi.softAP( wifi.p.networkName, wifi.p.networkPassword[0] == '\0' ? NULL : wifi.p.networkPassword, 1, 0, 1 );
    } else {
      WiFi.mode( WIFI_STA );
      WiFi.begin( wifi.p.networkName, wifi.p.networkPassword );
    }
    //WiFi modem sleep periodically blocks the CPU for tens of ms at unpredictable times to power-cycle
    //the radio - well-documented on ESP32 as breaking time-sensitive I2C/SPI transactions (exactly the
    //MPU9250 read failures seen once WiFi came up). Disabling it trades some power draw for reliability.
    WiFi.setSleep( false );
    wifi.wifiConnected = false;
    wifi.networkHash = wifi.p.networkHash;
  }
  if( WiFi.status() == WL_CONNECTED || wifi.p.wifiSetAP ) {
    if( !wifi.wifiConnected ) {
      if( wifi.p.wifiSetAP ) {
        strncpy( wifi.p.networkAddress, WiFi.softAPIP().toString().c_str(), sizeof( wifi.p.networkAddress ) - 1 );
      } else {
        strncpy( wifi.p.networkAddress, WiFi.localIP().toString().c_str(), sizeof( wifi.p.networkAddress ) - 1 );
      }
      Serial.printf( "GG\076G[ip=%s]\n", wifi.p.networkAddress );
      wifi.wifiConnected = true;
    }
    //DEBUG logging throttled to ~1/s - see periph_freertos.cpp for why
    static unsigned long lastWifiPrint = 0;
    if( millis() - lastWifiPrint > 1000 ) {
      lastWifiPrint = millis();
      DPRINTF( "[P] WiFi Active Step: IP=%s\n", wifi.p.networkAddress );
    }
    if( !wifi.client.connected() ) {
      //Discard an incomplete frame when the peer disconnects; a new client/session must begin at a header.
      wifi.receiveLength = 0;
      wifi.client = wifi.server.accept();
    }
    if( wifi.client.connected() ) {
      wifi.coms.currentTime = millis();
      com_step( &wifi.coms );
    }
  } else {
    wifi.wifiConnected = false;
    wifi.receiveLength = 0;
  }
}

void peripheral_wifiInit() {
  //NOT registered as persistent (0, not sizeof(wifi.p)): this board's EEPROM contains a validly-signed
  //but stale/blank wifi.p blob from an earlier flash, and firmware_handlePersistents() restores ALL
  //persistent peripherals from one shared blob on every boot - that was silently overwriting these
  //correct hardcoded WiFi AP defaults with blank credentials moments after boot (visible in logs as
  //"Attempting WiFi Connection: Network=, Password=" right after "Running Startup Commands").
  firmware_registerPeripheral( { "wifi", 0, sizeof( wifi ), &wifi, &peripheral_wifiInit, &peripheral_wifiLoop } );
  DPRINTF( "[P] Initializing WiFi\n" );
  wifi.p.wifiSetAP = true;
  strcpy( wifi.p.networkName,     "KAF_Quadcopter_Drone" );
  strcpy( wifi.p.networkPassword, "" );
  strcpy( wifi.p.networkAddress,  "unknown" );
  updateNetworkHash();
  wifi.coms = { 0, WIFI_COM_METHOD, false, true, COMS_BUFFER, &wifiReceiving, &wifiSending, &wifiSending };
  wifi.networkHash = ~wifi.p.networkHash;
  wifi.wifiConnected = false;
  wifi.receiveLength = 0;
  memset( wifi.receiveBuffer, 0, sizeof( wifi.receiveBuffer ) );
  peripheral_wifiLoop();
  wifi.server.begin();
  com_receiveMessage( COM_REQUEST_WIFI, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[P] Replying Request WiFi Command: Address=%s\n", wifi.p.networkAddress );
    extendedCom* respData = ( extendedCom* )*response;
    memcpy( respData->networkAddress, wifi.p.networkAddress, sizeof( extendedCom::networkAddress ) );
    return (unsigned short)sizeof( respData->networkAddress );
  }, []( const void* content, const packet_header header ) {} );
  com_receiveMessage( COM_SET_WIFI, sizeof( extendedCom::network ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[P] Replying Set WiFi Command\n" );
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {
    extendedCom* comContent = ( extendedCom* )content;
    memcpy( wifi.p.networkName, comContent->network.networkName, sizeof( extendedCom::network.networkName ) );
    memcpy( wifi.p.networkPassword, comContent->network.networkPassword, sizeof( extendedCom::network.networkPassword ) );
    wifi.p.wifiSetAP = comContent->network.enableAP;
    updateNetworkHash();
    DPRINTF( "[P] Executing Set WiFi Command: Name=%s, Password=%s AP=%u\n", wifi.p.networkName, wifi.p.networkPassword, wifi.p.wifiSetAP );
  } );
}
