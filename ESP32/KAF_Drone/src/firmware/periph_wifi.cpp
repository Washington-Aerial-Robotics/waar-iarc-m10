#include "../core/communication.h"
#include "../core/firmware.h"
#include "../auxilary/common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <string.h>
#include <WiFi.h>
#include <Arduino.h>
#endif

#define WIFI_COM_METHOD 2

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

static unsigned short wifiReceiving() {
  unsigned short length;
  if( ( length = (unsigned char)wifi.client.available() ) > 0 ) {
    wifi.client.setTimeout( 100 );
    length = length < sizeof( COMS_BUFFER ) ? length : sizeof( COMS_BUFFER );
    wifi.client.readBytes( COMS_BUFFER, length );
  }
  DPRINTF( "[P] WiFi Received Message\n" );
  return length;
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
    DPRINTF( "[P] Attempting WiFi Connection: Network=%s, Password=%s\n", wifi.p.networkName, wifi.p.networkPassword );
    if( wifi.p.wifiSetAP ) {
      WiFi.softAPConfig( IPAddress( 192, 168, 1, 240 ), IPAddress( 192, 168, 0, 1 ), IPAddress( 255, 255, 255, 0 ) );
      WiFi.mode( WIFI_AP );
      WiFi.softAP( wifi.p.networkName, wifi.p.networkPassword[0] == '\0' ? NULL : wifi.p.networkPassword, 1, 0, 1 );
    } else {
      WiFi.mode( WIFI_STA );
      WiFi.begin( wifi.p.networkName, wifi.p.networkPassword );
    }
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
    DPRINTF( "[P] WiFi Active Step: IP=%s\n", wifi.p.networkAddress );
    if( wifi.client.connected() || ( wifi.client = wifi.server.accept() ).connected() ) {
      wifi.coms.currentTime = millis();
      com_step( &wifi.coms );
    }
  } else {
    wifi.wifiConnected = false;
  }
}

void peripheral_wifiInit() {
  firmware_registerPeripheral( { "wifi", sizeof( wifi.p ), sizeof( wifi ), &wifi, &peripheral_wifiInit, &peripheral_wifiLoop } );
  DPRINTF( "[P] Initializing WiFi\n" );
  wifi.p.wifiSetAP = true;
  strcpy( wifi.p.networkName,     "KAF_Quadcopter_Drone" );
  strcpy( wifi.p.networkPassword, "" );
  strcpy( wifi.p.networkAddress,  "unknown" );
  updateNetworkHash();
  wifi.coms = { 0, WIFI_COM_METHOD, false, true, COMS_BUFFER, &wifiReceiving, &wifiSending, &wifiSending };
  wifi.networkHash = ~wifi.p.networkHash;
  wifi.wifiConnected = false;
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