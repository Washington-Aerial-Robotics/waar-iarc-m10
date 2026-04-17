#include "../core/communication.h"
#include "../core/firmware.h"
#include "common_data.h"

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
  } network;
  bool enableAP;
};
#pragma pack( pop )

static struct {
  radio coms;
  bool wifiConnected = false;
  bool attemptReconnect = true;
  bool wifiSetAP = true;
  char networkName[32];                           //Specifies the name of the WiFi network to connect
  char networkPassword[25];                       //Specifies the password of the WiFi network
  char networkAddress[127];                       //If connected, this records the IP address of the drone
  WiFiServer server = WiFiServer( 70 );
  WiFiClient client;
} wifi;

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

void peripheral_wifiLoop() {
  if( wifi.attemptReconnect ) {
    WiFi.disconnect();
    DPRINTF( "[P] Attempting WiFi Connection: Network=%s, Password=%s\n", wifi.networkName, wifi.networkPassword );
    if( wifi.wifiSetAP ) {
      WiFi.softAPConfig( IPAddress( 192, 168, 1, 240 ), IPAddress( 192, 168, 0, 1 ), IPAddress( 255, 255, 255, 0 ) );
      WiFi.mode( WIFI_AP );
      WiFi.softAP( wifi.networkName, NULL, 1, 0, 1 );
    } else {
      WiFi.mode( WIFI_STA );
      WiFi.begin( wifi.networkName, wifi.networkPassword );
    }
    wifi.wifiConnected = false;
    wifi.attemptReconnect = false;
  }
  if( WiFi.status() == WL_CONNECTED || wifi.wifiSetAP ) {
    if( !wifi.wifiConnected ) {
      if( wifi.wifiSetAP ) {
        strcpy( wifi.networkAddress, WiFi.softAPIP().toString().c_str() );
      } else {
        strcpy( wifi.networkAddress, WiFi.localIP().toString().c_str() );
      }
      Serial.printf( "GG\076G[ip=%s]\n", wifi.networkAddress );
      DPRINTF( "[P] WiFi Connected: IP=%s\n", wifi.networkAddress );
      wifi.wifiConnected = true;
    }
    if( wifi.client.connected() || ( wifi.client = wifi.server.accept() ).connected() ) {
      wifi.coms.currentTime = millis();
      com_step( &wifi.coms );
    }
  } else {
    wifi.wifiConnected = false;
  }
}

void peripheral_wifiInit() {
  memory mempage = { "wifi", sizeof( wifi ), &wifi };
  firmware_registerPeripheral( { 0, true, &mempage, &peripheral_wifiInit, &peripheral_wifiLoop } );
  DPRINTF( "[P] Initializing WiFi\n" );
  wifi.coms.currentTime = 0;
  wifi.coms.allowBroadcast = false;
  wifi.coms.fwdReply = true;
  wifi.coms.method = WIFI_COM_METHOD;
  wifi.coms.packet = COMS_BUFFER;
  wifi.coms.receiving = &wifiReceiving;
  wifi.coms.replying = &wifiSending;
  wifi.coms.sending = &wifiSending;
  wifi.wifiConnected = false;
  wifi.attemptReconnect = true;
  wifi.wifiSetAP = true;
  strcpy( wifi.networkName,     "KAF_Quadcopter_Drone" );
  strcpy( wifi.networkPassword, "password" );
  strcpy( wifi.networkAddress,  "unknown" );
  peripheral_wifiLoop();
  wifi.server.begin();
  com_receiveMessage( COM_REQUEST_WIFI, 0, []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[P] Replying Request WiFi Command: Address=%s\n", wifi.networkAddress );
    extendedCom* respData = ( extendedCom* )*response;
    memcpy( respData->networkAddress, wifi.networkAddress, sizeof( extendedCom::networkAddress ) );
    return (unsigned short)sizeof( respData->networkAddress );
  }, []( const void* content, const packet_header header ) {} );
  com_receiveMessage( COM_SET_WIFI, sizeof( extendedCom::network ), []( void** response, const void* content, const unsigned short len ) {
    DPRINTF( "[P] Replying Set WiFi Command\n" );
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) {
    extendedCom* respData = ( extendedCom* )content;
    memcpy( wifi.networkName, respData->network.networkName, sizeof( extendedCom::network.networkName ) );
    memcpy( wifi.networkPassword, respData->network.networkPassword, sizeof( extendedCom::network.networkPassword ) );
    wifi.attemptReconnect = true;
    DPRINTF( "[P] Executing Set WiFi Command: Name=%s, Password=%s\n", wifi.networkName, wifi.networkPassword );
  } );
  com_receiveMessage( COM_SET_WIFIAP, sizeof( extendedCom::enableAP ), []( void** response, const void* content, const unsigned short len ) {
    extendedCom* comContent = ( extendedCom* )content;
    DPRINTF( "[P] Replying Set WiFi Access Point Command: AP=%u\n", comContent->enableAP );
    wifi.wifiSetAP = comContent->enableAP;
    wifi.attemptReconnect = true;
    return (unsigned short)0;
  }, []( const void* content, const packet_header header ) { } );
}