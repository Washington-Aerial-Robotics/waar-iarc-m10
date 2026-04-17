#include "../core/communication.h"
#include "../core/firmware.h"
#include "common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <string.h>
#include <WiFi.h>
#include <Arduino.h>
#include <WebServer.h>
#endif

#define WEBSERVER_COM_METHOD 4
#define WEBSERVER_PORT 80

struct {
  bool serverInitialized = false;
  WebServer server = WebServer( WEBSERVER_PORT );

//DO NOT MODIFY ANYTHING ABOVE THIS LINE_________________________________________________________________________________

  String homepage = 
    "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
    "<link rel=\"icon\" href=\"data:,\">"
    "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}</style></head>"
    "<body><h1>ESP32 Drone Ground Station</h1>";
  String monitor =
    "";
  String manual =
    "";

//DO NOT MODIFY ANYTHING BELOW THIS LINE_________________________________________________________________________________

} webserver;

void peripheral_webserverLoop() {
  if( webserver.serverInitialized ) {
    webserver.server.handleClient();
  } else {
    webserver.server.begin();
    webserver.serverInitialized = true;
    DPRINTF( "[P] WebServer Started Successfully\n" );
  }
}

void peripheral_webserverInit() {
  memory mempage = { "webserver", sizeof( webserver ), &webserver };
  firmware_registerPeripheral( { 0, true, &mempage, &peripheral_webserverInit, &peripheral_webserverLoop } );
  DPRINTF( "[P] Initializing Web Server\n" );
  webserver.serverInitialized = false;
  webserver.server.on( "/", []() { webserver.server.send( 200, "text/html", webserver.homepage ); } );
  webserver.server.on( "/monitor", []() { webserver.server.send( 200, "text/html", webserver.monitor ); } );
  webserver.server.on( "/manual", []() { webserver.server.send( 200, "text/html", webserver.manual ); } );
}