#include "../core/firmware.h"
#include "../core/flight.h"
#include "common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include <math.h>
#endif

#define FLIGHT_TASK_PERIOD_MS 10
#define COM_TASK_VARIANCE_MS  15

extern FLIGHT_BUFFERTYPE;

static struct {
  unsigned long currentTime;
  void ( *flightTask )( void* );
  void ( *comTask )( void* );
} freertos;

void peripheral_freertosLoopComs() {
  vTaskDelay( 1 );
  delay( rand() % COM_TASK_VARIANCE_MS );
  DPRINTF( "[P] FreeRTOS Coms Step\n" );
}

void peripheral_freertosLoopFlight() {
  vTaskDelay( 0 );
  const unsigned long oldTime = freertos.currentTime;
  const unsigned int timeDiff = (int)( millis() - oldTime );
  if( timeDiff < FLIGHT_TASK_PERIOD_MS ) {
    delay( FLIGHT_TASK_PERIOD_MS - timeDiff );
  }
  freertos.currentTime = millis();
  FLIGHT_BUFFER.timeStep = ( freertos.currentTime - oldTime ) * 1e-3F;
  DPRINTF( "[P] FreeRTOS Flight Step: Current=%lu, Prior=%lu\n", freertos.currentTime, oldTime );
  DPRINTF( "[P] FreeRTOS Flight Performance: Step Period=%.3f, Completion Time=%.3f\n", FLIGHT_BUFFER.timeStep, timeDiff * 1e-3F );
}

void peripheral_freertosSetup( void ( *flightTask )( void* ), void ( *comTask )( void* ) ) {
  firmware_registerMemoryPage( { "freertos", sizeof( freertos ), &freertos } );
  freertos.currentTime = 0;
  freertos.comTask = comTask;
  freertos.flightTask = flightTask;
  DPRINTF( "[P] Executing FreeRTOS Task System\n" );
  delay( 500 );
  xTaskCreatePinnedToCore( flightTask, "flight_task", 5000, NULL, 1, NULL, 0 );
  delay( 500 );
  xTaskCreatePinnedToCore( comTask,    "com_task",    5000, NULL, 1, NULL, 1 );
  delay( 500 );
}