#include "../core/firmware.h"
#include "../core/flight.h"
#include "../auxilary/common_data.h"

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
  unsigned long currentFlightTime;
  void ( *flightTask )( void* );
  void ( *comTask )( void* );
} freertos;

void peripheral_freertosLoopComs() {
  vTaskDelay( 1 );
  delay( rand() % COM_TASK_VARIANCE_MS );
  //DEBUG logging throttled to ~1/s - unconditional per-loop DPRINTFs from both tasks were saturating
  //the 115200 baud Serial link, which was itself the dominant cause of inflated flight step periods
  static unsigned long lastComsPrint = 0;
  if( millis() - lastComsPrint > 1000 ) {
    lastComsPrint = millis();
    DPRINTF( "[P] FreeRTOS Coms Step\n" );
  }
}

void peripheral_freertosLoopFlight() {
  vTaskDelay( 0 );
  const unsigned long oldTime = freertos.currentFlightTime;
  const unsigned int timeDiff = (int)( millis() - oldTime );
  if( timeDiff < FLIGHT_TASK_PERIOD_MS ) {
    delay( FLIGHT_TASK_PERIOD_MS - timeDiff );
  }
  freertos.currentFlightTime = millis();
  FLIGHT_BUFFER.timeStep = ( freertos.currentFlightTime - oldTime ) * 1e-3F;
  static unsigned long lastFlightPrint = 0;
  if( millis() - lastFlightPrint > 1000 ) {
    lastFlightPrint = millis();
    DPRINTF( "[P] FreeRTOS Flight Step: Current=%lu, Prior=%lu\n", freertos.currentFlightTime, oldTime );
    DPRINTF( "[P] FreeRTOS Flight Performance: Step Period=%.3f, Completion Time=%.3f\n", FLIGHT_BUFFER.timeStep, timeDiff * 1e-3F );
  }
}

void peripheral_freertosSetup( void ( *flightTask )( void* ), void ( *comTask )( void* ) ) {
  firmware_registerPeripheral( { "freertos", 0, sizeof( freertos ), &freertos, NULLPTR, NULLPTR } );
  freertos.comTask = comTask;
  freertos.flightTask = flightTask;
  DPRINTF( "[P] Executing FreeRTOS Task System\n" );
  delay( 500 );
  //flight_task back on core 0: Arduino's own loopTask (which runs the sketch's loop()) is pinned to
  //core 1 by this build's ARDUINO_RUNNING_CORE=1 setting regardless of our own task placement, so
  //putting flight_task on core 1 (as an earlier attempt did) made it directly compete with loopTask
  //at equal priority for CPU time - a likely cause of the observed ~2x (20ms vs intended 10ms) step
  //periods and the resulting MPU9250 I2C read failures, separate from any WiFi-specific timing issue.
  //currentFlightTime is set here (not earlier) so the very first step-period measurement reflects the
  //actual gap since task creation, not the full elapsed boot time since currentFlightTime was zeroed.
  freertos.currentFlightTime = millis();
  BaseType_t flightResult = xTaskCreatePinnedToCore( flightTask, "flight_task", 5000, NULL, 1, NULL, 0 );
  if( flightResult != pdPASS ) {
    DPRINTF( "[P] FreeRTOS FATAL: flight_task creation failed, result=%d\n", flightResult );
  }
  delay( 500 );
  BaseType_t comResult = xTaskCreatePinnedToCore( comTask, "com_task", 5000, NULL, 1, NULL, 1 );
  if( comResult != pdPASS ) {
    DPRINTF( "[P] FreeRTOS FATAL: com_task creation failed, result=%d\n", comResult );
  }
  delay( 500 );
}