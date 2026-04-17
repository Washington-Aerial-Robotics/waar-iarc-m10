#include "../core/firmware.h"
#include "../core/flight.h"
#include "common_data.h"
#include "estimation.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <math.h>
#include <Arduino.h>
#include "../lib/SparkFun_BME280/SparkFunBME280.h"
#endif

extern SENSOR_BUFFERTYPE;

static struct {
  bool working = false;
  BME280 mySensor;
} bme;

void peripheral_bme280Loop() {
  if( bme.working ) {
    SENSOR_BUFFER.baro.humidity    = bme.mySensor.readFloatHumidity();
    SENSOR_BUFFER.baro.pressure    = bme.mySensor.readFloatPressure();
    SENSOR_BUFFER.baro.temperature = bme.mySensor.readTempC();
    SENSOR_BUFFER.baro.update      = true;
    DPRINTF( "[P] Barometer Data: Pressure=%f, Temperature=%f, Humidity=%f\n", 
        SENSOR_BUFFER.baro.pressure, SENSOR_BUFFER.baro.temperature, SENSOR_BUFFER.baro.humidity );
  } else {
    SENSOR_BUFFER.baro.humidity    = 0;
    SENSOR_BUFFER.baro.pressure    = 0;
    SENSOR_BUFFER.baro.temperature = 0;
    SENSOR_BUFFER.baro.update      = false;
  }
}

void peripheral_bme280Init() {
  memory mempage = { "bme280", sizeof( bme ), &bme };
  firmware_registerPeripheral( { 0, true, &mempage, &peripheral_bme280Init, &peripheral_bme280Loop } );
  DPRINTF( "[P] Initializing BME280\n" );
  bme.working = bme.mySensor.beginI2C();
  DPRINTF( "[P] BME 280 Success Status: %s\n", bme.working ? "Yes" : "No" );
}