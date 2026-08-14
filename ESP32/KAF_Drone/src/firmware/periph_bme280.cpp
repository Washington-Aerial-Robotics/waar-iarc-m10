#include "../core/firmware.h"
#include "../core/flight.h"
#include "../auxilary/common_data.h"
#include "../auxilary/estimation.h"

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
    SENSOR_BUFFER.baro.lastUpdateMillis = millis();
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
  //KNOWN ISSUE: this consistently fails ("BME 280 Success Status: No") on real hardware even with the
  //0x76/0x77 address fallback below, despite the chip working fine via a separate, standalone test
  //sketch (BME280I2C library) on the same wiring/pins. Not yet isolated further - no functional impact
  //currently since estimation.cpp's baro/altitude logic is a stub. Low priority, parked for now.
  firmware_registerPeripheral( { "bme280", 0, sizeof( bme ), &bme, &peripheral_bme280Init, &peripheral_bme280Loop } );
  DPRINTF( "[P] Initializing BME280\n" );
  bme.working = bme.mySensor.beginI2C();
  if( !bme.working ) {
    //library defaults to I2C address 0x76 (SDO tied low); some GY-BME280 boards default SDO high
    //(0x77) when left unconnected, so retry there before giving up
    bme.mySensor.setI2CAddress( 0x77 );
    bme.working = bme.mySensor.beginI2C();
  }
  DPRINTF( "[P] BME 280 Success Status: %s\n", bme.working ? "Yes" : "No" );
}

//Pre-flight sensor status accessor (commander.cpp's sensorStatus telemetry bitfield) - see the matching
//comment on peripheral_mpu9250Working() for why no synchronization is needed for this cross-core read.
bool peripheral_bme280Working() {
  return bme.working;
}