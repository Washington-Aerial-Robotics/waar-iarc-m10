#include "estimation.h"
#include "common_data.h"
#include "../core/firmware.h"
#include "../core/flight.h"
#include <math.h>

SENSOR_BUFFERTYPE;
extern FLIGHT_BUFFERTYPE;
extern void peripheral_bme280Init();
extern void peripheral_bme280Loop();
extern void peripheral_samm10qInit();
extern void peripheral_samm10qLoop();

//DO NOT MODIFY ANYTHING ABOVE THIS LINE_________________________________________________________________________________

//PUT INCLUDES HERE
//PUT MACROS HERE

struct {
  float baroStd;
  float baroP0;
  coordinate gpsStd;
  //PUT ALL GLOBAL VARIABLES HERE
  coordinate lastGpsXY;   //last quality-gated GPS x/y (z unused), held across ticks until GPS_STALE_MS
  float lastBaroZ;        //last computed barometric relative altitude, held across ticks until BARO_STALE_MS
} estimation;

//Standard barometric formula referenced to P0 (the ground-level pressure captured during calibration -
//see kafenv.cal.sensefilt[CALIB_BARO_P].ofst below), giving altitude in metres above that reference. This
//is deliberately simple (no temperature-lapse-rate correction beyond T's role in flight_filterSensor's own
//calibration) - adequate for a short (~1-2 minute) qualification flight at a stable ambient temperature,
//not a general-purpose long-duration altimeter.
static float baro2Altitude( const float T, const float P, const float P0 ) {
  if( P <= 0 || P0 <= 0 ) {
    return 0;
  }
  return 44330.0F * ( 1.0F - powf( P / P0, 1.0F / 5.255F ) );
}

static void estimate_init() {
  //PUT INITIALIZATION CODE HERE
  estimation.lastGpsXY = { 0, 0, 0 };
  estimation.lastBaroZ = 0;
}

//Combines the last quality-gated GPS x/y with the last valid barometric z. Deliberately simple: this is a
//"most recent valid sample, held until stale" hold, not a Kalman/complementary fusion of GPS+baro+IMU by
//itself - the actual GPS/IMU fusion (alpha-blending this correction against IMU-integrated dead-reckoning)
//already happens one layer up, in flight.cpp's positionEstimate(), via kafenv.cal.positionalpha. Baro is
//preferred for z over GPS's own altitude field because vertical GPS error is typically several times
//larger than horizontal error, and this qualification test needs ~1m-scale altitude precision (20ft/6m
//hold, 4 drones at the same nominal altitude).
static coordinate estimate_position( const sensors* sensor, const imu* imu, float baroZ, coordinate gps, bool haveBaro ) {
  return { gps.x, gps.y, haveBaro ? baroZ : gps.z };
}

//DO NOT MODIFY ANYTHING BELOW THIS LINE_________________________________________________________________________________

peripheral estimation_reset() {
  firmware_registerPeripheral( { "com_sense", 0, sizeof( SENSOR_BUFFER ), &SENSOR_BUFFER, NULLPTR, NULLPTR } );
  peripheral_bme280Init();
  peripheral_samm10qInit();
  DPRINTF( "[E] Resetting Estimation\n" );
  estimation.baroStd = 1e6;
  estimation.baroP0  = 101300;
  estimation.gpsStd  = { 1e6, 1e6, 1e6 };
  estimate_init();
  return { "estimation", 0, sizeof( estimation ), &estimation, [](){ estimation_reset(); }, NULLPTR };
}

bool estimation_step( coordinate* estimate ) {
  peripheral_bme280Loop();
  peripheral_samm10qLoop();
  const sensors* sensor = &SENSOR_BUFFER;
  if( estimate == NULLPTR ) {
    if( sensor->baro.update ) {
      estimation.baroStd = sqrtf( flight_calibrateSensor( CALIB_BARO_P, sensor->baro.pressure    ) + 
                                  flight_calibrateSensor( CALIB_BARO_T, sensor->baro.temperature ) );
      estimation.baroP0 = kafenv.cal.sensefilt[CALIB_BARO_P].ofst * kafenv.cal.sensefilt[CALIB_BARO_P].gain;
      kafenv.cal.sensefilt[CALIB_BARO_T].ofst = 0;
      DPRINTF( "[E] Barometer Calibration: P0=%.3f, stdev=%.3f\n", estimation.baroP0, estimation.baroStd );
    }
    if( sensor->gps.update ) {
      estimation.gpsStd = { sqrtf( flight_calibrateSensor( CALIB_GPS_X, sensor->gps.position.x ) ),
                            sqrtf( flight_calibrateSensor( CALIB_GPS_Y, sensor->gps.position.y ) ),
                            sqrtf( flight_calibrateSensor( CALIB_GPS_Z, sensor->gps.position.z ) ) };
      DPRINTF( "[E] GPS Trim: Offset=[ %f, %f, %f ]\n", 
          kafenv.cal.sensefilt[CALIB_GPS_X].ofst, kafenv.cal.sensefilt[CALIB_GPS_Y].ofst, kafenv.cal.sensefilt[CALIB_GPS_Z].ofst );
      DPRINTF( "[E] GPS Variance: Stdev=[ %f, %f, %f ]\n", 
          kafenv.cal.sensefilt[CALIB_GPS_X].stdv, kafenv.cal.sensefilt[CALIB_GPS_Y].stdv, kafenv.cal.sensefilt[CALIB_GPS_Z].stdv );
    }
    return true;
  } else {
    if( sensor->baro.update ) {
      estimation.lastBaroZ = baro2Altitude( flight_filterSensor( CALIB_BARO_T, sensor->baro.temperature ),
          flight_filterSensor( CALIB_BARO_P, sensor->baro.pressure ), estimation.baroP0 );
      DPRINTF( "[E] Barometer Values P=%.3f, T=%.3f, H=%.3f, Z=%.3f\n",
          sensor->baro.pressure, sensor->baro.temperature, sensor->baro.humidity, estimation.lastBaroZ );
    }
    if( sensor->gps.update ) {
      estimation.lastGpsXY = { flight_filterSensor( CALIB_GPS_X, sensor->gps.position.x ),
              flight_filterSensor( CALIB_GPS_Y, sensor->gps.position.y ),
              flight_filterSensor( CALIB_GPS_Z, sensor->gps.position.z ) };
    }
    //Do NOT offer a correction built from a stale or never-set GPS fix - the caller (flight.cpp's
    //positionEstimate()) treats a `false` return as "no correction this tick, keep dead-reckoning", which
    //is the existing, already-safe fallback. Previously this branch unconditionally called
    //estimate_position() and returned true even when sensor->gps.update was false, passing an
    //uninitialized/stale local `gps` variable into position control - the exact "silently continue with
    //bad coordinates" failure this replaces.
    if( !estimation_positionValid() ) {
      return false;
    }
    const bool haveBaro = ( millis() - sensor->baro.lastUpdateMillis ) <= BARO_STALE_MS;
    *estimate = estimate_position( sensor, &FLIGHT_BUFFER, estimation.lastBaroZ, estimation.lastGpsXY, haveBaro );
    return true;
  }
}

bool estimation_positionValid() {
  return SENSOR_BUFFER.gps.originSet
      && SENSOR_BUFFER.gps.lastFixMillis != 0
      && ( millis() - SENSOR_BUFFER.gps.lastFixMillis ) <= GPS_STALE_MS;
}