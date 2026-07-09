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
#include "../firmware/KalmanFilter3D.h"

//PUT MACROS HERE

struct {
  float baroStd;
  float baroP0;
  coordinate gpsStd;
  //PUT ALL GLOBAL VARIABLES HERE
  KalmanFilter3D kf;
} estimation;

static float baro2Altitude( const float T, const float P, const float P0 ) {
  //PUT CONVERSION CODE HERE
  return 0;
}

static void estimate_init() {
  estimation.kf = KalmanFilter3D(2, 47.654265, -122.306561); // mode 2 to convert gps to meters iniside filter
                                                             // 47.654265 for current lattitude in seattle
                                                             // -122.306561 for current longitude in se
}

static coordinate estimate_position( const sensors* sensor, const imu* imu, float baroZ, coordinate gps ) {
 
  
  // Not currently sent Baro varz, placeholder constant
  float VARZ_BAROMETER = 0.1;

  // data
  float posx = gps.x;
  float posy = gps.y;
  float posz = baroZ;

  float accx = imu->accelInput.x;
  float accy = imu->accelInput.y;
  float accz = imu->accelInput.z;

  float var_xy_pos = gps.stdv*gps.stdv;
  float var_z_pos = VARZ_BAROMETER;
  float var_acc_xyz = imu->accelInput.stdv;

  var_x_acc = imu->accelInput.stdv*imu->accelInput.stdv
  var_y_acc = var_x_acc
  var_z_acc = var_x_acc

  estimation.kf.update(posx, accx, posy, accy, posz, accz, var_xy_pos, var_x_acc, var_xy_pos, var_y_acc, var_z_pos, var_z_acc);
  // the kf object now has updated position, and position variance data

  coordinate newPos;
  newPos.x = estimation.kf.getPositionX();
  newPos.y = estimation.kf.getPositionY();
  newPos.z = estimation.kf.getPositionZ();

  // kf object tracks variance data, but currently there is nothing demanding it
  return newPos;
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
    float baroZ;
    coordinate gps;
    if( sensor->baro.update ) {
      baroZ = baro2Altitude( flight_filterSensor( CALIB_BARO_T, sensor->baro.temperature ),
          flight_filterSensor( CALIB_BARO_P, sensor->baro.pressure ), estimation.baroP0 );
      DPRINTF( "[E] Barometer Values P=%.3f, T=%.3f, H=%.3f, Z=%.3f",
          sensor->baro.pressure, sensor->baro.temperature, sensor->baro.humidity, baroZ );
    }
    if( sensor->gps.update ) {
      gps = { flight_filterSensor( CALIB_GPS_X, sensor->gps.position.x ),
              flight_filterSensor( CALIB_GPS_Y, sensor->gps.position.y ),
              flight_filterSensor( CALIB_GPS_Z, sensor->gps.position.z ) };
    }
    *estimate = estimate_position( sensor, &FLIGHT_BUFFER, baroZ, gps );
    return true;
  }
}