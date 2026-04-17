#include "../core/kaf_drone.h"

#define CALIB_BARO_P 0
#define CALIB_BARO_T 1
#define CALIB_GPS_X  2
#define CALIB_GPS_Y  3
#define CALIB_GPS_Z  4

struct sensors {
  struct {
    double longitude;
    double latitude;
    double altitude;
    coordinate position;
    bool update;
  } gps;
  struct {
    float pressure;
    float humidity;
    float temperature;
    bool update;
  } baro;
};

memory estimation_reset();
bool estimation_step( coordinate* estimate );