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
    coordinate position;      //metres, relative to the latched local-frame origin (see gps_setOrigin())
    bool update;               //true only on ticks where a fresh, quality-gated fix updated position
    bool originSet;             //false until gps_setOrigin() has latched a good fix as the local origin
    unsigned char fixQuality;   //raw NMEA GNGGA fix quality field (0 = invalid/no fix)
    unsigned char satellites;   //raw NMEA GNGGA satellite count
    float hdop;                 //raw NMEA GNGGA horizontal dilution of precision (lower is better)
    unsigned long lastFixMillis; //millis() timestamp of the last quality-gated accepted fix
  } gps;
  struct {
    float pressure;
    float humidity;
    float temperature;
    bool update;
    unsigned long lastUpdateMillis; //millis() timestamp of the last accepted barometer reading
  } baro;
};

//Minimum acceptable GPS fix quality to trust a position for autonomous flight - see periph_samm10q.cpp.
#define GPS_MIN_SATELLITES 6
#define GPS_MAX_HDOP       3.0F
#define GPS_STALE_MS       2000UL
#define BARO_STALE_MS      2000UL

peripheral estimation_reset();
bool estimation_step( coordinate* estimate );
//Rebases the barometric altitude to the same launch-local origin as GPS. A barometer is adopted only when
//its current sample is fresh; otherwise estimation continues using GPS-relative Z until another origin latch.
void estimation_latchLocalOrigin();
//Latches the current validated GPS fix as the local metric-frame origin (x=y=z=0). Returns false (and does
//not latch anything) if the current fix doesn't meet GPS_MIN_SATELLITES/GPS_MAX_HDOP/freshness - callers
//(e.g. the qualification state machine, before allowing a launch command) must check this return value
//rather than assuming an origin was set.
bool gps_setOrigin();
//True only when the most recent GPS fix was quality-gated-valid and received within GPS_STALE_MS - the
//single source of truth mode-entry guards and the qualification failure monitor should check before
//trusting kafenv.state.x for autonomous position/trajectory control.
bool estimation_positionValid();
//True after a validated fix has been explicitly latched as the local-frame origin. Unlike
//estimation_positionValid(), this does not imply that the latest fix is still fresh.
bool estimation_originSet();
