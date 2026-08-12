#include "../core/kaf_drone.h"
#include <stdint.h>

#define CMD_MODE_MASK    0b11111000
#define CMD_IDLE_MODE             0 //FLASH
#define CMD_NOMINAL_MODE          8 //FLASH STABILITY DISCONNECT BATTERY 
#define CMD_NULL_MODE            16 //
#define CMD_DESCENT_MODE         24 //FLASH STABILITY                    DESCENT

#define FLIGHTPATH_NONE           0
#define FLIGHTPATH_LAUNCH         1
#define FLIGHTPATH_LAND           2
#define FLIGHTPATH_POSLOCK        3
#define FLIGHTPATH_GLIDEPOINT     4
#define FLIGHTPATH_RETURNHOME     5
#define FLIGHTPATH_CIRCLE         6

#define GROUND_STATION_ID       'G'
#define PI_CONTROLLER_ID        'P'

//How long a POS_SETPOINT_MODE/TRAJECTORY_MODE setpoint (kafenv.cmd.setpointMillis, set by COM_SET_TRAJSETPT)
//may go without a fresh update before commander_step() treats it as lost communication with the Pi and
//falls back to FLIGHTPATH_LAND - the same emergency-descent path already used for ground-station disconnect
//and low battery. Deliberately short relative to those (Pi setpoints are expected at well over 1Hz).
#define SETPOINT_STALE_MS 500UL

#define AUTONOMY_TELEMETRY_VERSION 1
#define AUTONOMY_FLAG_POSITION_VALID 0x01
#define AUTONOMY_FLAG_ORIGIN_SET     0x02
#define AUTONOMY_FLAG_ACTUATION      0x04
#define AUTONOMY_FLAG_PI_STREAM      0x08
#define AUTONOMY_FLAG_SETPOINT_FRESH 0x10
#define AUTONOMY_FLAG_ATTITUDE_VALID 0x20
#define AUTONOMY_FLAG_CONTROL_CALIBRATED 0x40
#define AUTONOMY_FLAG_BATTERY_VALID      0x80

//Airframe-specific bounds for the measured hover feed-forward. Keeping these as named configuration
//limits makes the arm gate explicit; do not replace the intentionally-invalid default with a guessed value.
#define CONTROL_HOVER_THRUST_MIN 0.0F
#define CONTROL_HOVER_THRUST_MAX 1.0F
#define CONTROL_GRAVITY_MIN      5.0F
#define CONTROL_GRAVITY_MAX     15.0F

#pragma pack( push, 1 )
//Wire payload for COM_SET_TRAJSETPT (0x5d). All fields are little-endian on ESP32.
struct trajsetpoint {
  uint32_t sequence;
  float x, y, z, yaw;
  float vx, vy, vz;
};

//Wire payload for COM_REPLY_TELEMETRY (0x25). Quaternion order is ROS x,y,z,w.
struct autonomy_telemetry {
  uint8_t protocolVersion;
  uint8_t flightMode;
  uint8_t flags;
  uint8_t reserved;
  uint32_t setpointSequence;
  float position[3];
  float velocity[3];
  float quaternion[4];
  float angularVelocity[3];
  float battery;
};
#pragma pack( pop )

static_assert( sizeof( uint32_t ) == 4, "Autonomy protocol requires a 32-bit uint32_t" );
static_assert( sizeof( float ) == 4, "Autonomy protocol requires IEEE-754 32-bit floats" );
static_assert( sizeof( trajsetpoint ) == 32, "Unexpected COM_SET_TRAJSETPT wire size" );
static_assert( sizeof( autonomy_telemetry ) == 64, "Unexpected COM_REPLY_TELEMETRY wire size" );

//Returns false without changing flight state/configuration when the requested path is invalid or requires
//a position estimate that is not currently valid.
bool commander_setTrajectories( STDBYTE mode, const float args[4] );

//Protocol/mode guard hooks used by communication.cpp. Validation happens before a success response is sent;
//the corresponding accept function is called only after validation succeeded.
bool commander_validateFlightModeCommand( STDBYTE mode, unsigned char length, const float* values, STDBYTE sender );
void commander_acceptFlightModeCommand( STDBYTE mode, unsigned char length, STDBYTE sender );
void commander_acceptLegacySetpoint( STDBYTE sender, unsigned char length );
bool commander_canArm( STDBYTE sender );
bool commander_controlCalibrated();
bool commander_batteryValid();
bool commander_piStreamActive();
bool commander_piSetpointFresh( unsigned long currentTime );
void commander_resetPiSequence();

peripheral commander_reset();
void commander_step( const unsigned long currentTime );
