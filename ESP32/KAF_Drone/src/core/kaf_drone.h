/*  
 *  Author: Kent Fukuda
 *  KAF Drone: Flight Software Code
 *  Edit Date: January 24, 2026
 *  Version: 20260124
 *  
 *  kaf_drone.h
 *  Drone State Data Header File
 */

//CONFIGURATION_______________________________________________________________________________________________
#ifndef KAF_DRONE_CONFIG
#define KAF_DRONE_CONFIG
#ifdef _MSC_VER
#define ALT_DEFINE 1
#define PRINTF
#endif
#ifdef __GNUC__
#ifdef __MINGW64__
#include "mex.h"
#define ALT_DEFINE 0
#define PRINTF mexPrintf
#else
#define ALT_DEFINE 0
#include <Arduino.h>
#define PRINTF Serial.printf
#endif
#endif
#endif

#ifndef KAF_DRONE
#define KAF_DRONE

//MACROS______________________________________________________________________________________________________
#define MAXBYTE 255
#define NULLPTR 0
#define STDBYTE unsigned char
#define PERSIST 0
#define DPRINTF( ... ) //PRINTF( __VA_ARGS__ )
#define FPARLEN( F ) ( sizeof( F ) / sizeof( float ) )
#define FPFILL0( N, F ) for( unsigned char N = 0; N < FPARLEN( F ); N++ ) F[N] = 0
#define ITRVEC3( N ) for( unsigned char N = 0; N < 3; N++ )
#define FPGUARD( N, V ) if( !isfinite( N ) ) N = V

//STRUCTURES__________________________________________________________________________________________________
//coordinate definition
union coordinate {
  struct {
    float x;
    float y;
    float z;
  };
  struct {
    float Kp;
    float Ki;
    float Kd;
  };
  struct {
    float gain;
    float ofst;
    float stdv;
  };
  float f[3];
};

//memory page definition
struct memory {
  char name[14];
  unsigned short length;
  void* address;
};

//GLOBAL VARIABLES____________________________________________________________________________________________
struct drone_state {
  struct droneinfo {           //IDENTIFICATION INFORMATION
    STDBYTE deviceID;          //Character designating a unique ID of a drone
    STDBYTE flightMode;        //Current flight mode status of the drone
    STDBYTE trigger;           //Custom device software trigger action bits
    bool actuation;            //Configuration for if motor actuation is enabled or not
    unsigned int version;      //Flight software version
    float battery;             //Percentage amount of battery left
  } info;
  struct stateestimate {       //STATE ESTIMATION
    coordinate x;              //Position in m in the world frame
    coordinate v;              //Velocity in m/s in the world frame
    coordinate w;              //Angular rate coordinate in rad/s in the body frame
    coordinate q;              //Yaw q.z in rad world frame, quaternion elements q.x, q.y in the body frame
  } state;
  struct {                     //TRAJECTORY COMMANDS
    float setpoints[22];       //Setpoint float point data representing coordinates, values, or trajectories
    float motors[4];           //Variable to throttle the target PWM voltage as an unitless scalar 0<=1
  } cmd;
  struct calibrations {        //CALIBRATION DATA
    float anglealpha;          //Alpha filter value for merging predicted attitudes (rad/rad)
    float positionalpha;       //Time constant value for merging predicted position estimates (rad/s)
    float gravitation;         //Necessary minimum voltage required to sustain lift (V)
    coordinate xpid;           //PID values for the position controller (m, s)
    coordinate vpid;           //PID values for the velocity controller (m/s, s)
    coordinate qpid;           //PID values for the attitude controller (rad, s)
    coordinate wpid[3];        //PID values for the angular rate controller (rad/s, s)
    coordinate gyrofilt[3];    //NLA values for the gyroscope alpha filter (bytes -> rad/s)
    coordinate accelfilt[3];   //NLA values for the accelerometer alpha filter (bytes -> m/s2)
    coordinate magfilt[3];     //NLA values for the magnetometer alpha filter (bytes -> rad)
    coordinate sensefilt[8];   //NLA values for other sensor alpha filter
  } cal;
};
extern drone_state kafenv;

//UTILITY FUNCTIONS___________________________________________________________________________________________
memory kaf_reset();

#endif
