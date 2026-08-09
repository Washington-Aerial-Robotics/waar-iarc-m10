/*  
 *  Author: Kent Fukuda
 *  KAF Drone: Flight Software Code
 *  Edit Date: April 29th, 2026
 *  Version: 20260420
 *  
 *  kaf_drone.h
 *  Drone State Information Header File
 */

//CONFIGURATION_______________________________________________________________________________________________
#ifndef KAF_DRONE_CONFIG
#define KAF_DRONE_CONFIG
#ifdef _MSC_VER
#define ALT_DEFINE 1
int printf( const char* format, ... );
#define PRINTF printf
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

//Common macros for core flight software code
#define MAXBYTE 255
#define NULLPTR 0
#define STDBYTE unsigned char
#define PERSIST 1
#define DPRINTF( ... ) PRINTF( __VA_ARGS__ )
#define FLTSYNC while( kafenv.info.triggerLock != MAXBYTE ) { }
#define FPARLEN( F ) ( sizeof( F ) / sizeof( float ) )
#define FPFILL0( N, F ) for( unsigned char N = 0; N < FPARLEN( F ); N++ ) F[N] = 0
#define ITRVEC3( N ) for( unsigned char N = 0; N < 3; N++ )
#define FPGUARD( N, V ) if( !isfinite( N ) ) N = V

//STRUCTURES__________________________________________________________________________________________________

//coordinate and gain value definitions
union coordinate {
  struct {                     //STANDARD COORDINATE INFORMATION
    float x;                   //Position, positive direction is eastwards in world, right in body coordinates
    float y;                   //Position, positive direction is northwards in world, front in body coordinates
    float z;                   //Position, positive direction is up in world, up in body coordinates
  };
  struct {                     //PID GAIN INFORMATION
    float Kp;                  //Proportional PID gain value
    float Ki;                  //Integral PID gain value
    float Kd;                  //Derivative PID gain value
  };
  struct {                     //ALPHA FILTER INFORMATION
    float gain;                //Alpha filter DC gain, A value for Out=A*(In+B)
    float ofst;                //Alpha filter offset value, B value for Out=A*(In+B)
    float stdv;                //Alpha filter alpha standard deviation modifier, equivalent to pi^2*stdev^6
  };
  float f[3];                  //Vector array form of coordinate structure
};

//struct for describing firmware memory pages
struct peripheral {
  char name[12];               //Name of memory page
  unsigned short persist = 0;  //Length of memory to save to disk
  unsigned short length = 0;   //Length of memory page
  void* memory = NULLPTR;      //Address of start of memory page
  void( *init )() = NULLPTR;   //Address of the initialization function
  void( *loop )() = NULLPTR;   //Address of the looping function
};

//GLOBAL VARIABLES____________________________________________________________________________________________

//struct describing the state of an individual drone, including its identification, state, and commands
struct drone_state {
  struct droneinfo {           //IDENTIFICATION INFORMATION
    STDBYTE deviceID;          //Character designating a unique ID of a drone
    STDBYTE flightMode;        //Current flight mode status of the drone
    STDBYTE triggerLock;       //Device software trigger for getting a thread lock
    bool actuation;            //Configuration for if motor actuation is enabled or not
    unsigned int version;      //Flight software version
    float battery;             //Percentage amount of battery left
  } info;
  struct stateestimate {       //STATE ESTIMATION
    coordinate x;              //Position in m in the world frame
    coordinate v;              //Velocity in m/s in the world frame
    coordinate q;              //Yaw q.z in rad world frame, quaternion elements q.x, q.y in the body frame
    coordinate w;              //Angular rate coordinate in rad/s in the body frame
  } state;
  struct {                     //TRAJECTORY COMMANDS
    float setpoints[22];       //Setpoint float point data representing coordinates, values, or trajectories
    float motors[4];           //Variable to throttle the target PWM voltage as an unitless scalar 0<=1
  } cmd;
  struct calibrations {        //CALIBRATION DATA
    float anglealpha;          //Alpha filter value for merging predicted attitudes (rad/rad)
    float positionalpha;       //Time constant value for merging predicted position estimates (rad/s)
    float gravitation;         //Gravitational acceleration value measured from accelerometer (m/s2)
    coordinate gyrofilt[3];    //NLA values for the gyroscope alpha filter (bytes -> rad/s)
    coordinate accelfilt[3];   //NLA values for the accelerometer alpha filter (bytes -> m/s2)
    coordinate magfilt[3];     //NLA values for the magnetometer alpha filter (bytes -> rad)
    coordinate sensefilt[8];   //NLA values for other sensor alpha filter
    coordinate xpid;           //PID values for the position controller (m, s)
    coordinate vpid;           //PID values for the velocity controller (m/s, s)
    coordinate apid;           //PID values for the acceleration controller (m/s2, s)
    coordinate qpid;           //PID values for the attitude controller (rad, s)
    coordinate wpid[3];        //PID values for the angular rate controller (rad/s, s)
  } cal;
};
extern drone_state kafenv;     //Common state information about the drone

//UTILITY FUNCTIONS___________________________________________________________________________________________

// Description:      Resets the kafenv drone state struct to default values and returns memory information
// Input Parameters: N/A
// Return:           Peripheral struct describing the characteristics of the kafenv drone state struct
peripheral kaf_reset();

#endif
