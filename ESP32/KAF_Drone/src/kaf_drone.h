//PREAMBLE____________________________________________________________________________________________________
#ifdef _MSC_VER
#define ALT_DEFINE 1
#define PACKED 
#else
#define ALT_DEFINE 0
#define PACKED __attribute__((__packed__))
#endif
#ifndef NULL
#define NULL 0
#endif

//CONFIGURATIONS______________________________________________________________________________________________
//define once
#ifndef ESP32_PNT_DRONE
#define ESP32_PNT_DRONE
//maximum number of motors/devices
#define MOTOR_COUNT               4
#define DEVICE_COUNT              6
#define PERIPHERAL_COUNT         10
#define SEND_QUEUE_COUNT          3
//communication and network
#define NETWORK_DEVICE_TIMEOUT 8000
#define PLACEHOLDER_ID            0
#define INVALID_DEVICE_IDX     0xFF
#define INVALID_NETWORK_METHOD    0
//debug control
#define DEBUGPRINT                1
#define FIRMWARE_ENABLE           1
#define ALLOW_CUSTOM_FIRMWARE     1

//UTILITY MACROS______________________________________________________________________________________________
#define KF_REGVAR( P, T, N ) printf( "%c,%s,%04x,%04x\n", T, N, sizeof( P )                         \
                 (unsigned short)( ( (char*)( (void*)&( P ) ) ) - ( (char*)( (void*)&kafenv ) ) ) )
#define KF_VEC3( N, C ) for( int N = 0; N < 3; N++ ) C
#define KF_VECN( N, A, C ) for( int N = 0; N < A; N++ ) C
#define KF_BOUND( V, LB, UB ) V = V > UB ? UB : ( V < LB ? LB : V )
#define KF_FWD( D, S ) ( (fwd*)(void*)&kafenv.c.D.data.all[ ( S > sizeof( kafenv.c.D.data.all )     \
                       ? sizeof( kafenv.c.rx.data.all ) : S ) - sizeof( fwd ) ] )

//STRUCTURES__________________________________________________________________________________________________
//coordinate definition
typedef union coordinate {
  struct {
    float x;
    float y;
    float z;
    float stdev;
  };
  float f[4];
};

//simple filter definition
typedef struct measfilter {
  float gain;
  float offset;
  float frequency;
  float uncertainity;
  float stateX1;
  float stateX2;
  unsigned int measCount;
  float measTotal;
  float measDev;
};

//pid definition
typedef PACKED struct pid {
  float Kf;
  float Kp;
  float Ki;
  float Kd;
  float intlimit;
  float outlimit;
  float derlimit;
  float modula;
  float integ;
  float deriv;
  float error;
  float desired;
  float prevmeas;
  float lowpass[5];
};

//forwarding struct definition
typedef PACKED struct fwd {
  unsigned char targetID;
  unsigned char originID;
};

//full state struct definition
typedef PACKED struct state {
  coordinate x;//position
  coordinate v;//velocity
  coordinate t;//orientation
  coordinate w;//angular velocity
};

//GLOBAL VARIABLES____________________________________________________________________________________________
struct global_variables {
  struct {//DRONE STATE DATA
    unsigned char deviceID = 'U';                   //Character designating a unique ID of a drone
    unsigned char flightMode = 0;                   //
    unsigned int version = 0x00901900;              //
    state stateEstimate;                            //State estimate of the drone in the global reference
                                                    //frame. Position is cartesian, orientation is axial.
  } u;                                              //Drone data
  struct {//DRONE CONTROL DATA
    float motorSetpoint[MOTOR_COUNT];               //
    float posSetpoint[3] = { 0, 0, 0 };             //
    float cascadeSetpoint[3];                       //
    pid positionPID[3];                             //
    pid velocityPID[3];                             //
    pid attitudePID[3];                             //
    pid attiratePID[3];                             //
  } s;                                              //
  struct {//FLIGHT DATA
    unsigned long currentTime = 0;                  //Timestamp of flight sensor data acquisition time in ms
    unsigned int timeStep = 0;                      //Time interval in us between the last two flight ticks
    bool motorsEnabled = false;                     //
    float motorOutput[MOTOR_COUNT];                 //Variable to write to in order to set the target PWM
                                                    //voltage, specified in an unitless scalar 0<=1
    coordinate accelInput = { 0, 0, 0, 1e10 };      //
    coordinate gyroInput = { 0, 0, 0, 1e10 };       //
    coordinate magInput = { 0, 0, 0, 1e10 };        //
    coordinate gpsInput = { 0, 0, 0, 1e10 };        //
    coordinate baroInput = { 0, 0, 0, 1e10 };       //
    measfilter accelCalib[3];                       //
    measfilter gyroCalib[3];                        //
    measfilter magCalib[3];                         //
    measfilter gpsCalib[2];                         //
    measfilter baroCalib[1];                        //
  } f;                                              //Flight peripheral data
  struct {//NETWORK DATA
    unsigned long currentTime = 0;                  //Timestamp of communication received in ms
    char networkName[32] = "default";               //Specifies the name of the WiFi network to connect to
                                                    //upon initial setup or when resetConnection is raised
    char networkPassword[25] = "default";           //Specifies the password of the WiFi network
    char networkAddress[127] = "disconnected";      //If connected, this records the IP address of the drone
    bool attemptReconnect = true;                   //
    bool immediateForwardingReturn = false;         //
    unsigned short comTaskDelay = 0;                //
    unsigned char comStepSkips = 0;                 //
    unsigned char receiveMethod = 0;                //
    unsigned char receiveSize = 0;                  //
    unsigned char replySize = 0;                    //
    unsigned char nextMessageID = 0;                //
    struct {
      unsigned char remainingAttempts = 0;          //
      unsigned char deliveryMethod = 0;             //
      unsigned char packetSize = 0;                 //
      void( *handlingFunction )( bool );            //
    } sendPackets[SEND_QUEUE_COUNT];                //
    unsigned int deviceCount = 0;                   //Number of peer drones observed by this drone
    unsigned char deviceLookup[0xFF];               //Drone ID to devices struct array index lookup
    unsigned char deviceIndices[DEVICE_COUNT];      //
    struct {
      unsigned char deviceID;                       //Unique id of peer drone
      unsigned char liason;                         //
      unsigned char nodeOrder;                      //
      unsigned long lastSeen;                       //Last observation of peer drone in ms
      unsigned long lastRanging;                    //Last successful ranging with peer drone in ms
      coordinate position;                          //Position estimate in m of peer drone
      float distance;                               //Distance in m between peer drone and this drone
    } devices[DEVICE_COUNT];                        //Structure for each peer that the drone can observe
  } n;
  struct {//COMMUNICATION DATA
    union {
      PACKED struct {
        unsigned char toID;                          //ID of the intended receiver of the packet
        unsigned char fromID;                        //ID of the sender of the communication packet
        unsigned char messageType;                   //Message type of the communication packet
        unsigned char messageID;                     //unique identifier for message of the same subject
        PACKED union {
          unsigned char all[248];
          unsigned char byte;
          unsigned char bytes[ sizeof( all ) - sizeof( fwd ) ];
          coordinate coord;
          state stateEst;
          fwd errorForward;
          PACKED struct {
            coordinate position;
          } ranging1;
          PACKED struct {
            unsigned int processingTime;
            coordinate position;
          } ranging2;
          PACKED struct {
            unsigned int processingTime;
            unsigned int timeOfFlight;
          } ranging3;
          PACKED struct {
            unsigned char deviceID[6];
            unsigned char nodeOrder[6];
            unsigned int timeLastSeen[6];
          } deviceList;
          PACKED struct {
            unsigned int startPos;
            unsigned char length;
            unsigned char bytes[241];
          } globalVarTransfer;
          PACKED struct {
            char networkName[20];
            char networkPassword[20];
          } setWiFi;
          PACKED struct {
            unsigned char method;
            unsigned char length;
            unsigned char message[244];
          } setFwdSend;
        };
      } data;                                       //Data representation of transmission data
      unsigned char raw[ sizeof( data ) ];          //Buffer representation of transmission data
    } rx, tx, sx[SEND_QUEUE_COUNT];                 //Structs for loading receive/send coms data
  } c;                                              //Communication data
  struct {
    unsigned char version = 99;                     //
    unsigned char trigger = 0;                      //
    void( *comTaskQueue )() = NULL;                 //
    void( *flightTaskQueue )() = NULL;              //
    struct  {
      char name[15] = "default";                    //
      bool enabled = false;                         //
      unsigned char loopType;                       //
      unsigned int dataSize = 0;                    //
      void( *initFunction )( int ) = NULL;          //
      void( *loopFunction )() = NULL;               //
      void( *auxLoopFunction )() = NULL;            //
      void* data = NULL;                            //
    } peripheral[PERIPHERAL_COUNT];                 //
  } p;                                              //Peripheral data
};
extern struct global_variables kafenv;

//FUNCTIONS DEFINITONS________________________________________________________________________________________
//kaf drone utility functions
void kaf_dumpmemory();
void kaf_matmul( float* mat, float* in, float* out, unsigned char size );
void kaf_filterreset( measfilter* filter );
void kaf_filtercalib( measfilter* filter, float measured );
float kaf_filterstep( measfilter* filter, float measured, float dt );
void kaf_pidreset( pid* pid );
float kaf_pidstep( pid* pid, float set, float meas, float dt  );
unsigned char kaf_getnextvalid( unsigned char indicesIndex );
unsigned char kaf_getdevice( unsigned char deviceID ); 
unsigned char kaf_hasdevice( unsigned char deviceID );
unsigned char kaf_getforwarding( unsigned char deviceID );
void kaf_handlecoms( bool doSend, bool deReply, unsigned char method, unsigned char len, void( *sendFunc )( unsigned char*, unsigned char ) );
unsigned char kaf_addcoms( unsigned char attempts, unsigned char method, unsigned char size, void( *handler )( bool ) );
//software functions
void communicationRespond();                                     //
void communicationRecord();                                      //Called to communication data
void communicationStep();                                        //
void controlsStep();                                             //Called every cycle to calculate states
void softwareInit();                                             //
//firmware functions
#if FIRMWARE_ENABLE
void firmwareSetup();                                            //Function call to start the drone firmware
#endif

#endif