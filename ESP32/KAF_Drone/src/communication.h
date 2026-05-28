#define COM_HEADER_LEN               4//length of header
#define COM_NULL                     0//reserved
#define COM_ID              0b00111111//actual message type id of the packet
#define COM_CMD             0b01000000//message type is command that requires a response (SET, REQUEST)
#define COM_FWD             0b10000000//forwarding bit indicating if packet should be forwarded
//3A-3F required communication implementations
#define COM_SUCCESS         (       0 | 0x3A )//,,
#define COM_FAILURE         (       0 | 0x3B )//,,
#define COM_ACKNOWLEDGED    (       0 | 0x3C )//,,
#define COM_TRIGGER         ( COM_CMD | 0x3C )//,COM_ACKNOWLEDGED,
#define COM_PONG            (       0 | 0x3D )//,,
#define COM_PING            ( COM_CMD | 0x3D )//,COM_PONG,
#define COM_RET_DEVICES     (       0 | 0x3E )//deviceList,,
#define COM_GET_DEVICES     ( COM_CMD | 0x3E )//byte,COM_REPLY_DEVICES,
#define COM_RET_STATE       (       0 | 0x3F )//coord,
#define COM_GET_STATE       ( COM_CMD | 0x3F )//,COM_REPLY_STATE,
//01-0F communication method specific messages
#define COM_RANGING_1       ( COM_CMD | 0x01 )//ranging1,COM_RANGING_2,UWB
#define COM_RANGING_2       ( COM_CMD | 0x02 )//ranging2,COM_RANGING_3,UWB
#define COM_RANGING_3       (       0 | 0x02 )//ranging3,,UWB
#define COM_SET_MEMORY      ( COM_CMD | 0x03 )//,COM_SUCCESS,SERIAL
#define COM_REQUEST_MEMORY  ( COM_CMD | 0x04 )//,COM_REPLY_MEMORY,SERIAL
#define COM_REPLY_MEMORY    (       0 | 0x04 )//,,SERIAL
#define COM_KILL            (       0 | 0x05 )//,,WIFI
//10-19 memory and peripheral related messages
#define COM_SET_GLOBAL      ( COM_CMD | 0x10 )//globalVarTransfer,COM_SUCCESS,
#define COM_REQUEST_GLOBAL  ( COM_CMD | 0x11 )//globalVarTransfer,COM_REPLY_GLOBAL,
#define COM_REPLY_GLOBAL    (       0 | 0x11 )//globalVarTransfer,
#define COM_SET_CALIB       (       0 | 0x12 )
#define COM_REQUEST_CALIB   ( COM_CMD | 0x13 )
#define COM_REPLY_CALIB     ( COM_CMD | 0x13 )
#define COM_SET_MEMORY      (       0 | 0x14 )
#define COM_REQUEST_MEMORY  ( COM_CMD | 0x15 )
#define COM_REPLY_MEMORY    ( COM_CMD | 0x15 )
#define COM_SET_PERIPH      ( COM_CMD | 0x16 )
#define COM_REQUEST_PERIPH  ( COM_CMD | 0x17 )
#define COM_REPLY_PERIPH    (       0 | 0x17 )
#define COM_SET_WIFI        ( COM_CMD | 0x18 )//.COM_
#define COM_REQUEST_WIFI    ( COM_CMD | 0x19 )//,COM_REPLY_WIFI_IP,
#define COM_REPLY_WIFI      (       0 | 0x19 )//bytes,,
#define COM_SET_RESETPERIPH ( COM_CMD | 0x1A )
#define COM_SET_FWDMSG      ( COM_CMD | 0x1C )//,COM_ACK,
//20-27 state estimate and positioning related messages
#define COM_SET_ST_EST      ( COM_CMD | 0x20 )//stateEst,COM_SUCCESS,
#define COM_REQUEST_ST_EST  ( COM_CMD | 0x21 )//,COM_REPLY_ST_EST,
#define COM_REPLY_ST_EST    (       0 | 0x21 )//stateEst,,
#define COM_SET_POS         ( COM_CMD | 0x22 )//coord,COM_SUCCESS,
#define COM_REQUEST_POS     ( COM_CMD | 0x23 )//,COM_REPLY_POS,
#define COM_REPLY_POS       (       0 | 0x23 )//coord,,
#define COM_SET_ATT         ( COM_CMD | 0x24 )//coord,COM_SUCCESS,
#define COM_REQUEST_ATT     ( COM_CMD | 0x25 )//setFwdSend,COM_REPLY_ATT,
#define COM_REPLY_ATT       (       0 | 0x25 )//coord,,
//28-2F flight software algortihm commands
#define COM_SET_CTRL_MODE   ( COM_CMD | 0x28 )//byte,COM_SUCCESS,
#define COM_SET_MOTOR_CMD   ( COM_CMD | 0x29 )//coord,COM_SUCCESS,
#define COM_SET_POS_CMD     ( COM_CMD | 0x2A )//coord,COM_SUCCESS,

#define NULL_MODE           0x00
#define CALIBRATION_MODE    0x01
#define MOTOR_SETPOINT_MODE 0x02
#define POS_SETPOINT_MODE   0x03
#define TRAJECTORY_MODE     0x04
#define FLIGHT_MODE_BOUNDS  0x05