#ifdef __MINGW64__
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "stdio.h"
#define KAF_DRONE_CONFIG
#include "src/kaf_quadcopter_code.h"

using namespace matlab::data;
using matlab::mex::ArgumentList;

#define BRIDGE_COM_METHOD 0
#define COM_SEND_QUEUE_LENGTH 10
#define COM_BUFFER_LENGTH 400
static unsigned short receiveLength = 0;
static unsigned char receive[COM_BUFFER_LENGTH];
static unsigned char sendLength = 0;
static unsigned short sendLengths[COM_SEND_QUEUE_LENGTH];
static unsigned char send[COM_SEND_QUEUE_LENGTH][COM_BUFFER_LENGTH];

static void sending( void* ptr, unsigned short length ) {
    if( sendLength < COM_SEND_QUEUE_LENGTH && length <= COM_BUFFER_LENGTH ) {
        sendLengths[ sendLength ] = length;
        memcpy( send[ sendLength++ ], ptr, length );
    }
}

static radio bridge_radio = { 0, BRIDGE_COM_METHOD, true, true, &receive, []() { 
    unsigned short recv = receiveLength;
    receiveLength = 0;
    return recv; 
}, &sending, &sending };

static imu bridge_sensor = { 0, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, false, false, false };

class MexFunction  : public matlab::mex::Function {
public:
    void operator()( matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs ) {
        TypedArray<char16_t> command = inputs[0];
        ArrayFactory factory;
        switch( ( unsigned char )command[0] ) {
            case 'R' : {
                firmware_reset();
                TypedArray<uint8_t> droneID = inputs[1];
                kafenv.info.deviceID = droneID[0];
                outputs[0] = factory.createScalar( true );
                break;
            };
            case 'F' : {
                TypedArray<double> timeStep = inputs[1];
                TypedArray<double> sensors = inputs[2];
                TypedArray<char16_t> senseMask = inputs[3];
                bridge_sensor.timeStep = (float)timeStep[0];
                bridge_sensor.accelInput.x = (float)sensors[0];
                bridge_sensor.accelInput.y = (float)sensors[1];
                bridge_sensor.accelInput.z = (float)sensors[2];
                bridge_sensor.gyroInput.x = (float)sensors[3];
                bridge_sensor.gyroInput.y = (float)sensors[4];
                bridge_sensor.gyroInput.z = (float)sensors[5];
                bridge_sensor.magInput.x = (float)sensors[6];
                bridge_sensor.magInput.y = (float)sensors[7];
                bridge_sensor.magInput.z = (float)sensors[8];
                bridge_sensor.accelUpdate = senseMask[0] == 'T';
                bridge_sensor.gyroUpdate = senseMask[1] == 'T';
                bridge_sensor.magUpdate = senseMask[2] == 'T';
                flight_step( &bridge_sensor );
                TypedArray<double> motor = factory.createArray<double>( { 4, 1 } );
                for( unsigned char i = 0; i < 4; i++ ) motor[i] = kafenv.cmd.motors[i];
                TypedArray<double> rotation = factory.createArray<double>( { 9, 1 } );
                float rotData[9];
                flight_rotationMatrix( rotData );
                for( unsigned char i = 0; i < 9; i++ ) rotation[i] = rotData[i];
                TypedArray<double> state = factory.createArray<double>( { 12, 1 } );
                state[0] = (double)kafenv.state.x.x;
                state[1] = (double)kafenv.state.x.y;
                state[2] = (double)kafenv.state.x.z;
                state[3] = (double)kafenv.state.v.x;
                state[4] = (double)kafenv.state.v.y;
                state[5] = (double)kafenv.state.v.z;
                state[6] = (double)kafenv.state.q.x;
                state[7] = (double)kafenv.state.q.y;
                state[8] = (double)kafenv.state.q.z;
                state[9] = (double)kafenv.state.w.x;
                state[10] = (double)kafenv.state.w.y;
                state[11] = (double)kafenv.state.w.z;
                outputs[0] = motor;
                outputs[1] = rotation;
                outputs[2] = state;
                break;
            };
            case 'C' : {
                TypedArray<double> time = inputs[1];
                TypedArray<uint8_t> inPacket = inputs[2];
                sendLength = 0;
                receiveLength = ( unsigned short )inPacket.getDimensions()[1];
                receiveLength = receiveLength > COM_BUFFER_LENGTH ? COM_BUFFER_LENGTH : receiveLength;
                for( int i = 0; i < receiveLength; i++ ) {
                    receive[i] = inPacket[i];
                }
                bridge_radio.currentTime = ( unsigned long )time[0];
                com_step( &bridge_radio );
                CellArray outPackets = factory.createCellArray( { 1, (size_t)sendLength } );
                for( int i = 0; i < sendLength; i++ ) {
                    TypedArray<uint8_t> outPacket = factory.createArray<uint8_t>( { 1, (size_t)sendLengths[i] } );
                    for( int j = 0; j < sendLengths[i]; j++ ) {
                        outPacket[j] = send[i][j];
                    }
                    outPackets[i] = outPacket;
                }
                outputs[0] = outPackets;
                break;
            };
            default : { }
        }
    }
};
#endif