#include "../core/firmware.h"
#include "../core/flight.h"
#include "../auxilary/common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include <Wire.h>
#endif

#define MPU_I2C                                0x68
#define AK_I2C                                 0x0C
#define MPU_READ_SIZE                            14
#define AK_READ_SIZE                              8    //ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2
#define MPU_READ_ADDRESS                       0x3B
#define AK_READ_ADDRESS                        0x02    //ST1 (DRDY) - burst read must end at ST2 to latch the next sample
#define AK_ST1_DRDY                            0x01
#define AK_ST2_HOFL                            0x08
#define MPU_ACCEL_RANGE                        0x08     //0x00              0x08             0x10              0x18
#define MPU_GYRO_RANGE                         0x08     //0x00              0x08             0x10              0x18
#define MPU_ACCEL_SCALE 9.81                /  8192.0   //2g       16384.0, 4g       8192.0, 8g        4096.0, 16g,      2048.0
#define MPU_GYRO_SCALE  3.14159265359 / 180 /    65.536 //250deg/s 131.072, 500deg/s 65.536, 1000deg/s 32.768, 2000deg/s 16.384
#define AK_MAG_SCALE                 4912.0 / 32760.0   //

extern FLIGHT_BUFFERTYPE;

static struct {
  struct {                       //PERSISTENT MAGNETOMETER CALIBRATION (hard/soft-iron matrix, saved to disk)
    //Fit via tests/magnetometer_calibration.py against raw_mag_log_v2.txt - calibrated
    //magnitude mean=1.00, std=0.08 (~8% relative spread) across the sweep.
    //IMPORTANT: the DPRINTF below logs magInput AFTER this matrix is applied - if you ever need to
    //recollect calibration data again in the future, reset A/b to identity/zero first, or you'll be
    //fitting an ellipsoid to already-calibrated (nearly spherical) data instead of raw data.
    double A[3][3] = {
      {  0.003722,  0.000013,  0.000415 },
      {  0.000013,  0.004297, -0.000266 },
      {  0.000415, -0.000266,  0.004518 }
    };
    double b[3] = {
       173.166221,
       165.081628,
      -420.252058
    };
  } magCal;
  bool imuworking = false;
  bool magworking = false;
  unsigned int mpuMissCount = 0;
  unsigned int magMissCount = 0;
  unsigned char imuBytes[MPU_READ_SIZE];
  unsigned char magBytes[AK_READ_SIZE];
} mpu;

static unsigned char i2cSendByte( unsigned char address, unsigned char reg, unsigned char value ) {
  Wire.beginTransmission( address );
  Wire.write( reg );
  Wire.write( value );
  return Wire.endTransmission( true );
}

void peripheral_mpu9250Loop() {
  static unsigned long lastImuPrint = 0;
  const bool imuPrintDue = millis() - lastImuPrint > 1000;
  if( mpu.imuworking ) {
    Wire.beginTransmission( MPU_I2C );
    Wire.write( MPU_READ_ADDRESS );
    //full STOP (not a repeated-START) between the register-address write and the data read - WiFi's
    //internal driver tasks can delay the CPU right at that transition long enough to break a repeated
    //START's timing, causing requestFrom() to silently return 0 bytes once WiFi is active
    unsigned char endResult = Wire.endTransmission( true );
    unsigned char reqResult = Wire.requestFrom( MPU_I2C, MPU_READ_SIZE, true );
    if( reqResult == MPU_READ_SIZE ) {
      Wire.readBytes( mpu.imuBytes, MPU_READ_SIZE );
      ITRVEC3( q ) FLIGHT_BUFFER.accelInput.f[q] = (signed short)( mpu.imuBytes[ q * 2 ] << 8 | mpu.imuBytes[ q * 2 + 1 ] );
      FLIGHT_BUFFER.accelUpdate = true;
      ITRVEC3( q ) FLIGHT_BUFFER.gyroInput.f[q] = (signed short)( mpu.imuBytes[ q * 2 + 8 ] << 8 | mpu.imuBytes[ q * 2 + 9 ] );
      FLIGHT_BUFFER.gyroUpdate = true;
      FLIGHT_BUFFER.accelInput.x *= -1;
      FLIGHT_BUFFER.accelInput.z *= -1;
      FLIGHT_BUFFER.gyroInput.x  *= -1;
      FLIGHT_BUFFER.gyroInput.z  *= -1;
      mpu.mpuMissCount = 0;
      //DEBUG logging throttled to ~1/s - see periph_freertos.cpp for why
      if( imuPrintDue ) {
        lastImuPrint = millis();
        DPRINTF( "[P] MPU9250 Accelerometer: Value=[ %.3f, %.3f, %.3f ]\n", FLIGHT_BUFFER.accelInput.x,
            FLIGHT_BUFFER.accelInput.y, FLIGHT_BUFFER.accelInput.z );
        DPRINTF( "[P] MPU9250 Gyroscope: Value=[ %.3f, %.3f, %.3f ]\n", FLIGHT_BUFFER.gyroInput.x,
            FLIGHT_BUFFER.gyroInput.y, FLIGHT_BUFFER.gyroInput.z );
      }
    } else if( ++mpu.mpuMissCount > 50 ) {
      mpu.imuworking = false;
      //single explicit log at the moment reads give up, capturing the actual I2C result codes instead
      //of failing silently
      DPRINTF( "[P] MPU9250 DEBUG: imuworking gave up after %u misses, last endTransmission=%u, requestFrom=%u (want %u)\n",
          mpu.mpuMissCount, endResult, reqResult, MPU_READ_SIZE );
    }
  } else {
    //DEBUG: once the real read sequence has given up, probe once/sec with the simplest possible I2C
    //transaction (address-only, no register read) to tell whether the bus itself is dead after full
    //firmware startup, or whether it's specifically the register-read sequence above that's broken -
    //a bare Wire.begin() sketch was already confirmed to read this exact chip fine in isolation.
    if( imuPrintDue ) {
      lastImuPrint = millis();
      Wire.beginTransmission( MPU_I2C );
      unsigned char mpuProbe = Wire.endTransmission();
      Wire.beginTransmission( AK_I2C );
      unsigned char magProbe = Wire.endTransmission();
      DPRINTF( "[P] MPU9250 DEBUG: post-failure I2C probe MPU=%u, MAG=%u (0=ack, nonzero=no response)\n", mpuProbe, magProbe );
    }
    ITRVEC3( q ) FLIGHT_BUFFER.accelInput.f[q] = 0;
    FLIGHT_BUFFER.accelUpdate = false;
    ITRVEC3( q ) FLIGHT_BUFFER.gyroInput.f[q] = 0;
    FLIGHT_BUFFER.gyroUpdate = false;
  }
  if( mpu.magworking ) {
    Wire.beginTransmission( AK_I2C );
    Wire.write( AK_READ_ADDRESS );
    Wire.endTransmission( true ); //full STOP - see comment on the MPU9250 read above
    if( Wire.requestFrom( AK_I2C, AK_READ_SIZE, true ) == AK_READ_SIZE ) {
      Wire.readBytes( mpu.magBytes, AK_READ_SIZE );
      //discard stale (not DRDY) or overflowed (HOFL) samples rather than treat them as valid data
      if( ( mpu.magBytes[0] & AK_ST1_DRDY ) && !( mpu.magBytes[7] & AK_ST2_HOFL ) ) {
        signed short magAxisRaw[3];
        // AK8963 data registers are little-endian: HXL is followed by HXH. Data starts at index 1 (index 0 is ST1).
        ITRVEC3( q ) magAxisRaw[q] = (signed short)( mpu.magBytes[ q * 2 + 2 ] << 8 | mpu.magBytes[ q * 2 + 1 ] );
        //the AK8963 magnetometer die has its own axis frame relative to the MPU9250 accel/gyro die
        //(X/Y swapped, Z inverted); combined here with the same board-mounting correction applied to accel/gyro
        FLIGHT_BUFFER.magInput.x = -magAxisRaw[1];
        FLIGHT_BUFFER.magInput.y =  magAxisRaw[0];
        FLIGHT_BUFFER.magInput.z =  magAxisRaw[2];
        //apply calibration matrix: calibrated = A * ( raw - b )
        double magRaw[3];
        ITRVEC3( q ) magRaw[q] = FLIGHT_BUFFER.magInput.f[q] - mpu.magCal.b[q];
        ITRVEC3( q ) FLIGHT_BUFFER.magInput.f[q] = mpu.magCal.A[q][0] * magRaw[0] + mpu.magCal.A[q][1] * magRaw[1] + mpu.magCal.A[q][2] * magRaw[2];
        FLIGHT_BUFFER.magUpdate = true;
        mpu.magMissCount = 0;
        //DEBUG logging throttled to ~1/s - see periph_freertos.cpp for why
        if( imuPrintDue ) {
          DPRINTF( "[P] AK8963 Magnetometer: Value=[ %.3f, %.3f, %.3f ]\n", FLIGHT_BUFFER.magInput.x,
              FLIGHT_BUFFER.magInput.y, FLIGHT_BUFFER.magInput.z );
        }
      }
    } else if( ++mpu.magMissCount > 50 ) {
      mpu.magworking = false;
    }
  }
}

void peripheral_mpu9250Init() {
  firmware_registerPeripheral( { "mpu9250", sizeof( mpu.magCal ), sizeof( mpu ), &mpu, &peripheral_mpu9250Init, &peripheral_mpu9250Loop } );
  DPRINTF( "[P] Initializing MPU9250\n" );
  mpu.mpuMissCount = 0;
  //init wire
  Wire.begin( MPU_SDA, MPU_SCL );
  Wire.setTimeOut( 50 );
  //write a reset command to imu
  if( mpu.imuworking = i2cSendByte( MPU_I2C, 0x6B, 0x02 ) == 0 ) {
    delay( 20 );
    i2cSendByte( MPU_I2C, 0x1C, MPU_ACCEL_RANGE );//configure accelerometer
    i2cSendByte( MPU_I2C, 0x1B, MPU_GYRO_RANGE );//configure gyroscope
    i2cSendByte( MPU_I2C, 0x6A, 0x00 );//disable i2c master
    i2cSendByte( MPU_I2C, 0x37, 0x02 );//enable i2c bypass
  }
  DPRINTF( "[P] MPU Success Status: %s\n", mpu.imuworking ? "Yes" : "No" );
  //write a reset command to magnetometer
  if( mpu.magworking = i2cSendByte( AK_I2C, 0x0B, 0x01 ) == 0 ) {
    delay( 20 );
    i2cSendByte( AK_I2C, 0x0A, 0b00010110 );//set to continuous measurement
  }
  DPRINTF( "[P] AK Success Status: %s\n", mpu.magworking ? "Yes" : "No" );
  //set gains
  ITRVEC3( i ) kafenv.cal.accelfilt[i].gain = MPU_ACCEL_SCALE;
  kafenv.cal.accelfilt[2].ofst /= MPU_ACCEL_SCALE;
  ITRVEC3( i ) kafenv.cal.gyrofilt[i].gain = MPU_GYRO_SCALE;
  ITRVEC3( i ) kafenv.cal.magfilt[i].gain = AK_MAG_SCALE;
}

//Pre-flight sensor status accessors (commander.cpp's sensorStatus telemetry bitfield) - plain reads of
//these two bools, no synchronization: peripheral_mpu9250Loop() runs on the flight task core, these are
//read from the com task core in commander_step(), but single-byte bool reads/writes are atomic on this
//architecture and this is telemetry only, not control logic, matching this codebase's existing convention
//for cross-core telemetry reads (e.g. kafenv.state.x).
bool peripheral_mpu9250Working() {
  return mpu.imuworking;
}

bool peripheral_mpu9250MagWorking() {
  return mpu.magworking;
}
