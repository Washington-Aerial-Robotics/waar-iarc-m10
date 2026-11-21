#include "../core/firmware.h"
#include "../core/flight.h"
#include "../auxilary/common_data.h"

#if ALT_DEFINE
#include "altdef.h"
#else
#include <Arduino.h>
#include <Wire.h>
#endif

#define MPU_I2C                                0x68 // c
#define AK_I2C                                 0x0C // c
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

