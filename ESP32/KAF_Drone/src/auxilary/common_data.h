#define DW_IRQ   34
#define DW_RST   27
#define DW_SS     4  //verified against Makerfabs' own dw3000_board_config.h - this was correct originally
#define MPU_SDA  21  //not part of the DW3000's fixed pinout, safe to share with the ESP32 default I2C pins
#define MPU_SCL  22
#define UART1RX  16
#define UART1TX  17
#define ESC_PINS 25, 26, 32, 33

#define COMS_BUFFER       common_coms_buffer
#define FLIGHT_BUFFER     common_imu
#define SENSOR_BUFFER     common_sensor
#define COMS_BUFFERTYPE   char COMS_BUFFER[4096]
#define FLIGHT_BUFFERTYPE imu FLIGHT_BUFFER
#define SENSOR_BUFFERTYPE sensors SENSOR_BUFFER