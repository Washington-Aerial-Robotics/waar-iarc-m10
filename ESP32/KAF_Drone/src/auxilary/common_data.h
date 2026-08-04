#define DW_IRQ   34
#define DW_RST   27
#define DW_SS     4 
#define MPU_SDA  21
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