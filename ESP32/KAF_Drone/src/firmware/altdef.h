struct {
  int ( *available )();
  char ( *read )();
  void ( *readBytes )( char*, int );
  void ( *write )( char*, int );
  void ( *printf )( ... );
  char ( *begin )( unsigned int );
  char ( *setTimeout )( unsigned int );
} Serial;
#define memcpy( A, B, C )
#define delay( A )
#define millis() 0
#define strcpy( A, B )
#define IpAddress( A, B, C, D ) 0
#define WiFiServer( A ) {}
struct WiFiServer {
  void ( *begin )();
  WiFiClient ( *accept )();
};
struct {
  void ( *begin )( char*, char* );
  int ( *status )();
  int ( *disconnect )();
  void ( *softAP )( char*, char*, int, int, int );
  void ( *config )( int );
  void ( *softAPConfig )( int, int, int );
  void ( *mode )( int );
} WiFi;
struct WiFiClient {
  int ( *available )();
  void ( *readBytes )( char*, int );
  void ( *write )( char*, int );
  char ( *setTimeout )( unsigned int );
  void ( *flush )();
  bool ( *connected )();
};
struct Servo {
  void ( *attach )( int );
  void ( *writeMicroseconds )( int );
};
struct {
  void( *begin )( int, int );
  void( *setTimeOut )( int );
  void( *beginTransmission )( int );
  void( *write )( int );
  int( *endTransmission )( bool );
  int( *requestFrom )( int, int, bool );
  void( *readBytes )( unsigned char*, int );
} Wire;
struct {
  void( *begin )( int );
  void( *readBytes )( int, void*, int );
  void( *writeBytes )( int, void*, int );
  void( *commit )();
} EEPROM;
#define WIFI_AP 0 
#define WIFI_STA 0
#define IPAddress( A, B, C, D ) 0
#define pinMode( A, B )
#define WL_CONNECTED 1
#define vTaskDelay
#define xTaskCreatePinnedToCore
#define NULL 0
#define NAN           0
#define uint32_t unsigned int
struct {
  void( *restart )();
  void( *flashRead )( int, uint32_t*, int );
  void( *flashEraseSector )( int );
  void( *flashWrite )( int, uint32_t*, int );
} ESP;
#define memset( A, B, C )
#define rand()            0
struct BME280 {
  float( *readFloatHumidity )();
  float( *readFloatPressure )();
  float( *readTempC )();
  bool( *beginI2C )();
};