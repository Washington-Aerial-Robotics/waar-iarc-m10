struct {
  int ( *available )();
  char ( *read )();
  void ( *readBytes )( char*, int );
  void ( *write )( char*, int );
  void ( *printf )( ... );
  char ( *begin )( unsigned int );
  char ( *setTimeout )( unsigned int );
} Serial;
#define NAN 0.0F
#define isfinite( N ) true
double cos ( double num );
static float powf( float base, float exponent );
static long strtol( const char* str, char** str_end, int base );
static float strtof ( const char* str, char** str_end );
static double strtod ( const char* str, char** str_end );
static void* memset( void* dest, int ch, size_t count );
static void* memcpy( void* dest, const void* src, size_t count );
static int memcmp( const void* lhs, const void* rhs, size_t count );
static size_t strlen( const char* str );
static int rand();
#define delay( A )
#define millis() 0
#define strcpy( A, B )
#define strncpy( A, B, C )
#define IpAddress( A, B, C, D ) 0
#define WiFiServer( A ) {}
#define String char*
#define snprintf( A, B, C ) 0
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
struct {
  void ( *begin )( unsigned long, unsigned int, char, char );
  int ( *available )();
  int ( *read )();
  size_t ( *readBytes )(char *buffer, size_t length);
} Serial1;
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
#define DSHOT150 0
#define DSHOT300 1
#define DSHOT600 2
#define DSHOT1200 3
#define uint16_t unsigned short
struct DShotRMT {
  DShotRMT( uint16_t, int, bool );
  void ( *begin )();
  void ( *sendThrottlePercent )( float );
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
#define SERIAL_8N1 0
#define WIFI_AP 0 
#define WIFI_STA 0
#define IPAddress( A, B, C, D ) 0
#define pinMode( A, B )
#define digitalWrite( A, B )
#define WL_CONNECTED 1
#define vTaskDelay
#define xTaskCreatePinnedToCore
#define NULL 0
#define uint32_t unsigned int
struct {
  void( *restart )();
  void( *flashRead )( int, uint32_t*, int );
  void( *flashEraseSector )( int );
  void( *flashWrite )( int, uint32_t*, int );
} ESP;
struct BME280 {
  float( *readFloatHumidity )();
  float( *readFloatPressure )();
  float( *readTempC )();
  bool( *beginI2C )();
};
struct HTTPUpload {
  int status;
  int totalSize;
  int currentSize;
  unsigned char buf[2048];
};
struct WebServer {
  void( *begin )();
  void( *handleClient )();
  void( *on )( const char*, int, void( *p )(), ... );
  void( *send_P )( int, const char*, const char*, ... );
  void( *send )( int, String, String );
  int( *args )();
  bool( *hasArg )( const char* );
  String( *arg )( const char* );
  HTTPUpload& ( *upload )();
};
#define UPLOAD_FILE_START 0
#define UPLOAD_FILE_WRITE 1
#define UPLOAD_FILE_END 2
#define UPLOAD_FILE_ABORTED 3
#define WebServer( A ) {}
#define HTTP_POST 0
#define HTTP_GET  0