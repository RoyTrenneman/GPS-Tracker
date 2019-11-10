#define CONFIG_ESP32_ENABLE_COREDUMP_TO_FLASH 1
const char  apn[]      = "orange"; // Your APN
const char  gprsUser[] = "orange"; // User
const char  gprsPass[] = "orange"; // Password
const char  simPIN[]   = "0000"; // SIM card PIN code, if any
// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <Wire.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <TinyGPS.h>

// IP5306 register def
#define IP5306_ADDR          0X75
#define IP5306_REG_SYS_CTL0  0x00
#define IP5306_REG_SYS_CTL1  0x01
#define IP5306_REG_SYS_CTL2  0x02
#define IP5306_REG_READ0     0x70
#define IP5306_REG_READ1     0x71
#define IP5306_REG_READ3     0x78  
#define I2C_SDA              21
#define I2C_SCL              22

// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

#define GPS_TX               25
#define GPS_RX               33

#define GPS_POWER_ON         12

#include <PubSubClient.h> 
const char* mqttServer = "Your_Broker_MQTT_IP";                        
const int mqttPort = 23456;                                      
const char* mqttUser = "user";                                  
const char* mqttPassword = "*******";                          


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1
#define SerialGPS  Serial2

// Define the serial console for debug prints, if needed
//#define TINY_GSM_DEBUG SerialMon
#define DUMP_AT_COMMANDS
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient client2(client);
TinyGPS gps;
                                     
void startingMQTT() {

 // Starting MQTT                                           
 client2.setServer(mqttServer, mqttPort);                    
// client2.setCallback(callback);                              
}

void reconnect() {                                                                           
  yield();                                                                                   
  int a = 1 ;                                                                                
  //loop until connected                                                                     
      while (!client2.connected()) {                                                          
          yield();                                                                           
          if (client2.connect("ESP32", mqttUser, mqttPassword)) {                     
              client2.subscribe("sensor/#");
              Serial.println("mqtt client connected!");                                                
            } else {
              Serial.println("mqtt failed");
              delay(5000);                                                                   
            }                                                                                
            a++ ;                                                                            
          if (a > 5){                                                                        
             break ; // in order to avoid an infernal loop                                   
          }                                                                                
      }                                                                                    
}                                                                                            

void  setPowerBoostKeepOn(){

  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  Wire.endTransmission() ;
  delay(10);
  
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL1);
  Wire.write(0x1D);
  Wire.endTransmission() ;
  delay(10);
  
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL2);
  Wire.write(0x64);
  Wire.endTransmission();
  
}

void poweron_GPS() {
  digitalWrite(GPS_POWER_ON, HIGH);
}

void poweroff_GPS() {
  digitalWrite(GPS_POWER_ON, LOW);
}

void poweron_GSM() {
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  delay(3000);
}

void poweroff_gsm() {
  digitalWrite(MODEM_POWER_ON, LOW);
}

bool checkNetwork() {
 return (modem.isNetworkConnected());
}

bool connect_GSM() {
  
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    SerialMon.println(" fail");
    delay(10000);
    return false;
  }
  SerialMon.println(" OK");

  if (!modem.isNetworkConnected()) {
    return false;
  }
 
  SerialMon.println("Network connected");
  return true ; 
}


void GPS() {
   while (SerialGPS.available()) {
   Serial.write(SerialGPS.read());
 } 
}


void StartGPRS() {

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println("GPRS success");

  if (modem.isGprsConnected()) {
    Serial.println("GPRS connected");
 }
}

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  
  // Start I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  setPowerBoostKeepOn();
  
  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(GPS_POWER_ON, OUTPUT);
  
  poweron_GPS();

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);
  
  // Set GPS module baud rate and UART pins
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);
  
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else {
    sprintf(sz, "%ld", val);
  }
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
  
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}


void loop() {

//GPS();
 static int a = 0;
 a++;
  unsigned long age, chars = 0;
  unsigned short sentences = 0, failed = 0;
  float flat, flon;

  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  Serial.println();
  Serial.println("***********");
  Serial.println(chars);
  Serial.println(sentences);
  Serial.println(failed);
  Serial.println("***********");
 
  smartdelay(1000);
 if (a > 10) {
  Serial.println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Chars Sentences Checksum");
  Serial.println("          (deg)     (deg)      Age                      Age  (m)    RX    RX         Fail");
  a = 0 ;
 }
 
 if (sentences > 50 ) {
  poweroff_GPS();
  Serial.println("We have now enough data, send it"); 
  int count = 0 ; 
  //Start GSM module
  poweron_GSM();
  while (!connect_GSM()){
  count++;
   if (count > 5) {
    Serial.println("Failed to connect GSM, Going to deep sleep...");
    delay(1000); 
    ESP.deepSleep(60 * 1000000);   
   }  
  };
  delay(2);
  StartGPRS();

  //Start MQTT client
  startingMQTT(); 
  
  count = 0 ; 
  while (!client2.connected()) {         
    reconnect();
    count++;
    if (count > 5) {
     Serial.println("Failed to connect MQTT client, Going to deep sleep...");
     delay(1000); 
     ESP.deepSleep(60 * 1000000);   
    }                
  }
  Serial.println("Sending GPS coordinnates");                                
  String NMEA = "\{\"longitude\":" + String(flon, 7) + ",\"latitude\":" + String(flat, 7) + "}" ; 
  client2.publish("GPS/location", String(NMEA).c_str() ) ;
  delay(10000);
  Serial.println("Poweroff GSM...");
  poweroff_gsm();
  delay(5000);
  Serial.println("Going to deep sleep...");
  ESP.deepSleep(60 * 1000000);   
 }

}

