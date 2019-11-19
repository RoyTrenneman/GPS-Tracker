#include "FS.h"
#include "SPIFFS.h"
#define CONFIG_ESP32_ENABLE_COREDUMP_TO_FLASH 1
const char  apn[]      = "orange"; // Your APN
const char  gprsUser[] = "orange"; // User
const char  gprsPass[] = "orange"; // Password
const char  simPIN[]   = "0000"; // SIM card PIN code, if any
char        msg[180]   = "";
bool        wifi_mode  = false ;
// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
const char* ssid = "Your Wifi SSID";
const char* password =  "***";

// Define PIN Wifi mode
#define PIN_WIFI 19

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

// GPS pin definitions
#define GPS_TX               25
#define GPS_RX               33
#define GPS_POWER_ON         12

#include <PubSubClient.h> 
char mqttServer[] = "";
String  _mqttServer  ;
const int mqttPort = 23456;
const char* mqttUser = "user";
const char* mqttPassword = "****";


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1
// Set serial for GPS module data (to the module)
#define SerialGPS  Serial2

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS
//#define DEBUG_MSG
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#define HOST_NAME "remotedebug"
#include "RemoteDebug.h"        //https://github.com/JoaoLopesF/RemoteDebug
RemoteDebug Debug;


TinyGsmClient client(modem);
PubSubClient client2(client);
TinyGPS gps;
WiFiClient espClient;

void saveconf(String NewIP) {
       //Save config
       if (SPIFFS.begin())  {
    	debugV("saving config");
        File configFile = SPIFFS.open("/config", "w");
        if (!configFile) {
         debugV("failed to open config file for writing");
        }

        configFile.println(NewIP);
        Debug.printf("mqttServer IP saved:%s", NewIP.c_str() );

         //end save
       }
}

bool readconf() {
 if (SPIFFS.begin()) {
      if (SPIFFS.exists("/config")) {
       debugV("reading config file");
//file exists, reading and loading
       File configFile = SPIFFS.open("/config", "r");
        if (configFile) {
           // String next ;
            while(configFile.available()) {
            //Lets read line by line from the file
              _mqttServer        = configFile.readStringUntil('\n');
            }
            //Remove newline and extraneouscharacters
            _mqttServer.trim();
            _mqttServer.toCharArray(mqttServer, _mqttServer.length()+1);
            debugV("mqttServer IP read:%s", _mqttServer.c_str() );
         } else {
           debugV("Failed to read conf");
           return false;
         }
      } else {
        debugV("Failed to open file");
  return false;
      }
 return true;
 } else {
   debugV("Failed to start SPIFS");

 }
 return false; 
}

void UpgradeOTA() {
  ArduinoOTA.onStart([]() {
      Serial.print("Start");
    });
  ArduinoOTA.onEnd([]() {
      Serial.print("\nEnd");
    });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
  ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.print("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.print("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.print("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.print("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.print("End Failed");
    ESP.restart();
  });
  ArduinoOTA.begin();
}

void StartingWifi() {

  // Starting Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
   Serial.print("Connecting to WiFi..");
    }
  Serial.print("Connected to the WiFi network");
  Serial.print(WiFi.localIP());
}

void startingMQTT() {
// Debug.printf("mqttServer IP read:%s", _mqttServer.c_str() );
 // Starting MQTT
 Serial.print("trying to connect to ");
 Serial.print(mqttServer); 
 client2.setServer(mqttServer, mqttPort);
}

void reconnect() {
  yield();
  static int a = 1 ;
  //loop until connected
      while (!client2.connected()) {
          yield();
          if (client2.connect("ESP32", mqttUser, mqttPassword)) {
              Serial.print("mqtt client connected to the host!");
            } else {
              Serial.print("mqtt failed to connect");
              delay(1000);
            }
          a++ ;
          if (a > 5){
             break ; // in order to avoid an infernal loop
          }
      }
}

void fetchSMS() {
 Serial.print("cheking new SMS...");
 unsigned long start = millis();
 String NewIP ;
 do {
  int i = 0;
  bool newSMS;
   while (SerialAT.available()) {
   msg[i] = SerialAT.read();
   i++;
   newSMS = true;
   if (i >= 180) {
    break;
    }
   }

   if (newSMS) {
    msg[i] = '\0';
    newSMS = false;
    String sms = String(msg);
    sms.trim();
    if (sms.substring(0, 4).equals("+CMT")) {
     Serial.print("SMS is:");
     Serial.print(sms.substring(46));
     NewIP = sms.substring(46) ;
     NewIP.toCharArray(mqttServer, NewIP.length()+1);
     saveconf(NewIP);
    } 
   };
  } while (millis() - start < 10000 ) ;
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
 yield();  
  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(30000L)) {
    SerialMon.println(" fail");
    delay(10000);
    return false;
  }
  SerialMon.println(" OK");

  if (!modem.isNetworkConnected()) {
    return false;
  }
  modem.receiveSMS();

  SerialMon.println("Network connected");
  fetchSMS();
  return true ; 
}


void GPS() {
   while (SerialGPS.available()) {
   Serial.write(SerialGPS.read());
 } 
}


void StartGPRS() {

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.print(" fail");
    delay(10000);
    return;
  }
  Serial.print("GPRS success");

  if (modem.isGprsConnected()) {
    Serial.print("GPRS connected");
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
  pinMode(PIN_WIFI, INPUT_PULLUP); // Gpio to put the Wifi ON

  if (digitalRead(PIN_WIFI) == HIGH) {
  StartingWifi();
  UpgradeOTA();  
  wifi_mode = true;

  // Initialize RemoteDebug
    Debug.begin("192.168.1.13"); // Initialize the WiFi server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
    Debug.showColors(true); // Colors
   Debug.setSerialEnabled(true); // All messages too send to serial too, and can be see in serial monitor
  }

  readconf();

  poweron_GPS();

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);

  // Set GPS module baud rate and UART pins
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);
//  poweron_GSM();
//  while (!connect_GSM()){};
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
 if (wifi_mode) {
  ArduinoOTA.handle();
  debugHandle(); // Equal to SerialDebug
 }
//GPS();
 static int a = 0;
 a++;
 unsigned long age, chars = 0;
 unsigned short sentences = 0, failed = 0;
 float flat, flon;
 gps.f_get_position(&flat, &flon, &age);
 gps.stats(&chars, &sentences, &failed);

 #ifdef DEBUG_MSG
 print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
 print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
 print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
 print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
 print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
 print_date(gps);
 print_int(chars, 0xFFFFFFFF, 6);
 print_int(sentences, 0xFFFFFFFF, 10);
 debugV("GPS:%u ", sentences);
 print_int(failed, 0xFFFFFFFF, 9);
 Serial.print("");
 if (a > 5) {
  Serial.print("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date  Chars Sentences Checksum");
  Serial.print("          (deg)     (deg)      Age                      Age   RX    RX         Fail");
  a = 0 ;
 }
 #endif

 smartdelay(1000);

 if (sentences > 20) { //&& (!wifi_mode )) {
  poweroff_GPS();
  debugV("We have now enough data, send it"); 
  Serial.print("We have now enough data, send it"); 
  int count = 0 ; 
  //Start GSM module
  poweron_GSM();
  debugV("connecting GSM...");
  while (!connect_GSM()){
   debugHandle(); // Equal to SerialDebug
   
   count++;
   debugV("trying to connect GSM");
   if (count > 5) {
    debugV("Failed to connect GSM, Going to deep sleep...");
    Serial.print("Failed to connect GSM, Going to deep sleep...");
    delay(1000); 
    ESP.deepSleep(60 * 1000000);
   }  
  };
  delay(2000);
  debugV("trying to connect GPRS");
  StartGPRS();

  debugV("trying to connect to MQTT Bocker");
  debugV("mqttServer IP read:%s", _mqttServer.c_str() );
  //Start MQTT client
  startingMQTT(); 
  debugHandle(); // Equal to SerialDebug
  
  count = 0 ; 
  while (!client2.connected()) {
   reconnect();
   count++;
   if (count > 5) {
    debugV("Failed to connect MQTT client, Going to deep sleep...");
    Serial.print("Failed to connect MQTT client, Going to deep sleep...");
    delay(1000); 
    ESP.deepSleep(60 * 1000000);
   }
  }
  debugV("Sending GPS coordinnates");
  Serial.print("Sending GPS coordinnates");
  String NMEA = "\{\"longitude\":" + String(flon, 7) + ",\"latitude\":" + String(flat, 7) + "}" ;
  client2.publish("GPS/location", String(NMEA).c_str() ) ;
  delay(2000);
  client2.publish("GPS/location", String(NMEA).c_str() ) ;
  delay(10000);
  Serial.print("Poweroff GSM...");
  poweroff_gsm();
  delay(5000);
  Serial.print("Going to deep sleep...");
  ESP.deepSleep(5 * 60 * 1000000);   
 }

}

