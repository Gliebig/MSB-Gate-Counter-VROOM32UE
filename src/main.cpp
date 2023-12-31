/*
Gate Counter by Greg Liebig gliebig@sheboyganlights.org
Initial Build 12/5/2023 12:15 pm
Changed time format YYYY-MM-DD hh:mm:ss 12/13/23
Added MQTT to Hive MQTT and Fixed WIFI for Testing at home and on-site7 12/15/23

Purpose: suppliments Car Counter to improve traffic control and determine park capacity
Counts vehicles as they exit the park
Connects to WiFi and updates RTC on Boot
Uses an Optocoupler to read burried vehicle sensor for Ghost Controls Gate operating at 12V
DOIT DevKit V1 ESP32 with built-in WiFi & Bluetooth
SPI Pins
D5 - CS
D18 - CLK
D19 - MISO
D23 - MOSI

*/
#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
//#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include "NTPClient.h"
#include "WIFI.h"
#include "WiFiClientSecure.h"
#include "WiFiMulti.h"
#include "secrets.h"
#include "time.h"
//#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <AsyncTCP.h>
//#include <WebServer.h>
//#include <ElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
//#include <Arduino_JSON.h>

#define vehicleSensorPin 4
#define PIN_SPI_CS 5 // The ESP32 pin GPIO5
#define MQTT_KEEPALIVE 30

// HiveMQ Cloud Let's Encrypt CA certificate
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

//Setup Webserver Object
//WebServer server(80);
AsyncWebServer server(80);

//#include <DS3231.h>
RTC_DS3231 rtc;
int line1 =0;
int line2 =9;
int line3 = 20;
int line4 = 30;
int line5 = 42;
int line6 = 50;
int line7 = 53;

//Create Multiple WIFI Object

WiFiMulti wifiMulti;
WiFiClientSecure espGateCounter;
PubSubClient mqtt_client(espGateCounter);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (500)
char msg[MSG_BUFFER_SIZE];
int value = 0;

char mqtt_server[] = mqtt_Server;
char mqtt_username[] = mqtt_UserName;
char mqtt_password[] = mqtt_Password;
const int mqtt_port = mqtt_Port;

#define THIS_MQTT_CLIENT "espGateCounter" // Look at line 90 and set variable for WiFi Client secure & PubSubCLient 12/23/23
#define MQTT_PUB_TOPIC0  "msb/traffic/exit/hello"
#define MQTT_PUB_TOPIC1  "msb/traffic/exit/temp"
#define MQTT_PUB_TOPIC2  "msb/traffic/exit/time"
#define MQTT_PUB_TOPIC3  "msb/traffic/exit/count"
#define MQTT_PUB_TOPIC4  "msb/traffic/exit/inpark"
#define MQTT_PUB_TOPIC5  "msb/traffic/exit/timeout"

#define MQTT_SUB_TOPIC0  "msb/traffic/enter/count"
#define MQTT_SUB_TOPIC1  "msb/traffic/exit/resetcount"


//const uint32_t connectTimeoutMs = 10000;
uint16_t connectTimeOutPerAP=5000;
const char* ampm ="AM";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;
int16_t temp;

char buf2[25] = "YYYY-MM-DD hh:mm:ss";

int currentDay = 0;
int currentHour = 0;
int currentMin = 0;
int totalDailyCars = 0;
int carCounterCars =0;
int sensorBounceCount=0;
int sensorBounceRemainder;
bool sensorBounceFlag;

bool carPresentFlag = 0;

bool nocarTimerFlag = 0;
unsigned long nocarTimerMillis =0;
bool detectorState =1;
bool lastdetectorState =1;
unsigned long whileMillis; // used for debugging
unsigned long lastwhileMillis = 0;
unsigned long detectorStateLowMillis; // used for debugging
unsigned long lastdetectorStateLowMillis = 0;
unsigned long detectorStateHighMillis;
unsigned long lastdetectorStateHighMillis;

//###############################################################################################################
unsigned long nocarTimeoutMillis = 900; // Time required for High Pin to stay high to reset car in detection zone
unsigned long carpassingTimoutMillis = 6000; // Time delay to allow car to pass before checking for HIGN pin

//unsigned long highMillis = 0; //Grab the time when the vehicle sensor is high
unsigned long previousMillis; // Last time sendor pin changed state
unsigned long currentMillis; // Comparrison time holder
unsigned long carDetectedMillis;  // Grab the ime when sensor 1st trips
unsigned long lastcarDetectedMillis;  // Grab the ime when sensor 1st trips


unsigned long wifi_lastReconnectAttemptMillis;
unsigned long wifi_connectioncheckMillis = 5000; // check for connection every 5 sec
unsigned long mqtt_lastReconnectAttemptMillis;
unsigned long mqtt_connectionCheckMillis = 5000;
unsigned long nowwifi;
unsigned long nowmqtt;






File myFile; //used to write files to SD Card
File myFile2;

char days[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire, -1);
/*
unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}
*/



void setup_wifi() {
    Serial.println("Connecting to WiFi");
    display.println("Connecting to WiFi..");
    display.display();
    while(wifiMulti.run(connectTimeOutPerAP) != WL_CONNECTED) {
      //Serial.print(".");
    }
    Serial.println("Connected to the WiFi network");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.display();
   // print the SSID of the network you're attached to:
    display.setCursor(0, line1);
    display.print("SSID: ");
    display.println(WiFi.SSID());

    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP: ");
    Serial.println(ip);
    display.setCursor(0, line2);
    display.print("IP: ");
    display.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
    display.setCursor(0, line3);
    display.print("signal: ");
    display.print(rssi);
    display.println(" dBm");
    display.display();
    delay(5000);
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  payload[length] = '\0';
 
  if (strcmp(topic, MQTT_SUB_TOPIC0) == 0) {
     carCounterCars = atoi((char *)payload);
//     Serial.println(" Car Counter Updated");
    }
  
  if (strcmp(topic, MQTT_SUB_TOPIC1) == 0){
    totalDailyCars = atoi((char *)payload);
//    Serial.println(" Gate Counter Updated");
  }
  //  Serial.println(carCountCars);
  Serial.println();
}


void reconnect() {
  // Loop until we’re reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection… ");
    String clientId = THIS_MQTT_CLIENT;
    // Attempt to connect
    if (mqtt_client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected!");
      Serial.println("Waiting for Car");
      // Once connected, publish an announcement…
      mqtt_client.publish(MQTT_PUB_TOPIC0, "Hello from Gate Counter!");
      // … and resubscribe
      mqtt_client.subscribe(MQTT_PUB_TOPIC0);
    } else {
      Serial.print("failed, rc = ");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
  mqtt_client.subscribe(MQTT_SUB_TOPIC0);
  mqtt_client.subscribe(MQTT_SUB_TOPIC1);
}

void SetLocalTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time. Using Compiled Date");
    return;
  }
  //Following used for Debugging and can be commented out
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");

  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();

  // Convert NTP time string to set RTC
  char timeStringBuff[50]; //50 chars should be enough
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.println(timeStringBuff);
  rtc.adjust(DateTime(timeStringBuff));
}


void setup() {
  Serial.begin(115200);
  //Initialize Display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.display();

  //Initialize SD Card
  if (!SD.begin(PIN_SPI_CS)) {
    Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    while (1); // stop the program
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,line1);
    display.println("Check SD Card");
    display.display();
  }

  Serial.println(F("SD CARD INITIALIZED."));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,line1);
    display.println("SD Card Ready");
    display.display();
 
  if (!SD.exists("/GateCount.csv")) {
    Serial.println(F("GateCount.csv doesn't exist. Creating GateCount.csv file..."));
    // create a new file by opening a new file and immediately close it
    myFile = SD.open("/GateCount.csv", FILE_WRITE);
    myFile.close();
      // recheck if file is created or not & write Header
  if (SD.exists("/GateCount.csv")){
    Serial.println(F("GateCount.csv exists on SD Card."));
    myFile = SD.open("/GateCount.csv", FILE_APPEND);
    myFile.println("Date Time,Pass Timer,NoCar Timer,Bounces,Car#,Cars In Park,Temp,Last Car Millis, This Car Millis,Bounce Flag,Millis");
    myFile.close();
    Serial.println(F("Header Written to GateCount.csv"));
  }else{
    Serial.println(F("GateCount.csv doesn't exist on SD Card."));
  }
  }


  if (!SD.exists("/SensorBounces.csv")) {
    Serial.println(F("SensorBounces.csv doesn't exist. Creating SensorBounces.csv file..."));
    // create a new file by opening a new file and immediately close it
    myFile2 = SD.open("/SensorBounces.csv", FILE_WRITE);
    myFile2.close();
   // recheck if file is created or not & write Header
  if (SD.exists("/SensorBounces.csv")){
    Serial.println(F("SensorBounces.csv exists on SD Card."));
    myFile2 = SD.open("/SensorBounces.csv", FILE_APPEND);
    //("DateTime\t\t\tPassing Time\tLast High\tDiff\tLow Millis\tLast Low\tDiff\tBounce #\tCurent State\tCar#" )
    myFile2.println("Time,Pass Timer,Last High,Diff,No Car Timer,Low Millis,Last Low,Diff,Bounce#,Curent State,Car#,Millis");
    myFile2.close();
    Serial.println(F("Header Written to SensorBounces.csv"));
  }else{
    Serial.println(F("SensorBounces.csv doesn't exist on SD Card."));
  }
 
  }



  WiFi.mode(WIFI_STA); 
  wifiMulti.addAP(secret_ssid_AP_1,secret_pass_AP_1);
  wifiMulti.addAP(secret_ssid_AP_2,secret_pass_AP_2);
  wifiMulti.addAP(secret_ssid_AP_3,secret_pass_AP_3);
  wifiMulti.addAP(secret_ssid_AP_4,secret_pass_AP_4);
  wifiMulti.addAP(secret_ssid_AP_5,secret_pass_AP_5);
  
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0) {
      Serial.println("no networks found");
  } 
  else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }
  
  setup_wifi();
  espGateCounter.setCACert(root_ca);
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(callback);

  //If RTC not present, stop and check battery
  if (! rtc.begin()) {
    Serial.println("Could not find RTC! Check circuit.");
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,line1);
    display.println("Clock DEAD");
    display.display();
    while (1);
  }

  // Get NTP time from Time Server 
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  SetLocalTime();
  
  //Set Input Pin
  pinMode(vehicleSensorPin, INPUT_PULLUP);
 
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, line4);
  display.print("GATE Count");

  //SETUP WEB SEVER
  /*
    server.on("/", []() {
        server.send(200, "text/plain", "Hi! This is the Gate Counter.");
      });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");
 */
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am the GATE COUNTER ESP32.");
  });

  
  
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");


  Serial.println  ("Initializing Gate Counter");
    Serial.print("Temperature: ");
    temp=((rtc.getTemperature()*9/5)+32);
    Serial.print(temp);
    Serial.println(" F");
  display.display();
  delay(3000);
}

void loop() {
//  server.handleClient();
//  ElegantOTA.loop();

    // non-blocking WiFi and MQTT Connectivity Checks
    if (wifiMulti.run() == WL_CONNECTED) {
      // Check for MQTT connection only if wifi is connected
      if (!mqtt_client.connected()){
        nowmqtt=millis();
        if(nowmqtt - mqtt_lastReconnectAttemptMillis > mqtt_connectionCheckMillis){
          mqtt_lastReconnectAttemptMillis = nowmqtt;
          Serial.println("Attempting MQTT Connection");
          reconnect();
        }
          mqtt_lastReconnectAttemptMillis =0;
      } else {
        //keep MQTT client connected when WiFi is connected
        mqtt_client.loop();
      }
    } else {
        // Reconnect WiFi if lost, non blocking
        nowwifi=millis();
          if ((nowwifi - wifi_lastReconnectAttemptMillis) > wifi_connectioncheckMillis){
            setup_wifi();
          }
        wifi_lastReconnectAttemptMillis = 0;
    }


      
      DateTime now = rtc.now();
      temp=((rtc.getTemperature()*9/5)+32);
      //Reset Gate Counter at 5:00:00 pm
        if ((now.hour() == 17) && (now.minute() == 0) && (now.second() == 0)){
             totalDailyCars = 0;
         }
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, line1);
      //  display Day of Week
      display.print(days[now.dayOfTheWeek()]);

      //  Display Date
      display.print(" ");         
      display.print(months[now.month(), DEC +1]);
      display.print(" ");
      display.print(now.day(), DEC);
      display.print(", ");
      display.println(now.year(), DEC);
      
      // Convert 24 hour clock to 12 hours
      currentHour = now.hour();

      if (currentHour - 12 > 0) {
          ampm ="PM";
          currentHour = now.hour() - 12;
      }else{
          currentHour = now.hour();
          ampm = "AM";
      }

      //Display Time
      //add leading 0 to Hours & display Hours
      display.setTextSize(1);

      if (currentHour < 10){
        display.setCursor(0, line2);
        display.print("0");
        display.println(currentHour, DEC);
      }else{
        display.setCursor(0, line2);
        display.println(currentHour, DEC);
      }

      display.setCursor(14, line2);
      display.println(":");
 
      //Add leading 0 To Mintes & display Minutes 
      //  display.setTextSize(1);
      if (now.minute() < 10) {
        display.setCursor(20, line2);
        display.print("0");
        display.println(now.minute(), DEC);
      }else{
        display.setCursor(21, line2);
        display.println(now.minute(), DEC);
      }

      display.setCursor(34, line2);
      display.println(":");

      //Add leading 0 To Seconds & display Seconds
      //  display.setTextSize(1);
      if (now.second() < 10){
        display.setCursor(41, line2);
        display.print("0");
        display.println(now.second(), DEC);
      }else{
        display.setCursor(41, line2);
        display.println(now.second(), DEC);   
      }

      // Display AM-PM
      display.setCursor(56, line2);
      display.println(ampm); 

      // Display Temp
      // display.setTextSize(1);
      display.setCursor(73, line2);
      display.print("Temp: " );
      //display.setCursor(70, 10);
      display.println(temp, 0);

      // Display Gate Count
      display.setTextSize(1);
      display.setCursor(0, line3);
      display.print("Exiting: ");
      display.setTextSize(2); 
      
      display.setCursor(50, line3);           
      display.println(totalDailyCars);
      display.setTextSize(1);
      display.setCursor(0, line5);
      display.print("In Park: ");
      display.setTextSize(2); 
      display.setCursor(50, line5);
      display.println(carCounterCars-totalDailyCars);


      display.display();
      detectorState=digitalRead(vehicleSensorPin);
      // Count Cars Exiting
      // Sensing Vehicle  
      // Detector LOW when vehicle sensed, Normally HIGH
      if (detectorState == LOW) {
          lastwhileMillis = 0;
          lastdetectorStateLowMillis=0;
          nocarTimerMillis=0;
          sensorBounceCount = 0; //Sensor went low 1st time
          carPresentFlag = 1; // when detector senses car, set flag car is present.
          carDetectedMillis = millis(); // Freeze time when car was detected
          detectorStateHighMillis = 0;
          detectorStateLowMillis = millis()-carDetectedMillis;
          lastdetectorState=HIGH;
          DateTime now = rtc.now();
          char buf3[] = "YYYY-MM-DD hh:mm:ss"; //time of day when detector was tripped
          Serial.print("Car Triggered Detector at = ");
          Serial.print(carDetectedMillis);
          Serial.print(", Car Number Being Counted = ");         
          Serial.println (totalDailyCars+1) ;  //add 1 to total daily cars so car being detected is synced
          Serial.println("DateTime\t\tWhile\tLHigh\tDiff\tnoCar\tLow Millis\tLast LOW\tDiff\tBounce #\tCurent State\tCar#\tMillis" );  

          // When Sensor is tripped, figure out when car clears sensing zone & sensor remains HIGH for period of time
          // Then Reset Car Present Flag to 0
          while (carPresentFlag == 1) {
             detectorState = digitalRead(vehicleSensorPin);
             currentMillis = millis();

             whileMillis=millis()-carDetectedMillis; //   Record relative time passing         
                       //if detector state changes from lOW to HIGH a car cleared sensor. If it remains HIGH no car is detected
                      
                       if ((detectorState != lastdetectorState)  && (detectorState==HIGH)) {
                                          lastwhileMillis=whileMillis; 
                                          nocarTimerMillis = millis();       
                       }

                       if ((detectorState != lastdetectorState)  && (detectorState==LOW)) {
                          sensorBounceCount ++;  //count the state changes
                          //nocarTimerMillis = millis();
                       //}
                       
                       //Record Bounce
                          DateTime now = rtc.now();
                          char buf2[] = "YYYY-MM-DD hh:mm:ss";
                          //Count number of Bounces and check each 4 bounces

                          //start a timer to time how long senstor remains high when it switches states
                          detectorStateLowMillis=millis()-carDetectedMillis; //

                          

                          //Debugging Code Can be removed  **************************************************************************
                          Serial.print(now.toString(buf2));
                          Serial.print(" \t\t ");
                          Serial.print(whileMillis);
                          Serial.print(" \t ");
                          Serial.print(lastwhileMillis);
                          Serial.print(" \t ");
                          Serial.print(whileMillis-lastwhileMillis);
                          Serial.print(" \t ");
                          Serial.print(millis()-nocarTimerMillis);  
                          Serial.print(" \t ");
                          Serial.print(detectorStateLowMillis);                        
                          Serial.print(" \t\t ");   
                          Serial.print(lastdetectorStateLowMillis);
                          Serial.print(" \t\t ");   
                          Serial.print(detectorStateLowMillis-lastdetectorStateLowMillis);
                          Serial.print(" \t\t ");   
                          Serial.print(sensorBounceCount);
                          Serial.print(" \t\t ");              
                          Serial.print(detectorState);
                          //Serial.print(" \t\t ");
                          //Serial.print(lastdetectorState);
                          Serial.print(" \t\t ");
                          Serial.print(totalDailyCars+1);
                          Serial.print(" \t\t ");
                          Serial.print(millis());
                          Serial.println();
                         
                         //T("DateTime\t\t\tPassing Time\tLast High\tDiff\tLow Millis\tLast Low\tDiff\tBounce #\tCurent State\tCar#" )
                          myFile2 = SD.open("/SensorBounces.csv", FILE_APPEND);
                          if (myFile2) {
                              myFile2.print(now.toString(buf2));
                              myFile2.print(", "); 
                              myFile2.print(whileMillis);
                              myFile2.print(", "); 
                              myFile2.print (lastwhileMillis) ; 
                              myFile2.print(", ");
                              myFile2.print(whileMillis-lastwhileMillis);
                              myFile2.print(", ");
                              myFile2.print(millis()-nocarTimerMillis);
                              myFile2.print(", ");
                              myFile2.print(detectorStateLowMillis);
                              myFile2.print(", ");
                              myFile2.print(lastdetectorStateLowMillis);
                              myFile2.print(", ");
                              myFile2.print(detectorStateLowMillis-lastdetectorStateLowMillis);
                              myFile2.print(" , ");              
                              myFile2.print(sensorBounceCount);
                              myFile2.print(" , ");
                              myFile2.print(detectorState);
                              myFile2.print(" , ");
                              myFile2.print(totalDailyCars+1); //Prints this Car millis
                              myFile2.print(" , ");
                              myFile2.print(lastcarDetectedMillis); //Prints Last car millis
                              myFile2.print(" , ");
                              myFile2.print(carDetectedMillis); //Prints car number being detected
                              myFile2.print(" , ");
                              myFile2.print(millis()); //Prints current millis for debugging
                              myFile2.println();
                              myFile2.close();
                              //Serial.println(F(" Bounce Log Recorded SD Card."));
                          } else {
                              Serial.print(F("SD Card: Issue encountered while attempting to open the file GateCount.csv"));
                          }
                       
                           // end of debugging code ********************************************************************************* 
                           //Detector State is bouncing
                       } // end of if detector state is bouncing check

          // Dectector is HIGH CHECK CAR HAS CLEARED AND BREAK LOOP #################################################

                      //force count & reset if there is an undetectable car present 12/25/23
                      
                      //sensorBounceRemainder = sensorBounceCount % 4;
                      //if ((sensorBounceRemainder == 0) && (whileMillis-lastwhileMillis>1042)){
                      //  nocarTimerFlag = 0; 
                      //  sensorBounceFlag = 1;  
                      //}else{
                      
                            // Check added 12/21/23 to ensure no car is present for x millis
                            if (detectorState==HIGH)  {
                                         //lastwhileMillis=whileMillis;  
                              
                              // If no car is present and state does not change, then car has passed
                              if (((millis() - nocarTimerMillis) >= nocarTimeoutMillis) && (sensorBounceCount >=2)) { 
                                nocarTimerFlag = 0;
                              } 
                              //Resets if Loop sticks after 10 seconds and does not record a car.
                              if (millis() - carDetectedMillis > 10000) {
                                 Serial.println("Timeout! No Car Counted");
                        mqtt_client.publish(MQTT_PUB_TOPIC5, String(totalDailyCars+1).c_str());
                                 carPresentFlag=0;
                                 break;
                              }

                            } else {
                              //nocarTimerMillis = millis();   // Start or Reset Timer when pin goes high
                              nocarTimerFlag = 1;  // change state to active
                              }
                      //}


             //allow enough time for a car to pass and then make sure sensor remains high 12/23/23
             //Conditions that myst be met for a car to be clear and count the car ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
             //Main Reset car passing timer
             
             //if (((currentMillis - carDetectedMillis)>=carpassingTimoutMillis) && (detectorState == HIGH) && (nocarTimerFlag == 0)) {
               if (((detectorState == HIGH) && (nocarTimerFlag == 0)) || (detectorStateLowMillis-lastdetectorStateLowMillis>2000)) {
                  Serial.print(now.toString(buf3));
                  Serial.print(", Millis NoCarTimer = ");
                  Serial.print(currentMillis-nocarTimerMillis);
                  Serial.print(", Total Millis to pass = ");
                  Serial.println(currentMillis-carDetectedMillis);
                  //Serial.print(", ");
                  //Serial.print(String("DateTime::TIMESTAMP_FULL:\t")+now.timestamp(DateTime::TIMESTAMP_FULL));
                  //Serial.print(",1,"); 
                  totalDailyCars ++;     

                  // open file for writing Car Data
                  //"Date Time,Pass Timer,NoCar Timer,TotalExitCars,CarsInPark,Temp"
                  myFile = SD.open("/GateCount.csv", FILE_APPEND);
                  if (myFile) {
                      myFile.print(now.toString(buf3));
                      myFile.print(", ");
                      myFile.print (currentMillis-carDetectedMillis) ; 
                      myFile.print(", ");
                      myFile.print (currentMillis-nocarTimerMillis) ; 
                      myFile.print(", "); 
                      myFile.print (sensorBounceCount) ; 
                      myFile.print(", ");                       
                      myFile.print (totalDailyCars) ; 
                      myFile.print(", ");
                      myFile.print(carCounterCars-totalDailyCars);
                      myFile.print(", ");
                      myFile.print(temp);
                              myFile.print(" , ");
                              myFile.print(lastcarDetectedMillis); //Prints car number being detected
                              myFile.print(" , ");
                              myFile.print(carDetectedMillis); //Prints car number being detected
                      myFile.print(", ");
                      myFile.print(sensorBounceFlag);
                      myFile.print(", ");
                      myFile.println(millis());
                      myFile.close();
                      
                      Serial.print(F("Car Saved to SD Card. Car Number = "));
                      Serial.print(totalDailyCars);
                      Serial.print(F(" Cars in Park = "));
                      Serial.println(carCounterCars-totalDailyCars);  
                        mqtt_client.publish(MQTT_PUB_TOPIC1, String(temp).c_str());
                        mqtt_client.publish(MQTT_PUB_TOPIC2, now.toString(buf3));
                        mqtt_client.publish(MQTT_PUB_TOPIC3, String(totalDailyCars).c_str());
                        mqtt_client.publish(MQTT_PUB_TOPIC4, String(carCounterCars-totalDailyCars).c_str());
                        //snprintf (msg, MSG_BUFFER_SIZE, "Car #%ld,", totalDailyCars);
                        //Serial.print("Publish message: ");
                        //Serial.println(msg);
                        //mqtt_client.publish("msbGateCount", msg);
                      //}
                  } else {
                      Serial.print(F("SD Card: Issue encountered while attempting to open the file GateCount.csv"));
                  }
                  carPresentFlag = 0;
                  sensorBounceFlag = 0;
                  whileMillis = 0;
                  lastcarDetectedMillis=carDetectedMillis;
              }  // end of car passed check

             lastdetectorState=detectorState;
             lastdetectorStateLowMillis=detectorStateLowMillis;
             lastdetectorStateHighMillis=detectorStateHighMillis;


             //sensorBounceCount =0;
            
           } // end of while loop
      } // Start looking for next lOW on Vehicle sensor

      //loop forever looking for car and update time and counts
}