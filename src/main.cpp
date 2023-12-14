/*
Gate Counter by Greg Liebig gliebig@sheboyganlights.org
Initial Build 12/5/2023 12:15 pm
Changed time format YYYY-MM-DD hh:mm:ss 12/13/23
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
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include "NTPClient.h"
#include "WIFI.h"
#include "secrets.h"
#include "time.h"
//#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define vehicleSensorPin 4
#define PIN_SPI_CS 5 // The ESP32 pin GPIO5

//#include <DS3231.h>


RTC_DS3231 rtc;
char ssid[] = secret_ssid;
char pass[] = secret_pass;
char mqtt_u[] = mqtt_user;
char mqtt_p[] = mqtt_pass;
const char* ampm ="AM";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;
int16_t temp;

int currentDay = 0;
int currentHour = 0;
int currentMin = 0;
int totalDailyCars = 0;
int carPresent = 0;
int sensorBounces =0;
unsigned long detectorMillis = 0;
unsigned long debounceMillis = 12000; // Time required for my truck to pass totally
unsigned long nocarMillis = 3500; // Time required for High Pin to stay high to reset car in detection zone
unsigned long highMillis = 0; //Grab the time when the vehicle sensor is high
unsigned long previousMillis; // Last time sendor pin changed state
unsigned long currentMillis; // Comparrison time holder
unsigned long carDetectedMillis;  // Grab the ime when sensor 1st trips
int detectorState;
int previousdetectorState;
File myFile; //used to write files to SD Card
File myFile2;

char days[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};


Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire, -1);

void printWifiStatus() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.display();


  // print the SSID of the network you're attached to:
  display.setCursor(0, 0);
  display.print("SSID: ");
  display.println(WiFi.SSID());

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  display.setCursor(0, 10);
  display.print("IP: ");
  display.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  display.setCursor(0, 20);
  display.print("signal: ");
  display.print(rssi);
  display.println(" dBm");
  display.display();
  delay(10000);
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
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.display();

  if (!SD.begin(PIN_SPI_CS)) {
    Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    while (1); // stop the program
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Check SD Card");
    display.display();
  }

  Serial.println(F("SD CARD INITIALIZED."));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("SD Card Ready");
    display.display();
 
  if (!SD.exists("/GateCount.csv")) {
    Serial.println(F("GateCount.csv doesn't exist. Creating GateCount.csv file..."));
    // create a new file by opening a new file and immediately close it
    myFile = SD.open("/GateCount.csv", FILE_WRITE);
    myFile.close();
  }

  // recheck if file is created or not & write Header
  if (SD.exists("/GateCount.csv")){
    Serial.println(F("GateCount.csv exists on SD Card."));
    myFile = SD.open("/GateCount.csv", FILE_APPEND);
    myFile.println("Date Time,Millis,Car,TotalDailyCars,Temp");
    myFile.close();
    Serial.println(F("Header Written to GateCount.csv"));
  }else{
    Serial.println(F("GateCount.csv doesn't exist on SD Card."));
}
  if (!SD.exists("/SensorBounces.csv")) {
    Serial.println(F("SensorBounces.csv doesn't exist. Creating SensorBounces.csv file..."));
    // create a new file by opening a new file and immediately close it
    myFile2 = SD.open("/SensorBounces.csv", FILE_WRITE);
    myFile2.close();
  }

  // recheck if file is created or not & write Header
  if (SD.exists("/SensorBounces.csv")){
    Serial.println(F("SensorBounces.csv exists on SD Card."));
    myFile2 = SD.open("/SensorBounces.csv", FILE_APPEND);
    myFile2.println("Date Time,Millis,CarNum,sensorBounces");
    myFile2.close();
    Serial.println(F("Header Written to SensorBounces.csv"));
  }else{
    Serial.println(F("SensorBounces.csv doesn't exist on SD Card."));
}


  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
    display.println("Connecting to WiFi..");
    display.display();
  }
  Serial.println("Connected to the WiFi network");
  printWifiStatus();

  //If RTC not present, stop and check battery
  if (! rtc.begin()) {
    Serial.println("Could not find RTC! Check circuit.");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
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
  display.setCursor(0, 15);
  display.print("GATE Counter");

  Serial.println  ("Initializing Gate Counter");
    Serial.print("Temperature: ");
    temp=((rtc.getTemperature()*9/5)+32);
    Serial.print(temp);
    Serial.println(" F");
  display.display();
  delay(3000);
}

void loop() {
  DateTime now = rtc.now();
  temp=((rtc.getTemperature()*9/5)+32);
  //Reset Gate Counter at 5:00:00 pm
  if ((now.hour() == 17) && (now.minute() == 0) && (now.second() == 0)){
    totalDailyCars = 0;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  //  display Day of Week
  display.println(days[now.dayOfTheWeek()]);

  //  Display Date
  display.setTextSize(1);
  display.setCursor(23, 0);         
  display.println(months[now.month(), DEC +1]);

  display.setTextSize(1);
  display.setCursor(50, 0);
  display.println(now.day(), DEC);
  
  display.setTextSize(1);
  display.setCursor(63, 0);
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
    display.setCursor(0, 10);
    display.print("0");
    display.println(currentHour, DEC);
  }else{
    display.setCursor(0, 10);
    display.println(currentHour, DEC);
  }

  display.setCursor(14, 10);
  display.println(":");
 
  //Add leading 0 To Mintes & display Minutes 
//  display.setTextSize(1);
  if (now.minute() < 10) {
    display.setCursor(20, 10);
    display.print("0");
    display.println(now.minute(), DEC);
  }else{
    display.setCursor(21, 10);
    display.println(now.minute(), DEC);
  }

  display.setCursor(34, 10);
  display.println(":");

  //Add leading 0 To Seconds & display Seconds
//  display.setTextSize(1);
  if (now.second() < 10){
    display.setCursor(41, 10);
    display.print("0");
    display.println(now.second(), DEC);
  }else{
    display.setCursor(41, 10);
    display.println(now.second(), DEC);   
  }

// Display AM-PM
    display.setCursor(56, 10);
    display.println(ampm); 

//Display Temp
//  display.setTextSize(1);
  display.setCursor(73, 10);
  display.print("Temp: " );
  //display.setCursor(70, 10);
  display.println(temp, 0);

 // Display Gate Count
//  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Gate Count: ");
  display.println(totalDailyCars);

  display.display();

// Count Cars Exiting
  detectorState = digitalRead(vehicleSensorPin);
	currentMillis = millis(); // grab current time

	// Sensing Vehicle  
  if (detectorState == LOW) {
      carPresent = 1;
      carDetectedMillis = millis(); // Time car was detected
         Serial.print(carDetectedMillis);
         Serial.print(" , Car Triggered Detector: ");
         Serial.print(detectorState);
         Serial.print(", total daily cars = ");         
         Serial.println (totalDailyCars) ;    
         currentMillis = millis();
         sensorBounces = 0;

      // When Sensor is tripped, wait until sensor clears
      while (carPresent == 1) {
        detectorState = digitalRead(vehicleSensorPin);
        if (digitalRead(vehicleSensorPin) != digitalRead(vehicleSensorPin)) {
          currentMillis = millis();
          sensorBounces ++;
          Serial.print("Swtich Toggled ");
          Serial.print(digitalRead(vehicleSensorPin));
          Serial.print(" detectorState = ");
          Serial.println(detectorState);
          DateTime now = rtc.now();
          char buf2[] = "YYYY-MM-DD hh:mm:ss";
          Serial.print(now.toString(buf2));
          // open file for writing debugging information
          myFile2 = SD.open("/SensorBounces.csv", FILE_APPEND);
            if (myFile2) {
                myFile2.print(now.toString(buf2));
                myFile2.print(", "); 
                myFile2.print(millis()-carDetectedMillis);
                myFile2.print(", "); 
                myFile2.print (totalDailyCars) ; 
                myFile2.print(", ");
                myFile2.println(sensorBounces);
                myFile2.close();
                Serial.println(F("Log Recorded SD Card."));
            } else {
                Serial.print(F("SD Card: Issue encountered while attempting to open the file GateCount.csv"));
            }
        }
        
        if ((detectorState != LOW) && (millis()-currentMillis >= nocarMillis)) {
          //previousMillis = millis()-currentMillis;
          DateTime now = rtc.now();
          Serial.print(temp);
          Serial.print(", ");
          char buf2[] = "YYYY-MM-DD hh:mm:ss";
          Serial.print(now.toString(buf2));
          //Serial.print(String("DateTime::TIMESTAMP_FULL:\t")+now.timestamp(DateTime::TIMESTAMP_FULL));
          Serial.print(",1,"); 
          totalDailyCars ++;     
          Serial.println (totalDailyCars) ;  
          // open file for writing Car Data
          myFile = SD.open("/GateCount.csv", FILE_APPEND);
          if (myFile) {
              myFile.print(now.toString(buf2));
              myFile.print(", ");
              myFile.print (millis()-carDetectedMillis) ; 
              myFile.print(", 1 , "); 
              myFile.print (totalDailyCars) ; 
              myFile.print(", ");
              myFile.println(temp);
              myFile.close();
              Serial.println(F("Log Recorded SD Card."));
          } else {
              Serial.print(F("SD Card: Issue encountered while attempting to open the file GateCount.csv"));
          }
          carPresent = 0;
        }
            //previousMillis = currentMillis;
      }
        //  previousMillis = currentMillis;
  }
	
}