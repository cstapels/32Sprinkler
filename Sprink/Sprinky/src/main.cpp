#include <Arduino.h>
#include <constants.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Time.h>
#include <TimeLib.h>
#include "communication.h"
#include "utilities.h"
#include "ThingSpeak.h"
#include "EEPROM.h"

// Global variables
struct controlData memory;
struct controlData online;
int timeOffset = 4;

bool connectedBool = false;
String statusMessage = "";
bool eepromChanged = false;


// Network information
String ssid = "still_waters";
const char *password = "33turkeys511";
long timeReadChannelId = 444067;
const char *timeReadAPIKey = "NBU9ULD2EJ1ELM9T"; // time control channel

// ThingSpeak settings
char server[] = "api.thingspeak.com";
WiFiClient client;


// #define DEVICE1
#define DEVICE2
 //#define DEVICE3
// #define DEVICE4
// #define DEVICEDUCK

#ifdef DEVICE1
const char *writeAPIKey = "H6EVSHKVVKR8KNJP"; // battery channel
const char *readAPIKey = "VV1UZ2CRNCUJ8VER";  // control channel
long readChannelId = 597924;
long myChannelNumber = 592680;
#endif

#ifdef DEVICE2
const char *writeAPIKey = "VML7XDJTAELWPWH8"; // second battery channel
long myChannelNumber = 844390;                // channel 2
const char *readAPIKey = "OJXNM9UUX2V6P8UD";  // second control channel
long readChannelId = 844392;                  // channel2
#endif

#ifdef DEVICE3
const char *writeAPIKey = "0L6OFFAKEQTBBNBJ"; // third battery channel
long myChannelNumber = 1005405;               // channel 3
const char *readAPIKey = "BMKL2AZ5SB4SUNFZ";  // third control channel
long readChannelId = 1005406;                 // channel2
#endif

#ifdef DEVICE4
const char *writeAPIKey = "DXKLMJ1W7WMYDD8D"; // lone battery channel
long myChannelNumber = 10505221;              // channel 4
const char *readAPIKey = "P5SQIBO7FZNH9XBV";  // lone control channel
long readChannelId = 1050522;                 // channel2
#endif

#ifdef DEVICEDUCK
const char *writeAPIKey = "MEC37ONNJUT84A4U"; // Ducktank channel
long myChannelNumber = 1160823;               // channel 4
const char *readAPIKey = "CN409WRNXUVJYKGQ";  // Ducktank control channel
long readChannelId = 1160824;                 // channel2
#endif


// prototypes
int getTimeStatus();
int getPumpStatus(int batteryLevel, int waterLevel);
void getReadyToSleep(int statusCode);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000); // Give time for the serial monitor to connect
  Serial.println("Hello, Sprinky!");

  pinMode(BATT_PIN, INPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LEVEL_PIN, INPUT);
  pinMode(LEVEL_SENSE_LOW, INPUT);
  pinMode(LEVEL_SENSE_HIGH, INPUT);
  pinMode(LEVEL_CONTROL_PIN, OUTPUT);
  pinMode(TRIG, OUTPUT); // Initializing Trigger Output and Echo Input
  pinMode(ECHO, INPUT_PULLUP);
  pinMode(SERVO_BUTTON_PIN, INPUT_PULLUP);
  // connectWiFi();
  EEPROM.begin(16);

  // Additional setup code can go here
}

// memory map
// water
// sleep
// mode
//  fill hour
// connecton wake
//  Battery min
//  Level Min

// selectMode bits
// 1 sleep enable (and EEPROM write)
// 2 pump enable
// 3 overnight sleep
// 4 level check
// 5 battery check


void loop()
{

  blinkX(2, 150);
  // wake up
  // read eeprom
  // connct to wifi
  // read time
  // read instructions
  // do pumping
  // go to sleep
  readEEPROM();


  if ((WiFi.status() != WL_CONNECTED))
  {
    connectWiFi();
  }

  Serial.println("Connected bool" + String(connectedBool));

  if (connectedBool)
  {
    String theTime = readTSPTime();
    if (theTime == "time Failed")
    {
      theTime = "Failed to get time";
      delay(1000);
      // try again
      String theTime = readTSPTime();
      statusMessage += statusMessage + " Time Failed";
    }

     if (theTime == "time Failed")
    {
      connectedBool = false;
      int timeStatus = 3;

    }


    Serial.println(theTime);
    Serial.println("hour is " + String(hour()) + " minute is " + String(minute()));
    statusMessage += theTime + " ";
    readTSP();
  }

  int wifiStrength = getStrength(numberPoints);
  Serial.println(wifiStrength);
  int batteryLevel = getAnalog(BATT_PIN);
  int lowState = digitalRead(LEVEL_SENSE_LOW); //water level sensors
    Serial.println("low level is " + String(lowState));
  int highState = digitalRead(LEVEL_SENSE_HIGH);
    Serial.println("high level is " + String(highState));
  digitalWrite(LEVEL_POWER,HIGH);
  delay(10);
  int waterLevel = lowState + 10 * highState;
  digitalWrite(LEVEL_POWER,LOW);
  
  
//print the level states
  Serial.println("Water level is " + String(waterLevel));

  if ((online.selectMode & (1 << 3)) and (connectedBool))
  { // can't check time if we arent connected
    Serial.println("Time Check");
    // do stuff based on time

    int timeStatus = getTimeStatus();
    int pumpStatus = getPumpStatus(batteryLevel, waterLevel);

    if (pumpStatus)
    {
      runPump(PUMP_PIN, online.waterTime); // time in seconds
      statusMessage += "Pumped " + String(online.waterTime) + " seconds";
    }

    delay(100);

    // now write the values
    if (connectedBool)
    {
      httpRequest(batteryLevel, waterLevel, wifiStrength, 56, statusMessage);
    }

    getReadyToSleep(timeStatus);
  }
  // if not connected

  //run pump using the variables read from eeprom
  int pumpStatus = getPumpStatus(batteryLevel, waterLevel);
      if (pumpStatus)
    {
      runPump(PUMP_PIN, online.waterTime); // time in seconds
      statusMessage += "Pumped " + String(memory.waterTime) + " seconds";
    }


  esp_sleep_enable_timer_wakeup(500 * 1000000); // try again in ten minutes
  Serial.println("Going to sleep now to wait for better wifi");
  Serial.flush();
  esp_deep_sleep_start();


  Serial.flush();
  esp_deep_sleep_start();


}



