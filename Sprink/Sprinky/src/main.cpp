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

int sleepHour = 19;
int wakeHour = 4;
int batteryMin = -1;
int levelMin = 150;
bool connectedBool = false;
String statusMessage = "";
bool eepromChanged = false;
unsigned long lastConnectionTime = 0;

// Network information
String ssid = "still_waters";
const char *password = "33turkeys511";
long timeReadChannelId = 444067;
const char *timeReadAPIKey = "NBU9ULD2EJ1ELM9T"; // time control channel

// ThingSpeak settings
char server[] = "api.thingspeak.com";
WiFiClient client;

bool connectOnWake = 1;

// #define DEVICE1
#define DEVICE2
// #define DEVICE3
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
// 5 battery level check

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

  // if (memory.connectOnWake == 0)
  // {
  //   memory.connectOnWake = 1;
  // }
  // memory.connectOnWake = 1;
  // if (memory.connectOnWake != 1)
  // {
  //   int myGuess = random(1, 1000);
  //   Serial.println("MyGuess " + String(myGuess));
  //   if (memory.connectOnWake > myGuess)
  //   {
  //     connectWiFi();
  //   }
  // }
  // else
  // { // if connectOnWake==1

  //   if ((WiFi.status() != WL_CONNECTED))
  //   {
  //     connectWiFi();
  //   }
  // }

  if ((WiFi.status() != WL_CONNECTED))
  {
    connectWiFi();
  }

  Serial.println("Connected bool" + String(connectedBool));

  if (connectedBool)
  {
    String theTime = readTSPTime();
    Serial.println(theTime);
    Serial.println("hour is " + String(hour()) + " minute is " + String(minute()));
    statusMessage += theTime + " ";
    readTSP();
  }

  int wifiStrength = getStrength(numberPoints);
  Serial.println(wifiStrength);
  int batteryLevel = getAnalog(BATT_PIN);
  int lowState = digitalRead(LEVEL_SENSE_LOW);
  int highState = digitalRead(LEVEL_SENSE_HIGH);
  int waterLevel = lowState + 10 * highState; // change to read cap

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
      httpRequest(batteryLevel, waterLevel, wifiStrength, 10, statusMessage);
    }

    getReadyToSleep(timeStatus);
  }
  // if not connected

  esp_sleep_enable_timer_wakeup(500 * 1000000); // try again in ten minutes
  Serial.println("Going to sleep now to wait for better wifi");
  Serial.flush();
  esp_deep_sleep_start();

  // if ((hour() > sleepHour) or (hour() < wakeHour))
  // {
  //   Serial.println("off work hours");
  //   long sleepTime = 3600; // 1 hour
  //   online.waterTime = 0;  // dont water at night
  //                         //  calculate the sleep time
  //                         //  if the hour is greater than sleep hour then sleep for four hours
  //                         //  if the hour is less than wake hour then sleep for one hours
  //   if (hour() > sleepHour)
  //   {

  //     sleepTime = 21600; // 6 hours
  //     Serial.println("Sleep for every " + String(sleepTime) + " Seconds");
  //   }
  //   else
  //   {
  //     // if we are before wake hour then sleep for one hour
  //     Serial.print("before work hours ");
  //     sleepTime = 3600; // 1 hour
  //     statusMessage += "Before Work, Sleeping for " + String(sleepTime) + " seconds";
  //     Serial.println("Sleep for every " + String(sleepTime) + " Seconds");
  //   }
  //   esp_sleep_enable_timer_wakeup(sleepTime * 1000000);
  //   // now write the values before sleep
  //   if (connectedBool)
  //   {
  //     httpRequest(batteryLevel, waterLevel, wifiStrength, 10, statusMessage);
  //   }

  //   Serial.flush();
  //   esp_deep_sleep_start();
  // }

  // bool batteryGood = true;
  // bool levelGood = true;
  // // fill the tank if the battery is high enough
  // //  consider also dont fill if level is above some number. Should use modes to turn on
  // Serial.println("battery level " + String(batteryLevel) + " water " + String(waterLevel));

  // // online.batteryMin = -1;
  // if (online.selectMode & (1 << 5))
  // {
  //   if (batteryLevel < online.batteryMin)
  //   {
  //     batteryGood = false;
  //     Serial.println("Battery Check fail");
  //     statusMessage += "Battery below minimum of " + String(online.batteryMin);
  //   }
  // }

  // if (online.selectMode & (1 << 4))
  // {
  //   if (waterLevel < levelMin)
  //   {
  //     levelGood = false; // we have enough water
  //   }
  //   Serial.println("Level Check fail");
  // }
  // if (batteryGood)
  // {
  //   Serial.println("Battery above set level");

  // if (online.selectMode & 1) // if sleep mode is enabled
  // {
  //   // WriteEEPROM
  //   if (eepromChanged)
  //   {
  //     EEPROMWriteAll();
  //   }
  //   esp_sleep_enable_timer_wakeup(online.sleepTime * 1000000);

  //   Serial.println("Setup ESP8266 to sleep for every " + String(online.sleepTime) +
  //                  " Seconds");

  //   Serial.println("Going to sleep now");
  //   Serial.flush();
  //   esp_deep_sleep_start();
  // }

  // only make it here if wifi connect fail
}

int getTimeStatus()
{

  // check the time and return a status
  // 0 = before work hours
  // 1 = work hours
  // 2 = after work hours
  if (hour() < wakeHour)
  {
    Serial.println("before work hours");
    return 0; // before work hours
  }

  if ((hour() > sleepHour))
  {
    Serial.println("after work hours");
    return 2; // after work hours
  }

  Serial.println("work hours");
  return 1; // work hours
}

int getPumpStatus(int batteryLevel, int waterLevel)
{
  // 0 dont pump
  // 1 pump
  int pumpStatus = 0;

  bool batteryGood = true;
  bool levelGood = true;
  // fill the tank if the battery is high enough
  //  consider also dont fill if level is above some number. Should use modes to turn on
  Serial.println("battery level " + String(batteryLevel) + " water " + String(waterLevel));

  if ((online.waterTime > 0) and (online.selectMode & (1 << 2)))
  { // make sure the control setting allows watering and water time is positive
    pumpStatus = 1;
  }

  // online.batteryMin = -1;
  if (online.selectMode & (1 << 5))
  {
    if (batteryLevel < online.batteryMin)
    {
      batteryGood = false;
      Serial.println("Battery Check fail");
      statusMessage += "Battery below minimum of " + String(online.batteryMin);
      pumpStatus = 0; // dont pump
    }
    else
    {
      Serial.println("Battery above set level");
    }
  }

  if (online.selectMode & (1 << 4))
  {
    if (waterLevel < levelMin)
    {
      levelGood = false; // we have enough water
      Serial.println("Level Check fail");
      pumpStatus = 0; // dont pump
    }
  }

  return pumpStatus;
}

void getReadyToSleep(int statusCode)
{
  int sleepTime = 0;
  if (statusCode == 0)
  { // before work hours
    sleepTime = 3600;
  }
  if (statusCode == 1)
  {                               // work hours
    sleepTime = online.sleepTime; // use the online setting
  }
  if (statusCode == 2)
  { // after work hours
    sleepTime = 3600 * 6;
  }

  if (online.selectMode & 1) // if sleep mode is enabled
  {
    // WriteEEPROM
    if (eepromChanged)
    {
      EEPROMWriteAll();
    }

    esp_sleep_enable_timer_wakeup(sleepTime * 1000000);

    Serial.println("Setup ESP8266 to sleep for every " + String(sleepTime) +
                   " Seconds");

    Serial.println("Going to sleep now");

    Serial.flush();
    esp_deep_sleep_start();
  }
}