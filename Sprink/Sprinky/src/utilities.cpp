#include "utilities.h"
#include "constants.h" // For LEDPin
#include <WiFi.h>      // For WiFi.RSSI()
#include "EEPROM.h"
#include <Arduino.h>
#include <Time.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include "communication.h"

extern String statusMessage;

int sleepHour = 19;
int wakeHour = 4;
int batteryMin = -1;
int levelMin = 150;

// Make the LED blink a variable number of times with a variable delay.
void blinkX(int numTimes, int delayTime) {
  for (int g = 0; g < numTimes; g++) {
    // Turn the LED on and wait.
    digitalWrite(LEDPin, HIGH);
    delay(delayTime);

    // Turn the LED off and wait.
    digitalWrite(LEDPin, LOW);
    delay(delayTime);
  }
}

int getAnalog(int pin) {
  long value = analogRead(pin);
  return value;
}

// Take measurements of the Wi-Fi strength and return the average result.
int getStrength(int points) {
  // Return a default value if WiFi is not connected to avoid blocking or returning an ambiguous '0'.
  if (WiFi.status() != WL_CONNECTED) {
    return -100; // A typical "no signal" RSSI value
  }

  long rssi = 0;
  for (int i = 0; i < points; i++) {
    rssi += WiFi.RSSI();
    delay(20);
  }

  return rssi / points;
}

int runPump(int myPin, int timeOn) {
  Serial.println("filling tank length= " + online.waterTime);
  digitalWrite(myPin, HIGH);
  digitalWrite(LEDPin, HIGH);
  Serial.println("Pump on");
  delay(timeOn * 1000);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(LEDPin, LOW);
  Serial.println("Pump off");
  return 1;
}

void readEEPROM() {
  memory.waterTime = (EEPROM.read(WATER_ADDRESS) << 8) + EEPROM.read(WATER_ADDRESS + 1);
  Serial.println("water time " + String(memory.waterTime) + "first part " + String(EEPROM.read(WATER_ADDRESS) << 8) + "SP " + String(EEPROM.read(WATER_ADDRESS + 1)));
  memory.sleepTime = (EEPROM.read(SLEEP_ADDRESS) << 8) + EEPROM.read(SLEEP_ADDRESS + 1);
  Serial.println("sleep time " + String(memory.sleepTime));
  memory.selectMode = (EEPROM.read(MODE_ADDRESS) << 8) + EEPROM.read(MODE_ADDRESS + 1);
  Serial.println("mode " + String(memory.selectMode));
  memory.fillPondHour = EEPROM.read(FILL_TIME_ADDRESS);
  Serial.println("fill hour " + String(memory.fillPondHour));
  memory.connectOnWake = EEPROM.read(CONNECT_ON_WAKE_ADDRESS);
  Serial.println("conectonwake " + String(memory.connectOnWake));
  memory.batteryMin = (EEPROM.read(BATTERY_ADDRESS) << 8) + EEPROM.read(BATTERY_ADDRESS + 1);
  Serial.println("batt min eeprom " + String(memory.batteryMin));
  memory.levelMin = (EEPROM.read(LEVEL_ADDRESS) << 8) + EEPROM.read(LEVEL_ADDRESS + 1);
  Serial.println("level min eeprom " + String(memory.levelMin));

}



void EEPROMWriteAll(){
 Serial.println("Writing EEPROM");
      EEPROM.write(WATER_ADDRESS, online.waterTime >> 8);
      EEPROM.write(WATER_ADDRESS + 1, online.waterTime & 0xFF);
      EEPROM.write(SLEEP_ADDRESS, online.sleepTime >> 8);
      EEPROM.write(SLEEP_ADDRESS + 1, online.sleepTime & 0xFF);
      EEPROM.write(MODE_ADDRESS, online.selectMode >> 8);
      EEPROM.write(MODE_ADDRESS + 1, online.selectMode & 0xFF);
      EEPROM.write(FILL_TIME_ADDRESS, online.fillPondHour);
      EEPROM.write(CONNECT_ON_WAKE_ADDRESS, online.connectOnWake);
      EEPROM.write(BATTERY_ADDRESS, online.batteryMin >> 8);
      EEPROM.write(BATTERY_ADDRESS + 1, online.batteryMin & 0xFF);
      EEPROM.write(LEVEL_ADDRESS, online.levelMin >> 8);
      EEPROM.write(LEVEL_ADDRESS + 1, online.levelMin & 0xFF);
      EEPROM.commit();
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
    if (statusCode == 3)  //time chack failed
  { // after work hours
    sleepTime = 300;
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

int getTimeStatus()
{

  // check the time and return a status
  // 0 = before work hours
  // 1 = work hours
  // 2 = after work hours
//  3 time fail, sleep for a little while
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