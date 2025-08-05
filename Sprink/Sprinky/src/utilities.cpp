#include "utilities.h"
#include "constants.h" // For LEDPin
#include <WiFi.h>      // For WiFi.RSSI()
#include "EEPROM.h"
#include <Arduino.h>
#include <Time.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include "communication.h"

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