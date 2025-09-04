#include <Arduino.h>
#include <WiFi.h>
#include <Time.h>
#include "communication.h"
#include "constants.h"

#include "utilities.h"
#include <TimeLib.h>
#include "ThingSpeak.h"

unsigned long lastConnectionTime = 0;

void connectWiFi() {
  int tryCounter = 0;  
  Serial.println("Connecting to WiFi...");

  // Set WiFi to station mode and disconnect from an AP if it was previously connected.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Boost WiFi power to maximum. Helpful in weak signal areas.
  // Note: This increases power consumption.
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.begin(ssid, password);

 
  while (WiFi.status() != WL_CONNECTED) {
    if (tryCounter >= 6) {
      Serial.println("\nFailed to connect to WiFi.");
      connectedBool = false;
      online = memory; // Fallback to EEPROM values if connection fails
      return;
    }
    delay( WIFI_RETRY_SLEEP_S *1000);
    Serial.print(".");
    blinkX(1, 100);
    tryCounter++;
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  connectedBool = true;
  ThingSpeak.begin(client);
  blinkX(5, 100);
}

void readTSP() {
  //read the control channel
  int statusCode = ThingSpeak.readMultipleFields(readChannelId, readAPIKey);

  if (statusCode == 200) {
    online.waterTime = ThingSpeak.getFieldAsInt(1);
    online.sleepTime = ThingSpeak.getFieldAsInt(2);
    online.selectMode = ThingSpeak.getFieldAsInt(3);
    online.fillPondHour = ThingSpeak.getFieldAsInt(4);
    online.batteryMin = ThingSpeak.getFieldAsInt(5);
    online.levelMin = ThingSpeak.getFieldAsInt(6);
    online.connectOnWake = ThingSpeak.getFieldAsInt(7);
    if (online.waterTime != memory.waterTime) {
      eepromChanged = true;
    }
    if (online.sleepTime != memory.sleepTime) {
      eepromChanged = true;
    }
    if (online.selectMode != memory.selectMode) {
      eepromChanged = true;
    }
    if (online.fillPondHour != memory.fillPondHour) {
      eepromChanged = true;
    }
    if (online.batteryMin != memory.batteryMin) {
      eepromChanged = true;
    }
    if (online.levelMin != memory.levelMin) {
      eepromChanged = true;
    }
    if (online.connectOnWake != memory.connectOnWake) {
      eepromChanged = true;
    }
  }
      Serial.println("Water Time " + String(online.waterTime));
  Serial.println("Sleep Time " + String(online.sleepTime));
  Serial.println("Mode is " + String(online.selectMode));
  Serial.println("fill hour = " + String(online.fillPondHour));
  Serial.println("Batt Min " + String(online.batteryMin));
  Serial.println("level Min " + String(online.levelMin));
  Serial.println("Connect On Wake " + String(online.connectOnWake));
}

String readTSPTime() {
  //assume we are already connected
  // The default ThingSpeak timeout is 5000ms. We'll increase it to our custom value.
 
  int statusCode = ThingSpeak.readMultipleFields(timeReadChannelId, timeReadAPIKey);


  if (statusCode == 200) {
    int myYear = ThingSpeak.getFieldAsInt(1);
    int myMonth = ThingSpeak.getFieldAsInt(2);
    int myDay = ThingSpeak.getFieldAsInt(3);
    int myHour = ThingSpeak.getFieldAsInt(4);
    int myMin = ThingSpeak.getFieldAsInt(5);
    int mySecond = ThingSpeak.getFieldAsInt(6);

    setTime(myHour, myMin, mySecond, myDay, myMonth, myYear);

    String timeString = String(myYear) + String("-") + String(myMonth) + "-" + String(myDay) + " " + String(myHour) + ":" + String(myMin) + ":" + String(mySecond);

    return timeString;
  }
  return ("time Failed");
}

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}



void httpRequest(long field1Data, int field2Data, int field3Data, int field4Data, String status) {
 if (!client.connect(server, 80)) {

    Serial.println("Connection failed");
    lastConnectionTime = millis();
    client.stop();
    return;
  } else {

    // Create data string to send to ThingSpeak.
    String data = "field1=" + String(field1Data) + "&field2=" + String(field2Data) + "&field3=" + String(field3Data) + "&field4=" + String(field4Data) + "&status=" + status + ".json"; //shows how to include additional field data in http post

    // POST data to ThingSpeak.
    if (client.connect(server, 80)) {

      client.println("POST /update HTTP/1.1");
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("User-Agent: ESP32WiFi/1.1");
      client.println("X-THINGSPEAKAPIKEY: " + String(writeAPIKey));
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.print(data.length());
      client.print("\n\n");
      client.print(data);

      // Serial.println("RSSI = " + String(field1Data));
      lastConnectionTime = millis();
    }
    //getCall();
    String answer = getResponse();
    // Serial.println("here is the answer" + answer );
    //get the time out of the response
    // Serial.println("here is the short part "+ answer.substring(STRSTART, STRLEN));
    //send to json parser to get time
    //parseTime(answer.substring(STRSTART, STRLEN));

  }
  client.stop();

}

String getResponse() {
  String response;
  long startTime = millis();

  delay(200);
  while (client.available() < 1 & ((millis() - startTime) < TIMEOUT)) {
    delay(5);
  }

  if (client.available() > 0) { // Get response from server.
    char charIn;
    do {
      charIn = client.read(); // Read a char from the buffer.
      response += charIn; // Append the char to the string response.
    } while (client.available() > 0);
  }
  client.stop();

  return response;
}

