#pragma once

#include <Arduino.h>
#include "types.h" // For the controlData struct
#include <Time.h>
#include <TimeLib.h>

// By declaring these as 'extern', we tell the compiler that these variables
// are defined in another source file (in this case, main.cpp) and will be
// available at link time.
extern WiFiClient client;
extern bool connectedBool;
extern struct controlData online;
extern struct controlData memory;
extern bool eepromChanged;

// Network and API credentials
extern String ssid;
extern const char* password;
extern const char* writeAPIKey;
extern const char* readAPIKey;
extern long readChannelId;
extern const char* timeReadAPIKey;
extern long timeReadChannelId;
extern char server[];

// --- Function Prototypes ---
void connectWiFi();
void readTSP();
String readTSPTime();
void httpRequest(long field1Data, int field2Data, int field3Data, int field4Data, String status);
String getResponse();
int32_t getWiFiChannel(const char* ssid);