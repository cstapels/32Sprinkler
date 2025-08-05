#pragma once

#include <Arduino.h>

// --- Function Prototypes ---
void blinkX(int numTimes, int delayTime);
int getAnalog(int pin);
int getStrength(int points);
int runPump(int myPin, int timeOn);
void readEEPROM();
void EEPROMWriteAll();