#include <WiFi.h>
#include "ThingSpeak.h"
#include <ArduinoJson.h>
#include <Time.h>
#include <TimeLib.h>
#include "EEPROM.h"
#include <ESP32Servo.h>
#include <analogWrite.h>

#define DEVICE1
//#define DEVICE2
//#define DEVICE3
//#define DEVICE4
//#define DEVICEDUCK

#define SERVO_PIN 25
#define SERVO_ENABLE 5
#define CLOSE_POS 12
#define OPEN_POS 135

#define buttonPin 0
#define LEDPin 2
#define TIMEOUT 5000
#define BATT_PIN 35 //Changed on accident!! reset for next program
#define LEVEL_PIN 34
#define LEVEL_CONTROL_PIN 33
#define PUMP_PIN 27
#define TIME_TO_SLEEP 3600
#define STRSTART 17
#define STRLEN 54
#define TRIG 23
#define ECHO 4
#define DRAIN_PUMP 14
#define EMPTY_PUMP_PIN 14
#define FILL_SERVO_OPEN 17
#define FILL_SERVO_CLOSE 16
#define WATER_ADDRESS 2
#define SLEEP_ADDRESS 4
#define MODE_ADDRESS 6
#define FILL_TIME_ADDRESS 8
#define CONNECT_ON_WAKE_ADDRESS 10
#define BATTERY_ADDRESS 12
#define LEVEL_ADDRESS 14
#define PUMP2_TIME_ADDRESS 16
#define LASER_MAX 1000
#define TRIG 23
#define ECHO 4
#define SERVO_BUTTON_PIN 19

struct controlData {
  int waterTime;
  int sleepTime;
  int selectMode;;
  int fillPondHour;
  int batteryMin;
  int levelMin;
  int connectOnWake;
};

struct controlData memory;
struct controlData online;

Servo myservo;
int timeOffset = 4;
const char * timeSub; //time read from HTTP response
int sleepHour = 19;
int wakeHour = 4;
int batteryMin = -1;
int levelMin = 150;
bool connectedBool = false;
String statusMessage = "";
bool eepromChanged = false;

// Network information
char * ssid = "still_waters";
const char * password = "33turkeys511";

// ThingSpeak settings
char server[] = "api.thingspeak.com";

bool connectOnWake = 1;

//This is a test area
//const char * writeAPIKey = "H6EVSHKVVKR8KNJP"; //battery channel
//const char * readAPIKey = "VV1UZ2CRNCUJ8VER"; //control channel
//long readChannelId=597924;
//long myChannelNumber=983897;

#ifdef DEVICE1
const char * writeAPIKey = "H6EVSHKVVKR8KNJP"; //battery channel
const char * readAPIKey = "VV1UZ2CRNCUJ8VER"; //control channel
long readChannelId=597924;
long myChannelNumber=592680;
#endif

#ifdef DEVICE2
const char * writeAPIKey ="VML7XDJTAELWPWH8"; //second battery channel
long myChannelNumber=844390; //channel 2
const char * readAPIKey = "OJXNM9UUX2V6P8UD"; //second control channel
long readChannelId=844392; //channel2
#endif

#ifdef DEVICE3
const char * writeAPIKey ="0L6OFFAKEQTBBNBJ"; //third battery channel
long myChannelNumber=1005405; //channel 3
const char * readAPIKey = "BMKL2AZ5SB4SUNFZ"; //third control channel
long readChannelId=1005406; //channel2
#endif

#ifdef DEVICE4
const char * writeAPIKey ="DXKLMJ1W7WMYDD8D"; //lone battery channel
long myChannelNumber=10505221; //channel 4
const char * readAPIKey = "P5SQIBO7FZNH9XBV"; //lone control channel
long readChannelId=1050522; //channel2
#endif

#ifdef DEVICEDUCK
const char * writeAPIKey = "MEC37ONNJUT84A4U"; //Ducktank channel
long myChannelNumber = 1160823; //channel 4
const char * readAPIKey = "CN409WRNXUVJYKGQ"; //Ducktank control channel
long readChannelId = 1160824; //channel2
#endif

unsigned long lastConnectionTime = 0;
int measurementNumber = 0;
int servoPosition = 0;
int servoFlag = 0;
long lastTrigger = 0;
int intCounter = 0;

WiFiClient client;

//interrupt for servo button 
void IRAM_ATTR servoInterrupt() {
  if ((millis() - lastTrigger) > 400) {
    servoFlag = 1;
    intCounter++;
    Serial.println("Interrupt!");
    lastTrigger = millis();
  }
}

void setup() {

  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
  pinMode(BATT_PIN, INPUT);
  pinMode(LEDPin, OUTPUT);
  pinMode(DRAIN_PUMP, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LEVEL_PIN, INPUT);
  pinMode(LEVEL_CONTROL_PIN, OUTPUT);
  pinMode(EMPTY_PUMP_PIN, OUTPUT);
  pinMode(FILL_SERVO_OPEN, OUTPUT);
  pinMode(FILL_SERVO_CLOSE, OUTPUT);
  pinMode(TRIG, OUTPUT); // Initializing Trigger Output and Echo Input
  pinMode(ECHO, INPUT_PULLUP);
  pinMode(SERVO_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_ENABLE, OUTPUT);
  // connectWiFi();
  EEPROM.begin(16);
  //SERVO SETUP

  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50); // standard 50 hz servo
  myservo.attach(SERVO_PIN, 1000, 2000);
  attachInterrupt(digitalPinToInterrupt(SERVO_BUTTON_PIN), servoInterrupt, FALLING);
}
//memory map
//water
//sleep
//mode
// fill hour
//connecton wake
// Battery min
// Level Min

//selectMode bits 
//1 sleep enable (and EEPROM write)
//2 pump enable
//3 overnight sleep
//4 level check
//5 battery level check

void loop() {
  blinkX(2, 150);

  const int numberPoints = 3;
  //String waterTime,sleepTime,selectModeString,fillPondHourString,batteryString,levelString;

  readEEPROM();
  //if mode indicates connect every time
  //otherwise mode indicate random percent to try connect
  if (memory.connectOnWake == 0) {
    memory.connectOnWake = 1;
  }
  memory.connectOnWake = 1;
  if (memory.connectOnWake != 1) {
    int myGuess = random(1, 1000);
    Serial.println("MyGuess " + String(myGuess));
    if (memory.connectOnWake > myGuess) {
      connectWiFi();
    }
  } else { //if connectOnWake==1

    if ((WiFi.status() != WL_CONNECTED)) {
      connectWiFi();
    }
  }
  Serial.println("Connected bool" + String(connectedBool));

  if (connectedBool) {
    readTSP();
  }

  int wifiStrength = getStrength(numberPoints);
  Serial.println(wifiStrength);
  int batteryLevel = getAnalog(BATT_PIN);

  int waterLevel = 10; //change to read cap

  //now write the values
  if (connectedBool) {
    httpRequest(batteryLevel, waterLevel, wifiStrength, 10);
  }

  //check servo interrupt
  if (servoFlag == 1) {
    moveServo();
    servoFlag = 0;
  }

  if ((online.selectMode & (1 << 3)) and(connectedBool)) { //can't check time if we arent connected
    Serial.println("Time Check");
    //do stuff based on time
    if (hour() < wakeHour) {
      openServo();
      delay(100);
    }
    if (hour() > sleepHour) {
      closeServo();
      delay(100);
    }
    if ((hour() > sleepHour) or(hour() < wakeHour)) {

      online.waterTime = 0; //dont water at night
      long sleepTime = 14400;
      Serial.println("off work hours");
      esp_sleep_enable_timer_wakeup(sleepTime * 1000000 ULL);

      Serial.println("Sleep for every " + String(sleepTime) + " Seconds");

      Serial.flush();
      esp_deep_sleep_start();
    }
  }

  bool batteryGood = true;
  bool levelGood = true;
  //fill the tank if the battery is high enough
  // consider also dont fill if level is above some number. Should use modes to turn on
  Serial.println("battery level" + String(batteryLevel) + " water " + String(waterLevel));
  //online.batteryMin = -1;
  if (online.selectMode & (1 << 5)) {
    if (batteryLevel < online.batteryMin) {
      batteryGood = false;
      Serial.println("Battery Check");
    }
  }


  if (online.selectMode & (1 << 4)) {
    if (waterLevel < levelMin) {
      levelGood = false; //we have enough water
    }
    Serial.println("Level Check");
  }
  if (batteryGood) {
    Serial.println("Battery above set level");
    if (levelGood) {
      Serial.println("Water level more than minimum");
      //int wTime = waterTime.toInt();
      if ((online.waterTime > 0) and(online.selectMode & (1 << 2))) {
        Serial.println("filling tank length= " + online.waterTime);
        runPump(PUMP_PIN, online.waterTime); // time in seconds
        delay(5);
      }
    }
  }
  delay(1000);

  if (online.selectMode & 1) {
    //WriteEEPROM
    if (eepromChanged) {
      EEPROMWriteAll();
     
    }
    esp_sleep_enable_timer_wakeup(online.sleepTime * 1000000 ULL);

    Serial.println("Setup ESP8266 to sleep for every " + String(online.sleepTime) +
      " Seconds");

    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
  }
  //only make it here if wifi connect fail
       esp_sleep_enable_timer_wakeup(500 * 1000000ULL); //try again in ten minutes
       Serial.println("Going to sleep now to wait for better wifi");
      Serial.flush();
      esp_deep_sleep_start();  
}

int runPump(int myPin, int timeOn) {
  digitalWrite(myPin, HIGH);
  Serial.println("Pump on");
  delay(timeOn * 1000);
  digitalWrite(PUMP_PIN, LOW);
  Serial.println("Pump off");
}

void connectWiFi() {
  int tryCounter = 0;
  WiFi.begin(ssid, password);
  while ((WiFi.status() != WL_CONNECTED)) {
    if (tryCounter > 5) {
      connectedBool = false;
      online=memory;
      // esp_sleep_enable_timer_wakeup(500 * 1000000ULL); //try again inten minutes
       //Serial.println("Going to sleep now");
      //Serial.flush();
      //esp_deep_sleep_start();
      return;
    }

    delay(7500);
    Serial.println("Connecting to WiFi");
    tryCounter++;
  }
  Serial.println("Connected");
  connectedBool = true;
  ThingSpeak.begin(client);
  tryCounter = 0;
  blinkX(5, 100);
}

void httpRequest(long field1Data, int field2Data, int field3Data, int field4Data) {

  if (!client.connect(server, 80)) {

    Serial.println("Connection failed");
    lastConnectionTime = millis();
    client.stop();
    return;
  } else {

    // Create data string to send to ThingSpeak.
    String data = "field1=" + String(field1Data) + "&field2=" + String(field2Data) + "&field3=" + String(field3Data) + "&field4=" + String(field4Data) + ".json"; //shows how to include additional field data in http post

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
    parseTime(answer.substring(STRSTART, STRLEN));

  }
  client.stop();

}

// Take measurements of the Wi-Fi strength and return the average result.
int getStrength(int points) {
  long rssi = 0;
  long averageRSSI = 0;

  for (int i = 0; i < points; i++) {
    rssi += WiFi.RSSI();
    delay(20);
  }

  averageRSSI = rssi / points;
  return averageRSSI;
}

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

String getCall(int myField) {
  if (!client.connect(server, 80)) {

    Serial.println("Connection failed");
    lastConnectionTime = millis();
    client.stop();
    return "Connection failed";
  }

  String url = "http://api.thingspeak.com/channels/" + String(readChannelId) + "/fields/" + String(myField) + "/last.txt?api_key=" + String(readAPIKey);
  Serial.println(url);
  client.println(String("GET ") + url);

  unsigned long timeout = millis();
  String answer = getResponse();
  Serial.print("answer is ");
  Serial.println(answer);
  return answer;

}

// Wait for a response from the server indicating availability,
// and then collect the response and build it into a string.

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

int getAnalog(int pin) {
  long value = analogRead(pin);
  return value;
}

int parseTime(String input) {
  Serial.println("time input is ");
  Serial.println(input);
  // char jsonObject[1024];
  //StaticJsonBuffer<512> jsonBuffer;

  //JsonObject& root=jsonBuffer.parseObject(input);
  //if (!root.success()) {
  //Serial.println( "parseObject() failed");
  //return 0;
  // }
  //timeSub= root["created_at"];
  //String timeString=String(timeSub);
  char charMonth[4];
  long timeYear = input.substring(19, 23).toInt();
  String strMonth = input.substring(15, 17);
  static
  const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  sprintf(charMonth, "%s", strMonth);
  int timeMonth = (strstr(month_names, charMonth) - month_names) / 3 + 1;

  int timeDay = input.substring(11, 14).toInt();
  //Serial.println(String(timeDay));
  int timeHour = input.substring(23, 25).toInt();
  Serial.println(String(timeHour));
  int timeMin = input.substring(26, 28).toInt();
  Serial.println(input.substring(26, 28));
  int timeSec = input.substring(29, 31).toInt();

  setTime(timeHour, timeMin, timeSec, timeDay, timeMonth, timeYear);
  adjustTime(-timeOffset * 3600);
  Serial.println("time is " + String(hour()) + ":" + String(minute()) + ":" + String(second()) + "," + " " + String(month()) + " " + String(day()) + " " + String(year()));

}
/*
int getLaser(){
    VL53L0X_RangingMeasurementData_t measure;
    digitalWrite (LASER_SHDWN, HIGH);
    // delay(100);
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X"));
        return 0;
    }
   
    delay(1000);
    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
   
    if (measure.RangeStatus != 4) { // phase failures have incorrect data
        int myDistance = measure.RangeMilliMeter; digitalWrite (LASER_SHDWN, LOW);
        if (myDistance>LASER_MAX){
            myDistance=LASER_MAX;
        }
        Serial.print("Distance (mm): "); Serial.println(myDistance);
        return myDistance;
        } else {
        Serial.println(" out of range ");
        digitalWrite (LASER_SHDWN, LOW);
        return 0;
    }
}
*/

void runPondFillTest() {
  Serial.println("Fill Test");
  digitalWrite(FILL_SERVO_CLOSE, LOW);
  digitalWrite(FILL_SERVO_OPEN, HIGH);
  delay(2000);
  digitalWrite(FILL_SERVO_CLOSE, HIGH);
  digitalWrite(FILL_SERVO_OPEN, LOW);
  delay(2000);
  digitalWrite(FILL_SERVO_CLOSE, LOW);
  digitalWrite(FILL_SERVO_OPEN, HIGH);
  delay(2000);
  digitalWrite(FILL_SERVO_CLOSE, HIGH);
  digitalWrite(FILL_SERVO_OPEN, LOW);
  delay(2000);
}

void runPondEmptyTest() {
  Serial.println("empty now");
  statusMessage += " empty test";
  digitalWrite(EMPTY_PUMP_PIN, HIGH);
  Serial.println("Empty pump High");
  delay(2000);
  digitalWrite(EMPTY_PUMP_PIN, LOW);
  Serial.println("Empty pump low");
  delay(2000);
  digitalWrite(EMPTY_PUMP_PIN, HIGH);
  Serial.println("Empty pump High");
  delay(2000);
  digitalWrite(EMPTY_PUMP_PIN, LOW);
  Serial.println("Empty pump low");
  delay(2000);

}

int getUltraDist(int trig, int echo) {
  float distance = 0;
  float sumDistance = 0;
  int n = 5;
  for (int i = 0; i < n; i++) {

    digitalWrite(trig, LOW); // Set the trigger pin to low for 2uS
    delayMicroseconds(2);

    digitalWrite(trig, HIGH); // Send a 10uS high to trigger ranging
    delayMicroseconds(20);

    digitalWrite(trig, LOW); // Send pin low again
    int distance = pulseIn(echo, HIGH, 26000); // Read in times pulse

    distance = distance / 58; //Convert the pulse duration to distance
    //You can add other math functions to calibrate it well

    Serial.print("Distance ");
    Serial.print(distance);
    Serial.println("cm");
    sumDistance = sumDistance + distance / float(n);
  }

  return sumDistance;
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

void readTSP() {

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
  // Serial.println("fill mem " +String(online.fillPondHour));
}

void openServo() {
  Serial.println(" Open servo ");
  int pos;
  digitalWrite(SERVO_ENABLE, HIGH);
  delay(200);
  for (pos = CLOSE_POS; pos <= OPEN_POS; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos); // tell servo to go to position in variable 'pos'
    delay(50); // waits 15ms for the servo to reach the position
  }

  digitalWrite(SERVO_ENABLE, LOW);
  delay(200);
  servoPosition = 1;

}

void closeServo() {
  Serial.println(" Close servo");
  int pos;
  digitalWrite(SERVO_ENABLE, HIGH);
  delay(200);

  for (pos = OPEN_POS; pos >= CLOSE_POS; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos); // tell servo to go to position in variable 'pos'
    delay(50); // waits 15ms for the servo to reach the position
  }
  digitalWrite(SERVO_ENABLE, LOW);
  delay(200);

  servoPosition = 0;
}

void moveServo() {
  if (intCounter % 2 == 1) {
    blinkX(10, 100);
    openServo();
  } else {
    blinkX(2, 600);
    closeServo();

  }
  servoFlag = 0;
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
      EEPROM.write(BATTERY_ADDRESS + 1, online.batteryMin);
      EEPROM.write(LEVEL_ADDRESS, online.levelMin >> 8);
      EEPROM.write(LEVEL_ADDRESS + 1, online.levelMin & 0xFF);
      EEPROM.commit();
}
