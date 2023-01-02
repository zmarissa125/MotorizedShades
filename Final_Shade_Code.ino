// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP_EEPROM.h>
#endif
#ifdef ESP32
#include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProBlinds.h"

#define WIFI_SSID         //ADD HERE
#define WIFI_PASS         //ADD HERE
#define APP_KEY           //ADD HERE
#define APP_SECRET        //ADD HERE
#define BLINDS_ID         //ADD HERE
#define BAUD_RATE         9600

#define MAXLOC 60
#define MIDLOC 30
#define MINLOC 0


// NODEMCU PINS:
const int encode1 = 4;
const int encode2 = 5;
const int muxPins[3] = {
  16, // s0
  0, // s1
  2, // s2
};
const int motorSig[2][2] = {
  {14, 12},
  {13, 15}
};

const int led = 1;

// MUX PINS:
const int top = 5;
const int mid = 4;
const int bot = 3;
const int pwr = 2;
const int cs1 = 1;
const int cs2 = 0;
bool changed = false;

// ------------------------------------------------------------------------------------------------------------------
// Analog pin muliplexor setup

const int muxChannel[8][4] = {
  {0, 0, 0, 0}, //channel 0
  {1, 0, 0, 0}, //channel 1
  {0, 1, 0, 0}, //channel 2
  {1, 1, 0, 0}, //channel 3
  {0, 0, 1, 0}, //channel 4
  {1, 0, 1, 0}, //channel 5
  {0, 1, 1, 0}, //channel 6
  {1, 1, 1, 0}, //channel 7
};

int readMux(int channel, bool digital) {
  for (int i = 0; i < 3; i ++) {
    digitalWrite(muxPins[i], muxChannel[channel][i]);
  }
  //read the value at the SIG pin
  if (digital) {
    return (analogRead(A0) > 700) ? 1 : 0;
  }
  return analogRead(A0);
}
// ------------------------------------------------------------------------------------------------------------------
// Track shades location

struct LocationStruct {
  int shade1, shade2;
} locations;

volatile int shade1Step = 0;
volatile int shade2Step = 0;
void ICACHE_RAM_ATTR shade1Count() {
  shade1Step ++;
}
void ICACHE_RAM_ATTR shade2Count() {
  shade2Step ++;
}


// ------------------------------------------------------------------------------------------------------------------
// Power outage protections and location retrieval on startup
bool outageCheck(){
  if (readMux(pwr, false) < 700 && changed){
    //detachInterrupt(digitalPinToInterrupt(encode1));
    //detachInterrupt(digitalPinToInterrupt(encode2));
    EEPROM.put(0, locations);
    EEPROM.commit();
    changed = false;
    }
  }
// ------------------------------------------------------------------------------------------------------------------
// Shade Control
struct moveData {
  int shade1Pin, shade2Pin, stepNeed1, stepNeed2, dir1, dir2;
};

struct moveData calcShade(int newLoc) {
  moveData newData;
  newData.shade1Pin = motorSig[0][newLoc - locations.shade1 > 0]; //turn direction
  newData.shade2Pin = motorSig[1][newLoc - locations.shade2 > 0]; //turn direction 2
  newData.dir1 = (newLoc - locations.shade1 > 0) ? 1 : -1;
  newData.dir2 = (newLoc - locations.shade2 > 0) ? 1 : -1;
  newData.stepNeed1 = abs(newLoc - locations.shade1);
  newData.stepNeed2 = abs(newLoc - locations.shade2);
  if (newLoc<20 && newLoc >= 0){newData.stepNeed2 += (2 * newData.dir2 * -1);} //offsets
  else if (newLoc <= 40){newData.stepNeed2 += (newData.dir2 * -1);}
  return newData;
}

bool moveShade(int newLoc) {
  moveData actionData = calcShade(newLoc);
  SinricProBlinds &myBlinds = SinricPro[BLINDS_ID];
  //Serial.print("\nSteps needed: ");
  //Serial.println(actionData.stepNeed1);
  //Serial.println(actionData.dir1);
  shade1Step = 0, shade2Step = 0;
  digitalWrite(actionData.shade1Pin, HIGH);
  digitalWrite(actionData.shade2Pin, HIGH);
  while (shade1Step != actionData.stepNeed1 || shade2Step != actionData.stepNeed2) {
    if (shade1Step >= actionData.stepNeed1) {digitalWrite(actionData.shade1Pin, LOW);}
    if (shade2Step >= actionData.stepNeed2) {digitalWrite(actionData.shade2Pin, LOW);}
    if (readMux(cs1, false) > 250) || readMux(cs2, false) > 250) {
      digitalWrite(led, HIGH);
      //Serial.println("Current High");
      break;}
    //if (readMux(pwr, false) < 700) {break;}
    delay(100);
  }
  digitalWrite(actionData.shade1Pin, LOW);
  digitalWrite(actionData.shade2Pin, LOW);
  changed = true;
  locations.shade1 += shade1Step * actionData.dir1; //update to new positions
  locations.shade2 += shade2Step * actionData.dir2;
  return true;
}

// ------------------------------------------------------------------------------------------------------------------
// Wifi Control events

bool onRangeValue(const String &deviceId, int &position) {
  //Serial.printf("Device %s set position to %d\r\n", deviceId.c_str(), position);
  return moveShade(map(abs(position), 0, 100, MINLOC, MAXLOC));
  //return true; // request handled properly
}

bool onAdjustRangeValue(const String &deviceId, int &positionDelta) {
  //Serial.printf("Device %s position changed about %i to %d\r\n", deviceId.c_str(), positionDelta, locations.shade1);
  return moveShade(locations.shade1 + (positionDelta / 100) * locations.shade1);
  //return true; // request handled properly
}

// ------------------------------------------------------------------------------------------------------------------
// Button control
void buttonCheck() {
  if (readMux(top, true)) {
    //Serial.println("top");
    moveShade(MINLOC);
  }
  else if (readMux(mid, true)) {
    //Serial.println("mid");
    moveShade(MIDLOC);
  }
  else if (readMux(bot, true)) {
    //Serial.println("bot");
    moveShade(MAXLOC);
  }
}

// ------------------------------------------------------------------------------------------------------------------
// setup function for WiFi connection
void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(20);
    //Serial.print(".");
    checkWifi(); //need delay 20
  }
  
  IPAddress localIP = WiFi.localIP();
  //Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

volatile int passes = 0;
void checkWifi(){
  if (WiFi.status() != WL_CONNECTED){passes += 1;}
  else {digitalWrite(led, LOW);}
  if (passes == 15){digitalWrite(led, HIGH);} // Blink LED when connected
  else if (passes == 30){
    passes = 0; 
    digitalWrite(led, LOW);
  }
}

void setupSinricPro() {
  // get a new Blinds device from SinricPro
  SinricProBlinds &myBlinds = SinricPro[BLINDS_ID];
  myBlinds.onRangeValue(onRangeValue);
  myBlinds.onAdjustRangeValue(onAdjustRangeValue);

  // setup SinricPro  
  SinricPro.onConnected([]() {
    //Serial.printf("Connected to SinricPro\r\n");
  });
  SinricPro.onDisconnected([]() {
    //Serial.printf("Disconnected from SinricPro\r\n");
  });
  SinricPro.begin(APP_KEY, APP_SECRET);
  myBlinds.sendPowerStateEvent(true);
}

// ------------------------------------------------------------------------------------------------------------------
// main setup and loop
void setup() {
  //Serial.begin(BAUD_RATE);
  //Serial.printf("\r\n\r\n");
  
  EEPROM.begin(sizeof(LocationStruct));
  if(EEPROM.percentUsed()>=0){
     EEPROM.get(0, locations);
     //Serial.println("EEPROM has data from a previous run.");
     //Serial.print(EEPROM.percentUsed());
     //Serial.println("% of ESP flash space currently used");
  }
  //Serial.printf("Shade1: %d\n", locations.shade1);
  //Serial.printf("Shade2: %d\n", locations.shade2);
  pinMode(led, FUNCTION_3);
  pinMode(led,OUTPUT);
  pinMode(encode1, INPUT);
  pinMode(encode2, INPUT);
  for (int i = 0; i < 3; i++){
    pinMode(muxPins[i], OUTPUT);
  }

  for (int i = 0; i < 2; i++){
    pinMode(motorSig[i][0], OUTPUT);
    pinMode(motorSig[i][1], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(encode1), shade1Count, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encode2), shade2Count, CHANGE);
  setupWiFi();
  setupSinricPro();
}

void loop() {
  SinricPro.handle();
  buttonCheck();
  outageCheck();
  delay(20);

}
