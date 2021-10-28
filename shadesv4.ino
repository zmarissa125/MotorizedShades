#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsClient.h> // get it from https://github.com/Links2004/arduinoWebSockets/releases 
#include <ArduinoJson.h> // get it from https://arduinojson.org/ or install via Arduino library manager
#include <StreamString.h>
#include <AceButton.h>
#include <ESP_EEPROM.h>

using namespace ace_button;

ESP8266WiFiMulti WiFiMulti;
WebSocketsClient webSocket;
WiFiClient client;

#define MyApiKey  // TODO: Change to your sinric API Key. Your API Key is displayed on sinric.com dashboard
#define MySSID  // TODO: Change to your Wifi network SSID
#define MyWifiPassword  // TODO: Change to your Wifi network password
#define shadesSinricID  // TODO: Change to your sinric shades ID
#define HEARTBEAT_INTERVAL 300000 // 5 Minutes

int completelyUp = 0;
int completelyDown = 14;
int midway = 7;

int shade1c = 0;
int shade1cc = 5;
int shade2c = 2;
int shade2cc = 15;

int anal = A0;


int b_1 = 14;
int b_2 = 12;
int b_3 = 13;
int ledPin = 16;

int shade1Encoder = 4;
int shade2Encoder = 10;


AceButton button1(b_1);
AceButton button2(b_2);
AceButton button3(b_3);

void buttonHandler(AceButton*, uint8_t, uint8_t);


uint64_t heartbeatTimestamp = 0;
bool isConnected = false;

void setPowerStateOnServer(String deviceId, String value);

// deviceId is the ID assgined to your smart-home-device in sinric.com dashboard. Copy it from dashboard and paste it here




//stores and modifies location of shade to prevent damage to shade
struct LocationStruct{
  int shade1Loc;
  int shade2Loc;
} locations;

volatile int shade1Rot = 0;
volatile int shade2Rot = 0;

// allows for count - needed so it doesn't "bounce"
void ICACHE_RAM_ATTR shade1Count() {
  detachInterrupt(digitalPinToInterrupt(shade1Encoder));
  shade1Rot ++;
  attachInterrupt(digitalPinToInterrupt(shade1Encoder), shade1Count, RISING);
 }

void ICACHE_RAM_ATTR shade2Count() {
  detachInterrupt(digitalPinToInterrupt(shade2Encoder));
  shade2Rot ++;
  attachInterrupt(digitalPinToInterrupt(shade2Encoder), shade2Count, RISING);
 }


void shadeControl(int rotationsNeeded1, int rotationsNeeded2) {
  Serial.println("");
  Serial.print("previous location: ");
  Serial.print(locations.shade1Loc);
  Serial.print("     ");
  Serial.println(locations.shade2Loc);
  shade1Rot = 0;
  shade2Rot = 0;
  int temp1 = 0;
  int temp2 = 0;
  int s1dir, s2dir;


  if (rotationsNeeded1 > 0){
    digitalWrite(shade1c, LOW);
    digitalWrite(shade1cc, HIGH);
    Serial.println(rotationsNeeded1);
    s1dir = 1;
  }
  else if (rotationsNeeded1 < 0){
    digitalWrite(shade1c, HIGH);
    digitalWrite(shade1cc, LOW);
    rotationsNeeded1 *= -1;
    Serial.println(rotationsNeeded1);
    s1dir = -1;
  }

  if (rotationsNeeded2 > 0){
    digitalWrite(shade2c, LOW);
    digitalWrite(shade2cc, HIGH);
    Serial.println(rotationsNeeded2);
    s2dir = 1;
  }
  if (rotationsNeeded2 < 0){
    digitalWrite(shade2c, HIGH);
    digitalWrite(shade2cc, LOW);
    rotationsNeeded2 *= -1;
    Serial.println(rotationsNeeded2);
    s2dir = -1;
  }
  
  while (shade1Rot < rotationsNeeded1 or shade2Rot < rotationsNeeded2){
    int sensorValueA = analogRead(anal); 
    float current = sensorValueA * (5.0/1023.0);  //measures voltage but pretty much reflects the current
    delay(50);
    if (shade1Rot != temp1){
      Serial.println(shade1Rot);
      temp1 = shade1Rot;
      locations.shade1Loc = locations.shade1Loc + 1 * s1dir; 
      //EEPROM.put(0, locations);
      //EEPROM.commit();
    }
    if (shade2Rot != temp2){
      Serial.print("           ");
      Serial.println(shade2Rot);
      temp2 = shade2Rot;
      locations.shade2Loc = locations.shade2Loc + 1 * s2dir; 
      //EEPROM.put(0, locations);
      //EEPROM.commit();
    }
    
    if (shade1Rot >= rotationsNeeded1){
      digitalWrite(shade1cc, LOW);
      digitalWrite(shade1c, LOW);
    }
    if (shade2Rot >= rotationsNeeded2){
       digitalWrite(shade2cc, LOW);
       digitalWrite(shade2c, LOW);
    }

    /*if (shade2Rot >= rotationsNeeded and shade1Rot >= rotationsNeeded){
      break;
    }*/
    
    if (current >= .80){
      Serial.println("Too much current");
      Serial.println(current);
      break;
    }   
  } 
  
  digitalWrite(shade1cc, LOW);
  digitalWrite(shade1c, LOW);
  digitalWrite(shade2cc, LOW);
  digitalWrite(shade2c, LOW);
  EEPROM.put(0, locations);
  EEPROM.commit();
  Serial.print("New location: ");
  Serial.print(locations.shade1Loc);
  Serial.print("     ");
  Serial.println(locations.shade2Loc);
}



void changeBlinds(int newLocation) {

  if ((newLocation != locations.shade1Loc or newLocation != locations.shade2Loc) and newLocation >= completelyUp and newLocation <= completelyDown){
    int rotationsNeeded1 = newLocation - locations.shade1Loc;
    int rotationsNeeded2 = newLocation - locations.shade2Loc;
    shadeControl(rotationsNeeded1, rotationsNeeded2);
  }
}


  





void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {  
  switch (type) {
    case WStype_DISCONNECTED:
      isConnected = false;
      Serial.printf("[WSc] Webservice disconnected from sinric.com!\n");
      break;

    case WStype_CONNECTED:
      isConnected = true;
      Serial.printf("[WSc] Service connected to sinric.com at url: %s\n", payload);
      Serial.printf("Waiting for commands from sinric.com ...\n");
      digitalWrite(ledPin, HIGH);
      delay(1500);
      digitalWrite(ledPin, LOW);
      delay(1000);
      digitalWrite(ledPin, HIGH);
      delay(1500);
      digitalWrite(ledPin, LOW);

      break;

    case WStype_BIN:
      Serial.printf("[WSc] get binary length: %u\n", length);
      break;

    case WStype_TEXT: {
        
        Serial.println("");
        Serial.println("");
        Serial.printf("[WSc] get text: %s\n", payload);
        // Example payloads

        // For Switch or Light device types
        // {"deviceId": xxxx, "action": "setPowerState", value: "ON"} // https://developer.amazon.com/docs/device-apis/alexa-powercontroller.html

        // For Light device type
        // Look at the light example in github

        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject((char*)payload);
        String deviceId = json ["deviceId"];
        String action = json ["action"];

        if (deviceId == shadesSinricID){
        
          if (action == "AdjustBrightness") { // Switch or Light
            float value = json ["value"];
            int newLocation = locations.shade1Loc - (value * .01) * locations.shade1Loc;
            if (newLocation >= completelyUp and newLocation <= completelyDown){
              changeBlinds(newLocation);
            }  
            else if (newLocation < completelyUp){
              changeBlinds(completelyUp);
            }
            else if (newLocation > completelyDown){
              changeBlinds(completelyDown);
            }
          }
            
          
    
          if (action == "SetBrightness") { // Switch or Light
            int value = json ["value"];
            int newLocation = 15 - map(abs(value), 0, 100, completelyUp, completelyDown);
            if (newLocation >= completelyUp and newLocation <= completelyDown){
              changeBlinds(newLocation);   
          }
          }
          
          else if (action = "setPowerState") {
            String value = json ["value"];
            if (value == "ON") {
              changeBlinds(completelyUp);
            }
            else if (value == "OFF") {
              changeBlinds(completelyDown);
    
            }
          }
          
          else if (action == "test") {
            Serial.println("[WSc] received test command from sinric.com");
          }
        }
      break;
    }
  }
}






void setup()
{
  //current readings to determine location?
  Serial.begin(115200);
  Serial.println("");
  Serial.println("RESTARTED");
  pinMode (shade1cc, OUTPUT);
  pinMode (shade1c, OUTPUT);
  pinMode (shade2cc, OUTPUT);
  pinMode (shade2c, OUTPUT);
  pinMode (shade1Encoder, INPUT);
  pinMode (shade2Encoder, INPUT);
  
  pinMode(b_1, INPUT);
  pinMode(b_2, INPUT);
  pinMode(b_3, INPUT);

  pinMode(ledPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(shade1Encoder), shade1Count, RISING);
  attachInterrupt(digitalPinToInterrupt(shade2Encoder), shade2Count, RISING);

  
  EEPROM.begin(sizeof(LocationStruct));
  
  if(EEPROM.percentUsed()>=0){
     EEPROM.get(0, locations.shade1Loc);
     EEPROM.get(4, locations.shade2Loc);
     Serial.println("EEPROM has data from a previous run.");
     Serial.print(EEPROM.percentUsed());
     Serial.println("% of ESP flash space currently used");
  }
  
  else {
    Serial.println("EEPROM size changed - EEPROM data zeroed - commit() to make permanent"); 
    locations.shade1Loc = 0;
    locations.shade2Loc = 0;
    EEPROM.put(0, locations);
    boolean saved = EEPROM.commit();
    Serial.print("Saved: ");
    Serial.println(saved);
  }
  
  button1.init(b_1);
  button2.init(b_2);
  button3.init(b_3);

  ButtonConfig* config = ButtonConfig::getSystemButtonConfig();
  config->setEventHandler(buttonHandler);

  
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(MySSID, MyWifiPassword);
  //WiFiMulti.addAP(MySSID, MyWifiPassword);
  Serial.print("Connecting to Wifi: ");
  Serial.println(MySSID);


  // Waiting for Wifi connect
  while (WiFi.status() != WL_CONNECTED) {
    if (digitalRead(b_1) == HIGH) {
      Serial.println("");
      Serial.println("");
      Serial.println("upper button pushed");
      changeBlinds(completelyUp);
    }
    else if (digitalRead(b_2) == HIGH) {
      Serial.println("");
      Serial.println("");
      Serial.println("middle button pushed");
      changeBlinds(midway);
    }

    else if (digitalRead(b_3) == HIGH){
      Serial.println("");
      Serial.println("");
      Serial.println("lower button pushed");
      changeBlinds(completelyDown);
    }
    delay(100);
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("WiFi connected. ");
  }

  
  // server address, port and URL
  webSocket.begin("iot.sinric.com", 80, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);
  webSocket.setAuthorization("apikey", MyApiKey);

  // try again every 5000ms if connection has failed
  webSocket.setReconnectInterval(5000); // If you see 'class WebSocketsClient' has no member named 'setReconnectInterval' error update arduinoWebSockets
}}





void loop() {
  delay(20);
  webSocket.loop();
  button1.check();
  button2.check();
  button3.check();
  if (isConnected) {
    uint64_t now = millis();

    // Send heartbeat in order to avoid disconnections during ISP resetting IPs over night. Thanks @MacSass
    if ((now - heartbeatTimestamp) > HEARTBEAT_INTERVAL) {
      heartbeatTimestamp = now;
      webSocket.sendTXT("H");
    }

  }

}





void buttonHandler(AceButton* button, uint8_t eventType, uint8_t /* buttonState */) { 
  if (eventType == AceButton::kEventReleased) {
  Serial.println("");
  Serial.println("");
  switch (button->getPin()) {
    case 14:
      Serial.println("upper button pushed");
      changeBlinds(completelyUp);
      //setPowerStateOnServer(shadesSinricID, "ON");
      break;
    case 12:
      Serial.println("middle button pushed");
      changeBlinds(midway);
      //setBrightnessOnServer(shadesSinricID, 50);
      break;
     case 13:
      Serial.println("lower button pushed");
      changeBlinds(completelyDown);
      //setPowerStateOnServer(shadesSinricID, "OFF");
      break;
  }
  }
}
