#include <ESP8266WiFi.h>
#include <NTPClient.h>      // https://github.com/arduino-libraries/NTPClient
#include "RestClient.h"     // https://github.com/fabianofranca/ESP8266RestClient
#include <WiFiUdp.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

/* *************************************
 *  
 *  NTP Timekeeping Variables
 *  
 *  ************************************
 */

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP); // By default 'time.nist.gov' is used with 60 seconds update interval and no timezone offset

/* *************************************
 *  
 *  Server, Sensor ID, and Wifi Variables
 *  
 *  ************************************
 */

RestClient rclient = RestClient("demo.ballotbots.io");
String sensorName = "esp8266+Motion+Logger"; // in URI escapted format
String sensorId = "x8266b"; // in URI escapted format
String stationId = "Evan+Office"; // in URI escapted format

/* *************************************
 *  
 *  Sensor Specific Variables (pins, calibration, etc.)
 *  
 *  ************************************
 */

int calibrationTime = 30;  //the time for sensor to calibrate
long unsigned int lowIn;   //the time when the sensor outputs a low impulse         
long unsigned int pause = 1;  //the amount of milliseconds sensor has to be low before we assume all motion has stopped
boolean lockLow = true;
boolean takeLowTime;  
int i = 1;
int pirPin = 16;    //the digital pin connected to the PIR sensor's output
int redLedPin = 0;
int blueLedPin = 2;

/* *************************************
 *  
 *  setup() function
 *  
 *  ************************************
 */

void setup() {
  Serial.begin(115200);
  pinMode(pirPin, INPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  digitalWrite(redLedPin, HIGH); // start with Red LED off
  digitalWrite(blueLedPin, HIGH); // start with Blue LED off
  delay(100);

//initialize WIFI and REST Client

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset saved settings
  wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("BallotBot Config", "password");


  
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

//  rclient.begin(ssid,password);

  Serial.print("Wifi Status: ");
  Serial.println(WiFi.status());

// calibrate sensor

  calibrateSensor(calibrationTime);

// set time

  setNTPtime(timeClient);

  postBatteryLevelToServer(rclient, timeClient, sensorName, sensorId, stationId);
}


/* *************************************
 *  
 *  loop() function
 *  
 *  ************************************
 */
void loop() {

  if(millis()%60000 == 0){
    // Serial.println("Time Update: ");
    if(timeClient.update()){
//      Serial.print("Success");
//      Serial.println(timeClient.getEpochTime());
    }
    else{
//      Serial.print("Failed");
    }
  }
   
  if(digitalRead(pirPin) == HIGH) {
      digitalWrite(blueLedPin,LOW);
      if(lockLow) {      //makes sure we wait for transition to LOW before any further OUTPUT
        lockLow = false;
        Serial.print(i++);
        Serial.print(", ");
        Serial.println("motion detected");
        postToServer(rclient, timeClient, sensorName, sensorId, stationId);
        int level = getBatteryLevel();
      }
    }
    
  if(digitalRead(pirPin) == LOW) {
    digitalWrite(blueLedPin,HIGH);    
      if(takeLowTime) {
        lowIn = millis();     //save time of transition from HIGH to LOW
        takeLowTime = false;  //make sure this is done a start of LOW phase
      } //if sensor LOW for more than pause time, assume no more motion is happening
      
      if(!lockLow && millis() - lowIn > pause) { //ensures codeblock is executed only after new motion
        lockLow = true;
      }
    }

  if(millis()%300000 == 0){
    postBatteryLevelToServer(rclient, timeClient, sensorName, sensorId, stationId);
  }
}

/* *************************************
 *  
 *  helper functions
 *  
 *  ************************************
 */

void setNTPtime(NTPClient &ntpClient){
  int timeSetTimeout = 30; // seconds to keep trying to update time from ntp server before giving up
  
  ntpClient.begin(); // NTPClient library uses time.nist.gov server

  while(timeSetTimeout > 0 && !ntpClient.update()){
    Serial.println("Time Update: Failed");
    timeSetTimeout--;
  }

  Serial.println("Time Update: Succeeded");
  Serial.println(ntpClient.getEpochTime());
}

void calibrateSensor(int calibrationTime){
  Serial.println("calibrating sensor ");     
  
  for(int i = 0; i < calibrationTime; i++){
    Serial.print(".");
    digitalWrite(blueLedPin,LOW); // blink LED
    delay(500);
    digitalWrite(blueLedPin,HIGH);
    delay(500);
  }
  
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(50);
}

void postToServer(RestClient &rclient, NTPClient &ntpClient, String sensorName, String sensorId, String stationId){
  String response = "";
        
  String body = "sensorName=" + sensorName + "&sensorId=" + sensorId + "&stationId=" + stationId + "&dateTime=" + String(ntpClient.getEpochTime());

  char charBody[body.length()+1];
  body.toCharArray(charBody,body.length()+1);

  int statusCode = rclient.post("/api/SensorData",charBody,&response); 
  if(statusCode == 201){
    Serial.println("POST Success!");
    Serial.print("Status Code: ");
    Serial.println(statusCode);
    Serial.println("Response:");
    Serial.println(response);
  }
  else {
    Serial.println("POST Failed");
    Serial.print("Status Code: ");
    Serial.println(statusCode);
  }
}

void postBatteryLevelToServer(RestClient &rclient, NTPClient &ntpClient, String sensorName, String sensorId, String stationId){
  Serial.println("Sending Battery Level to Server"); 
  String response = "";
        
  String body = "sensorName=" + sensorName + "&sensorId=" + sensorId + "&stationId=" + stationId + "&dateTime=" + String(ntpClient.getEpochTime()) + "&batteryLevel=" + getBatteryLevel();

  char charBody[body.length()+1];
  body.toCharArray(charBody,body.length()+1);

  int statusCode = rclient.post("/api/SensorStats",charBody,&response); 
  if(statusCode == 201){
    Serial.println("POST Success!");
    Serial.print("Status Code: ");
    Serial.println(statusCode);
    Serial.println("Response:");
    Serial.println(response);
  }
  else {
    Serial.println("POST Failed");
    Serial.print("Status Code: ");
    Serial.println(statusCode);
  }
}

int getBatteryLevel(){
  Serial.println("Getting Battery Level.......................");
  float batteryLevel = analogRead(A0);

  // convert battery level to percent
  int batteryPercent = map(batteryLevel, 574, 753, 0, 100);
  Serial.print("Battery level: "); Serial.print(batteryLevel); Serial.print(" ("); Serial.print(batteryPercent); Serial.println("%)");
  return batteryPercent;
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

