#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include "RestClient.h"
#include <WiFiUdp.h>

/* *************************************
 *  
 *  NTP Timekeeping Variables
 *  
 *  ************************************
 */

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP); // By default 'time.nist.gov' is used with 60 seconds update interval and no offset

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
const char* ssid     = "[SSID]";
const char* password = "[PASSWORD]";

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

  rclient.begin(ssid,password);

// calibrate sensor

  calibrateSensor(calibrationTime);

// set time

  setNTPtime(timeClient);

}


/* *************************************
 *  
 *  loop() function
 *  
 *  ************************************
 */
void loop() {

  if(millis()%10000 == 0){
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

