#include <ESP32Encoder.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "ESPTelnet.h"
#include "MotorDrive.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

// Access Point credentials
const char *ap_ssid = "SWB";
const char *ap_password = "Swb@2025@@";
ESPTelnet telnet;
AsyncWebServer server(80);

// BNO055 IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Motor 1 Pins (Left Motor)
#define M1_PWM_PIN 33
#define M1_DIR_PIN 32
#define M1_BRE_PIN 27
#define M1_ENCODER_A 16
#define M1_ENCODER_B 17

// Motor 2 Pins (Right Motor)
#define M2_PWM_PIN 25
#define M2_DIR_PIN 26
#define M2_BRE_PIN 23
#define M2_ENCODER_A 18
#define M2_ENCODER_B 19

// Define static IP settings
IPAddress local_IP(192, 168, 10, 235); // Set your desired IP
IPAddress gateway(192, 168, 1, 1);     // Router's IP (default gateway)
IPAddress subnet(255, 255, 255, 0);    // Subnet mask
IPAddress primaryDNS(8, 8, 8, 8);      // Optional: Primary DNS
IPAddress secondaryDNS(8, 8, 4, 4);    // Optional: Secondary DNS

// Constants for your robot configuration
const float WHEEL_RADIUS = 20.0; // cm (adjust based on your wheels)
const float WHEEL_SEPARATION = 56.2; // cm (distance between wheels, adjust as needed)
const int MAX_PWM = 255; // Maximum PWM value for your motors

MotorDrive     Hammer_Head(M1_PWM_PIN, M1_DIR_PIN, M1_BRE_PIN, M1_ENCODER_A, M1_ENCODER_B,
    M2_PWM_PIN, M2_DIR_PIN, M2_BRE_PIN, M2_ENCODER_A, M2_ENCODER_B,
    WHEEL_RADIUS, WHEEL_SEPARATION, MAX_PWM
    );

// IMU calibration variables
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
int samples = 500;

unsigned long previousMillis = 0;
const unsigned long interval = 50;  // Interval in milliseconds (adjustable)

// JSON buffer
String incomingData = "";

// Function prototypes
void setupWiFi();
void onTelnetInput(String str);

void calibrateBNO055();

// WiFi setup
void setupWiFi()
{
  WiFi.mode(WIFI_STA);  
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);

  WiFi.begin(ap_ssid, ap_password);
  while (!WiFi.status() == WL_CONNECTED)
  {
  }
}

// void onTelnetInput(String str){
//     Hammer_Head.parseCommandJSON(str.c_str());
// }

// BNO055 calibration
void calibrateBNO055()
{
  Serial.println("Calibrating...");
  sensors_event_t event;

  for (int i = 0; i < samples; i++)
  {
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    accelX_offset += event.acceleration.x;
    accelY_offset += event.acceleration.y;
    accelZ_offset += event.acceleration.z - 9.81;

    bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyroX_offset += event.gyro.x;
    gyroY_offset += event.gyro.y;
    gyroZ_offset += event.gyro.z;

    delay(5);
  }

  accelX_offset /= samples;
  accelY_offset /= samples;
  accelZ_offset /= samples;
  gyroX_offset /= samples;
  gyroY_offset /= samples;
  gyroZ_offset /= samples;

  Serial.println("Calibration complete.");
}


void setup(){

	Serial.begin(115200);
  Hammer_Head.begin();

    // telnet and wifi
	setupWiFi();
	//ElegantOTA.begin(&server, "admin", "admin"); // Start ElegantOTA
	server.begin();
	telnet.begin(23, false);
  //telnet.onInputReceived(onTelnetInput);

    //
  Wire.begin(21, 22); // ESP32 default I2C pins
  
  // Initialize BNO055
  if (!bno.begin())
  {
    Serial.println("Could!");
    while (1)
      ;
  }

  bno.setMode(OPERATION_MODE_NDOF);

  calibrateBNO055();
  
}

void loop(){
  //ElegantOTA.loop();
  telnet.loop();

  JsonDocument doc;
  Hammer_Head.updatedistance();

  //   // IMU data
  sensors_event_t event;

  unsigned long currentMillis = millis();  // Get current time
  unsigned long timeNow =  currentMillis / 100; // Get current time

  if (currentMillis - previousMillis >= interval) {
    
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float correctedAx = event.acceleration.x - accelX_offset;
    float correctedAy = event.acceleration.y - accelY_offset;
    float correctedAz = event.acceleration.z - accelZ_offset;
    
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
    float correctedGx = event.gyro.x - gyroX_offset;
    float correctedGy = event.gyro.y - gyroY_offset;
    float correctedGz = event.gyro.z - gyroZ_offset;
    
    // Send JSON data
    JsonObject imu = doc.createNestedObject("imu");
    imu["a"]["x"] = correctedAx;
    imu["a"]["y"] = correctedAy;
    imu["a"]["z"] = correctedAz;
    imu["g"]["x"] = correctedGx;
    imu["g"]["y"] = correctedGy;
    imu["g"]["z"] = correctedGz;

    JsonObject robot = doc.createNestedObject("rob");
    robot["dl"] = Hammer_Head.getLeftDistance();
    robot["dr"] = Hammer_Head.getRightDistance();
      
        // // Calculate and display velocities
        // float leftSpeed, rightSpeed;
        // Hammer_Head.getMotorSpeeds(leftSpeed, rightSpeed);
        // float linearVel, angularVel;
        // Hammer_Head.calculateRobotVelocity(linearVel, angularVel);
 
        // Serial.print("Time: " ); Serial.print(timeNow);
        // Serial.print("  Lev: "); Serial.print(leftSpeed);
        // Serial.print("  riv: "); Serial.print(rightSpeed);

        // Serial.print("  Lv: "); Serial.print(linearVel);
        // Serial.print("  Av: "); Serial.print(angularVel);
        // Serial.print("   ");

    previousMillis = currentMillis;  // Reset the timer
    String jsonString;
    serializeJson(doc, jsonString);
    Serial.println(jsonString);
  }

  while (Serial.available() > 0)
   {
     char c = Serial.read();
     if (c == '\n')
     {
       Hammer_Head.parseCommandJSON(incomingData.c_str());
       incomingData = "";
     }
     else
     {
      incomingData += c;
    }
  }

}