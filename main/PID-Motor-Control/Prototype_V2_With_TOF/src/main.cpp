#include <ESP32Encoder.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include "ESPTelnet.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Access Point credentials
const char *ap_ssid = "TP-Link_DFD0";
const char *ap_password = "2025@2025";
ESPTelnet telnet;
AsyncWebServer server(80);

// JSON buffer
JsonDocument doc;

//Relays
#define CHARGING_PERCENTAGE_PIN 34
#define CHARGING_IS_CONNECTED_PIN 35

#define ADC_RESOLUTION 4059
#define VREF 3300

int amplifyingPercentage = 11;
int ThresholdVoltage = 25;


//TOFs
#define distanceThresholdLeft 50
#define distanceThresholdRight 50
#define distanceThresholdFront 50

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Define three sets of trigger and echo pins
const int trigPins[3] = {17, 4, 18}; // Trig pins for sensors 1, 2, 3
const int echoPins[3] = {16, 5, 19};    // Echo pins for sensors 1, 2, 3

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// Arrays to store measurements for all three sensors
long durations[3];
int distanceCm[3];
int distanceInch[3];

// BNO055 IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
int samples = 500;

unsigned long previousMillis = 0;
const unsigned long interval = 50;  // Interval in milliseconds (adjustable)

// Function prototypes
void setupWiFi();
void calibrateBNO055();
int updateTOFDistances();
int updateChargingPercentage();
int updateChargingState();


// WiFi setup
void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ap_ssid, ap_password);
  while (!WiFi.status() == WL_CONNECTED)
  {
  }
}

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

int updateTOFDistances(){
  digitalWrite(trigPins[0], LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPins[0], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[0], LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    durations[0] = pulseIn(echoPins[0], HIGH);
    
    // Calculate the distance
    distanceCm[0] = durations[0] * SOUND_SPEED / 2;
    
    // Convert to inches
    distanceInch[0] = distanceCm[0] * CM_TO_INCH;

    digitalWrite(trigPins[1], LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPins[1], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[1], LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    durations[1] = pulseIn(echoPins[1], HIGH);
    
    // Calculate the distance
    distanceCm[1] = durations[1] * SOUND_SPEED / 2;
    
    // Convert to inches
    distanceInch[1] = distanceCm[1] * CM_TO_INCH;
  

    digitalWrite(trigPins[2], LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPins[2], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[2], LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    durations[2] = pulseIn(echoPins[2], HIGH);
    
    // Calculate the distance
    distanceCm[2] = durations[2] * SOUND_SPEED / 2;
    
    // Convert to inches
    distanceInch[2] = distanceCm[2] * CM_TO_INCH;
    
    // // Print the distance in the Serial Monitor
    // Serial.print("S0: ");
    // Serial.print(distanceCm[0]);
    // Serial.print(" cm , ");
    // Serial.print("S1: ");
    // Serial.print(distanceCm[1]);
    // Serial.print(" cm , ");
    // Serial.print("S2: ");
    // Serial.print(distanceCm[2]);
    // Serial.println(" cm");

    if (distanceCm[0] < distanceThresholdLeft || distanceCm[1] < distanceThresholdRight || distanceCm[2] < distanceThresholdFront){
      return 1;
    }
    else {
      return 0;
    }
}


// Returns voltage in 0.1V units (e.g., 33 = 3.3V, 12 = 1.2V)
int updateChargingPercentage() {
  int raw_value = analogRead(CHARGING_PERCENTAGE_PIN);
  // Calculate voltage in 0.1V steps: (raw * 3300 / 4095) / 100 = (raw * 33 / 4095)
  int voltage_tenths = (raw_value * 33) / 4095;
  return (voltage_tenths * amplifyingPercentage); // Adjust if needed
}

// Returns 1 if voltage > threshold (in 0.1V units)
int updateChargingState() {
  int raw_value = analogRead(CHARGING_IS_CONNECTED_PIN);
  int voltage_tenths = (raw_value * 33) / 4095;
  if (voltage_tenths > (ThresholdVoltage)) { // Compare in 0.1V units
    return 1;
  }
  return 0;
}

void setup() {

	Serial.begin(115200);

  pinMode(CHARGING_PERCENTAGE_PIN,INPUT_PULLDOWN);
  pinMode(CHARGING_IS_CONNECTED_PIN,INPUT_PULLDOWN);

  // Initialize BNO055
  Wire.begin(21, 22); // ESP32 default I2C pins
  if (!bno.begin())
  {
    Serial.println("Could!");
    while (1)
      ;
  }
  bno.setMode(OPERATION_MODE_NDOF);
  calibrateBNO055();

  // Initialize all sensor pins
  for (int i = 0; i < 3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(500);
  display.clearDisplay();

  display.setTextSize(1); // Smaller text to fit all sensors
  display.setTextColor(WHITE);
}

void loop() {
  
  JsonDocument doc;

  // IMU data
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
    robot["TEO"] =  updateTOFDistances();
    robot["CP"] = updateChargingPercentage();
    robot["CI"] = updateChargingState();  
  
    previousMillis = currentMillis;  // Reset the timer
    String jsonString;
    serializeJson(doc, jsonString);
    Serial.println(jsonString);
  }
}