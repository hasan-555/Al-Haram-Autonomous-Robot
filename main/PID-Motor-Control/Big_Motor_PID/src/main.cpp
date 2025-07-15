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

#define i_mot_l_pin 12
#define FILTER_SIZE 30

uint16_t rawValue;
float voltage = 0.0;
float current = 0.0;
float current_offset = 0;
float currentFiltered = 0;
float currentFilteredNew = 0;
float currentBuffer[FILTER_SIZE] = {0};
int bufindex = 0;

// Voltage Divider Ratio
const float dividerRatio = 3.3 / 5.0; // 5V down to 3.3V

// ACS712 Parameters (for 30A version)
const float sensitivity = 0.066;      // 66mV per Amp
const float zeroCurrentVoltage = 2.5; // 2.5V is the midpoint for 0A
int pwmOutput = 0;

// JSON buffer
String incomingData = "";

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


// PI Controller Parameters
const float Kp = 20.0; // Proportional gain (tune this)
const float Ki = 15.0;  // Integral gain (tune this)
float targetCurrent = 0.4; // Desired current setpoint (amps)
float integralError = 0.0; // Integral term for PI controller
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 10; // Control loop interval (ms)

// PID Speed Controller Parameters
const float speedKp = 0.32;  // Proportional gain for speed (tune this)
const float speedKi = 0.4;  // Integral gain for speed (tune this)
const float speedKd = 0.09; // Derivative gain for speed (tune this)
float targetSpeed = 0.1;    // Desired speed setpoint (cm/s)
float integralErrorSpeed = 0.0; // Integral term for PID speed controller
float lastErrorSpeed = 0.0;    // Previous error for PID speed controller
unsigned long lastSpeedControlTime = 0;
const unsigned long speedControlInterval = 20; // Speed control loop interval (ms)
const float maxCurrent = 15.0; // Maximum allowable current setpoint (amps, adjust as needed)
const float K = 0.05;

// Constants for your robot configuration
const float WHEEL_RADIUS = 0.2032; // m (adjust based on your wheels)
const float WHEEL_SEPARATION = 0.562; // m (distance between wheels, adjust as needed)
const int MAX_PWM = 255; // Maximum PWM value for your motors

MotorDrive     Hammer_Head(M1_PWM_PIN, M1_DIR_PIN, M1_BRE_PIN, M1_ENCODER_A, M1_ENCODER_B,
    M2_PWM_PIN, M2_DIR_PIN, M2_BRE_PIN, M2_ENCODER_A, M2_ENCODER_B,
    WHEEL_RADIUS, WHEEL_SEPARATION, MAX_PWM
    );

unsigned long previousMillis = 0;
const unsigned long interval = 100;  // Interval in milliseconds (adjustable)


void setup(){

	Serial.begin(115200);
  Hammer_Head.begin();

  delay(500);
  analogReadResolution(12);
  
  // Calculate mean
  float sum = 0;
  
  for (int i=0 ; i<5000 ; i++){

  rawValue = analogRead(i_mot_l_pin);
  voltage = (rawValue / 4095.0) * 3.3; // Convert ADC to voltage (0-3.3V)
  float originalVoltage = voltage / dividerRatio;
  
  // Calculate current (A)
  current = ((originalVoltage - zeroCurrentVoltage) / sensitivity);
  sum = sum + current;

  }
  current_offset = sum / 5000;

  delay(5000);
}

void loop(){

  Hammer_Head.updatedistance();
  
  unsigned long currentMillis = millis();  // Get current time
  unsigned long timeNow =  currentMillis / 100; // Get current time
  
  rawValue = analogRead(i_mot_l_pin);
  voltage = (rawValue / 4095.0) * 3.3; // Convert ADC to voltage (0-3.3V)
  float originalVoltage = voltage / dividerRatio;
  // Calculate current (A)
  current = ((originalVoltage - zeroCurrentVoltage) / sensitivity) - current_offset;
  
  static float runningSum = 0.0;
  runningSum -= currentBuffer[bufindex]; // Subtract oldest value
  currentBuffer[bufindex] = current;     // Add new value
  runningSum += current;                 // Add to running sum
  bufindex = (bufindex + 1) % FILTER_SIZE;
  currentFiltered = runningSum / FILTER_SIZE;

 // Hammer_Head.parseCommandJSON("{\"m\":{\"v\":15,\"w\":0}}");
  if (currentMillis - previousMillis >= interval) {

      // // Calculate and display velocities
      float leftSpeed, rightSpeed;
      Hammer_Head.calculateMotorSpeeds();
      Hammer_Head.getMotorSpeeds(leftSpeed, rightSpeed);
      
      if (leftSpeed < 0){
        currentFilteredNew = - currentFiltered;
      }
      else{
        currentFilteredNew = currentFiltered;
      }
      // float linearVel, angularVel;
      // Hammer_Head.calculateRobotVelocity(linearVel, angularVel);

        // Serial.print("T: " ); Serial.print(timeNow);
      Serial.print(">");
      Serial.print(" C: ");
      Serial.print(currentFilteredNew);
      Serial.print(",");
      Serial.print("TC: ");
      Serial.print(targetCurrent);
      Serial.print(",");
      Serial.print("  Lev: "); 
      Serial.print(leftSpeed);
      Serial.println();
                    // Serial.print("  riv: "); Serial.println(rightSpeed);

        // Serial.print("  Lv: "); Serial.print(linearVel);
        // Serial.print("  Av: "); Serial.print(angularVel);
        // Serial.print("   ");

      previousMillis = currentMillis;  // Reset the timer
  }

// PI Controller for Motor 1 Current
  if (currentMillis - lastControlTime >= controlInterval) {

    // Calculate error
    float error = targetCurrent - currentFilteredNew;

    // Update integral term      currentFiltered = - sum / FILTER_SIZE; (with anti-windup)
    integralError += error * (controlInterval / 1000.0); // Convert ms to seconds

    // Optional: Limit integral to prevent windup
    const float integralLimit = 100.0;
    if (integralError > integralLimit) integralError = integralLimit;
    if (integralError < -integralLimit) integralError = -integralLimit;

    // Calculate PI output (PWM)
    float piOutput = Kp * error + Ki * integralError;

    // Limit PWM output
    pwmOutput = (int)piOutput;
    if (pwmOutput > MAX_PWM) pwmOutput = MAX_PWM;
    if (pwmOutput < -MAX_PWM) pwmOutput = -MAX_PWM;

    // Set motor direction and PWM
    if (pwmOutput >= 0) {
      Hammer_Head.setMotor(1, 0, abs(pwmOutput),0);// Assuming Motor 1 is left motor
      // Set direction forward (adjust based on your MotorDrive class
    } else {
      Hammer_Head.setMotor(1, 1, abs(pwmOutput),0);// Assuming Motor 1 is left motor

    }

    lastControlTime = currentMillis;
  }

  //   while (Serial.available() > 0)
  //  {
  //    char c = Serial.read();
  //    if (c == '\n')
  //    {
  //      Hammer_Head.parseCommandJSON(incomingData.c_str());
  //      incomingData = "";
  //    }
  //    else
  //    {
  //     incomingData += c;
  //   }
  // }

}