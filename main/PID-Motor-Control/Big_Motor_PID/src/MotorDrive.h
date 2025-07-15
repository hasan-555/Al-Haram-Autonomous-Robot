#ifndef MOTORDRIVE_H
#define MOTORDRIVE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Encoder.h>

class MotorDrive {
public:
    MotorDrive(
    int pwmPin1, int dirPin1, int brakePin1, int encA1, int encB1,
    int pwmPin2, int dirPin2, int brakePin2, int encA2, int encB2,
    float wheelRadius, float wheelSeparation, int maxPWM = 255
    );

    void begin();
    void drive(float linearVelX, float angularVelZ);
    void setMotor(int motorID, int direction, int pwmVal, int brake);
    void stopMotor(int motorID);
    void reset();

    float getLeftDistance();
    float getRightDistance();

    void parseCommandJSON(const String& jsonString);
    void updatedistance();

    // Add these public method declarations
    void calculateMotorSpeeds();  // Updates _leftSpeed and _rightSpeed
    void getMotorSpeeds(float& leftSpeed, float& rightSpeed);  // Gets current motor speeds
    void calculateRobotVelocity(float& linearVel, float& angularVel);  // Calculates robot velocity

    // Add to public section
    void setTargetDistance(float leftDistance, float rightDistance);
    void checkAndStopAtDistance();
    bool hasReachedTargetDistance();

private:

    int _m1_pwm, _m1_dir, _m1_brake, _m1_encA, _m1_encB;
    int _m2_pwm, _m2_dir, _m2_brake, _m2_encA, _m2_encB;
    float _wheelRadius, _wheelSeparation;
    int _maxPWM;

    ESP32Encoder _enc1, _enc2;

    float left_distance = 0;
    float right_distance = 0;

    // Add these to your private section
    unsigned long _lastUpdateTime = 0;
    float _prevLeftDistance = 0;
    float _prevRightDistance = 0;
    float _leftSpeed = 0;
    float _rightSpeed = 0;

        // Add to private section
    float _targetLeftDistance = 0;
    float _targetRightDistance = 0;
    bool _distanceControlEnabled = false;

};

#endif
