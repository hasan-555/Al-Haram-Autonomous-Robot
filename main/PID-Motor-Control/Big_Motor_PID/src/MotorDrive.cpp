#include "MotorDrive.h"

MotorDrive::MotorDrive(
    int pwmPin1, int dirPin1, int brakePin1, int encA1, int encB1,
    int pwmPin2, int dirPin2, int brakePin2, int encA2, int encB2,
    float wheelRadius, float wheelSeparation, int maxPWM
) :
    _m1_pwm(pwmPin1), _m1_dir(dirPin1), _m1_brake(brakePin1), _m1_encA(encA1), _m1_encB(encB1),
    _m2_pwm(pwmPin2), _m2_dir(dirPin2), _m2_brake(brakePin2), _m2_encA(encA2), _m2_encB(encB2),
    _wheelRadius(wheelRadius), _wheelSeparation(wheelSeparation), _maxPWM(maxPWM) {}

    void MotorDrive::begin() {
        pinMode(_m1_pwm, OUTPUT);
        pinMode(_m1_dir, OUTPUT);
        pinMode(_m1_brake, OUTPUT);
        pinMode(_m2_pwm, OUTPUT);
        pinMode(_m2_dir, OUTPUT);
        pinMode(_m2_brake, OUTPUT);
    
        drive(0, 0);

        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        _enc1.attachHalfQuad(_m1_encA, _m1_encB);
        _enc2.attachHalfQuad(_m2_encA, _m2_encB);
        _enc1.setCount(0);
        _enc2.setCount(0);
    }

void MotorDrive::drive(float linearVelX, float angularVelZ) {
 // Calculate wheel velocities in cm/s
 float left_wheel_vel = linearVelX - (angularVelZ * _wheelSeparation / 2.0);
 float right_wheel_vel = linearVelX + (angularVelZ * _wheelSeparation / 2.0);
 
 // Convert to PWM values (assuming linear relationship between PWM and speed)
 int left_pwm = constrain(abs(left_wheel_vel) * _maxPWM / (2 * _wheelRadius * PI), 0, _maxPWM);
 int right_pwm = constrain(abs(right_wheel_vel) * _maxPWM / (2 * _wheelRadius * PI), 0, _maxPWM);
 
 // Determine directions (1 for forward, 0 for backward)
 int left_dir = (left_wheel_vel >= 0) ? 1 : 0;
 int right_dir = (right_wheel_vel < 0) ? 1 : 0;

 setMotor(1, left_dir, left_pwm, 0);  // Left motor
 setMotor(2, right_dir, right_pwm, 0); // Right motor
}

void MotorDrive::setMotor(int motorID, int direction, int pwmVal, int brake) {
    int pwm_pin = (motorID == 1) ? _m1_pwm : _m2_pwm;
    int dir_pin = (motorID == 1) ? _m1_dir : _m2_dir;
    int bre_pin = (motorID == 1) ? _m1_brake : _m2_brake;
  
    if (pwmVal == 0){
      brake = 1;
    }
  
    if (brake == 1)
    {
      digitalWrite(bre_pin, HIGH); // this is inverse
      analogWrite(pwm_pin, 0); // Stop PWM when braking
    }
    else
    {
      digitalWrite(bre_pin, LOW);
      analogWrite(pwm_pin, pwmVal);
      digitalWrite(dir_pin, (direction == 1) ? LOW : HIGH);
    }
}

void MotorDrive::stopMotor(int motorID) {
    setMotor(motorID, 0, 0, 1);
}

void MotorDrive::updatedistance() {
  
  float wheel_turns_m1 =  ((int32_t)_enc1.getCount()) / 8192.0;
  float wheel_turns_m2 =  ((int32_t)_enc2.getCount()) / 8192.0;

  left_distance  = wheel_turns_m1 * 12.965 * PI;
  right_distance = -wheel_turns_m2 * 12.923 * PI;
}

void MotorDrive::reset() {
    _enc1.setCount(0);
    _enc2.setCount(0);
    left_distance = 0;
    right_distance = 0;

    stopMotor(1);
    stopMotor(2);
}

float MotorDrive::getLeftDistance() {
    return left_distance;
}

float MotorDrive::getRightDistance() {
    return right_distance;
}

void MotorDrive::parseCommandJSON(const String& jsonString) {
JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error)
  {
    return;
  }

  float m_linear_x = doc["m"]["v"].as<float>();
  float m_angular_z = doc["m"]["w"].as<float>();
  
//   float new_linear = m_linear_x / 10;
//   float new_angular = m_angular_z / 1000;

  float new_linear = (((m_linear_x / 10)) / 0.417) / 10;       
  float new_angular = m_angular_z / 1000;

  drive(new_linear, new_angular);

}


void MotorDrive::calculateMotorSpeeds() {
    unsigned long currentTime = millis();
    if (_lastUpdateTime == 0) {
        _lastUpdateTime = currentTime;
        return;
    }
    
    // Calculate time difference in seconds
    float dt = (currentTime - _lastUpdateTime) / 1000.0;
    if (dt <= 0) return;  // Avoid division by zero
    
    // Update distances first
    updatedistance();
    
    // Calculate speeds (distance change over time)
    _leftSpeed = (left_distance - _prevLeftDistance) / dt;
    _rightSpeed = (right_distance - _prevRightDistance) / dt;
    
    // Store current values for next calculation
    _prevLeftDistance = left_distance;
    _prevRightDistance = right_distance;
    _lastUpdateTime = currentTime;
}

void MotorDrive::getMotorSpeeds(float& leftSpeed, float& rightSpeed) {
    leftSpeed = _leftSpeed;
    rightSpeed = _rightSpeed;
}

void MotorDrive::calculateRobotVelocity(float& linearVel, float& angularVel) {
    // Calculate motor speeds first
    calculateMotorSpeeds();
    
    // Convert wheel speeds to robot velocities
    // linear velocity = average of both wheel speeds
    linearVel = (_leftSpeed + _rightSpeed) / 2.0;
    
    // angular velocity = (rightSpeed - leftSpeed) / wheel separation
    angularVel = (_rightSpeed - _leftSpeed) / _wheelSeparation;
}

void MotorDrive::setTargetDistance(float leftDistance, float rightDistance) {
    _targetLeftDistance = leftDistance;
    _targetRightDistance = rightDistance;
    _distanceControlEnabled = true;
    reset(); // Reset encoders when setting new target
}

void MotorDrive::checkAndStopAtDistance() {
    if (!_distanceControlEnabled) return;
    
    updatedistance(); // Update current distance measurements
    
    bool leftReached = fabs(left_distance) >= fabs(_targetLeftDistance);
    bool rightReached = fabs(right_distance) >= fabs(_targetRightDistance);
    
    if (leftReached) {
        stopMotor(1);
    }
    if (rightReached) {
        stopMotor(2);
    }
    
    // If both motors reached their targets, disable distance control
    if (leftReached && rightReached) {
        _distanceControlEnabled = false;
    }
}

bool MotorDrive::hasReachedTargetDistance() {
    if (!_distanceControlEnabled) return true;
    
    updatedistance();
    bool leftReached = fabs(left_distance) >= fabs(_targetLeftDistance);
    bool rightReached = fabs(right_distance) >= fabs(_targetRightDistance);
    
    return leftReached && rightReached;
}