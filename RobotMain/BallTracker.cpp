// BallTracker.cpp - Implementation of ball tracking module

#include "BallTracker.h"

BallTracker::BallTracker(int leftPWM, int leftD1, int leftD2, 
                         int rightPWM, int rightD1, int rightD2) {
  // Store pin numbers (unchanged)
  leftMotorPWM = leftPWM;
  leftMotorDir1 = leftD1;
  leftMotorDir2 = leftD2;
  rightMotorPWM = rightPWM;
  rightMotorDir1 = rightD1;
  rightMotorDir2 = rightD2;
  
  // Default values (unchanged)
  leftMotorCompensation = 1.0;
  rightMotorCompensation = 1.0;
  
  // Initialize ball position (unchanged)
  ballX = -1;  // -1 means no ball detected
  ballY = -1;
  
  // Default target position (center of frame) (unchanged)
  targetX = 320;  // Assuming 640x480 resolution
  targetY = 240;
  
  // PID for X axis (rotation) - Modified for PID_v1
  KpX = 0.5;
  KiX = 0.0;
  KdX = 0.1;
  xSetpoint = 0;  // We want error to be 0 (ball centered)
  xInput = 0;     // Current error
  xOutput = 0;    // Output to adjust motor speeds
  
  // PID for Y axis (forward/backward) - Modified for PID_v1
  KpY = 0.5;
  KiY = 0.0;
  KdY = 0.1;
  ySetpoint = 0;  // We want error to be 0 (ball centered)
  yInput = 0;     // Current error
  yOutput = 0;    // Output to adjust motor speeds
  
  // After creating the PID objects:
  xPID = new PID(&xInput, &xOutput, &xSetpoint, KpX, KiX, KdX, DIRECT);
  xPID->SetMode(AUTOMATIC);
  xPID->SetOutputLimits(-255, 255);
  xPID->SetSampleTime(50); // Set sample time to 50ms

  yPID = new PID(&yInput, &yOutput, &ySetpoint, KpY, KiY, KdY, DIRECT);
  yPID->SetMode(AUTOMATIC);
  yPID->SetOutputLimits(-255, 255);
  yPID->SetSampleTime(50); // Set sample time to 50ms

  
  // AutoTune setup
  xTuner = NULL;
  yTuner = NULL;
  xTuningActive = false;
  yTuningActive = false;
  aTuneStep = 50;
  aTuneNoise = 1;
  aTuneLookBack = 20;
  
  // State (unchanged)
  isActive = false;
  ballDetected = false;
  
  // Speed (unchanged)
  baseSpeed = 200;
}

BallTracker::~BallTracker() {
  if (xPID != NULL) {
    delete xPID;
  }
  
  if (yPID != NULL) {
    delete yPID;
  }
  
  if (xTuner != NULL) {
    delete xTuner;
  }
  
  if (yTuner != NULL) {
    delete yTuner;
  }
}

void BallTracker::begin() {
  // Set pin modes (unchanged)
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);

  xPID->SetSampleTime(50); // Use a default value or the one from main.ino
  yPID->SetSampleTime(50); // Use a default value or the one from main.ino

  stopMotors();
}

void BallTracker::setMotorCompensation(float left, float right) {
  leftMotorCompensation = left;
  rightMotorCompensation = right;
}

void BallTracker::setPIDX(double p, double i, double d) {
  KpX = p;
  KiX = i;
  KdX = d;
  
  xPID->SetTunings(KpX, KiX, KdX);
}

void BallTracker::setPIDY(double p, double i, double d) {
  KpY = p;
  KiY = i;
  KdY = d;
  
  yPID->SetTunings(KpY, KiY, KdY);
}

void BallTracker::startAutoTuneX() {
  if (xTuningActive) return;
  
  if (xTuner != NULL) {
    delete xTuner;
  }
  
  xTuner = new PID_ATune(&xInput, &xOutput);
  xTuner->SetNoiseBand(aTuneNoise);
  xTuner->SetOutputStep(aTuneStep);
  xTuner->SetLookbackSec((int)aTuneLookBack);
  xTuningActive = true;
}

void BallTracker::stopAutoTuneX() {
  if (!xTuningActive) return;
  
  xTuningActive = false;
  
  // Get the tuning parameters
  KpX = xTuner->GetKp();
  KiX = xTuner->GetKi();
  KdX = xTuner->GetKd();
  
  // Apply the new tuning values
  xPID->SetTunings(KpX, KiX, KdX);
  
  // Clean up
  delete xTuner;
  xTuner = NULL;
}

void BallTracker::startAutoTuneY() {
  if (yTuningActive) return;
  
  if (yTuner != NULL) {
    delete yTuner;
  }
  
  yTuner = new PID_ATune(&yInput, &yOutput);
  yTuner->SetNoiseBand(aTuneNoise);
  yTuner->SetOutputStep(aTuneStep);
  yTuner->SetLookbackSec((int)aTuneLookBack);
  yTuningActive = true;
}
void BallTracker::stopAutoTuneY() {
  if (!yTuningActive) return;
  
  yTuningActive = false;
  
  // Get the tuning parameters
  KpY = yTuner->GetKp();
  KiY = yTuner->GetKi();
  KdY = yTuner->GetKd();
  
  // Apply the new tuning values
  yPID->SetTunings(KpY, KiY, KdY);
  
  // Clean up
  delete yTuner;
  yTuner = NULL;
}

void BallTracker::setBaseSpeed(int speed) {
  baseSpeed = speed;
}

void BallTracker::setTargetPosition(int x, int y) {
  targetX = x;
  targetY = y;
}

void BallTracker::updateBallPosition(int x, int y) {
  // If both x and y are -1, no ball is detected
  if (x == -1 && y == -1) {
    ballDetected = false;
  } else {
    ballX = x;
    ballY = y;
    ballDetected = true;
  }
}

void BallTracker::autoTuneLoop() {
  if (!ballDetected) return;
  
  // Calculate input values (errors)
  xInput = targetX - ballX;
  yInput = targetY - ballY;
  
  // Run AutoTune iterations
  byte xVal = 0;
  byte yVal = 0;
  
  if (xTuningActive) {
    xVal = xTuner->Runtime();
  }
  
  if (yTuningActive) {
    yVal = yTuner->Runtime();
  }
  
  // Apply motor controls based on tuning outputs
  int leftSpeed = 0;
  int rightSpeed = 0;
  
  // Handle X-axis control (rotation)
  if (abs(xInput) > 20) {  // Dead zone of 20 pixels
    int turnSpeed = constrain(abs(xOutput), 120, baseSpeed);
    
    if (xOutput > 0) {
      // Need to turn left
      leftSpeed -= turnSpeed;
      rightSpeed += turnSpeed;
    } else {
      // Need to turn right
      leftSpeed += turnSpeed;
      rightSpeed -= turnSpeed;
    }
  }
  
  // Handle Y-axis control (forward/backward)
  if (abs(yInput) > 20) {  // Dead zone of 20 pixels
    int moveSpeed = constrain(abs(yOutput), 120, baseSpeed);
    
    if (yOutput > 0) {
      // Need to move forward
      leftSpeed += moveSpeed;
      rightSpeed += moveSpeed;
    } else {
      // Need to move backward
      leftSpeed -= moveSpeed;
      rightSpeed -= moveSpeed;
    }
  }
  
  // Apply final motor speeds
  if (leftSpeed > 0 && rightSpeed > 0) {
    // Moving forward with possible rotation
    int adjLeftSpeed = leftSpeed * leftMotorCompensation;
    int adjRightSpeed = rightSpeed * rightMotorCompensation;
    
    digitalWrite(leftMotorDir1, HIGH);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed, 120, 255));
    
    digitalWrite(rightMotorDir1, HIGH);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else if (leftSpeed < 0 && rightSpeed < 0) {
    // Moving backward with possible rotation
    int adjLeftSpeed = abs(leftSpeed) * leftMotorCompensation;
    int adjRightSpeed = abs(rightSpeed) * rightMotorCompensation;
    
    digitalWrite(leftMotorDir1, LOW);
    digitalWrite(leftMotorDir2, HIGH);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed, 120, 255));
    
    digitalWrite(rightMotorDir1, LOW);
    digitalWrite(rightMotorDir2, HIGH);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else if (leftSpeed >= 0 && rightSpeed <= 0) {
    // Turning right
    int adjLeftSpeed = abs(leftSpeed) * leftMotorCompensation;
    int adjRightSpeed = abs(rightSpeed) * rightMotorCompensation;
    
    digitalWrite(leftMotorDir1, HIGH);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed, 120, 255));
    
    digitalWrite(rightMotorDir1, LOW);
    digitalWrite(rightMotorDir2, HIGH);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else if (leftSpeed <= 0 && rightSpeed >= 0) {
    // Turning left
    int adjLeftSpeed = abs(leftSpeed) * leftMotorCompensation;
    int adjRightSpeed = abs(rightSpeed) * rightMotorCompensation;
    
    digitalWrite(leftMotorDir1, LOW);
    digitalWrite(leftMotorDir2, HIGH);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed, 120, 255));
    
    digitalWrite(rightMotorDir1, HIGH);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else {
    // Fallback - stop
    stopMotors();
  }
  
  // Check if AutoTune is finished
  if (xVal != 0) {
    stopAutoTuneX();
  }
  
  if (yVal != 0) {
    stopAutoTuneY();
  }
}

void BallTracker::calculatePID() {
  if (!ballDetected) {
    // If no ball detected, reset PID
    xPID->SetMode(MANUAL);
    yPID->SetMode(MANUAL);
    xOutput = 0;
    yOutput = 0;
    xPID->SetMode(AUTOMATIC);
    yPID->SetMode(AUTOMATIC);
    return;
  }
  
  // Calculate error for X axis (horizontal position)
  xInput = targetX - ballX;
  
  // Calculate error for Y axis (vertical position)
  yInput = targetY - ballY;
  
  // Compute PID outputs
  xPID->Compute();
  yPID->Compute();
}

void BallTracker::updateControl() {
  if (!isActive || !ballDetected) {
    stopMotors();
    return;
  }
  
  // If in AutoTune mode, run that instead
  if (xTuningActive || yTuningActive) {
    autoTuneLoop();
    return;
  }
  
  // Calculate PID values
  calculatePID();
  
  // Define dead zones
  const int xDeadzone = 20;  // Pixels from center
  const int yDeadzone = 20;  // Pixels from center
  
  // Check if we're close enough to target position
  bool xCentered = abs(xInput) < xDeadzone;
  bool yCentered = abs(yInput) < yDeadzone;
  
  if (xCentered && yCentered) {
    // Ball is centered, stop motors
    stopMotors();
    return;
  }
  
  // Determine motor actions based on PID outputs
  int leftSpeed = 0;
  int rightSpeed = 0;
  
  // Handle X-axis control (rotation)
  if (!xCentered) {
    int turnSpeed = constrain(abs(xOutput), 120, baseSpeed);
    
    if (xOutput > 0) {
      // Need to turn left (ball is to the right of target)
      leftSpeed -= turnSpeed;
      rightSpeed += turnSpeed;
    } else {
      // Need to turn right (ball is to the left of target)
      leftSpeed += turnSpeed;
      rightSpeed -= turnSpeed;
    }
  }
  
  // Handle Y-axis control (forward/backward)
  if (!yCentered) {
    int moveSpeed = constrain(abs(yOutput), 120, baseSpeed);
    
    if (yOutput > 0) {
      // Need to move forward (ball is below target)
      leftSpeed += moveSpeed;
      rightSpeed += moveSpeed;
    } else {
      // Need to move backward (ball is above target)
      leftSpeed -= moveSpeed;
      rightSpeed -= moveSpeed;
    }
  }
  
  // Apply final motor speeds
  if (leftSpeed > 0 && rightSpeed > 0) {
    // Moving forward with possible rotation
    int adjLeftSpeed = leftSpeed * leftMotorCompensation;
    int adjRightSpeed = rightSpeed * rightMotorCompensation;
    
    // Set motor directions and speeds
    digitalWrite(leftMotorDir1, HIGH);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed, 120, 255));
    
    digitalWrite(rightMotorDir1, HIGH);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else if (leftSpeed < 0 && rightSpeed < 0) {
    // Moving backward with possible rotation
    int adjLeftSpeed = abs(leftSpeed) * leftMotorCompensation;
    int adjRightSpeed = abs(rightSpeed) * rightMotorCompensation;
    
    // Set motor directions and speeds
    digitalWrite(leftMotorDir1, LOW);
    digitalWrite(leftMotorDir2, HIGH);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed, 120, 255));
    
    digitalWrite(rightMotorDir1, LOW);
    digitalWrite(rightMotorDir2, HIGH);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else if (leftSpeed >= 0 && rightSpeed <= 0) {
    // Turning right
    int adjLeftSpeed = abs(leftSpeed) * leftMotorCompensation;
    int adjRightSpeed = abs(rightSpeed) * rightMotorCompensation;
    
    // Set motor directions and speeds
    digitalWrite(leftMotorDir1, HIGH);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed, 120, 255));
    
    digitalWrite(rightMotorDir1, LOW);
    digitalWrite(rightMotorDir2, HIGH);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else if (leftSpeed <= 0 && rightSpeed >= 0) {
    // Turning left
    int adjLeftSpeed = abs(leftSpeed) * leftMotorCompensation;
    int adjRightSpeed = abs(rightSpeed) * rightMotorCompensation;
    
    // Set motor directions and speeds
    digitalWrite(leftMotorDir1, LOW);
    digitalWrite(leftMotorDir2, HIGH);
    analogWrite(leftMotorPWM, constrain(adjLeftSpeed,120, 255));
    
    digitalWrite(rightMotorDir1, HIGH);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, constrain(adjRightSpeed, 120, 255));
  } 
  else {
    // Fallback - stop
    stopMotors();
  }
}

// Movement functions (unchanged)
void BallTracker::moveForward(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed,120, 255);
  rightSpeed = constrain(rightSpeed, 120, 255);
  
  // Left motor forward
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor forward
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BallTracker::moveBackward(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 120, 255);
  rightSpeed = constrain(rightSpeed, 120, 255);
  
  // Left motor backward
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor backward
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BallTracker::turnLeft(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 120, 255);
  rightSpeed = constrain(rightSpeed, 120, 255);
  
  // Left motor backward
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor forward
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BallTracker::turnRight(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 120, 255);
  rightSpeed = constrain(rightSpeed, 120, 255);
  
  // Left motor forward
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor backward
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BallTracker::stopMotors() {
  // Stop both motors
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, 0);
  
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, 0);
}
