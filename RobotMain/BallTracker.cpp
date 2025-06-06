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
  
  leftMotorCompensation = 1.0;
  rightMotorCompensation = 0.82;
  
  // Initialize ball position (unchanged)
  ballX = -1;  // -1 means no ball detected 
  ballY = -1;
  
  // Default target position (center of frame) (unchanged)
  targetX = 160;  // Assuming 640x480 resolution
  targetY = 120;
  
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

float BallTracker::getUltrasonicDistance() {
  // Envoie une impulsion de 10µs sur le trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Lit la durée du signal ECHO
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms

  // Convertit la durée en distance (cm)
  float distance_cm = duration * 0.034 / 2;

  // Si aucun écho reçu
  if (duration == 0) return -1;

  return distance_cm;
}

void BallTracker::SetError(float er){
  error = er;
}

void BallTracker::Set_ball_radius(float R){
  ballR = R;
}
float BallTracker::PID_Compute() {
    if (error == 2) return 0; // balle non détectée

    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float output = kp * error + ki * integral + kd * derivative;
    return output;
}

void BallTracker::setMotorSpeeds(float *vitesse_gauche,float * vitesse_droite) {
  float commande = PID_Compute();
     *vitesse_gauche = 110 - commande;
     *vitesse_droite = 110 + commande;

    // Clamp (optionnel)
    if (*vitesse_gauche > 255) *vitesse_gauche = 255;
    if (*vitesse_gauche < 0) *vitesse_gauche = 0;

    if (*vitesse_droite > 255) *vitesse_droite = 255;
    if (*vitesse_droite < 0) *vitesse_droite = 0;
}

void BallTracker::updateControl() {
  if(error == 2){
    turnRight(100);
  }
  if(ballR>60 || getUltrasonicDistance()<10){
    stopMotors();
  }
    float vitesse_gauche ;
    float vitesse_droite ;
    setMotorSpeeds(&vitesse_gauche,&vitesse_droite);
  
  // Left motor forward
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, vitesse_gauche);
  
  // Right motor forward
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, vitesse_droite);
  
}
