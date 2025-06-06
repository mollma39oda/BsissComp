#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include <PID_v1.h>

enum LineState {
  LINE_STATE_UNKNOWN,
  // Add other states if needed
};

class LineFollower {
public:
  LineFollower(int farLeftIR, int leftIR, int centerIR, int rightIR, int farRightIR,
               int leftPWM, int leftD1, int leftD2, 
               int rightPWM, int rightD1, int rightD2,
               int trig, int echo);

  ~LineFollower();
  void begin();
  void setPID(double p, double i, double d);
  void setMotorCompensation(float left, float right);
  void setBaseSpeed(int speed);
  float readDistance();
  bool checkObstacle(int servoAngle);
  void setObstacleAvoidanceTiming(unsigned long stopTime, unsigned long turn, 
                                  unsigned long pause, unsigned long forwardTime);

  // Main update function
  int update(int servoAngle);

private:
  int determineLineError();

  // Pins
  int farLeftIRPin, leftIRPin, centerIRPin, rightIRPin, farRightIRPin;
  int leftMotorPWM, leftMotorDir1, leftMotorDir2;
  int rightMotorPWM, rightMotorDir1, rightMotorDir2;
  int trigPin, echoPin;

  // PID
  double Kp, Ki, Kd;
  double lineSetpoint, lineInput, lineOutput;
  PID* linePID;

  // Motor compensation
  float leftMotorCompensation, rightMotorCompensation;
  int baseSpeed;

  // State
  LineState lineState, lastLineState;
  unsigned long stateChangeTime;
  unsigned long turnDuration;
  unsigned long continuousTurnTime, maxContinuousTurnTime;

  // Obstacle
  bool obstacleDetected;
  bool obstacleAvoidanceMode;
  int obstacleAvoidanceState;
  unsigned long lastObstacleTime;
  unsigned long initialStopTime, turnTime, stabilizePauseTime, forwardAfterObstacleTime;
  int tJunctionCount;
  bool inTurnState;
  unsigned long lastTJunctionTime;

  int previousLineError;
  int lastNonZeroError;
  int allBlackCount;

  // Motor control helpers
  void setMotor(byte dirPin1, byte dirPin2, int pwmValue, byte motorPWM);
  void moveForward(int speed);
  void moveForwardDifferential(int leftSpeed, int rightSpeed);
  void turnLeft(int speed);
  void turnRight(int speed);
  void moveBackward(int speed);
  void stopMotors();

  unsigned long measurePulse(uint8_t pin, uint8_t state, unsigned long timeout);
};

#endif // LINEFOLLOWER_H