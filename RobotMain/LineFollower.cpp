#include "LineFollower.h"

// Special value for lost line
#define LINE_LOST 999

// Constructor
LineFollower::LineFollower(int farLeftIR, int leftIR, int centerIR, int rightIR, int farRightIR,
                         int leftPWM, int leftD1, int leftD2, 
                         int rightPWM, int rightD1, int rightD2,
                         int trig, int echo) {
  // Pin assignments for 5 IR sensors
  farLeftIRPin = farLeftIR;
  leftIRPin = leftIR;
  centerIRPin = centerIR;
  rightIRPin = rightIR;
  farRightIRPin = farRightIR;
  
  // FLIP: Assign left motor variables to right pins and vice versa
  leftMotorPWM = rightPWM;
  leftMotorDir1 = rightD1;
  leftMotorDir2 = rightD2;
  rightMotorPWM = leftPWM;
  rightMotorDir1 = leftD1;
  rightMotorDir2 = leftD2;
  
  trigPin = trig;
  echoPin = echo;

  // PID
  Kp = 40.0;
  Ki = 0.0;
  Kd = 18.0;
  lineSetpoint = 0;
  lineInput = 0;
  lineOutput = 0;
  linePID = nullptr;

  leftMotorCompensation = 1.0;
  rightMotorCompensation = 0.82;
  baseSpeed = 220;

  lineState = LINE_STATE_UNKNOWN;
  lastLineState = LINE_STATE_UNKNOWN;
  stateChangeTime = 0;
  turnDuration = 0;
  continuousTurnTime = 0;
  maxContinuousTurnTime = 0;

  obstacleDetected = false;
  obstacleAvoidanceMode = false;
  obstacleAvoidanceState = 0;
  lastObstacleTime = 0;
  initialStopTime = 350;
  turnTime = 150;
  stabilizePauseTime = 180;
  forwardAfterObstacleTime = 50;
  tJunctionCount = 0;
  inTurnState = false;
  lastTJunctionTime = 0;

  previousLineError = 0;
  lastNonZeroError = 0;
  allBlackCount = 0;
}

LineFollower::~LineFollower() {
  if (linePID != nullptr) delete linePID;
}

void LineFollower::begin() {
  pinMode(farLeftIRPin, INPUT);
  pinMode(leftIRPin, INPUT);
  pinMode(centerIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  pinMode(farRightIRPin, INPUT);

  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  linePID = new PID(&lineInput, &lineOutput, &lineSetpoint, Kp, Ki, Kd, DIRECT);
  linePID->SetMode(AUTOMATIC);
  linePID->SetOutputLimits(-baseSpeed, baseSpeed);
  linePID->SetSampleTime(5);

  stopMotors();
}

unsigned long LineFollower::measurePulse(uint8_t pin, uint8_t state, unsigned long timeout) {
  unsigned long start = micros();
  unsigned long timeout_time = start + timeout;
  while (digitalRead(pin) != state) {
    if (micros() > timeout_time) return 0;
  }
  start = micros();
  while (digitalRead(pin) == state) {
    if (micros() > timeout_time) return 0;
  }
  return micros() - start;
}

void LineFollower::setPID(double p, double i, double d) {
  Kp = p;
  Ki = i;
  Kd = d;
  if (linePID) linePID->SetTunings(Kp, Ki, Kd);
}

void LineFollower::setMotorCompensation(float left, float right) {
  leftMotorCompensation = left;
  rightMotorCompensation = right;
}

void LineFollower::setBaseSpeed(int speed) {
  baseSpeed = speed;
  if (linePID) linePID->SetOutputLimits(-baseSpeed, baseSpeed);
}

float LineFollower::readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = measurePulse(echoPin, HIGH, 23200);
  float distance = duration * 0.034 / 2;
  if (distance < 5) return 50;
  return distance;
}

bool LineFollower::checkObstacle(int servoAngle) {
  if (servoAngle >= 70 && servoAngle <= 90) {
    float distance = readDistance();
    if (distance < 20.0) {
      obstacleDetected = true;
      return true;
    }
  }
  obstacleDetected = false;
  return false;
}

void LineFollower::setMotor(byte dirPin1, byte dirPin2, int pwmValue, byte motorPWM) {
  if (pwmValue >= 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    analogWrite(motorPWM, pwmValue);
  } else {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    analogWrite(motorPWM, -pwmValue);
  }
}

void LineFollower::moveForward(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, leftSpeed);

  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, rightSpeed);
}

void LineFollower::moveForwardDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed * leftMotorCompensation, 0, 255);
  rightSpeed = constrain(rightSpeed * rightMotorCompensation, 0, 255);

  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, leftSpeed);

  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, rightSpeed);
}

void LineFollower::turnLeft(int speed) {
  int actualSpeed = constrain(speed, 0, 255);
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, actualSpeed);

  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, actualSpeed);
}

void LineFollower::turnRight(int speed) {
  int actualSpeed = constrain(speed, 0, 255);
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, actualSpeed);

  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, actualSpeed);
}

void LineFollower::moveBackward(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, leftSpeed);

  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, rightSpeed);
}

void LineFollower::stopMotors() {
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, 0);
}

void LineFollower::setObstacleAvoidanceTiming(unsigned long stopTime, unsigned long turn, 
                                              unsigned long pause, unsigned long forwardTime) {
  initialStopTime = stopTime;
  turnTime = turn;
  stabilizePauseTime = pause;
  forwardAfterObstacleTime = forwardTime;
}

int LineFollower::determineLineError() {
    // LOW = black, HIGH = white
    bool sensors[5] = {
      digitalRead(farLeftIRPin) == LOW,
      digitalRead(leftIRPin)     == HIGH,
      digitalRead(centerIRPin)   == HIGH,
      digitalRead(rightIRPin)    == HIGH,
      digitalRead(farRightIRPin) == LOW
    };
    int weights[5] = {-2, -1, 0, 1, 2};
    int sum = 0, count = 0;

    for (int i = 0; i < 5; i++) {
        if (sensors[i]) {
            sum += weights[i];
            count++;
        }
    }

    // All black: all 5 are black (LOW)
    if (count == 5) return 100;
    // All white: all 5 are white (HIGH)
    if (count == 0) return LINE_LOST;
    // Weighted average
    return (int)round((float)sum / count);
}

int LineFollower::update(int servoAngle) {
    int requestedServoAngle = 80;
    unsigned long currentTime = millis();

    // --- Obstacle avoidance logic ---
    if (!obstacleAvoidanceMode) {
        obstacleDetected = checkObstacle(servoAngle);
        if (obstacleDetected) {
            obstacleAvoidanceMode = true;
            obstacleAvoidanceState = 0;
            lastObstacleTime = currentTime;
            return 80;
        }
    }
    if (obstacleAvoidanceMode) {
        switch (obstacleAvoidanceState) {
            case 0:
                stopMotors();
                if (currentTime - lastObstacleTime > initialStopTime) {
                    obstacleAvoidanceState = 1;
                    lastObstacleTime = currentTime;
                }
                requestedServoAngle = 80;
                break;
            case 1:
                stopMotors();
                requestedServoAngle = 0;
                if (currentTime - lastObstacleTime > stabilizePauseTime) {
                    obstacleAvoidanceState = 2;
                    lastObstacleTime = currentTime;
                }
                break;
            case 2:
                stopMotors();
                if (currentTime - lastObstacleTime > stabilizePauseTime) {
                    obstacleAvoidanceState = 3;
                    lastObstacleTime = currentTime;
                }
                requestedServoAngle = 80;
                break;
            case 3:
                turnRight(baseSpeed);
                if (currentTime - lastObstacleTime > 500) {
                    obstacleAvoidanceState = 4;
                    lastObstacleTime = currentTime;
                }
                requestedServoAngle = 80;
                break;
            case 4:
                stopMotors();
                if (currentTime - lastObstacleTime > stabilizePauseTime) {
                    obstacleAvoidanceState = 5;
                    lastObstacleTime = currentTime;
                }
                requestedServoAngle = 80;
                break;
            case 5:
                moveForward(baseSpeed);
                if (currentTime - lastObstacleTime > 400) {
                    obstacleAvoidanceState = 6;
                    lastObstacleTime = currentTime;
                }
                requestedServoAngle = 170;
                break;
            case 6:
                turnLeft(baseSpeed);
                if (currentTime - lastObstacleTime > 550) {
                    obstacleAvoidanceState = 7;
                    lastObstacleTime = currentTime;
                }
                requestedServoAngle = 170;
                break;
            case 7:
                moveForward(baseSpeed);
                if (currentTime - lastObstacleTime > 400) {
                    obstacleAvoidanceState = 8;
                    lastObstacleTime = currentTime;
                }
                requestedServoAngle = 80;
                break;
            case 8:
                moveForward(baseSpeed);
                if (digitalRead(leftIRPin) == LOW || digitalRead(rightIRPin) == HIGH ||
                    digitalRead(centerIRPin) == HIGH || digitalRead(farLeftIRPin) == HIGH ||
                    digitalRead(farRightIRPin) == LOW) {
                    obstacleAvoidanceMode = false;
                }
                if (currentTime - lastObstacleTime > turnTime * 5) {
                    obstacleAvoidanceMode = false;
                }
                requestedServoAngle = 80;
                break;
        }
        return requestedServoAngle;
    }

    // --- Responsive Line Following Logic ---
    int err = determineLineError();

    // All black: stop after 5 times
    if (err == 100) {
        allBlackCount++;
        if (allBlackCount >= 5) {
            stopMotors();
            return requestedServoAngle;
        }
        err = previousLineError;
    } else {
        allBlackCount = 0;
    }

    // For very fast robots, use aggressive PID
    lineInput = err;
    linePID->Compute();

    if (err == 0) {
        previousLineError = err;
        lastNonZeroError = 0;
        int leftSpeed = baseSpeed - lineOutput;
        int rightSpeed = baseSpeed + lineOutput;
        moveForwardDifferential(rightSpeed, leftSpeed);
    } else if (err == LINE_LOST) {
        // Lost the line! Repeat last steering
        if (lastNonZeroError < 0) {
            turnLeft(baseSpeed);
        } else if (lastNonZeroError > 0) {
            turnRight(baseSpeed);
        } else {
            moveForward(baseSpeed);
        }
    } else if (err < 0) {
        previousLineError = err;
        lastNonZeroError = err;
        turnLeft(baseSpeed);
    } else if (err > 0) {
        previousLineError = err;
        lastNonZeroError = err;
        turnRight(baseSpeed);
    }
    return requestedServoAngle;
}