#include "LineFollower.h"

// Constructor - Modified to remove AutoTune
LineFollower::LineFollower(int leftIR, int rightIR, 
                         int leftPWM, int leftD1, int leftD2, 
                         int rightPWM, int rightD1, int rightD2,
                         int trig, int echo) {
  // Store pin numbers
  leftIRPin = leftIR;
  rightIRPin = rightIR;
  leftMotorPWM = leftPWM;
  leftMotorDir1 = leftD1;
  leftMotorDir2 = leftD2;
  rightMotorPWM = rightPWM;
  rightMotorDir1 = rightD1;
  rightMotorDir2 = rightD2;
  trigPin = trig;
  echoPin = echo;
  
  // Default PID values
  Kp = 50.0;
  Ki = 0.01;
  Kd = 20.0;
  
  // PID_v1 setup
  lineSetpoint = 0; // We want error to be 0 (line centered)
  lineInput = 0;    // Current error
  lineOutput = 0;   // Output to adjust motor speeds
  
  // Initialize with NULL, will create in begin()
  linePID = NULL;
  
  // Motor parameters
  leftMotorCompensation = 1.0;
  rightMotorCompensation = 0.82;
  baseSpeed = 220;
  
  // Tracking variables
  inTurnState = false;
  tJunctionCount = 0;
  lastTJunctionTime = 0;
  
  // Obstacle detection
  obstacleDetected = false;
  obstacleAvoidanceMode = false;
  obstacleAvoidanceState = 0;
  lastObstacleTime = 0;
  initialStopTime = 500;
  turnTime = 1000;
  stabilizePauseTime = 300;
  forwardAfterObstacleTime = 500;
  
  // Line following state tracking
  lastLineState = LINE_STATE_UNKNOWN;
  lineState = LINE_STATE_UNKNOWN;
  stateChangeTime = 0;

  continuousTurnTime = 0;
}

// Destructor to clean up PID objects
LineFollower::~LineFollower() {
  if (linePID != NULL) {
    delete linePID;
  }
}

void LineFollower::begin() {
  // Set pin modes
  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initialize PID controller
  linePID = new PID(&lineInput, &lineOutput, &lineSetpoint, Kp, Ki, Kd, DIRECT);
  linePID->SetMode(AUTOMATIC);
  linePID->SetOutputLimits(-baseSpeed, baseSpeed);
  linePID->SetSampleTime(10);
  
  // Initialize motors to stop
  stopMotors();
}

// Implementation of the measurePulse function
unsigned long LineFollower::measurePulse(uint8_t pin, uint8_t state, unsigned long timeout) {
  unsigned long start = micros();
  unsigned long timeout_time = start + timeout;
  
  // Wait for pulse to start
  while (digitalRead(pin) != state) {
    if (micros() > timeout_time) return 0;
  }
  
  // Measure pulse width
  start = micros();
  while (digitalRead(pin) == state) {
    if (micros() > timeout_time) return 0;
  }
  
  return micros() - start;
}

// Set PID parameters
void LineFollower::setPID(double p, double i, double d) {
  Kp = p;
  Ki = i;
  Kd = d;
  
  if (linePID != NULL) {
    linePID->SetTunings(Kp, Ki, Kd);
  }
}

// Set motor compensation
void LineFollower::setMotorCompensation(float left, float right) {
  leftMotorCompensation = left;
  rightMotorCompensation = right;
}

// Set base speed
void LineFollower::setBaseSpeed(int speed) {
  baseSpeed = speed;
  
  // Update PID output limits if PID is initialized
  if (linePID != NULL) {
    linePID->SetOutputLimits(-baseSpeed, baseSpeed);
  }
}

// Check for obstacles
bool LineFollower::checkObstacle(int servoAngle) {
  // Only check straight ahead (when servo is at around 80 degrees)
  if (servoAngle >= 70 && servoAngle <= 90) {
    float distance = readDistance();
    // Consider an obstacle if distance is less than 15cm
    if (distance < 10.0) {
      obstacleDetected = true;
      return true;
    }
  }
  obstacleDetected = false;
  return false;
}

// Read distance from ultrasonic sensor
float LineFollower::readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Use optimized pulse measurement instead of pulseIn()
  long duration = measurePulse(echoPin, HIGH, 23200); // ~4m max range
  float distance = duration * 0.034 / 2;
  if (distance == 0) return 100;
  
  return distance;
}

// Determine line state based on sensor readings
LineState LineFollower::determineLineState() {
  bool leftSensor = digitalRead(leftIRPin);
  bool rightSensor = digitalRead(rightIRPin);
  
  // T-junction detection (both sensors detect the line)
  if (leftSensor == LOW && rightSensor == LOW) {
    // Only count if enough time has passed since last junction
    if (millis() - lastTJunctionTime > 1000) {
      tJunctionCount++;
      lastTJunctionTime = millis();
    }
    return LINE_STATE_TJUNCTION;
  }
  // Line is left of center
  else if (leftSensor == LOW && rightSensor == HIGH) {
    return LINE_STATE_LEFT;
  }
  // Line is right of center
  else if (leftSensor == HIGH && rightSensor == LOW) {
    return LINE_STATE_RIGHT;
  }
  // No line detected
  else if (leftSensor == HIGH && rightSensor == HIGH) {
    return LINE_STATE_LOST;
  }
  
  // Default case - should not reach here
  return lastLineState;
}

// Set individual motor direction and speed - Fixed to ensure proper motor activation
void LineFollower::setMotor(byte dirPin1, byte dirPin2, int pwmValue, byte motorPWM) {
  if (pwmValue >= 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    analogWrite(motorPWM, pwmValue);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    analogWrite(motorPWM, -pwmValue);
  }
}

// Implementation of moveForwardDifferential in LineFollower.cpp
void LineFollower::moveForwardDifferential(int leftSpeed, int rightSpeed) {
  // For 01 state (left sensor on line), right motor is faster
  // For 10 state (right sensor on line), left motor is faster
  
  if (lineState == LINE_STATE_LEFT) {  // 01 state
    int actualLeftSpeed = leftSpeed * leftMotorCompensation;
    int actualRightSpeed = rightSpeed * rightMotorCompensation * 1.5; // Right motor faster
    
    actualLeftSpeed = constrain(actualLeftSpeed, 0, 255);
    actualRightSpeed = constrain(actualRightSpeed, 0, 255);
    
    digitalWrite(leftMotorDir1, HIGH);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, actualLeftSpeed);
    
    digitalWrite(rightMotorDir1, HIGH);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, actualRightSpeed);
  }
  else if (lineState == LINE_STATE_RIGHT) {  // 10 state
    int actualLeftSpeed = leftSpeed * leftMotorCompensation * 1.5; // Left motor faster
    int actualRightSpeed = rightSpeed * rightMotorCompensation;
    
    actualLeftSpeed = constrain(actualLeftSpeed, 0, 255);
    actualRightSpeed = constrain(actualRightSpeed, 0, 255);
    
    digitalWrite(leftMotorDir1, HIGH);
    digitalWrite(leftMotorDir2, LOW);
    analogWrite(leftMotorPWM, actualLeftSpeed);
    
    digitalWrite(rightMotorDir1, HIGH);
    digitalWrite(rightMotorDir2, LOW);
    analogWrite(rightMotorPWM, actualRightSpeed);
  }
}


// Forward movement with same speed for both motors
void LineFollower::moveForward(int speed) {
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

// Turn left at specified speed
void LineFollower::turnRight(int speed) {
  int actualSpeed = speed;
  actualSpeed = constrain(actualSpeed, 0, 255);
  
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, 210);
  
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, 210 *  1.2);
}

// Turn right at specified speed
void LineFollower::turnLeft(int speed) {
  int actualSpeed = speed;
  actualSpeed = constrain(actualSpeed, 0, 255);
  
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, 210);
  
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, 210 *  1.2);
}

// Backward movement at specified speed
void LineFollower::moveBackward(int speed) {
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

// Stop all motors
void LineFollower::stopMotors() {
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, 0);
  
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, 0);
}

// Set obstacle avoidance timing parameters
void LineFollower::setObstacleAvoidanceTiming(unsigned long stopTime, unsigned long turn, 
                                             unsigned long pause, unsigned long forwardTime) {
  initialStopTime = stopTime;
  turnTime = turn;
  stabilizePauseTime = pause;
  forwardAfterObstacleTime = forwardTime;
}

// Main update function - Completely revised for better line following
int LineFollower::update(int servoAngle) {
  int requestedServoAngle = 80;
  unsigned long currentTime = millis();
  
  // Check for obstacles if not already in avoidance mode
  if (!obstacleAvoidanceMode) {
    obstacleDetected = checkObstacle(servoAngle);
    
    if (obstacleDetected) {
      obstacleAvoidanceMode = true;
      obstacleAvoidanceState = 0;
      lastObstacleTime = currentTime;
      return 80;
    }
  }
  
 // Obstacle avoidance logic - Modified to avoid on right side
if (obstacleAvoidanceMode) {
  switch (obstacleAvoidanceState) {
    case 0: // Stop when obstacle detected
      stopMotors();
      
      if (currentTime - lastObstacleTime > initialStopTime) {
        obstacleAvoidanceState = 1;
        lastObstacleTime = currentTime;
      }
      requestedServoAngle = 80;
      break;
      
    case 1: // Turn servo to check right side
      stopMotors();
      requestedServoAngle = 0; // Look right
      
      if (currentTime - lastObstacleTime > stabilizePauseTime) {
        obstacleAvoidanceState = 2;
        lastObstacleTime = currentTime;
      }
      break;
      
    case 2: // Check distance on right side
      stopMotors();
      // Continue to next state regardless of distance on right
      if (currentTime - lastObstacleTime > stabilizePauseTime) {
        obstacleAvoidanceState = 3;
        lastObstacleTime = currentTime;
      }
      requestedServoAngle = 80; // Keep looking right
      break;
      
    case 3: // Turn right
      turnRight(baseSpeed);
      
      if (currentTime - lastObstacleTime > turnTime) {
        obstacleAvoidanceState = 4;
        lastObstacleTime = currentTime;
      }
      requestedServoAngle = 80;
      break;
      
    case 4: // Pause after turn to stabilize
      stopMotors();
      
      if (currentTime - lastObstacleTime > stabilizePauseTime) {
        obstacleAvoidanceState = 5;
        lastObstacleTime = currentTime;
      }
      requestedServoAngle = 80;
      break;
      
    case 5: // Move forward
      moveForward(baseSpeed);
      
      if (currentTime - lastObstacleTime > forwardAfterObstacleTime) {
        obstacleAvoidanceState = 6;
        lastObstacleTime = currentTime;
      }
      requestedServoAngle = 170;
      break;
      
    case 6: // Turn left to get back parallel to original path
      turnLeft(baseSpeed);
      
      if (currentTime - lastObstacleTime > turnTime*1.2) {
        obstacleAvoidanceState = 7;
        lastObstacleTime = currentTime;
      }
      requestedServoAngle = 170; // Look forward
      break;
      
    case 7: // Move forward
      moveForward(baseSpeed);
      
      if (currentTime - lastObstacleTime > forwardAfterObstacleTime) {
        obstacleAvoidanceState = 8;
        lastObstacleTime = currentTime;
      }
      requestedServoAngle = 80;
      break;
      
    case 8: // Go forward until finding the line
      moveForward(baseSpeed);
      
      // Exit if we find the line or if we've been going too long
      if (digitalRead(leftIRPin) == LOW || digitalRead(rightIRPin) == LOW) {
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

  
  
  // Read sensor values
  bool leftSensor = digitalRead(leftIRPin);   // 1 = no line, 0 = line
  bool rightSensor = digitalRead(rightIRPin); // 1 = no line, 0 = line
  
  // Track previous and current states
  lastLineState = lineState;
  
  // Determine current line state
if (leftSensor == HIGH && rightSensor == HIGH) {
  lineState = LINE_STATE_LOST;      // 11 - forward
} else if (leftSensor == LOW && rightSensor == LOW) {
  lineState = LINE_STATE_TJUNCTION; // 00 - T-junction
  
  // Only count T-junction if robot was going straight forward before
  if (lastLineState == LINE_STATE_LOST && millis() - lastTJunctionTime > 500) {
    tJunctionCount++;
    lastTJunctionTime = millis();
  }
} else if (leftSensor == HIGH && rightSensor == LOW) {
  lineState = LINE_STATE_RIGHT;     // 10 - turn right
} else {
  lineState = LINE_STATE_LEFT;      // 01 - turn left
}

// Check if state has changed
if (lineState != lastLineState) {
  stateChangeTime = currentTime;
  // Reset continuous turn time when state changes
  continuousTurnTime = 0;
} else if ((lineState == LINE_STATE_LEFT || lineState == LINE_STATE_RIGHT) && 
          lastLineState == lineState) {
  // Increment continuous turn time if staying in the same turning state
  continuousTurnTime = currentTime - stateChangeTime;
}

// Stop completely after 5 T-junction detections only if going straight
if (tJunctionCount >= 5) {
  stopMotors();
  return 80;
}
  // Calculate error for PID based on sensor state
if (lineState == LINE_STATE_LEFT) {
  lineInput = -1.0;  // Line is to the left
} else if (lineState == LINE_STATE_RIGHT) {
  lineInput = 1.0;   // Line is to the right
} else {
  lineInput = 0.0;   // Centered or unknown
}

// Compute PID output
linePID->Compute();

  // Line following logic based on sensor state
  switch (lineState) {
    case LINE_STATE_LOST: // 11 - both sensors off line
      // Default behavior - move forward
      moveForward(baseSpeed);
      break;
      
    case LINE_STATE_TJUNCTION: // 00 - both sensors on line (T-junction)
      // Check if we were turning (potential 80° turn)
      if (lastLineState == LINE_STATE_LEFT || lastLineState == LINE_STATE_RIGHT) {
        // 80° turn detected - motors in opposite directions
        if (lastLineState == LINE_STATE_LEFT) {
          // Was turning left, now sharp left
          digitalWrite(leftMotorDir1, LOW);
          digitalWrite(leftMotorDir2, HIGH);
          analogWrite(leftMotorPWM, baseSpeed);
          
          digitalWrite(rightMotorDir1, HIGH);
          digitalWrite(rightMotorDir2, LOW);
          analogWrite(rightMotorPWM, baseSpeed);
        } else {
          // Was turning right, now sharp right
          digitalWrite(leftMotorDir1, HIGH);
          digitalWrite(leftMotorDir2, LOW);
          analogWrite(leftMotorPWM, baseSpeed);
          
          digitalWrite(rightMotorDir1, LOW);
          digitalWrite(rightMotorDir2, HIGH);
          analogWrite(rightMotorPWM, baseSpeed);
        }
      } else {
        // Straight T-junction - continue forward
        moveForward(baseSpeed);
      }
      break;
      
    case LINE_STATE_RIGHT: // 10 - right sensor on line
      if (continuousTurnTime < 10) {
        // Normal right turn - both motors forward, right slower
      int leftSpeed = baseSpeed;
      int rightSpeed = baseSpeed - abs(lineOutput);
      rightSpeed = max(rightSpeed, 200); // Prevent speed from being too low
        
        digitalWrite(leftMotorDir1, HIGH);
        digitalWrite(leftMotorDir2, LOW);
        analogWrite(leftMotorPWM, leftSpeed * leftMotorCompensation);
        
        digitalWrite(rightMotorDir1, HIGH);
        digitalWrite(rightMotorDir2, LOW);
        analogWrite(rightMotorPWM, rightSpeed * rightMotorCompensation);
      } else {
        // After 100ms in right turn, reverse left motor for sharper turn
        digitalWrite(leftMotorDir1, LOW);
        digitalWrite(leftMotorDir2, HIGH);
        analogWrite(leftMotorPWM, baseSpeed * leftMotorCompensation);
        
        digitalWrite(rightMotorDir1, HIGH);
        digitalWrite(rightMotorDir2, LOW);
        analogWrite(rightMotorPWM, baseSpeed * rightMotorCompensation);
      }
      break;
      
    case LINE_STATE_LEFT: // 01 - left sensor on line
      if (continuousTurnTime < 10) {
        int leftSpeed = baseSpeed - abs(lineOutput);
        int rightSpeed = baseSpeed;
        leftSpeed = max(leftSpeed, 200); // Prevent speed from being too low
        
        digitalWrite(leftMotorDir1, HIGH);
        digitalWrite(leftMotorDir2, LOW);
        analogWrite(leftMotorPWM, leftSpeed * leftMotorCompensation);
        
        digitalWrite(rightMotorDir1, HIGH);
        digitalWrite(rightMotorDir2, LOW);
        analogWrite(rightMotorPWM, rightSpeed * rightMotorCompensation);
      } else {
        // After 100ms in left turn, reverse right motor for sharper turn
        digitalWrite(leftMotorDir1, HIGH);
        digitalWrite(leftMotorDir2, LOW);
        analogWrite(leftMotorPWM, baseSpeed * leftMotorCompensation);
        
        digitalWrite(rightMotorDir1, LOW);
        digitalWrite(rightMotorDir2, HIGH);
        analogWrite(rightMotorPWM, baseSpeed * rightMotorCompensation);
      }
      break;
      
    default:
      // Default to prev state
      break;
  }
  
  return 80; // Default servo angle
}