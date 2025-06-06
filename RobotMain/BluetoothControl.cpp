// BluetoothControl.cpp - Implementation of Bluetooth control

#include "BluetoothControl.h"

BluetoothControl::BluetoothControl(int leftPWM, int leftD1, int leftD2, 
                                   int rightPWM, int rightD1, int rightD2) {
  // Store pin numbers
  leftMotorPWM = leftPWM;
  leftMotorDir1 = leftD1;
  leftMotorDir2 = leftD2;
  rightMotorPWM = rightPWM;
  rightMotorDir1 = rightD1;
  rightMotorDir2 = rightD2;

  leftMotorCompensation = 1.0;
  rightMotorCompensation = 0.82;

  // Initialize mode and control variables
  // Initialize mode variable
  mode = 'I'; // 'I' for idle

  xPosition = 0;
  yPosition = 0;
  velocity = 0;
  
  // Initialize timeout tracking
  lastCommandTime = millis();
  
  isActive = false;
}

void BluetoothControl::begin() {
  // Set pin modes
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);
  
  // Initialize motors to stop
  stopMotors();
}

void BluetoothControl::setMotorCompensation(float left, float right) {
  leftMotorCompensation = left;
  rightMotorCompensation = right;
}

// Update the processCommand method to work with const char* instead of String
void BluetoothControl::processCommand(const char* command) {
  // Print received command for debugging
  Serial.print("Received command: ");
  Serial.println(command);
  
  // Check if command is valid
  if (!command || command[0] == '\0') {
    Serial.println("Command is empty");
    return;
  }
  
  // Handle mode commands (single character)
  if (strlen(command) == 1) {
    mode = command[0];
    Serial.print("Mode set to: ");
    Serial.println(mode);
    return;
  }
  
  // Try to parse as joystick command (should be in X,Y,V format) dir,v
  char cmdCopy[32]; // Create a copy we can modify
  strncpy(cmdCopy, command, sizeof(cmdCopy) - 1);
  cmdCopy[sizeof(cmdCopy) - 1] = '\0'; // Ensure null termination
  
  // Parse the values using strtok
        char* token = strtok(cmdCopy, ",");
  if (token) {
    direction = token;

    token = strtok(NULL, ",");
    if (token) {
      velocity = atoi(token);

      Serial.print("Direction : ");
      Serial.print(direction);
      Serial.print(" | Vitesse : ");
      Serial.println(velocity);
    } else {
      Serial.println("Valeur manquante après la virgule.");
    }
  } 
}

void BluetoothControl::updateControl() {
  if (!isActive) {
    stopMotors();
    return;
  }
  
  // Check for timeout - reset values if no changes in COMMAND_TIMEOUT ms
 /* if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    // Timeout occurred, reset values
    if (xPosition != 0 || yPosition != 0 || velocity != 0) {
      Serial.println("Command timeout - resetting values to 0,0,0");
      xPosition = 0;
      yPosition = 0;
      velocity = 0;
      stopMotors();
    }
  }*/
  
  // Add debug output to verify values being received
  Serial.print("Joystick: X=");
  Serial.print(direction);
  Serial.print(", V=");
  Serial.println(velocity);
  
  // Rest of your original updateControl code
 /* if (xPosition == 170 && yPosition == 140 || xPosition == 0 && yPosition == 0 || xPosition == 255 && yPosition == 255) {
    stopMotors(); // Joystick in center position
  }  
  if (210 > xPosition && xPosition > 130) {
    // Mainly forward/backward movement
    if (yPosition < 100) {
      moveForward(velocity);
    } else if (yPosition > 180) {
      moveBackward(velocity);
    }
  } 
  else if (yPosition < 180 && yPosition > 100) {
    // Mainly left/right turning
    if (xPosition > 210) {
      turnRight(velocity);
    } else if (xPosition < 130) {
      turnLeft(velocity);
    }
  } else {
    stopMotors();
  } */
  if(direction == "FW"){
     moveForward(velocity);
  }
  else if(direction ==  "BW"){
         moveBackward(velocity);
  }
  else if(direction ==  "LF"){
      turnLeft(velocity);
  }
  else if(direction == "RG"){
    turnRight(velocity);
  }
  else {
     stopMotors();
  }
}

void BluetoothControl::moveForward(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 0, 170);
  rightSpeed = constrain(rightSpeed, 0, 170);
  
  // Left motor forward
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor forward
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BluetoothControl::moveBackward(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 0, 170);
  rightSpeed = constrain(rightSpeed, 0, 170);
  
  // Left motor backward
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor backward
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BluetoothControl::turnLeft(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 0, 170);
  rightSpeed = constrain(rightSpeed, 0, 170);
  
  // Left motor backward
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor forward
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BluetoothControl::turnRight(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 0, 170);
  rightSpeed = constrain(rightSpeed, 0, 170);
  
  // Left motor forward
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor backward
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BluetoothControl::stopMotors() {
  // Stop both motors
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, 0);
  
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, 0);
}
