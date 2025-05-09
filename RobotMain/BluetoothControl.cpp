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
  
  // Default values
  leftMotorCompensation = 1.0;
  rightMotorCompensation = 1.0;
  
  // Initialize mode and control variables
  // Initialize mode variable
  mode = 'I'; // 'I' for idle

  xPosition = 0;
  yPosition = 0;
  velocity = 0;
  
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

void BluetoothControl::processCommand(String command) {
  // Extract first character for mode if string is not empty
  if (command.length() > 0) {
    mode = command.charAt(0);
    
    // Check if this is a joystick command (contains commas)
    int firstComma = command.indexOf(',');
    if (firstComma > 0) {
      int secondComma = command.indexOf(',', firstComma + 1);
      int thirdComma = command.indexOf(',', secondComma + 1);
      
      if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
        // Parse the X,Y,V values
        xPosition = command.substring(0, firstComma).toInt();
        yPosition = command.substring(firstComma + 1, secondComma).toInt();
        velocity = command.substring(thirdComma + 1).toInt();
      }
    }
  }
}



void BluetoothControl::updateControl() {
  if (!isActive) {
    stopMotors();
    return;
  }
  
  // Determine action based on joystick position
  // Dead zone in the middle (-10 to 10)
  if (xPosition == 170 && yPosition == 140) {
    stopMotors(); // Joystick in center position
  } 
  else if (210 > xPosition && xPosition> 130) {
    // Mainly forward/backward movement
    if (yPosition > 180) {
      moveForward(velocity);
    } else if (yPosition < 100) {
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
  } 
  else {
    // Combined movement (differential steering)
    int leftSpeed, rightSpeed;
    
    if (yPosition > 0) {
      // Moving forward with turning
      if (xPosition > 0) {
        // Forward right
        turnRight(velocity);
      } else {
        // Forward left
        turnLeft(velocity);
      }
      
    } else {
      // Moving backward with turning
      if (xPosition > 0) {
        // Backward right
        turnRight(velocity);
      } else {
        // Backward left
        turnLeft(velocity);
      }
    }
  }
}

void BluetoothControl::moveForward(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 180, 255);
  rightSpeed = constrain(rightSpeed, 180, 255);
  
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
  leftSpeed = constrain(leftSpeed, 180, 255);
  rightSpeed = constrain(rightSpeed,180, 255);
  
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
  leftSpeed = constrain(leftSpeed, 180, 255);
  rightSpeed = constrain(rightSpeed, 180, 255);
  
  // Left motor backward
  digitalWrite(leftMotorDir1, LOW);
  digitalWrite(leftMotorDir2, HIGH);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor forward
  digitalWrite(rightMotorDir1, HIGH);
  digitalWrite(rightMotorDir2, LOW);
  analogWrite(rightMotorPWM, rightSpeed);
}

void BluetoothControl::turnRight(int speed) {
  int leftSpeed = speed * leftMotorCompensation;
  int rightSpeed = speed * rightMotorCompensation;
  
  // Ensure speeds are within bounds
  leftSpeed = constrain(leftSpeed, 180, 255);
  rightSpeed = constrain(rightSpeed, 180, 255);
  
  // Left motor forward
  digitalWrite(leftMotorDir1, HIGH);
  digitalWrite(leftMotorDir2, LOW);
  analogWrite(leftMotorPWM, leftSpeed);
  
  // Right motor backward
  digitalWrite(rightMotorDir1, LOW);
  digitalWrite(rightMotorDir2, HIGH);
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
