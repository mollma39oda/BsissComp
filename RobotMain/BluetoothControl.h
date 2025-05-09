
#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <Arduino.h>

class BluetoothControl {
  private:
    // Pin definitions for motors
    int leftMotorPWM;
    int leftMotorDir1;
    int leftMotorDir2;
    int rightMotorPWM;
    int rightMotorDir1;
    int rightMotorDir2;
    
    // Motor compensation
    float leftMotorCompensation;
    float rightMotorCompensation;
    
    // Command variables
    char mode; // 3 chars + null terminator
    int xPosition;
    int yPosition;
    int velocity;
    
    // Control state
    bool isActive;
    
  public:
    // Constructor
    BluetoothControl(int leftPWM, int leftD1, int leftD2, 
                      int rightPWM, int rightD1, int rightD2);
    
    // Initialize
    void begin();
    
    // Set motor compensation
    void setMotorCompensation(float left, float right);
    
    // Process command from Bluetooth
    void processCommand(String command);
    
    // Update control based on joystick values
    void updateControl();
    
    // Movement functions
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stopMotors();
    
    // Get functions
    // Get functions
    char getMode() { return mode; }
    int getXPosition() { return xPosition; }
    int getYPosition() { return yPosition; }
    int getVelocity() { return velocity; }
    bool isControlActive() { return isActive; }
    
    // Set active state
    void setActive(bool active) { isActive = active; }
};

#endif