#ifndef BALL_TRACKER_H
#define BALL_TRACKER_H

#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

class BallTracker {
  private:
    // Pin definitions for motors (unchanged)
    int leftMotorPWM;
    int leftMotorDir1;
    int leftMotorDir2;
    int rightMotorPWM;
    int rightMotorDir1;
    int rightMotorDir2;
    
    // Motor compensation (unchanged)
    float leftMotorCompensation;
    float rightMotorCompensation;
    
    // Ball position (unchanged)
    int ballX;
    int ballY;
    
    // Target position (center point) (unchanged)
    int targetX;
    int targetY;
    
    // PID for X axis (rotation) - Modified for PID_v1
    double KpX, KiX, KdX;
    double xSetpoint, xInput, xOutput;
    PID *xPID;
    
    // PID for Y axis (forward/backward) - Modified for PID_v1
    double KpY, KiY, KdY;
    double ySetpoint, yInput, yOutput;
    PID *yPID;
    
    // AutoTune variables for X-axis
    PID_ATune *xTuner;
    bool xTuningActive;
    
    // AutoTune variables for Y-axis
    PID_ATune *yTuner;
    bool yTuningActive;
    
    // AutoTune settings
    double aTuneStep, aTuneNoise;
    unsigned int aTuneLookBack;
    
    // Control state (unchanged)
    bool isActive;
    bool ballDetected;
    
    // Base speed (unchanged)
    int baseSpeed;
    
  public:
    // Constructor
    BallTracker(int leftPWM, int leftD1, int leftD2, 
                int rightPWM, int rightD1, int rightD2);
    
    // Destructor
    ~BallTracker();
    
    // Initialize
    void begin();
    
    // Set motor compensation (unchanged)
    void setMotorCompensation(float left, float right);
    
    // Set PID parameters - Modified for PID_v1
    void setPIDX(double p, double i, double d);
    void setPIDY(double p, double i, double d);
    
    // AutoTune methods for X-axis
    void startAutoTuneX();
    void stopAutoTuneX();
    bool isAutoTuningX() { return xTuningActive; }
    
    // AutoTune methods for Y-axis
    void startAutoTuneY();
    void stopAutoTuneY();
    bool isAutoTuningY() { return yTuningActive; }
    
    void autoTuneLoop();
    // Add to public section:
    void setXSampleTime(int newSampleTime) { xPID->SetSampleTime(newSampleTime); }
    void setYSampleTime(int newSampleTime) { yPID->SetSampleTime(newSampleTime); }

    // Other methods (mostly unchanged)
    void setBaseSpeed(int speed);
    void setTargetPosition(int x, int y);
    void updateBallPosition(int x, int y);
    void calculatePID();
    void updateControl();
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void stopMotors();
    int getBallX() { return ballX; }
    int getBallY() { return ballY; }
    bool isBallDetected() { return ballDetected; }
    void setActive(bool active) { isActive = active; }
    bool isTrackerActive() { return isActive; }
};

#endif
