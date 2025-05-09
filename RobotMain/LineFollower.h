#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include <PID_v1.h>

// Define line states
enum LineState {
  LINE_STATE_UNKNOWN,
  LINE_STATE_LEFT,
  LINE_STATE_RIGHT,
  LINE_STATE_TJUNCTION,
  LINE_STATE_LOST
};

class LineFollower {
  private:
    // Pin definitions
    int leftIRPin;
    int rightIRPin;
    int leftMotorPWM;
    int leftMotorDir1;
    int leftMotorDir2;
    int rightMotorPWM;
    int rightMotorDir1;
    int rightMotorDir2;
    int trigPin;
    int echoPin;
    
    // PID control variables
    double Kp, Ki, Kd;
    double lineSetpoint, lineInput, lineOutput;
    PID *linePID;
    
    // Other variables
    float leftMotorCompensation;
    float rightMotorCompensation;
    int baseSpeed;
    
    // Line following state tracking
    LineState lineState;
    LineState lastLineState;
    unsigned long stateChangeTime;
    unsigned long turnDuration;
    unsigned long continuousTurnTime;
    unsigned long maxContinuousTurnTime;
    
    // Obstacle detection
    bool obstacleDetected;
    bool obstacleAvoidanceMode;
    int obstacleAvoidanceState;
    unsigned long lastObstacleTime;
    unsigned long initialStopTime;
    unsigned long turnTime;
    unsigned long stabilizePauseTime;
    unsigned long forwardAfterObstacleTime;
    byte tJunctionCount;
    bool inTurnState;
    unsigned long lastTJunctionTime;
    
    // Function to read ultrasonic sensor
    float readDistance();
    
    // Determine line state from sensor readings
    LineState determineLineState();
    
  public:
    // Constructor
    LineFollower(int leftIR, int rightIR, 
                int leftPWM, int leftD1, int leftD2, 
                int rightPWM, int rightD1, int rightD2,
                int trig, int echo);
    
    // Destructor to clean up PID objects
    ~LineFollower();
    
    // Initialize
    void begin();
    
    // Set PID parameters
    void setPID(double p, double i, double d);
    
    // Set PID sample time
    void setSampleTime(int newSampleTime) { linePID->SetSampleTime(newSampleTime); }

    // Optimized non-blocking pulse measurement for ultrasonic sensor
    unsigned long measurePulse(uint8_t pin, uint8_t state, unsigned long timeout);

    // Motor control methods
    void setMotorCompensation(float left, float right);
    void setBaseSpeed(int speed);
    bool checkObstacle(int servoAngle);
    void calculatePID();
    void moveForward(int speed);
    void moveForwardDifferential(int leftSpeed, int rightSpeed);  // New method
    void turnLeft(int speed);
    void turnRight(int speed);
    void moveBackward(int speed);
    void stopMotors();
    int update(int servoAngle);
    bool isObstacleDetected() { return obstacleDetected; }
    bool isAvoidingObstacle() { return obstacleAvoidanceMode; }
    int getObstacleState() { return obstacleAvoidanceState; }
    void setObstacleAvoidanceTiming(unsigned long stopTime, unsigned long turn, 
                                   unsigned long pause, unsigned long forwardTime);
    byte getTJunctionCount() { return tJunctionCount; }
    void resetTJunctionCount() { tJunctionCount = 0; }
    void setMotor(byte dirPin1, byte dirPin2, int pwmValue, byte motorPWM);
};

#endif
