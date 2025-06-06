// Optimized main.ino - Adapted for robust line loss handling
#include "LineFollower.h"
#include "BluetoothControl.h"
#include "BallTracker.h"
#include <PID_v1.h>
#include <avr/wdt.h> // Watchdog timer for auto-recovery

// Pin definitions using byte type to save memory
const byte LEFT_MOTOR_PWM = 9;
const byte LEFT_MOTOR_DIR1 = 8;
const byte LEFT_MOTOR_DIR2 = 7;
const byte RIGHT_MOTOR_PWM = 10;
const byte RIGHT_MOTOR_DIR1 = 12;
const byte RIGHT_MOTOR_DIR2 = 13;

// Updated IR sensor pins (from right to left: 6, A1, A0, 11, 5)
const byte FAR_RIGHT_IR_PIN = 6;
const byte RIGHT_IR_PIN = 11;
const byte CENTER_IR_PIN = 4;
const byte LEFT_IR_PIN = A1;
const byte FAR_LEFT_IR_PIN = 5;

const byte TRIG_PIN = A5;
const byte ECHO_PIN = A4;
// Button setup - add these at the global variable section
const byte Button = 2;  // Button pin
bool that = false;      // Toggle state
bool lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // Debounce time in milliseconds

// CENTRALIZED CONFIGURATION - easy to modify in one place
// Motor configuration
const float RIGHT_MOTOR_COMPENSATION = 1.2;
const float LEFT_MOTOR_COMPENSATION = 1.0;
const byte MIN_MOTOR_SPEED = 150; // Higher minimum speed to avoid stalling
const byte MAX_MOTOR_SPEED = 255;
const byte BASE_SPEED = 220;
// PID configuration
const double LINE_KP = 50.0;
const double LINE_KI = 0.01;
const double LINE_KD = 20.0;
const double BALL_X_KP = 1.0;
const double BALL_X_KI = 0.0;
const double BALL_X_KD = 0.0;
const double BALL_Y_KP = 1.0;
const double BALL_Y_KI = 0.0;
const double BALL_Y_KD = 0.0;
const int PID_SAMPLE_TIME = 10; // ms - reduced frequency saves CPU

// Obstacle avoidance timing parameters (in milliseconds)
const unsigned long INITIAL_STOP_TIME = 100;
const unsigned long TURN_TIME = 500;
const unsigned long STABILIZE_PAUSE_TIME = 50;
const unsigned long FORWARD_AFTER_OBSTACLE_TIME = 1000;

// Communication parameters
const unsigned long UART_BAUD = 57600;
const unsigned long COMM_INTERVAL = 100; // ms between updates

// Operating modes
enum OperatingMode : byte {
  MODE_IDLE,
  MODE_LINE_FOLLOWER,
  MODE_BLUETOOTH_CONTROL, 
  MODE_BALL_TRACKER
};

// Current state variables
OperatingMode currentMode = MODE_LINE_FOLLOWER;
char inputBuffer[32]; // Fixed size buffer instead of String
byte bufferPos = 0;
bool messageComplete = false;
unsigned long lastCommunicationTime = 0;
byte servoAngle = 80;
byte lastSentServoAngle = 255; // Initialize different to trigger first send
unsigned long lastServoUpdateTime = 0;

// T-junction counter
byte tJunctionCount = 0;
unsigned long lastTJunctionTime = 0;

// Create instances of modules - updated constructor call for LineFollower
LineFollower lineFollower(FAR_LEFT_IR_PIN, LEFT_IR_PIN, CENTER_IR_PIN, RIGHT_IR_PIN, FAR_RIGHT_IR_PIN, 
                         LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, 
                         RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2,
                         TRIG_PIN, ECHO_PIN);

BluetoothControl btControl(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, 
                          RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2);

BallTracker ballTracker(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, 
                        RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2);

void setup() {
  // Initialize Serial for communication
  Serial.begin(UART_BAUD);
  
  // Initialize all modules
  lineFollower.begin();
  btControl.begin();
  ballTracker.begin();
  
  // Set motor compensation for all modules
  lineFollower.setMotorCompensation(LEFT_MOTOR_COMPENSATION, RIGHT_MOTOR_COMPENSATION);
  btControl.setMotorCompensation(LEFT_MOTOR_COMPENSATION, RIGHT_MOTOR_COMPENSATION);
  ballTracker.setMotorCompensation(LEFT_MOTOR_COMPENSATION, RIGHT_MOTOR_COMPENSATION);
  
  // Set PID parameters and sample times for line follower
  lineFollower.setPID(LINE_KP, LINE_KI, LINE_KD);
  lineFollower.setBaseSpeed(BASE_SPEED);
    
  // Set PID parameters and sample times for ball tracker
  ballTracker.setPIDX(BALL_X_KP, BALL_X_KI, BALL_X_KD);
  ballTracker.setPIDY(BALL_Y_KP, BALL_Y_KI, BALL_Y_KD);
  ballTracker.setXSampleTime(PID_SAMPLE_TIME);
  ballTracker.setYSampleTime(PID_SAMPLE_TIME);
  ballTracker.setBaseSpeed(BASE_SPEED);

  // Set obstacle avoidance timing parameters
  lineFollower.setObstacleAvoidanceTiming(
    INITIAL_STOP_TIME, 
    TURN_TIME, 
    STABILIZE_PAUSE_TIME, 
    FORWARD_AFTER_OBSTACLE_TIME
  );
  
  // Initialize all IR sensor pins
  pinMode(FAR_LEFT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(CENTER_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  pinMode(FAR_RIGHT_IR_PIN, INPUT);
  pinMode(Button, INPUT);

  // Set target position for ball tracker (center of frame)
  ballTracker.setTargetPosition(320, 240);
  
  // Start in LINE_FOLLOWER mode by default
  switchMode(currentMode);
  
  // Enable watchdog timer to automatically reset if code hangs
  wdt_enable(WDTO_2S); // 2-second timeout
}

void loop() {
 // Read the button with debounce
   int reading = digitalRead(Button);
  int buttonState; 
  // Check if the button state has changed
  if (reading != lastButtonState) {
    // Reset debounce timer
    lastDebounceTime = millis();
  }
  
  // Only change state if debounce time has elapsed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If button state is stable and it's pressed (HIGH)
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        that = !that;
      }
    }
  }
  lastButtonState = reading;
   
  // Save the button state for next comparison
  
  // Reset watchdog timer
  wdt_reset();
  
  // Check for input from ESP32-CAM
  checkESPCommunication();
  
  // Handle each mode's operation
  switch (currentMode) {
    case MODE_LINE_FOLLOWER:
    if(that == true ){
      handleLineFollowerMode();
    }
    if(that == false){
        stopMotors();
    }
      break;
    case MODE_BLUETOOTH_CONTROL:
      handleBluetoothControlMode();
      break;
    case MODE_BALL_TRACKER:
      handleBallTrackerMode();
      break;
    case MODE_IDLE:
    default:
      stopMotors();
      break;
  }
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_DIR1, LOW);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
  analogWrite(LEFT_MOTOR_PWM, 0);
  
  digitalWrite(RIGHT_MOTOR_DIR1, LOW);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

void checkESPCommunication() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || bufferPos >= sizeof(inputBuffer)-1) {
      // End of message
      inputBuffer[bufferPos] = '\0'; // Null terminate
      messageComplete = true;
      bufferPos = 0;
      
      // Debug: Print the complete message
      Serial.print("Received message: ");
      Serial.println(inputBuffer);
      break;
    } else {
      // Add to buffer
      inputBuffer[bufferPos++] = c;
    }
  }
  
  // Process complete message
  if (messageComplete) {
    processESPMessage(inputBuffer);
    messageComplete = false;
    lastCommunicationTime = millis();
  }
   // Send servo angle update to ESP32-CAM only when changed
  if (millis() - lastServoUpdateTime > COMM_INTERVAL) {
    if (servoAngle != lastSentServoAngle) {
      // Use compact format: 'S' + angle as digits
      Serial.print("S:");
      Serial.println(servoAngle);
      lastSentServoAngle = servoAngle;
    }
    lastServoUpdateTime = millis();
  }
}

void processESPMessage(const char* message) {
  // Message format optimization
  // Debug: Print the message being processed
  Serial.print("Processing message: ");
  Serial.println(message);
   
  // Message format optimization
  if (message[0] == 0) return; // Empty message
  
  // Handle different message types based on first character
  switch (message[0]) {
    case 'B': // WiFi command (previously Bluetooth)
      // Pass only the command part (skip 'B')
      processWiFiCommand(message + 1);
      break;
      
    case 'J': // Special case for joystick data (if formatted as "Jx,y,v")
      // Pass joystick data to Bluetooth control (skip 'J')
      btControl.processCommand(message + 1);
      break;
      
    case 'X': // Ball position update - format: "Xnnn,nnn"
      {
        char* pos = strchr(message, ',');
        if (pos) {
          int x = atoi(message + 1);  // Skip the 'X'
          int y = atoi(pos + 1);     // Skip the comma
          ballTracker.updateBallPosition(x, y);
          ballTracker.SetError(x);
          ballTracker.Set_ball_radius(y);
        }
      }
      break;
      
    default:
      // If message contains commas, it might be direct joystick data
      if (strchr(message, ',')) {
        btControl.processCommand(message);
      }
      break;
  }
}

void processWiFiCommand(const char* command) {
  // Debug the incoming command
  Serial.print("WiFi command received: ");
  Serial.println(command);
  
  // Look for joystick data format (contains commas)
  if (strchr(command, ',')) {
    // This is likely joystick data, pass it to Bluetooth control
    btControl.processCommand(command);
    return;
  }
  
  // Handle mode commands
  if (command[0] != '\0') {
    OperatingMode mode;
    
    switch (command[0]) {
      case 'S':
        mode = MODE_IDLE;
        break;
      case 'A':
        mode = MODE_LINE_FOLLOWER;
        break;
      case 'M':
        mode = MODE_BLUETOOTH_CONTROL;
        break;
      case 'B':
        mode = MODE_BALL_TRACKER;
        break;
      default:
        return; // Invalid command
    }
    
    // Process the mode command in Bluetooth control as well
    btControl.processCommand(command);
    
    // Switch the robot's operating mode
    switchMode(mode);
  }
}

void switchMode(OperatingMode newMode) {
  // Only switch mode if it's different from current mode
  if (newMode == currentMode) return;
  
  // Deactivate all modules
  lineFollower.setBaseSpeed(0);
  btControl.setActive(false);
  ballTracker.setActive(false);
  
  // Activate the requested module
  switch (newMode) {
    case MODE_LINE_FOLLOWER:
      lineFollower.setBaseSpeed(BASE_SPEED);
      servoAngle = 80; // Reset servo to look forward
      Serial.println("L"); // 'L' for Line follower mode
      break;
      
    case MODE_BLUETOOTH_CONTROL:
      btControl.setActive(true);
      Serial.println("B"); // 'B' for Bluetooth/WiFi control mode
      break;
      
    case MODE_BALL_TRACKER:
      ballTracker.setActive(true);
      Serial.println("T"); // 'T' for Tracker mode
      break;
      
    case MODE_IDLE:
      Serial.println("I"); // 'I' for Idle mode
      break;
  }
  
  currentMode = newMode;
}

void handleLineFollowerMode() {
  // Update line follower and check for T-junctions or crossroads
  bool farLeftSensor = digitalRead(FAR_LEFT_IR_PIN);
  bool leftSensor = digitalRead(LEFT_IR_PIN);
  bool centerSensor = digitalRead(CENTER_IR_PIN);
  bool rightSensor = digitalRead(RIGHT_IR_PIN);
  bool farRightSensor = digitalRead(FAR_RIGHT_IR_PIN);
  
  // Detect T-junction or crossroad (3+ sensors detect line)
  int activeSensors = (!farLeftSensor) + (!leftSensor) + (!centerSensor) + (!rightSensor) + (!farRightSensor);
  if (activeSensors >= 3) {
    // Only count if enough time has passed since last junction
    if (millis() - lastTJunctionTime > 1000) {
      tJunctionCount++;
      lastTJunctionTime = millis();
    }
  }
  
  // Update line follower
  servoAngle = lineFollower.update(servoAngle); // Robust line loss handling is now in LineFollower.cpp
}

void handleBluetoothControlMode() {
  btControl.updateControl();
}

void handleBallTrackerMode() {
  if (ballTracker.isTrackerActive() && ballTracker.isBallDetected()) {
    ballTracker.updateControl();
  } else {
    // If ball not detected, spin to search for it
    ballTracker.turnRight(180);
  }
}