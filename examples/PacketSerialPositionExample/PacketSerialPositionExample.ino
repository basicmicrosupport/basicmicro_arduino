/**
 * Roboclaw PacketSerial Position Control Example
 * 
 * This sketch demonstrates position control using Basicmicro Roboclaw's
 * PacketSerial mode. It focuses on the SpeedAccelDecelPosition commands
 * for precise position control with speed ramping.
 * 
 * This implementation maintains an internal position tracker rather than
 * continuously reading the encoder values, which can be more efficient
 * and better represents the intended control paradigm.
 * 
 * Features demonstrated:
 * - Position tracking using expected positions
 * - Individual motor position control
 * - Synchronized dual motor positioning
 * - Absolute vs. relative positioning
 * - Different acceleration/deceleration profiles
 * 
 * Roboclaw Configuration:
 * 1. Connect Roboclaw to computer via USB
 * 2. Launch Basicmicro Motion Studio software
 * 3. Set 'Serial Mode' to 'Packet Serial'
 * 4. Configure PID parameters for position control
 * 5. Set baud rate to 38400 (or match the value in this sketch)
 * 6. Note the device address (default is 0x80)
 * 7. Click 'Write Settings' to save configuration
 * 
 * Hardware Setup:
 * - Arduino board with hardware serial (e.g., Arduino Mega, Due)
 * - Roboclaw motor controller 
 * - Two motors with quadrature encoders
 * - Connect Arduino TX pin to Roboclaw S1 (RX)
 * - Connect Arduino RX pin to Roboclaw S2 (TX)
 * - Connect Arduino GND to Roboclaw GND
 * 
 * Note: Position control requires properly tuned PID settings in the Roboclaw.
 * 
 * Created by: Your Name
 * Date: March 20, 2025
 */

#include <RoboClaw.h>

// Hardware Serial for Roboclaw (Serial1)
// For Arduino Mega/Due - use Serial1, Serial2, or Serial3
#define ROBOCLAW_SERIAL Serial1

// Roboclaw address (default is 0x80)
#define RC_ADDRESS 0x80

// Create RoboClaw object
RoboClaw roboclaw(&ROBOCLAW_SERIAL, 10000); // 10ms timeout

// Position control parameters
#define MAX_SPEED 2000       // Maximum speed in encoder counts per second
#define ACCEL_RATE 4000      // Acceleration rate in counts per second^2
#define DECEL_RATE 4000      // Deceleration rate in counts per second^2
#define FAST_SPEED 3000      // Fast speed for quick movements
#define SLOW_SPEED 1000      // Slow speed for precise movements
#define FAST_ACCEL 6000      // Fast acceleration
#define SLOW_ACCEL 2000      // Slow acceleration
#define FAST_DECEL 8000      // Fast deceleration
#define SLOW_DECEL 2000      // Slow deceleration

// Position definitions
#define HOME_POSITION 0
#define POSITION_1 1000      // 1000 encoder counts
#define POSITION_2 5000      // 5000 encoder counts
#define POSITION_3 10000     // 10000 encoder counts

// Timing constants
#define MOVE_TIMEOUT 15000   // Maximum time to wait for a movement (ms)
#define DISPLAY_INTERVAL 500 // How often to display position during movement (ms)

// Position tracking variables
int32_t expectedPositionM1 = 0;
int32_t expectedPositionM2 = 0;
bool positionInitialized = false;

// Demo status tracking
int currentDemo = 0;
int totalDemos = 6;

void setup() {
  // Initialize hardware serial for debugging
  Serial.begin(115200);
  Serial.println("Roboclaw PacketSerial Position Control Example");
  Serial.println("-------------------------------------------");
  
  // Initialize serial communication with Roboclaw
  ROBOCLAW_SERIAL.begin(38400);
  roboclaw.begin();
  
  // Safety first - make sure motors are stopped
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  
  Serial.println("Roboclaw initialized.");
  
  // Initialize position tracking by reading current encoder values
  initializePositionTracking();
  
  Serial.println("\nStarting position control demo sequence in 3 seconds...");
  delay(3000);
}

void loop() {
  // Run through a series of position control demonstrations
  
  switch(currentDemo) {
    case 0:
      demoBasicPositioning();
      break;
    case 1:
      demoSynchronizedPositioning();
      break;
    case 2:
      demoRelativePositioning();
      break;
    case 3:
      demoSpeedProfiles();
      break;
    case 4:
      demoSequentialPositions();
      break;
    case 5:
      demoBackToHome();
      break;
    default:
      // Reset the demo counter and start over
      currentDemo = 0;
      Serial.println("\n=== Restarting demo sequence ===\n");
      delay(3000);
      return;
  }
  
  // Move to the next demo
  currentDemo++;
  
  // Brief pause between demonstrations
  delay(2000);
}

/**
 * Initialize position tracking by reading encoder values once
 */
void initializePositionTracking() {
  if (positionInitialized) {
    return;
  }
  
  // Read the current encoder positions once at startup
  bool valid1, valid2;
  uint8_t status1, status2;
  
  expectedPositionM1 = roboclaw.ReadEncM1(RC_ADDRESS, &status1, &valid1);
  expectedPositionM2 = roboclaw.ReadEncM2(RC_ADDRESS, &status2, &valid2);
  
  if (valid1 && valid2) {
    Serial.println("Position tracking initialized");
    Serial.print("Initial positions: M1=");
    Serial.print(expectedPositionM1);
    Serial.print(", M2=");
    Serial.println(expectedPositionM2);
    
    positionInitialized = true;
  } else {
    Serial.println("Failed to initialize position tracking");
    // Try again in the next loop iteration
  }
}

/**
 * Optionally reset encoders and position tracking to zero
 */
void resetPositionTracking() {
  // Reset the Roboclaw encoders
  roboclaw.ResetEncoders(RC_ADDRESS);
  
  // Update our expected positions
  expectedPositionM1 = 0;
  expectedPositionM2 = 0;
  
  Serial.println("Encoders and position tracking reset to zero");
}

/**
 * Demonstration 1: Basic individual motor positioning
 */
void demoBasicPositioning() {
  Serial.println("\n=== Demo 1: Basic Individual Motor Positioning ===");
  
  // Move Motor 1 to Position 1
  Serial.print("Moving Motor 1 to position ");
  Serial.println(POSITION_1);
  bool success = moveToPosition(1, POSITION_1, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 1);
  
  delay(1000);
  
  // Move Motor 2 to Position 1
  Serial.print("Moving Motor 2 to position ");
  Serial.println(POSITION_1);
  success = moveToPosition(2, POSITION_1, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 2);
  
  delay(1000);
  
  // Move Motor 1 back to home
  Serial.println("Moving Motor 1 back to home position");
  success = moveToPosition(1, HOME_POSITION, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 1);
  
  delay(1000);
  
  // Move Motor 2 back to home
  Serial.println("Moving Motor 2 back to home position");
  success = moveToPosition(2, HOME_POSITION, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 2);
}

/**
 * Demonstration 2: Synchronized dual motor positioning
 */
void demoSynchronizedPositioning() {
  Serial.println("\n=== Demo 2: Synchronized Dual Motor Positioning ===");
  
  // Move both motors to Position 2 simultaneously
  Serial.print("Moving both motors to position ");
  Serial.println(POSITION_2);
  bool success = moveToPositionBoth(POSITION_2, POSITION_2, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
  
  delay(1500);
  
  // Move both motors to different positions simultaneously
  Serial.println("Moving motors to different positions");
  Serial.print("Motor 1 to ");
  Serial.print(POSITION_1);
  Serial.print(", Motor 2 to ");
  Serial.println(POSITION_3);
  
  success = moveToPositionBoth(POSITION_1, POSITION_3, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
  
  delay(1500);
  
  // Move both motors back to home
  Serial.println("Moving both motors back to home position");
  success = moveToPositionBoth(HOME_POSITION, HOME_POSITION, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
}

/**
 * Demonstration 3: Relative positioning
 */
void demoRelativePositioning() {
  Serial.println("\n=== Demo 3: Relative Positioning ===");
  
  // First, ensure we're at a known position
  moveToPosition(1, HOME_POSITION, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  moveToPosition(2, HOME_POSITION, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  
  // Move forward 1000 counts from current position
  Serial.println("Moving both motors +1000 counts from current position");
  int32_t newPosM1 = expectedPositionM1 + 1000;
  int32_t newPosM2 = expectedPositionM2 + 1000;
  
  bool success = moveToPositionBoth(newPosM1, newPosM2, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
  
  delay(1500);
  
  // Move backward 500 counts from current position
  Serial.println("Moving both motors -500 counts from current position");
  newPosM1 = expectedPositionM1 - 500;
  newPosM2 = expectedPositionM2 - 500;
  
  success = moveToPositionBoth(newPosM1, newPosM2, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
  
  delay(1500);
  
  // Move to exactly 1500 counts (absolute positioning)
  Serial.println("Moving both motors to absolute position 1500");
  success = moveToPositionBoth(1500, 1500, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
}

/**
 * Demonstration 4: Different speed and acceleration profiles
 */
void demoSpeedProfiles() {
  Serial.println("\n=== Demo 4: Speed and Acceleration Profiles ===");
  
  // First, ensure we're at a known position
  moveToPositionBoth(HOME_POSITION, HOME_POSITION, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  delay(1000);
  
  // Fast movement to Position 3
  Serial.println("Fast movement to position 3 (high speed, high accel)");
  bool success = moveToPositionBoth(
                   POSITION_3, POSITION_3, 
                   FAST_SPEED, FAST_SPEED, 
                   FAST_ACCEL, FAST_DECEL);
  displayResult(success, 0);
  
  delay(1500);
  
  // Slow, precise movement back to Position 2
  Serial.println("Slow, precise movement to position 2 (low speed, low accel)");
  success = moveToPositionBoth(
              POSITION_2, POSITION_2, 
              SLOW_SPEED, SLOW_SPEED, 
              SLOW_ACCEL, SLOW_DECEL);
  displayResult(success, 0);
  
  delay(1500);
  
  // Mixed speed profile (fast accel, slow speed)
  Serial.println("Mixed profile: Fast accel, slow speed, fast decel");
  success = moveToPositionBoth(
              POSITION_1, POSITION_1, 
              SLOW_SPEED, SLOW_SPEED, 
              FAST_ACCEL, FAST_DECEL);
  displayResult(success, 0);
  
  delay(1500);
  
  // Back to home with standard profile
  Serial.println("Back to home with standard profile");
  success = moveToPositionBoth(
              HOME_POSITION, HOME_POSITION, 
              MAX_SPEED, MAX_SPEED, 
              ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
}

/**
 * Demonstration 5: Sequential position movements
 */
void demoSequentialPositions() {
  Serial.println("\n=== Demo 5: Sequential Position Movements ===");
  
  // Define a sequence of positions to visit
  int32_t positions[] = {POSITION_1, POSITION_2, POSITION_3, POSITION_1, HOME_POSITION};
  
  for (int i = 0; i < 5; i++) {
    Serial.print("Moving to position ");
    Serial.println(positions[i]);
    
    bool success = moveToPositionBoth(
                     positions[i], positions[i], 
                     MAX_SPEED, MAX_SPEED, 
                     ACCEL_RATE, DECEL_RATE);
    displayResult(success, 0);
    
    // Shorter delay between sequential positions
    delay(1000);
  }
}

/**
 * Demonstration 6: Return to home position
 */
void demoBackToHome() {
  Serial.println("\n=== Demo 6: Back to Home Position ===");
  
  // Move to a non-zero position first
  Serial.print("Moving to position ");
  Serial.print(POSITION_2);
  Serial.println(" first");
  
  moveToPositionBoth(POSITION_2, POSITION_2, MAX_SPEED, MAX_SPEED, ACCEL_RATE, DECEL_RATE);
  delay(1500);
  
  // Now return to home
  Serial.println("Returning to home position (0)");
  bool success = moveToPositionBoth(
                   HOME_POSITION, HOME_POSITION, 
                   MAX_SPEED, MAX_SPEED, 
                   ACCEL_RATE, DECEL_RATE);
  displayResult(success, 0);
  
  Serial.println("\nAll demonstrations complete!");
}

/*
 * HELPER FUNCTIONS
 */

/**
 * Move a specified motor to an absolute position
 * 
 * @param motor Motor number (1 or 2)
 * @param position Target position in encoder counts
 * @param speed Maximum speed in encoder counts per second
 * @param accel Acceleration rate
 * @param decel Deceleration rate
 * @return True if command was sent successfully
 */
bool moveToPosition(int motor, int32_t position, uint32_t speed, uint32_t accel, uint32_t decel) {
  bool success = false;
  
  // Send the appropriate command based on which motor
  if (motor == 1) {
    success = roboclaw.SpeedAccelDecelPositionM1(
                RC_ADDRESS, accel, speed, decel, position, 1); // 1 = immediate execution
                
    if (success) {
      // Update expected position
      expectedPositionM1 = position;
    }
  } else if (motor == 2) {
    success = roboclaw.SpeedAccelDecelPositionM2(
                RC_ADDRESS, accel, speed, decel, position, 1); // 1 = immediate execution
                
    if (success) {
      // Update expected position
      expectedPositionM2 = position;
    }
  }
  
  if (!success) {
    Serial.println("Failed to send position command");
    return false;
  }
  
  // Wait for the movement to complete
  return waitForMotorIdle(motor, MOVE_TIMEOUT);
}

/**
 * Move both motors to absolute positions simultaneously
 * 
 * @param position1 Target position for Motor 1 in encoder counts
 * @param position2 Target position for Motor 2 in encoder counts
 * @param speed1 Maximum speed for Motor 1
 * @param speed2 Maximum speed for Motor 2
 * @param accel Acceleration rate for both motors
 * @param decel Deceleration rate for both motors
 * @return True if command was sent successfully
 */
bool moveToPositionBoth(int32_t position1, int32_t position2,
                         uint32_t speed1, uint32_t speed2,
                         uint32_t accel, uint32_t decel) {
  // Send the command for both motors
  bool success = roboclaw.SpeedAccelDecelPositionM1M2(
                   RC_ADDRESS, 
                   accel, speed1, decel, position1,
                   accel, speed2, decel, position2,
                   1); // 1 = immediate execution
  
  if (!success) {
    Serial.println("Failed to send position command for both motors");
    return false;
  }
  
  // Update expected positions
  expectedPositionM1 = position1;
  expectedPositionM2 = position2;
  
  // Wait for both movements to complete
  return waitForMotorsIdle(MOVE_TIMEOUT);
}

/**
 * Wait for a motor to complete its movement (become idle)
 * 
 * @param motor Motor number (1 or 2)
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return True if motor becomes idle, false if timed out
 */
bool waitForMotorIdle(int motor, unsigned long timeout_ms) {
  unsigned long startTime = millis();
  unsigned long lastUpdateTime = 0;
  
  while (millis() - startTime < timeout_ms) {
    // Read buffer status
    uint8_t depth1, depth2;
    bool valid = roboclaw.ReadBuffers(RC_ADDRESS, depth1, depth2);
    
    if (!valid) {
      Serial.println("Error reading buffer status");
      delay(100);
      continue;
    }
    
    // Check if the motor is idle (bit 7 set and buffer empty)
    bool motorIdle = false;
    
    if (motor == 1) {
      motorIdle = ((depth1 & 0x80) && (depth1 & 0x7F) == 0);
    } else {
      motorIdle = ((depth2 & 0x80) && (depth2 & 0x7F) == 0);
    }
    
    if (motorIdle) {
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.println(" movement complete");
      return true;
    }
    
    // Display expected position and status periodically
    if (millis() - lastUpdateTime >= DISPLAY_INTERVAL) {
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.print(" moving to position: ");
      Serial.print((motor == 1) ? expectedPositionM1 : expectedPositionM2);
      
      // Show buffer status
      Serial.print(" (Buffer: ");
      Serial.print((motor == 1) ? (depth1 & 0x7F) : (depth2 & 0x7F));
      Serial.print("/32, State: ");
      Serial.print((motor == 1) ? ((depth1 & 0x80) ? "Idle" : "Running") : 
                                ((depth2 & 0x80) ? "Idle" : "Running"));
      Serial.println(")");
      
      lastUpdateTime = millis();
    }
    
    // Small delay to prevent excessive serial communication
    delay(10);
  }
  
  // If we get here, we timed out
  Serial.print("Timeout waiting for Motor ");
  Serial.print(motor);
  Serial.println(" to complete movement");
  return false;
}

/**
 * Wait for both motors to complete their movements
 * 
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return True if both motors become idle, false if timed out
 */
bool waitForMotorsIdle(unsigned long timeout_ms) {
  unsigned long startTime = millis();
  unsigned long lastUpdateTime = 0;
  
  bool motor1Done = false;
  bool motor2Done = false;
  
  while (millis() - startTime < timeout_ms) {
    // Read buffer status
    uint8_t depth1, depth2;
    bool valid = roboclaw.ReadBuffers(RC_ADDRESS, depth1, depth2);
    
    if (!valid) {
      Serial.println("Error reading buffer status");
      delay(100);
      continue;
    }
    
    // Check if motors are idle
    if (!motor1Done) {
      motor1Done = ((depth1 & 0x80) && (depth1 & 0x7F) == 0);
      if (motor1Done) {
        Serial.println("Motor 1 movement complete");
      }
    }
    
    if (!motor2Done) {
      motor2Done = ((depth2 & 0x80) && (depth2 & 0x7F) == 0);
      if (motor2Done) {
        Serial.println("Motor 2 movement complete");
      }
    }
    
    // If both motors are done, return success
    if (motor1Done && motor2Done) {
      return true;
    }
    
    // Display expected positions and status periodically
    if (millis() - lastUpdateTime >= DISPLAY_INTERVAL) {
      Serial.print("Moving to: M1=");
      Serial.print(expectedPositionM1);
      Serial.print(motor1Done ? " (Done)" : " (Moving)");
      Serial.print(", M2=");
      Serial.print(expectedPositionM2);
      Serial.println(motor2Done ? " (Done)" : " (Moving)");
      
      // Show buffer status
      Serial.print("Buffers: M1=");
      Serial.print(depth1 & 0x7F);
      Serial.print("/32 ");
      Serial.print((depth1 & 0x80) ? "Idle" : "Running");
      Serial.print(", M2=");
      Serial.print(depth2 & 0x7F);
      Serial.print("/32 ");
      Serial.println((depth2 & 0x80) ? "Idle" : "Running");
      
      lastUpdateTime = millis();
    }
    
    // Small delay to prevent excessive serial communication
    delay(10);
  }
  
  // If we get here, we timed out
  Serial.println("Timeout waiting for motors to complete movement");
  return false;
}

/**
 * Display result of a movement operation
 * 
 * @param success Whether the movement succeeded
 * @param motor Motor number (0 for both motors)
 */
void displayResult(bool success, int motor) {
  if (success) {
    if (motor == 0) {
      Serial.println("Both motors successfully completed movement");
    } else {
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.println(" successfully completed movement");
    }
    
    // Display final expected positions
    Serial.print("Expected positions: M1=");
    Serial.print(expectedPositionM1);
    Serial.print(", M2=");
    Serial.println(expectedPositionM2);
  } else {
    if (motor == 0) {
      Serial.println("Failed to complete movement for both motors");
    } else {
      Serial.print("Motor ");
      Serial.print(motor);
      Serial.println(" failed to complete movement");
    }
  }
}

/**
 * Optionally verify actual vs. expected position
 * This can be called occasionally to ensure the expected positions
 * match the actual encoder readings
 */
void verifyPositions() {
  bool valid1, valid2;
  uint8_t status1, status2;
  
  int32_t actualM1 = roboclaw.ReadEncM1(RC_ADDRESS, &status1, &valid1);
  int32_t actualM2 = roboclaw.ReadEncM2(RC_ADDRESS, &status2, &valid2);
  
  if (valid1 && valid2) {
    Serial.println("Position verification:");
    Serial.print("Motor 1: Expected=");
    Serial.print(expectedPositionM1);
    Serial.print(", Actual=");
    Serial.print(actualM1);
    Serial.print(", Difference=");
    Serial.println(expectedPositionM1 - actualM1);
    
    Serial.print("Motor 2: Expected=");
    Serial.print(expectedPositionM2);
    Serial.print(", Actual=");
    Serial.print(actualM2);
    Serial.print(", Difference=");
    Serial.println(expectedPositionM2 - actualM2);
  } else {
    Serial.println("Failed to read actual encoder positions");
  }
}