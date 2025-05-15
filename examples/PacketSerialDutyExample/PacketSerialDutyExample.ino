/**
 * Roboclaw PacketSerial Control Example
 * 
 * This sketch demonstrates advanced motor control using Basicmicro Roboclaw's
 * PacketSerial mode. It shows how to:
 * 
 * 1. Control individual motors directly
 * 2. Control both motors simultaneously for tank-style movement
 * 3. Implement mixed throttle/steering control for differential drive
 * 4. Use acceleration control for smoother movement
 * 
 * Roboclaw Configuration:
 * 1. Connect Roboclaw to computer via USB
 * 2. Launch Basicmicro Motion Studio software
 * 3. Set 'Serial Mode' to 'Packet Serial'
 * 4. Set baud rate to 38400 (or match the value in this sketch)
 * 5. Note the device address (default is 0x80)
 * 6. Click 'Write Settings' to save configuration
 * 
 * Hardware Setup:
 * - Arduino board (Uno, Mega, etc.)
 * - Roboclaw motor controller configured for Packet Serial mode
 * - Connect Arduino TX pin to Roboclaw S1 (RX)
 * - Connect Arduino RX pin to Roboclaw S2 (TX)
 * - Connect Arduino GND to Roboclaw GND
 * 
 * Note: This example uses Hardware Serial for reliable communication.
 * For Arduino Uno/Nano, consider using AltSoftSerial or a board with
 * multiple hardware serial ports like Arduino Mega.
 * 
 * Created by: Your Name
 * Date: March 20, 2025
 */

#include <RoboClaw.h>

// Hardware Serial for Roboclaw (Serial1)
// For Arduino Mega/Due - use Serial1, Serial2, or Serial3
// For boards with one hardware serial, use AltSoftSerial or SoftwareSerial
#define ROBOCLAW_SERIAL Serial1

// Roboclaw address (default is 0x80)
#define RC_ADDRESS 0x80

// Create RoboClaw object
RoboClaw roboclaw(&ROBOCLAW_SERIAL, 10000); // 10ms timeout

// Motor control constants
#define MOTOR_FORWARD 1
#define MOTOR_REVERSE -1
#define MOTOR_STOP 0

// Duty cycle values (±32767 represents ±100% duty cycle)
#define DUTY_MAX 32767      // 100% duty cycle
#define DUTY_MIN -32767     // -100% duty cycle
#define DUTY_HALF 16384     // ~50% duty cycle
#define DUTY_QUARTER 8192   // ~25% duty cycle

// Acceleration values (speed change per second, 0-655360)
// Note: A value of 0 means maximum acceleration (655360)
#define ACCEL_SLOW 10000    // Gentle acceleration
#define ACCEL_MEDIUM 100000 // Medium acceleration
#define ACCEL_FAST 500000   // Rapid acceleration
#define ACCEL_MAX 0         // Maximum acceleration (uses 655360)

// Timing constants
#define MOVE_DURATION 2000  // Duration for each movement demo (ms)
#define PAUSE_DURATION 1000 // Pause between demonstrations (ms)

void setup() {
  // Initialize hardware serial for debugging
  Serial.begin(115200);
  Serial.println("Roboclaw PacketSerial Control Example");
  Serial.println("-------------------------------------");
  
  // Initialize serial communication with Roboclaw
  ROBOCLAW_SERIAL.begin(38400);
  roboclaw.begin();
  
  // Safety first - make sure motors are stopped
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  
  Serial.println("Roboclaw initialized. Starting demo in 3 seconds...");
  delay(3000);
}

void loop() {
  // ----------------------------------------
  // Part 1: Individual Motor Control
  // ----------------------------------------
  Serial.println("\n=== Individual Motor Control ===");
  
  // Motor 1 Forward
  Serial.println("Motor 1 Forward (50% duty)");
  roboclaw.DutyM1(RC_ADDRESS, DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop Motor 1
  roboclaw.DutyM1(RC_ADDRESS, 0);
  delay(PAUSE_DURATION);
  
  // Motor 1 Reverse
  Serial.println("Motor 1 Reverse (50% duty)");
  roboclaw.DutyM1(RC_ADDRESS, -DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop Motor 1
  roboclaw.DutyM1(RC_ADDRESS, 0);
  delay(PAUSE_DURATION);
  
  // Motor 2 Forward
  Serial.println("Motor 2 Forward (50% duty)");
  roboclaw.DutyM2(RC_ADDRESS, DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop Motor 2
  roboclaw.DutyM2(RC_ADDRESS, 0);
  delay(PAUSE_DURATION);
  
  // Motor 2 Reverse
  Serial.println("Motor 2 Reverse (50% duty)");
  roboclaw.DutyM2(RC_ADDRESS, -DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop Motor 2
  roboclaw.DutyM2(RC_ADDRESS, 0);
  delay(PAUSE_DURATION);
  
  // ----------------------------------------
  // Part 2: Tank Style Control (Both Motors)
  // ----------------------------------------
  Serial.println("\n=== Tank Style Control ===");
  
  // Forward (both motors forward)
  Serial.println("Forward (both motors)");
  roboclaw.DutyM1M2(RC_ADDRESS, DUTY_HALF, DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // Backward (both motors reverse)
  Serial.println("Backward (both motors)");
  roboclaw.DutyM1M2(RC_ADDRESS, -DUTY_HALF, -DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // Spin left (Motor 1 reverse, Motor 2 forward)
  Serial.println("Spin Left");
  roboclaw.DutyM1M2(RC_ADDRESS, -DUTY_HALF, DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // Spin right (Motor 1 forward, Motor 2 reverse)
  Serial.println("Spin Right");
  roboclaw.DutyM1M2(RC_ADDRESS, DUTY_HALF, -DUTY_HALF);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // ----------------------------------------
  // Part 3: Mixed Throttle/Steering Control
  // ----------------------------------------
  Serial.println("\n=== Mixed Throttle/Steering Control ===");
  
  // Forward with slight right turn
  Serial.println("Forward with slight right turn");
  mixedControl(DUTY_HALF, -DUTY_QUARTER);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // Forward with slight left turn
  Serial.println("Forward with slight left turn");
  mixedControl(DUTY_HALF, DUTY_QUARTER);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // Backward with slight right turn
  Serial.println("Backward with slight right turn");
  mixedControl(-DUTY_HALF, -DUTY_QUARTER);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // Backward with slight left turn
  Serial.println("Backward with slight left turn");
  mixedControl(-DUTY_HALF, DUTY_QUARTER);
  delay(MOVE_DURATION);
  
  // Stop both motors
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // ----------------------------------------
  // Part 4: Acceleration-Controlled Movement
  // ----------------------------------------
  Serial.println("\n=== Acceleration-Controlled Movement ===");
  
  // Slow ramp up to full speed
  Serial.println("Forward with slow acceleration");
  roboclaw.DutyAccelM1M2(RC_ADDRESS, ACCEL_SLOW, DUTY_MAX, DUTY_MAX);
  delay(MOVE_DURATION * 2);
  
  // Quick stop
  roboclaw.DutyM1M2(RC_ADDRESS, 0, 0);
  delay(PAUSE_DURATION);
  
  // Medium acceleration reverse
  Serial.println("Reverse with medium acceleration");
  roboclaw.DutyAccelM1M2(RC_ADDRESS, ACCEL_MEDIUM, -DUTY_MAX, -DUTY_MAX);
  delay(MOVE_DURATION);
  
  // Decelerate to stop using acceleration control
  Serial.println("Deceleration to stop");
  roboclaw.DutyAccelM1M2(RC_ADDRESS, ACCEL_SLOW, 0, 0);
  delay(MOVE_DURATION);
  
  // Individual motor acceleration control
  Serial.println("Individual motor acceleration control");
  roboclaw.DutyAccelM1(RC_ADDRESS, ACCEL_MEDIUM, DUTY_MAX);
  delay(500);
  roboclaw.DutyAccelM2(RC_ADDRESS, ACCEL_MEDIUM, DUTY_MAX);
  delay(MOVE_DURATION);
  
  // Decelerate both motors to stop
  roboclaw.DutyAccelM1M2(RC_ADDRESS, ACCEL_MEDIUM, 0, 0);
  delay(PAUSE_DURATION);
  
  Serial.println("\nDemo sequence complete. Restarting in 3 seconds...");
  delay(3000);
}

/**
 * Implement mixed throttle/steering control for differential drive
 * 
 * @param throttle Forward/backward speed (-32767 to 32767)
 * @param steering Left/right steering value (-32767 to 32767, positive=left, negative=right)
 */
void mixedControl(int throttle, int steering) {
  // Constrain values to valid range
  throttle = constrain(throttle, -32767, 32767);
  steering = constrain(steering, -32767, 32767);
  
  // Calculate left and right motor values
  int leftMotor, rightMotor;
  
  if (steering == 0) {
    // Pure forward/backward motion
    leftMotor = throttle;
    rightMotor = throttle;
  } 
  else if (throttle == 0) {
    // Pure rotation
    leftMotor = steering;
    rightMotor = -steering;
  }
  else {
    // Mixed throttle and steering
    if (steering > 0) {
      // Turning left
      leftMotor = throttle - steering;
      rightMotor = throttle;
    } else {
      // Turning right
      leftMotor = throttle;
      rightMotor = throttle + steering; // Note: steering is negative here
    }
  }
  
  // Constrain final values
  leftMotor = constrain(leftMotor, -32767, 32767);
  rightMotor = constrain(rightMotor, -32767, 32767);
  
  // Send commands to Roboclaw
  roboclaw.DutyM1M2(RC_ADDRESS, leftMotor, rightMotor);
  
  // Debug output
  Serial.print("Mixed Control: Throttle=");
  Serial.print(throttle);
  Serial.print(", Steering=");
  Serial.print(steering);
  Serial.print(" → Left Motor=");
  Serial.print(leftMotor);
  Serial.print(", Right Motor=");
  Serial.println(rightMotor);
}

/**
 * Implement mixed throttle/steering with acceleration control
 * 
 * @param accel Acceleration value (0-655360, 0 means maximum acceleration)
 * @param throttle Forward/backward speed (-32767 to 32767)
 * @param steering Left/right steering value (-32767 to 32767, positive=left, negative=right)
 */
void mixedControlWithAccel(uint32_t accel, int throttle, int steering) {
  // Constrain values to valid range
  throttle = constrain(throttle, -32767, 32767);
  steering = constrain(steering, -32767, 32767);
  
  // Calculate left and right motor values
  int leftMotor, rightMotor;
  
  if (steering == 0) {
    // Pure forward/backward motion
    leftMotor = throttle;
    rightMotor = throttle;
  } 
  else if (throttle == 0) {
    // Pure rotation
    leftMotor = steering;
    rightMotor = -steering;
  }
  else {
    // Mixed throttle and steering
    if (steering > 0) {
      // Turning left
      leftMotor = throttle - steering;
      rightMotor = throttle;
    } else {
      // Turning right
      leftMotor = throttle;
      rightMotor = throttle + steering; // Note: steering is negative here
    }
  }
  
  // Constrain final values
  leftMotor = constrain(leftMotor, -32767, 32767);
  rightMotor = constrain(rightMotor, -32767, 32767);
  
  // Send commands to Roboclaw with acceleration control
  roboclaw.DutyAccelM1M2(RC_ADDRESS, accel, leftMotor, rightMotor);
  
  // Debug output
  Serial.print("Mixed Control with Accel: ");
  Serial.print(accel);
  Serial.print(", Throttle=");
  Serial.print(throttle);
  Serial.print(", Steering=");
  Serial.print(steering);
  Serial.print(" → Left Motor=");
  Serial.print(leftMotor);
  Serial.print(", Right Motor=");
  Serial.println(rightMotor);
}