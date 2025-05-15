/**
 * Roboclaw Simple Serial Control Example
 * 
 * This sketch demonstrates how to control a Basicmicro Roboclaw motor controller
 * using Simple Serial mode, which provides basic motor control with minimal overhead.
 * 
 * In Simple Serial mode, single byte commands control motor speed and direction:
 * - Value 0: Emergency stop for both motors
 * - Values 1-127: Control Motor 1
 *   - 1-63: Reverse (1 = full reverse, 63 = slowest reverse)
 *   - 64: Stop Motor 1
 *   - 65-127: Forward (65 = slowest forward, 127 = full forward)
 * - Values 128-255: Control Motor 2
 *   - 128-191: Reverse (128 = full reverse, 191 = slowest reverse) 
 *   - 192: Stop Motor 2
 *   - 193-255: Forward (193 = slowest forward, 255 = full forward)
 * 
 * Roboclaw Configuration:
 * 1. Connect Roboclaw to computer via USB
 * 2. Launch Basicmicro Motion Studio software
 * 3. Select the 'General Settings' tab
 * 4. Set 'Serial Mode' to 'Simple Serial'
 * 5. Set baud rate to 38400 (default for Simple Serial)
 * 6. Click 'Write Settings' to save configuration to Roboclaw
 * 
 * Hardware Setup:
 * - Arduino board
 * - Roboclaw motor controller configured via USB for Simple Serial mode
 * - Connect Arduino pin 10 (RX) to Roboclaw S2 (TX)
 * - Connect Arduino pin 11 (TX) to Roboclaw S1 (RX) 
 * - Connect Arduino GND to Roboclaw GND
 * 
 * Note: SoftwareSerial has limitations in Arduino. For more reliable control,
 * consider using a hardware serial port if available on your board.
 * 
 * Created by: Your Name
 * Date: March 20, 2025
 */

#include <SoftwareSerial.h>

// Create a software serial port for communication with the Roboclaw
// Pin 10 (RX from Roboclaw), Pin 11 (TX to Roboclaw)
SoftwareSerial roboclawSerial(10, 11);

// Motor control constants
const byte BOTH_MOTORS_STOP = 0;

// Motor 1 control values
const byte MOTOR1_FULL_REVERSE = 1;
const byte MOTOR1_STOP = 64;
const byte MOTOR1_FULL_FORWARD = 127;
const byte MOTOR1_HALF_REVERSE = 32;  // ~50% reverse
const byte MOTOR1_HALF_FORWARD = 96;  // ~50% forward

// Motor 2 control values
const byte MOTOR2_FULL_REVERSE = 128;
const byte MOTOR2_STOP = 192;
const byte MOTOR2_FULL_FORWARD = 255;
const byte MOTOR2_HALF_REVERSE = 160;  // ~50% reverse
const byte MOTOR2_HALF_FORWARD = 224;  // ~50% forward

// Timing constants
const unsigned int MOVE_DURATION = 2000;  // Duration of movement commands
const unsigned int STOP_DURATION = 1000;  // Duration of stop commands

void setup() {
  // Initialize the hardware serial for debugging
  Serial.begin(115200);
  Serial.println("Roboclaw Simple Serial Control Example");
  Serial.println("------------------------------------");
  
  // Initialize the software serial port to communicate with Roboclaw
  roboclawSerial.begin(38400);  // Default baud rate for Simple Serial mode
  
  // Safety delay to ensure everything is initialized
  delay(1000);
  
  Serial.println("Starting motor sequence in 3 seconds...");
  delay(3000);
}

void loop() {
  // Demonstrate various motor control patterns
  
  // 1. Motor 1 full forward, Motor 2 full reverse
  Serial.println("Motor 1: Full Forward, Motor 2: Full Reverse");
  sendMotorCommand(MOTOR1_FULL_FORWARD, MOTOR2_FULL_REVERSE);
  delay(MOVE_DURATION);
  
  // Stop both motors briefly
  stopBothMotors();
  delay(STOP_DURATION);
  
  // 2. Motor 1 half forward
  Serial.println("Motor 1: Half Forward, Motor 2: Stopped");
  sendMotorCommand(MOTOR1_HALF_FORWARD, MOTOR2_STOP);
  delay(MOVE_DURATION);
  
  // Stop both motors briefly
  stopBothMotors();
  delay(STOP_DURATION);
  
  // 3. Motor 1 full reverse, Motor 2 full forward
  Serial.println("Motor 1: Full Reverse, Motor 2: Full Forward");
  sendMotorCommand(MOTOR1_FULL_REVERSE, MOTOR2_FULL_FORWARD);
  delay(MOVE_DURATION);
  
  // Stop both motors briefly
  stopBothMotors();
  delay(STOP_DURATION);
  
  // 4. Motor 2 half reverse
  Serial.println("Motor 1: Stopped, Motor 2: Half Reverse");
  sendMotorCommand(MOTOR1_STOP, MOTOR2_HALF_REVERSE);
  delay(MOVE_DURATION);
  
  // Stop both motors briefly
  stopBothMotors();
  delay(STOP_DURATION);
  
  // 5. Both motors stopped using individual stop commands
  Serial.println("Both motors stopped (individual commands)");
  sendMotorCommand(MOTOR1_STOP, MOTOR2_STOP);
  delay(MOVE_DURATION);
  
  // 6. Both motors at full speed in opposite directions
  Serial.println("Both motors at full speed in opposite directions");
  sendMotorCommand(MOTOR1_FULL_FORWARD, MOTOR2_FULL_REVERSE);
  delay(MOVE_DURATION);
  
  // Emergency stop using single command
  Serial.println("Emergency stop (single command)");
  emergencyStop();
  delay(STOP_DURATION * 2);
  
  Serial.println("Movement sequence complete. Repeating...\n");
}

/**
 * Send commands to control both motors
 * 
 * @param motor1Value Control value for Motor 1 (1-127)
 * @param motor2Value Control value for Motor 2 (128-255)
 */
void sendMotorCommand(byte motor1Value, byte motor2Value) {
  // Optionally log the values being sent
  Serial.print("Sending: Motor1=");
  Serial.print(motor1Value);
  Serial.print(", Motor2=");
  Serial.println(motor2Value);
  
  // In Simple Serial mode, we send individual bytes for each motor
  // We can send them back-to-back
  roboclawSerial.write(motor1Value);
  roboclawSerial.write(motor2Value);
}

/**
 * Emergency stop both motors using the special value 0
 */
void emergencyStop() {
  Serial.println("EMERGENCY STOP");
  roboclawSerial.write(BOTH_MOTORS_STOP);
}

/**
 * Stop both motors by sending the stop value to each
 */
void stopBothMotors() {
  Serial.println("Stopping both motors");
  sendMotorCommand(MOTOR1_STOP, MOTOR2_STOP);
}

/**
 * Helper function to set Motor 1 to a specific speed
 * 
 * @param speed Speed from -100 to 100 (negative=reverse, positive=forward)
 */
void setMotor1Speed(int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -100, 100);
  
  byte value;
  if (speed == 0) {
    value = MOTOR1_STOP;
  } else if (speed > 0) {
    // Map 1-100 to 65-127 (forward)
    value = map(speed, 1, 100, MOTOR1_STOP + 1, MOTOR1_FULL_FORWARD);
  } else {
    // Map -100 to -1 to 1-63 (reverse)
    value = map(speed, -100, -1, MOTOR1_FULL_REVERSE, MOTOR1_STOP - 1);
  }
  
  roboclawSerial.write(value);
}

/**
 * Helper function to set Motor 2 to a specific speed
 * 
 * @param speed Speed from -100 to 100 (negative=reverse, positive=forward)
 */
void setMotor2Speed(int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -100, 100);
  
  byte value;
  if (speed == 0) {
    value = MOTOR2_STOP;
  } else if (speed > 0) {
    // Map 1-100 to 193-255 (forward)
    value = map(speed, 1, 100, MOTOR2_STOP + 1, MOTOR2_FULL_FORWARD);
  } else {
    // Map -100 to -1 to 128-191 (reverse)
    value = map(speed, -100, -1, MOTOR2_FULL_REVERSE, MOTOR2_STOP - 1);
  }
  
  roboclawSerial.write(value);
}