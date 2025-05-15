/**
 * Roboclaw RC Mode Control Example
 * 
 * This sketch demonstrates how to control a Basicmicro Roboclaw motor controller
 * using standard RC (Radio Control) signals generated from an Arduino.
 * 
 * In RC mode, the Roboclaw accepts standard servo PWM signals (1000-2000 microseconds)
 * to control motor speed and direction, similar to how RC receivers control servos.
 * 
 * Hardware Setup:
 * - Arduino board
 * - Roboclaw motor controller configured for RC mode (refer to Roboclaw user manual)
 * - Connect Arduino pin 5 to Roboclaw S1 input (controls Motor 1)
 * - Connect Arduino pin 6 to Roboclaw S2 input (controls Motor 2)
 * - Connect the Arduino GND to Roboclaw signal ground
 * 
 * RC Signal Values:
 * - 1500 μs = Center/Stop position
 * - 1000-1499 μs = Forward speed (lower value = faster forward)
 * - 1501-2000 μs = Reverse speed (higher value = faster reverse)
 * 
 * Created by: Your Name
 * Date: March 20, 2025
 */

#include <Servo.h> 
 
// Create servo objects to generate RC PWM signals
// Note: We're not actually controlling servos, but using the Servo library
// to generate RC-compatible PWM signals for the Roboclaw
Servo rcSignalMotor1;  // Controls Roboclaw channel 1
Servo rcSignalMotor2;  // Controls Roboclaw channel 2

// RC signal constants (in microseconds)
const int RC_CENTER = 1500;  // Neutral/stop position
const int RC_FORWARD_MAX = 1000;  // Maximum forward speed
const int RC_REVERSE_MAX = 2000;  // Maximum reverse speed
const int RC_FORWARD_HALF = 1250;  // Half forward speed
const int RC_REVERSE_HALF = 1750;  // Half reverse speed
 
// Pin assignments
const int MOTOR1_PIN = 5;  // Arduino pin connected to Roboclaw S1
const int MOTOR2_PIN = 6;  // Arduino pin connected to Roboclaw S2

// Motion duration (milliseconds)
const int MOTION_DURATION = 2000;  // Time for each movement pattern

void setup() 
{
  // Initialize serial monitor for debugging
  Serial.begin(115200);
  Serial.println("Roboclaw RC Mode Control Example");
  Serial.println("-------------------------------");
  
  // Initialize the RC signal pins for both motors
  rcSignalMotor1.attach(MOTOR1_PIN);
  rcSignalMotor2.attach(MOTOR2_PIN);
  
  // Start with motors stopped
  stopAllMotors();
  
  Serial.println("RC signals initialized. Motors starting in 2 seconds...");
  delay(2000);
}

void loop() 
{
  // Pattern 1: Motor 1 forward, Motor 2 reverse at half speed
  Serial.println("Motor 1: Half Forward, Motor 2: Half Reverse");
  rcSignalMotor1.writeMicroseconds(RC_FORWARD_HALF);  // Half forward speed
  rcSignalMotor2.writeMicroseconds(RC_REVERSE_HALF);  // Half reverse speed
  delay(MOTION_DURATION);
  
  // Stop briefly between direction changes
  stopAllMotors();
  delay(500);
  
  // Pattern 2: Motor 1 reverse, Motor 2 forward at half speed
  Serial.println("Motor 1: Half Reverse, Motor 2: Half Forward");
  rcSignalMotor1.writeMicroseconds(RC_REVERSE_HALF);  // Half reverse speed
  rcSignalMotor2.writeMicroseconds(RC_FORWARD_HALF);  // Half forward speed
  delay(MOTION_DURATION);
  
  // Stop briefly between direction changes
  stopAllMotors();
  delay(500);
  
  // Pattern 3: Both motors forward at different speeds
  Serial.println("Both motors forward at different speeds");
  rcSignalMotor1.writeMicroseconds(RC_FORWARD_HALF);        // Half forward
  rcSignalMotor2.writeMicroseconds(RC_FORWARD_HALF + 125);  // Quarter forward
  delay(MOTION_DURATION);
  
  // Stop briefly between patterns
  stopAllMotors();
  delay(500);
}

/**
 * Stop all motors by sending the center/neutral signal (1500μs)
 */
void stopAllMotors() {
  Serial.println("Stopping all motors");
  rcSignalMotor1.writeMicroseconds(RC_CENTER);
  rcSignalMotor2.writeMicroseconds(RC_CENTER);
}

/**
 * Set specific speed for a motor
 * 
 * @param motor The servo object controlling the motor
 * @param speed Speed value from -100 to 100 (negative = reverse, positive = forward)
 */
void setMotorSpeed(Servo &motor, int speed) {
  // Constrain speed value to valid range
  speed = constrain(speed, -100, 100);
  
  // Convert speed percentage (-100 to 100) to microseconds (1000-2000)
  int pulseWidth;
  
  if (speed == 0) {
    // Stopped
    pulseWidth = RC_CENTER;
  } 
  else if (speed > 0) {
    // Forward - map 0-100 to 1500-1000 (note inverted range for RC signals)
    pulseWidth = map(speed, 0, 100, RC_CENTER, RC_FORWARD_MAX);
  } 
  else {
    // Reverse - map -100-0 to 1500-2000
    pulseWidth = map(speed, -100, 0, RC_REVERSE_MAX, RC_CENTER);
  }
  
  motor.writeMicroseconds(pulseWidth);
}
