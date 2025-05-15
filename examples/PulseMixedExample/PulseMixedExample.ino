/**
 * Roboclaw RC Mode with Mixing - Differential Drive Control Example
 * 
 * This sketch demonstrates how to control a Basicmicro Roboclaw motor controller
 * in RC mode with mixing enabled for differential drive robots (tank-style steering).
 * 
 * When RC mixing is enabled on the Roboclaw:
 * - Channel 1 (S1) controls throttle (forward/backward movement)
 * - Channel 2 (S2) controls steering (left/right turning)
 * 
 * The Roboclaw internally mixes these signals to properly drive the left and right
 * motors for differential steering, making it ideal for rovers, tanks, and other
 * skid-steer vehicles.
 * 
 * Hardware Setup:
 * - Arduino board
 * - Roboclaw motor controller configured for RC mode with MIXING ENABLED
 *   (refer to Roboclaw user manual for DIP switch settings)
 * - Connect Arduino pin 5 to Roboclaw S1 input (throttle)
 * - Connect Arduino pin 6 to Roboclaw S2 input (steering)
 * - Connect the Arduino GND to Roboclaw signal ground
 * 
 * RC Signal Values:
 * - 1500 μs = Center/Neutral position (no movement)
 * - Throttle (S1):
 *   - 1501-2000 μs = Forward (higher value = faster forward)
 *   - 1000-1499 μs = Reverse (lower value = faster reverse)
 * - Steering (S2):
 *   - 1501-2000 μs = Turn Left (higher value = sharper left turn)
 *   - 1000-1499 μs = Turn Right (lower value = sharper right turn)
 * 
 * Created by: Your Name
 * Date: March 20, 2025
 */

#include <Servo.h> 

// Create servo objects to generate RC PWM signals
Servo throttleSignal;  // Controls forward/backward movement (S1 pin)
Servo steeringSignal;  // Controls left/right turning (S2 pin)

// RC signal constants (in microseconds)
const int RC_CENTER = 1500;  // Neutral/center position
const int RC_FULL_MAX = 2000;  // Maximum forward/left
const int RC_FULL_MIN = 1000;  // Maximum reverse/right

// Moderate movement settings
const int RC_MODERATE_FORWARD = 1600;  // Moderate forward speed
const int RC_MODERATE_REVERSE = 1400;  // Moderate reverse speed
const int RC_MODERATE_LEFT = 1600;     // Moderate left turn
const int RC_MODERATE_RIGHT = 1400;    // Moderate right turn

// Pin assignments
const int THROTTLE_PIN = 5;  // Arduino pin connected to Roboclaw S1 (throttle)
const int STEERING_PIN = 6;  // Arduino pin connected to Roboclaw S2 (steering)

// Motion duration (milliseconds)
const int MOTION_DURATION = 2000;  // Time for each movement pattern
const int PAUSE_DURATION = 500;    // Pause between movements

void setup() 
{ 
  // Initialize serial monitor for debugging
  Serial.begin(115200);
  Serial.println("Roboclaw RC Mode with Mixing - Differential Drive Control");
  Serial.println("-------------------------------------------------------");
  
  // Initialize the RC signal pins
  throttleSignal.attach(THROTTLE_PIN);  // S1 - Throttle control
  steeringSignal.attach(STEERING_PIN);  // S2 - Steering control
  
  // Start with robot stopped (neutral signals)
  stopRobot();
  
  Serial.println("RC signals initialized. Robot starting in 3 seconds...");
  delay(3000);
}

void loop() 
{
  // Demonstrate all basic movement patterns
  
  // 1. Forward movement (straight)
  Serial.println("Moving Forward");
  moveRobot(RC_MODERATE_FORWARD, RC_CENTER);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // 2. Backward movement (straight)
  Serial.println("Moving Backward");
  moveRobot(RC_MODERATE_REVERSE, RC_CENTER);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // 3. Turn left in place (spin left)
  Serial.println("Turning Left (in place)");
  moveRobot(RC_CENTER, RC_MODERATE_LEFT);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // 4. Turn right in place (spin right)
  Serial.println("Turning Right (in place)");
  moveRobot(RC_CENTER, RC_MODERATE_RIGHT);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // 5. Forward while turning left (arc left)
  Serial.println("Forward + Left Turn (arc left)");
  moveRobot(RC_MODERATE_FORWARD, RC_MODERATE_LEFT);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // 6. Forward while turning right (arc right)
  Serial.println("Forward + Right Turn (arc right)");
  moveRobot(RC_MODERATE_FORWARD, RC_MODERATE_RIGHT);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // 7. Backward while turning left (reverse arc left)
  Serial.println("Backward + Left Turn (reverse arc left)");
  moveRobot(RC_MODERATE_REVERSE, RC_MODERATE_LEFT);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // 8. Backward while turning right (reverse arc right)
  Serial.println("Backward + Right Turn (reverse arc right)");
  moveRobot(RC_MODERATE_REVERSE, RC_MODERATE_RIGHT);
  delay(MOTION_DURATION);
  stopRobot();
  delay(PAUSE_DURATION);
  
  // Pause before repeating the sequence
  Serial.println("Movement sequence complete. Repeating...");
  delay(1000);
}

/**
 * Move the robot with specific throttle and steering values
 * 
 * @param throttleValue RC pulse width for throttle (1000-2000 μs)
 * @param steeringValue RC pulse width for steering (1000-2000 μs)
 */
void moveRobot(int throttleValue, int steeringValue) {
  // Ensure values are within valid RC signal range
  throttleValue = constrain(throttleValue, RC_FULL_MIN, RC_FULL_MAX);
  steeringValue = constrain(steeringValue, RC_FULL_MIN, RC_FULL_MAX);
  
  // Send RC signals to the Roboclaw
  throttleSignal.writeMicroseconds(throttleValue);
  steeringSignal.writeMicroseconds(steeringValue);
  
  // Log the movement values
  Serial.print("Throttle: ");
  Serial.print(throttleValue);
  Serial.print(" | Steering: ");
  Serial.println(steeringValue);
}

/**
 * Stop the robot by sending neutral signals to both channels
 */
void stopRobot() {
  Serial.println("Stopping");
  throttleSignal.writeMicroseconds(RC_CENTER);
  steeringSignal.writeMicroseconds(RC_CENTER);
  // Small delay to ensure the stop command is processed
  delay(100);
}

/**
 * Set throttle and steering by percentage values
 * This function provides a more intuitive interface by using percentages
 * 
 * @param throttlePercent Throttle percentage (-100 to 100, negative=reverse)
 * @param steeringPercent Steering percentage (-100 to 100, negative=right)
 */
void moveRobotPercent(int throttlePercent, int steeringPercent) {
  // Constrain percentages to valid range
  throttlePercent = constrain(throttlePercent, -100, 100);
  steeringPercent = constrain(steeringPercent, -100, 100);
  
  // Convert percentages to microseconds
  int throttleValue, steeringValue;
  
  // Convert throttle percentage to microseconds
  if (throttlePercent >= 0) {
    // Forward: 0 to 100% maps to 1500 to 2000 μs
    throttleValue = map(throttlePercent, 0, 100, RC_CENTER, RC_FULL_MAX);
  } else {
    // Reverse: -100 to 0% maps to 1000 to 1500 μs
    throttleValue = map(throttlePercent, -100, 0, RC_FULL_MIN, RC_CENTER);
  }
  
  // Convert steering percentage to microseconds
  if (steeringPercent >= 0) {
    // Left: 0 to 100% maps to 1500 to 2000 μs
    steeringValue = map(steeringPercent, 0, 100, RC_CENTER, RC_FULL_MAX);
  } else {
    // Right: -100 to 0% maps to 1000 to 1500 μs
    steeringValue = map(steeringPercent, -100, 0, RC_FULL_MIN, RC_CENTER);
  }
  
  // Move the robot with the calculated values
  moveRobot(throttleValue, steeringValue);
}