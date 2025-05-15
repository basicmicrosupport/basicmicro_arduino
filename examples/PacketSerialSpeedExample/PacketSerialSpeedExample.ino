/**
 * Roboclaw PacketSerial Speed Control Example
 * 
 * This sketch demonstrates how to control a Basicmicro Roboclaw motor controller
 * using speed commands in PacketSerial mode. It covers all speed-related commands
 * EXCEPT those with distance arguments (which are covered in a separate example).
 * 
 * Commands demonstrated:
 * - Speed control with/without acceleration
 * - Individual motor control
 * - Synchronized dual motor control
 * - PID settings adjustment
 * - Reading current speed values
 * 
 * Roboclaw Configuration:
 * 1. Connect Roboclaw to computer via USB
 * 2. Launch Basicmicro Motion Studio software
 * 3. Set 'Serial Mode' to 'Packet Serial'
 * 4. Configure PID parameters appropriately for your motors
 * 5. Set baud rate to 38400 (or match the value in this sketch)
 * 6. Note the device address (default is 0x80)
 * 7. Click 'Write Settings' to save configuration
 * 
 * Hardware Setup:
 * - Arduino board with hardware serial (e.g., Arduino Mega, Due, etc.)
 * - Roboclaw motor controller 
 * - Two quadrature encoder motors
 * - Connect Arduino TX pin to Roboclaw S1 (RX)
 * - Connect Arduino RX pin to Roboclaw S2 (TX)
 * - Connect Arduino GND to Roboclaw GND
 * 
 * Created by: Your Name
 * Date: March 20, 2025
 */

#include <RoboClaw.h>

// Hardware Serial for Roboclaw (Serial1)
// For Arduino Mega/Due - use Serial1, Serial2, or Serial3
// For boards with one hardware serial, consider a hardware serial alternative
#define ROBOCLAW_SERIAL Serial1

// Roboclaw address (default is 0x80)
#define RC_ADDRESS 0x80

// Create RoboClaw object
RoboClaw roboclaw(&ROBOCLAW_SERIAL, 10000); // 10ms timeout

// Speed constants - in encoder counts per second
#define SPEED_ZERO 0
#define SPEED_SLOW 1000
#define SPEED_MEDIUM 3000
#define SPEED_FAST 5000
#define SPEED_MAX 7000

// Acceleration constants - in encoder counts per second^2
#define ACCEL_SLOW 2000
#define ACCEL_MEDIUM 5000 
#define ACCEL_FAST 10000

// Timing constants
#define MOVE_DURATION 3000  // Duration for each movement demonstration (ms)
#define PAUSE_DURATION 1000 // Pause between demonstrations (ms)
#define READ_INTERVAL 500   // How often to read and display speed values (ms)

// Status tracking
unsigned long lastSpeedReadTime = 0;

void setup() {
  // Initialize hardware serial for debugging
  Serial.begin(115200);
  Serial.println("Roboclaw PacketSerial Speed Control Example");
  Serial.println("------------------------------------------");
  
  // Initialize serial communication with Roboclaw
  ROBOCLAW_SERIAL.begin(38400);
  roboclaw.begin();
  
  // Safety first - make sure motors are stopped
  roboclaw.SpeedM1M2(RC_ADDRESS, 0, 0);
  
  Serial.println("Roboclaw initialized. Starting demo in 3 seconds...");
  delay(3000);
}

void loop() {
  // Part 1: Basic Speed Control (no acceleration)
  // ----------------------------------------------
  Serial.println("\n=== Basic Speed Control (No Acceleration) ===");
  
  // Motor 1 Forward
  Serial.println("Motor 1 Forward (Medium Speed)");
  roboclaw.SpeedM1(RC_ADDRESS, SPEED_MEDIUM);
  runForDuration(MOVE_DURATION);
  
  // Stop Motor 1
  roboclaw.SpeedM1(RC_ADDRESS, SPEED_ZERO);
  delay(PAUSE_DURATION);
  
  // Motor 1 Reverse
  Serial.println("Motor 1 Reverse (Medium Speed)");
  roboclaw.SpeedM1(RC_ADDRESS, -SPEED_MEDIUM);
  runForDuration(MOVE_DURATION);
  
  // Stop Motor 1
  roboclaw.SpeedM1(RC_ADDRESS, SPEED_ZERO);
  delay(PAUSE_DURATION);
  
  // Motor 2 Forward
  Serial.println("Motor 2 Forward (Medium Speed)");
  roboclaw.SpeedM2(RC_ADDRESS, SPEED_MEDIUM);
  runForDuration(MOVE_DURATION);
  
  // Stop Motor 2
  roboclaw.SpeedM2(RC_ADDRESS, SPEED_ZERO);
  delay(PAUSE_DURATION);
  
  // Motor 2 Reverse
  Serial.println("Motor 2 Reverse (Medium Speed)");
  roboclaw.SpeedM2(RC_ADDRESS, -SPEED_MEDIUM);
  runForDuration(MOVE_DURATION);
  
  // Stop Motor 2
  roboclaw.SpeedM2(RC_ADDRESS, SPEED_ZERO);
  delay(PAUSE_DURATION);
  
  // Both Motors - Same direction
  Serial.println("Both Motors Forward (Different Speeds)");
  roboclaw.SpeedM1M2(RC_ADDRESS, SPEED_FAST, SPEED_MEDIUM);
  runForDuration(MOVE_DURATION);
  
  // Stop Both Motors
  roboclaw.SpeedM1M2(RC_ADDRESS, SPEED_ZERO, SPEED_ZERO);
  delay(PAUSE_DURATION);
  
  // Both Motors - Opposite directions
  Serial.println("Motors in Opposite Directions (Rotate in place)");
  roboclaw.SpeedM1M2(RC_ADDRESS, SPEED_MEDIUM, -SPEED_MEDIUM);
  runForDuration(MOVE_DURATION);
  
  // Stop Both Motors
  roboclaw.SpeedM1M2(RC_ADDRESS, SPEED_ZERO, SPEED_ZERO);
  delay(PAUSE_DURATION);
  
  // Part 2: Acceleration-Controlled Speed
  // -------------------------------------
  Serial.println("\n=== Speed Control with Acceleration ===");
  
  // Slow acceleration to fast speed (Motor 1)
  Serial.println("Motor 1: Slow Acceleration to Fast Speed");
  roboclaw.SpeedAccelM1(RC_ADDRESS, ACCEL_SLOW, SPEED_FAST);
  runForDuration(MOVE_DURATION + 1000); // Extra time to reach speed
  
  // Rapid deceleration to stop
  Serial.println("Motor 1: Rapid Deceleration to Stop");
  roboclaw.SpeedAccelM1(RC_ADDRESS, ACCEL_FAST, SPEED_ZERO);
  delay(PAUSE_DURATION + 1000);
  
  // Medium acceleration to medium reverse speed (Motor 2)
  Serial.println("Motor 2: Medium Acceleration to Medium Reverse Speed");
  roboclaw.SpeedAccelM2(RC_ADDRESS, ACCEL_MEDIUM, -SPEED_MEDIUM);
  runForDuration(MOVE_DURATION);
  
  // Medium deceleration to stop
  Serial.println("Motor 2: Medium Deceleration to Stop");
  roboclaw.SpeedAccelM2(RC_ADDRESS, ACCEL_MEDIUM, SPEED_ZERO);
  delay(PAUSE_DURATION + 1000);
  
  // Controlled acceleration for both motors simultaneously
  Serial.println("Both Motors: Controlled Acceleration to Different Speeds");
  roboclaw.SpeedAccelM1M2(RC_ADDRESS, ACCEL_MEDIUM, ACCEL_SLOW, SPEED_FAST, SPEED_MEDIUM);
  runForDuration(MOVE_DURATION + 1000);
  
  // Decelerate both motors to stop
  Serial.println("Both Motors: Decelerate to Stop");
  roboclaw.SpeedAccelM1M2(RC_ADDRESS, ACCEL_MEDIUM, ACCEL_MEDIUM, SPEED_ZERO, SPEED_ZERO);
  delay(PAUSE_DURATION + 1000);
  
  // Part 3: Reading Speed Values
  // ----------------------------
  Serial.println("\n=== Reading Current Speed Values ===");
  
  // Set motors to different speeds
  Serial.println("Setting motors to different speeds");
  roboclaw.SpeedM1M2(RC_ADDRESS, SPEED_MEDIUM, SPEED_SLOW);
  
  // Read speeds for a few seconds
  Serial.println("Reading current speed values for 5 seconds:");
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    readAndDisplaySpeeds();
    delay(500);
  }
  
  // Stop motors
  roboclaw.SpeedM1M2(RC_ADDRESS, SPEED_ZERO, SPEED_ZERO);
  delay(PAUSE_DURATION);
  
  // Part 4: Minimum and Maximum Speed Settings
  // -----------------------------------------
  Serial.println("\n=== Minimum and Maximum Speed Settings ===");
  
  // Read current min/max settings
  uint32_t minimumSpeed;
  uint32_t maximumSpeed;
  bool valid = roboclaw.ReadMinMaxMainVoltages(RC_ADDRESS, &minimumSpeed, &maximumSpeed);
  
  if (valid) {
    Serial.print("Current Min Speed: ");
    Serial.println(minimumSpeed);
    Serial.print("Current Max Speed: ");
    Serial.println(maximumSpeed);
  }
  
  // Example of how to set min/max speed (uncommenting may affect your settings)
  /*
  Serial.println("Setting new min/max speed values");
  // Set minimum speed to 500 counts/s and maximum to 8000 counts/s
  roboclaw.SetMinMaxMainVoltages(RC_ADDRESS, 500, 8000);
  delay(100);
  
  // Read back to confirm settings
  valid = roboclaw.ReadMinMaxMainVoltages(RC_ADDRESS, &minimumSpeed, &maximumSpeed);
  if (valid) {
    Serial.print("New Min Speed: ");
    Serial.println(minimumSpeed);
    Serial.print("New Max Speed: ");
    Serial.println(maximumSpeed);
  }
  */
  
  Serial.println("\nDemo sequence complete. Restarting in 3 seconds...");
  delay(3000);
}

/**
 * Read current speeds from both motors and display them
 */
void readAndDisplaySpeeds() {
  uint8_t status1, status2;
  bool valid1, valid2;
  
  // Read motor 1 speed
  int32_t speed1 = roboclaw.ReadSpeedM1(RC_ADDRESS, &status1, &valid1);
  
  // Read motor 2 speed
  int32_t speed2 = roboclaw.ReadSpeedM2(RC_ADDRESS, &status2, &valid2);
  
  if (valid1 && valid2) {
    Serial.print("Motor 1 Speed: ");
    Serial.print(speed1);
    Serial.print(" | Motor 2 Speed: ");
    Serial.println(speed2);
  } else {
    Serial.println("Error reading speed values");
  }
}

/**
 * Run the current command for a specified duration while reading speed values
 */
void runForDuration(unsigned long duration) {
  unsigned long startTime = millis();
  
  while (millis() - startTime < duration) {
    // Read and display speeds periodically
    if (millis() - lastSpeedReadTime >= READ_INTERVAL) {
      readAndDisplaySpeeds();
      lastSpeedReadTime = millis();
    }
    
    // Small delay to prevent overwhelming serial output
    delay(50);
  }
}

/**
 * Get status description based on status byte
 */
const char* getStatusDescription(uint8_t status) {
  switch (status) {
    case 0:
      return "Normal";
    case 1:
      return "Warning: High Current";
    case 2:
      return "Warning: High Temperature";
    // Add more status codes as needed
    default:
      return "Unknown";
  }
}