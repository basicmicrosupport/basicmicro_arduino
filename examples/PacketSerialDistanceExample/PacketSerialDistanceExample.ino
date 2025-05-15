/**
 * Basicmicro Motor Controller - Movement Demo
 * 
 * This sketch demonstrates using helper functions for precision movement control
 * with Basicmicro (Roboclaw) motor controllers. The helper functions are defined
 * in the MotorHelpers.h file.
 */

#include <Basicmicro.h>  // Include the Basicmicro/Roboclaw library
#include "DistanceHelpers.h" // Include our helper functions

// Serial port for communication with the controller
#define SERIAL_PORT Serial1

// Connection settings
#define MOTOR_ADDRESS 0x80  // Default address for Roboclaw
#define BAUDRATE 38400      // Default baud rate

// Create Basicmicro controller instance
Basicmicro roboclaw(&SERIAL_PORT);

// Movement parameters
#define DEFAULT_ACCEL 500   // Default acceleration rate (counts/s²)
#define DEFAULT_DECEL 500   // Default deceleration rate (counts/s²)
#define DEFAULT_SPEED 2000  // Default speed (counts/s)

void setup() {
  // Initialize main serial port for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 5000); // Wait for serial connection
  
  // Initialize motor controller serial port
  SERIAL_PORT.begin(BAUDRATE);
  
  Serial.println("Basicmicro Motor Controller - Movement Demo");
  Serial.println("=========================================");
  
  // Reset encoders at startup
  roboclaw.ResetEncoders(MOTOR_ADDRESS);
  delay(100);
  
  // Run the movement examples
  movementExamples(roboclaw, MOTOR_ADDRESS);
}

void loop() {
  // Nothing in the main loop - all examples are run from setup
}

/**
 * Run a series of example movements to demonstrate the helper functions
 */
void movementExamples(Basicmicro &roboclaw, uint8_t address) {
  Serial.println("Starting movement examples...");
  
  // Example 1: Simple point-to-point movement
  Serial.println("\n1. Simple point-to-point movement");
  Serial.println("--------------------------------");
  float distance = 5000;
  float maxSpeed = 2000;
  float accel = DEFAULT_ACCEL;
  float decel = DEFAULT_DECEL;
  int direction = 1; // Forward
  
  Serial.println("Moving forward 5000 counts at speed 2000...");
  pointToPointM1(roboclaw, address, distance, maxSpeed, accel, decel, direction, true);
  
  // Wait for movement to complete
  waitForMovementComplete(roboclaw, address);
  delay(1000);
  
  // Example 2: Short point-to-point movement
  Serial.println("\n2. Short movement (can't reach max speed)");
  Serial.println("---------------------------------------");
  float shortDistance = 500;
  float shortMaxSpeed = 2000;
  
  Serial.println("Moving forward 500 counts...");
  pointToPointM1(roboclaw, address, shortDistance, shortMaxSpeed, accel, decel, direction, true);
  
  // Wait for movement to complete
  waitForMovementComplete(roboclaw, address);
  delay(1000);
  
  // Example 3: Complex chained movement
  Serial.println("\n3. Chained movement with multiple segments");
  Serial.println("----------------------------------------");
  
  const int numSegments = 5;
  float segments[numSegments] = {1000, 2000, 3000, 2000, 0}; // Last segment for deceleration
  float speedMagnitudes[numSegments-1] = {1000, 2000, 3000, 1500};
  int directions[numSegments-1] = {1, 1, -1, 1};
  
  Serial.println("Executing 5-segment movement with direction changes:");
  for (int i = 0; i < numSegments - 1; i++) {
    Serial.print("Segment ");
    Serial.print(i + 1);
    Serial.print(": Distance=");
    Serial.print(segments[i]);
    Serial.print(", Speed=");
    Serial.print(speedMagnitudes[i] * directions[i]);
    Serial.println(directions[i] == 1 ? " (Forward)" : " (Backward)");
  }
  Serial.println("Final segment: Deceleration to stop");
  
  chainedMovementM1(roboclaw, address, segments, speedMagnitudes, directions, numSegments, accel, decel, true);
  
  // Wait for movement to complete
  waitForMovementComplete(roboclaw, address);
  delay(1000);
  
  // Example 4: Dual motor movement
  Serial.println("\n4. Dual motor movement");
  Serial.println("---------------------");
  Serial.println("Moving both motors independently:");
  
  // Motor 1 parameters
  float distanceM1 = 5000;
  float speedM1 = 1500;
  float accelM1 = 400;
  float decelM1 = 500;
  int directionM1 = 1;  // Forward
  
  // Motor 2 parameters
  float distanceM2 = 3000;
  float speedM2 = 2000;
  float accelM2 = 600;
  float decelM2 = 400;
  int directionM2 = -1; // Backward
  
  Serial.print("Motor 1: ");
  Serial.print(distanceM1);
  Serial.print(" counts forward at speed ");
  Serial.println(speedM1);
  
  Serial.print("Motor 2: ");
  Serial.print(distanceM2);
  Serial.print(" counts backward at speed ");
  Serial.println(speedM2);
  
  dualMotorPointToPoint(roboclaw, address,
                      distanceM1, speedM1, accelM1, decelM1, directionM1,
                      distanceM2, speedM2, accelM2, decelM2, directionM2,
                      true);
  
  // Wait for both motors to complete
  waitForMovementComplete(roboclaw, address, true, true);
  
  Serial.println("\n5. Immediate vs. Buffered Commands");
  Serial.println("--------------------------------");
  
  // Reset encoders
  roboclaw.ResetEncoders(address);
  delay(100);
  
  // Start a movement that we will interrupt
  Serial.println("Starting long movement...");
  float longDistance = 10000;
  float longSpeed = 1000;
  
  // Flag 1 = immediate execution
  roboclaw.SpeedAccelDistanceM1(address, 500, longSpeed, longDistance, 1);
  
  // Let it run for a short time
  delay(2000);
  
  Serial.println("Interrupting with immediate command...");
  // Execute a new command immediately (flag 1)
  float shortInterruptDistance = 2000;
  float shortInterruptSpeed = 2000;
  roboclaw.SpeedAccelDistanceM1(address, 1000, shortInterruptSpeed, shortInterruptDistance, 1);
  
  // Wait for this movement to complete
  waitForMovementComplete(roboclaw, address);
  
  delay(1000);
  
  // Reset encoders
  roboclaw.ResetEncoders(address);
  delay(100);
  
  // Now demonstrate buffered commands
  Serial.println("\nDemonstrating buffered commands:");
  
  // First movement
  Serial.println("Executing first movement...");
  float distance1 = 3000;
  float speed1 = 1500;
  
  // Flag 1 = immediate execution
  roboclaw.SpeedAccelDistanceM1(address, 500, speed1, distance1, 1);
  
  // Buffer second movement
  Serial.println("Buffering second movement...");
  float distance2 = 4000;
  float speed2 = 2500;
  
  // Flag 0 = buffered execution
  roboclaw.SpeedAccelDistanceM1(address, 800, speed2, distance2, 0);
  
  // Buffer final stop
  Serial.println("Buffering final stop...");
  float stopDistance = calculateDecelDistance(speed2, 500);
  
  // Flag 0 = buffered execution
  roboclaw.SpeedAccelDistanceM1(address, 500, 0, stopDistance, 0);
  
  // Wait for all movements to complete
  waitForMovementComplete(roboclaw, address);
  
  Serial.println("All movement examples complete");
}