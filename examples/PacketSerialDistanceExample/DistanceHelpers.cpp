/**
 * MotorHelpers.cpp - Distance Movement Helper Functions for Basicmicro Motor Controllers
 * 
 * Implementation of functions declared in MotorHelpers.h
 */

#include "MotorHelpers.h"
#include <Arduino.h>
#include <Basicmicro.h>

float calculateDecelDistance(float speed, float decel) {
  // Use the kinematic equation: distance = speed² / (2 * deceleration)
  return (speed * speed) / (2.0f * decel);
}

float calculatePeakSpeed(float initialSpeed, float distance, float accel, float decel) {
  // Get the direction and magnitude of the initial speed
  float direction = initialSpeed >= 0 ? 1.0f : -1.0f;
  float initialSpeedMag = abs(initialSpeed);
  
  // Calculate the distance needed to stop from initial speed
  float stopDistance = calculateDecelDistance(initialSpeedMag, decel);
  
  // If stopping distance exceeds available distance, we can't even stop
  if (stopDistance >= distance) {
    // Calculate the speed at the end of the distance
    float finalSpeedSq = initialSpeedMag * initialSpeedMag - 2.0f * decel * distance;
    if (finalSpeedSq <= 0) {
      return 0; // Can just barely stop in this distance
    }
    float finalSpeed = sqrt(finalSpeedSq);
    return finalSpeed * direction; // Maintain direction
  }
  
  // Case where we need to accelerate then decelerate
  // For a short move, we need to solve for the peak speed
  // Using the quadratic formula: a*v² + b*v + c = 0
  float a = 1.0f/(2.0f*accel) + 1.0f/(2.0f*decel);
  float b = -initialSpeedMag*initialSpeedMag/(2.0f*accel);
  float c = -distance;
  
  // Calculate the discriminant
  float discriminant = b*b - 4.0f*a*c;
  
  // Ensure the discriminant is positive (should always be for valid inputs)
  if (discriminant < 0) {
    return initialSpeed; // Fallback case, shouldn't happen with valid inputs
  }
  
  // Use the positive root for v_peak
  float peakSpeedMag = sqrt((-b + sqrt(discriminant)) / (2.0f*a));
  
  // Return the peak speed with the original direction
  return peakSpeedMag * direction;
}

float calculateSwitchPoint(float totalDistance, float accel, float decel, 
                          float initialSpeed, float maxSpeed) {
  // Get magnitudes
  float initialSpeedMag = abs(initialSpeed);
  float maxSpeedMag = abs(maxSpeed);
  
  // Calculate the theoretical accel and decel distances
  float accelDist = (maxSpeedMag * maxSpeedMag - initialSpeedMag * initialSpeedMag) / (2.0f * accel);
  float decelDist = (maxSpeedMag * maxSpeedMag) / (2.0f * decel);
  
  // Check if this is a short move (can't reach top speed)
  if (totalDistance < (accelDist + decelDist)) {
    // For a short move with non-zero initial speed, use a modified formula
    return (totalDistance * decel + initialSpeedMag * initialSpeedMag * accel / (2.0f * decel)) / (accel + decel);
  } else {
    // Standard move with full acceleration, cruise, and deceleration
    return totalDistance - decelDist;
  }
}

bool calculateChainedSegment(float startSpeed, float targetSpeedMagnitude, float distance,
                            float accel, float decel, int direction,
                            float &segmentDistance, float &finalSpeed) {
  if (distance <= 0 || accel <= 0 || decel <= 0 || (direction != 1 && direction != -1)) {
    return false;
  }
  
  // Target speed with direction applied
  float targetSpeed = targetSpeedMagnitude * direction;
  
  // The sign of a speed determines direction
  bool startForward = startSpeed >= 0;
  bool targetForward = targetSpeed >= 0;
  
  // Check if we're changing direction
  bool changingDirection = (startForward != targetForward) && (startSpeed != 0) && (targetSpeed != 0);
  
  if (changingDirection) {
    // First need to stop, then accelerate in the other direction
    // Distance needed to stop from current speed
    float stopDistance = calculateDecelDistance(abs(startSpeed), decel);
    
    // If we don't have enough distance to stop
    if (stopDistance >= distance) {
      // Calculate the speed at the end of the distance
      float finalSpeedSq = startSpeed * startSpeed - 2.0f * decel * distance * (startForward ? 1 : -1);
      if (finalSpeedSq <= 0) {
        // Can just barely stop
        finalSpeed = 0;
      } else {
        finalSpeed = sqrt(finalSpeedSq) * (startForward ? 1 : -1);
      }
      segmentDistance = distance;
      return true;
    }
    
    // We can stop with distance to spare
    // Calculate how much distance remains after stopping
    float remainingDistance = distance - stopDistance;
    
    // Calculate how fast we can accelerate in the new direction
    float maxAccelSpeed = sqrt(2.0f * accel * remainingDistance);
    
    // Limit to target speed
    if (maxAccelSpeed > abs(targetSpeed)) {
      finalSpeed = targetSpeed;
    } else {
      finalSpeed = maxAccelSpeed * (targetForward ? 1 : -1);
    }
    
    segmentDistance = distance;
    return true;
  }
  
  // Same direction or starting from zero
  // Are we accelerating or decelerating?
  bool isAccelerating = abs(targetSpeed) > abs(startSpeed);
  float rate = isAccelerating ? accel : decel;
  
  // Calculate the distance needed for the speed change
  float speedChangeSq = targetSpeed * targetSpeed - startSpeed * startSpeed;
  float speedChangeDistance = abs(speedChangeSq) / (2.0f * rate);
  
  // If we can't reach the target speed within the given distance
  if (speedChangeDistance > distance) {
    // Calculate the maximum speed we can reach
    float finalSpeedSq = startSpeed * startSpeed + 2.0f * rate * distance * (isAccelerating ? 1 : -1);
    if (finalSpeedSq <= 0) {
      // Can't even maintain direction
      finalSpeed = 0;
    } else {
      finalSpeed = sqrt(finalSpeedSq) * (startForward ? 1 : -1);
    }
  } else {
    // We can reach the target speed with distance to spare
    finalSpeed = targetSpeed;
  }
  
  segmentDistance = distance;
  return true;
}

bool chainedMovementM1(Basicmicro &roboclaw, uint8_t address,
                      float segments[], float speedMagnitudes[], int directions[], int numSegments,
                      float accel, float decel, bool immediate) {
  if (numSegments < 2) {
    return false;
  }
  
  float currentSpeed = 0;
  bool success = true;
  
  // Process all segments except the last one
  for (int i = 0; i < numSegments - 1; i++) {
    float segmentDistance, finalSpeed;
    
    if (!calculateChainedSegment(currentSpeed, speedMagnitudes[i], segments[i],
                              accel, decel, directions[i], segmentDistance, finalSpeed)) {
      return false;
    }
    
    // Buffer flag:
    // 0 = Buffer command (after current command)
    // 1 = Execute immediately (cancel any buffered commands)
    uint8_t flag = (i == 0 && immediate) ? 1 : 0;
    
    success = roboclaw.SpeedAccelDistanceM1(
                address, accel, finalSpeed, segmentDistance, flag);
    
    if (!success) {
      return false;
    }
    
    currentSpeed = finalSpeed;
  }
  
  // Process the final segment (deceleration to zero)
  float finalDistance = calculateDecelDistance(abs(currentSpeed), decel);
  
  // If the final segment distance is provided and greater than what we need to stop
  if (segments[numSegments-1] > finalDistance) {
    finalDistance = segments[numSegments-1];
  }
  
  // Final segment flag: Always buffered (0)
  uint8_t flag = 0;
  
  success = roboclaw.SpeedAccelDistanceM1(
              address, decel, 0, finalDistance, flag);
  
  return success;
}

bool chainedMovementM2(Basicmicro &roboclaw, uint8_t address,
                      float segments[], float speedMagnitudes[], int directions[], int numSegments,
                      float accel, float decel, bool immediate) {
  if (numSegments < 2) {
    return false;
  }
  
  float currentSpeed = 0;
  bool success = true;
  
  // Process all segments except the last one
  for (int i = 0; i < numSegments - 1; i++) {
    float segmentDistance, finalSpeed;
    
    if (!calculateChainedSegment(currentSpeed, speedMagnitudes[i], segments[i],
                              accel, decel, directions[i], segmentDistance, finalSpeed)) {
      return false;
    }
    
    // Buffer flag:
    // 0 = Buffer command (after current command)
    // 1 = Execute immediately (cancel any buffered commands)
    uint8_t flag = (i == 0 && immediate) ? 1 : 0;
    
    success = roboclaw.SpeedAccelDistanceM2(
                address, accel, finalSpeed, segmentDistance, flag);
    
    if (!success) {
      return false;
    }
    
    currentSpeed = finalSpeed;
  }
  
  // Process the final segment (deceleration to zero)
  float finalDistance = calculateDecelDistance(abs(currentSpeed), decel);
  
  // If the final segment distance is provided and greater than what we need to stop
  if (segments[numSegments-1] > finalDistance) {
    finalDistance = segments[numSegments-1];
  }
  
  // Final segment flag: Always buffered (0)
  uint8_t flag = 0;
  
  success = roboclaw.SpeedAccelDistanceM2(
              address, decel, 0, finalDistance, flag);
  
  return success;
}

bool pointToPointM1(Basicmicro &roboclaw, uint8_t address, 
                   float distance, float maxSpeed, float accel, float decel, 
                   int direction, bool immediate) {
  // Create a 2-segment movement (acceleration+cruise, deceleration)
  const int numSegments = 2;
  float segments[numSegments] = {distance, 0}; // Second distance is calculated by chainedMovementM1
  float speedMagnitudes[numSegments-1] = {maxSpeed};
  int directions[numSegments-1] = {direction};
  
  // Use the more general chainedMovementM1 function
  return chainedMovementM1(roboclaw, address, segments, speedMagnitudes, directions, numSegments,
                          accel, decel, immediate);
}

bool pointToPointM2(Basicmicro &roboclaw, uint8_t address, 
                   float distance, float maxSpeed, float accel, float decel, 
                   int direction, bool immediate) {
  // Create a 2-segment movement (acceleration+cruise, deceleration)
  const int numSegments = 2;
  float segments[numSegments] = {distance, 0}; // Second distance is calculated by chainedMovementM2
  float speedMagnitudes[numSegments-1] = {maxSpeed};
  int directions[numSegments-1] = {direction};
  
  // Use the more general chainedMovementM2 function
  return chainedMovementM2(roboclaw, address, segments, speedMagnitudes, directions, numSegments,
                          accel, decel, immediate);
}

bool dualMotorChainedMovement(Basicmicro &roboclaw, uint8_t address,
                            float segmentsM1[], float speedMagnitudesM1[], int directionsM1[], int numSegmentsM1,
                            float segmentsM2[], float speedMagnitudesM2[], int directionsM2[], int numSegmentsM2,
                            float accel, float decel, bool immediate) {
  // Execute the chained movements independently for each motor
  bool successM1 = chainedMovementM1(roboclaw, address, segmentsM1, speedMagnitudesM1, directionsM1, 
                                   numSegmentsM1, accel, decel, immediate);
  
  bool successM2 = chainedMovementM2(roboclaw, address, segmentsM2, speedMagnitudesM2, directionsM2, 
                                   numSegmentsM2, accel, decel, immediate);
  
  // Return success if both motors' command sequences were set up successfully
  return successM1 && successM2;
}

bool dualMotorPointToPoint(Basicmicro &roboclaw, uint8_t address,
                         float distanceM1, float maxSpeedM1, float accelM1, float decelM1, int directionM1,
                         float distanceM2, float maxSpeedM2, float accelM2, float decelM2, int directionM2,
                         bool immediate) {
  // Create 2-segment movements for each motor
  const int numSegments = 2;
  
  // Motor 1 parameters
  float segmentsM1[numSegments] = {distanceM1, 0};
  float speedMagnitudesM1[numSegments-1] = {maxSpeedM1};
  int directionsM1[numSegments-1] = {directionM1};
  
  // Motor 2 parameters
  float segmentsM2[numSegments] = {distanceM2, 0};
  float speedMagnitudesM2[numSegments-1] = {maxSpeedM2};
  int directionsM2[numSegments-1] = {directionM2};
  
  // Use the more general dualMotorChainedMovement function
  return dualMotorChainedMovement(roboclaw, address,
                                segmentsM1, speedMagnitudesM1, directionsM1, numSegments,
                                segmentsM2, speedMagnitudesM2, directionsM2, numSegments,
                                accelM1, decelM1, immediate);
}

bool waitForMovementComplete(Basicmicro &roboclaw, uint8_t address, 
                           bool waitForM1, bool waitForM2,
                           unsigned long timeout_ms) {
  uint8_t depth1, depth2;
  unsigned long startTime = millis();
  unsigned long lastUpdateTime = 0;
  
  do {
    // Check buffer status
    roboclaw.ReadBuffers(address, depth1, depth2);
    
    // Check if both requested motors are done
    // A motor is done when:
    // 1. Its buffer is empty (depth & 0x7F = 0)
    // 2. AND bit 7 is set indicating idle state (depth & 0x80 != 0)
    bool m1Done = !waitForM1 || ((depth1 & 0x7F) == 0 && (depth1 & 0x80));
    bool m2Done = !waitForM2 || ((depth2 & 0x7F) == 0 && (depth2 & 0x80));
    
    if (m1Done && m2Done) {
      // Both requested motors have completed their movements
      Serial.println("Movement complete");
      return true;
    }
    
    // Print status every 250ms
    if (millis() - lastUpdateTime >= 250) {
      lastUpdateTime = millis();
      
      // Read current position and speed
      uint8_t status1, status2;
      bool valid1, valid2;
      uint32_t enc1 = 0, enc2 = 0;
      uint32_t speed1 = 0, speed2 = 0;
      
      if (waitForM1) {
        enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
        speed1 = roboclaw.ReadSpeedM1(address, &status1, &valid1);
      }
      
      if (waitForM2) {
        enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
        speed2 = roboclaw.ReadSpeedM2(address, &status2, &valid2);
      }
      
      // Print motor 1 info if waiting for it
      if (waitForM1) {
        Serial.print("M1: Pos=");
        Serial.print(enc1);
        Serial.print(", Speed=");
        Serial.print(speed1);
        Serial.print(", Buffer=");
        Serial.print(depth1 & 0x7F); // Mask off bit 7 to show only buffer count
        Serial.print("/32");
        Serial.print(", State=");
        Serial.print((depth1 & 0x80) ? "Idle" : "Running");
      }
      
      // Print motor 2 info if waiting for it
      if (waitForM2) {
        if (waitForM1) Serial.print(" | ");
        Serial.print("M2: Pos=");
        Serial.print(enc2);
        Serial.print(", Speed=");
        Serial.print(speed2);
        Serial.print(", Buffer=");
        Serial.print(depth2 & 0x7F); // Mask off bit 7 to show only buffer count
        Serial.print("/32");
        Serial.print(", State=");
        Serial.print((depth2 & 0x80) ? "Idle" : "Running");
      }
      
      Serial.println();
    }
    
    // Check for timeout if specified
    if (timeout_ms > 0 && (millis() - startTime > timeout_ms)) {
      Serial.println("Movement timed out!");
      return false;
    }
    
  } while (true);
}