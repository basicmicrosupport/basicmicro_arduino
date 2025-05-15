/**
 * MotorHelpers.h - Distance Movement Helper Functions for Basicmicro Motor Controllers
 * 
 * This file provides helper functions for calculating and executing
 * precise point-to-point movements, chained movements, and dual-motor
 * coordinated movements using the Basicmicro motor controllers.
 */

#ifndef MOTOR_HELPERS_H
#define MOTOR_HELPERS_H

#include <Basicmicro.h>

/**
 * Calculate the distance needed to decelerate from a given speed to zero
 * 
 * @param speed Current speed in encoder counts per second (magnitude)
 * @param decel Deceleration rate in counts/second² (always positive)
 * @return Distance required to stop
 */
float calculateDecelDistance(float speed, float decel);

/**
 * Calculate the peak speed that can be reached in a short move
 * starting from a non-zero initial speed
 * 
 * @param initialSpeed Initial speed in counts/second (signed value)
 * @param distance Total distance to be traveled (always positive)
 * @param accel Acceleration rate in counts/second² (always positive)
 * @param decel Deceleration rate in counts/second² (always positive)
 * @return The peak speed that will be reached (signed value, maintains direction)
 */
float calculatePeakSpeed(float initialSpeed, float distance, float accel, float decel);

/**
 * Calculate the switch point (where to start decelerating) for a move
 * 
 * @param totalDistance Total distance to travel
 * @param accel Acceleration rate
 * @param decel Deceleration rate
 * @param initialSpeed Initial speed (can be non-zero)
 * @param maxSpeed Maximum allowed speed 
 * @return Distance at which to switch from acceleration to deceleration
 */
float calculateSwitchPoint(float totalDistance, float accel, float decel, 
                          float initialSpeed, float maxSpeed);

/**
 * Calculate the distance and speed for a segment in a chained movement
 *
 * @param startSpeed Current speed at the start of this segment (signed value)
 * @param targetSpeedMagnitude Desired end speed magnitude for this segment (always positive)
 * @param distance Total distance for this segment (always positive)
 * @param accel Acceleration rate in counts/second² (always positive)
 * @param decel Deceleration rate in counts/second² (always positive)
 * @param direction Direction for this segment: 1 for forward, -1 for backward
 * @param segmentDistance Output: Distance to command for this segment (always positive)
 * @param finalSpeed Output: Speed to command for this segment (signed value)
 * @return true if calculation successful
 */
bool calculateChainedSegment(float startSpeed, float targetSpeedMagnitude, float distance,
                            float accel, float decel, int direction,
                            float &segmentDistance, float &finalSpeed);

/**
 * Execute a chained movement with multiple segments on Motor 1,
 * ending with a stop at the final position.
 * 
 * @param roboclaw Reference to the Roboclaw controller
 * @param address Controller address 
 * @param segments Array of distance segments to execute (all positive values)
 * @param speedMagnitudes Array of speed magnitudes for each segment (all positive values)
 * @param directions Array of directions for each segment (1 = forward, -1 = backward)
 * @param numSegments Number of segments (must be at least 2)
 * @param accel Acceleration rate
 * @param decel Deceleration rate
 * @param immediate Whether the first command executes immediately (true) or buffers (false)
 * @return true if commands were sent successfully
 */
bool chainedMovementM1(Basicmicro &roboclaw, uint8_t address,
                      float segments[], float speedMagnitudes[], int directions[], int numSegments,
                      float accel, float decel, bool immediate = true);

/**
 * Execute a chained movement with multiple segments on Motor 2,
 * ending with a stop at the final position.
 * 
 * @param roboclaw Reference to the Roboclaw controller
 * @param address Controller address 
 * @param segments Array of distance segments to execute (all positive values)
 * @param speedMagnitudes Array of speed magnitudes for each segment (all positive values)
 * @param directions Array of directions for each segment (1 = forward, -1 = backward)
 * @param numSegments Number of segments (must be at least 2)
 * @param accel Acceleration rate
 * @param decel Deceleration rate
 * @param immediate Whether the first command executes immediately (true) or buffers (false)
 * @return true if commands were sent successfully
 */
bool chainedMovementM2(Basicmicro &roboclaw, uint8_t address,
                      float segments[], float speedMagnitudes[], int directions[], int numSegments,
                      float accel, float decel, bool immediate = true);

/**
 * Execute a point-to-point movement on Motor 1 as a special case of chained movement
 * 
 * @param roboclaw Reference to the Roboclaw controller
 * @param address Controller address
 * @param distance Total distance to travel (always positive)
 * @param maxSpeed Maximum desired speed magnitude (always positive)
 * @param accel Acceleration rate (always positive)
 * @param decel Deceleration rate (always positive)
 * @param direction Direction: 1 for forward, -1 for backward
 * @param immediate Whether to execute immediately (true) or buffer after current command (false)
 * @return true if commands were sent successfully
 */
bool pointToPointM1(Basicmicro &roboclaw, uint8_t address, 
                   float distance, float maxSpeed, float accel, float decel, 
                   int direction, bool immediate = true);

/**
 * Execute a point-to-point movement on Motor 2 as a special case of chained movement
 * 
 * @param roboclaw Reference to the Roboclaw controller
 * @param address Controller address
 * @param distance Total distance to travel (always positive)
 * @param maxSpeed Maximum desired speed magnitude (always positive)
 * @param accel Acceleration rate (always positive)
 * @param decel Deceleration rate (always positive)
 * @param direction Direction: 1 for forward, -1 for backward
 * @param immediate Whether to execute immediately (true) or buffer after current command (false)
 * @return true if commands were sent successfully
 */
bool pointToPointM2(Basicmicro &roboclaw, uint8_t address, 
                   float distance, float maxSpeed, float accel, float decel, 
                   int direction, bool immediate = true);

/**
 * Execute independent chained movements on both motors
 * 
 * @param roboclaw Reference to the Roboclaw controller
 * @param address Controller address
 * @param segmentsM1 Array of distance segments for Motor 1
 * @param speedMagnitudesM1 Array of speed magnitudes for Motor 1
 * @param directionsM1 Array of directions for Motor 1
 * @param numSegmentsM1 Number of segments for Motor 1
 * @param segmentsM2 Array of distance segments for Motor 2
 * @param speedMagnitudesM2 Array of speed magnitudes for Motor 2
 * @param directionsM2 Array of directions for Motor 2
 * @param numSegmentsM2 Number of segments for Motor 2
 * @param accel Acceleration rate for both motors
 * @param decel Deceleration rate for both motors
 * @param immediate Whether to execute immediately (true) or buffer after current command (false)
 * @return true if commands were sent successfully
 */
bool dualMotorChainedMovement(Basicmicro &roboclaw, uint8_t address,
                            float segmentsM1[], float speedMagnitudesM1[], int directionsM1[], int numSegmentsM1,
                            float segmentsM2[], float speedMagnitudesM2[], int directionsM2[], int numSegmentsM2,
                            float accel, float decel, bool immediate = true);

/**
 * Execute a dual-motor point-to-point movement as a special case of chained movement
 * 
 * @param roboclaw Reference to the Roboclaw controller
 * @param address Controller address
 * @param distanceM1 Total distance for Motor 1 (always positive)
 * @param maxSpeedM1 Maximum desired speed magnitude for Motor 1 (always positive)
 * @param accelM1 Acceleration rate for Motor 1
 * @param decelM1 Deceleration rate for Motor 1
 * @param directionM1 Direction for Motor 1: 1 for forward, -1 for backward
 * @param distanceM2 Total distance for Motor 2 (always positive)
 * @param maxSpeedM2 Maximum desired speed magnitude for Motor 2 (always positive)
 * @param accelM2 Acceleration rate for Motor 2
 * @param decelM2 Deceleration rate for Motor 2
 * @param directionM2 Direction for Motor 2: 1 for forward, -1 for backward
 * @param immediate Whether to execute immediately (true) or buffer after current command (false)
 * @return true if commands were sent successfully
 */
bool dualMotorPointToPoint(Basicmicro &roboclaw, uint8_t address,
                         float distanceM1, float maxSpeedM1, float accelM1, float decelM1, int directionM1,
                         float distanceM2, float maxSpeedM2, float accelM2, float decelM2, int directionM2,
                         bool immediate = true);

/**
 * Helper function to wait for movement to complete for one or both motor channels
 * 
 * @param roboclaw Reference to the Roboclaw controller
 * @param address Controller address
 * @param waitForM1 Whether to wait for Motor 1
 * @param waitForM2 Whether to wait for Motor 2
 * @param timeout_ms Maximum wait time in milliseconds (0 = wait forever)
 * @return true if movement completed, false if timed out
 */
bool waitForMovementComplete(Basicmicro &roboclaw, uint8_t address, 
                           bool waitForM1 = true, bool waitForM2 = false,
                           unsigned long timeout_ms = 0);

#endif // MOTOR_HELPERS_H