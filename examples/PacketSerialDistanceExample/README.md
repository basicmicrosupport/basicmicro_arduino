# Basicmicro Motor Controller Movement Library

This Arduino library provides precision movement control functions for Basicmicro (Roboclaw) motor controllers, enabling complex motor trajectories with proper acceleration and deceleration profiles.

## Features

- Point-to-point movements with controlled acceleration and deceleration
- Chained movement sequences with multiple segments
- Dual-motor coordinated movements
- Support for direction changes within movement chains
- Advanced calculations for optimal speed profiles in short movements
- Detailed movement monitoring and status reporting

## Hardware Requirements

- Arduino-compatible board (e.g., Arduino Uno, Mega, ESP32)
- Basicmicro Roboclaw motor controller
- Motors with quadrature encoders
- Power supply appropriate for your motors

## Software Dependencies

- [Basicmicro Roboclaw Arduino Library](https://github.com/basicmicro/roboclaw_arduino_library)

## Installation

1. Install the Basicmicro Roboclaw Arduino library using the Arduino Library Manager
2. Download this repository and place the files in your Arduino project folder:
   - `MotorController.ino` (Main sketch)
   - `MotorHelpers.h` (Function declarations)
   - `MotorHelpers.cpp` (Function implementations)

## Usage Examples

The included `MotorController.ino` sketch demonstrates several movement types:

### Basic Point-to-Point Movement

```cpp
// Move motor 1 forward 5000 counts at 2000 counts/second
pointToPointM1(roboclaw, address, 5000, 2000, 500, 500, 1, true);
waitForMovementComplete(roboclaw, address);
```

### Short Movement (Can't Reach Maximum Speed)

```cpp
// Move a short distance where peak speed is calculated automatically
pointToPointM1(roboclaw, address, 500, 2000, 500, 500, 1, true);
waitForMovementComplete(roboclaw, address);
```

### Chained Movement with Multiple Segments

```cpp
const int numSegments = 5;
float segments[numSegments] = {1000, 2000, 3000, 2000, 0};
float speedMagnitudes[numSegments-1] = {1000, 2000, 3000, 1500};
int directions[numSegments-1] = {1, 1, -1, 1};

chainedMovementM1(roboclaw, address, segments, speedMagnitudes, 
                 directions, numSegments, 500, 500, true);
waitForMovementComplete(roboclaw, address);
```

### Dual-Motor Movement

```cpp
dualMotorPointToPoint(roboclaw, address,
                    5000, 1500, 400, 500, 1,   // Motor 1 params
                    3000, 2000, 600, 400, -1,  // Motor 2 params
                    true);
waitForMovementComplete(roboclaw, address, true, true);
```

## Function Reference

### Movement Calculation Functions

- `calculateDecelDistance` - Calculate distance needed to decelerate from a given speed
- `calculatePeakSpeed` - Calculate maximum achievable speed in a short movement
- `calculateSwitchPoint` - Determine where to start decelerating in a movement
- `calculateChainedSegment` - Calculate parameters for a segment in a chained movement

### Movement Execution Functions

- `pointToPointM1` / `pointToPointM2` - Execute a single point-to-point movement
- `chainedMovementM1` / `chainedMovementM2` - Execute a multi-segment movement
- `dualMotorPointToPoint` - Coordinate point-to-point movements on both motors
- `dualMotorChainedMovement` - Coordinate complex movements on both motors
- `waitForMovementComplete` - Wait for motor movements to complete

## How It Works

The library uses kinematic equations to calculate optimal acceleration and deceleration profiles based on the specified distance, maximum speed, and acceleration/deceleration rates. For short movements where the motor can't reach maximum speed, it automatically calculates the peak achievable speed.

Commands can be chained using the Roboclaw's command buffer system, allowing for complex movement sequences without microcontroller intervention between segments.

## License

This project is released under the MIT License.

## Credits

This library was developed to provide advanced movement control for robotics projects using Basicmicro motor controllers.