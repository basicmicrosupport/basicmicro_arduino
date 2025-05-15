/**
 * Roboclaw Status and Data Monitoring Example
 * 
 * This sketch demonstrates how to read all available status and data information
 * from a Basicmicro Roboclaw motor controller without sending any movement commands.
 * It periodically polls the controller for information like:
 * 
 * - Version and serial number
 * - Encoder readings and position
 * - Temperature readings
 * - Current/Voltage measurements
 * - Error statuses
 * - Battery and logic voltages
 * - PWM and PID settings
 * - Buffer status
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
 * - Arduino board with hardware serial (e.g., Arduino Mega, Due)
 * - Roboclaw motor controller 
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
#define ROBOCLAW_SERIAL Serial1

// Roboclaw address (default is 0x80)
#define RC_ADDRESS 0x80

// Create RoboClaw object
RoboClaw roboclaw(&ROBOCLAW_SERIAL, 10000); // 10ms timeout

// Update intervals in milliseconds
const unsigned long BASIC_INFO_INTERVAL = 5000;    // Version, SN, etc.
const unsigned long ENCODER_INTERVAL = 1000;       // Encoder readings
const unsigned long POWER_INTERVAL = 2000;         // Current, voltage, etc.
const unsigned long TEMPERATURE_INTERVAL = 3000;   // Temperature readings
const unsigned long STATUS_INTERVAL = 1000;        // Status and error flags
const unsigned long SETTINGS_INTERVAL = 10000;     // Controller settings

// Timing variables
unsigned long lastBasicInfoTime = 0;
unsigned long lastEncoderTime = 0;
unsigned long lastPowerTime = 0;
unsigned long lastTemperatureTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSettingsTime = 0;

void setup() {
  // Initialize hardware serial for debugging
  Serial.begin(115200);
  Serial.println("Roboclaw Status and Data Monitoring Example");
  Serial.println("------------------------------------------");
  
  // Initialize serial communication with Roboclaw
  ROBOCLAW_SERIAL.begin(38400);
  roboclaw.begin();
  
  Serial.println("Roboclaw monitor initialized. Starting data collection...");
  delay(2000);
  
  // Get initial basic information
  readBasicInfo();
}

void loop() {
  // Read various data at different intervals to avoid overwhelming 
  // both the serial connection and the console output
  
  // Basic information (version, serial number)
  if (millis() - lastBasicInfoTime >= BASIC_INFO_INTERVAL) {
    readBasicInfo();
    lastBasicInfoTime = millis();
  }
  
  // Encoder readings
  if (millis() - lastEncoderTime >= ENCODER_INTERVAL) {
    readEncoders();
    lastEncoderTime = millis();
  }
  
  // Power readings (current, voltage)
  if (millis() - lastPowerTime >= POWER_INTERVAL) {
    readPowerStatus();
    lastPowerTime = millis();
  }
  
  // Temperature readings
  if (millis() - lastTemperatureTime >= TEMPERATURE_INTERVAL) {
    readTemperatures();
    lastTemperatureTime = millis();
  }
  
  // Status and error flags
  if (millis() - lastStatusTime >= STATUS_INTERVAL) {
    readControllerStatus();
    lastStatusTime = millis();
  }
  
  // Controller settings
  if (millis() - lastSettingsTime >= SETTINGS_INTERVAL) {
    readControllerSettings();
    lastSettingsTime = millis();
  }
  
  // Short delay to prevent excessive CPU usage
  delay(50);
}

/**
 * Read and display basic controller information
 */
void readBasicInfo() {
  Serial.println("\n=== BASIC CONTROLLER INFORMATION ===");
  
  // Read version string
  char version[48];
  uint8_t length = sizeof(version);
  
  bool valid = roboclaw.ReadVersion(RC_ADDRESS, version, length);
  if (valid) {
    Serial.print("Firmware Version: ");
    Serial.println(version);
  } else {
    Serial.println("Failed to read firmware version");
  }
  
  // Read controller serial number
  uint32_t serialNum;
  valid = roboclaw.ReadSerialNumber(RC_ADDRESS, &serialNum);
  if (valid) {
    Serial.print("Serial Number: ");
    Serial.println(serialNum, HEX);
  } else {
    Serial.println("Failed to read serial number");
  }
  
  // Read board name (if supported by firmware)
  char boardName[48];
  valid = roboclaw.ReadBoardName(RC_ADDRESS, boardName, length);
  if (valid) {
    Serial.print("Board Name: ");
    Serial.println(boardName);
  }
}

/**
 * Read and display encoder positions and velocities
 */
void readEncoders() {
  Serial.println("\n=== ENCODER DATA ===");
  
  uint8_t status1, status2;
  bool valid1, valid2;
  
  // Read encoder counts
  uint32_t enc1 = roboclaw.ReadEncM1(RC_ADDRESS, &status1, &valid1);
  uint32_t enc2 = roboclaw.ReadEncM2(RC_ADDRESS, &status2, &valid2);
  
  if (valid1 && valid2) {
    Serial.print("Encoder 1 Count: ");
    Serial.print(enc1);
    Serial.print(" | Status: ");
    Serial.println(status1);
    
    Serial.print("Encoder 2 Count: ");
    Serial.print(enc2);
    Serial.print(" | Status: ");
    Serial.println(status2);
  } else {
    Serial.println("Failed to read encoder counts");
  }
  
  // Read encoder speeds
  int32_t speed1 = roboclaw.ReadSpeedM1(RC_ADDRESS, &status1, &valid1);
  int32_t speed2 = roboclaw.ReadSpeedM2(RC_ADDRESS, &status2, &valid2);
  
  if (valid1 && valid2) {
    Serial.print("Encoder 1 Speed: ");
    Serial.print(speed1);
    Serial.print(" | Status: ");
    Serial.println(status1);
    
    Serial.print("Encoder 2 Speed: ");
    Serial.print(speed2);
    Serial.print(" | Status: ");
    Serial.println(status2);
  } else {
    Serial.println("Failed to read encoder speeds");
  }
  
  // Read encoder distance (if supported)
  uint32_t distance1 = roboclaw.ReadEncoderModeM1(RC_ADDRESS, &valid1);
  uint32_t distance2 = roboclaw.ReadEncoderModeM2(RC_ADDRESS, &valid2);
  
  if (valid1 && valid2) {
    Serial.print("Encoder 1 Mode: ");
    Serial.println(distance1);
    Serial.print("Encoder 2 Mode: ");
    Serial.println(distance2);
  }
}

/**
 * Read and display power-related data (current, voltage)
 */
void readPowerStatus() {
  Serial.println("\n=== POWER STATUS ===");
  
  bool valid;
  
  // Read main battery voltage
  float mainBattery = roboclaw.ReadMainBatteryVoltage(RC_ADDRESS, &valid);
  if (valid) {
    Serial.print("Main Battery: ");
    Serial.print(mainBattery / 10.0, 1); // Convert to volts
    Serial.println("V");
  } else {
    Serial.println("Failed to read main battery voltage");
  }
  
  // Read logic battery voltage
  float logicBattery = roboclaw.ReadLogicBatteryVoltage(RC_ADDRESS, &valid);
  if (valid) {
    Serial.print("Logic Battery: ");
    Serial.print(logicBattery / 10.0, 1); // Convert to volts
    Serial.println("V");
  } else {
    Serial.println("Failed to read logic battery voltage");
  }
  
  // Read motor currents
  int16_t current1 = roboclaw.ReadCurrentM1(RC_ADDRESS, &valid);
  if (valid) {
    Serial.print("Motor 1 Current: ");
    Serial.print(current1 / 100.0, 2); // Convert to amps
    Serial.println("A");
  } else {
    Serial.println("Failed to read Motor 1 current");
  }
  
  int16_t current2 = roboclaw.ReadCurrentM2(RC_ADDRESS, &valid);
  if (valid) {
    Serial.print("Motor 2 Current: ");
    Serial.print(current2 / 100.0, 2); // Convert to amps
    Serial.println("A");
  } else {
    Serial.println("Failed to read Motor 2 current");
  }
  
  // Read both currents simultaneously
  int16_t mcurrent1, mcurrent2;
  valid = roboclaw.ReadCurrents(RC_ADDRESS, &mcurrent1, &mcurrent2);
  if (valid) {
    Serial.print("Motors Current (simultaneous read): M1=");
    Serial.print(mcurrent1 / 100.0, 2);
    Serial.print("A, M2=");
    Serial.print(mcurrent2 / 100.0, 2);
    Serial.println("A");
  } else {
    Serial.println("Failed to read both motor currents");
  }
}

/**
 * Read and display temperature data
 */
void readTemperatures() {
  Serial.println("\n=== TEMPERATURE DATA ===");
  
  bool valid;
  
  // Read temperature for motor 1
  uint16_t temp1 = roboclaw.ReadTemp(RC_ADDRESS, &valid);
  if (valid) {
    Serial.print("Controller Temperature 1: ");
    Serial.print(temp1 / 10.0, 1); // Convert to Celsius
    Serial.println("°C");
  } else {
    Serial.println("Failed to read Temperature 1");
  }
  
  // Read temperature for motor 2
  uint16_t temp2 = roboclaw.ReadTemp2(RC_ADDRESS, &valid);
  if (valid) {
    Serial.print("Controller Temperature 2: ");
    Serial.print(temp2 / 10.0, 1); // Convert to Celsius
    Serial.println("°C");
  } else {
    Serial.println("Failed to read Temperature 2");
  }
  
  // Read both temperatures simultaneously
  uint16_t mtemp1, mtemp2;
  valid = roboclaw.ReadTemps(RC_ADDRESS, &mtemp1, &mtemp2);
  if (valid) {
    Serial.print("Temperatures (simultaneous read): T1=");
    Serial.print(mtemp1 / 10.0, 1);
    Serial.print("°C, T2=");
    Serial.print(mtemp2 / 10.0, 1);
    Serial.println("°C");
  } else {
    Serial.println("Failed to read both temperatures");
  }
}

/**
 * Read and display controller status and error flags
 */
void readControllerStatus() {
  Serial.println("\n=== CONTROLLER STATUS ===");
  
  bool valid;
  
  // Read status flags
  uint16_t status = roboclaw.ReadError(RC_ADDRESS, &valid);
  if (valid) {
    Serial.print("Status/Error: 0x");
    Serial.println(status, HEX);
    
    // Decode error bits
    if (status == 0) {
      Serial.println("No errors detected");
    } else {
      if (status & 0x0001) Serial.println("- Error: Motor 1 Over Current");
      if (status & 0x0002) Serial.println("- Error: Motor 2 Over Current");
      if (status & 0x0004) Serial.println("- Error: Emergency Stop");
      if (status & 0x0008) Serial.println("- Error: Temperature 1 Error");
      if (status & 0x0010) Serial.println("- Error: Temperature 2 Error");
      if (status & 0x0020) Serial.println("- Error: Main Battery High");
      if (status & 0x0040) Serial.println("- Error: Main Battery Low");
      if (status & 0x0080) Serial.println("- Error: Logic Battery High");
      if (status & 0x0100) Serial.println("- Error: Logic Battery Low");
      if (status & 0x0200) Serial.println("- Error: Motor 1 Driver Fault");
      if (status & 0x0400) Serial.println("- Error: Motor 2 Driver Fault");
      if (status & 0x0800) Serial.println("- Error: Main Battery High Shutdown");
      if (status & 0x1000) Serial.println("- Error: Main Battery Low Shutdown");
      if (status & 0x2000) Serial.println("- Error: Fault");
      // Add more error codes as needed
    }
  } else {
    Serial.println("Failed to read error status");
  }
  
  // Read buffer length
  uint8_t depth1, depth2;
  valid = roboclaw.ReadBuffers(RC_ADDRESS, depth1, depth2);
  if (valid) {
    Serial.print("Command Buffers: M1=");
    Serial.print(depth1 & 0x7F); // Lower 7 bits = buffer count
    Serial.print("/32, M2=");
    Serial.print(depth2 & 0x7F);
    Serial.println("/32");
    
    Serial.print("Motor Status: M1=");
    Serial.print((depth1 & 0x80) ? "Idle" : "Running");
    Serial.print(", M2=");
    Serial.println((depth2 & 0x80) ? "Idle" : "Running");
  } else {
    Serial.println("Failed to read buffer status");
  }
}

/**
 * Read and display controller settings and configuration
 */
void readControllerSettings() {
  Serial.println("\n=== CONTROLLER SETTINGS ===");
  
  bool valid;
  
  // Read PID parameters for Motor 1
  float kp1, ki1, kd1, kiMax1;
  uint32_t deadzone1, min1, max1;
  valid = roboclaw.ReadM1VelocityPID(RC_ADDRESS, &kp1, &ki1, &kd1, &kiMax1, &deadzone1, &min1, &max1);
  if (valid) {
    Serial.println("Motor 1 Velocity PID Settings:");
    Serial.print("  KP: ");
    Serial.println(kp1, 4);
    Serial.print("  KI: ");
    Serial.println(ki1, 4);
    Serial.print("  KD: ");
    Serial.println(kd1, 4);
    Serial.print("  KI Max: ");
    Serial.println(kiMax1);
    Serial.print("  Deadzone: ");
    Serial.println(deadzone1);
    Serial.print("  Min/Max: ");
    Serial.print(min1);
    Serial.print("/");
    Serial.println(max1);
  } else {
    Serial.println("Failed to read Motor 1 PID settings");
  }
  
  // Read PID parameters for Motor 2
  float kp2, ki2, kd2, kiMax2;
  uint32_t deadzone2, min2, max2;
  valid = roboclaw.ReadM2VelocityPID(RC_ADDRESS, &kp2, &ki2, &kd2, &kiMax2, &deadzone2, &min2, &max2);
  if (valid) {
    Serial.println("Motor 2 Velocity PID Settings:");
    Serial.print("  KP: ");
    Serial.println(kp2, 4);
    Serial.print("  KI: ");
    Serial.println(ki2, 4);
    Serial.print("  KD: ");
    Serial.println(kd2, 4);
    Serial.print("  KI Max: ");
    Serial.println(kiMax2);
    Serial.print("  Deadzone: ");
    Serial.println(deadzone2);
    Serial.print("  Min/Max: ");
    Serial.print(min2);
    Serial.print("/");
    Serial.println(max2);
  } else {
    Serial.println("Failed to read Motor 2 PID settings");
  }
  
  // Read main battery voltage settings
  float minVoltage, maxVoltage;
  valid = roboclaw.ReadMinMaxMainVoltages(RC_ADDRESS, &minVoltage, &maxVoltage);
  if (valid) {
    Serial.println("Main Battery Settings:");
    Serial.print("  Min Voltage: ");
    Serial.print(minVoltage / 10.0, 1);
    Serial.println("V");
    Serial.print("  Max Voltage: ");
    Serial.print(maxVoltage / 10.0, 1);
    Serial.println("V");
  } else {
    Serial.println("Failed to read main battery settings");
  }
  
  // Read logic battery voltage settings
  valid = roboclaw.ReadMinMaxLogicVoltages(RC_ADDRESS, &minVoltage, &maxVoltage);
  if (valid) {
    Serial.println("Logic Battery Settings:");
    Serial.print("  Min Voltage: ");
    Serial.print(minVoltage / 10.0, 1);
    Serial.println("V");
    Serial.print("  Max Voltage: ");
    Serial.print(maxVoltage / 10.0, 1);
    Serial.println("V");
  } else {
    Serial.println("Failed to read logic battery settings");
  }
  
  // Read PWM mode
  uint8_t pwmMode;
  valid = roboclaw.ReadPWMMode(RC_ADDRESS, &pwmMode);
  if (valid) {
    Serial.print("PWM Mode: ");
    switch (pwmMode) {
      case 0:
        Serial.println("PWM 0-127");
        break;
      case 1:
        Serial.println("RC Mode");
        break;
      case 2:
        Serial.println("Analog Mode");
        break;
      case 3:
        Serial.println("Simple Serial Mode");
        break;
      default:
        Serial.println("Unknown");
    }
  } else {
    Serial.println("Failed to read PWM mode");
  }
}