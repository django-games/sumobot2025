#include <Arduino.h>

#define USE_EXTERNAL_CONTROL false

// Code for a SumoBot robot car using an Arduino UNO R3,
// four DC motor drivers and four motors.

// DRIVERS

/// We're using the BTS7960 motor driver to control four DC motors,
/// hence we have four drivers.
/// Each driver has 6 pins for control, connected to the Arduino:
/// - R_PWM and L_PWM (pulse-width modulation for forward/R and backward/L motion, controls velocity)
/// - R_EN and L_EN (enable input pins for right and left motors, this is an on/off switch)
/// - R_IS and L_IS (current sensing pins for right and left motors)

/// In our setup, we don't use the current sensing pins (R_IS and L_IS),
/// so they will be set to LOW (by connecting to GND).
/// The enable pins (R_EN and L_EN) will be set to HIGH to enable the motors,
/// by connecting them to VCC (5V).
/// We will ONLY use one direction (forward/R) at all times.
/// This is because if we set a non-zero value on both R_PWM and L_PWM,
/// the motors will go kaput! We don't want that!
/// Therefore, we will use a PWM pin for R_PWM, and set L_PWM to LOW
/// through GND.

/// The naming convention for each driver corresponds to its location on the car.

/// This setup means we will have 24 pins to connect in total, of which
/// 4 require PWM control directly from the Arduino board (R_PWM for each driver),
/// 8 of them are enable pins (R_EN and L_EN for each driver) connected to VCC (5V),
/// and 12 of them are current sensing pins (R_IS and L_IS for each driver) + L_PWM
/// connected to GND.

// PIN ASSIGNMENT
/// PWM pins (forward motion)
const int TOP_LEFT_R_PWM = 10;
const int TOP_RIGHT_R_PWM = 5;
const int BOTTOM_LEFT_R_PWM = 11;
const int BOTTOM_RIGHT_R_PWM = 3;

/// PWM pins (backward motion) (hardwired to GND, do NOT leave floating)
/// TOP_LEFT_L_PWM, TOP_RIGHT_L_PWM, BOTTOM_LEFT_L_PWM, and BOTTOM_RIGHT_L_PWM

/// IS pins (current sensing, hardwired to GND)
/// TOP_LEFT_R_IS, TOP_RIGHT_R_IS, BOTTOM_LEFT_R_IS, BOTTOM_RIGHT_R_IS
/// TOP_LEFT_L_IS, TOP_RIGHT_L_IS, BOTTOM_LEFT_L_IS, BOTTOM_RIGHT_L_IS

/// EN pins (enable inputs, hardwired to VCC)
/// TOP_RIGHT_R_EN, TOP_RIGHT_L_EN, BOTTOM_LEFT_R_EN, BOTTOM_LEFT_L_EN
/// TOP_LEFT_R_EN, TOP_LEFT_L_EN, BOTTOM_RIGHT_R_EN, BOTTOM_RIGHT_L_EN

// CONTROL PINS
// For control, we will connect to an external EP32 that will interface
// with the pilot through Wi-Fi. From the Arduino's perspective,
// we will receive a bitmask with two pins, and convert that into
// one of four possible instructions: forward, stop, left, right
const int CONTROL_PIN_LOW = 7;
const int CONTROL_PIN_HIGH = 8;

// OTHER CONSTANTS
const int DELAY_TIME_MS = 300; // Delay time in milliseconds for motor speed changes
const unsigned long COMMAND_TIMEOUT_MS = 1000;
unsigned long lastCommandTime = 0;

const int SPEED_DELTA = 5;
const int SPEED_DELTA_DELAY_MS = 50;
const int TARGET_SPEED = 100;

// Non-blocking ramping variables
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
unsigned long lastRampTime = 0;

/////
// MAIN ARDUINO LOGIC
////

void setup() {
  // Initializing the serial ports
  Serial.begin(9600);

  // Initializing the PWM pins for the motors
  // (forward motion)
  pinMode(TOP_LEFT_R_PWM, OUTPUT);
  pinMode(TOP_RIGHT_R_PWM, OUTPUT);
  pinMode(BOTTOM_LEFT_R_PWM, OUTPUT);
  pinMode(BOTTOM_RIGHT_R_PWM, OUTPUT);

  // Initializing the pins for the external controller
  pinMode(CONTROL_PIN_LOW, INPUT);
  pinMode(CONTROL_PIN_HIGH, INPUT);

  // Ensure all motors are stopped at the start
  setTargetSpeeds(0, 0);

  // Stabilizing before starting the loop
  delay(4 * DELAY_TIME_MS);
}

void loop() {
  // Handle non-blocking ramping first
  updateMotorRamping();

  if (USE_EXTERNAL_CONTROL)
  {
    runExternalControlled();

    if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
      setTargetSpeeds(0, 0);
    }
  }
  else {
    runTestMode();
  }
}

/////
// OPERATION MODES
////

void runExternalControlled()
{
  // Read the current status of both flags
  int lowBit = digitalRead(CONTROL_PIN_LOW);
  int highBit = digitalRead(CONTROL_PIN_HIGH);

  if (lowBit || highBit) {
    lastCommandTime = millis();  // Update last command time when signal is detected
  }

  if (lowBit && highBit) {
    setTargetSpeeds(TARGET_SPEED, TARGET_SPEED);
  } else if (highBit) {
    setTargetSpeeds(TARGET_SPEED / 2, TARGET_SPEED);
  } else if (lowBit) {
    setTargetSpeeds(TARGET_SPEED, TARGET_SPEED / 2);
  } else {
    setTargetSpeeds(0, 0);
  }
}

void runTestMode()
{
  // We will iterate in speed increments of 10
  // from 0 to TARGET_SPEED for forward motion on all motors one by one
  // in circular order.
  int i;

  for(i = 0; i <= TARGET_SPEED; i = i + 10) {
    moveMotors(i, i, i, i);
    delay(DELAY_TIME_MS);
  }
}

/////
// HELPER FUNCTIONS - DIRECTIONAL MOVEMENT
////

void setTargetSpeeds(int leftSpeed, int rightSpeed) {
  // Set target speeds for non-blocking ramping
  targetLeftSpeed = leftSpeed;
  targetRightSpeed = rightSpeed;
}

void updateMotorRamping() {
  // Non-blocking ramping function - call this every loop iteration
  if (millis() - lastRampTime < SPEED_DELTA_DELAY_MS) {
    return; // Not time to update yet
  }

  lastRampTime = millis();

  // Ramp left motors
  if (abs(currentLeftSpeed - targetLeftSpeed) <= SPEED_DELTA) {
    currentLeftSpeed = targetLeftSpeed;
  } else if (currentLeftSpeed < targetLeftSpeed) {
    currentLeftSpeed = constrain(currentLeftSpeed + SPEED_DELTA, currentLeftSpeed, targetLeftSpeed);
  } else {
    currentLeftSpeed = constrain(currentLeftSpeed - SPEED_DELTA, targetLeftSpeed, currentLeftSpeed);
  }

  // Ramp right motors
  if (abs(currentRightSpeed - targetRightSpeed) <= SPEED_DELTA) {
    currentRightSpeed = targetRightSpeed;
  } else if (currentRightSpeed < targetRightSpeed) {
    currentRightSpeed = constrain(currentRightSpeed + SPEED_DELTA, currentRightSpeed, targetRightSpeed);
  } else {
    currentRightSpeed = constrain(currentRightSpeed - SPEED_DELTA, targetRightSpeed, currentRightSpeed);
  }

  // Apply speeds to motors
  moveMotors(currentLeftSpeed, currentRightSpeed, currentLeftSpeed, currentRightSpeed);
}

/////
// HELPER FUNCTIONS - DRIVER INTERFACE
////

void moveMotors(int topLeftSpeed, int topRightSpeed, int bottomLeftSpeed, int bottomRightSpeed) {
  // Send a specific speed value to each of the motors, constrained to a safe range
  analogWrite(TOP_LEFT_R_PWM, validateSpeedValue(topLeftSpeed));
  analogWrite(TOP_RIGHT_R_PWM, validateSpeedValue(topRightSpeed));
  analogWrite(BOTTOM_LEFT_R_PWM, validateSpeedValue(bottomLeftSpeed));
  analogWrite(BOTTOM_RIGHT_R_PWM, validateSpeedValue(bottomRightSpeed));
}

int validateSpeedValue(int speed) {
  // This function ensures the speed value is within the valid range of 0 to 255
  return constrain(speed, 0, 255);
}
