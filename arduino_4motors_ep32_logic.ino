#include <Arduino.h>

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
/// Forward direction will be controlled by the R_PWM pins,
/// and will be our main direction with complete control over speed.

/// The L_PWM pins will be used for backward motion.
/// but through digital pins, not PWM. So they will be either HIGH or LOW.
/// We will take EXTRA care to ensure that we never set both R_PWM and L_PWM
/// to a non-zero value at the same time, as this would cause a short circuit
/// and potentially damage the motors or the drivers.
/// This is because if we set a non-zero value on both R_PWM and L_PWM,
/// the motors will go kaput! We don't want that!

/// The naming convention for each driver corresponds to its location on the car.

/// This setup means we will have 24 pins to connect in total, of which
/// 4 require PWM control directly from the Arduino board (R_PWM for each driver),
/// 4 require digital control for backward motion (L_PWM for each driver),
/// 8 of them are enable pins (R_EN and L_EN for each driver) connected to VCC (5V),
/// and 8 of them are current sensing pins (R_IS and L_IS for each driver)
/// connected to GND.

// PIN ASSIGNMENT
/// PWM pins (forward motion)
const int TOP_LEFT_R_PWM = 10;
const int TOP_RIGHT_R_PWM = 5;
const int BOTTOM_LEFT_R_PWM = 11;
const int BOTTOM_RIGHT_R_PWM = 3;

/// PWM pins (backward motion) (we will use Digital Pins 2, 4, 12 and 13 for these)
const int TOP_LEFT_L_PWM = 2;
const int TOP_RIGHT_L_PWM = 4;
const int BOTTOM_LEFT_L_PWM = 12;
const int BOTTOM_RIGHT_L_PWM = 13;

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
const int CONTROL_PIN_LEFT = 7;
const int CONTROL_PIN_RIGHT = 8;
const int CONTROL_PIN_REVERSE = 6;

// OTHER CONSTANTS
const unsigned long COMMAND_TIMEOUT_MS = 2000;
unsigned long lastCommandTime = 0;

// Speed variables
const int MAX_SPEED = 255;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int isReverse = 0;

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
  pinMode(CONTROL_PIN_LEFT, INPUT);
  pinMode(CONTROL_PIN_RIGHT, INPUT);
  pinMode(CONTROL_PIN_REVERSE, INPUT);

  // Ensure all motors are stopped at the start
  setTargetSpeeds(0, 0);
  isReverse = 0;  // Start in forward mode

  // Stabilizing before starting the loop
  delay(1000);
}

void loop() {
  // Send movement signal
  moveMotors(targetLeftSpeed, targetRightSpeed, targetLeftSpeed, targetRightSpeed);

  // Get new signal, if any
  runExternalControlled();

  // In case the last command was too long ago, stop the motors
  // to prevent them from running indefinitely without a command
  // This is a safety feature to ensure the motors stop if no command is received
  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    setTargetSpeeds(0, 0);
  }
}

/////
// OPERATION MODES
////

void runExternalControlled()
{
  // Read the current status of both flags
  int reversePin = digitalRead(CONTROL_PIN_REVERSE);
  int leftBit = digitalRead(CONTROL_PIN_LEFT);
  int rightBit = digitalRead(CONTROL_PIN_RIGHT);

  if (leftBit == 1 || rightBit == 1 || reversePin == 1) {
    lastCommandTime = millis();  // Update last command time when signal is detected
  }

  if (reversePin == HIGH) {
    isReverse = 1;  // Set reverse mode
  } else {
    isReverse = 0;  // Set forward mode
  }

  // This logic only applies when the robot is not in reverse mode
  if (leftBit == 1 && rightBit == 1) {
    setTargetSpeeds(MAX_SPEED, MAX_SPEED);
  } else if (rightBit == 1) {
    setTargetSpeeds(0, MAX_SPEED);
  } else if (leftBit == 1) {
    setTargetSpeeds(MAX_SPEED, 0);
  } else {
    setTargetSpeeds(0, 0);
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

/////
// HELPER FUNCTIONS - DRIVER INTERFACE
////

void moveMotors(int topLeftSpeed, int topRightSpeed, int bottomLeftSpeed, int bottomRightSpeed) {
  if (isReverse == 0) {
    // Send a specific speed value to each of the motors, constrained to a safe range
    digitalWrite(TOP_LEFT_L_PWM, LOW);
    digitalWrite(TOP_RIGHT_L_PWM, LOW);
    digitalWrite(BOTTOM_LEFT_L_PWM, LOW);
    digitalWrite(BOTTOM_RIGHT_L_PWM, LOW);

    // Set the PWM values for forward motion
    analogWrite(TOP_LEFT_R_PWM, validateSpeedValue(topLeftSpeed));
    analogWrite(TOP_RIGHT_R_PWM, validateSpeedValue(topRightSpeed));
    analogWrite(BOTTOM_LEFT_R_PWM, validateSpeedValue(bottomLeftSpeed));
    analogWrite(BOTTOM_RIGHT_R_PWM, validateSpeedValue(bottomRightSpeed));
  }
  else {
    // In reverse mode, we will use the L_PWM pins for backward motion
    analogWrite(TOP_LEFT_R_PWM, 0);
    analogWrite(TOP_RIGHT_R_PWM, 0);
    analogWrite(BOTTOM_LEFT_R_PWM, 0);
    analogWrite(BOTTOM_RIGHT_R_PWM, 0);

    // Set the PWM values for backward motion
    digitalWrite(TOP_LEFT_L_PWM, HIGH);
    digitalWrite(TOP_RIGHT_L_PWM, HIGH);
    digitalWrite(BOTTOM_LEFT_L_PWM, HIGH);
    digitalWrite(BOTTOM_RIGHT_L_PWM, HIGH);
  }
}

int validateSpeedValue(int speed) {
  // This function ensures the speed value is within the valid range of 0 to 255
  return constrain(speed, 0, 255);
}
