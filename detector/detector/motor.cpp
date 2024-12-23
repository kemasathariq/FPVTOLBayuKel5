#include <AccelStepper.h>

// Pin definitions for the stepper motors
const int stepPinX = 2;
const int dirPinX = 5;
const int stepPinY = 3;
const int dirPinY = 6;
const int stepPinZ = 4;  // Step pin for Z-axis motor
const int dirPinZ = 7;  // Direction pin for Z-axis motor
const int enablePin = 8; // ENABLE pin for the motor driver

// Define the maximum speeds and accelerations
const int maxSpeedX = 800;  // Maximum speed for X-axis motor
const int maxSpeedY = 800;  // Maximum speed for Y-axis motor
const int maxSpeedZ = 800;  // Maximum speed for Z-axis motor
const int accelerationX = 400;  // Acceleration for X-axis motor
const int accelerationY = 400;  // Acceleration for Y-axis motor
const int accelerationZ = 400;  // Acceleration for Z-axis motor

// Define steps per cm for X and Y axes
const float stepsPerCmX = 50.0;  // Steps per cm for X-axis
const float stepsPerCmY = 29.0;  // Steps per cm for Y-axis

// Define boundaries for the axes (in steps)
const int maxStepsX = 2500;  // Maximum steps for X-axis (50 cm)
const int maxStepsY = 1450;  // Maximum steps for Y-axis (50 cm)
const int maxStepsZ = 400;   // Maximum steps for Z-axis movement

// Define grid parameters
const int gridSpacingCm = 5;  // Spacing between points in the grid (cm)
const int gridSpacingStepsX = gridSpacingCm * stepsPerCmX;  // Spacing in steps for X-axis
const int gridSpacingStepsY = gridSpacingCm * stepsPerCmY;  // Spacing in steps for Y-axis
const int gridMaxX = maxStepsX / gridSpacingStepsX;  // Number of points in X direction
const int gridMaxY = maxStepsY / gridSpacingStepsY;  // Number of points in Y direction

// Create AccelStepper objects for each motor
AccelStepper stepperX(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper stepperY(AccelStepper::DRIVER, stepPinY, dirPinY);
AccelStepper stepperZ(AccelStepper::DRIVER, stepPinZ, dirPinZ);

void setup() {
  Serial.begin(9600);  // For debugging
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);  // Enable motor driver

  // Configure stepper motor parameters
  stepperX.setMaxSpeed(maxSpeedX);
  stepperX.setAcceleration(accelerationX);
  stepperY.setMaxSpeed(maxSpeedY);
  stepperY.setAcceleration(accelerationY);
  stepperZ.setMaxSpeed(maxSpeedZ);
  stepperZ.setAcceleration(accelerationZ);

  Serial.println("Stepper system ready");
}

void loop() {
  for (int yIndex = 0; yIndex <= gridMaxY; yIndex++) {
    for (int xIndex = 0; xIndex <= gridMaxX; xIndex++) {
      // Calculate the target positions in steps
      int targetX_steps = xIndex * gridSpacingStepsX;
      int targetY_steps = yIndex * gridSpacingStepsY;

      // Move X and Y to the target position
      stepperX.moveTo(targetX_steps);
      stepperY.moveTo(targetY_steps);

      while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
        stepperX.run();
        stepperY.run();

        // Debugging
        Serial.print("Moving to X: ");
        Serial.print(stepperX.currentPosition() / stepsPerCmX);
        Serial.print(" cm, Y: ");
        Serial.println(stepperY.currentPosition() / stepsPerCmY);
      }

      Serial.println("XY Position reached");

      // Move Z motor down
      stepperZ.moveTo(maxStepsZ);
      while (stepperZ.distanceToGo() != 0) {
        stepperZ.run();
      }
      Serial.println("Z Motor down");

      // Move Z motor back up
      stepperZ.moveTo(0);
      while (stepperZ.distanceToGo() != 0) {
        stepperZ.run();
      }
      Serial.println("Z Motor up");
    }
  }

  // After scanning the entire area, return to home position
  stepperX.moveTo(0);
  stepperY.moveTo(0);
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.run();
    stepperY.run();
  }
  Serial.println("Returned to home position");

  delay(5000);  // Optional delay before repeating the scan
}