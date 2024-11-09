// Mike Murray 2024-11-08
// OMRON XY Stage Code
// Define motor and limit switch pins
const int xEnablePin = 9;
const int xDirPin = 10;
const int xPulsePin = 11;

const int yEnablePin = 12;
const int yDirPin = 13;
const int yPulsePin = 14;

const int xMinPin = 5;
const int xMaxPin = 4;
const int yMinPin = 7;
const int yMaxPin = 6;

const int emergencyStopPin = 31;

const float lead_screw_pitch = 2.0;
const int motor_steps_per_rev = 200;
const float microsteps = 1.0;
const float steps_per_mm = (motor_steps_per_rev) / (4.0 * lead_screw_pitch);

#include <AccelStepper.h>
AccelStepper stepperX(AccelStepper::DRIVER, xPulsePin, xDirPin);
AccelStepper stepperY(AccelStepper::DRIVER, yPulsePin, yDirPin);

float X1 = 90;
float Y1 = -57;
const float traySpacing = -25.4; // Adjust this if Y should go up
const int numRows = 10;
const int numCols = 2;

//variables for the amount of time to wait before changing position in grid routine
float wait1 = 10 * 1000;
float wait2 = 5 * 1000;

enum State {
  IDLE,
  HOMING,
  GRID_ROUTINE,
  ENDING
};

State currentState = IDLE;

void setup() {
  Serial.begin(9600);

  pinMode(xMinPin, INPUT_PULLUP);
  pinMode(xMaxPin, INPUT_PULLUP);
  pinMode(yMinPin, INPUT_PULLUP);
  pinMode(yMaxPin, INPUT_PULLUP);

  pinMode(xEnablePin, OUTPUT);
  pinMode(yEnablePin, OUTPUT);
  digitalWrite(xEnablePin, LOW);
  digitalWrite(yEnablePin, LOW);

  pinMode(emergencyStopPin, INPUT_PULLUP);

  stepperX.setMaxSpeed(500);
  stepperX.setAcceleration(500);
  stepperY.setMaxSpeed(500);
  stepperY.setAcceleration(500);

  homeMotors();
}

void loop() {
  // Check for emergency stop
  if (isEmergencyStopActivated()) {
    handleEmergencyStop();
    return;
  }

  // Execute current state
  switch (currentState) {
    case IDLE:
      // Prompt for next action
      promptForNextAction();
      break;

    case HOMING:
      homeMotors();
      currentState = IDLE; // Go back to idle after homing
      break;

    case GRID_ROUTINE:
      gridRoutine();
      currentState = IDLE; // Go back to idle after the grid routine
      break;


    case ENDING:
      endRoutine();
      currentState = IDLE; // Go back to idle after ending
      break;
    default:
      break;
  }
}

bool isEmergencyStopActivated() {
  return digitalRead(emergencyStopPin) == LOW;
}

void handleEmergencyStop() {
  stopMotors();
  Serial.println("Emergency stop activated. Please reset.");
  while (isEmergencyStopActivated()); // Wait until the emergency stop is reset
}

void stopMotors() {
  digitalWrite(xEnablePin, HIGH);
  digitalWrite(yEnablePin, HIGH);
  stepperX.stop();
  stepperY.stop();
  if (currentState == ENDING){
    Serial.println("Motors stopped.");
  }else{
    Serial.println("Motors stopped due to emergency stop.");
  }
}

void checkLimits() {
  if (digitalRead(xMinPin) == LOW || digitalRead(xMaxPin) == LOW || 
      digitalRead(yMinPin) == LOW || digitalRead(yMaxPin) == LOW) {
    stopMotors();
  }
}

void moveToPosition(float xPos, float yPos) {
  Serial.print("Moving to position: X=");
  Serial.print(xPos);
  Serial.print(", Y=");
  Serial.println(yPos);
  
  // Adjust these signs based on your hardware configuration
  stepperX.moveTo(xPos * steps_per_mm);  // Keep X as positive if moving right
  stepperY.moveTo(yPos * steps_per_mm);    // Change sign here if Y needs to move up

  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    if (isEmergencyStopActivated()) {
      stopMotors();
      return;
    }
    stepperX.run();
    stepperY.run();
    checkLimits();
  }
  
  Serial.print("Reached position: X=");
  Serial.print(xPos);
  Serial.print(" Y=");
  Serial.println(yPos);
}

void promptForNextAction() {
  Serial.println("Choose action: (H)ome, (G)rid Routine, (E)nd");

  unsigned long startMillis = millis(); // Get the current time for timeout
  bool inputReceived = false;
  
  // Set timeout duration
  unsigned long timeoutDuration = 120000; // 2 minutes (adjust as needed)

  while (!inputReceived && millis() - startMillis < timeoutDuration) {  // Wait for the specified timeout period
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 'H' || input == 'h') {
        currentState = HOMING; // Change state to homing
        inputReceived = true;  // Break the loop after valid input
      } else if (input == 'G' || input == 'g') {
        currentState = GRID_ROUTINE; // Change state to grid routine
        inputReceived = true;  // Break the loop after valid input
      } else if (input == 'E' || input == 'e') {
        currentState = ENDING; // Change state to ending
        inputReceived = true;  // Break the loop after valid input
      } else {
        Serial.println("Invalid input. Please enter 'H', 'G', or 'E'.");
      }
    }
  }

  // After timeout, only print "Returning to idle" once and switch to IDLE
  if (!inputReceived) {
    Serial.println("No valid input received within timeout period. Returning to idle.");
    currentState = IDLE;  // Default to idle if no input after timeout
  }
}

void gridRoutine() {
  float currentX1 = X1; // Store the current X1 value
  for (int col = 1; col <= numCols; col++) {

    for (int row = 1; row <= numRows; row++) {
      float modifiedY =  Y1 + (float)row * traySpacing;
      moveToPosition(currentX1, modifiedY);
      Serial.print("Column ");
      Serial.print(col);
      Serial.print(", ");
      Serial.println(row);
      delay(wait2);
    }

    currentX1 += 152.4; // Update X1 for the next column
    delay(wait2);

  }
  Serial.println("Grid Routine Completed. Moving to Loading Position.");
  moveToPosition(currentX1, Y1); // Move back to the original position
}

void loadNextPlate() {
  Serial.println("Loading next plate...");
  moveToPosition(X1, Y1); // Move to the desired position for loading
}


void endRoutine() {
  Serial.println("Ending operations...");
  stopMotors();
}

void homeMotors() {
  Serial.println("Starting homing routine...");
  
  // Homing logic for X axis
  stepperX.setSpeed(-300);
  while (digitalRead(xMinPin) != LOW) {
    stepperX.runSpeed();
    if (isEmergencyStopActivated()) {
      stopMotors();
      return;
    }
  }
  stepperX.setCurrentPosition(0);
  Serial.println("X axis homed.");

  // Backup from the limit switch
  stepperX.move(5 * steps_per_mm);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
    if (isEmergencyStopActivated()) {
      stopMotors();
      return;
    }
  }

  // Homing logic for Y axis
  stepperY.setSpeed(300);
  while (digitalRead(yMinPin) != LOW) {
    stepperY.runSpeed();
    if (isEmergencyStopActivated()) {
      stopMotors();
      return;
    }
  }
  stepperY.setCurrentPosition(0);
  Serial.println("Y axis homed.");

  // Backup from the limit switch
  stepperY.move(-5 * steps_per_mm);
  while (stepperY.distanceToGo() != 0) {
    stepperY.run();
    if (isEmergencyStopActivated()) {
      stopMotors();
      return;
    }
  }
  Serial.println("Homing complete.");
}
