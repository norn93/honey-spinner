#include <HX711.h>

#include <SPI.h>
#include "RF24.h"

#define STEPPER_RESET 2
#define STEPPER_DIR_PIN 3
#define STEPPER_STEP_PIN 4
#define STEPPER_MS1 6
#define STEPPER_MS2 5

#define RADIO_1 7 //TODO, name these
#define RADIO_2 8

#define LOAD_CELL_DT_PIN A4
#define LOAD_CELL_SCK_PIN A3

#define LIMIT_UPPER_PIN A0
#define LIMIT_LOWER_PIN A1

// Custom data types
union mixedValue {
  bool asBool;
  char asChar;
  int asInt;
  float asFloat;
  long int asLongInt;
};

struct parameterPair {
  char id;
  mixedValue value;
};

// 0 = 1Node is Parent
// 1 = 2Node is Child
// 2 = stepper practice
// 3 = load cell practice
// 4 = calibration practice
byte addresses[][6] = {"1Node", "2Node"};
int radioNumber = 1;

// STEPPER TIMING
unsigned long stepperLastStep; //last time we stepped
unsigned long stepperLastUpdate; //last time we updated accel
long int stepperDelay; //how long do we wait between stepper pulses

// STEPPER STATE
bool stepperState; //is the stepper at a 1 or 0?
bool stepperArrived = false; //are we at our destination?

// STEPPER OPTIONS
long int stepperVelocityUpdateInterval = 10000; //how often we update our velocity calculation
bool stepperInvertDirection = true; //which way do we want to be positive/negative
int stepperMicroStepping = 1; //microstepping factor 1/n
float stepperMaxSpeed = 0.005; //m/s 0.005 is more comfortable
float stepperMaxAcceleration = 0.04; //m/s^2 //try 0.04
float stepperMinSpeed = 0.0001; //speed to below to allow up to have arrived
float stepperSmoothingFactor = 1.2; //fudge factor to prevent overshoot
bool stepperResetOnStop = true; // false if you want holding force
float stepperMaximumPosition = 1.0; // maximum position in metres
float stepperMinimumPosition = 0.0; // minimum position in metres

// STEPPER INITIAL CONDITIONS
float stepperPositionMetres = 0.0; //m
float stepperVelocity = 0.0; //m/s
float stepperAcceleration = 0.0; //m/s^2

// STEPPER RATIOS
float stepperStepsPerRevolution = 200 * stepperMicroStepping; //how many steps it takes to do a revolution
float stepperMetresPerRevolution = 0.002; // pitch of the screw

// STEPPER SETPOINT
float stepperSetpoint; //setpoint position in meters
long int setpointSteps; //out current setpoint position in steps
long int stepperPositionSteps; //the current position of the stepper in steps

// STEPPER CALIBRATION
char stepperCalibrationState = 'n';
// n not calibrated, starting position
// s state variable enabled
// c calibrated
bool stepperCalibrationMovementDirection = false;
// false is towards the low
// true is towards the high
float stepperCalibrationClearance = 0.001; //m

// LOAD CELL
const int loadCellBufferSize = 8;
long int loadCellBuffer[loadCellBufferSize];

// Radio
RF24 radio(RADIO_1, RADIO_2);

// Load cell
HX711 loadcell;

void setup() {
  // Get everything going
  Serial.begin(115200);
  radio.begin();

  // Set the radio to low
  radio.setPALevel(RF24_PA_LOW);

  // Open a writing and reading pipe on each radio, with opposite addresses
  if (radioNumber) {
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
  } else {
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
  }

  // Start the radio listening for data
  radio.startListening();

  // Check the chip is connected
  if (radio.isChipConnected()) {
    Serial.println("Connected to radio!");
  } else {
    Serial.println("Not connected...");
    while (1) {}
  }

  if (radioNumber == 1 || radioNumber == 2 || radioNumber == 4) {
    setUpStepper();
  }

  if (radioNumber == 1 || radioNumber == 3) {
    loadcell.begin(LOAD_CELL_DT_PIN, LOAD_CELL_SCK_PIN);
    loadcell.tare();
    // Optionally perform other calibration
  }

  if (radioNumber == 1 || radioNumber == 4) {
    // Set up limit switches
    pinMode(LIMIT_UPPER_PIN, INPUT);
    pinMode(LIMIT_LOWER_PIN, INPUT);
    digitalWrite(LIMIT_LOWER_PIN, HIGH);
    digitalWrite(LIMIT_UPPER_PIN, HIGH);
  }
}

void loop() {
  if (radioNumber == 0) {

    // First, stop listening so we can talk.
    radio.stopListening();

    parameterPair sendingParam;

    sendingParam.id = 'C';
    // Remember to check type of variable
    sendingParam.value.asFloat = 0.0009;

    if (!radio.write(&sendingParam, sizeof(sendingParam))) {
      Serial.println("Sending data failed");
    }

    switch (sendingParam.id) {
      case 'L':
        getLoadCellData();
        Serial.println("Got load cell data!");
        break;
    }

    // Delay so we don't spam requests
    delay(1000);

  } else if (radioNumber == 1) {

    handleCalibration();

    handleStepper();

    handleLoadCell();

    handleRadio();

  } else if (radioNumber == 2) {

    handleStepper();

  } else if (radioNumber == 3) {

    handleLoadCell();

  } else if (radioNumber == 4) {

    handleCalibration();

    handleStepper();

  }
}

void setUpStepper() {
  // Set up the stepper
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_MS1, OUTPUT);
  pinMode(STEPPER_MS2, OUTPUT);
  pinMode(STEPPER_RESET, OUTPUT);
  digitalWrite(STEPPER_STEP_PIN, LOW);
  digitalWrite(STEPPER_DIR_PIN, HIGH);
  digitalWrite(STEPPER_RESET, HIGH); // Begin with stepper disabled
  // full step (0,0), half step (1,0), 1/4 step (0,1), and 1/8 step (1,1 : default).
  if (stepperMicroStepping == 1) {
    digitalWrite(STEPPER_MS1, LOW);
    digitalWrite(STEPPER_MS2, LOW);
  } else if (stepperMicroStepping == 2) {
    digitalWrite(STEPPER_MS1, HIGH);
    digitalWrite(STEPPER_MS2, LOW);
  } else if (stepperMicroStepping == 4) {
    digitalWrite(STEPPER_MS1, LOW);
    digitalWrite(STEPPER_MS2, HIGH);
  } else if (stepperMicroStepping == 8) {
    digitalWrite(STEPPER_MS1, HIGH);
    digitalWrite(STEPPER_MS2, HIGH);
  } else {
    Serial.println("Invalid microstepping");
    while (1) {}
  }
}

void setStepperPosition(float setpoint) {

  // Clamp the setpoint to bounds if we are calibrated
  if (stepperCalibrationState == 'c') {
    setpoint = min(max(stepperMinimumPosition, setpoint), stepperMaximumPosition);
  }

  // Set the current setpoint for the motor
  stepperArrived = false;

  // Enable the stepper
  digitalWrite(STEPPER_RESET, LOW);

  if (setpoint != stepperSetpoint) { //only update the setpoint if it's different
    stepperSetpoint = setpoint;
    setpointSteps = round(stepperSetpoint / (stepperMetresPerRevolution / stepperStepsPerRevolution));
  } else {
    return;
  }
}

void handleStepper() {
  //Move the stepper according to the algorithm

  // Check if we've arrived
  if (stepperPositionSteps == setpointSteps && stepperVelocity < stepperMinSpeed) {
    if (stepperResetOnStop) {
      digitalWrite(STEPPER_RESET, HIGH);
    }
    stepperArrived = true;
  }

  // If we've already arrrived, return
  if (stepperArrived == true) {
    return;
  }

  // Update derived parameters
  stepperStepsPerRevolution = 200 * stepperMicroStepping;

  // Calculate current position in m
  stepperPositionMetres = stepperPositionSteps * stepperMetresPerRevolution / stepperStepsPerRevolution;

  // Calculate required direction
  int stepperDirection = 2 * (stepperVelocity < 0 && stepperAcceleration > 0 || stepperVelocity > 0 && stepperAcceleration < 0) - 1;

  // Calculate time until we could stop
  float stepperTimeUntilStop;
  if (stepperAcceleration != 0) {
    stepperTimeUntilStop = -stepperVelocity / stepperAcceleration * stepperDirection;
  } else {
    stepperTimeUntilStop = -stepperVelocity / 0.0000000001 * stepperDirection;
  }

  // Calculate displacement until we could stop
  float stepperDisplacementUntilStop = stepperPositionMetres + (stepperVelocity * stepperSmoothingFactor) * stepperTimeUntilStop + 0.5 * stepperAcceleration * stepperDirection * pow(stepperTimeUntilStop, 2);

  // Calculate new acceleration to steer us in the correct direction
  stepperAcceleration = stepperMaxAcceleration * (2 * (stepperDisplacementUntilStop < stepperSetpoint) - 1);

  // Measure the time/s
  unsigned long currentTime = micros();
  unsigned long timeDifferenceStep = currentTime - stepperLastStep;
  unsigned long timeDifferenceAccel = currentTime - stepperLastUpdate;

  // Update the velocity, if necessary
  float stepperAngularVelocity;
  if (timeDifferenceAccel > stepperVelocityUpdateInterval) {
    stepperVelocity += stepperAcceleration * ((float)stepperVelocityUpdateInterval / 1000000);
    // Clamp the velocity
    stepperVelocity = min(max(-stepperMaxSpeed, stepperVelocity), stepperMaxSpeed);
    stepperAngularVelocity = 2 * M_PI * stepperVelocity / stepperMetresPerRevolution;
    float stepperDelayFloat = 1000000 * abs(2 * M_PI / stepperStepsPerRevolution / stepperAngularVelocity / 2);
    stepperDelay = round(stepperDelayFloat);
    stepperLastUpdate = currentTime;
  }

  // Update the stepper step, if required
  if (timeDifferenceStep > stepperDelay) {
    stepperState = !stepperState;
    // Check motor direction
    if (stepperVelocity > 0) {
      digitalWrite(STEPPER_DIR_PIN, stepperInvertDirection);
    } else {
      digitalWrite(STEPPER_DIR_PIN, !stepperInvertDirection);
    }
    digitalWrite(STEPPER_STEP_PIN, stepperState);
    stepperLastStep = currentTime;
    // Update actual stepper position
    if (stepperState) {
      stepperPositionSteps += (2 * (stepperVelocity > 0)) - 1;
    }
  }
}

void handleLoadCell() {

  if (loadcell.is_ready()) {
    long reading = loadcell.read();
    //Serial.println(reading);
    addToLoadCellBuffer(reading);
  } else {
    // Do nothing
  }
}

void addToLoadCellBuffer(long int value) {
  for (int i = loadCellBufferSize - 1; i > 0; i--) {
    loadCellBuffer[i] = loadCellBuffer[i - 1];
  }
  loadCellBuffer[0] = value;
}

void sendLoadCellData() {
  // First, stop listening so we can talk.
  radio.stopListening();

  if (!radio.write(&loadCellBuffer, sizeof(loadCellBuffer))) {
    Serial.println("Sending load cell data failed");
  }

  // Back to how we were
  radio.startListening();
}

void getLoadCellData() {
  // First, stop listening so we can talk.
  radio.startListening();

  if (radio.available()) {
    while (radio.available()) {
      radio.read(&loadCellBuffer, sizeof(loadCellBuffer));
    }
  }
}

void printLoadCellBuffer() {
  for (int i = 0; i < loadCellBufferSize; i++) {
    Serial.print(loadCellBuffer[i]);
    Serial.print(", ");
  }
  Serial.println("");
}

void handleRadio() {

  // Start the radio listening for data
  radio.startListening();

  parameterPair recievedParam;

  if (radio.available()) {
    while (radio.available()) {
      radio.read(&recievedParam, sizeof(recievedParam));
    }

    Serial.print("Recieved instruction id: ");
    Serial.println(recievedParam.id);

    switch (recievedParam.id) {
      case 'u':
        stepperVelocityUpdateInterval = recievedParam.value.asLongInt;
        break;
      case 'i':
        stepperInvertDirection = recievedParam.value.asBool;
        break;
      case 'm':
        stepperMicroStepping = recievedParam.value.asInt;
        break;
      case 'v':
        stepperMaxSpeed = recievedParam.value.asFloat;
        break;
      case 's':
        setStepperPosition(recievedParam.value.asFloat);
        break;
      case 'a':
        stepperMaxAcceleration = recievedParam.value.asFloat;
        break;
      case 't':
        stepperMinSpeed = recievedParam.value.asFloat;
        break;
      case 'f':
        stepperSmoothingFactor = recievedParam.value.asFloat;
        break;
      case 'r':
        stepperResetOnStop = recievedParam.value.asBool;
        break;
      case '1':
        stepperMaximumPosition = recievedParam.value.asFloat;
        break;
      case '0':
        stepperMinimumPosition = recievedParam.value.asFloat;
        break;
      case 'p':
        stepperMetresPerRevolution = recievedParam.value.asFloat;
        break;

      case 'L':
        sendLoadCellData();
        break;

      case 'C':
        if (stepperCalibrationState == 'c') {
          stepperCalibrationState = 'n';
        }
        break;

      case 'c':
        stepperCalibrationClearance = recievedParam.value.asFloat;
        break;
    }
  }
}

void handleCalibration() {

  if (stepperCalibrationState == 'c') {
    return; // Do nothing
  }

  bool upper = !digitalRead(LIMIT_UPPER_PIN);
  bool lower = !digitalRead(LIMIT_LOWER_PIN);

  // first, check where we are
  if (lower) {
    // We're at the low side
    //Serial.println("Low side");
    stepperVelocity = 0;
    stepperPositionSteps = 0;
    stepperMinimumPosition = stepperCalibrationClearance;
    setStepperPosition(1);
    stepperCalibrationState = 's';
    stepperCalibrationMovementDirection = true;
  } else if (upper) {
    //We're at the high side
    //Serial.println("High side");
    stepperVelocity = 0;
    if (stepperCalibrationState == 's') {
      // Then we're done, go to the middle
      //Serial.println("Done, going to middle");
      stepperMaximumPosition = stepperPositionMetres - stepperCalibrationClearance;
      stepperCalibrationState = 'c';
      stepperCalibrationMovementDirection = false;
      setStepperPosition((stepperMaximumPosition - stepperMinimumPosition) / 2);
    } else {
      // We need to go the low side first
      // but we also need to reverse the direction of the motor,
      // since we went the wrong way
      //Serial.println("Got to the wrong side, going back to the low side");
      stepperInvertDirection = !stepperInvertDirection;
      stepperCalibrationMovementDirection = false;
      setStepperPosition(-1);
    }
  } else {
    // We don't know where we are
    if (stepperCalibrationMovementDirection) {
      setStepperPosition(1);
    } else {
      setStepperPosition(-1);
    }
  }
}
