#include <HX711.h>

#include <SPI.h>
#include "RF24.h"

#define STEPPER_RESET 2
#define STEPPER_DIR_PIN 3
#define STEPPER_STEP_PIN 4
#define STEPPER_MS1 6
#define STEPPER_MS2 5

#define LOAD_CELL_DT_PIN A4
#define LOAD_CELL_SCK_PIN A3

#define PAYLOAD_SIZE 4

// 0 = 1Node is Parent
// 1 = 2Node is Child
// 2 = stepper practice
// 3 = load cell practice
byte addresses[][6] = {"1Node", "2Node"};
int radioNumber = 1;

// STEPPER TIMING
unsigned long stepperLastStep; //last time we stepped
unsigned long stepperLastUpdate; //last time we updated accel
long int stepperVelocityUpdateInterval = 10000; //how often we update our velocity calculation
long int stepperDelay; //how long do we wait between stepper pulses

// STEPPER STATE
bool stepperState; //is the stepper at a 1 or 0?
bool stepperArrived = false; //are we at our destination?

// STEPPER OPTIONS
bool stepperInvertDirection = true; //which way do we want to be positive/negative
int stepperMicroStepping = 1; //microstepping factor 1/n
float stepperMaxSpeed = 0.005; //m/s
float stepperMaxAcceleration = 0.04; //m/s^2
float stepperMinSpeed = 0.0001; //speed to below to allow up to have arrived
float stepperSmoothingFactor = 1.2; //fudge factor to prevent overshoot
bool stepperResetOnStop = true; // false if you want holding force
float stepperMaximumPosition = 1.0; // maximum position in metres
float stepperMinimumPosition = 0.0; // minimum position in metres

// STEPPER INITIAL CONDITIONS
float stepperPositionMetres = 0.87; //m
float stepperVelocity = 0.0; //m/s
float stepperAcceleration = 0.0; //m/s^2

// STEPPER RATIOS
float stepperStepsPerRevolution = 200 * stepperMicroStepping; //how many steps it takes to do a revolution
float stepperMetresPerRevolution = 0.002; // pitch of the screw

// STEPPER SETPOINT
float stepperSetpoint; //setpoint position in meters
long int setpointSteps; //out current setpoint position in steps
long int stepperPositionSteps = round(stepperPositionMetres * stepperStepsPerRevolution / stepperMetresPerRevolution); //the current position of the stepper in steps

void setStepperPosition(float setpoint) {

  // Clamp the setpoint to bounds
  setpoint = min(max(stepperMinimumPosition, setpoint), stepperMaximumPosition);
  
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

void updateStepper() {
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

// Serial input for parent
char inChar = 'a';
char sentChar;

// Radio
RF24 radio(7, 8);

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

  if (radioNumber == 1 || radioNumber == 2) {
    // Set up the stepper
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_MS1, OUTPUT);
    pinMode(STEPPER_MS2, OUTPUT);
    pinMode(STEPPER_RESET, OUTPUT);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    digitalWrite(STEPPER_DIR_PIN, HIGH);
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
    digitalWrite(STEPPER_RESET, LOW);
    Serial.println("Steppers enabled!");

    // Set the first setpoint to the current position
    setStepperPosition(stepperPositionMetres);
  }

  if (radioNumber == 1 || radioNumber == 3) {
    loadcell.begin(LOAD_CELL_DT_PIN, LOAD_CELL_SCK_PIN);
    //    Serial.println(micros());
    //    Serial.println(micros());
    //    Serial.println(loadcell.get_units(10), 2);
    //    Serial.println(micros());
  }
}

void loop() {
  if (radioNumber == 0) {

    if (sentChar != inChar) {
      sentChar = inChar;

      // First, stop listening so we can talk.
      radio.stopListening();

      // Take the time, and send it.  This will block until complete
      Serial.println("Now sending");
      float setpoint = stepperPositionMetres;

      union {
        byte asBytes[4];
        float asFloat;
      } data;

      data.asFloat = setpoint;

      // Temporary hack to make the position update in a useful way
      data.asBytes[2] = inChar;

      if (!radio.write(&data.asBytes, sizeof(data.asBytes))) {
        Serial.println("failed");
      }

      // Serial input
      Serial.print("inChar: ");
      Serial.println(inChar);
      Serial.println(data.asFloat, 10);

    }

  } else if (radioNumber == 1) {

    updateStepper();

    byte receivedData[PAYLOAD_SIZE];

    union {
      byte asBytes[4];
      float asFloat;
    } data;

    if ( radio.available()) {
      // Variable for the received timestamp
      while (radio.available()) {
        radio.read(&receivedData, 4);
      }

      data.asBytes[0] = receivedData[0];
      data.asBytes[1] = receivedData[1];
      data.asBytes[2] = receivedData[2];
      data.asBytes[3] = receivedData[3];

      Serial.println(data.asFloat, 10);

      setStepperPosition(data.asFloat);
    }

  } else if (radioNumber == 2) {

    updateStepper();

  } else if (radioNumber == 3) {
    if (loadcell.is_ready()) {
      long reading = loadcell.read();
      //Serial.print("HX711 reading: ");
      Serial.println(sizeof(reading));
      Serial.println(reading);
    } else {
      //Serial.println("HX711 not found.");
    }
  }
}

void serialEvent() {
  //Serial.println("got serial");
  if (radioNumber == 0) {
    while (Serial.available()) {
      // get the new byte:
      char lastChar = inChar;
      inChar = (char)Serial.read();

      if (inChar == '\n') {
        inChar = lastChar;
      }
    }
  }
}
