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

#define PAYLOAD_SIZE 6

// 0 = 1Node is Parent
// 1 = 2Node is Child
// 2 = stepper practice
// 3 = load cell practice
byte addresses[][6] = {"1Node", "2Node"};
int radioNumber = 2;

// STEPPER
// The current state of the stepper
//#define STEPPER_MIN 0.005
//#define STEPPER_MAX 0.995
//#define STEPPER_ACCELERATION_PROPORTION 0.2
//#define STEPPER_DECCELERATION_PROPORTION 0.2
//#define STEPPER_MAX_SPEED 0.005
//#define STEPPER_MAX_ACCELERATION 0.001
//#define STEPPER_STEPS_PER_REV 200
bool stepperState;
unsigned long stepperLastStep; //last time we stepped
unsigned long stepperLastAccel; //last time we updated accel
float stepperPosition = 0.5; //the current position of the stepper in m
float stepperMaxSpeed = 0.005;
float stepperMinSpeed = 0.0001;
float stepperVelocity = 0.0; //m/s
float stepperAcceleration = 0.001; //m/s^2
float stepperMetresPerRevolution = 0.002;
//float stepperAngularVelocity = 2 * M_PI * stepperVelocity / stepperMetresPerRevolution;
//float stepperAngularAcceleration = 2 * M_PI * stepperAcceleration / stepperMetresPerRevolution;
int stepperStepsPerRevolution = 200;
//int stepperDelay = 2 * M_PI / stepperStepsPerRevolution * 1000000 / stepperAngularVelocity / 2; //us
long int stepperDelay = 100000;

void setStepperPosition(float setpoint) {
  float position_delta = setpoint - stepperPosition;
  Serial.print("Position delta: ");
  Serial.println(position_delta);

  int movement_direction = 2 * (position_delta > 0) - 1;
  Serial.print("movement_direction: ");
  Serial.println(movement_direction);

  if (movement_direction == 1) { //positive movement is CW
    digitalWrite(STEPPER_DIR_PIN, LOW);
  } else {
    digitalWrite(STEPPER_DIR_PIN, HIGH);
  }

  float steps_per_metre = stepperStepsPerRevolution / stepperMetresPerRevolution;
  Serial.print("Steps per m: ");
  Serial.println(steps_per_metre);

  long int step_delta = round(steps_per_metre * position_delta);
  Serial.print("Steps delta: ");
  Serial.println(step_delta);

  //We need to check which case we're in: is the ramp up and down going to take longer than
  //the full movement? Assuming the same acce/decell, is the ramp longer than half the movement?
  //time for ramp up = max_speed/accel;
  float ramp_time = stepperMaxSpeed / stepperAcceleration;

  // s = 1/2 a t^2
  float ramp_distance = 0.5 * stepperAcceleration * ramp_time * ramp_time;

  if (2 * ramp_distance > abs(position_delta)) {
    Serial.println("We won't make it to full speed!");

    //adjust ramp_distance
    ramp_distance = abs(position_delta) / 2;

    ramp_time = sqrt(2 * ramp_distance / stepperAcceleration);
  }

  Serial.print("Ramp time (s): ");
  Serial.println(ramp_time);

  Serial.print("Ramp distance (m): ");
  Serial.println(ramp_distance, 10);

  long int ramp_steps = round(steps_per_metre * ramp_distance);

  Serial.print("ramp_steps: ");
  Serial.println(ramp_steps);

  long int stop_accel_step = 0 + movement_direction * ramp_steps;
  long int start_deccel_step = step_delta - movement_direction * ramp_steps;

  Serial.print("stop_accel_step: ");
  Serial.println(stop_accel_step);
  Serial.print("start_deccel_step: ");
  Serial.println(start_deccel_step);

  long int velocity_update = 10000;

  float velocity_increment = stepperAcceleration / (1000000 / velocity_update);
  Serial.print("velocity_increment: ");
  Serial.println(velocity_increment, 10);

  long int current_step = 0;
  while (current_step != step_delta) {
    unsigned long currentTime = micros();
    unsigned long timeDifferenceStep = currentTime - stepperLastStep;
    unsigned long timeDifferenceAccel = currentTime - stepperLastAccel;

    float stepperAngularVelocity = 2 * M_PI * stepperVelocity / stepperMetresPerRevolution;

    if (timeDifferenceAccel > velocity_update) {
      if (abs(current_step) < abs(stop_accel_step)) {
        stepperVelocity += movement_direction * velocity_increment;
        stepperLastAccel = micros();
      } else if (abs(current_step) > abs(start_deccel_step)) {
        stepperVelocity -= movement_direction * velocity_increment;
        if (abs(stepperVelocity) < stepperMinSpeed) {
          stepperVelocity = movement_direction * stepperMinSpeed;
        }
        stepperLastAccel = micros();
      }
    }

    stepperAngularVelocity = 2 * M_PI * stepperVelocity / stepperMetresPerRevolution;
    stepperDelay = abs(2 * M_PI / stepperStepsPerRevolution * 1000000 / stepperAngularVelocity / 2);

    bool moving; //are we moving?
    if (stepperVelocity != 0) {
      moving = true;
    } else {
      moving = false;
    }

    if (moving && timeDifferenceStep > stepperDelay) {
      stepperState = !stepperState;
      digitalWrite(STEPPER_STEP_PIN, stepperState);
      stepperLastStep = micros();
      if (stepperState) {
        current_step += movement_direction;
      }
    }
  }

  return;
}

// Serial input for parent
char inChar;

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
    Serial.println("Connected!");
  } else {
    Serial.println("Not connected...");
    while (1) {}
  }

  if (radioNumber == 1 || radioNumber == 2) {
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_MS1, OUTPUT);
    pinMode(STEPPER_MS2, OUTPUT);
    pinMode(STEPPER_RESET, OUTPUT);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    digitalWrite(STEPPER_DIR_PIN, HIGH);
    // Full stepping mode
    // full step (0,0), half step (1,0), 1/4 step (0,1), and 1/8 step (1,1 : default).
    digitalWrite(STEPPER_MS1, LOW);
    digitalWrite(STEPPER_MS2, LOW);
    digitalWrite(STEPPER_RESET, LOW);
    Serial.println("Steppers enabled!");
    Serial.print("stepperDelay: ");
    Serial.println(stepperDelay);
  }

  if (radioNumber == 1 || radioNumber == 3) {
    loadcell.begin(LOAD_CELL_DT_PIN, LOAD_CELL_SCK_PIN);
    Serial.println(micros());
    Serial.println(micros());
    Serial.println(loadcell.get_units(10), 2);
    Serial.println(micros());
  }
}

void loop() {
  if (radioNumber == 0) {
    // First, stop listening so we can talk.
    radio.stopListening();

    // Take the time, and send it.  This will block until complete
    Serial.println("Now sending");
    unsigned long start_time = micros();

    //    Serial.println("Unpacking an unsigned long example:");
    //    char byte1, byte2, byte3, byte4;
    //    byte1 = start_time;
    //    byte2 = start_time >> 8;
    //    byte3 = start_time >> 16;
    //    byte4 = start_time >> 24;
    //    Serial.println((unsigned char)byte1);
    //    Serial.println((unsigned char)byte2);
    //    Serial.println((unsigned char)byte3);
    //    Serial.println((unsigned char)byte4);

    // Pack up data
    byte data[PAYLOAD_SIZE];
    data[0] = start_time >> 8 * 0;
    data[1] = start_time >> 8 * 1;
    data[2] = start_time >> 8 * 2;
    data[3] = start_time >> 8 * 3;
    data[4] = inChar;
    data[5] = 22;

    if (!radio.write(&data, sizeof(data))) {
      Serial.println("failed");
    }

    // Now, continue listening
    radio.startListening();

    // Set up a timeout period, get the current microseconds
    unsigned long started_waiting_at = micros();
    // Set up a variable to indicate if a response was received or not
    boolean timeout = false;

    // While nothing is received
    while (!radio.available()) {
      if (micros() - started_waiting_at > 200000 ) {
        // If waited longer than 200ms, indicate timeout and exit while loop
        timeout = true;
        break;
      }
    }

    // Describe the results
    if (timeout) {
      Serial.println("Failed, response timed out.");
    } else {
      // Grab the response, compare, and send to debugging spew

      byte data_in[PAYLOAD_SIZE];
      radio.read(&data_in, PAYLOAD_SIZE);

      unsigned long end_time = micros();

      // Get the time, in the first 4 bytes
      unsigned long got_time;
      got_time = (unsigned long)data_in;

      // Get the second two results
      char result1, result2;
      result1 = data_in[4];
      result2 = data_in[5];

      // Spew it
      Serial.print("Sent ");
      Serial.print(start_time);
      Serial.print(", Got response ");
      Serial.print(got_time);
      Serial.print(", Round-trip delay ");
      Serial.print(end_time - start_time);
      Serial.println(" microseconds");
      Serial.print("Result1: ");
      Serial.println((unsigned char)result1);
      Serial.print("Result2: ");
      Serial.println((unsigned char)result2);
    }

    // Serial input
    Serial.print("inChar: ");
    Serial.println(inChar);

    // Try again 1s later
    delay(1000);

  } else if (radioNumber == 1) {

    unsigned long got_time;

    byte data[PAYLOAD_SIZE];

    if ( radio.available()) {
      // Variable for the received timestamp
      while (radio.available()) {
        radio.read(&data, PAYLOAD_SIZE);
      }

      // Pull out the data
      byte operand1 = data[4];
      byte operand2 = data[5];
      // Double the first number
      byte result1 = operand1 * 2;
      // Add them both together
      byte result2 = operand1 + operand2;
      // Put the data back in
      data[4] = result1;
      data[5] = result2;

      //Get the stepper delay
      stepperDelay = (int)operand1 * 10;
      //Limit the stepper delay
      if (stepperDelay < 500) {
        stepperDelay = 500;
      }

      radio.stopListening();
      radio.write(&data, PAYLOAD_SIZE);
      radio.startListening();
      Serial.print("Sent response ");
      Serial.println(got_time);
    }

    unsigned long currentTime = micros();
    unsigned long timeDifference = currentTime - stepperLastStep;

    if (timeDifference > stepperDelay) {
      stepperState = !stepperState;
      digitalWrite(STEPPER_STEP_PIN, stepperState);
      stepperLastStep = micros();
      if (stepperState) {
        stepperPosition += 1;
      }
    }

    if (stepperPosition > 200) {
      digitalWrite(STEPPER_RESET, HIGH);
      while (1) {}
    }

  } else if (radioNumber == 2) {
    setStepperPosition(0.499);
    setStepperPosition(0.501);
    setStepperPosition(0.498);
    setStepperPosition(0.502);
    setStepperPosition(0.496);
    setStepperPosition(0.504);
    digitalWrite(STEPPER_RESET, HIGH);
    while (1) {}

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
  Serial.println("got serial");
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
