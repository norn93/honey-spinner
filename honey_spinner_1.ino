#include <SPI.h>
#include "RF24.h"

#define PAYLOAD_SIZE 6

// 0 = 1Node is Parent
// 1 = 2Node is Child
byte addresses[][6] = {"1Node", "2Node"};
bool radioNumber = 0;

// Radio
RF24 radio(7, 8);

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
    data[4] = 17;
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

      // Get the time, if the first 4 bytes
      unsigned long got_time;
      got_time = (unsigned long)data_in;

      // Get the second two results
      char result1, result2;
      result1 = data_in[4];
      result2 = data_in[5];

      unsigned long end_time = micros();

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
    // Try again 1s later
    delay(1000);

  } else if (radioNumber == 1) {
    unsigned long got_time;

    byte data[PAYLOAD_SIZE];

    if ( radio.available()) {
      // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read(&data, PAYLOAD_SIZE);             // Get the payload
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
      
      radio.stopListening();                                        // First, stop listening so we can talk
      radio.write(&data, PAYLOAD_SIZE);              // Send the final one back.
      radio.startListening();                                       // Now, resume listening so we catch the next packets.
      Serial.print("Sent response ");
      Serial.println(got_time);
    }
  }
}
