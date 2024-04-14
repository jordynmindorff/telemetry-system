#include <Wire.h>
#include <math.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define RF_CE 4
#define RF_CSN 6

void decode(float *decoded, uint8_t *encoded, uint8_t num_values);

RF24 radio(RF_CE, RF_CSN);

const uint8_t address[1] = {0x18};

void setup() {
  Serial.begin(115200);

  // Radio setup
  while(!radio.begin()) {
    Serial.println("Failed init on radio");
    delay(5000);
  }

  radio.openReadingPipe(1, address); // 0 = "reading pipe number"
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  uint8_t pipe;
  if (radio.available(pipe)) {
    Serial.println("Avail!");
    // Read over the air
    uint8_t received[12]{};
    radio.read(&received, sizeof(received));

    float values[6]{};
    decode(values, received, 6); // Decode the values

    Serial.print("Velocity: ");
    Serial.println(values[0]);
    Serial.print("Altitude: ");
    Serial.println(values[1]);
    Serial.print("Course: ");
    Serial.println(values[2]);
    Serial.print("Distance from Start: ");
    Serial.println(values[3]);
    Serial.print("Temperature: ");
    Serial.println(values[4]);
    Serial.print("Humidity: ");
    Serial.println(values[5]);
  }

  Serial.println("Cycle complete.");
  delay(1000);
}

// Custom decoding implementation
void decode(float *decoded, uint8_t *encoded, uint8_t num_values) {
  for (uint8_t i{}; i < num_values; i++) {
    uint16_t together = ((uint16_t)encoded[i*2] << 8) | encoded[i*2 + 1];

    // Out of bounds
    if(together == 0xFFFF) {
      decoded[i] = NAN;
      continue;
    }

    decoded[i] = (float)together / 100;
  }
}