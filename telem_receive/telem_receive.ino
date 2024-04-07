#include <Wire.h>
#include <limits>

void setup() {
  Serial.begin(115200);
}

void loop() {
  // TODO: Receive data with actual radio module
  uint8_t received[12] = {0x0};
  float values[6] = {0.0};

  decode(values, received, 6);
}

// Custom decoding implementation
void decode(float *decoded, uint8_t *encoded, uint8_t num_values) {
  for (uint8_t i{}; i < num_values; i++) {
    uint16_t together = ((uint16_t)encoded[i*2] << 8) | encoded[i*2 + 1];

    // Out of bounds
    if(together == 0xFFFF) {
      decoded[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }

    decoded[i] = (float)together / 100;
  }
}