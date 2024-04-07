#include "DFRobot_GNSS.h"
#include <Wire.h>
#include <cmath>

#define SHT_ADDR 0x44

uint8_t shtBuf[8];
uint8_t sendBuf[16];
uint8_t error;

float initialLat{};
float initialLon{};

DFRobot_GNSS_I2C gnss(&Wire, GNSS_DEVICE_ADDR);

float distance(float lat1, float lon1, float lat2, float lon2);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (!gnss.begin()) {
    Serial.println("Failed init on GPS");
  }

  gnss.enablePower();
  gnss.setGnss(eGPS_BeiDou_GLONASS); // Use all available sources

  delay(100);
}

void loop() {
  // Do temp & humidity stuff
  Wire.beginTransmission(SHT_ADDR);

  shtBuf[0] = 0xFD; // Read high precision command
  Wire.write(shtBuf, 1);

  error = Wire.endTransmission();
  delay(100); // Datasheet says wait (and learned the hard way that its correct)

	if ( error != 0 ) {
    Serial.println("Error on I2C send.");
    return; // Supposedly equivalent to continue in arduino environment
	}

  // Read the 6 bytes of data into buffer (arduino has a comparatively weird workflow for this...)
  Wire.requestFrom(SHT_ADDR, 6);

  for(int i = 0; i < 6; i++) {
    uint8_t byte = Wire.read(); // Receive a byte
    shtBuf[i] = byte;
  }

  // Combine the bytes for temp
  int16_t val_temp = ((int16_t)shtBuf[0] << 8) | (int16_t)(shtBuf[1]);

  // Combine the bytes for humidity
  int16_t val_humidity = ((int16_t)shtBuf[3] << 8) | (int16_t)(shtBuf[4]);

  // Convert via formulas from datasheet
  float temperature = -45.0 + 175.0 * (val_temp/65535.0);
  float humidity = -6.0 + 125.0 * (val_humidity/65535.0);

  Serial.print("Temp:");
  Serial.println(temp);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  // GPS
  sLonLat_t latRaw = gnss.getLat();
  sLonLat_t longRaw = gnss.getLon();
  float altitude = gnss.getAlt();
  float velocity = gnss.getSog();
  float course = gnss.getCog();

  // Take decimal degrees format
  float lat = latRaw.latitudeDegree;
  float lon = lonRaw.lonitudeDegree;

  // Set initial values if this is the first iteration
  if (initialLat == 0) {
    initialLat = lat;
    initialLon = lon;
  }

  float distance = distance(initialLat, initialLon, lat, lon);

  // Encode radio payload
  float values[] = {velocity, altitude, course, distance, temperature, humidity};
  uint8_t payload[12] = {};

  encode(values, payload, 6);

  // TODO: Send payload via radio

  Serial.println("Cycle complete");
  delay(2000);
}

// Given a pair of lat longs, get the distance (thanks StackOverflow & mathematicians)
float distance(float lat1, float lon1, float lat2, float lon2) {
    const float r = 6371.0; // Earth's radius in kilometers
    const float p = M_PI / 180.0; // Conversion factor from degrees to radians

    float a = 0.5 - cos((lat2 - lat1) * p) / 2.0 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2.0;

    return 2.0 * r * asin(sqrt(a));
}

// Custom encoding scheme for values to transmit
void encode(float *source, uint8_t *target, uint8_t num_values) {
  for (uint8_t i{}; i < num_values; i++) {
    // Handle out of bounds for my encoding scheme
    if (std::abs(source[i]) > 500) {
      target[i*2] = 0xFF;
      target[i*2 + 1] = 0xFF;

      continue;
    }

    uint16_t together = uint16_t(std::abs(source[i]) * 100);

    target[i*2] = (uint8_t)((together & 0xFF00) >> 8);
    target[i*2 + 1] = (uint8_t)(together & 0xFF);
  }
}