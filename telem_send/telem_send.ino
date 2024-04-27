#include "DFRobot_GNSS.h"
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

// I2C Definitions
#define I2C_SCL PB6
#define I2C_SDA PB7
#define SHT_ADDR 0x44
TwoWire *i2c_bus = new TwoWire(I2C_SDA, I2C_SCL);

// SPI Definitions
#define RF_CE PB1
#define RF_CSN PF0
#define SPI_MISO PB4
#define SPI_MOSI PB5
#define SPI_SCLK PB3

// Peripheral Definitions
DFRobot_GNSS_I2C gnss(i2c_bus, GNSS_DEVICE_ADDR);
RF24 radio(RF_CE, RF_CSN);

float distance(float lat1, float lon1, float lat2, float lon2);
void encode(float *source, uint8_t *target, uint8_t num_values);

uint8_t shtBuf[8];
uint8_t error;

float initialLat{};
float initialLon{};

const uint8_t address[1] = {0x18};

void setup() {
  Serial.begin(115200);
  printf_begin();

  // Also calls begin() on the i2c_bus
  while (!gnss.begin()) {
    Serial.println("Failed init on GPS");
    delay(5000);
  }

  gnss.enablePower();
  gnss.setGnss(eGPS_BeiDou_GLONASS); // Use all available sources

  // SPI & Radio Setup
  SPI.setMISO(SPI_MISO);
  SPI.setMOSI(SPI_MOSI);
  SPI.setSCLK(SPI_SCLK);

  while(!radio.begin()) {
    Serial.println("Failed init on radio");
    delay(5000);
  }

  const uint8_t address[1] = {0x18};

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void loop() {
  // Do temp & humidity stuff
  i2c_bus->beginTransmission(SHT_ADDR);

  shtBuf[0] = 0xFD; // High precision command
  i2c_bus->write(shtBuf, 1);


  error = i2c_bus->endTransmission();
  delay(100); // Datasheet says wait (and learned the hard way that its correct)

	if ( error != 0 ) {
    Serial.println("Error on I2C send.");
    delay(5000);
    return; // Equivalent to continue in arduino environment
	}

  // Read the 6 bytes of data into buffer
  i2c_bus->requestFrom(SHT_ADDR, 6);

  for(int i{}; i < 6; i++) {
    uint8_t byte = i2c_bus->read(); // Receive a byte
    shtBuf[i] = byte;
  }

  // Combine the bytes for temp
  int16_t val_temp{((int16_t)shtBuf[0] << 8) | (int16_t)(shtBuf[1])};

  // Combine the bytes for humidity
  int16_t val_humidity{((int16_t)shtBuf[3] << 8) | (int16_t)(shtBuf[4])};

  // Convert via formulas from datasheet
  float temperature{-45.0 + 175.0 * (val_temp/65535.0)};
  float humidity{-6.0 + 125.0 * (val_humidity/65535.0)};

  // GPS
  sLonLat_t latRaw{gnss.getLat()};
  sLonLat_t lonRaw{gnss.getLon()};
  float altitude{gnss.getAlt()};
  float velocity{gnss.getSog()};
  float course{gnss.getCog()};

  // Take decimal degrees format
  float lat{latRaw.latitudeDegree};
  float lon{lonRaw.lonitudeDegree};

  // Set initial values if this is the first iteration
  if (initialLat == 0) {
    initialLat = lat;
    initialLon = lon;
  }

  Serial.println(lat);
  float dist{distance(initialLat, initialLon, lat, lon)};

  // Encode radio payload
  float values[] = {velocity, altitude, course, dist, temperature, humidity};
  uint8_t payload[12] = {};
  encode(values, payload, 6);

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

  bool sent{radio.write(payload, sizeof(payload))}; // Send via radio

  if(!sent) {
    Serial.println("Tx fail!");
  }
  // radio.printPrettyDetails();

  Serial.println("Cycle complete");
  delay(1000);
}

// Given a pair of lat longs, get the distance (thanks StackOverflow & mathematicians)
float distance(float lat1, float lon1, float lat2, float lon2) {
    const float r{6371.0}; // Earth's radius in kilometers

    float a{0.5 - cos(radians(lat2 - lat1)) / 2.0 + cos(radians(lat1)) * cos(radians(lat2)) * (1 - cos(radians(lon2 - lon1))) / 2.0};

    return 2.0 * r * asin(sqrt(a));
}

// Custom encoding scheme for values to transmit
void encode(float *source, uint8_t *target, uint8_t num_values) {
  for (uint8_t i{}; i < num_values; i++) {
    // Handle out of bounds for my encoding scheme
    if (abs(source[i]) > 500) {
      target[i*2] = 0xFF;
      target[i*2 + 1] = 0xFF;

      continue;
    }

    uint16_t together{uint16_t(abs(source[i]) * 100)};

    target[i*2] = (uint8_t)((together & 0xFF00) >> 8);
    target[i*2 + 1] = (uint8_t)(together & 0xFF);
  }
}