#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SHT_ADDR 0x44

uint8_t shtBuf[8];
uint8_t sendBuf[16];
uint8_t error;

int16_t val_temp;
int16_t val_humidity;
float temp;
float humidity;

Adafruit_MPU6050 mpu;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Init MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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
	} else {
		// Read the 6 bytes of data into buffer (arduino has a comparatively weird workflow for this...)
		Wire.requestFrom(SHT_ADDR, 6);

    for(int i = 0; i < 6; i++) {
      uint8_t byte = Wire.read(); // Receive a byte
      shtBuf[i] = byte;
    }

    // Combine the bytes for temp
    val_temp = ((int16_t)shtBuf[0] << 8) | (int16_t)(shtBuf[1]);

    // Combine the bytes for humidity
    val_humidity = ((int16_t)shtBuf[3] << 8) | (int16_t)(shtBuf[4]);

    // Convert via formulas from datasheet - digital sensing principle!
    temp = -45.0 + 175.0*(val_temp/65535.0);
    humidity = -6.0 + 125.0*(val_humidity/65535.0);

    Serial.print("Temp:");
    Serial.println(temp);
    Serial.print("Humidity: ");
    Serial.println(humidity);

    // MPU6050 - we have a better source for temperature, so let's ignore this one
    sensors_event_t a, g, temp_dummy;
    mpu.getEvent(&a, &g, &temp_dummy);

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
  }

  Serial.println("Cycle complete");
  delay(2000);
}
