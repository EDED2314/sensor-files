#include <Wire.h>
#include <SPI.h>


#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1030)

Adafruit_BMP3XX bmp;

#include <Adafruit_LSM6DS33.h>
Adafruit_LSM6DS33 lsm6ds;

#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;


void setup(void) {

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("");

  //-----------------------------------------------

  Serial.println("Adafruit LSM6DS+LIS3MDL test!");

  bool lsm6ds_success, lis3mdl_success;

  // hardware I2C mode, can pass in address & alt Wire

  lsm6ds_success = lsm6ds.begin_I2C();
  lis3mdl_success = lis3mdl.begin_I2C();

  if (!lsm6ds_success) {
    Serial.println("Failed to find LSM6DS chip");
  }
  if (!lis3mdl_success) {
    Serial.println("Failed to find LIS3MDL chip");
  }
  if (!(lsm6ds_success && lis3mdl_success)) {
    while (1) {
      delay(10);
    }
  }

  //-----------------------------------------------

  Serial.println("Adafruit BMP388 / BMP390 test!");

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  //-----------------------------------------------

  Serial.println("LSM6DS and LIS3MDL Found!");

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  Serial.println("Accelerometer range set to: +-8G");
  //2, 4, 8, 16

  lsm6ds.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
  Serial.println("Accelerometer data rate set to: 3.33 KHz");
  //0, 12_5, 26, 52, 104, 208,416, 833, 1_66K, 3_33K, 6_66K

  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );
  Serial.println("Gyro range set to: 500 DPS");
  //125, 250, 500, 1000, 2000, 4000

  lsm6ds.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  Serial.println("Gyro data rate set to: 1.66 KHz ");
  //0, 12_5, 26, 52, 104, 208,416, 833, 1_66K, 3_33K, 6_66K

  //-----------------------------------------------

  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  Serial.println("Magnetometer data rate set to: 1000 Hz ");
  //0_625, 1_25, 2_5, 5, 10, 20, 40, 80, 155, 300, 560, 1000

  lis3mdl.setRange(LIS3MDL_RANGE_8_GAUSS);
  Serial.println("Range set to: +=8 Gauss");
  //4, 8, 12, 16

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Magnetometer performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Magnetometer operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

}

void loop() {

  sensors_event_t accel, gyro, mag, temp;

  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  if (temp.temperature >= 0) {
    digitalWrite(7, HIGH);
  }

  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x, 4);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y, 4);
  Serial.print(" \t\tZ: ");
  Serial.print(accel.acceleration.z, 4);
  Serial.println(" \tm/s^2 ");

  Serial.print("Gyro  X: ");
  Serial.print(gyro.gyro.x, 4);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z, 4);
  Serial.println(" \tradians/s ");

  Serial.print("Mag   X: ");
  Serial.print(mag.magnetic.x, 4);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z, 4);
  Serial.println(" \tuTesla ");
  Serial.println();

  //-----------------------------------------------

  if (! bmp.performReading()) {
    Serial.println("BMP Failed to perform reading :(");
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();

  //-----------------------------------------------

  Serial.println("-----------------------------------------------");
  Serial.println();

  delay(1000);

}
