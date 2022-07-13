
#include <Wire.h>

#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#include <Adafruit_LSM6DSOX.h>
Adafruit_LSM6DSOX lsm6ds;

#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;


int led = 13;
const int chipSelect = BUILTIN_SDCARD;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  pinMode(led, OUTPUT);
  Serial.begin(9600);


  lsm6ds.begin_I2C();
  lis3mdl.begin_I2C();


  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);

  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_560_HZ);

  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_POWERDOWNMODE);


  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!



  SD.begin(chipSelect);

}

void loop() {

  digitalWrite(led, HIGH);   

  myFile = SD.open("Data.txt", FILE_WRITE);

  
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  sensors_event_t accel, gyro, mag, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  float board_temperature;
  float pressure = bmp.pressure / 100.0;
  float aprox_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  board_temperature = (temp.temperature + bmp.temperature) / 2;


  float accel_x = accel.acceleration.x;
  float accel_y = accel.acceleration.y;
  float accel_z = accel.acceleration.z;

  float gyro_x = gyro.gyro.x;
  float gyro_y = gyro.gyro.y;
  float gyro_z = gyro.gyro.z;

  float mag_x = mag.magnetic.x;
  float mag_y = mag.magnetic.y;
  float mag_z = mag.magnetic.z;
  
  int hourr = hour();
  int minutee = minute();
  int secondd = second();
  int total_seconds = hourr * 360 + minutee * 60 + secondd;

  // "Time(s),Pressure(hPa),Altitude(m),Accel x(m/s^2),Accel y(m/s^2),Accel z(m/s^2),Gyro x(radians/s),Gyro y(radians/s), Gyro z(radians/s), Mag x (uTesla), Mag y (uTesla), Mag z (uTesla), Temperature(C)"
  myFile.print(total_seconds);
  myFile.print(",");
  myFile.print(pressure);
  myFile.print(",");
  myFile.print(aprox_alt);
  myFile.print(",");
  myFile.print(accel_x);
  myFile.print(",");
  myFile.print(accel_y);
  myFile.print(",");
  myFile.print(accel_z);
  myFile.print(",");
  myFile.print(gyro_x);
  myFile.print(",");
  myFile.print(gyro_y);
  myFile.print(",");
  myFile.print(gyro_z);
  myFile.print(",");
  myFile.print(mag_x);
  myFile.print(",");
  myFile.print(mag_y);
  myFile.print(",");
  myFile.print(mag_z);
  myFile.print(",");
  myFile.println(board_temperature);

  myFile.close();



}
