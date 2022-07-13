#include <TimeLib.h>

#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM6DS33.h>
// montgomery sea level pressure
#define SEALEVELPRESSURE_HPA (1013)

Adafruit_BMP3XX bmp;
Adafruit_LSM6DS33 lsm6ds33;

// mode 0 means read mode, mode 1 means write mode
const int mode = 1;
File myFile;

void setup() {
  Serial.begin(9600);
  //  while (!Serial) {
  //    delay(1);
  //  }

  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  pinMode(2, OUTPUT);
  SD.begin(10);
  Serial.println("initialization done.");

  if (mode == 0) {
    myFile = SD.open("Data.txt");
    Serial.print(myFile.name());
    Serial.println(":");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }

    myFile.close();
    return;
  }
  if (SD.exists("Data.txt")) {
    SD.remove("Data.txt");
  }
  myFile = SD.open("Data.txt", FILE_WRITE);
  myFile.println("Time(s),Pressure(hPa),Altitude(m),Accel x(m/s^2),Accel y(m/s^2),Accel z(m/s^2),Gyro x(radians/s),Gyro y(radians/s), Gyro z(radians/s),Temperature(C)");
  myFile.close();
  Serial.println("Adafruit BMP388 / BMP390 test");

  bmp.begin_I2C();

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);


  lsm6ds33.begin_I2C();
//  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_4000_DPS);
//  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
//  lsm6ds33.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
//lsm6ds33.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);

  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2
}

void loop() {

  if (mode == 0) {
    return;
  }
  digitalWrite(2, HIGH);


  myFile = SD.open("Data.txt", FILE_WRITE);

  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }


  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

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

  int hourr = hour();
  int minutee = minute();
  int secondd = second();
  int total_seconds = hourr * 360 + minutee * 60 + secondd;

  // "Time(s),Pressure(hPa),Altitude(m),Accel x(m/s^2),Accel y(m/s^2),Accel z(m/s^2),Gyro x(radians/s),Gyro y(radians/s), Gyro z(radians/s),Temperature(C)"

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
  myFile.println(board_temperature);
  //  myFile.print(",");
  //  myFile.print(",");
  //  myFile.print(",");
  //  myFile.print(",");

  myFile.close();

}
