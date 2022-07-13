#include <SparkFun_Qwiic_KX13X.h>

#include <Arduino_LSM6DS3.h>


#include <SD.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h> 
#include "Adafruit_BMP3XX.h"
/* ... */



// i2c


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

QwiicKX134 kxAccel;
outputData myData;

//sd card stuf
File myFile;

int sensorPin = 7; 

void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("hi");
 kxAccel.begin();
Serial.println("hi");
 kxAccel.initialize(DEFAULT_SETTINGS);
 //kxAccel.setRange(KX134_RANGE32G);
 // this is our lsm6d3 
 Serial.println("hi");
 IMU.begin();
Serial.println("hi");
 Serial.println("hi");
  
/*
 // THIS IS FOR THE SD CARD :/
  SPI.begin();
  SD.begin(10);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
   Serial.println("Writing to test.txt...");
   myFile.println("testing 1, 2, 3.");
    // close the file:
   myFile.close();
   Serial.println("done.");

  // re-open the file for reading:
  myFile = SD.open("test.txt", FILE_READ);
    // Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();

*/
  
}

void loop() 
{ /* ask it to read in the data */ 
  /* Get a new sensor event */ 
  myData = kxAccel.getAccelData(); 
  Serial.print("X: ");
  Serial.print(myData.xData, 4);
  Serial.print("g ");
  Serial.print(" Y: ");
  Serial.print(myData.yData, 4);
  Serial.print("g ");
  Serial.print(" Z: ");
  Serial.print(myData.zData, 4);
  Serial.println("g ");

 int reading = analogRead(sensorPin); 
 float voltage = reading * 5.0;
 voltage /= 1024.0; 
 float temperatureC = (voltage - 0.5) * 100 ;
 Serial.print(temperatureC); 
 Serial.println(" degrees C");
 

  
  float x, y, z;  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    Serial.println("LSM6D Acceleration in G's");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
  float a, b, c;
   if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(a, b, c);
    Serial.println("LSM6D Gyroscope in degrees/second");
    Serial.print(a);
    Serial.print('\t');
    Serial.print(b);
    Serial.print('\t');
    Serial.println(c);
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
  delay(100);
}
