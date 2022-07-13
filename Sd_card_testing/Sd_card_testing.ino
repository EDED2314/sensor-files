#include <SD.h>
 
File myFile;
void setup()
{
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  SD.begin(10);
  Serial.println("initialization done.");
 
  // open the file. note that only one file can be open at a time, so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
 
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
  // close the file:
    
    Serial.println("done.");
    myFile.close();
  }
  myFile = SD.open("test.txt");
  while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    

   Serial.println(myFile.name());
    myFile.close();
 // SD.remove("test.txt");
}
 
void loop()
{
  // nothing happens after setup
}
