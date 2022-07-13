#include <xSW01.h>

#include <si1133_config.h>
#include <xSL01.h>

#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <SSD1306init.h>
#include <xOD01.h>

#include <xCore.h>
#include <xVersion.h>


void setup() {
  Wire.begin();

  xSL01.begin();
  xSW01.begin();
  xOD01.begin();
  xOD01.setX();
  delay(1000);
}

void loop() {
  xSW01.poll();
  xSL01.poll();
  if (SW01.getAltitude() < 3){
    xOD01.print("Luminosity: ");
    xOD01.print(SL01.getLUX());
    xOD01.println(" LUX");
  } else if (SW01.getAltitude() = 3) {
    xOD01.print("Temperature: ");
    xOD01.print(SW01.getTemperature_C());
    xOD01.println(" Degrees Celsius");
  } else if (SW01.getAltitude() > 3) {
    xOD01.print("Humidity: ");
    xOD01.print(SW01.getHumidity());
    xOD01.println("%");
  }
}
