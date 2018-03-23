/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

This example is for a 128x64 size display using I2C to communicate
3 pins are required to interface (2 I2C and one reset)

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"


#define OLED_DC A0
#define OLED_RST A1
#define OLED_CS A2

SYSTEM_MODE(MANUAL);  // keep cellular off unless...
STARTUP(USBSerial1.begin());  // Enable second serial port over USB

//int8_t DC, int8_t RST, int8_t CS)
Adafruit_SSD1306 display(OLED_DC, OLED_RST, OLED_CS);



void setup()   {
  Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x32)
  // init done

  display.display(); // show splashscreen
  delay(2000);
  display.clearDisplay();   // clears the screen and buffer


  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Hello, world!");
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.println(3.141592);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print("TextSize2");
  display.display();
  delay(2000);
  display.clearDisplay();   // clears the screen and buffer

}


void loop() {

/*  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(2,2);
  display.println("NATE!");
  display.display();
  delay(3000);

  // invert the display
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);


  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(2,2);
  display.println("ALEX!");
  display.display();
  delay(3000);

  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
*/

}
