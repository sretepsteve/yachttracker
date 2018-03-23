
//
//
//  YachtTracker - built on Asset Tracker 2 by Particle.
//
//  Uses GPS, Accelerometer to monitor position and report
//
//
//

// Group libraries
#include <AssetTracker2.h>
#include <Particle.h>
#include <math.h>
#include <ctype.h>
// Local libraries
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"




#define STARTING_LATITUDE_LONGITUDE_ALTITUDE "40.4700,-75.5000,500"
#define CLICKTHRESHHOLD 100
#define PUBLISH_DELAY (5 * 60 * 1000)    // in motion publish every x min
#define NO_MOTION_IDLE_SLEEP_DELAY (5 * 60 * 1000)  // no motion for 5 minutes
#define HOW_LONG_SHOULD_WE_SLEEP (6 * 60 * 60)  // wakeup every 6 hours
// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
#define MAX_IDLE_CHECKIN_DELAY (HOW_LONG_SHOULD_WE_SLEEP - 60)  //


AssetTracker2 GPS = AssetTracker2();    // Flexibility to split drivers
#define mySerial Serial1  // GPS on hardware UART on TX/RX pins
AssetTracker2 ATaccel = AssetTracker2();
Adafruit_LIS3DH ada_accel = Adafruit_LIS3DH(A2);  // Direct address for some fn
FuelGauge fuel;

SYSTEM_MODE(SEMI_AUTOMATIC);  // keep cellular off unless...
STARTUP(USBSerial1.begin());  // Enable second serial port over USB
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

//  Main loop variables
bool usingcell = false;

int lastSecond = 0;
int lastLevel = 0;
bool ledState = false;

unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
unsigned long lastReading = 0;
time_t lastIdleCheckin = 0;

//  Accel variables
float roll = 0;
float pitch = 0;
int measx=0, measy=0, measz=0, magnitude=0;
float ax=0, ay=0, az=0;
float alphax = 0.05;
float alphay = 0.05;
float alphaz = 0.05;
String dispAccel = String("");


// Display variables
#define OLED_DC B3
#define OLED_RST B4
#define OLED_CS B5
Adafruit_SSD1306 display(OLED_DC, OLED_RST, OLED_CS);


// GPS Variables
String dispGPS1 = String("");
String dispGPS2 = String("");
float latitude = 0, longitude = 0;
float altitude = 0, fixQuality = 0;
int satellites;

//  SETUP EVERYTHING

void setup() {
    // delay on reboot
    delay(10000);

    initDisplay();

    Serial.begin(9600);     // USB Serial
    USBSerial1.begin(28800);// USB Serial second port for GPS data
    GPS.updateGPS();  //  keep buffer empty

    initGPS();
    GPS.updateGPS();  //  keep buffer empty
    initAccel();
    GPS.updateGPS();  //  keep buffer empty
    initGPIO();

    // Reset variables (maybe from after wakeup from sleep?)
    lastMotion = 0;
    lastPublish = 0;

}


void loop() {
    // refresh time from clock
    if (lastIdleCheckin == 0) {
        lastIdleCheckin = Time.now();
    }

    unsigned long now = millis();

    if (lastMotion > now) { lastMotion = now; }  // Why?  Wrapping counter?
    //if (lastPublish > now) { lastPublish = now; }

    GPS.updateGPS();


    readAccel();

    readGPS();

    updateDisplay();

    // we'll be woken by motion, lets keep listening for more motion.
    // if we get two in a row, then we'll connect to the internet and start reporting in.
    bool hasMotion = digitalRead(WKP);
    digitalWrite(D7, (hasMotion) ? HIGH : LOW);
    if (hasMotion) {
        Serial.println("Motion");
        lastMotion = now;

        if (usingcell == true) {
            if (Particle.connected() == false) {
                Serial.println("Conn motion!");
                Particle.connect();
            }
        }
    }



    // use the real-time-clock here, instead of millis.
    if ((Time.now() - lastIdleCheckin) >= MAX_IDLE_CHECKIN_DELAY) {
        Serial.println("Idle timer");
        // it's been too long!  Lets say hey!
        if (usingcell == true) {
            if (Particle.connected() == false) {
                Particle.connect();
            }
            Serial.println("Pub status");
            Particle.publish(String("_status"), " Idle");
        }
        lastIdleCheckin = Time.now();
    }


    // have we published recently?
    //Serial.println("lastPublish is " + String(lastPublish));
    if (((millis() - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0)) {
        lastPublish = millis();
        Serial.println("Delay timer");

        if (usingcell == true) {
            publishGPS();
        }
    }


//    // use "now" instead of millis...  If it takes us a REALLY long time to connect, we don't want to
//    // accidentally idle out.
//    if ((now - lastMotion) > NO_MOTION_IDLE_SLEEP_DELAY) {
//        // hey, it's been longer than xx minutes and nothing is happening, lets go to sleep.
//        // if the accel triggers an interrupt, we'll wakeup earlier than that.
//
//        Particle.publish(MY_NAME + String("_status"), "sleeping!");
//
//        lastPublish = 0;
//        lastMotion = 0;
//
//        // Hey GPS, please stop using power, kthx.
//        digitalWrite(D6, HIGH);
//
//        // lets give ourselves a chance to settle, deal with anything pending, achieve enlightenment...
//        delay(10*1000);
//        System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP);
//    }

//    if ((now - lastReading) > 2500) {
//        lastReading = now;
//        int currentLevel = getLevelReading();
//
//        if (lastLevel != currentLevel) {
//            publishLevel();
//        }
//        lastLevel = currentLevel;
//        delay(1000);
//    }

    delay(5);
}


//   GPS FUNCTIONS

void initGPS() {
    // electron asset tracker shield needs this to enable the power to the gps module.

    GPS.gpsOn();        // Setup GPS on the tracker, 9600 N81
    GPS.antennaInternal();
    delay(2000);    // give the module a long time to warm up.
    while (mySerial.available()) USBSerial1.write(mySerial.read());

}


void readGPS()  {

    latitude  = convertDegMinToDecDeg(GPS.readLat());
    longitude = convertDegMinToDecDeg(GPS.readLon());
    altitude = GPS.getAltitude();
    fixQuality = GPS.getFixQuality();
    satellites = GPS.getSatellites();

    //pubGPS = String::format("L %3.4f, %3.4f, H %4.0f, Q %3.2",latitude,longitude,altitude,fixQuality);
    dispGPS1 = String::format("L %3.4f, %3.4f",latitude,longitude);
    dispGPS2 = String::format("A %4.0f, F %3.2f, S %d", altitude, fixQuality, satellites);
    USBSerial1.println(dispGPS1 + dispGPS2);

}



double convertDegMinToDecDeg (float degMin) {
    //http://arduinodev.woofex.net/2013/02/06/adafruit_gps_forma/
    double min = 0.0;
    double decDeg = 0.0;

    //get the minutes, fmod() requires double
    min = fmod((double)degMin, 100.0);

    //rebuild coordinates in decimal degrees
    degMin = (int) ( degMin / 100 );
    decDeg = degMin + ( min / 60 );

return decDeg;
}








//  ACCEL FUNCTIONS


void initAccel() {
    // Find and setup the accelerometer
    ATaccel.begin();

    // listen for single-tap events at the threshold
    // keep the pin high for 1s, wait 1s between clicks
    //uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow
    ada_accel.setClick(1, CLICKTHRESHHOLD);//, 0, 100, 50);
}



void readAccel() {
    // Read Accelerometer
    ATaccel.readXYZ(&measx, &measy, &measz);

    //measx = accel.accel.x;
    //measy = accel.accel.y;
    //measz = accel.accel.z;

    //magnitude = sqrt((accel.x*accel.x)+(accel.y*accel.y)+(accel.z*accel.z));

    //Smooth accelerometer values
    ax += alphax * (measx-ax);
    ay += alphay * (measy-ay);
    az += alphaz * (measz-az);

    roll = atan2(ax, az)*180/M_PI;
    pitch = atan2(-ay, sqrt(ax * ax + az * az))*180/M_PI;

//    if (abs(roll) > 5 or abs(pitch) > 5)  {digitalWrite(led2, HIGH);}

    // Create a nice string with commas between x,y,z
    dispAccel = String::format("R %3.1f, P %3.1f",roll,pitch);
    // Send that acceleration to the serial port where it can be read by USB
    USBSerial1.println(dispAccel);
    // delay(pubAccel.length());

}

/*
void setClick(uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow) {
  if (!c) {
    //disable int
    uint8_t r = readRegister8(LIS3DH_REG_CTRL3);
    r &= ~(0x80); // turn off I1_CLICK
    writeRegister8(LIS3DH_REG_CTRL3, r);
    writeRegister8(LIS3DH_REG_CLICKCFG, 0);
    return;
  }
  // else...

  writeRegister8(LIS3DH_REG_CTRL3, 0x80); // turn on int1 click
  writeRegister8(LIS3DH_REG_CTRL5, 0x08); // latch interrupt on int1

  if (c == 1)
    writeRegister8(LIS3DH_REG_CLICKCFG, 0x15); // turn on all axes & singletap
  if (c == 2)
    writeRegister8(LIS3DH_REG_CLICKCFG, 0x2A); // turn on all axes & doubletap


  writeRegister8(LIS3DH_REG_CLICKTHS, clickthresh); // arbitrary
  writeRegister8(LIS3DH_REG_TIMELIMIT, timelimit); // arbitrary
  writeRegister8(LIS3DH_REG_TIMELATENCY, timelatency); // arbitrary
  writeRegister8(LIS3DH_REG_TIMEWINDOW, timewindow); // arbitrary
}

uint8_t Adafruit_LIS3DH::getClick(void) {
  return readRegister8(LIS3DH_REG_CLICKSRC);
}

*/




//  DISPLAY FUNCTIONS
void initDisplay()  {

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x32)
  // init done

  display.display(); // show splashscreen
  //delay(2000);
  //display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
}


void updateDisplay()  {
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(dispAccel);
  display.println(dispGPS1);
  display.println(dispGPS2);
  display.display();


}





//  GPIO FUNCTIONS

void initGPIO() {
    // POWER TEMPERATURE SENSOR
	pinMode(A1,OUTPUT);
    pinMode(B5,OUTPUT);
    digitalWrite(B5, HIGH);
    digitalWrite(A1, LOW);

    // Bue LED for blinking on accelerometer.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);


    // digitalWrite(A5, HIGH);         // power pin for water sensor  ?? This is SPI for Accel??
}


int getLevelReading() {

    //
    int emptyLevelValue = 3500;
    int fullLevelValue = 1800;
    // about 2 inches of water ->
    //int levelValue = analogRead(D0) - 2434;

    //delay(50);
    int levelReading = analogRead(A4);
    int levelValue = map(levelReading, fullLevelValue, emptyLevelValue, 0, 100);
    levelValue = 100 - levelValue;  // flip it

    Serial.println("water level is " + String(levelReading) + " percentage full is " + String(levelValue));

    return levelValue;
}





//  PUBLISH FUNCTIONS


void publishGPS() {
    unsigned int msSinceLastMotion = (millis() - lastMotion);
    int motionInTheLastMinute = (msSinceLastMotion < 60000);

    /*
    String gps_line = String::format("%f,%f,%f,%f,
        convertDegMinToDecDeg(GPS.latitude), convertDegMinToDecDeg(GPS.longitude), GPS.altitude, GPS.speed);
    */

//    String gps_line =
//          "{\"lat\":"    + String(convertDegMinToDecDeg(GPS.latitude))
//        + ",\"lon\":-"   + String(convertDegMinToDecDeg(GPS.longitude))
//        + ",\"a\":"     + String(GPS.altitude)
//        + ",\"q\":"     + String(GPS.fixquality)
//        + ",\"spd\":"   + String(GPS.speed)
//        + ",\"mot\":"   + String(motionInTheLastMinute)
//        + ",\"s\": "  + String(GPS.satellites)
//        + ",\"vcc\":"   + String(fuel.getVCell())
//        + ",\"soc\":"   + String(fuel.getSoC())
//        + "}";
//
//    Particle.publish(MY_NAME + String("_location"), gps_line, 60, PRIVATE);

    if ((latitude != 0) && (longitude != 0)) {
        String trkJsonLoc = String("{")
            + "\"c_lat\":" + String(latitude)
            + ",\"c_lng\":" + String(longitude)
            + ",\"c_unc\":" + String(GPS.getFixQuality())
            + ",\"c_alt\":" + String(GPS.getAltitude())
            + "}";
        if (usingcell == true) Particle.publish("trk/loc", trkJsonLoc, 60, PRIVATE);

     }


    float batteryVoltage = fuel.getVCell();
    CellularSignal signalInfo = Cellular.RSSI();
    String devJson = String("{")
            + "\"vcell\":" + String::format("%.4f", batteryVoltage)
            + ",\"cell_rssi\":" + String(signalInfo.rssi)
            + ",\"cell_qual\":" + String(signalInfo.qual)
            + "}";
    if (usingcell == true) Particle.publish("trk/dev", devJson, 60, PRIVATE);

//          int value = rand() * 100;
//     String sensorJson = String("{")
//            + "\"level\":" + String::format("%d", value)
//            + "}";
//     Particle.publish("trk/level", sensorJson, 60, PRIVATE);
}
