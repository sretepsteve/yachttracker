//
//
//  YachtTracker - built on:
//      Particle Electron Asset Tracker 2
//      Adafruit OLED 128x64 display
//      Adafruit BNO055 IMU with accel, gyro, mag, temp
//
//  Monitors position, attitude, and heading for change
//  and will notify user depending on degree of change
//
/*  The publishing routines have three modes.

SLEEPING - Device stops publishing updates, though still remains alert
for triggers.

UPDATE - Regular updating on a timed interval, mostly to maintain the
connection and provide evidence that all is well.  This should be
roughly every 20 minutes or so, but can be changed.

ALERT - Higher update rate based on some trigger condition.  Device remains
in this mode until the triggered condition is removed for a given time period.

??If the trigger continues for more than 4 hours continuous, the board will
not go into alert mode for another 24 hours.

*/
//  ALERT will publish new data more frequently

// Group libraries
#include <Particle.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

// Local libraries
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GPS.h"
#include "Adafruit_LIS3DH.h"
#include "particle-BNO055.h"


SYSTEM_MODE(SEMI_AUTOMATIC);  // keep cellular off unless...
SYSTEM_THREAD(ENABLED);

STARTUP(USBSerial1.begin());  // Enable second serial port over USB
#define highspeedserial USBSerial1     // for monitoring high speed output
#define consoleserial Serial           // for console update messages
// STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));  // For deep sleep

#define gpsSerial Serial1  // GPS on hardware UART on TX/RX pins
Adafruit_GPS GPS = Adafruit_GPS(&gpsSerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2);  // Accel on Asset Tracker
Adafruit_BNO055 bno = Adafruit_BNO055(55);  // IMU unit
FuelGauge fuel;

//  Main loop variables
bool usingcell = false;

int lastSecond = 0;
int lastLevel = 0;
bool ledState = false;

unsigned long now = 0;
unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
unsigned long lastReading = 0;
time_t lastIdleCheckin = 0;

uint32_t timer = 0;

uint8_t CLICKTHRESHHOLD = 100;
unsigned long PUBLISH_INTERVAL = (20 * 60 *1000);  // Update position every 20 min
unsigned long PUBLISH_DELAY = (5 * 60 * 1000);    // in motion publish every 5 min
unsigned long NO_MOTION_IDLE_SLEEP_DELAY = (5 * 60 * 1000); // no motion for 5 minutes
unsigned long HOW_LONG_SHOULD_WE_SLEEP = (6 * 60 * 60);  // wakeup every 6 hours
unsigned long MAX_IDLE_CHECKIN_DELAY = (HOW_LONG_SHOULD_WE_SLEEP - 60);  //



//  Accel variables
float roll = 0;
float pitch = 0;
int measx=0, measy=0, measz=0, magnitude=0;
float ax=0, ay=0, az=0;
float alphax = 0.05;
float alphay = 0.05;
float alphaz = 0.05;
String dispAccel = String("");


// BNO variables
sensors_event_t event;
float imu_x=0, imu_y=0, imu_z=0;
String dispimu = String("");


// Display variables
#define OLED_DC B3
#define OLED_RST B4
#define OLED_CS B5
Adafruit_SSD1306 display(OLED_DC, OLED_RST, OLED_CS);


// GPS Variables
float latitude = 0, longitude = 0;
float altitude = 0, fixquality = 0;
int satellites;
String dispGPS1 = String("");
String dispGPS2 = String("");
String dispGPS3 = String("");
bool usingInterrupt = false;
bool GPSECHO = false;

// Trigger variables
bool hasMotion = false;
float fixedlatitude = 40.49170;
float fixedlongitude = -75.51520;
float fixedimu_x=180.0, fixedimu_y=0, fixedimu_z=0;
float triggerimu_x=10.0, triggerimu_y=5.0, triggerimu_z=5.0;
float triggerdistance = 10/1000;  // distance in meters
float triggerspeed = 0.1;
float driftdistance = 0;
String dispTrig1 = String("");
String dispTrig2 = String("");


// Publish variables
String trkJsonLoc = String("");


//  SETUP EVERYTHING

void setup() {
    // delay on reboot
    delay(10000);

    initDisplay();
    delay(20000);

    consoleserial.begin(9600);     //  Serial for console updates
    highspeedserial.begin(28800);// USB Serial second port for GPS data

    initGPS();
    GPS.read();  //  keep buffer empty

    initAccel();
    GPS.read();  //  keep buffer empty

    initbno();
    GPS.read(); // keep buffer empty

    initGPIO();
    GPS.read();  //  keep buffer empty

    // Reset variables (maybe from after wakeup from sleep?)
    lastMotion = 0;
    lastPublish = 0;

}


void loop() {

    readAccel();

    readbno();

    readGPS();

    updateDisplay();

    checktriggers();

    updatedata();

}


//  TRIGGER FUNCTIONS

void checktriggers()  {

  // refresh time from clock if it happens to be zero
  if (lastIdleCheckin == 0) {
      lastIdleCheckin = Time.now();
  }

  now = millis();
  if (lastMotion > now) { lastMotion = now; }  // Why?  Wrapping counter?
  //if (lastPublish > now) { lastPublish = now; }

  // we'll be woken by motion, lets keep listening for more motion.
  // if we get two in a row, then we'll connect to the internet and start reporting in.
  hasMotion = digitalRead(WKP);


    fixedlatitude += 0.0001 * (latitude-fixedlatitude);
    fixedlongitude += 0.0001 * (longitude-fixedlongitude);


      driftdistance = getDistanceFromLatLonInKm(fixedlatitude,fixedlongitude,latitude,longitude);
      dispTrig2 = String("");

      if (driftdistance > triggerdistance) {
        dispTrig2 += String("D");
        }
      if (GPS.speed > triggerspeed)  {
        dispTrig2 += String("V");
        }
      if ( abs(imu_x) - fixedimu_x > triggerimu_x) {
        dispTrig2 += String("H");
        }
      if ( abs(imu_y) - fixedimu_y > triggerimu_y) {
        dispTrig2 += String("P");
      }
      if ( abs(imu_z) - fixedimu_z > triggerimu_z) {
        dispTrig2 += String("R");
      }


      dispTrig1 = String::format("%1.3f %3.5f",driftdistance, GPS.speed);
}


float getDistanceFromLatLonInKm(float lat1,float lon1, float lat2, float lon2) {
        float R = 6371;   // Radius of the earth in km
        float dLat = deg2rad(abs(lat2-lat1));  // deg2rad below
        float dLon = deg2rad(abs(lon2-lon1));
        float a =
          sin(dLat/2) * sin(dLat/2) +
          cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
          sin(dLon/2) * sin(dLon/2)
          ;
        float c = 2 * atan2(sqrt(a), sqrt(1-a));
        float d = R * c; // Distance in km
        return d;
      }


float deg2rad(float deg) {
        return deg * (3.14159265/180);
    }



void updatedata()  {



  //    if (alerting) {
  //
  //    }
  //    else
  //    {
  //
  //    }

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



//    delay(2);




}



//   GPS FUNCTIONS

void initGPS() {
    // electron asset tracker shield needs this to enable the power to the gps module.
      pinMode(D6,OUTPUT);
      digitalWrite(D6,LOW);

        GPS.begin(9600);        // Setup serial port 9600 N81
        while (gpsSerial.available()) USBSerial1.write(gpsSerial.read());

        delay(2000);    // give the module a long time to warm up.
        while (gpsSerial.available()) USBSerial1.write(gpsSerial.read());


        // request only GGA and RMC sentences
//        USBSerial1.println("Requesting only RMC GGA using PUBX,40");
        GPS.sendCommand("$PUBX,40,GGA,1,1,0,0,0,0*5A");
        delay(100);
        GPS.sendCommand("$PUBX,40,RMC,1,1,0,0,0,0*47");
        delay(100);
        GPS.sendCommand("$PUBX,40,PUBX,1,1,0,0,0,0*04");
        delay(100);

        //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        GPS.sendCommand("$PUBX,40,GLL,0,0,0,0,0,0*5C");
        delay(100);
        while (gpsSerial.available()) USBSerial1.write(gpsSerial.read());
        GPS.sendCommand("$PUBX,40,VTG,0,0,0,0,0,0*5E");
        delay(100);
        while (gpsSerial.available()) USBSerial1.write(gpsSerial.read());
        GPS.sendCommand("$PUBX,40,GSA,0,0,0,0,0,0*4E");
        delay(100);
        while (gpsSerial.available()) USBSerial1.write(gpsSerial.read());
        //GPS.sendCommand("$PUBX,40,GSV,0,0,0,0,0,0*59");
        GPS.sendCommand("$PUBX,40,GSV,1,1,0,0,0,0*59");
        delay(200);
        while (gpsSerial.available()) USBSerial1.write(gpsSerial.read());

//    GPS.antennaInternal();  needed??
//    delay(2000);    // give the module a long time to warm up.
//    while (gpsSerial.available()) USBSerial1.write(gpsSerial.read());

//      timer = millis();  // WHY is this here?
}


void readGPS()  {
    // in case you are not using the interrupt above, you'll
    // need to 'hand query' the GPS, not suggested :(
    if (! usingInterrupt) {
      // read data from the GPS in the 'main loop'

      while (gpsSerial.available()) {

      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) USBSerial1.print(c);



    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//      highspeedserial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  }

}

/*    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();

    // approximately every 10 seconds or so, print out the current stats
    if (millis() - timer > 30000) {
      timer = millis(); // reset the timer

      Serial.print("\nTime: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print("Fix: "); Serial.println((int)GPS.fix);
      Serial.print("Sats: "); Serial.print(GPS.satellitesreceived);
      Serial.print(" / "); Serial.println(GPS.meansignal);
//      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      if (GPS.fix) {
//        Serial.print("Location: ");
//        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//        Serial.print(", ");
//        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        Serial.print("Location: ");
        Serial.print(GPS.latitudeDegrees, 4);
        Serial.print(", ");
        Serial.println(GPS.longitudeDegrees, 4);

        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
    }
*/

//      GPS.read();
//    latitude  = GPS.latitude();
//    longitude = GPS.longitude();
//    altitude = GPS.altitude();
//    fixquality = GPS.fixquality();
//    satellites = GPS.satellites();

    //pubGPS = String::format("L %3.4f, %3.4f, H %4.0f, Q %3.2",latitude,longitude,altitude,fixQuality);
    dispGPS1 = String::format("%3.5f %3.5f",GPS.latitudeDegrees,GPS.longitudeDegrees);
    dispGPS2 = String::format("%2.0f, %d/%d", GPS.meansignal, GPS.satellitesreceived, GPS.numSVg);
    dispGPS3 = ""; // String::format("signal: %3.3f", GPS.meansignal);
//    USBSerial1.println(dispGPS1 + dispGPS2);

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
    accel.begin();

    // listen for single-tap events at the threshold
    // keep the pin high for 1s, wait 1s between clicks
    //uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow
    accel.setClick(1, CLICKTHRESHHOLD);//, 0, 100, 50);
}



void readAccel() {
    // Read Accelerometer
    accel.read();
    //accel.read(&measx, &measy, &measz);
    measx = accel.x;
    measy = accel.y;
    measz = accel.z;

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
//    USBSerial1.println(dispAccel);

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


//      IMU BNO FUNCTIONS


void initbno()  {
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
}

void readbno()  {
  bno.getEvent(&event);

  imu_x = event.orientation.x;
  imu_y = event.orientation.y;
  imu_z = event.orientation.z;

  dispimu = String::format("x%3.1f y%3.1f z%3.1f",imu_x,imu_y,imu_z);

  /* Display the floating point data
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  */

}


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
//  display.println(dispAccel);
  display.println(dispGPS1);
  display.println(dispGPS2);
  display.println(dispimu);
  display.println(dispTrig1);
  display.println(dispTrig2);
  display.display();

}





//  GPIO FUNCTIONS

void initGPIO() {
    // POWER TEMPERATURE SENSOR
//    pinMode(A1,OUTPUT);
//    pinMode(B5,OUTPUT);
//    digitalWrite(B5, HIGH);
//    digitalWrite(A1, LOW);

    // Bue LED for blinking on accelerometer.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);


    // digitalWrite(A5, HIGH);         // power pin for water sensor  ?? This is SPI for Accel??
}


int getLevelReading() {

    //
//    int emptyLevelValue = 3500;
//    int fullLevelValue = 1800;
    // about 2 inches of water ->
    //int levelValue = analogRead(D0) - 2434;

    //delay(50);
//    int levelReading = analogRead(A4);
//    int levelValue = map(levelReading, fullLevelValue, emptyLevelValue, 0, 100);
//    levelValue = 100 - levelValue;  // flip it

//    Serial.println("water level is " + String(levelReading) + " percentage full is " + String(levelValue));

//    return levelValue;
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
            + ",\"c_unc\":" + String(fixquality)
            + ",\"c_alt\":" + String(altitude)
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
