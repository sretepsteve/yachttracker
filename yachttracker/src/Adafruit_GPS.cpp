/***********************************
This is our GPS library

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  // Only include software serial on AVR platforms (i.e. not on Due).
  #include <SoftwareSerial.h>
#endif
#include <Adafruit_GPS.h>

// how long are max NMEA lines to parse?
#define MAXLINELENGTH 120

// we double buffer: read one line in and leave one for the main program
volatile char line1[MAXLINELENGTH];
volatile char line2[MAXLINELENGTH];
// our index into filling the current line
volatile uint8_t lineidx=0;
// pointers to the double buffers
volatile char *currentline;
volatile char *lastline;
volatile boolean recvdflag;
volatile boolean inStandbyMode;


boolean Adafruit_GPS::parse(char *nmea) {
  // do checksum check
//  USBSerial1.println(nmea);
  // first look if we even have one
  if (nmea[strlen(nmea)-4] == '*') {
    uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
    sum += parseHex(nmea[strlen(nmea)-2]);

    // check checksum
    for (uint8_t i=2; i < (strlen(nmea)-4); i++) {
      sum ^= nmea[i];
    }
    if (sum != 0) {
      // bad checksum :(
      return false;
    }
  }
  int32_t degree;
  long minutes;
  char degreebuff[10];
  // look for a few common sentences
  if ((strstr(nmea, "$GPGGA")) || (strstr(nmea, "$GNGGA"))) {
    // found GGA
    // $xxGGA,time,lat,NS,long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs<CR><LF>
//    Serial.print("G");
    char *p = nmea;
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    // parse out latitude
    p = strchr(p, ',')+1;
    if (',' != *p)    // means not-empty field??
    {
      strncpy(degreebuff, p, 2);  // First two chars, degrees
      p += 2;                     // advance beyond degrees
      degreebuff[2] = '\0';       // Terminate string with null
      degree = atol(degreebuff) * 10000000;  // convert degrees to integer
      strncpy(degreebuff, p, 2); // minutes now into buffer
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);  // next 4 chars into buffer
      degreebuff[6] = '\0';       // Terminate string with null
      minutes = 50 * atol(degreebuff) / 3;   //
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
      latitudeDegrees += int(latitude/100);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') lat = 'N';
      else if (p[0] == 'S') lat = 'S';
      else if (p[0] == ',') lat = 0;
      else return false;
    }

    // parse out longitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (longitude-100*int(longitude/100))/60.0;
      longitudeDegrees += int(longitude/100);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'W') longitudeDegrees *= -1.0;
      if (p[0] == 'W') lon = 'W';
      else if (p[0] == 'E') lon = 'E';
      else if (p[0] == ',') lon = 0;
      else return false;
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      fixquality = atoi(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      satellites = atoi(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      HDOP = atof(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      altitude = atof(p);
    }

    p = strchr(p, ',')+1;
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      geoidheight = atof(p);
    }
    return true;
  }
  if ((strstr(nmea, "$GPRMC")) || (strstr(nmea, "$GNRMC"))) {
   // found RMC
   // $xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
//    Serial.print("R");
    char *p = nmea;

    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    p = strchr(p, ',')+1;
    // Serial.println(p);
    if (p[0] == 'A')
      fix = true;
    else if (p[0] == 'V')
      fix = false;
    else
      return false;

    // parse out latitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      long degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      long minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
      latitudeDegrees += int(latitude/100);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') lat = 'N';
      else if (p[0] == 'S') lat = 'S';
      else if (p[0] == ',') lat = 0;
      else return false;
    }

    // parse out longitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (longitude-100*int(longitude/100))/60.0;
      longitudeDegrees += int(longitude/100);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'W') longitudeDegrees *= -1.0;
      if (p[0] == 'W') lon = 'W';
      else if (p[0] == 'E') lon = 'E';
      else if (p[0] == ',') lon = 0;
      else return false;
    }
    // speed
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      speed = atof(p);
    }

    // angle
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      angle = atof(p);
    }

    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      uint32_t fulldate = atof(p);
      day = fulldate / 10000;
      month = (fulldate % 10000) / 100;
      year = (fulldate % 100);
    }
    // we dont parse the remaining, yet!
    // Magnetic Variation
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      magvariation = atof(p);
    }

    return true;
  }

  if (strstr(nmea, "$GPGSV")) {
   // found GPGSV
   // $xxGSV,numMsg,msgNum,numSV,{,sv,elv,az,cno},signalId*cs<CR><LF>
//    Serial.print("S");
    char *p = nmea;

    p = strchr(p, ',')+1; // numMsg
    uint8_t numMsg = atoi(p);

    p = strchr(p, ',')+1; // msgNum
    msgNum = atoi(p);
//    Serial.printf(" S%d",msgNum);

    p = strchr(p, ',')+1; // numSV
    if (',' != *p)
    {
      numSVg = atof(p);
    }
    if (msgNum <= 5) {
      for (uint8_t i = 0; i < 4; i++) {  // for each sat in the GSV line
        uint8_t s = (msgNum-1)*4 + i;
//        Serial.print(s);
        p = strchr(p, ',')+1; // sv
        if (',' != *p)
        {
          sv[s] = atoi(p);
        }
        p = strchr(p, ',')+1; // el
        if (',' != *p)
        {
          el[s] = atoi(p);
        }
        p = strchr(p, ',')+1; // az
        if (',' != *p)
        {
          az[s] = atoi(p);
        }
        p = strchr(p, ',')+1; // cno
        if (',' != *p)
        {
          cno[s] = atoi(p);
        }
//      USBSerial1.printlnf("  sv %d, el %d, az %d, cno %d", sv[s], el[s], az[s], cno[s]);

      }
    }
    meansignal = 0;
    satellitesreceived = 0;
    float sum = 0;
    for (uint8_t i = 0; i < 65; i++) {
      sum += cno[i];
//      USBSerial1.printf(",%d", cno[i]);
      if (cno[i] != 0) { satellitesreceived++;  }
      }
      meansignal = sum / satellitesreceived;
//      USBSerial1.println("");
//      Serial.println("");
//      USBSerial1.printlnf("Sats: %d, sum: %f",satellitesreceived, sum);

    return true;
  }


  return false;     //  Escape parsing routine without a sentence
}

char Adafruit_GPS::read(void) {
  char c = 0;

  if (paused) return c;

#if defined(__AVR__) && defined(USE_SW_SERIAL)
  if(gpsSwSerial) {
    if(!gpsSwSerial->available()) return c;
    c = gpsSwSerial->read();
  } else
#endif
  {
    if(!gpsHwSerial->available()) return c;
    c = gpsHwSerial->read();
  }

  //Serial.print(c);

 // if (c == '$') {         //please don't eat the dollar sign - rdl 9/15/14
 //    currentline[lineidx] = 0;
 //    lineidx = 0;
 // }
  if (c == '\n') {
    currentline[lineidx] = 0;

    if (currentline == line1) {
      currentline = line2;
      lastline = line1;
    } else {
      currentline = line1;
      lastline = line2;
    }

    //Serial.println("----");
    //Serial.println((char *)lastline);
    //Serial.println("----");
    lineidx = 0;
    recvdflag = true;
  }

  currentline[lineidx++] = c;
  if (lineidx >= MAXLINELENGTH)
    lineidx = MAXLINELENGTH-1;

  return c;
}

#if defined(__AVR__) && defined(USE_SW_SERIAL)
// Constructor when using SoftwareSerial or NewSoftSerial
#if ARDUINO >= 100
Adafruit_GPS::Adafruit_GPS(SoftwareSerial *ser)
#else
Adafruit_GPS::Adafruit_GPS(NewSoftSerial *ser)
#endif
{
  common_init();     // Set everything to common state, then...
  gpsSwSerial = ser; // ...override gpsSwSerial with value passed.
}
#endif

// Constructor when using HardwareSerial
Adafruit_GPS::Adafruit_GPS(HardwareSerial *ser) {
  common_init();  // Set everything to common state, then...
  gpsHwSerial = ser; // ...override gpsHwSerial with value passed.
}

// Initialization code used by all constructor types
void Adafruit_GPS::common_init(void) {
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  gpsSwSerial = NULL; // Set both to NULL, then override correct
#endif
  gpsHwSerial = NULL; // port pointer in corresponding constructor
  recvdflag   = false;
  paused      = false;
  lineidx     = 0;
  currentline = line1;
  lastline    = line2;

  hour = minute = seconds = year = month = day =
    fixquality = satellites = 0; // uint8_t
  lat = lon = mag = 0; // char
  fix = false; // boolean
  milliseconds = 0; // uint16_t
  latitude = longitude = geoidheight = altitude =
    speed = angle = magvariation = HDOP = 0.0; // float
}

void Adafruit_GPS::begin(uint32_t baud)
{
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  if(gpsSwSerial)
    gpsSwSerial->begin(baud);
  else
#endif
    gpsHwSerial->begin(baud);

  delay(10);
}

void Adafruit_GPS::sendCommand(const char *str) {
#if defined(__AVR__) && defined(USE_SW_SERIAL)
  if(gpsSwSerial)
    gpsSwSerial->println(str);
  else
#endif
    gpsHwSerial->println(str);
}

boolean Adafruit_GPS::newNMEAreceived(void) {
  return recvdflag;
}

void Adafruit_GPS::pause(boolean p) {
  paused = p;
}

char *Adafruit_GPS::lastNMEA(void) {
  recvdflag = false;
  return (char *)lastline;
}

// read a Hex value and return the decimal equivalent
uint8_t Adafruit_GPS::parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

boolean Adafruit_GPS::waitForSentence(const char *wait4me, uint8_t max) {
  char str[20];

  uint8_t i=0;
  while (i < max) {
    if (newNMEAreceived()) {
      char *nmea = lastNMEA();
      strncpy(str, nmea, 20);
      str[19] = 0;
      i++;

      if (strstr(str, wait4me))
	return true;
    }
  }

  return false;
}

boolean Adafruit_GPS::LOCUS_StartLogger(void) {
  sendCommand(PMTK_LOCUS_STARTLOG);
  recvdflag = false;
  return waitForSentence(PMTK_LOCUS_STARTSTOPACK);
}

boolean Adafruit_GPS::LOCUS_StopLogger(void) {
  sendCommand(PMTK_LOCUS_STOPLOG);
  recvdflag = false;
  return waitForSentence(PMTK_LOCUS_STARTSTOPACK);
}

boolean Adafruit_GPS::LOCUS_ReadStatus(void) {
  sendCommand(PMTK_LOCUS_QUERY_STATUS);

  if (! waitForSentence("$PMTKLOG"))
    return false;

  char *response = lastNMEA();
  uint16_t parsed[10];
  uint8_t i;

  for (i=0; i<10; i++) parsed[i] = -1;

  response = strchr(response, ',');
  for (i=0; i<10; i++) {
    if (!response || (response[0] == 0) || (response[0] == '*'))
      break;
    response++;
    parsed[i]=0;
    while ((response[0] != ',') &&
	   (response[0] != '*') && (response[0] != 0)) {
      parsed[i] *= 10;
      char c = response[0];
      if (isDigit(c))
        parsed[i] += c - '0';
      else
        parsed[i] = c;
      response++;
    }
  }
  LOCUS_serial = parsed[0];
  LOCUS_type = parsed[1];
  if (isAlpha(parsed[2])) {
    parsed[2] = parsed[2] - 'a' + 10;
  }
  LOCUS_mode = parsed[2];
  LOCUS_config = parsed[3];
  LOCUS_interval = parsed[4];
  LOCUS_distance = parsed[5];
  LOCUS_speed = parsed[6];
  LOCUS_status = !parsed[7];
  LOCUS_records = parsed[8];
  LOCUS_percent = parsed[9];

  return true;
}

// Standby Mode Switches
boolean Adafruit_GPS::standby(void) {
  if (inStandbyMode) {
    return false;  // Returns false if already in standby mode, so that you do not wake it up by sending commands to GPS
  }
  else {
    inStandbyMode = true;
    sendCommand(PMTK_STANDBY);
    //return waitForSentence(PMTK_STANDBY_SUCCESS);  // don't seem to be fast enough to catch the message, or something else just is not working
    return true;
  }
}

boolean Adafruit_GPS::wakeup(void) {
  if (inStandbyMode) {
   inStandbyMode = false;
    sendCommand("");  // send byte to wake it up
    return waitForSentence(PMTK_AWAKE);
  }
  else {
      return false;  // Returns false if not in standby mode, nothing to wakeup
  }
}
