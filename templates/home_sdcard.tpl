{ "model":"homeSdcard" }
/*
  senseBox:home - Citizen Sensingplatform
  Version: sdcard_1.0
  Date: 2017-09-23
  Homepage: https://www.sensebox.de https://www.opensensemap.org
  Author: Institute for Geoinformatics, University of Muenster
  Note: Sketch for senseBox:home Ethernet Edition
  Model: homeEthernet
  Email: support@sensebox.de
  Code is in the public domain.
  https://github.com/sensebox/node-sketch-templater
*/

// TODO: set up RTC for timestamps!


#include <SD.h>

/* ------------------------------------------------------------------------- */
/* ------------------------------Configuration------------------------------ */
/* ------------------------------------------------------------------------- */

// Interval of measuring and submitting values in seconds
const unsigned int postingInterval = 60e3;

// senseBox ID
const char SENSEBOX_ID[] PROGMEM = "@@SENSEBOX_ID@@";

// Number of sensors
// Change this number if you add or remove sensors
// do not forget to remove or add the sensors on opensensemap.org
static const uint8_t NUM_SENSORS = @@NUM_SENSORS@@;

// sensor IDs
@@SENSOR_IDS|toProgmem@@

// Uncomment the next line to get debugging messages printed on the Serial port
// Do not leave this enabled for long time use
// #define ENABLE_DEBUG

const char LOGFILE_PATH[] PROGMEM = "datalog.csv";

/* ------------------------------------------------------------------------- */
/* --------------------------End of Configuration--------------------------- */
/* ------------------------------------------------------------------------- */

#include "BMP280.h"
#include <HDC100X.h>
#include <Makerblog_TSL45315.h>
#include <SPI.h>
#include <VEML6070.h>
#include <Wire.h>
#include <avr/wdt.h>

#ifdef ENABLE_DEBUG
#define DEBUG(str) Serial.println(str)
#else
#define DEBUG(str)
#endif

// Sensor Instances
Makerblog_TSL45315 TSL = Makerblog_TSL45315(TSL45315_TIME_M4);
HDC100X HDC(0x43);
BMP280 BMP;
VEML6070 VEML;

File logfile;

typedef struct measurement {
  const char *sensorId;
  float value;
} measurement;

measurement measurements[NUM_SENSORS];
uint8_t num_measurements = 0;

// buffer for sprintf
char buffer[150];

void addMeasurement(const char *sensorId, float value) {
  measurements[num_measurements].sensorId = sensorId;
  measurements[num_measurements].value = value;
  num_measurements++;
}

/**
 * copied from SD example sketch
 * created  28 Mar 2011
 * by Limor Fried
 * modified 9 Apr 2012
 * by Tom Igoe
 */
bool checkSdcard (uint8_t chipSelect) {
  // set up variables using the SD utility library functions:
  Sd2Card card;
  SdVolume volume;
  SdFile root;

  DEBUG_OUT.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    DEBUG_OUT.println("initialization failed. Things to check:");
    DEBUG_OUT.println("* is a card inserted?");
    DEBUG_OUT.println("* is your wiring correct?");
    DEBUG_OUT.println("* did you change the chipSelect pin to match your shield or module?");
    return false;
  } else {
    DEBUG_OUT.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  DEBUG_OUT.print("\tCard type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      DEBUG_OUT.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      DEBUG_OUT.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      DEBUG_OUT.println("SDHC");
      break;
    default:
      DEBUG_OUT.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    DEBUG_OUT.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return false;
  }

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  DEBUG_OUT.print("\tVolume type is FAT");
  DEBUG_OUT.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
  volumesize *= volume.clusterCount();     // we'll have a lot of clusters
  volumesize *= 512;                       // SD card blocks are always 512 bytes
  DEBUG_OUT.print("\tVolume size (bytes): ");
  DEBUG_OUT.print(volumesize);
  DEBUG_OUT.print(" bytes (");
  volumesize /= 1024;
  volumesize /= 1024;
  DEBUG_OUT.print(volumesize);
  DEBUG_OUT.println(" MB)");

  return true;
}

void writeLogToStream() {
  // iterate through the measurements array
  for (uint8_t i = 0; i < num_measurements; i++) {
    sprintf_P(buffer, PSTR("%S,"), measurements[i].sensorId);
    // arduino sprintf just returns "?" for floats, use dtostrf
    dtostrf(measurements[i].value, 9, 2, &buffer[strlen(buffer)]);

    // sensor ID
    stream.print(&measurements[i].sensorId)
    stream.print(',');

    // measurement
    stream.print(measurements[i].value, 2);
    stream.print(',');

    // ISODate
    // FIXME: get from RTC, not GPS??
    // stream.print(gps.date.year());
    // stream.print(F("-"));
    // if (gps.date.month() < 10) stream.print(F("0"));
    // stream.print(gps.date.month());
    // stream.print(F("-"));
    // if (gps.date.day() < 10) stream.print(F("0"));
    // stream.print(gps.date.day());
    // stream.print(F("T"));
    // stream.print(gps.time.hour());
    // stream.print(F(":"));
    // if (gps.time.minute() < 10) stream.print(F("0"));
    // stream.print(gps.time.minute());
    // stream.print(F(":"));
    // if (gps.time.second() < 10) stream.print(F("0"));
    // stream.print(gps.time.second());
    // stream.print(F("Z"));
    // stream.print(',');

    stream.print(millis(), 2);
    stream.print(',');

    // // location
    // stream.print(String(gps.location.lng(), 6));
    // stream.print(',');
    // stream.print(String(gps.location.lat(), 6));

    stream.println();
  }
}

void submitValues() {
  #ifdef ENABLE_DEBUG
  writeLogToStream(Serial);
  #endif

  logfile = SD.open(LOGFILE_PATH, FILE_WRITE);
  if (logfile) {
    DEBUG("writing to logfile ");
    DEBUG(LOGFILE_PATH);
    DEBUG(" ... ");

    write_log_to_stream(logfile);

    logfile.close();
    DEBUG("done.");
    // reset number of measurements
    num_measurements = 0;
  } else {
    // FIXME: this will never be evaluated, even if the card is removed??
    DEBUG("error opening logfile. rebooting");
    reboot();
  }
}

void reboot () {
  delay(5000);
  cli();
  wdt_enable(WDTO_60MS);
  while (1) ;
}

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);

  DEBUG(F("Initializing SDCARD..."));
  if (checkSdcard(SD_CHIPSELECT)) {
    reboot();
  }

  if (!SD.begin(SD_CHIPSELECT)) {
    DEBUG(F("can't open SD card, though a card was found..."));
    reboot();
  }

  // Sensor initialization
  DEBUG(F("Initializing sensors..."));
  VEML.begin();
  delay(500);
  HDC.begin(HDC100X_TEMP_HUMI, HDC100X_14BIT, HDC100X_14BIT, DISABLE);
  TSL.begin();
  BMP.begin();
  BMP.setOversampling(4);
  DEBUG(F("done!"));
  DEBUG(F("Starting loop in 30 seconds."));
  HDC.getTemp();
  delay(30000);
}

void loop() {
  // capture loop start timestamp
  unsigned long start = millis();

  // read measurements from sensors
  addMeasurement(TEMPERSENSOR_ID, HDC.getTemp());
  delay(200);
  addMeasurement(RELLUFSENSOR_ID, HDC.getHumi());

  double tempBaro, pressure;
  char result;
  result = BMP.startMeasurment();
  if (result != 0) {
    delay(result);
    result = BMP.getTemperatureAndPressure(tempBaro, pressure);
    addMeasurement(LUFTDRSENSOR_ID, pressure);
  }

  addMeasurement(BELEUCSENSOR_ID, TSL.readLux());
  addMeasurement(UVINTESENSOR_ID, VEML.getUV());

  submitValues();

  // schedule next round of measurements
  for (;;) {
    unsigned long now = millis();
    unsigned long elapsed = now - start;
    if (elapsed >= postingInterval)
      return;
  }
}
