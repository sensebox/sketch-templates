/*
  senseBox:home - Citizen Sensing Platform
  Version: ethernetv2_0.2
  Date: 2018-07-20
  Homepage: https://www.sensebox.de https://www.opensensemap.org
  Author: Reedu GmbH & Co. KG
  Note: Sketch for senseBox:home Ethernet MCU Edition
  Model: homeV2Ethernet
  Email: support@sensebox.de
  Code is in the public domain.
  https://github.com/sensebox/node-sketch-templater
*/

#include <Ethernet2.h>

/* ------------------------------------------------------------------------- */
/* ------------------------------Configuration------------------------------ */
/* ------------------------------------------------------------------------- */

//Sensor Setup
#define HDC_CONNECTED
#define BMP_CONNECTED
#define TSL_VEML_CONNECTED
#define SDS_CONNECTED 

// Absolute number of sensors registered on openSenseMap
static const uint8_t NUM_SENSORS = @@NUM_SENSORS@@; 

//ID of Serial Port where the SDS is connected
#ifdef SDS_CONNECTED
#define SDS_UART_PORT (@@SERIAL_PORT@@) 
#endif

//Configure static IP setup (only needed if DHCP is disabled)
IPAddress myIp(192, 168, 0, 42);
IPAddress myDns(8, 8, 8, 8);
IPAddress myGateway(192, 168, 0, 177);
IPAddress mySubnet(255, 255, 255, 0);

// MAC Address
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
EthernetClient client;

// Interval of measuring and submitting values in seconds
const unsigned int postingInterval = 10e3;

// address of the server to send to
const char server[] PROGMEM = "@@INGRESS_DOMAIN@@";

// senseBox ID
const char SENSEBOX_ID[] PROGMEM = "@@SENSEBOX_ID@@"; //Lange N8 der Wissenschaft

// sensor IDs
@@SENSOR_IDS|toProgmem@@

/* ------------------------------------------------------------------------- */
/* --------------------------End of Configuration--------------------------- */
/* ------------------------------------------------------------------------- */

//Libraries
#include <senseBoxIO.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HDC1000.h>
#include <Makerblog_TSL45315.h>
#include <VEML6070.h>
#include <SDS011-select-serial.h>

// Sensor Instances
#ifdef HDC_CONNECTED
Adafruit_HDC1000 HDC = Adafruit_HDC1000();
#endif
#ifdef BMP_CONNECTED
Adafruit_BMP280 BMP;
#endif
#ifdef TSL_VEML_CONNECTED
Makerblog_TSL45315 TSL = Makerblog_TSL45315(TSL45315_TIME_M4);
VEML6070 VEML;
#endif
#ifdef SDS_CONNECTED
SDS011 SDS(SDS_UART_PORT);
#endif

int dataLength;

typedef struct measurement {
  const char *sensorId;
  float value;
} measurement;

measurement measurements[NUM_SENSORS];
uint8_t num_measurements = 0;

// buffer for sprintf
char buffer[200];
char measurementsBuffer[NUM_SENSORS * 35];

void addMeasurement(const char *sensorId, float value) {
  Serial.print(F("Value: "));
  Serial.println(value);
  measurements[num_measurements].sensorId = sensorId;
  measurements[num_measurements].value = value;
  num_measurements++;
  dataLength += 24 + 1; //length of ID + ','
  if (value == 0) dataLength += 4; // 0.00
  else 
    //dataLength += String((int)value * 100).length() + 1; //length of measurement value + decimal digit
  // to test:
    dataLength += String((int)value).length() + 3;
}

void writeMeasurementsToClient() {
  // iterate throug the measurements array 
  for (uint8_t i = 0; i < num_measurements; i++) 
  {
    //convert float to char[]
    float temp = measurements[i].value;
    int intPart = (int)measurements[i].value;
    temp -= intPart;
    temp *= 100; //2 decimal places
    int fracPart = (int)temp;
    sprintf_P(buffer, PSTR("%s,%i.%02i\n"), measurements[i].sensorId, intPart, fracPart);
    // transmit buffer to client
    client.print(buffer);
    Serial.print(buffer);
    //dataLength += String(buffer).length();
  }

  // reset num_measurements
  num_measurements = 0;
}

void submitValues() {
  Serial.println(dataLength);
  // close any connection before send a new request.
  if (client.connected()) {
    client.stop();
    delay(1000);
  }
  bool connected = false;
  char _server[strlen_P(server)];
  strcpy_P(_server, server);
  for (uint8_t timeout = 2; timeout != 0; timeout--) {
    Serial.println(F("connecting..."));
    connected = client.connect(_server, 80);
    if (connected == true) {
      Serial.println(F("Connection successful, transferring..."));
      // construct the HTTP POST request:
      sprintf_P(buffer,
                PSTR("POST /boxes/%s/data HTTP/1.1\nHost: %s\nContent-Type: "
                     "text/csv\nConnection: close\nContent-Length: %i\n\n"),
                SENSEBOX_ID, server, dataLength);
      Serial.print(buffer);

      // send the HTTP POST request:
      client.print(buffer);

      // send measurements
      writeMeasurementsToClient();

      // send empty line to end the request
      client.println();

      uint16_t timeout = 0;
      // allow the response to be computed

      while (timeout <= 5000) {
        delay(10);
        timeout = timeout + 10;
        //                Serial.println(timeout);
        if (client.available()) {
          break;
        }
      }
      delay(1000);
      while (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if the server's disconnected, stop the client:
        if (!client.connected()) {
          Serial.println();
          Serial.println("disconnecting from server.");
          client.stop();
          break;
        }
      }Serial.println("done!");

      // reset number of measurements
      num_measurements = 0;
      break;
    }
    delay(1000);
  }

  if (connected == false) {
    // Reset durchführen
    Serial.println(F("connection failed. Restarting..."));
    delay(5000);
    noInterrupts();
    NVIC_SystemReset();
    while (1)
      ;
  }
}

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  while(!Serial);
  Serial.println(F("----senseBox:home V2----\n"));
  delay(100);
  #ifdef SDS_CONNECTED
  SDS_UART_PORT.begin(9600);
  delay(1000);
  #endif

  Serial.print(F("xBee1 spi enable..."));
  senseBoxIO.SPIselectXB1(); // select XBEE1 spi
  Serial.println(F("done."));
  senseBoxIO.powerXB1(false);delay(200);
  Serial.print(F("xBee1 power on..."));
  senseBoxIO.powerXB1(true); // power ON XBEE1
  Serial.println(F("done"));
  senseBoxIO.powerI2C(false);delay(200);
  senseBoxIO.powerI2C(true);

  Ethernet.init(23);
  delay(100);
  // start the Ethernet connection:
  Serial.print(F("Trying to configure Ethernet using DHCP..."));
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("failed! Please check your cable connection."));
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, myIp);
  }else Serial.println(F("done."));
  // give the Ethernet shield a second to initialize:
  delay(1000);
  // init I2C/wire library
  Wire.begin();
  // Sensor initialization
  Serial.println(F("Initializing sensors:"));
  #ifdef HDC_CONNECTED
  Serial.print(F("HDC (Temp/Humi Sensor)..."));
  HDC.begin();
  Serial.println(F("done."));
  delay(50);
  #endif
  #ifdef BMP_CONNECTED
  Serial.print(F("BMP (Pressure Sensor)..."));
  BMP.begin(0x76);
  Serial.println(F("done."));
  delay(50);
  #endif
  #ifdef TSL_VEML_CONNECTED
  Serial.print(F("TSL (LUX-Sensor)..."));
  TSL.begin();
  Serial.println(F("done."));
  delay(50);
  Serial.print(F("VEML (UV-Sensor)..."));
  VEML.begin();
  Serial.println(F("done."));
  delay(50);
  #endif
  Serial.println(F("done!\n"));
  Serial.println(F("Starting loop in 3 seconds."));
  delay(3000);
}

void loop() {
  // capture loop start timestamp
  unsigned long start = millis();
  dataLength = NUM_SENSORS - 1; // excluding linebreak after last measurement

  // read measurements from sensors
  #ifdef HDC_CONNECTED
    float temp = HDC.readTemperature();
    addMeasurement(TEMPERSENSOR_ID, temp);
    delay(200);
    float humi = HDC.readHumidity();
    addMeasurement(RELLUFSENSOR_ID, humi);
  #endif
  #ifdef BMP_CONNECTED
    float tempBaro, pressure, altitude;
    tempBaro = BMP.readTemperature();
    pressure = BMP.readPressure()/100;
    altitude = BMP.readAltitude(1013.25); //1013.25 = sea level pressure
    addMeasurement(LUFTDRSENSOR_ID, pressure);
  #endif
  #ifdef TSL_VEML_CONNECTED
    uint32_t lux = TSL.readLux();
    addMeasurement(BELEUCSENSOR_ID, lux);
    float uv = VEML.getUV();
    addMeasurement(UVINTESENSOR_ID, uv);
  #endif
  #ifdef SDS_CONNECTED
  uint8_t attempt = 0;
  float pm10, pm25;
  while (attempt < 5) {
    bool error = SDS.read(&pm25, &pm10);
    if (!error) {
      addMeasurement(PM10SENSOR_ID, pm10);
      Serial.print("PM10: ");
      Serial.println(pm10);
      addMeasurement(PM25SENSOR_ID, pm25);
      Serial.print("PM2.5: ");
      Serial.println(pm25);
      break;
    }
    attempt++;
  }
  #endif
  Serial.println(F("Submit values"));
  submitValues();
  // schedule next round of measurements
  for (;;) {
    unsigned long now = millis();
    unsigned long elapsed = now - start;
    if (elapsed >= postingInterval)
      return;
  }
}
