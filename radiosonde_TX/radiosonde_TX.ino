/* UNIS Radiosonde transmitter software

  The hardware comprises an Adafruit Feather M0 board with a built-in LoRa radio. The additional components
  are a GPS, several thermistors, BME280 sensors (pressure, temperature, humidity).

  The Norwegian regulations on the use of the 433MHz ISM band state a maximum transmit power of 10mW ERP as
  well as a maximum 10% air time.

  So, assuming we are using a quarter-wave monopole antenna (gain 5.19dBi), then using
  the maximum transmit power 10mW ERP = 10dBm ERP = 12.15dBm EIRP, the LoRa transceiver can
  use up to 7dBm transmit power.


  Hardware connections for BME280 are as follows

  Feather pin      BM280 pin
   3V3               VIN   (purple wire)
   GND               GND   (black)
   SCL  (32)         SCK   (white)
   SDA  (31)         SDI   (grey)
                     CS -> connect to VIN to properly disengage SPI-mode, we use I2C.

  The GPS is connected to hardware serial RX0 and TX0 (pin 16)

  The SHT85 sensor is connected to I2C - note that while Adafruit modules, such as the BME280 board, do not
  need extra pull-up resistors, SHT85 does need 10k pull-ups for SCL and SDA.
*/



//#define DEBUG // Comment out when not connected to the computer

// Callsign for the radiosonde (six characters)
const char callsign[] = "unis02";

// Time between each transmission (ms)
#define INTERVAL 1000

// Watchdog timer (ms)
#define WATCHDOG 5000  // If you don't "pet" the watchdog within this time period, it will bark and reset the system

// Frequency to use
#define RF95_FREQ 434.2

// Pin definitions for Adafruit Feather M0 board
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define VBAT_PIN A7 // Half of the battery voltage, hardwired in Adafruit Feather-boards
#define NTC_EXT A1
#define NTC_INT A0

// Pin definitions for extra LEDs
#define BME_ERROR 5
#define SHT_ERROR 6

// Basic parameters for the thermistors for quick temperature calculations during debugging
#define T25 298.15

// The internal thermistor is a Kyocera AVX NJ28QA0503FCC, 50k at room temperature
// The fixed resistor in the voltage divider is 47k
#define BETA_INT 4250
#define R25_INT 50000.0
#define RINT_FIXED 47000.0

// The external thermistor is a TE Connectivity GA10K3A1IA, 10k at room temperature
// The fixed resistor is 10k

#define BETA_EXT 3976.0
#define R25_EXT 10000.0
#define REXT_FIXED 10000.0


// Housekeeping flags: if the bit is set, then there is a problem
#define GPSNOFIX 0
#define BME280FAULT 1
#define SHT85FAULT 2


//============================================================================================================
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>
#include <RH_RF95.h>          // https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html
#include <TinyGPS++.h>        // http://arduiniana.org/libraries/tinygpsplus/
#include <Adafruit_BME280.h>  // https://github.com/adafruit/Adafruit_BME280_Library
#include "SHTSensor.h"        // https://github.com/Sensirion/arduino-sht

#ifndef DEBUG
#include <Adafruit_SleepyDog.h> // https://github.com/adafruit/Adafruit_SleepyDog
#endif

//-----------------------------------------------------------------------------
// Get ready to use the LoRA transceiver, BME280, SHT85 and GPS

RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

SHTSensor sht; // The Sensirion SHT85 humidity sensor

TinyGPSPlus gps;


//-------------------------------------------------------------------------------------
// Telemetry packet variables


uint8_t tx_buffer[100];       // Copy data to this buffer when preparing the telemetry package
uint8_t tx_counter;         // Number of bytes to be sent
unsigned long thistime, last_tx_time; // Internal timekeeping variables for measuring time-on-air

uint16_t packetnum;
uint8_t housekeeping;
float latitude;
float longitude;
float altitude_meters;
uint8_t hours;
uint8_t minutes;
uint8_t seconds;
uint8_t number_of_satellites; // This provides additional info about the quality of the GPS fix
float pressure;  // Read from BME280 if it is working, otherwise from SHT85
float humidity;  // Read from SHT85 if it is working, otherwise from BME280
float temperature; // Read SHT85 if it is working, otherwise from BME280
uint16_t ntc_internal; // Internal 50k NTC with a 47k fixed resistor
uint16_t ntc_external; // External 10k NTC with a 10k fixed resistor
uint16_t battery_level;

//---------------------------------------------------------------------------------------
// In case the LoRa transceiver has problems, the only easy way to find it out is to
// blink the LED on the board. The telemetry package provides information for other
// failures

void errorBlinker_LoRa(void)
{
#ifdef DEGUG
  Serial.println("**************LoRA ERROR");
#endif
  while (1) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

//---------------------------------------------------------------------------------------------------------
// Init the BM280 pressure, humidity and temperature sensor

void initBME280(void)
{
  if (!bme.begin()) {
#ifdef DEBUG
    Serial.println("BME280 init failed...");
#endif
    bitSet(housekeeping, BME280FAULT); // BME failure detected
    digitalWrite(BME_ERROR, HIGH);
    return;
  }

#ifdef DEBUG
  Serial.println("BME280 init ok");
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
  bme_temp->printSensorDetails();
#endif

  // Instead of just using the standard values, see chapter
  // 3.5 "Recommended modes of operation" in the datasheet
}

//---------------------------------------------------------------------------------------------------------
// Init the SHT85 humidity sensor

void initSHT85(void)
{
  if (!sht.init()) {
#ifdef DEBUG
    Serial.println("SHT85 init failed...");
#endif
    bitSet(housekeeping, SHT85FAULT); // SHT85 error detected
    digitalWrite(SHT_ERROR, HIGH);
    return;
  }
#ifdef DEBUG
  Serial.println("SHT85 init ok");
#endif
}


//---------------------------------------------------------------------------------------------------------
// Init the LoRa transceiver
// - if the transceiver init fails, this routine will stop *everything* and remain blinking the LED

void initLoRa(void)
{
  // The defaults are
  // - mode BW=125kHz, CR=4/5, Sf=128 chips/symbol (SF=7, 2^7=128), CRC on
  // - frequency 434MHz
  // - TX power 13dBm
  //
  // See also  https://medium.com/home-wireless/testing-lora-radios-with-the-limesdr-mini-part-2-37fa481217ff

  digitalWrite(RFM95_RST, LOW); // Reset the transceiver first
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
#ifdef DEBUG
    Serial.println("LoRa: init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
#endif
    errorBlinker_LoRa();
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM, so we need some additional settings

  rf95.setSignalBandwidth(125000); // kHz
  rf95.setCodingRate4(5); // CR=4/5
  rf95.setSpreadingFactor(7); // 2^7=128 chips/symbol
  //
  if (!rf95.setFrequency(RF95_FREQ)) {
#ifdef DEBUG
    Serial.println("LoRa: setFrequency failed");
#endif
    errorBlinker_LoRa();
  }
#ifdef DEBUG
  Serial.print("LoRa: frequency set : ");
  Serial.println(RF95_FREQ);
#endif

  rf95.setTxPower(7, false); // The Norwegian regulations state max 10mW ERP...
}

//-------------------------------------------------------------------------------------
// Using a simple formula, convert the raw adc-value into degrees Celsius
// This function is only used when debugging

#ifdef DEBUG

float temperatureNTC(uint16_t adc, float beta, float r25, float rfixed)
{
  float a = adc / 4095.0;
  float r = a / (1.0 - a) * rfixed;
  float T = beta / (logf(r / r25) + beta / T25);
  return T - 273.15;
}

#endif


//=========================================================================================================
// After reset, start setting up the subsystems and sensors.

void setup()
{
#ifndef DEBUG
  Watchdog.disable();
#endif
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(RFM95_CS, OUTPUT);
  digitalWrite(RFM95_CS, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(BME_ERROR, OUTPUT);
  digitalWrite(BME_ERROR, LOW);

  pinMode(SHT_ERROR, OUTPUT);
  digitalWrite(SHT_ERROR, LOW);


  // Blink a few times quickly at every reset: useful when debugging weird watchdog resets...
  for (uint8_t i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(BME_ERROR, HIGH);
    digitalWrite(SHT_ERROR, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(BME_ERROR, LOW);
    digitalWrite(SHT_ERROR, LOW);
    delay(100);
  }

#ifndef DEBUG
  int watchms = Watchdog.enable(WATCHDOG);
#endif

  analogReference(AR_DEFAULT); // This uses the power supply 3.3V as the reference, fine for thermistors
  // analogReference(AR_INTERNAL2V23); // but if measuring the battery, should we use this instead?
  analogReadResolution(12);

#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) { // Wait for the connection
    delay(1);
  }
#endif

  Serial1.begin(9600); // Hardware serial for GPS

  initLoRa();
  initBME280();
  initSHT85();
}

//--------------------------------------------------------------------------------------------------------
// All analog voltage measurements are actually an average of 16 measurements
// Note that there was a bug in the SAMD21 Arduino library that resulted in incorrect ADC-values!
// If the temperature measurements seem odd, then it is likely you should update the Arduino software...

uint16_t readAnalogSensor(int pin)
{
  uint32_t value = 0;
  for (uint8_t i = 0; i < 16; i++) {
    value += analogRead(pin);
  }
  value = value >> 4; // Divide by 2^4=16

  return (uint16_t)value;
}

//--------------------------------------------------------------------------------------------------------
// Update pressure values by reading from BME280
// - if there is a fault with SHT85, use the humidity and temperature from BME280 instead

void updateBME(void)
{
  if (bitRead(housekeeping, BME280FAULT) == 0) {
    pressure = bme.readPressure() / 100.0; // Convert to hPa
    if (bitRead(housekeeping, SHT85FAULT) == 1) {
      humidity = bme.readHumidity(); // If the SHT85 is not working, we'll use BME280 instead
      temperature = bme.readTemperature();
    }
  } else {
    temperature = -999;
    pressure = -1;
    humidity = -1;
  }
}

//--------------------------------------------------------------------------------------------------------
// Update humidity from SHT85

void updateSHT85(void)
{
  if (bitRead(housekeeping, SHT85FAULT) == 0) {

    if (sht.readSample()) {
      humidity = sht.getHumidity();
      temperature = sht.getTemperature();
    } else {
      bitSet(housekeeping, SHT85FAULT);
      digitalWrite(SHT_ERROR, HIGH);

#ifdef DEBUG
      Serial.println("*** ERROR reading SHT85");
#endif
    }
  }
}


// -------------------------------------------------------------------------------------------------------
// Update position information from the GPS
void updateGPS(void)
{
  // Check for new data from GPS
  while (Serial1.available())
    gps.encode(Serial1.read());

  // The location is considered valid only if it sufficiently recent. As we expect new data once per
  // second, if the last data is older than 1.5 seconds, then it should be considered bad i.e. "no GPS lock"

  if (gps.location.isValid() && gps.location.age() < 1500) {
    latitude = (float)gps.location.lat();
    longitude = (float)gps.location.lng();
    altitude_meters = (float)gps.altitude.meters();
    hours = (uint8_t)gps.time.hour();
    minutes = (uint8_t)gps.time.minute();
    seconds = (uint8_t)gps.time.second();
    number_of_satellites = (uint8_t)gps.satellites.value();
    bitClear(housekeeping, GPSNOFIX); // We've got a fix
  } else {
    bitSet(housekeeping, GPSNOFIX); // Indicate that there is no GPS fix
    latitude = 0;
    longitude = 0;
    altitude_meters = 0;
    hours = 0;
    minutes = 0;
    seconds = 0;
    number_of_satellites = 0;
  }
}


//=================================================================================================================
//=================================================================================================================
// The measurement loop should not to run at "full speed". The first obvious reason is to reduce the power
// consumption. We are essentially active only for a short time (read the sensors and transmit data for less
// than 100ms every second). So, a good approach would be to "sleep" when not doing anything.
// Practically all modern microcontrollers have a sleep/hibernate mode where the power consumption is reduced to
// a fraction of the normal.
//
// The second reason is related to the measurements themselves. E.g. the datasheet for SHT85 recommends a max 10% duty cycle to
// keep the self-heating of the sensor below 0.1 degrees.
//
// However, there is a complicating factor, which is the GPS. For the decoding of the NMEA data to work correctly,
// one needs to call gps.encode(Serial1.read()) "frequently enough" to not miss any characters. At 9600bps, one
// character (start bit + 8 bits + stop bit) takes about 1ms. Most serial line implementations have an RX buffer
// whose size can be adjusted. But whether one can put the microcontroller to sleep and still receive everything
// correctly is not obvious. To confuse things even more, Adafruit's M0 featherboards differ from Arduino M0 boards
// in how the hardware serial line is implemented: one should take a closer look at the source code.
//
// Nevertheless, with a 1200mAh LiPo, the electronics will run for several hours and optimising the power consumption
// is not really necessary. So, if we wish to have fresh data once per second, we can use that as a
// guideline. So, sensors are read once per second, after which the data are sent to the ground station.


void loop()
{
  updateGPS();

  // Check whether it is time to send new data. Nominal measurement cycle is once per second.
  thistime = millis();
  if (thistime - last_tx_time > INTERVAL) {

    // Do all measurements first, so that all measurements are up-to-date if we will
    // be transmitting fresh data
    updateBME(); // Pressure, humidity and temperature from BME280
    updateSHT85();
    ntc_internal = readAnalogSensor(NTC_INT);
    ntc_external = readAnalogSensor(NTC_EXT);
    battery_level = readAnalogSensor(VBAT_PIN);

    // Prepare the data packet to be sent
    packetnum++;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#ifndef DEBUG
    Watchdog.reset(); // Make sure the watchdog is still sleeping...
#endif

    tx_counter = 0;

    // The callsign is a string, do not transmit '\0', the end of string character
    memcpy(&tx_buffer[tx_counter], callsign, sizeof(callsign) - 1);
    tx_counter += sizeof(callsign) - 1;

    memcpy(&tx_buffer[tx_counter], &packetnum, sizeof(packetnum));
    tx_counter += sizeof(packetnum);

    memcpy(&tx_buffer[tx_counter], &housekeeping, sizeof(housekeeping));
    tx_counter += sizeof(housekeeping);

    memcpy(&tx_buffer[tx_counter], &latitude, sizeof(latitude));
    tx_counter += sizeof(latitude);

    memcpy(&tx_buffer[tx_counter], &longitude, sizeof(longitude));
    tx_counter += sizeof(longitude);

    memcpy(&tx_buffer[tx_counter], &altitude_meters, sizeof(altitude_meters));
    tx_counter += sizeof(altitude_meters);

    memcpy(&tx_buffer[tx_counter], &hours, sizeof(hours));
    tx_counter += sizeof(hours);

    memcpy(&tx_buffer[tx_counter], &minutes, sizeof(minutes));
    tx_counter += sizeof(minutes);

    memcpy(&tx_buffer[tx_counter], &seconds, sizeof(seconds));
    tx_counter += sizeof(seconds);

    memcpy(&tx_buffer[tx_counter], &number_of_satellites, sizeof(number_of_satellites));
    tx_counter += sizeof(number_of_satellites);

    memcpy(&tx_buffer[tx_counter], &pressure, sizeof(pressure));
    tx_counter += sizeof(pressure);

    memcpy(&tx_buffer[tx_counter], &humidity, sizeof(humidity));
    tx_counter += sizeof(pressure);

    memcpy(&tx_buffer[tx_counter], &temperature, sizeof(temperature));
    tx_counter += sizeof(temperature);

    memcpy(&tx_buffer[tx_counter], &ntc_external, sizeof(ntc_internal));
    tx_counter += sizeof(ntc_internal);

    memcpy(&tx_buffer[tx_counter], &ntc_external, sizeof(ntc_external));
    tx_counter += sizeof(ntc_external);

    memcpy(&tx_buffer[tx_counter], &battery_level, sizeof(battery_level));
    tx_counter += sizeof(battery_level);


#ifdef DEBUG
    unsigned long txtime = millis();
#endif

    rf95.send((uint8_t *)tx_buffer, tx_counter);
    rf95.waitPacketSent(); // Wait here until everything has been sent
    last_tx_time = thistime;

#ifdef DEBUG
    unsigned long txend = millis();
    Serial.print("Transmit time ");
    Serial.print(thistime);
    Serial.println(" ms");
    Serial.print("  Number of bytes: ");
    Serial.println(tx_counter);
    Serial.print("  Time on air ");
    Serial.print(txend - txtime);
    Serial.println(" ms");

    Serial.print("  Housekeeping ");
    Serial.println(housekeeping, BIN);


    Serial.println("  GPS");
    if (bitRead(housekeeping, GPSNOFIX) == 0) {
      Serial.print("    Lat=");
      Serial.println(latitude, 6);
      Serial.print("    Lon=");
      Serial.println(longitude, 6);
      Serial.print("    Alt=");
      Serial.println(altitude_meters);
      Serial.print("    Hours=");
      Serial.println(hours);
      Serial.print("    Minutes=");
      Serial.println(minutes);
      Serial.print("    Seconds=");
      Serial.println(seconds);
      Serial.print("    Satellites=");
      Serial.println(number_of_satellites);
    } else  {
      Serial.println("    No fix");
    }

    if (bitRead(housekeeping, BME280FAULT) == 0) {
      Serial.print("  Pressure (BME)=");
      Serial.println(pressure);
    }
    if (bitRead(housekeeping, SHT85FAULT) == 0) {
      Serial.print("  Temperature (SHT)=");
      Serial.println(sht.getTemperature());
      Serial.print("  Humidity (SHT)=");
      Serial.println(humidity);
    } else if (bitRead(housekeeping, BME280FAULT) == 0) {
      Serial.print("  Humidity (BME)=");
      Serial.println(bme.readHumidity());
      Serial.print("  Temperature (BME)=");
      Serial.println(temperature);
    }

    Serial.print("  INT=");
    Serial.print(ntc_internal);
    Serial.print(" (");
    Serial.print(temperatureNTC(ntc_internal, BETA_INT, R25_INT, RINT_FIXED));
    Serial.println(")");


    Serial.print("  EXT=");
    Serial.print(ntc_external);
    Serial.print(" (");
    Serial.print(temperatureNTC(ntc_external, BETA_EXT, R25_EXT, REXT_FIXED));
    Serial.println(")");

    Serial.print("  Battery=");
    Serial.println((float)battery_level * 0.00161132);  // 2*ADC*Vref/4096 (measuring half of the voltage, Vref=3.3V)

#endif
  }
  delay(1); // TODO: check whether this could be increased or replaced with a sleep functionality without losing GPS data
}
