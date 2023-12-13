# A small payload for weather balloons (radiosondes)

## Background

The [University Centre in Svalbard](https://www.unis.no) teaches several courses on upper atmospheric and space physics. The interesting processes take place at altitudes from about 80km upwards and in-situ measurements are carried out with either sounding rockets or satellites. However, many of the challenges with actual spaceflight instruments are present also when using high-altitude balloons with small payloads. The added benefit is that our meteorology students can use these student-built payloads with their weather balloons...

Instead of using ready-made commercial radiosondes from, e.g., Vaisala, this balloon payload is normally built, programmed and tested by the students. A major part is to calibrate the sensors.

## System overview

The payload uses an [Adafruit Feather](https://www.adafruit.com/product/3179) board which includes a [LoRa](https://en.wikipedia.org/wiki/LoRa) radio transceiver module. The ground station is simply another identical board with an antenna. We use a custom PCB for the payload to include additional LEDs, thermistors, sensors for pressure and humidity, orientation (IMU), and a GNSS-module for time and position.

The flight software is written in C++ (Arduino), but the ground station code is written in CircuitPython.

### Telemetry range

The transmit antenna is a simple "whip" or a piece of wire of correct length, which depends on the used frequency. For the receiver, one can use a similar whip for shorter distances. Our students have also built a simple quarter-wavelength ground-plane antennas for improved reception.

One should note that, in practice, there are constraints also for "license-free" radio transmissions in the ISM-band. For example, the Norwegian regulation limits both the maximum transmit power (10mW E.R.P.) as well as the transmission time (max 10% air time) when using the band 433-434Mhz without a radio licence. This implementation sends a data package once per second with an air time of less than 100ms. If there are no obstructions between the payload and the ground station, the range is easily over 50km.

### Satellite positioning

Most small GPS-modules will work without problems. We are using Adafruit's [Mini GPS](https://www.adafruit.com/product/4415) connected to the microcontroller's hardware serial port.

### Pressure, temperature and humidity

For the temperature, we use a voltage divider with a fixed resistor and an NTC-thermistor to provide a voltage output that varies with temperature. This is read using the analog-to-digital converter.

The pressure is measured using a breakout board for [BME280](https://www.adafruit.com/product/2652). While this sensor can also be used for temperature and humidity, our meteorology folks have had better success when using Sensirion's [SHT085](https://www.sensirion.com/) devices for recording relative humidities in field. Both BME280 and SHT085 use I2C for communicating with the microcontroller board.
