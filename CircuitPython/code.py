# UNIS Radiosonde telemetry receiver
#
# MS, 13 Dec 2023

RADIO_FREQ_MHZ = 434.0

# ------------------------------------------------------------------------------
import time
import board
import busio
import digitalio
import adafruit_rfm9x
import struct
import math

# ------------------------------------------------------------------------------
# Configure the LEDs
# - one external LED for blinking when receiving data
# - the built-in LED will blink when receiving data, too
# - one external LED for indicating timeouts

ledActive = digitalio.DigitalInOut(board.D6)
ledActive.direction = digitalio.Direction.OUTPUT
ledActive.value = False

ledTimeout = digitalio.DigitalInOut(board.D5)
ledTimeout.direction = digitalio.Direction.OUTPUT
ledTimeout.value = False

ledBoard = digitalio.DigitalInOut(board.LED)
ledBoard.direction = digitalio.Direction.OUTPUT
ledBoard.value = False


# ------------------------------------------------------------------------------
# Configure the LoRa transceiver

CS = digitalio.DigitalInOut(board.RFM9X_CS)
RESET = digitalio.DigitalInOut(board.RFM9X_RST)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# Set the TX power to maximum allowed in Norway... (without a license)
rfm9x.tx_power = 7

#-------------------------------------------------------------------------------
# A quick conversion of A/D-conversion values to temperatures
# - 298.15K is the room temperature (25C)
def ntc2temp(adc, beta, r25, rfixed):
    a=adc/4095.0
    r=a/(1.0-a)*rfixed
    return beta/(math.log(r/r25)+beta/298.15)-273.15

# =========================================================================
# The main loop runs forever
# - wait for telemetry packets that are 43 bytes in size
# - unpack the telemetry data into variables
# - print out data: note that especially for the GNSS location we need to 
#   ensure there are a sufficient number of digits..

print("# Receiving at", RADIO_FREQ_MHZ, "MHz...")
print("#    Callsign, packetnum, housekeeping, Lat, Lon, Alt, HH, MM, SS, N of satellites, ...");
print("#    p, H, T, T_int, T_ext, adc_int, adc_ext, adc_vbat, rssi")
print("#")

while True:
    packet = rfm9x.receive(timeout=5.0)
    if packet is None:
        ledActive.value = False
        ledBoard.value = False
        ledTimeout.value = True
        print("... waiting")
    else:
        ledActive.value = not ledActive.value
        ledBoard.value = not ledBoard.value
        ledTimeout.value = False
        rssi = rfm9x.last_rssi
#        print("-----------------------------------------")
#        print("Received signal strength: {0} dB".format(rssi))
#        print("Packet length: {0} bytes".format(len(packet)))
        if len(packet)==43:
            callsign, packetnum, hk, lat, lon, alt, hh, mm, ss, nsat, p, H, T, ntc_int, ntc_ext, bat=struct.unpack('<6sHBfffBBBBfffHHH', packet)            # Print out comma-separated-values (CSV) for simple capture into a text file and then
            Tint=ntc2temp(ntc_int,4250.0,50e3,47e3)
            Text=ntc2temp(ntc_ext,3976.0,10e3,10e3)
            print('%s,%i,%i,%.7f,%.7f,%.1f,%i,%i,%i,%.1f,%.1f,%.1f,,%.1f, %.1f, %i,%i,%i,%i' 
                %(callsign,packetnum,hk,lat,lon,alt,hh,mm,ss,p,H,T,Tint, Text, ntc_int, ntc_ext,bat,rssi))
            
