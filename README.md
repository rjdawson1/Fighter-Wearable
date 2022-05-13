# Fighter-Wearable
Wearable device to monitor the vitals of first responders in the field

## How to use:

Download the .ino file along with the libraries.zip. Extract the .zip and move the downloaded libraries where your arduino libraries are located.
Then run .ino of which version you prefer.

### Components needed
*2 LoRa Sx1278 RA-02 chips: https://www.amazon.com/SX1278-Module-Wireless-Spectrum-Transmission/dp/B07KDQWKNQ
*Sparkfun Zed F9R: https://www.sparkfun.com/products/16344
*Sparkfun Max30101 Pulse OX: https://www.sparkfun.com/products/15219
*AD5933 PMOD IA: https://projects.digilentinc.com/products/pmod-ia

### To Get the Receiver Working
Please refer to https://github.com/andiconi/FighterMonitor.

### Version One 
used for a single device, it will contantly transmit data to the Pi.

### Version Two 
used for multiple devices where the Pi pings each device. Change the ID for each device programmed.

### Wiring
We used an ESP32 Huzzah. <br />
ESP32 --> LoRa Sx1278 RA-02  <br />
GND --> GND <br />
3.3V --> 3.3V  <br />
RST --> GPIO15 <br />
DIO0 --> GPIO32 <br />
NSS --> GPIO14 <br />
MOSI --> MOSI <br />
MISO --> MISO <br />
SCK --> SCK <br />

ESP32 --> GNSS <br />
GND --> GND <br />
3.3V --> 3.3V  <br />
SDA --> SDA <br />
SCL --> SCL <br />

ESP32 --> MAX30101 & MAX32664 sparkfun breakout <br />
MFIO --> GPIO33 <br />
RST --> GPIO21  <br />
SCL --> SCL <br />
SDA --> SDA  <br />
3.3V --> 3.3V <br />
GND --> GND <br />

ESP32 --> AD5933 PMMOD IA  <br />
GND --> SEL <br />
SCL --> SCL <br />
SDA --> SDA <br />
GND --> GND <br />
3.3V --> VCC <br />

RASPBERRY PI 4 --> LoRa Sx1278 RA-02  <br />
3.3V --> 3.3V <br />
GND --> GND <br />
GPIO10 --> MOSI <br />
GPIO9 --> MISO  <br />
GPIO11 --> SCK <br />
GPIO8 --> NSS <br />
GPIO4 --> DIO0 <br />
GPIO22 --> RST <br />
GPIO17 --> DIO1 <br />
GPIO18 --> DIO2 <br />
GPIO27 --> DIO3 <br />

