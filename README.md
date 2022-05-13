# Fighter-Wearable
Wearable device to monitor the vitals of first responders in the field

## How to use:

Download the .ino file along with the libraries.zip. Extract the .zip and move the downloaded libraries where your arduino libraries are located.
Then run .ino of which version you prefer.

### Wiring
We used an ESP32 Huzzah. <br />
ESP32 --> LoRa <br />
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

### To Get the Receiver Working
You will need a Raspberry PI 4 with two LoRa sx1278 RA-02 Chips.
Please refer to https://github.com/andiconi/FighterMonitor.

### Version One 
used for a single device, it will contantly transmit data to the Pi.

### Version Two 
used for multiple devices where the Pi pings each device. Change the ID for each device programmed.


