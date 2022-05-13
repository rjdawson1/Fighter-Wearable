# Fighter-Wearable
Wearable device to monitor the vitals of first responders in the field

## How to use:

Download the .ino file along with the libraries.zip. Extract the .zip and move the downloaded libraries where your arduino libraries are located.
Then run .ino of which version you prefer.

### Wiring
We used an ESP32 Huzzah. 
ESP32 --> LoRa
GND --> GND
3.3V --> 3.3V 
RST --> GPIO15 
DIO0 --> GPIO32
NSS --> GPIO14
MOSI --> MOSI
MISO --> MISO 
SCK --> SCK

ESP32 --> GNSS
GND --> GND 
3.3V --> 3.3V 
SDA --> SDA 
SCL --> SCL

ESP32 --> MAX30101 & MAX32664 sparkfun breakout
MFIO --> GPIO33
RST --> GPIO21 
SCL --> SCL 
SDA --> SDA 
3.3V --> 3.3V
GND --> GND

ESP32 --> AD5933 PMMOD IA 
GND --> SEL
SCL --> SCL
SDA --> SDA
GND --> GND
3.3V --> VCC

### To Get the Receiver Working
You will need a Raspberry PI 4 with two LoRa sx1278 RA-02 Chips.
Please refer to https://github.com/andiconi/FighterMonitor.

### Version One 
used for a single device, it will contantly transmit data to the Pi.

### Version Two 
used for multiple devices where the Pi pings each device. Change the ID for each device programmed.


