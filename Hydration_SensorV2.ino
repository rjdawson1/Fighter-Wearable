#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "AD5933.h"
#include <String.h>


TaskHandle_t Task1;
TaskHandle_t Task2;

#define START_FREQ  (50000)
#define FREQ_INCR   (0)
#define NUM_INCR    (10)
#define REF_RESIST   (10000)

//IMPEDANCE GLOBAL
double gain_factor;
double gain[NUM_INCR+1];
int phase[NUM_INCR+1];
byte PGA_Gain = PGA_GAIN_X1;
byte OUTPUT_RANGE = CTRL_OUTPUT_RANGE_3;
int Setting = 0;
int refrence = 10000;
double True_Imp = 0;
bool SEND;
//LORA GLOBAL VAR
int counter = 1;
int DIO0Pin = 32;
int RSTPin = 15;
int NSSPin = 14;
int ID = 3456;
char R[10];

//HR GLOBAL VAR
int resPin = 21;
int mfioPin = 33;
int algoRange = 100; // ADC Range (0-100%)
int algoStepSize = 20; // Step Size (0-100%)
int algoSens = 100; // Sensitivity (0-100%)
int algoSamp = 10; // Number of samples to average (0-255)
int width = 411;
int samples = 400; 
int pulseWidthVal;
int sampleVal;
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin);

//GPS GLOBAL
long lastTime = 0;
long Data[5] = {0,0,0,0,0};
//                                  
//1 byte oxygen
unsigned char oxLoRaBuf;
//1 byte hr
unsigned char hrLoRaBuf;
//2 byte imp
unsigned short impLoRaBuf;
//4 byte longitude
unsigned int longLoRaBuf;
//4 byte latitude
unsigned int latLoRaBuf;

AD5933 Sensor;
SFE_UBLOX_GNSS myGNSS;
bioData body; 
int pingedID;
void setup() {
  pinMode(DIO0Pin, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  HR_setup();
  GNSS_setup();
  lora_setup();
  Imp_Setup();
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);  
  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
}

//Task2code: LoRa task on core 0
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    int packetSize = LoRa.parsePacket();
    if (packetSize){
    for (int i = 0; i < packetSize; i++) {
      R[i] = (char)LoRa.read();
    }
    
    pingedID = atoi(R);
    if(pingedID == ID){
      Serial.println("Detected ID");
      lora_send();
    }
  }
  }
}


//Task1code: 
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
  read_hr();
  GNSS_start();
  avgImp();
  } 
}
void loop() {
}
void lora_setup(){
  LoRa.setPins(NSSPin,RSTPin,DIO0Pin);


  if (!LoRa.begin(434E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(11);
  //LoRa.setCodingRate4(8);  //USE FOR RANGE
  //LoRa.setSignalBandwidth(8);
  LoRa.setOCP(140);
//  LoRa.onReceive(onReceive);
//  LoRa.receive();
  //LoRa.setPreambleLengt(8);
  //LoRa.enableCrc();
  //LoRa.setGain(0);
  
}
void lora_send(){
  
  //display packet
  Serial.print("Sending packet: ");
  //Serial.print("Device 2: 434 MHz: ");
  
  Serial.print(ID);
  Serial.print("  ");
  Serial.print(Data[0]);
  Serial.print(",");
  Serial.print(Data[1]);
  Serial.print(",");
  Serial.print(Data[2]);
  Serial.print(",");
  Serial.print(Data[3]);
  Serial.print(",");
  Serial.println(Data[4]);
  

  // send packet
  LoRa.beginPacket();
  //LoRa.print("Device 2: 434MHz: ");
  LoRa.print(ID);
  LoRa.print("  ");
  LoRa.print(Data[0]);
  LoRa.print(" ");
  LoRa.print(Data[1]);
  LoRa.print(" ");
  LoRa.print(Data[2]);
  LoRa.print(" ");
  LoRa.print(Data[3]);
  LoRa.print(" ");
  LoRa.print(Data[4]);
  LoRa.endPacket(); 
}

void GNSS_setup(){
   

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void GNSS_start(){
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    //Serial.print(F("Lat: "));
    //Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    //Serial.print(F(" Long: "));
    //Serial.print(longitude);
    //Serial.print(F(" (degrees * 10^-7)"));
/*
    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
    */
    Data[0] = latitude;
    Data[1] = longitude;
    
  }

  //return Data;
  
}
void HR_setup(){
  int result = bioHub.begin();
  if (result == 0) //Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!!!");
 
  Serial.println("Configuring Sensor...."); 
  int error = bioHub.configBpm(MODE_TWO); // Configuring just the BPM settings. 
  if(error == 0){ // Zero errors
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }

  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up. 
  Serial.println("Loading up the buffer with data....");
  delay(4000); 
  
}

void read_hr(){
  body = bioHub.readBpm();
  

  /*
  Serial.print("Heartrate: ");
  Serial.println(body.heartRate); 
  Serial.print("Confidence: ");
  Serial.println(body.confidence);   
  Serial.print("Oxygen: ");
  Serial.println(body.oxygen);
  */
  Data[2] = body.heartRate;
  Data[3] = body.oxygen;    
}

double frequencySweepRaw() {
    // Create variables to hold the impedance data and track frequency
    int real, imag, i = 0, cfreq = START_FREQ/1000;
    double impedance;
    double impRetsum = 0;
    double avg;
    double resRetsum = 0;
    double avg2;
    
    // Initialize the frequency sweep
    if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
          AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             Serial.println("Could not initialize frequency sweep...");
         }

    // Perform the actual sweep
    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Get the frequency data for this frequency point
        if (!AD5933::getComplexData(&real, &imag)) {
            Serial.println("Could not get raw frequency data...");
        }

        // Compute impedance
        double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
        Serial.print("real: ");
        Serial.print((real));
        Serial.print(" imag: ");
        Serial.print((imag));
        impedance = 1/(magnitude*gain_factor);
        Serial.print(" imp: ");
        Serial.println((impedance - 10000));
        impRetsum += (impedance  - 10000);
        resRetsum += 13917 - real;
        
        avg2 = resRetsum/11;                                 
        //Serial.println(avg2);
        // Increment the frequency
        i++;
        cfreq += FREQ_INCR/1000;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
    }

    //Serial.println("Frequency sweep complete!");

    // Set AD5933 power mode to standby when finished
    if (!AD5933::setPowerMode(POWER_STANDBY)){
        Serial.println("Could not set to standby...");
    }
    avg = impRetsum/11;
    Serial.println(avg2);
    return avg;
}
void avgImp(){
  double sum = 0;
  long avg =0; 
  double secondSum =0; 
  int i =0;
  int j = 0;
  for (j; j < 5; j++){
    sum += frequencySweepRaw();
    
  }
  
  avg = (sum/j);
  //Serial.println(j);
  Data[4] = avg;
}

void Imp_Setup(){
   Serial.println("AD5933 Test Started!");

  // Perform initial configuration. Fail if any one of these fail.
  if (!(Sensor.reset() &&
        Sensor.setInternalClock(true) &&
        Sensor.setStartFrequency(START_FREQ) &&
        Sensor.setIncrementFrequency(FREQ_INCR) &&
        Sensor.setNumberIncrements(NUM_INCR) &&
        Sensor.setPGAGain(PGA_Gain) &&
        Sensor.setRange(OUTPUT_RANGE)))
        {
            Serial.println("FAILED in initialization!");
            while (true) ;
        }
               
      if (AD5933::calibrate(gain, phase, REF_RESIST, NUM_INCR+1)){
      Serial.println("Calibrated!");
    }
    else{
      Serial.println("Calibration failed...");
    }

  if(PGA_Gain == PGA_GAIN_X1){
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_1){                       //1.8V
      gain_factor = 0.00000000492333;
      Setting = 11;
    }
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_2){                     //800mV
      gain_factor = 0.0000000050143;
      Setting = 12;
      //Serial.println(gain_factor)
    }
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_3){                    //300mV
      gain_factor = 0.00000000571896; //two point calibrtion was used to find the GF for this case for 10.0 K ohms (this is close to expected Bio-Impedance + 10K)
      Setting = 13;
    }
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_4){                    //180mV
      gain_factor = 0.0000000109963;
      Setting = 14;
    }
  }
   if(PGA_Gain == PGA_GAIN_X5){
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_1){
      gain_factor = 0.000000002283;
      Setting = 51;
    }
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_2){
      gain_factor = 0.0000000004648;
      Setting = 52;
    }
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_3){
      gain_factor = 0.0000000011043;
      Setting = 53;
    }
    if(OUTPUT_RANGE == CTRL_OUTPUT_RANGE_4){
      gain_factor = 0.0000000024459;
    Setting = 54;
    }
}
}
