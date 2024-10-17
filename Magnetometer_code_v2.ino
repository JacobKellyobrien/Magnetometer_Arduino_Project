//Install Libraries
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <Adafruit_SSD1327.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "SdFat.h"

//Assign a unique ID to the sensors and add definitions
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CLK 13
#define LIS2MDL_MISO 12
#define LIS2MDL_MOSI 11
#define LIS2MDL_CS 10
#define DEV_I2C Wire
#define SerialPort Serial
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16
#define OLED_RESET -1

//SD card definitions & set up
#define SD_CS_PIN 23

SdFat SD;
File32 myFile;
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);

// Components
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);
Adafruit_SSD1327 display(128, 128, &Wire, OLED_RESET, 1000000);


int counter = 0; //start the counter at 0
const int buttonPin = 5; // Button is set to pin 5 (Next to SCL)
int buttonState = 0; //set the button state

//Temporary file name, edit in future to be date using RTC.
String file_name = "tester_button.csv";



/* Setup ---------------------------------------------------------------------*/


void setup()
{
  // Setting up the LED pin adn the internal pullup swith for the button
  pinMode(LedPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();

  // Switch off VL53L4CX satellite component.
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  //Initialize VL53L4CX satellite component.
  sensor_vl53l4cx_sat.InitSensor(0x12);

  // Start Measurements
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  
  //if the display is not loading correctly, send a fail message to the serial print
  if ( ! display.begin(0x3D) ) {
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }

  //Test and configure the display
  display.setTextSize(2);
  display.setTextColor(SSD1327_WHITE);
  display.setCursor(0,0);
  display.println("Loading...");
  display.display();

  //ensure the magnetometer is working correctly, display error code if not. 
  if(!lis2mdl.begin())
  {
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    display.clearDisplay();
    display.println("NO LIS2MDL Found");
    display.display();
    while(1);
  }
  
  //Sd card stuffz  
  Serial.print ("Initializing SD Card");
  //display an error message to the display and to the serial if the SD card either fails initialization of cannot be found.
  while (!SD.begin(config)){
    Serial.println("Failed! Retrying...");
    display.println("No SDcard Found");
    display.display();
    delay(5000);
  }
  
  Serial.println("Initialization Complete");  

  //Set the file to edit to myFile and open the file
  myFile = SD.open(file_name, FILE_WRITE);

  //If the button is set to the on(record) position, display a message to reset the button to the off position and restart the arduino. 
  buttonState = digitalRead(buttonPin);
  while (buttonState == LOW){
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("   Press ");
    display.print("  Button       & ");
    display.print("   Restart to");
    display.print(" Continue");
    display.display();
    
    delay(1000);
  }

  //Open the myFile CSV and save the headers for the measuremeants. Display an error message if it doesnt work.
  if (myFile) {
    Serial.print("Writing to ");
    Serial.println(file_name);
    myFile.print(" ");
    myFile.println(" ");
    myFile.println("Distance, X, Y, Z, Elapsed Time (sec)");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
    display.clearDisplay();
    display.println("Error Opening File");
  }
}

void loop()
{
  //set up the TOF
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  // Get a new sensor event
  sensors_event_t event;
  lis2mdl.getEvent(&event);

  //set the buttonstate to read the input of the button
  buttonState = digitalRead(buttonPin);

  //Led on
  digitalWrite(LedPin, HIGH);
  
  //set the status of the TOF and check how many objects are found (Not useful for Now)
  status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
  no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

  //When the button is set to record
  if (buttonState == LOW) {
    Serial.println("Button Pressed");
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print("D=");
    display.println(pMultiRangingData->RangeData[j].RangeMilliMeter); //Distance
    display.print("X: "); 
    display.println(event.magnetic.x); //X magnetic field measurement
    display.print("Y: ");
    display.println(event.magnetic.y); //Y magnetic field measurement
    display.print("Z: ");
    display.println(event.magnetic.z); //Z magnetic field measurement
    display.println("");
    display.print("Time:");
    display.println(counter); //Counter (how long its been recording)
    display.print("Saving:Yes");
    display.display(); 

    //Display the code to the serial input for checking purposes 
    Serial.print("D =");
    Serial.println(pMultiRangingData->RangeData[j].RangeMilliMeter);
    Serial.print("X: ");
    Serial.println(event.magnetic.x); //X magnetic field measurement
    Serial.print("Y: ");
    Serial.println(event.magnetic.y); //Y magnetic field measurement
    Serial.print("Z: ");
    Serial.println(event.magnetic.z); //Z magnetic field measurement
    Serial.print("Time: ");
    Serial.println(counter);
    Serial.print("Recording...");

    //SD card initiate
    myFile = SD.open(file_name, FILE_WRITE);

    //Save the information to the SD card
    if (myFile){
      myFile.println("");
      myFile.print("");
      myFile.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      myFile.print(",");
      myFile.print(abs(event.magnetic.x));
      myFile.print(",");
      myFile.print(abs(event.magnetic.y));
      myFile.print(",");
      myFile.print(abs(event.magnetic.z));
      myFile.print(",");
      myFile.print(counter);
      //close the SD card
      myFile.close();
    } else{
      //if an error occurs opening the file, display the error to the serial print.
        Serial.println("Error opening file...");
      
      }


    SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
    
    counter ++; //increase the count timer
    digitalWrite(LedPin, LOW); //LED off
    delay(1000); //Delay everything by 1 second

  } else{
    //IF button is set to off (initial state)
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0,0);
      display.print("D=");
      display.println(pMultiRangingData->RangeData[j].RangeMilliMeter); //Distance
      display.print("X: "); 
      display.println(event.magnetic.x); //X magnetic field measurement
      //display.println("");
      display.print("Y: ");
      display.println(event.magnetic.y); //Y magnetic field measurement
      //display.println("");
      display.print("Z: ");
      display.println(event.magnetic.z); //Z magnetic field measurement
      display.println("");
      display.print("Time:");
      display.println(counter); //This will only increase during the recording phase, so itll show how many seconds it has recorded for. 
      display.print("Saving:No");
      display.display();

      //also save to the serial input
      Serial.print("D =");
      Serial.println(pMultiRangingData->RangeData[j].RangeMilliMeter);
      Serial.print("X: ");
      Serial.println(event.magnetic.x); // X magnetic Field
      Serial.print("Y: ");
      Serial.println(event.magnetic.y); //Y magnetic Field
      Serial.print("Z: ");
      Serial.println(event.magnetic.z); //Z Magnetic Field
      Serial.print("Time: ");
      Serial.println(counter);
      Serial.print("NOT Recording");

      if (status == 0) {
        status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
      }
      digitalWrite(LedPin, LOW); //LED off
      delay(1000); //Delay messages by 1 second. 
  }
}

