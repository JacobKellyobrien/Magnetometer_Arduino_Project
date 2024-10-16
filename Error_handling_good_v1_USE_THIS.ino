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
const int buttonPin = 5; //
int buttonState = 0;

//Temporary file name, edit in future to be date using RTC.
String file_name = "tester_button.csv";



/* Setup ---------------------------------------------------------------------*/


void setup()
{
  // Led.
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


  
  display.clearDisplay();
  //display.display();
  if ( ! display.begin(0x3D) ) {
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }
  display.clearDisplay();

  //display tester
  display.setTextSize(2);
  display.setTextColor(SSD1327_WHITE);
  display.setCursor(0,0);
  display.println("Loading...");
  display.display();

  //LI2MDL
  if(!lis2mdl.begin())
  {
  /* There was a problem detecting the LIS2MDL ... check your connections */
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    display.clearDisplay();
    display.println("NO LIS2MDL Found");
    display.display();
    while(1);
  }
  //Sd card stuffz  
  
  Serial.print ("Initializing SD Card");

  while (!SD.begin(config)){
    Serial.println("Failed! Retrying...");
    //display.clearDisplay();
    display.println("No SDcard Found");
    display.display();

    delay(5000);
  }
  
  Serial.println("Initialization Complete");  

  myFile = SD.open(file_name, FILE_WRITE);

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
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

    /* Get a new sensor event */
  sensors_event_t event;
  lis2mdl.getEvent(&event);

  buttonState = digitalRead(buttonPin);

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(LedPin, HIGH);
  

  status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
  no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
  //snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
  //SerialPort.print(report);

//      SerialPort.print("status=");
 //     SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
  if (buttonState == LOW) {
    Serial.println("Button Pressed");
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print("D=");
    display.println(pMultiRangingData->RangeData[j].RangeMilliMeter);
    display.print("X: "); 
    display.println(event.magnetic.x);
    display.print("Y: ");
    display.println(event.magnetic.y);
    display.print("Z: ");
    display.println(event.magnetic.z);
    display.println("");
    display.print("Time:");
    display.println(counter);
    display.print("Saving:Yes");
    display.display();

    Serial.print("D =");
    Serial.println(pMultiRangingData->RangeData[j].RangeMilliMeter);
    Serial.print("X: ");
    Serial.println(event.magnetic.x);
    Serial.print("Y: ");
    Serial.println(event.magnetic.y);
    Serial.print("Z: ");
    Serial.println(event.magnetic.z);
    Serial.print("Time: ");
    Serial.println(counter);
    Serial.print("Recording...");

    //SD card initiate
    myFile = SD.open(file_name, FILE_WRITE);

    if (myFile){
      //Serial.print("Begining to Write to CSV");
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
      
      myFile.close();
      //Serial.print("wrote dun");
    } else{
        Serial.println("Error opening file...");
      
      }


    SerialPort.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
    
    counter ++;
    digitalWrite(LedPin, LOW);
    delay(1000);

  } else{
      Serial.println("Button Released");
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0,0);
      display.print("D=");
      display.println(pMultiRangingData->RangeData[j].RangeMilliMeter);
      display.print("X: "); 
      display.println(event.magnetic.x);
      //display.println("");
      display.print("Y: ");
      display.println(event.magnetic.y);
      //display.println("");
      display.print("Z: ");
      display.println(event.magnetic.z);
      display.println("");
      display.print("Time:");
      display.println(counter);
      display.print("Saving:No");
      display.display();

      Serial.print("D =");
      Serial.println(pMultiRangingData->RangeData[j].RangeMilliMeter);
      Serial.print("X: ");
      Serial.println(event.magnetic.x);
      Serial.print("Y: ");
      Serial.println(event.magnetic.y);
      Serial.print("Z: ");
      Serial.println(event.magnetic.z);
      Serial.print("Time: ");
      Serial.println(counter);
      Serial.print("NOT Recording");

      if (status == 0) {
        status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
      }
      //counter ++;
      digitalWrite(LedPin, LOW);
      delay(1000);
  }
}

