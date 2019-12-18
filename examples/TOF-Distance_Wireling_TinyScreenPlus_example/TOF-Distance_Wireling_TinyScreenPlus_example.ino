/*************************************************************************
 * Time of Flight Distance Sensor Wireling 
 * This example shows how to use continuous mode to take, and print
 * range measurements with the VL53L0X Wireling. It is based on
 * vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.
 *
 * The range readings are in units of mm.
 * 
 * Hardware by: TinyCircuits
 * Modified by: Laveréna Wienclaw for TinyCircuits 
 * Last Modified: 12/18/19
 *************************************************************************/

#include <Wire.h>       // For I2C communication
#include "VL53L0X.h"    // For interfacing with the Time-of-Flight Distance sensor
#include <TinyScreen.h> // For interfacing with the TinyScreen+
#include <Wireling.h>   // For interfacing with Wirelings

// TinyScreen+ variables
TinyScreen display = TinyScreen(TinyScreenPlus);
const int background = TS_8b_Black;

VL53L0X distanceSensor; // Name of sensor 
const int tofPort = 0;  // Port # of sensor (Found on Wireling Adapter Board)

const int averageCount = 1;
int average[averageCount];
int averagePos = 0;

#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

void setup() {
  delay(200);              // Sensor Startup time
  SerialMonitorInterface.begin(115200); // Set Baud Rate
  Wire.begin();            // Begin I2C communication

  // Enable power & select port
  Wireling.begin(); 
  // Select the port of the distance sensor (this number corresponds 
  // with port #'s on the Wireling adapter board)
  Wireling.selectPort(tofPort);

  // TinyScreen appearance variables are set
  display.begin();
  display.setFlip(true);
  display.setBrightness(15); 
  display.setFont(thinPixel7_10ptFontInfo);
  display.fontColor(TS_8b_White, background);

  // Initialize the distance sensor and set a timeout
  distanceSensor.init();
  distanceSensor.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100))
  distanceSensor.setMeasurementTimingBudget(200000);
  distanceSensor.startContinuous();
}

void loop() {
  // Calculate the average position of the distance sensor
  unsigned long averageRead = 0;
  average[averagePos] = distanceSensor.readRangeContinuousMillimeters();
  averagePos++;
  if (averagePos >= averageCount)averagePos = 0;
  for (int i = 0; i < averageCount; i++) {
    averageRead += (unsigned long)average[i];
  }
  averageRead /= (unsigned long)averageCount;

  // Print the average position to the TinyScreen and the Serial Monitor
  printToScreen(averageRead);
  SerialMonitorInterface.print(averageRead);
  SerialMonitorInterface.println("mm");
  
}

// Print averageRead to the TinyScreen
void printToScreen(unsigned long averageRead){
  // This will make the screen look a little unsteady but is needed in order
  // to clear old values 
  display.clearScreen();
  
  display.setCursor(0, 0); 
  display.print("TOF Sensor Test"); 
  
  display.setCursor(0, 12);
  display.print(averageRead);

  if (distanceSensor.timeoutOccurred()) {
    display.print("-");
  }
}
