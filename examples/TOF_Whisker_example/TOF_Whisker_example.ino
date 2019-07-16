/*************************************************************************
 * This example shows how to use continuous mode to take
 * range measurements with the VL53L0X. It is based on
 * vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.
 *
 * The range readings are in units of mm.
 * 
 * Modified by: Laverena Wienclaw for Tiny Circuits 
 *************************************************************************/

#include <Wire.h>       // For I2C communication
#include "VL53L0X.h"    // For interfacing with the Time-of-Flight Distance sensor
#include <TinyScreen.h> // For interfacing with the TinyScreen+

// TinyScreen+ variables
TinyScreen display = TinyScreen(TinyScreenPlus);
const int background = TS_8b_Black;

VL53L0X distanceSensor; // Name of sensor 
const int tofPort = 0;  // Port # of sensor (Found on Whisker Adapter Board)

const int averageCount = 1;
int average[averageCount];
int averagePos = 0;

void setup() {
  delay(200);              // Sensor Startup time
  SerialUSB.begin(115200); // Set Baud Rate
  Wire.begin();            // Begin I2C communication

  // TinyScreen appearance variables are set
  display.begin();
  display.setFlip(true);
  display.setBrightness(15); 
  display.setFont(thinPixel7_10ptFontInfo);
  display.fontColor(TS_8b_White, background);

  // Select the port of the distance sensor (this number corresponds 
  // with port #'s on the Whisker adapter board)
  selectPort(tofPort);

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
  SerialUSB.print(averageRead);
  SerialUSB.println("mm");
  printToScreen(averageRead);
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

// **This function is necessary for all Whisker boards attached through an Adapter board**
// Selects the correct address of the port being used in the Adapter board
void selectPort(int port) {
  Wire.beginTransmission(0x70);
  Wire.write(0x04 + port);
  Wire.endTransmission(0x70);
}
