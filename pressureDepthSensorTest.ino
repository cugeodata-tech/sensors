// Pressure Sensor test code 8/9/22 Haoyu (Leo) Wang, revised by Diane Brancazio dianeb@mit.edu
// last rev 4/7/22
// This code is used to test Adafruit Pressure+Temperature sensor on the LPS35HW breakout board
// Overview of the product at https://learn.adafruit.com/lps35hw-water-resistant-pressure-sensor
// You will need to install the Adafruit LPS35HW Library and the Adafruit BusIO library 
// using the Library Manager in the Arduino IDE.
// This program reads in the pressure and converts it to depth underwater in cm or ft.
// The conversion equation comes from the Hydrostatic Water Pressure Formula P = ⍴gh (⍴ is the greek letter rho, lowercase)
//   P is the pressure of a fluid at a depth in water. Sensor unit is hectoPascals which is 100 Pascals
//   ⍴ is the density of the fluid (997.05 kg/m^3 for freshwater, and 1023.6 kg/m^3 for saltwater)(, and h is the column of water above
//   g is the acceleration of gravity (9.807 m/s^2)
//   h is the height of the fluid column (in meters)
// Solving for h, the equation is h = P/(⍴g).  For the sensor unit of hPa and output of cm, it is h = (P/(⍴g))/10000
// Calculated through, it is h = P / 0.9778 (freshwater),and h = P / 1.0038 (saltwater).  Units are meters.
// these values are set as constants - choose the appropriate one for your code
// For accurate readings of depth underwater, submerge the sensor at the very top of the water and hit Reset on the Arduino 
// TO calibrate the robot or a measuring tape to the top of the water, use the variable "depthOffset" currently set to -10

//Connections and wiring notes for Serial Peripheral Interface (SPI) Logic
//Vin - power in from Arduino - connect to 5V (3.3V is ok too)
//GND - connect to Ground from Arduino
//SCK - SPI Clock pin, connect to pin 13
//SDO (MISO) - Serial Data Out / Microcontroller In Sensor Out pin, connect to pin 12
//SDI (MOSI) - Serial Data In / Microcontroller Out Sensor In pin, connect to pin 11
//CS - Chip Select pin, drop it low to start an SPI transaction, connect to pin 10

//Connections and wiring notes for I2C interface, with discrete wires (or the easy way -
// using a JST-SH 4-pin connector to the STEMMA QT header on the board)
//Vin (Red wire on JST-SH) - power in from Arduino - connect to 5V (3.3V is ok too)
//GND (Black wire on JST-SH) - Ground from Arduino - connect to GND
//SCK (Yellow wire on JST-SH) - the I2C clock pin, connect to A5 or SCL (near AREF)
//SDI (Blue wire on JST-SH) - the I2C data pin, connect to A4 or SDA (near AREF)
//SDO and CS pins are not needed in I2C protocol

#include <Adafruit_LPS35HW.h>
#include <stdio.h>
Adafruit_LPS35HW sensor = Adafruit_LPS35HW();

//Digital pins for SPI interface, no pin definitions for I2C interface
// #define LPS_CS   6
// #define LPS_SCK  12
// #define LPS_MISO 10
// #define LPS_MOSI 8
#define red 2  //LEDs to indicate depth
#define green 3
#define blue 4

#define cmConvertFreshwater 0.9778 //see conversion notes above
#define cmConvertSaltwater 1.0038
float depth, temperatureC, temperatureF, pressure_hpa, pressure_psi;
//You can change these values for testing 
int upper_limit = 40;   //Depth upper limit
int bottom_limit = 50;  //Depth lower limit

int depthOffset = 0; // calibration number to make the top of the robot or measuring tape be at 0

void setup() {
  Serial.begin(115200); //start the serial monitor
  while (!Serial) { delay(1); } //wait for Serial connection

  //uncomment the line below for I2C connections
  Serial.println("Starting");
  sensor.begin_I2C();
  //uncomment the line below for SPI connections
  //sensor.begin_SPI(LPS_CS, LPS_SCK, LPS_MISO, LPS_MOSI);
  sensor.zeroPressure();  //equates the pressure reading with 0 every time program starts
}

void loop() {
  //retrieving and printing the temperature detected onto the monitor
  temperatureC = sensor.readTemperature();
  temperatureF = temperatureC *9/5 + 32;
  Serial.print("Temperature: ");
//  Serial.print(temperatureC); 
//  Serial.println(" °C");
  Serial.print(temperatureF);
  printf("%d", temperatureF);
  Serial.println(" °F");
  
  //retrieving and printing the pressure detected onto the monitor
  pressure_hpa = sensor.readPressure();
  Serial.print("Pressure: ");
  Serial.print(pressure_hpa,3); 
  Serial.println(" hPa");

  depth = sensor.readPressure() / cmConvertFreshwater - depthOffset; //  use cmConvertSaltwater for saltwater
  Serial.print("Depth: ");
  Serial.print(depth);
  Serial.print(" cm,   ");
  //Uncomment the following 2 lines for measurements in ft
  Serial.print(depth * .0328084); //convert cm to ft
  Serial.println(" ft");
    if (depth > bottom_limit){
    digitalWrite(red,LOW);
    digitalWrite(green,LOW);
    digitalWrite(blue,HIGH);
  } else if (depth < upper_limit){
    digitalWrite(blue,LOW);
    digitalWrite(green,LOW);
    digitalWrite(red,HIGH);
  } else{
    digitalWrite(blue,LOW);
    digitalWrite(red,LOW);
    digitalWrite(green,HIGH);
  }
  Serial.println();

  //Senses temperature and pressure every 100ms
  //Change the time for more/less frequent sensing
  delay(100);
}
