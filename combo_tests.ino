#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h" // underwater temp
#include <Adafruit_LPS35HW.h>  //pressure
#include <stdio.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Combined sensor tests for all sensors currently connected to Arduino - BMP3 pressure/temp, Gravity TDS and turbidity, and analog temp sensor.

Adafruit_LPS35HW sensor = Adafruit_LPS35HW();

#define ONE_WIRE_BUS 11
#define SEALEVELPRESSURE_HPA (1013.25)
#define red 2  //LEDs to indicate depth
#define green 3
#define blue 4
#define cmConvertFreshwater 0.9778  //see conversion notes above
#define cmConvertSaltwater 1.0038
float depth, temperatureC, temperatureF, pressure_hpa, pressure_psi;
//You can change these values for testing
int upper_limit = 5;   //Depth upper limit
int bottom_limit = 9;  //Depth lower limit

int depthOffset = 0;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  bmp.begin_I2C(0x77);
  sensor.begin_I2C();

  while (!Serial);
  Serial.println("Adafruit BMP390 test, LPS35HW test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire

    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
   while (1);
  sensors.begin();
  
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  sensor.zeroPressure();
}

void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();

    //retrieving and printing the temperature detected onto the monitor
  temperatureC = sensor.readTemperature();
  temperatureF = temperatureC * 9 / 5 + 32;
  Serial.print("Temperature: ");
  //  Serial.print(temperatureC);
  //  Serial.println(" °C");
  Serial.print(temperatureF);
  printf("%d", temperatureF);
  Serial.println(" °F");

  //retrieving and printing the pressure detected onto the monitor
  pressure_hpa = sensor.readPressure();
  Serial.print("Pressure: ");
  Serial.print(pressure_hpa, 3);
  Serial.println(" hPa");

  depth = sensor.readPressure() / cmConvertFreshwater - depthOffset;  //  use cmConvertSaltwater for saltwater
  Serial.print("Depth: ");
  Serial.print(depth);
  Serial.print(" cm,   ");
  //Uncomment the following 2 lines for measurements in ft
  Serial.print(depth * .0328084);  //convert cm to ft
  Serial.println(" ft");
  if (depth > bottom_limit) {
    digitalWrite(red, LOW);
    digitalWrite(green, LOW);
    digitalWrite(blue, HIGH);
  } else if (depth < upper_limit) {
    digitalWrite(blue, LOW);
    digitalWrite(green, LOW);
    digitalWrite(red, HIGH);
  } else {
    digitalWrite(blue, LOW);
    digitalWrite(red, LOW);
    digitalWrite(green, HIGH);
  }
  Serial.println();

  // Turbidity sensor
  int sensorValue = analogRead(A1);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  Serial.println(voltage); // print out the value you read:

   // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures(); 

  // Temp sensor

  Serial.print("Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(sensors.getTempCByIndex(0)); 
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(sensors.getTempFByIndex(0));
  delay(1000);

  delay(2000);
}