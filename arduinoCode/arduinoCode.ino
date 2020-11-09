#include "support.h"

#include <HX711.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define LIS3DH_CLK 22 //SCL pin
#define LIS3DH_MISO 24 //SDO pin
#define LIS3DH_MOSI 26 //SDA pin
#define LIS3DH_CS 28 //CS pin
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

//cell a
int DOUT_A = 14;
int CLK_A = 15;
double location_A[] = {5, 0, 0};
double totalForce;
coordinates scale_A;
//cell b
int DOUT_B = 16;
int CLK_B = 17;
double location_B[] = {-5, 0, 0};
coordinates scale_B;
//cell c
int DOUT_C = 18;
int CLK_C = 19;
double location_C[] = {5, 1, 0};
coordinates scale_C;
//cell d
int DOUT_D = 20;
int CLK_D = 21;
double location_D[] = {-5, 1, 0};
coordinates scale_D;

coordinates correction;
double forces[4];
coordinates locations[4];
double omega = 0.104719755;

HX711 loadCell_A;
HX711 loadCell_B;
HX711 loadCell_C;
HX711 loadCell_D;
double calibration_A = 145000;
double calibration_B = 145000;
double calibration_C = 145000;
double calibration_D = 145000;
double sampleTime = 0.5;

void setup() {
  Serial.begin(9600);
  initializeAccelerometer(lis);
  
  setupScales(loadCell_A, DOUT_A, CLK_A, calibration_A);
  scale_A = coordFromArray(location_A);
  
  setupScales(loadCell_B, DOUT_B, CLK_B, calibration_B);
  scale_B = coordFromArray(location_B);
  
  setupScales(loadCell_C, DOUT_C, CLK_C, calibration_C);
  scale_C = coordFromArray(location_C);
  
  setupScales(loadCell_D, DOUT_D, CLK_D, calibration_D);
  scale_D = coordFromArray(location_D);
  
  locations[0] = scale_A; locations[1] = scale_B; locations[2] = scale_C; locations[3] = scale_D;
}

void initializeAccelerometer(Adafruit_LIS3DH &lis)
{
  while (!Serial) delay(1);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LIS3DH test!");

  if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");
  lis.getDataRate();
}

void setupScales(HX711 &loadCell, int dout, int clk, double calibrationFactor)
{
  loadCell.begin(dout, clk);
  loadCell.set_scale(calibrationFactor);
  loadCell.tare();
  loadCell.read_average();
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t event;
  lis.getEvent(&event);
  coordinates com;

  if (Serial.available())
  {
    while (Serial.available())
    {
    Serial.read();
    }
    printAccelerometer(event); 
    double radius = radiusOfRotation(omega, event.acceleration.x);
    Serial.print("Radius of rotation: "); Serial.print(radius);
    
    forces[0] = getLoading(loadCell_A);
    Serial.print("\nLoading on scale A: "); Serial.print(forces[0]);
    forces[1] = getLoading(loadCell_B);
    Serial.print("\nLoading on scale B: "); Serial.print(forces[1]);
    forces[2] = getLoading(loadCell_C);
    Serial.print("\nLoading on scale C: "); Serial.print(forces[2]);
    forces[3] = getLoading(loadCell_D);
    Serial.print("\nLoading on scale D: "); Serial.print(forces[3]);
    
    Serial.print("\n\nLocation of cell A: ");
    printCoord(locations[0], false);
    Serial.print("\nLocation of cell B: ");
    printCoord(locations[1], false);
    Serial.print("\n\nLocation of cell C: ");
    printCoord(locations[2], false);
    Serial.print("\nLocation of cell D: ");
    printCoord(locations[3], false);
    
    com = comLocation(forces, locations, 2);
    Serial.print("\nLocation of COM: ");
    printCoord(com, false);
    totalForce = sumArr(forces, 2);
    correction = correctionMoment(totalForce, com);
    Serial.print("\nRequired correction: ");
    printCoord(correction, true);
    Serial.print("\n\n\n");
  }
  delay(100);
}

void printAccelerometer(sensors_event_t event)
{
    Serial.print("\n\n----NEW TEST----\n\n");
  /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("\nX: "); Serial.print(event.acceleration.x);
    Serial.print(" \tY: "); Serial.print(event.acceleration.y);
    Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
    Serial.println(" m/s^2 ");
    Serial.println();

}

double getLoading(HX711 scale)
{
  double reading = scale.get_units();
  return reading;
}

void printCoord(coordinates coord, boolean mag)
{
  Serial.print("\nx: "); Serial.print(coord.x); Serial.print("\ty: "); Serial.print(coord.y); Serial.print("\tz: "); Serial.print(coord.z);
  if (mag)
  {
    Serial.print("\tmag: "); Serial.print(coord.magnitude);
  }
  Serial.print("\n");
}
