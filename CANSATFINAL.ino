#include <DFRobot_BMX160.h>
#include <TinyGPS.h>
#include "DFRobot_BMP388_I2C.h"
#include "DFRobot_BMP388.h"
#include "Wire.h"
#include "SPI.h"
#include "bmp3_defs.h"
#include "Adafruit_CCS811.h"


Adafruit_CCS811 ccs;

DFRobot_BMX160 bmx160;
DFRobot_BMP388_I2C bmp388;
TinyGPS gps; // create gps object

#define CALIBRATE_Altitude
float seaLevel;
float lat,lon;

void setup() {
  Serial.begin(9600);
  
  
  //init the hardware bmx160  
  if (bmx160.begin() != true){
    Serial.println("init false");
    while(1);}
   bmp388.set_iic_addr(BMP3_I2C_ADDR_PRIM);
  /*Initialize bmp388*/
  while(bmp388.begin()){
    Serial.println("Initialize error!");
    delay(1000);
  }
  seaLevel = bmp388.readSeaLevel(525.0);
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  // Wait for the sensor to be ready
  while(!ccs.available());
  delay(100);
}

void loop() {
  bmx160SensorData Omagn, Ogyro, Oaccel;
  bmx160.getAllData(&Omagn, &Ogyro, &Oaccel);
 
  float Pressure = bmp388.readPressure();
  float Temperature = bmp388.readTemperature();
  #ifdef CALIBRATE_Altitude
  /* Read the calibrated altitude */
  float altitude = bmp388.readCalibratedAltitude(seaLevel);
  #else
  /* Read the altitude */
  float altitude = bmp388.readAltitude();
  #endif
  
  Serial.print(Omagn.x); Serial.print("  ");
  Serial.print(Omagn.y); Serial.print("  ");
  Serial.print(Omagn.z); Serial.print("  ");
  Serial.print(Ogyro.x); Serial.print("  ");
  Serial.print(Ogyro.y); Serial.print("  ");
  Serial.print(Ogyro.z); Serial.print("  ");
  Serial.print(Oaccel.x); Serial.print("  ");
  Serial.print(Oaccel.y); Serial.print("  ");
  Serial.print(Oaccel.z); Serial.print("  ");
  Serial.print(Pressure); Serial.print("  ");
  Serial.print(Temperature); Serial.print("  ");
  Serial.print(altitude); Serial.print("  ");

  if(ccs.available()){
    if(!ccs.readData()){
  Serial.print(ccs.geteCO2());Serial.print("  ");
  Serial.print(ccs.getTVOC());Serial.print("  ");
    }
    
    else{
      Serial.println("ERROR!");
      while(1);
    }
    
  }
  while(Serial1.available()){ 
    if(gps.encode(Serial1.read()))
    { 
      gps.f_get_position(&lat,&lon); 
      Serial.print(lat,6);
      Serial.print(","); 
      Serial.print(lon,6); 
      Serial.println(", ");
    }
  }
  Serial.println("");
  delay(1000);
}
  // put your main code here, to run repeatedly:
