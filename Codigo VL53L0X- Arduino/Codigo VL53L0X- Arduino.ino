#include "Adafruit_VL53L0X.h"
 
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
 
void setup() {
  //Serial.begin(9600);
  Serial.begin(9600); 
  Serial.println("S");
  if (!lox.begin()) {
    Serial.println(F("E"));
    while(1);
  }
}
 
 
void loop() {
  VL53L0X_RangingMeasurementData_t measure;     
  lox.rangingTest(&measure, false); 
  if (measure.RangeStatus != 4){
    Serial.write(0x01);
    Serial.write(0x01);
    int data=measure.RangeMilliMeter;
    //Serial.println(data);
    Serial.write(data>>8);
    Serial.write(data);
  }
}
