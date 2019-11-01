#include <Wire.h>
#include <VL53L0X.h>
 
VL53L0X sensor;
 
void setup() {
  //Serial.begin(9600);
  Serial.begin(9600); 
  Serial.println("S");
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
   
  sensor.setMeasurementTimingBudget(50000); //50ms
  

}
 
 
void loop() {
  
    Serial.write(0xF0);
    Serial.write(0xF0);
    Serial.write(0xF0);
    int data=sensor.readRangeSingleMillimeters();
    //Serial.println(data);
    Serial.write(data>>8);
    Serial.write(data);
  
}
