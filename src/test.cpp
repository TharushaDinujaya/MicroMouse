#include "test.h"
#include<Arduino.h>
#include "motors.h"
#include "Adafruit_VL53L0X.h"
#include "user.h"


// this holds the measurement
extern VL53L0X_RangingMeasurementData_t measure1;
extern VL53L0X_RangingMeasurementData_t measure2;
extern VL53L0X_RangingMeasurementData_t measure3;
extern VL53L0X_RangingMeasurementData_t measure4;

void test() {
  while (true) {
    int front = measure1.RangeMilliMeter;
    if (front > 80) {
      Serial.printf("Distance (front): %d \n", front);
      forward(150);
    } else {
      int leftD = measure4.RangeMilliMeter;
      int rightD = measure2.RangeMilliMeter;

     if (leftD > 100) {
        Serial.printf("Distance (left): %f \n", leftD);
        left(150);
        delay(ROTATE_TIME);
        stop();
      } else if (rightD < 100) {
        Serial.printf("Distance (right): %f \n", rightD);
        right(150);
        delay(ROTATE_TIME);
        stop();
      } else {
        Serial.printf("Stopped !");
        stop();
      }
    }
  }
}