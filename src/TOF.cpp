// #include "TOF.h"
// #include<Arduino.h>

// // objects for the vl53l0x
// Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
// Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

// // this holds the measurement
// VL53L0X_RangingMeasurementData_t measure1;
// VL53L0X_RangingMeasurementData_t measure2;
// VL53L0X_RangingMeasurementData_t measure3;
// VL53L0X_RangingMeasurementData_t measure4;

// void initializeTOF() {
    
//     pinMode(SHT_LOX1, OUTPUT);
//     pinMode(SHT_LOX2, OUTPUT);
//     pinMode(SHT_LOX3, OUTPUT);
//     pinMode(SHT_LOX4, OUTPUT);

//     Serial.println("Shutdown pins inited...");

//     digitalWrite(SHT_LOX1, LOW);
//     digitalWrite(SHT_LOX2, LOW);
//     digitalWrite(SHT_LOX3, LOW);
//     digitalWrite(SHT_LOX4, LOW);

//     Serial.println("Both in reset mode...(pins are low)");
// }

// void setID() {
//     // all reset
//     digitalWrite(SHT_LOX1, LOW);
//     digitalWrite(SHT_LOX2, LOW);
//     digitalWrite(SHT_LOX3, LOW);
//     digitalWrite(SHT_LOX4, LOW);
//     delay(10);
//     // all unreset
//     digitalWrite(SHT_LOX1, HIGH);
//     digitalWrite(SHT_LOX2, HIGH);
//     digitalWrite(SHT_LOX3, HIGH);
//     digitalWrite(SHT_LOX4, HIGH);
//     delay(10);

//     // activating LOX1 and reseting LOX2
//     digitalWrite(SHT_LOX1, HIGH);
//     digitalWrite(SHT_LOX2, LOW);
//     digitalWrite(SHT_LOX3, LOW);
//     digitalWrite(SHT_LOX4, LOW);

//     // initing LOX1
//     if (!lox1.begin(LOX1_ADDRESS)) {
//         Serial.println(F("Failed to boot first VL53L0X"));
//         while (1)
//         ;
//     }
//     delay(10);

//     // activating LOX2
//     digitalWrite(SHT_LOX2, HIGH);
//     delay(10);

//     //initing LOX2
//     if (!lox2.begin(LOX2_ADDRESS)) {
//         Serial.println(F("Failed to boot second VL53L0X"));
//         while (1)
//         ;
//     }
//     // activating LOX3
//     digitalWrite(SHT_LOX3, HIGH);
//     delay(10);

//     //initing LOX2
//     if (!lox3.begin(LOX3_ADDRESS)) {
//         Serial.println(F("Failed to boot second VL53L0X"));
//         while (1)
//         ;
//     }

//     // activating LOX4
//     digitalWrite(SHT_LOX4, HIGH);
//     delay(10);

//     //initing LOX2
//     if (!lox4.begin(LOX4_ADDRESS)) {
//         Serial.println(F("Failed to boot second VL53L0X"));
//         while (1)
//         ;
//     }
// }