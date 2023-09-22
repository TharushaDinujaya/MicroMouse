#include "TOF.h"
#include "Adafruit_VL53L0X.h"
#include <Arduino.h>

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1; // LEFT ORIGINAL TOF (FRONT)
VL53L0X_RangingMeasurementData_t measure2; // LEFT EXTRA TOF (BACK)
VL53L0X_RangingMeasurementData_t measure3; // RIGHT ORIGINAL TOF (FRONT)
VL53L0X_RangingMeasurementData_t measure4; // FRONT TOF
VL53L0X_RangingMeasurementData_t measure5; // RIGHT EXTRA TOF (BACK)

void initializeTOF()
{

    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    pinMode(SHT_LOX4, OUTPUT);
    pinMode(SHT_LOX5, OUTPUT);

    Serial.println("Shutdown pins inited...");

    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);
    digitalWrite(SHT_LOX5, LOW);

    Serial.println("Both in reset mode...(pins are low)");
}

void setID()
{
    // all reset
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);
    digitalWrite(SHT_LOX5, LOW);
    delay(10);
    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    digitalWrite(SHT_LOX3, HIGH);
    digitalWrite(SHT_LOX4, HIGH);
    digitalWrite(SHT_LOX5, HIGH);
    delay(10);

    // activating LOX1 and reseting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);
    digitalWrite(SHT_LOX5, LOW);

    // initing LOX1
    if (!lox1.begin(LOX1_ADDRESS))
    {
        Serial.println(F("Failed to boot first VL53L0X"));
        while (1)
            ;
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // initing LOX2
    if (!lox2.begin(LOX2_ADDRESS))
    {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
            ;
    }
    // activating LOX3
    digitalWrite(SHT_LOX3, HIGH);
    delay(10);

    // initing LOX3
    if (!lox3.begin(LOX3_ADDRESS))
    {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
            ;
    }

    // activating LOX4
    digitalWrite(SHT_LOX4, HIGH);
    delay(10);

    // initing LOX4
    if (!lox4.begin(LOX4_ADDRESS))
    {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
            ;
    }

    // activating LOX5
    digitalWrite(SHT_LOX5, HIGH);
    delay(10);

    if (!lox5.begin(LOX5_ADDRESS))
    {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
            ;
    }
}

DistanceMetrix loadDistnaces()
{
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);
    lox3.rangingTest(&measure3, false);
    lox4.rangingTest(&measure4, false);
    lox5.rangingTest(&measure5, false);

    DistanceMetrix distances;
    distances.leftFront = measure1.RangeMilliMeter;
    distances.leftBack = measure2.RangeMilliMeter;
    distances.rightFront = measure3.RangeMilliMeter;
    distances.rightBack = measure5.RangeMilliMeter;
    distances.front = measure4.RangeMilliMeter;

    return distances;
}
