// #ifndef TOF_H
// #define TOF_H

// #include "Adafruit_VL53L0X.h"

// // TOF initialization parameters and variable creations
// // address we will assign if dual sensor is present
// #define LOX1_ADDRESS 0x30
// #define LOX2_ADDRESS 0x31
// #define LOX3_ADDRESS 0x32
// #define LOX4_ADDRESS 0x34
// #define LOX5_ADDRESS 0x35

// // set the pins to shutdown
// #define SHT_LOX1 18 // FRONT
// #define SHT_LOX2 4  // RIGHT
// #define SHT_LOX3 2  // RIGHT SECONDARY
// #define SHT_LOX4 19 // LEFT SECONDARY
// #define SHT_LOX5 5  // LEFT

// struct DistanceMetrix
// {
//     int leftFront;
//     int leftBack;
//     int rightFront;
//     int rightBack;
//     int front;
// };

// void
// initializeTOF();

// /*
//     Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
//     Keep sensor #1 awake by keeping XSHUT pin high
//     Put all other sensors into shutdown by pulling XSHUT pins low
//     Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
//     Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
//     Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
//  */
// void setID();
// DistanceMetrix loadDistnaces();

// #endif