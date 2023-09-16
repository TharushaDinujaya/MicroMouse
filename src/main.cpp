#include "ESP32SharpIR.h"
#include "Adafruit_VL53L0X.h"
#include "maze.h"
#include "solver.h"
#include "motors.h"
// #include "TOF.h"
#include "IR.h"
#include "user.h"
// #include "test.h"
#include "mouse.h"

#include <Arduino.h>

// TOF initialization parameters and variable creations
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x34

// set the pins to shutdown
#define SHT_LOX1 2 // FRONT
#define SHT_LOX2 4 // RIGHT
#define SHT_LOX3 18 // BACK
#define SHT_LOX4 19 // LEFT

// create two arrays to represent wall map and flood index map with predefine sizes
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
Orient orient = NORTH; // initial orientation of the mouse
char pathString[MAX_PATH_LENGTH]; // path string to store the path
int pathLength = 0; // length of the path

// initial coordination of the mouse
int X = 0;
int Y = 0;

void test() {
  while (true) {
    int front = measure1.RangeMilliMeter;
    int leftD = measure4.RangeMilliMeter;
    int rightD = measure2.RangeMilliMeter;
    Serial.printf("front: %d, left: %d, right: %d \n", front, leftD, rightD);

    if (front > 80) {
      Serial.printf("Distance (front): %d \n", front);
      forward(150);
    } else {
      

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

void log(const char* message) {
  #if DEBUG
    Serial.println(message);
  #endif
}

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;

void initializeTOF() {
    
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    pinMode(SHT_LOX4, OUTPUT);

    Serial.println("Shutdown pins inited...");

    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);

    Serial.println("Both in reset mode...(pins are low)");
}

void setID() {
    // all reset
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);
    delay(10);
    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    digitalWrite(SHT_LOX3, HIGH);
    digitalWrite(SHT_LOX4, HIGH);
    delay(10);

    // activating LOX1 and reseting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);

    // initing LOX1
    if (!lox1.begin(LOX1_ADDRESS)) {
        Serial.println(F("Failed to boot first VL53L0X"));
        while (1)
        ;
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    //initing LOX2
    if (!lox2.begin(LOX2_ADDRESS)) {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
        ;
    }
    // activating LOX3
    digitalWrite(SHT_LOX3, HIGH);
    delay(10);

    //initing LOX2
    if (!lox3.begin(LOX3_ADDRESS)) {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
        ;
    }

    // activating LOX4
    digitalWrite(SHT_LOX4, HIGH);
    delay(10);

    //initing LOX2
    if (!lox4.begin(LOX4_ADDRESS)) {
        Serial.println(F("Failed to boot second VL53L0X"));
        while (1)
        ;
    }
}

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  initializeMotors();
  initializeIR();
  initializeTOF();
  log("Initializing the mouse...");

  setID();
  initializeMaze(wallMap, floodMap); // array initialize routines goes here
}

void loop() {
    lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
    lox2.rangingTest(&measure2, false);
    lox3.rangingTest(&measure3, false);
    lox4.rangingTest(&measure4, false);  // pass in 'true' to get debug data printout!

    log("Starting the program...");
    // blink the LED to indicate the start of the program
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    
    #if TEST
      test();
    #endif

    #if SEARCH_RUN
    for (int i = 0; i < ROUNDS; i++) {
      // run the mouse until it reaches the destination
      log("Running the mouse...");
      bool running = true;
      X = 0, Y = 0; // reset the initial coordination of the mouse
      // reset the path string and path length
      pathLength = 0;
      while (running) {
        log("Running...");
        running = run();
      }
      #if RETURN_TO_START
        returnToStart(pathString, pathLength); // return to the start point
      #endif
    }
    #endif

    #if FAST_RUN
      bool running = true;
      X = 0, Y = 0; // reset the initial coordination of the mouse
      // reset the path string and path length
      pathLength = 0;
      while (running) {
        log("Running...");
        running = run();
      }
      returnToStart(pathString, pathLength); // return to the start point
    #endif
    // end of the program
}







