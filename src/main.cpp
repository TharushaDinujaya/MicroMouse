#include "ESP32SharpIR.h"
#include "Adafruit_VL53L0X.h"
#include "maze.h"
#include "solver.h"
#include "motors.h"
#include "TOF.h"
#include "IR.h"
#include "user.h"
#include "test.h"
#include "mouse.h"

#include <Arduino.h>

// create two arrays to represent wall map and flood index map with predefine sizes
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
Orient orient = NORTH; // initial orientation of the mouse
char pathString[MAX_PATH_LENGTH]; // path string to store the path
int pathLength = 0; // length of the path

// initial coordination of the mouse
int X = 0;
int Y = 0;

void log(const char* message) {
  #if DEBUG
    Serial.println(message);
  #endif
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







