#include <Arduino.h>
#include "maze.h"
#include "solver.h"
#include "ESP32SharpIR.h"
#include <stdint.h>

int pwmChannel1 = 0;
int pwmChannel2 = 1;    // Selects channel 0
int frequence = 30000;  // PWM frequency of 1 KHz
int resolution = 8;     // 8-bit resolution, 256 possible values

// Motor A

int pwmA = 13;
int in1A = 14;
int in2A = 12;

// Motor B

int pwmB = 4;
int in1B = 19;
int in2B = 18;

// Speed control potentiometers


// Motor Speed Values - Start at zero

int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

ESP32SharpIR sensor1(ESP32SharpIR::GP2Y0A21YK0F, 26);
ESP32SharpIR sensor2(ESP32SharpIR::GP2Y0A21YK0F, 35);
ESP32SharpIR sensor3(ESP32SharpIR::GP2Y0A21YK0F, 34);


// create two arrays fo rrepresent wall map and flood index map with predefine sizes
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
Orient orient = NORTH; // initial orientation of the mouse

// initial coordination of the mouse
uint8_t X = 0;
uint8_t Y = 0;

bool run(void);

void setup() {
  // board initializing routines goes here
  sensor1.setFilterRate(0.1f);
  sensor2.setFilterRate(0.1f);
  sensor3.setFilterRate(0.1f);

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Configuration of channel 0 with the chosen frequency and resolution
  ledcSetup(pwmChannel1, frequence, resolution);
  ledcSetup(pwmChannel2, frequence, resolution);

  // Assigns the PWM channel to pin 23
  ledcAttachPin(pwmA, pwmChannel1);
  ledcAttachPin(pwmB, pwmChannel2);

  // array initialize routines goes here
  initializeMaze(wallMap, floodMap);
}

void loop() {
    // // reset the flood index array
    // resetFloodMap(floodMap);

    // // update the flood index of cells based on wall configutrations
    // updateFullFloodArray(floodMap, wallMap);

    // // get the position of the cell mouse should go next
    // Point next = getNext(floodMap, wallMap, {0, 0}, orient);

    // // move the mouse to the given position with given orientation
    // go(orient, next);

    // // check if the mouse has reached the destination
    // while (!isFinished(floodMap, next)) {
    //   // update the flood index of cells based on wall configutrations
    //   optimizedFloodMapFill(floodMap, wallMap, next, orient, FORWARD);

    //   // get the position of the cell mouse should go next
    //   next = getNext(floodMap, wallMap, next, orient);

    //   // move the mouse to the given position with given orientation
    //   go(orient, next);
    // }

    // // if the mouse has reached the destination, turn on the led
    // digitalWrite(LED_BUILTIN, HIGH);
    // run the mouse until it reaches the destination
    bool running = true;
    while (running) {
      running = run();
    }

    // TODO: find the way back to the destination
    
    // TODO: follow the calculated path

    // end of the program

}

void rotateMouse(uint8_t direction) {
  // TODO: rotate the mouse according to given target direction
  switch (direction)
  {
  case LEFT:
    /* code */
    break;
  case RIGHT:
    /* code */
    break;
  case FORWARD:
    // nothing to do here
    break;
  default:
    break;
  }
}

void moveMouseForward(void) {
  // TODO: apply the logic to move the mouse until it reaches the middle of the next cell
  while (true) {
    // TODO: keep align with the walls
  }
}

bool run(void) {
  // reset the flood map
  resetFloodMap(floodMap);
  
  // TODO:  first identifying the walls around the mouse
  bool leftWall = sensor1.getDistance() < 10; // TODO: should change the implementation
  bool rightWall = sensor3.getDistance() < 10; // TODO: should change the implementation
  bool frontWall = sensor2.getDistance() < 10;  // TODO: should change the implementation

  // update the wall map according to sensor inputs
  if (leftWall) setWalls(wallMap, {X, Y}, orient, LEFT);
  if (rightWall) setWalls(wallMap, {X, Y}, orient, RIGHT);
  if (frontWall) setWalls(wallMap, {X, Y}, orient, FORWARD);

  // update the flood map according to wall map
  updateFullFloodArray(floodMap, wallMap);

  // get the next cell to go
  Point next = getNext(floodMap, wallMap, {X, Y}, orient);

  // TODO: move the mouse to the next cell
      // TODO: if point is different from current point rotate the mouse until find the correct orientation
      
      // TODO: else 
        // TODO: rotate the mouse according to given target orientation
        
        // TODO: move the mouse forward until it reaches the next cell
  if (next.x == X && next.y == Y) {
    // rotatet the mouse until it find the appropriate oreintation
    rotateMouse(RIGHT); // TODO: change the priority of the rotation to LEFT/RIGHT if changes needed 
    orient = getAbsDirection(orient, RIGHT); // set the new orientation after rotate
    return true;
  } 
    
    
  // rotate the mouse to the given orientation
  // find the direction we should rotate the mouse
  int direction = findDirection({X, Y}, next, orient);
  // TODO: rotate the mouse according to given target direction
  rotateMouse(direction);
  // move the mouse forward until it reaches the next cell
  // TODO: move the mouse forward until it reaches the next cell
  moveMouseForward();
  // update the current position of the mouse and orientation
  X = next.x;
  Y = next.y;
  orient = getAbsDirection(orient, direction);
  // check whether mouse has reached the destination
  if (isFinished(floodMap, next)) {
    return false;
  }
  return true;
}