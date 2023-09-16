#include "Adafruit_VL53L0X.h"
#include "mouse.h"
#include "user.h"
#include "maze.h"
#include "motors.h"
#include "solver.h"
#include <Arduino.h>


// this holds the measurement
extern VL53L0X_RangingMeasurementData_t measure1;
extern VL53L0X_RangingMeasurementData_t measure2;
extern VL53L0X_RangingMeasurementData_t measure3;
extern VL53L0X_RangingMeasurementData_t measure4;

extern int floodMap[MAZE_SIZE][MAZE_SIZE];
extern int wallMap[MAZE_SIZE][MAZE_SIZE];
extern Orient orient;
extern char pathString[MAX_PATH_LENGTH];
extern int pathLength;

extern int X;
extern int Y;


void rotateMouse(int direction) {
  // rotate the mouse according to given target direction
  switch (direction)
  {
  case LEFT:
    left(ROTATE_SPEED); // rotate the motors at given speed
    delay(ROTATE_TIME); // maintain the rotation until specified time is expired
    stop();
    pathString[pathLength++] = 'l'; // update the pathString and pathLength according to the rotation
    break;
  case RIGHT:
    right(ROTATE_SPEED); // rotate the motors at given speed
    delay(ROTATE_TIME); // maintain the rotation until specified time is expired
    stop();
    pathString[pathLength++] = 'r'; // update the pathString and pathLength according to the rotation
    break;
  case FORWARD:
    // nothing to do here
    break;
  default:
    break;
  }
}

void moveMouseForward(void) {
  // get the distance from the front TOF sensor at the beginning
  int frontDistance = measure1.RangeMilliMeter;
  int initialDistance = frontDistance;

  // start the motor drive to move forward
  forward(FORWARD_SPEED); // move the mouse forward under given speed
  while (frontDistance > initialDistance - CELL_SIZE) {
    // TODO: keep align with the walls
    delay(50);
    frontDistance = measure1.RangeMilliMeter; // distance from the front TOF sensor
  }
  // forward(125);
  // delay(1200);
  stop(); //  stop the motor drive

}

bool run(void) {
  // reset the flood map
  resetFloodMap(floodMap);
  
  bool leftWall = measure4.RangeMilliMeter < SIDE_MIN_DISTANCE; // check whether there is a wall on the left side
  bool rightWall = measure2.RangeMilliMeter < SIDE_MIN_DISTANCE; // check whether there is a wall on the right side
  bool frontWall = measure1.RangeMilliMeter<  FRONT_MIN_DISTANCE; // check whether there is a wall on the front side

  // update the wall map according to sensor inputs
  if (leftWall) setWalls(wallMap, {X, Y}, orient, LEFT);
  if (rightWall) setWalls(wallMap, {X, Y}, orient, RIGHT);
  if (frontWall) setWalls(wallMap, {X, Y}, orient, FORWARD);

  // update the flood map according to wall map
  updateFullFloodArray(floodMap, wallMap);

  // get the next cell to go
  Point next = getNext(floodMap, wallMap, {X, Y}, orient);

  // TODO: move the mouse to the next cell
      // if point is different from current point rotate the mouse until find the correct orientation
      
      // else 
        // rotate the mouse according to given target orientation
        
        // move the mouse forward until it reaches the next cell
  if (next.x == X && next.y == Y) {
    // rotatet the mouse until it find the appropriate oreintation
    rotateMouse(RIGHT); // TODO: change the priority of the rotation to LEFT/RIGHT if changes needed 
    orient = getAbsDirection(orient, RIGHT); // set the new orientation after rotate
    // update the pathString and pathLength
    pathString[pathLength++] = 'r';
    return true;
  } 
    
    
  // rotate the mouse to the given orientation
  // find the direction we should rotate the mouse
  int direction = findDirection({X, Y}, next, orient);
  // rotate the mouse according to given target direction
  rotateMouse(direction);
  // move the mouse forward until it reaches the next cell
  // move the mouse forward until it reaches the next cell
  moveMouseForward();
  pathString[pathLength++] = 'f'; // update the pathString and pathLength according to the forward movement
  // delay(1000);
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

void returnToStart(const char pathString[MAX_PATH_LENGTH], int pathLength) {

  // first rotate the mouse reversed
  rotateMouse(RIGHT);
  rotateMouse(RIGHT); // now mouse in the reversed direction

  for (int i = pathLength; i > 0; i--) {
    char c = pathString[i - 1];

    if (c == 'f') moveMouseForward(); // forward the mouse
    else if (c == 'l') rotateMouse(RIGHT); // turn right the mouse
    else if (c == 'r') rotateMouse(LEFT); // turn left the mouse
  }

  // again reverse the direction of the mouse
  rotateMouse(RIGHT);
  rotateMouse(RIGHT);

}