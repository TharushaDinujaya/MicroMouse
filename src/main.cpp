#include "ESP32SharpIR.h"
#include "maze.h"
#include "solver.h"

#include <Arduino.h>

#define RETURN_TO_START 0
#define FAST_RUN 0

#define pwmChannel1 0
#define pwmChannel2 1    // Selects channel 0
#define frequency 30000  // PWM frequency of 1 KHz
#define resolution 8   // 8-bit resolution, 256 possible values

// Motor A
#define pwmA 13
#define in1A 14
#define in2A 12

// Motor B
#define pwmB 4
#define in1B 19
#define in2B 18

// Speed control potentiometers


// Motor Speed Values - Start at zero
int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

ESP32SharpIR sensor1(ESP32SharpIR::GP2Y0A21YK0F, 26);
ESP32SharpIR sensor2(ESP32SharpIR::GP2Y0A21YK0F, 35);
ESP32SharpIR sensor3(ESP32SharpIR::GP2Y0A21YK0F, 34);


// create two arrays to represent wall map and flood index map with predefine sizes
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
Orient orient = NORTH; // initial orientation of the mouse

// initial coordination of the mouse
int X = 0;
int Y = 0;

bool run(void);

void setup() {
  // board initializing routines goes here
  sensor1.setFilterRate(0.0f);
  sensor2.setFilterRate(0.0f);
  sensor3.setFilterRate(0.0f);

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Configuration of channel 0 with the chosen frequency and resolution
  ledcSetup(pwmChannel1, frequency, resolution);
  ledcSetup(pwmChannel2, frequency, resolution);

  // Assigns the PWM channel to pin 23
  ledcAttachPin(pwmA, pwmChannel1);
  ledcAttachPin(pwmB, pwmChannel2);

  // array initialize routines goes here
  initializeMaze(wallMap, floodMap);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    // run the mouse until it reaches the destination
    bool running = true;
    while (running) {
      running = run();
    }

    #if RETURN_TO_START
    // TODO: find the way back to the destination
    
    // TODO: follow the calculated path

    #endif

    #if FAST_RUN
    // TODO: fast run implementation
    #endif
    // end of the program

}

void forward(int speed) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void reverse(int speed) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void left(int speed) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void right(int speed) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  ledcWrite(pwmChannel1, speed);  // 1.65 V
  ledcWrite(pwmChannel2, speed);  // 1.65 V
}

void stop() {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
}



void rotateLeft(void) {
  // TODO: implement the rotate left mechanism through motor drive and gyroscope
}

void rotateRight(void) {
  // TODO: implement the rotate right mechanism through motor drive and gyroscope
}

void rotateMouse(int8_t direction) {
  // rotate the mouse according to given target direction
  switch (direction)
  {
  case LEFT:
    left(150); // TODO: adjust the speed of the motor
    break;
  case RIGHT:
    right(150); // TODO: adjust the speed of the motor
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
  // TODO: get the distance from the front TOF sensor at the beginning
  float frontDistance = sensor2.getDistance();
  float initialDistance = frontDistance;
  // TODO: start the motor drive to move forward
  forward(150); // TODO: adjust the speed of the motor
  while (frontDistance > initialDistance - CELL_SIZE) {
    // TODO: keep align with the walls
    // TODO: get the distance from the front TOF sensor
    delay(100);
  }

  // TODO: stop the motor drive
  stop();

}

bool run(void) {
  // reset the flood map
  resetFloodMap(floodMap);
  
  // TODO:  first identifying the walls around the mouse
  bool leftWall = sensor1.getDistanceFloat() < 10; // TODO: should change the implementation
  bool rightWall = sensor3.getDistanceFloat() < 10; // TODO: should change the implementation
  bool frontWall = sensor2.getDistanceFloat() < 10;  // TODO: should change the implementation

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