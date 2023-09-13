#include <Arduino.h>
#include<maze.h>
#include<solver.h>

// create two arrays fo rrepresent wall map and flood index map with predefine sizes
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];
Orient orient = NORTH; // initial orientation of the mouse

void setup() {
  // board initializing routines goes here

  // array initialize routines goes here
  initializeMaze(wallMap, floodMap);
}

void loop() {
  // put your main code here, to run repeatedly:
}