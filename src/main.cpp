#include <Arduino.h>
#include "maze.h"
#include "solver.h"

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
    // if the button is pressed, start the maze solving algorithm
    // reset the flood index array
    resetFloodMap(floodMap);

    // update the flood index of cells based on wall configutrations
    updateFullFloodArray(floodMap, wallMap);

    // get the position of the cell mouse should go next
    Point next = getNext(floodMap, wallMap, {0, 0}, orient);

    // move the mouse to the given position with given orientation
    go(orient, next);

    // check if the mouse has reached the destination
    while (!isFinished(floodMap, next)) {
      // update the flood index of cells based on wall configutrations
      optimizedFloodMapFill(floodMap, wallMap, next, orient, FORWARD);

      // get the position of the cell mouse should go next
      next = getNext(floodMap, wallMap, next, orient);

      // move the mouse to the given position with given orientation
      go(orient, next);
    }

    // if the mouse has reached the destination, turn on the led
    digitalWrite(LED_BUILTIN, HIGH);

}