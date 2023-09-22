#ifndef SOLVER_H
#define SOLVER_H

#include "maze.h"
#include "queue.h"

// include utility function to run the maze solving algorithm
void initializeMaze();                                       // initialize the maze array with empty cells
void resetFloodMap();                                        // reset the flood index array
void flooded();                                              // flooded the whole flood index map based on new wall structure
void setWalls(Point current, Orient orient, int direction);  // set the walls of the given point in the given direction
bool isFinished(Point current);                              // check whether the mouse has reached the destination
Point findNext(Point current, Orient orient);                // find th next point mouse need to go                                                                                // get the position of the cell mouse should go next
int findDirection(Point current, Point next, Orient orient); // check if the mouse has reached the destination

Orient getAbsDirection(Orient orient, int direction);
// bool isReversed(Orient orient1, Orient orient2);

#endif
