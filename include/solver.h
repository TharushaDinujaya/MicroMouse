#ifndef SOLVER_H
#define SOLVER_H

#include"maze.h"
#include"queue.h"


// include utility function to run the maze solving algorithm
void initializeMaze(int wallMap[][MAZE_SIZE], int floodMap[][MAZE_SIZE]); // initialize the maze array with empty cells
void updateFullFloodArray(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE], Queue *q); // update the flood index of cells based on wall configutrations
void resetFloodMap(int floodMap[][MAZE_SIZE]); // reset the flood index array
Point getNext(const struct Cell** maze); // get the position of the cell mouse should go next
void go(Orient orient, Point point, struct Cell** maze); // move the mouse to the given position with given orientation
void updateWalls(struct Cell** maze);

Orient getAbsDirection(Orient orient, int direction);

#endif
