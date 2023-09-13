#ifndef SOLVER_H
#define SOLVER_H

#include"maze.h"
#include"queue.h"


// include utility function to run the maze solving algorithm
void initializeMaze(int wallMap[][MAZE_SIZE], int floodMap[][MAZE_SIZE]); // initialize the maze array with empty cells
void updateFullFloodArray(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE]); // update the flood index of cells based on wall configutrations
void resetFloodMap(int floodMap[][MAZE_SIZE]); // reset the flood index array
Point getNext(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE], Point current, Orient orient); // get the position of the cell mouse should go next
void go(Orient orient, Point point); // move the mouse to the given position with given orientation
void setWalls(int wallMap[][MAZE_SIZE], Point current, Orient orient, int direction);
bool isFinished(int floodMap[][MAZE_SIZE], Point current); // check if the mouse has reached the destination
void optimizedFloodMapFill(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE]);

Orient getAbsDirection(Orient orient, int direction);

#endif
