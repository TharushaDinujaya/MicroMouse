#ifndef SOLVER_H
#define SOLVER_H

#include<maze.h>

// include utility function to run the maze solving algorithm
void update_flood_array(struct Cell** maze); // update the flood index of cells based on wall configutrations
Point get_next(const struct Cell** maze); // get the position of the cell mouse should go next
void go(Orient orient, Point point, struct Cell** maze); // move the mouse to the given position with given orientation
void update_walls(struct Cell** maze);

#endif
