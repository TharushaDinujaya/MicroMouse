// Description: This file contains the maze structure and its related functions.
#ifndef MAZE_H
#define MAZE_H

#include<ctype.h>

#define MAZE_SIZE 14
#define UNKNOWN 9999


enum Orient {
    FORWARD = 0,
    RIGHT,
    BACKWARD,
    LEFT
};

// symbols for represent wall configuration of cells in the maze
#define WALL_NONE 0
#define WALL_LEFT 1
#define WALL_FRONT 2
#define WALL_RIGHT 4
#define WALL_BACK 8


// struct for represent the Point in the maze (Cell position)
typedef struct Point {
    int x;
    int y;
};

// structure for represent a cell in the maze
typedef struct Cell {
    int walls;
    int flood_id;
};

#endif