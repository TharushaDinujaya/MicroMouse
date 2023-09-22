// Description: This file contains the maze structure and its related functions.
#ifndef MAZE_H
#define MAZE_H

#define MAZE_SIZE 5
#define MAZE_CELLS (MAZE_SIZE * MAZE_SIZE)
#define UNDEFINED 9999
#define ORIENT_OFFSET 100
#define CELL_SIZE 148

enum Orient
{
    NORTH = ORIENT_OFFSET + 0,
    EAST = ORIENT_OFFSET + 1,
    SOUTH = ORIENT_OFFSET + 2,
    WEST = ORIENT_OFFSET + 3
};

// definitions for represent the orientation of the robot
#define LEFT -1
#define RIGHT 1
#define FORWARD 0

// symbols for represent wall configuration of cells in the maze
#define WALL_NONE 0
#define WALL_LEFT 1
#define WALL_FRONT 2
#define WALL_RIGHT 4
#define WALL_BACK 8

// struct for represent the Point in the maze (Cell position)
struct Point
{
    int x;
    int y;
};

// structure for represent a cell in the maze
struct Cell
{
    int walls;
    int flood_id;
};

#endif