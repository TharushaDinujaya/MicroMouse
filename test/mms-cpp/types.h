#ifndef TYPES_H
#define TYPE_H

// symbols for represent wall configuration of cells in the maze
#define WALL_NONE 0
#define WALL_LEFT 1
#define WALL_FRONT 2
#define WALL_RIGHT 4
#define WALL_BACK 8

#define OFFSET 100

enum Orient {
    NORTH = OFFSET + 0,
    EAST = OFFSET + 1,
    SOUTH = OFFSET + 2,
    WEST = OFFSET + 3
};

#define LEFT -1
#define RIGHT 1
#define FORWARD 0

struct Point {
    int x;
    int y;
};

#endif