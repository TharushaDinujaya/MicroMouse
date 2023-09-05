#ifndef TYPES_H
#define TYPE_H

// symbols for represent wall configuration of cells in the maze
#define WALL_NONE 0
#define WALL_LEFT 1
#define WALL_FRONT 2
#define WALL_RIGHT 4
#define WALL_BACK 8

enum Orient {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

#define LEFT 111
#define RIGHT 222
#define FORWARD 333


#endif