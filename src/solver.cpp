#include"maze.h"
#include"solver.h"
#include"queue.h"
#include "stack.h"
#include"user.h"

// helper function implementations
bool isLeftWall(int wallMap[][MAZE_SIZE], Point current) {
    return wallMap[current.x][current.y] & WALL_LEFT;
}

bool isRightWall(int wallMap[][MAZE_SIZE], Point current) {
    return wallMap[current.x][current.y] & WALL_RIGHT;
}

bool isFrontWall(int wallMap[][MAZE_SIZE], Point current) {
    return wallMap[current.x][current.y] & WALL_FRONT;
}

bool isBackWall(int wallMap[][MAZE_SIZE], Point current) {
    return wallMap[current.x][current.y] & WALL_BACK;
}

bool isReversed(Orient orient1, Orient orient2) {
    int diff = orient1 - orient2;
    return diff == -2 || diff == 2;
}

Point getNextPoint(int x, int y, Orient orient, int direction) {

    // get the absolute direction using the current orientation and the direction
    Orient absOrientation = getAbsDirection(orient, direction);
    
    Point p = {x, y};
    // based on abs direction return the next cell
    switch (absOrientation)
    {
    case NORTH:
        p = {x, y + 1};
        break;
    case EAST:
        p = {x + 1, y};
        break;
    case SOUTH:
        p = {x, y - 1};
        break;
    case WEST:
        p = {x - 1, y};
        break;
    default:
        break;
    }

    return p;
}

Orient getAbsDirection(Orient orient, int direction) {
    int tmp = (orient + direction - ORIENT_OFFSET) % 4;
    if (tmp < 0) tmp += 4;

    return (Orient) (tmp + ORIENT_OFFSET);
}

// implementation of the solving algorithm functions
void initializeMaze(int wallMap[][MAZE_SIZE], int floodMap[][MAZE_SIZE]) {
    // initialize the maze array with empty cells
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            wallMap[i][j] = WALL_NONE;
            floodMap[i][j] = UNDEFINED;
        }
    }

    // set the boundry walls of the maze
    for (int i = 0; i < MAZE_SIZE; i++) {
        wallMap[i][0] |= WALL_BACK;
        wallMap[i][MAZE_SIZE - 1] |= WALL_FRONT;
        wallMap[0][i] |= WALL_LEFT;
        wallMap[MAZE_SIZE - 1][i] |= WALL_RIGHT;
    }

    // set the destination cell's flood index as zero
    floodMap[FINISHING_X][FINISHING_Y] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y] = 0;
    floodMap[FINISHING_X][FINISHING_Y + 1] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y + 1] = 0;

}

void resetFloodMap(int floodMap[][MAZE_SIZE]) {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            floodMap[i][j] = UNDEFINED;
        }
    }

    // fill the middle 4 cells with 0
    floodMap[FINISHING_X][FINISHING_Y] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y] = 0;
    floodMap[FINISHING_X][FINISHING_Y + 1] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y + 1] = 0;
}

void updateFullFloodArray(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE]) {

    static Queue *q = createQueue(DEFAULT_SIZE); // create a queue to store the cells to be updated
    // reset the given queue
    resetQueue(q);

    // add the middle 4 cells to the queue
    enqueue(q, {FINISHING_X, FINISHING_Y});
    enqueue(q, {FINISHING_X + 1, FINISHING_Y});
    enqueue(q, {FINISHING_X, FINISHING_Y + 1});
    enqueue(q, {FINISHING_X + 1, FINISHING_Y + 1});

    // run the while loop until the queue is not empty
    while (!empty(q)) {
        // fetch the next cell from the queue
        Point current = dequeue(q);
        // get the flood index of the current position
        int current_flood_index = floodMap[current.x][current.y];

        // check if the cell on the left is not a wall and the flood index is undefined
        if (!isLeftWall(wallMap, current) && floodMap[current.x - 1][current.y] == UNDEFINED) {
            floodMap[current.x - 1][current.y] = current_flood_index + 1;
            enqueue(q, {current.x - 1, current.y}); // add the cell to the queue
        }
        // check if the cell on the right is not a wall and the flood index is undefined
        if (!isRightWall(wallMap, current) && floodMap[current.x + 1][current.y] == UNDEFINED) {
            floodMap[current.x + 1][current.y] = current_flood_index + 1;
            enqueue(q, {current.x + 1, current.y}); // add the cell to the queue
        } 
        
        // check if the cell on the front is not a wall and the flood index is undefined
        if (!isFrontWall(wallMap, current) && floodMap[current.x][current.y + 1] == UNDEFINED) {
            floodMap[current.x][current.y + 1] = current_flood_index + 1;
            enqueue(q, {current.x, current.y + 1}); // add the cell to the queue
        } 
        
        // check if the cell on the back is not a wall and the flood index is undefined
        if (!isBackWall(wallMap, current) && floodMap[current.x][current.y - 1] == UNDEFINED) {
            floodMap[current.x][current.y - 1] = current_flood_index + 1;
            enqueue(q, {current.x, current.y - 1}); // add the cell to the queue
        }


    }
}

Point getNext(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE], Point current, Orient orient) {

    Point next;

    int northIndex = floodMap[current.x][current.y + 1];
    int eastIndex = floodMap[current.x + 1][current.y];
    int southIndex = floodMap[current.x][current.y - 1];
    int westIndex = floodMap[current.x - 1][current.y];

    int minIndex = UNDEFINED;
    if (minIndex > northIndex && !isFrontWall(wallMap, current) && !isReversed(orient, NORTH)) {
        minIndex = northIndex;
    }

    if (minIndex > eastIndex && !isRightWall(wallMap, current) && !isReversed(orient, EAST)) {
        minIndex = eastIndex;
    }

    if (minIndex > southIndex && !isBackWall(wallMap, current) && !isReversed(orient, SOUTH)) {
        minIndex = southIndex;
    }

    if (minIndex > westIndex && !isLeftWall(wallMap, current) && !isReversed(orient, WEST)) {
        minIndex = westIndex;
    }
    
    if (minIndex == UNDEFINED) {
        // if no cell is found return the current cell
        return current;
    }

    // get the next cell according to the min flood index
    if (minIndex == northIndex) {
        next = {current.x, current.y + 1};
    } else if (minIndex == eastIndex) {
        next = {current.x + 1, current.y};
    } else if (minIndex == southIndex) {
        next = {current.x, current.y - 1};
    } else if (minIndex == westIndex) {
        next = {current.x - 1, current.y};
    }
    return next;
}

void setWalls(int wallMap[][MAZE_SIZE], Point current, Orient orient, int direction) {

    // get the next cell according to the directions
    Point next = getNextPoint(current.x, current.y, orient, direction);
    Orient absOrient = getAbsDirection(orient, direction);

    switch (absOrient)
    {
    case NORTH:
        wallMap[current.x][current.y] |= WALL_FRONT;
        wallMap[next.x][next.y] |= WALL_BACK;
        break;

    case EAST:
        wallMap[current.x][current.y] |= WALL_RIGHT;
        wallMap[next.x][next.y] |= WALL_LEFT;
        break;
    
    case SOUTH:
        wallMap[current.x][current.y] |= WALL_BACK;
        wallMap[next.x][next.y] |= WALL_FRONT;
        break;

    case WEST:
        wallMap[current.x][current.y] |= WALL_LEFT;
        wallMap[next.x][next.y] |= WALL_RIGHT;
        break;
    
    default:
        break;
    }

}

bool isFinished(int floodMap[][MAZE_SIZE], Point current) {
    // check if the current position is the destination
    return floodMap[current.x][current.y] == 0;
}

int findDirection(Point current, Point next, Orient orient) {
    
    Orient nextCellOrient;

    if (next.y == current.y + 1) nextCellOrient = NORTH;
    else if (next.y == current.y - 1) nextCellOrient = SOUTH;
    else if (next.x == current.x + 1) nextCellOrient = EAST;
    else if (next.x == current.x - 1) nextCellOrient = WEST;

    // return the direction we need to rotate according to current orientation
    int tmp = nextCellOrient - orient;
    if (tmp == 3) return LEFT;
    if (tmp == -3) return RIGHT;
    else return tmp;
}


void optimizedFloodMapFill(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE], Point current, Orient orient, int direction) {

    static Stack *s = createStack(DEFAULT_SIZE);

    // reset the stack first
    resetStack(s);
    // TODO: algorithm goes here

}