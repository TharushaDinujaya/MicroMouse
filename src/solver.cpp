#include"maze.h"
#include"solver.h"
#include"queue.h"

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

void updateFullFloodArray(int floodMap[][MAZE_SIZE], int wallMap[][MAZE_SIZE], Queue *q) {
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
