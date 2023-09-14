#include<iostream>
#include"queue.h"
#include"types.h"

#define UNDEFINED 99999
#define WIDTH 16
#define HEIGHT 16

// function declarations
int **initializeFloodMap();
int **initializeWallMap();
void update_flood_map(int **floodMap, int** wallMap);
void reset_flood_map(int **floodMap);

// helper function implementations
bool isLeftWall(int **wallMap, Point current) {
    return wallMap[current.x][current.y] & WALL_LEFT;
}

bool isRightWall(int **wallMap, Point current) {
    return wallMap[current.x][current.y] & WALL_RIGHT;
}

bool isFrontWall(int **wallMap, Point current) {
    return wallMap[current.x][current.y] & WALL_FRONT;
}

bool isBackWall(int **wallMap, Point current) {
    return wallMap[current.x][current.y] & WALL_BACK;
}

void printFloodMap(int **floodMap);

int main() {
    // create an array of ints to represent the flood map
    int** floodMap = initializeFloodMap();
    int** wallMap = initializeWallMap();

    update_flood_map(floodMap, wallMap);
    printFloodMap(floodMap);
}

int **initializeFloodMap() {
    int **floodMap = new int*[WIDTH]; // create a 2D array of ints
    for (int i = 0; i < WIDTH; i++) {
        floodMap[i] = new int[HEIGHT];
    }
    // set the flood index of all the cells to undefined and set the middle 4 cells to 0
    reset_flood_map(floodMap);

    return floodMap;
}

void reset_flood_map(int** floodMap) {
    for (int i = 0; i < WIDTH; i++) {
        for (int j = 0; j < HEIGHT; j++) {
            floodMap[i][j] = UNDEFINED;
        }
    }

    // calculate the middle of the maze
    int middleX = WIDTH / 2;
    int middleY = HEIGHT / 2;
    // fill the middle 4 cells with 0
    floodMap[middleX][middleY] = 0;
    floodMap[middleX + 1][middleY] = 0;
    floodMap[middleX][middleY + 1] = 0;
    floodMap[middleX + 1][middleY + 1] = 0;
}

void update_flood_map(int **floodMap, int** wallMap) {
    // create a new queue to keep track of the cells to be updated
    queue *q = create_queue();

    // calculate the middle of the maze
    int middleX = WIDTH / 2;
    int middleY = HEIGHT / 2;
    // add the middle 4 cells to the queue
    enqueue(q, {middleX, middleY});
    enqueue(q, {middleX + 1, middleY});
    enqueue(q, {middleX, middleY + 1});
    enqueue(q, {middleX + 1, middleY + 1});

    // run the while loop until the queue is not empty
    while (!empty(q)) {
        // fetch the next cell from the queue
        Point current = dequeue(q);
        std::cout << "Current: " << current.x << ", " << current.y << std::endl;
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
    // delete the queue
    free_queue(q);
}

int **initializeWallMap() {
    int **wallMap = new int*[WIDTH]; // create a 2D array of ints
    for (int i = 0; i < WIDTH; i++) {
        wallMap[i] = new int[HEIGHT];
    }

    for (int i = 0; i < WIDTH; i++) {
        for (int j = 0; j < HEIGHT; j++) {
            wallMap[i][j] = WALL_NONE;
        }
    }
    // setup the boundary walls of the maze;
    for (int i = 0 ; i < WIDTH; i++) {
        wallMap[i][0] |= WALL_BACK;
        wallMap[i][HEIGHT - 1] |= WALL_FRONT;
    }

    for (int j = 0; j < HEIGHT; j++) {
        wallMap[0][j] |= WALL_LEFT;
        wallMap[WIDTH - 1][j] |= WALL_RIGHT;
    }

    return wallMap;
}

void printFloodMap(int** floodMap) {
    std::cout << "Flood Map: " << std::endl;
    for (int i = 0; i < WIDTH; i++) {
        std::cout << "[";
        for (int j = 0; j < HEIGHT; j++) {
            std::cout << floodMap[i][j] << ", ";
        }
        std::cout << "]" << std::endl;
    }
}