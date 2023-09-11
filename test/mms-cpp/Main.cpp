#include <iostream>
#include <string>

#include "API.h"
#include"queue.h"
#include "util.h"
#include "tracker.h"


#define UNDEFINED 99999
// finsihing point coordinate of the maze
#define FINISHING_X 7
#define FINISHING_Y 7

//  initialization function declarations
int **initializeFloodMap();
int **initializeWallMap();
void reset_flood_map(int **floodMap);

// updating function declarations
void update_flood_map(int **floodMap, int** wallMap, queue *q);
Point getNextFloodIndex(int x, int y, Orient orient, int direction);
void updateWalls(int **wallMap, int x, int y, Orient orient, int direction);
void setWall(int **wallMap, int x, int y, int wall);
bool isFinished(int x, int y);

void printFloodMap(int** floodMap);

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

int main(int argc, char* argv[]) {
    // create an array of ints to represent the flood map
    int** floodMap = initializeFloodMap();
    int** wallMap = initializeWallMap();
    // initialize a queue with default size
    queue *q = create_queue();

    log("Running...");
    API::setColor(0, 0, 'A');
    API::setText(0, 0, "abc");

    // starting position of the maze
    int x = 0, y = 0;
    Orient orient = NORTH; // starting orienttation of the mouse
    while (true) {
        update_flood_map(floodMap, wallMap, q);
        printFloodMap(floodMap);

        int leftIndex = UNDEFINED, rightIndex = UNDEFINED, frontIndex = UNDEFINED;
        Point next = {-1, -1};
        int min_index = LEFT;
        // observe the existence of the walls
        // check LEFT
        if (!API::wallLeft()) {
            Point p = getNextFloodIndex(x, y, orient, LEFT);
            leftIndex = floodMap[p.x][p.y];
        } else {
            // update the wall map
            updateWalls(wallMap, x, y, orient, LEFT);
        }
        // check RIGHT
        if (!API::wallRight()) {
            Point p = getNextFloodIndex(x, y, orient, RIGHT);
            rightIndex = floodMap[p.x][p.y];
        } else {
            updateWalls(wallMap, x, y, orient, RIGHT);

        }
        // check FRONT
        if (!API::wallFront()) {
            Point p = getNextFloodIndex(x, y, orient, FORWARD);
            frontIndex = floodMap[p.x][p.y];
        } else {
            updateWalls(wallMap, x, y, orient, FORWARD);

        }

        // find the minimum flood index
        int min = leftIndex;
        if (min > frontIndex) {
            min = frontIndex;
            min_index = FORWARD;
        }
        if (min > rightIndex) {
            min = rightIndex;
            min_index = RIGHT;
        }


        if (min == UNDEFINED) {
            log("UNDEFINED TURNING RIGHT");
            API::turnRight();
            orient = getAbsDirection(orient, RIGHT);
            // reset the flood map
            reset_flood_map(floodMap);
            continue;
        }

        Point p;
        // move to the cell with the minimum flood index
        if (min_index == LEFT) {
            log("LEFT");
            API::turnLeft();
            API::moveForward();
            p = getNextFloodIndex(x, y, orient, LEFT);  
            orient = getAbsDirection(orient, LEFT); // update the current position
        } else if (min_index == RIGHT) {
            log("RIGHT");
            API::turnRight();
            API::moveForward();
            p = getNextFloodIndex(x, y, orient, RIGHT);
            orient = getAbsDirection(orient, RIGHT);; // update the current position
        } else if (min_index == FORWARD) {
            log("FRONT");
            API::moveForward();
            p = getNextFloodIndex(x, y, orient, FORWARD); // update the current position
        }
        x = p.x; y = p.y; // update the current position
        updateTrack(floodMap, wallMap, x, y); // update the path
        // reset the flood map
        reset_flood_map(floodMap);

        if (isFinished(x, y)) break;
    }
    
    // clenup arrays
    delete[] floodMap;
    delete[] wallMap;
    free_queue(q); // delete the queue
}

// initializing function implementations
int **initializeFloodMap() {
    int **floodMap = new int*[API::mazeWidth()]; // create a 2D array of ints
    for (int i = 0; i < API::mazeWidth(); i++) {
        floodMap[i] = new int[API::mazeHeight()];
    }
    // set the flood index of all the cells to undefined and set the middle 4 cells to 0
    reset_flood_map(floodMap);

    return floodMap;
}

void reset_flood_map(int** floodMap) {
    for (int i = 0; i < API::mazeWidth(); i++) {
        for (int j = 0; j < API::mazeHeight(); j++) {
            floodMap[i][j] = UNDEFINED;
        }
    }

    // calculate the middle of the maze
    int middleX = API::mazeWidth() / 2;
    int middleY = API::mazeHeight() / 2;
    // fill the middle 4 cells with 0
    floodMap[FINISHING_X][FINISHING_Y] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y] = 0;
    floodMap[FINISHING_X][FINISHING_Y + 1] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y + 1] = 0;
}

int **initializeWallMap() {
    int **wallMap = new int*[API::mazeWidth()]; // create a 2D array of ints
    for (int i = 0; i < API::mazeWidth(); i++) {
        wallMap[i] = new int[API::mazeHeight()];
    }

    for (int i = 0; i < API::mazeWidth(); i++) {
        for (int j = 0; j < API::mazeHeight(); j++) {
            wallMap[i][j] = WALL_NONE;
        }
    }
    // setup the boundary walls of the maze;
    for (int i = 0 ; i < API::mazeWidth(); i++) {
        wallMap[i][0] |= WALL_BACK;
        wallMap[i][API::mazeHeight() - 1] |= WALL_FRONT;
    }

    for (int j = 0; j < API::mazeHeight(); j++) {
        wallMap[0][j] |= WALL_LEFT;
        wallMap[API::mazeWidth() - 1][j] |= WALL_RIGHT;
    }

    return wallMap;
}


// updating function implementations
void update_flood_map(int **floodMap, int** wallMap, queue *q) {
    // reset the given queue
    queue_reset(q);

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

Point getNextFloodIndex(int x, int y, Orient orient, int direction) {
    // get the absolute direction using the current orientation and the direction
    Orient absOrientation = getAbsDirection(orient, direction);
    
    // based on abs direction return the next cell
    switch (absOrientation)
    {
    case NORTH:
        return {x, y + 1};
    case EAST:
        return {x + 1, y};
    case SOUTH:
        return {x, y - 1};
    case WEST:
        return {x - 1, y};
    default:
        break;
    }

    return {-1, -1};
}

void setWall(int **wallMap, int x, int y, int wall) {
    if (x >= 0 && y >= 0 && x < API::mazeWidth() && y < API::mazeHeight())
        wallMap[x][y] |= wall;

    // based on direction set the wall on the maze
    switch (wall)
    {
    case WALL_LEFT:
        API::setWall(x, y, 'w');
        break;
    case WALL_RIGHT:
        API::setWall(x, y, 'e');
        break;
    case WALL_FRONT:
        API::setWall(x, y, 'n');
        break;
    case WALL_BACK:
        API::setWall(x, y, 's');
        break;
    default:
        break;
    }
}

void updateWalls(int **wallMap, int x, int y, Orient orient, int direction) {

    Orient absOrient = getAbsDirection(orient, direction);

    switch (absOrient)
    {
    case NORTH:
        setWall(wallMap, x, y, WALL_FRONT);
        setWall(wallMap, x, y + 1, WALL_BACK);
        break;
    case EAST:
        setWall(wallMap, x, y, WALL_RIGHT);
        setWall(wallMap, x + 1, y, WALL_LEFT);
        break;
    case SOUTH:
        setWall(wallMap, x, y, WALL_BACK);
        setWall(wallMap, x, y - 1, WALL_FRONT);
        break;
    case WEST:
        setWall(wallMap, x, y, WALL_LEFT);
        setWall(wallMap, x - 1, y, WALL_RIGHT);
        break;
    default:
        break;
    }

}

bool isFinished(int x, int y) {

    if ((x == FINISHING_X || x == FINISHING_X + 1) && (y == FINISHING_Y || y == FINISHING_Y + 1)) {
        return true;
    }
    return false;
}

void printFloodMap(int** floodMap) {
    for (int i = 0; i < API::mazeWidth(); i++) {
        for (int j = 0; j < API::mazeHeight(); j++) {
            API::setText(i, j, std::to_string(floodMap[i][j]));
        }
    }
}