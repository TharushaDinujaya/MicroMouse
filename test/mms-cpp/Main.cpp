#include <iostream>
#include <string>

#include "API.h"
#include"queue.h"
#include"types.h"

#define UNDEFINED 99999

// function declarations
int **initializeFloodMap();
int **initializeWallMap();
void reset_flood_map(int **floodMap);

// other helper functions
bool isLeftWall(int **wallMap, Point current);
bool isRightWall(int **wallMap, Point current);
bool isFrontWall(int **wallMap, Point current);
bool isBackWall(int **wallMap, Point current);

Point getFloodIndex(int **floodMap, int x, int y, Orient orient, int direction);
Orient updateOrient(Orient currentOrient, int direction);
void updateWalls(int **wallMap, int x, int y, Orient orient, int direction);
void setWall(int **wallMap, int x, int y, int wall);
bool isFinished(int x, int y);

void update_flood_map(int **floodMap, int** wallMap);

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

int main(int argc, char* argv[]) {
    // create an array of ints to represent the flood map
    int** floodMap = initializeFloodMap();
    int** wallMap = initializeWallMap();

    log("Running...");
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");

    // starting position of the maze
    int x = 0, y = 0;
    Orient orient = NORTH;
    while (true) {
        update_flood_map(floodMap, wallMap);

        int leftIndex = UNDEFINED, rightIndex = UNDEFINED, frontIndex = UNDEFINED;
        Point next = {-1, -1};
        int min_index = LEFT;
        // observe the existence of the walls
        // check LEFT
        if (!API::wallLeft()) {
            Point p = getFloodIndex(floodMap, x, y, orient, LEFT);
            leftIndex = floodMap[p.x][p.y];
        } else {
            log("left detected");
            // update the wall map
            updateWalls(wallMap, x, y, orient, LEFT);
        }
        // check RIGHT
        if (!API::wallRight()) {
            Point p = getFloodIndex(floodMap, x, y, orient, RIGHT);
            rightIndex = floodMap[p.x][p.y];
        } else {
            log("right detected");
            updateWalls(wallMap, x, y, orient, RIGHT);
        }
        // check FRONT
        if (!API::wallFront()) {
            Point p = getFloodIndex(floodMap, x, y, orient, FORWARD);
            frontIndex = floodMap[p.x][p.y];
        } else {
            log("front detected");
            updateWalls(wallMap, x, y, orient, FORWARD);
        }

        // find the minimum flood index
        int min = leftIndex;
        if (min > rightIndex) {
            min = rightIndex;
            min_index = RIGHT;
        }
        if (min > frontIndex) {
            min = frontIndex;
            min_index = FORWARD;
        }

        if (min == UNDEFINED) {
            API::turnRight();
            orient = updateOrient(orient, RIGHT);
            // reset the flood map
            reset_flood_map(floodMap);
            continue;
        }

        Point p;
        // move to the cell with the minimum flood index
        if (min_index == LEFT) {
            API::turnLeft();
            API::moveForward();
            p = getFloodIndex(floodMap, x, y, orient, LEFT);  
            orient = updateOrient(orient, LEFT); // update the current position
        } else if (min_index == RIGHT) {
            API::turnRight();
            API::moveForward();
            p = getFloodIndex(floodMap, x, y, orient, RIGHT);
            orient = updateOrient(orient, RIGHT); // update the current position
        } else if (min_index == FORWARD) {
            API::moveForward();
            p = getFloodIndex(floodMap, x, y, orient, FORWARD); // update the current position
        }
        x = p.x; y = p.y; // update the current position
        // reset the flood map
        reset_flood_map(floodMap);

        if (isFinished(x, y)) break;
    }
    
    // clenup arrays
    delete[] floodMap;
    delete[] wallMap;
}

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
    floodMap[middleX][middleY] = 0;
    floodMap[middleX - 1][middleY] = 0;
    floodMap[middleX][middleY - 1] = 0;
    floodMap[middleX - 1][middleY - 1] = 0;
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

void update_flood_map(int **floodMap, int** wallMap) {
    // create a new queue to keep track of the cells to be updated
    queue *q = create_queue();

    // calculate the middle of the maze
    int middleX = API::mazeWidth() / 2;
    int middleY = API::mazeHeight() / 2;
    // add the middle 4 cells to the queue
    enqueue(q, {middleX, middleY});
    enqueue(q, {middleX + 1, middleY});
    enqueue(q, {middleX, middleY + 1});
    enqueue(q, {middleX + 1, middleY + 1});

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
    // delete the queue
    free_queue(q);
}

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

Point getFloodIndex(int **floodMap, int x, int y, Orient orient, int direction) {

    if (orient == NORTH) {
        if (direction == LEFT) {
            return {x - 1, y};
        } else if (direction == RIGHT) {
            return {x + 1, y};
        } else if (direction == FORWARD) {
            return {x, y + 1};
        }
    } else if (orient == EAST) {
        if (direction == LEFT) {
            return {x, y + 1};
        } else if (direction == RIGHT) {
            return {x, y - 1};
        } else if (direction == FORWARD) {
            return {x + 1, y};
        }
    } else if (orient == SOUTH) {
        if (direction == LEFT) {
            return {x + 1, y};
        } else if (direction == RIGHT) {
            return {x - 1, y};
        } else if (direction == FORWARD) {
            return {x, y - 1};
        }
    } else if (orient == WEST) {
        if (direction == LEFT) {
            return {x, y - 1};
        } else if (direction == RIGHT) {
            return {x, y + 1};
        } else if (direction == FORWARD) {
            return {x - 1, y};
        }
    }
    return {-1, -1};
}

Orient updateOrient(Orient currentOrient, int direction) {
    if (currentOrient == NORTH) {
        if (direction == LEFT) {
            return WEST;
        } else if (direction == RIGHT) {
            return EAST;
        } else if (direction == FORWARD) {
            return NORTH;
        }
    } else if (currentOrient == EAST) {
        if (direction == LEFT) {
            return NORTH;
        } else if (direction == RIGHT) {
            return SOUTH;
        } else if (direction == FORWARD) {
            return EAST;
        }
    } else if (currentOrient == SOUTH) {
        if (direction == LEFT) {
            return EAST;
        } else if (direction == RIGHT) {
            return WEST;
        } else if (direction == FORWARD) {
            return SOUTH;
        }
    } else if (currentOrient == WEST) {
        if (direction == LEFT) {
            return SOUTH;
        } else if (direction == RIGHT) {
            return NORTH;
        } else if (direction == FORWARD) {
            return WEST;
        }
    }

    return NORTH;
}

void setWall(int **wallMap, int x, int y, int wall) {
    if (x >= 0 && y >= 0 && x < API::mazeWidth() && y < API::mazeHeight())
        wallMap[x][y] |= wall;
}

void updateWalls(int **wallMap, int x, int y, Orient orient, int direction) {

    // update the appropriate walls based on the orientation and direction
    switch (orient)
    {
    case NORTH:
        if (direction == LEFT) {
            setWall(wallMap, x, y, WALL_LEFT);
            setWall(wallMap, x - 1, y, WALL_RIGHT);
        } else if (direction == RIGHT) {
            setWall(wallMap, x, y, WALL_RIGHT);
            setWall(wallMap, x + 1, y, WALL_LEFT);
        } else if (direction == FORWARD) {
            setWall(wallMap, x, y, WALL_FRONT);
            setWall(wallMap, x, y + 1, WALL_BACK);
        }
        break;

    case EAST:
        if (direction == LEFT) {
            setWall(wallMap, x, y, WALL_FRONT);
            setWall(wallMap, x, y + 1, WALL_BACK);
        } else if (direction == RIGHT) {
            setWall(wallMap, x, y, WALL_BACK);
            setWall(wallMap, x, y - 1, WALL_FRONT);
        } else if (direction == FORWARD) {
            setWall(wallMap, x, y, WALL_RIGHT);
            setWall(wallMap, x + 1, y, WALL_LEFT);
        }
        break;

    case SOUTH:
        if (direction == LEFT) {
            setWall(wallMap, x, y, WALL_RIGHT);
            setWall(wallMap, x + 1, y, WALL_LEFT);
        } else if (direction == RIGHT) {
            setWall(wallMap, x, y, WALL_LEFT);
            setWall(wallMap, x - 1, y, WALL_RIGHT);
        } else if (direction == FORWARD) {
            setWall(wallMap, x, y, WALL_BACK);
            setWall(wallMap, x, y - 1, WALL_FRONT);
        }
        break;

    case WEST:
        if (direction == LEFT) {
            setWall(wallMap, x, y, WALL_BACK);
            setWall(wallMap, x, y - 1, WALL_FRONT);
        } else if (direction == RIGHT) {
            setWall(wallMap, x, y, WALL_FRONT);
            setWall(wallMap, x, y + 1, WALL_BACK);
        } else if (direction == FORWARD) {
            setWall(wallMap, x, y, WALL_LEFT);
            setWall(wallMap, x - 1, y, WALL_RIGHT);
        }
        break;
    
    default:
        break;
    }

}

bool isFinished(int x, int y) {
    int middleX = API::mazeWidth() / 2;
    int middleY = API::mazeHeight() / 2;

    if ((x == middleX || x == middleX - 1) && (y == middleY || y == middleY - 1)) {
        return true;
    }
    return false;
}