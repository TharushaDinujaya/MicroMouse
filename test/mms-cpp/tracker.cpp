#include "types.h"
#include "util.h"
#include "tracker.h"
#include "API.h"
#include <iostream>

#define MAX_COUNT 300

// color symbol for the maze
#define RED 'R'
#define GREEN 'G'
#define DEFAULT 'A'

Point getMinimumNeighbour(int **floodMap, int** wallMap, int x, int y);
void renderColorMap();
void unregisterTrack();
bool isValid(Point point);

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

// define static global color map for this maze
static char colorMap[MAZE_WIDTH][MAZE_HEIGHT] = {DEFAULT};

// define theh color map for this maze
void updateTrack(int** floodMap, int** wallMap, int x, int y) {
    
    // set the previous path as RED
    unregisterTrack();

    // update the color map
    // starting from the starting position
    int count = 0;
    while (floodMap[x][y] != 0 && count < MAX_COUNT) {
        // get the neighbor with the lowest flood value
        Point nextPoint = getMinimumNeighbour(floodMap, wallMap, x, y);
        // set color of this point to green
        colorMap[x][y] = GREEN;
        x = nextPoint.x;
        y = nextPoint.y;
        count++;
    }

    // finally render the color map
    renderColorMap();
}

Point getMinimumNeighbour(int **floodMap, int** wallMap, int x, int y) {
    Point p = {x, y};
    int value_N = INT32_MAX, value_E = INT32_MAX, value_W = INT32_MAX, value_S = INT32_MAX;
    if (isValid({x, y + 1})) value_N = floodMap[x][y + 1];
    if (isValid({x + 1, y})) value_E = floodMap[x + 1][y];
    if (isValid({x - 1, y})) value_W = floodMap[x - 1][y];
    if (isValid({x, y - 1})) value_S = floodMap[x][y - 1];

    int min = value_W;
    Point minPoint = {x - 1, y};

    if (value_N < min) {
        min = value_N;
        minPoint = {x, y + 1};
    }

    if (value_E < min) {
        min = value_E;
        minPoint = {x + 1, y};
    }


    if (value_S < min) {
        min = value_S;
        minPoint = {x, y - 1};
    }

    return minPoint;
}

void renderColorMap() {
    for (int i = 0; i < API::mazeWidth(); i++) {
        for (int j = 0; j < API::mazeHeight(); j++)
        {   
            API::setColor(i, j, colorMap[i][j]);
        }
        
    }
}

void unregisterTrack() {

    for (int i = 0; i < API::mazeWidth(); i++) {
        for (int j = 0; j < API::mazeHeight(); j++)
        {   
            // if (colorMap[i][j] == GREEN) {
            //     colorMap[i][j] = RED;
            // } else if (colorMap[i][j] == RED) {
            //     colorMap[i][j] = DEFAULT;
            // }
            colorMap[i][j] = DEFAULT;
        }
        
    }
}

bool isValid(Point point) {
    return point.x >= 0 && point.x < API::mazeWidth() && point.y >= 0 && point.y < API::mazeHeight();
}