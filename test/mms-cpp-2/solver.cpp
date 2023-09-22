#include "solver.h"
#include "queue.h"
#include "maze.h"
#include <string>
#include <iostream>
#include "API.h"

// declare the 2D int arrays fo represent wall structure and flood index map
int wallMap[MAZE_SIZE][MAZE_SIZE];
int floodMap[MAZE_SIZE][MAZE_SIZE];

// helper functions declarations

// function for check whether particular wall exists in the given cell
bool leftWall(Point p);
bool rightWall(Point p);
bool frontWall(Point p);
bool backWall(Point p);
bool isReversed(Orient orient1, Orient orient2);                 // function for hceck whether the given orientations are in reversed directions
bool isValid(Point p);                                           // check whether the given point is valid in the maze
Point getNextPoint(Point current, Orient orient, int direction); // get the next point indicate by the direction and current point

// function for initialize the wall map and flood index map
void initializeMaze()
{
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            wallMap[i][j] = WALL_NONE;
            floodMap[i][j] = UNDEFINED;
        }
    }

    // fill  the border of the maze with walls
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        wallMap[0][i] |= WALL_LEFT;              // fill the left boundary walls
        wallMap[MAZE_SIZE - 1][i] |= WALL_RIGHT; // fill the right boundary walls
        wallMap[i][0] |= WALL_BACK;              // fill the lower boudnary walls
        wallMap[i][MAZE_SIZE - 1] |= WALL_FRONT; // fill teh higher boundary walls
    }
}

// function for reset the flood map downto initial state
void resetFloodMap()
{
    // first set the index of all cells to undefined state
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            floodMap[i][j] = UNDEFINED;
        }
    }

    // now set the destination cell flood index to zero
    floodMap[FINISHING_X][FINISHING_Y] = 0;
    floodMap[FINISHING_X][FINISHING_Y + 1] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y] = 0;
    floodMap[FINISHING_X + 1][FINISHING_Y + 1] = 0;
}

// function for completly flooded the maze with the flood indexes based on the wall map
void flooded()
{
    // first reset the complete flood map
    resetFloodMap();
    // reset the queue also
    resetQueue();

    // add the finishing coordinates to the queue
    enqueue({FINISHING_X, FINISHING_Y});
    enqueue({FINISHING_X, FINISHING_Y + 1});
    enqueue({FINISHING_X + 1, FINISHING_Y});
    enqueue({FINISHING_X + 1, FINISHING_Y + 1});

    while (!queueEmpty())
    {
        // extract the element from the queue
        Point p = dequeue();
        // get the flood index of extracted point
        int floodIndex = floodMap[p.x][p.y];
        int x = p.x, y = p.y;

        // check if the cell on the left is not a wall and the flood index is undefined
        if (!leftWall(p) && floodMap[x - 1][y] == UNDEFINED)
        {
            floodMap[x - 1][y] = floodIndex + 1;
            enqueue({x - 1, y});
        }

        // check if the cell on the right is not a wall and the flood index is undefined
        if (!rightWall(p) && floodMap[x + 1][y] == UNDEFINED)
        {
            floodMap[x + 1][y] = floodIndex + 1;
            enqueue({x + 1, y});
        }

        // check if the cell on the front is not a wall and the flood index is undefined
        if (!frontWall(p) && floodMap[x][y + 1] == UNDEFINED)
        {
            floodMap[x][y + 1] = floodIndex + 1;
            enqueue({x, y + 1});
        }

        // check if the cell on the back is not a wall and the flood index is undefined
        if (!backWall(p) && floodMap[x][y - 1] == UNDEFINED)
        {
            floodMap[x][y - 1] = floodIndex + 1;
            enqueue({x, y - 1});
        }
    }
}

void setWalls(Point current, Orient orient, int direction)
{
    // get the next point and absolute orientation
    Point next = getNextPoint(current, orient, direction);
    Orient absOrt = getAbsDirection(orient, direction);
    int x = current.x, y = current.y;

    int currentWall = WALL_NONE, nextWall = WALL_NONE;

    switch (absOrt)
    {
    case NORTH:
        currentWall = WALL_FRONT;
        nextWall = WALL_BACK;
        break;
    case SOUTH:
        currentWall = WALL_BACK;
        nextWall = WALL_FRONT;
        break;
    case EAST:
        currentWall = WALL_RIGHT;
        nextWall = WALL_LEFT;
        break;
    case WEST:
        currentWall = WALL_LEFT;
        nextWall = WALL_RIGHT;
        break;
    default:
        break;
    }

    wallMap[current.x][current.y] |= currentWall;
    if (isValid(next))
    {
        wallMap[next.x][next.y] |= nextWall;
    }

    // based on direction set the wall on the maze
    switch (currentWall)
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

Point findNext(Point point, Orient orient)
{
    // find the cell with the minimum flood index
    int minFloodIndex = UNDEFINED;
    Point next = point;
    int x = point.x, y = point.y;

    if (!leftWall(point) && floodMap[x - 1][y] < minFloodIndex && !isReversed(orient, WEST))
    {
        minFloodIndex = floodMap[x - 1][y];
        next = {x - 1, y};
    }

    if (!rightWall(point) && floodMap[x + 1][y] < minFloodIndex && !isReversed(orient, EAST))
    {
        minFloodIndex = floodMap[x + 1][y];
        next = {x + 1, y};
    }

    if (!frontWall(point) && floodMap[x][y + 1] < minFloodIndex && !isReversed(orient, NORTH))
    {
        minFloodIndex = floodMap[x][y + 1];
        next = {x, y + 1};
    }

    if (!backWall(point) && floodMap[x][y - 1] < minFloodIndex && !isReversed(orient, SOUTH))
    {
        minFloodIndex = floodMap[x][y - 1];
        next = {x, y - 1};
    }

    if (minFloodIndex == UNDEFINED)
        return point;

    return next;
}

int findDirection(Point current, Point next, Orient orient)
{

    Orient nextCellOrient;

    if (next.y == current.y + 1)
        nextCellOrient = NORTH;
    else if (next.y == current.y - 1)
        nextCellOrient = SOUTH;
    else if (next.x == current.x + 1)
        nextCellOrient = EAST;
    else if (next.x == current.x - 1)
        nextCellOrient = WEST;

    // return the direction we need to rotate according to current orientation
    int tmp = nextCellOrient - orient;
    if (tmp == 3)
        return LEFT;
    if (tmp == -3)
        return RIGHT;
    else
        return tmp;
}

bool isFinished(Point current)
{
    int x = current.x, y = current.y;

    return (x == FINISHING_X || x == FINISHING_X + 1) && (y == FINISHING_Y || y == FINISHING_Y + 1);
}

// helper function implementations
bool leftWall(Point p)
{
    return wallMap[p.x][p.y] & WALL_LEFT;
}

bool rightWall(Point p)
{
    return wallMap[p.x][p.y] & WALL_RIGHT;
}

bool frontWall(Point p)
{
    return wallMap[p.x][p.y] & WALL_FRONT;
}

bool backWall(Point p)
{
    return wallMap[p.x][p.y] & WALL_BACK;
}

bool isReversed(Orient orient1, Orient orient2)
{
    int diff = orient1 - orient2;
    return diff == -2 || diff == 2;
}

bool isValid(Point p)
{
    return p.x >= 0 && p.x < MAZE_SIZE && p.y >= 0 && p.y < MAZE_SIZE;
}

Orient getAbsDirection(Orient orient, int direction)
{
    int tmp = (orient + direction - ORIENT_OFFSET) % 4;
    if (tmp < 0)
        tmp += 4;

    return (Orient)(tmp + ORIENT_OFFSET);
}

Point getNextPoint(Point current, Orient orient, int direction)
{
    // get the absolute direction first
    Orient absOrt = getAbsDirection(orient, direction);
    int x = current.x, y = current.y;

    switch (absOrt)
    {
    case NORTH:
        return {x, y + 1};
    case SOUTH:
        return {x, y - 1};
    case EAST:
        return {x + 1, y};
    case WEST:
        return {x - 1, y};
    }

    return current; // never reach to this point
}

void printFloodMap()
{
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            std::cerr << wallMap[i][j] << " ";
        }
        std::cerr << "\n";
    }
}