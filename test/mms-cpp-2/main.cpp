#include <iostream>
#include <string>

#include "maze.h"
#include "queue.h"
#include "solver.h"
#include "API.h"

int pathLength = 0;
char pathString[700];
void searchRun();
void returnToStart();

void log(const std::string &text)
{
    std::cerr << text << std::endl;
}

int main(int argc, char *argv[])
{
    log("Running...");
    API::setColor(0, 0, 'A');
    API::setText(0, 0, "abc");

    initializeMaze();
    for (int i = 0; i < 5; i++)
    {
        searchRun();
        log("search run finished");
        returnToStart();
    }
}

void searchRun()
{
    pathLength = 0;
    int X = 0, Y = 0;
    Orient orient = NORTH;
    while (true)
    {
        // printFloodMap();
        // check for walls
        if (API::wallLeft())
        {
            setWalls(Point{X, Y}, orient, LEFT);
            API::setWall(X, Y, 'l');
        }

        if (API::wallRight())
        {
            setWalls(Point{X, Y}, orient, RIGHT);
            API::setWall(X, Y, 'r');
        }

        if (API::wallFront())
        {
            setWalls(Point{X, Y}, orient, FORWARD);
            API::setWall(X, Y, 'f');
        }

        // flooded the map
        flooded();
        // find the next point
        Point next = findNext({X, Y}, orient);
        if (next.x == X && next.y == Y)
        {
            API::turnRight();
            // API::turnRight();
            orient = getAbsDirection(orient, RIGHT);
            // orient = getAbsDirection(orient, RIGHT);
            pathString[pathLength++] = 'R';
            // pathString[pathLength++] = 'R';
            continue;
        }

        // find the direction
        int direction = findDirection({X, Y}, next, orient);
        if (direction == LEFT)
        {
            API::turnLeft();
            pathString[pathLength++] = 'L';
            orient = getAbsDirection(orient, direction);
        }
        else if (direction == RIGHT)
        {
            API::turnRight();
            pathString[pathLength++] = 'R';
            orient = getAbsDirection(orient, direction);
        }
        // else if (direction == ROTATE)
        // {
        //     API::turnRight();
        //     API::turnRight();
        //     pathString[pathLength++] = 'R';
        //     pathString[pathLength++] = 'R';
        //     orient = getAbsDirection(orient, RIGHT);
        //     orient = getAbsDirection(orient, RIGHT);
        // }
        API::moveForward();
        pathString[pathLength++] = 'F';

        // update the position
        X = next.x;
        Y = next.y;

        if (isFinished({X, Y}))
            break;
    }
}

void returnToStart()
{
    // first reversed the mouse
    API::turnRight();
    API::turnRight();

    for (int i = 0; i < pathLength; i++)
    {
        std::cerr << pathString[i] << ", ";
    }

    for (int i = pathLength - 1; i >= 0; i--)
    {
        char c = pathString[i];
        if (c == 'L')
        {
            API::turnRight();
        }
        else if (c == 'R')
        {
            API::turnLeft();
        }
        else if (c == 'F')
        {
            API::moveForward();
        }
    }

    API::turnRight();
    API::turnRight();
}
