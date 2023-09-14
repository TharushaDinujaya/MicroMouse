#include"types.h"

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

Orient getAbsDirection(Orient orient, int direction) {
    int tmp = (orient + direction - OFFSET) % 4;
    if (tmp < 0) tmp += 4;

    return (Orient) (tmp + OFFSET);
}