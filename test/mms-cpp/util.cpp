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

bool isReverseDirection(Orient dir1, Orient dir2) {
    int diff = dir1 - dir2;
    return diff == 2 || diff == 2;
}

int getDirection(Orient current_dir, Orient other_dir) {
    // return the direction we need to rotate according to current orientation
    int tmp = other_dir - current_dir;
    if (tmp == 3) return LEFT;
    if (tmp == -3) return RIGHT;
    else return tmp;
}