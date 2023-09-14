#ifndef UTIL_H
#define UTIL_H

bool isLeftWall(int **wallMap, Point current);
bool isRightWall(int **wallMap, Point current);
bool isFrontWall(int **wallMap, Point current);
bool isBackWall(int **wallMap, Point current);

Orient getAbsDirection(Orient currentOrient, int direction);


#endif