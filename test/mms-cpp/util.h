#ifndef UTIL_H
#define UTIL_H

bool isLeftWall(int **wallMap, Point current);
bool isRightWall(int **wallMap, Point current);
bool isFrontWall(int **wallMap, Point current);
bool isBackWall(int **wallMap, Point current);

Orient getAbsDirection(Orient currentOrient, int direction);
bool isReverseDirection(Orient dir1, Orient dir2);
int getDirection(Orient current_dir, Orient other_dir);
#endif