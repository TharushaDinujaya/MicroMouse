#ifndef MOUSE_H
#define MOUSE_H

#include "user.h"

bool run(); // main running loop of the micromouse
void returnToStart(const char pathString[MAX_PATH_LENGTH], int pathLength); // return to the start point

#endif