#ifndef STACK_H
#define STACK_H

#include "maze.h"

struct Stack
{
    Point *stack;
    int top;
    int size;
};

#define DEFAULT_SIZE 100

Stack *getStack();
void resetStack();
Point pop();
void push(Point point);
bool emptyStack();

#endif