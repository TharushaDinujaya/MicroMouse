#include"maze.h"

#ifndef STACK_H
#define STACK_H

struct Stack {
    Point* stack;
    int top;
    int size;
};

#define DEFAULT_SIZE 20

Stack* createStack(int size = DEFAULT_SIZE);
void deleteStack(Stack* stack);
void resetStack(Stack* stack);
Point pop(Stack* stack);
void push(Stack* stack, Point point);
bool empty(Stack* stack);

#endif