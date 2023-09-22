#include "stack.h"

Point stack_arr[DEFAULT_SIZE];
Stack stack = {stack_arr, 0, DEFAULT_SIZE};

// get a pointer to the stack
Stack *getStack()
{
    return &stack;
}

// reset the stack
void resetStack()
{
    stack.top = 0;
}

// get the top element in the stack
Point pop()
{
    Point point = stack.stack[stack.top];
    stack.top--;
    return point;
}

// insert an element into the stack
void push(Point point)
{
    stack.top++;
    stack.stack[stack.top] = point;
}

// check whether the stack is empty
bool emptyStack()
{
    return stack.top == 0;
}
