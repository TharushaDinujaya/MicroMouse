#include"maze.h"
#include"stack.h"

using namespace std;

bool empty(Stack* stack){
    return (stack->top == 0);
}



Stack* createStack(int size = DEFAULT_SIZE){
    
        Stack* stack = new Stack;
        stack->stack = new Point[size];
        stack->top = 0;
        stack->size = size;
    
        return stack;

}
void deleteStack(Stack* stack){
    delete[] stack->stack;
    delete stack;
}

void resetStack(Stack* stack){
    stack->top = 0;
}

Point pop(Stack* stack){
    if (empty(stack)){
        return;
    }
    else{
        Point p = stack->stack[ -- (stack->top) ];
        return p;
    }
}

void push(Stack* stack, Point point){
    if (stack->top == stack->size){
        
        return;
    }
    else{
        stack->stack[stack->top ++] = point;
    }
}


