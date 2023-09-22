#ifndef QUEUE_H
#define QUEUE_H

#include "maze.h"

struct Queue
{
    Point *queue;
    int front;
    int back;
    int size;
};

#define DEFAULT_SIZE 100

Queue *getQueue();
void resetQueue();
Point dequeue();
void enqueue(Point point);
bool queueEmpty();

#endif