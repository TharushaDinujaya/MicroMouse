#include"maze.h"

#ifndef QUEUE_H
#define QUEUE_H

struct Queue {
    Point* queue;
    int front;
    int back;
    int size;
};

#define DEFAULT_SIZE 20

Queue* createQueue(int size = DEFAULT_SIZE);
void deleteQueue(Queue* queue);
void resetQueue(Queue* queue);
Point dequeue(Queue* queue);
void enqueue(Queue* queue, Point point);
bool empty(Queue* queue);

#endif