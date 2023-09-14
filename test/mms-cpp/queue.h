#include "types.h"

#ifndef QUEUE_H
#define QUEUE_H
#define QUEUE_SIZE 100

struct queue {
    Point* arr;
    int size;
    int first;
    int last;
};

queue *create_queue(int size = QUEUE_SIZE);
void queue_reset(queue *q);
void enqueue(queue *q, Point value);
Point dequeue(queue *q);
bool empty(queue *q);
void free_queue(queue *q);

#endif // QUEUE_H