#include "queue.h"

// queue function implemenettions
queue *create_queue(int size) {
    queue* q = new queue;
    q->arr = new Point[size];
    q->size = size;
    q->first = -1;
    q->last = -1;

    return q;
}

void enqueue(queue *q, Point value) {
    if ((q->last + 1) % q->size == q->first) throw -1; // queue is full

    q->last = (q->last + 1) % q->size;
    q->arr[q->last] = value;
}

Point dequeue(queue *q) {
    if (q->first == q->last) throw -1; // queue is empty

    q->first = (q->first + 1) % q->size;
    return q->arr[q->first];
    
}

bool empty(queue *q) {
    return q->first == q->last;
}

void free_queue(queue *q) {
    delete[] q->arr;
    delete q;
}

// end of queue implementations

