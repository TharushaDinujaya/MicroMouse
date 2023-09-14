#include"maze.h"
#include"queue.h"

bool empty(Queue* queue){
    return (queue->front == queue->back);
}


Queue* createQueue(int size){

    Queue* queue = new Queue;
    queue->queue = new Point[size];
    queue->front = -1;
    queue->back = -1;
    queue->size = size;

    return queue;
}

void deleteQueue(Queue* queue){
    delete[] queue->queue;
    delete queue;
}


void resetQueue(Queue* queue){
    queue->front = -1;
    queue->back = -1;

}


Point dequeue(Queue *q) {
    if (q->front == q->back) return {0, 0}; // queue is empty

    q->front = (q->front + 1) % q->size;
    return q->queue[q->front];
    
}


void enqueue(Queue *q, Point value) {
    if ((q->back + 1) % q->size == q->front) return; // queue is full

    q->back = (q->back + 1) % q->size;
    q->queue[q->back] = value;
}
