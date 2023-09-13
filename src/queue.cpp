#include"maze.h"
#include"queue.h"

using namespace std;

bool empty(Queue* queue){
    return (queue->front == queue->back);
}


Queue* createQueue(int size = DEFAULT_SIZE){

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
    if (q->front == q->back) throw -1; // queue is empty

    q->front = (q->front + 1) % q->size;
    return q->queue[q->front];
    
}



void enqueue(Queue *q, Point value) {
    if ((q->back + 1) % q->size == q->front) throw -1; // queue is full

    q->back = (q->back + 1) % q->size;
    q->queue[q->back] = value;
}
