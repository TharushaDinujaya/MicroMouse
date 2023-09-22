#include "queue.h"

Point queue_arr[DEFAULT_SIZE];
Queue queue = {queue_arr, 0, 0, DEFAULT_SIZE};

// get a pointer to the queue
Queue *getQueue()
{
    return &queue;
}

// reset the queue
void resetQueue()
{
    queue.front = 0;
    queue.back = 0;
}

// get the first element in the queue
Point dequeue()
{
    Point point = queue.queue[queue.front];
    queue.front = (queue.front + 1) % queue.size;
    return point;
}

// insert an element into the queue
void enqueue(Point point)
{
    queue.queue[queue.back] = point;
    queue.back = (queue.back + 1) % queue.size;
}

// check whether the queue is empty
bool queueEmpty()
{
    return queue.front == queue.back;
}