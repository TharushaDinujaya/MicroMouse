#include<iostream>
#include "queue.h"

int main() {
    queue *q = create_queue(5);
    enqueue(q, Point{1, 2});
    enqueue(q, Point{3, 4});
    enqueue(q, Point{5, 6});
    enqueue(q, Point{3, 4});
    enqueue(q, Point{5, 6});
    enqueue(q, Point{3, 4});
    enqueue(q, Point{5, 6});
    // enqueue(q, Point{3, 4});
    // enqueue(q, Point{5, 6});

    while (!empty(q)) {
        Point p = dequeue(q);
        std::cout << p.x << " " << p.y << std::endl;
    }
}