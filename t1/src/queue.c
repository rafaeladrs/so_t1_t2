#include "queue.h"

#include <assert.h>
#include <stdlib.h>

struct queue {
    int* items;
    int capacity;
    int size;
    int front;
    int back;
};

Queue* queue_create(int capacity) {
    int* items = malloc(sizeof(int) * capacity);
    Queue* queue = malloc(sizeof(Queue));

    *queue = (Queue) {
        .items = items,
        .capacity = capacity,
        .size = 0,
        .front = -1,
        .back = -1,
    };

    return queue;
}

void queue_destroy(Queue* queue) {
    free(queue->items);
    free(queue);
}

int queue_capacity(Queue* queue) {
    return queue->capacity;
}

int queue_size(Queue* queue) {
    return queue->size;
}

bool queue_push(Queue* queue, int value) {
    if (queue->size == queue->capacity) return false;

    if (queue->front == -1) queue->front = 0;
    queue->back = (queue->back + 1) % queue->capacity;
    queue->items[queue->back] = value;

    queue->size++;

    return true;
}

int queue_pop(Queue* queue) {
    assert(queue->size > 0);

    int x = queue->items[queue->front];

    if (queue->front == queue->back) {
        queue->front = -1;
        queue->back = -1;
    } else {
        queue->front = (queue->front + 1) % queue->capacity;
    }

    queue->size--;

    return x;
}
