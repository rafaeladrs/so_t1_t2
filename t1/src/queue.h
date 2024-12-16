#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>

typedef struct queue Queue;

Queue* queue_create(int capacity);

void queue_destroy(Queue* queue);

int queue_capacity(Queue* queue);

int queue_size(Queue* queue);

bool queue_push(Queue* queue, int value);

int queue_pop(Queue* queue);

#endif // QUEUE_H
