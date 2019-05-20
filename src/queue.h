/*
 * queue.h
 *
 *  Created on: Apr 13, 2019
 *      Author: ialex
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include <stdio.h>
#include "diag/Trace.h"

// C program for array implementation of queue
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

// A structure to represent a queue
struct Queue
{
    int front, rear, size;
    int capacity;
    uint16_t* array;
};

struct Queue* createQueue(int capacity);

int isFull(struct Queue* queue);

int isEmpty(struct Queue* queue);

void enqueue(struct Queue* queue, int item);

uint16_t dequeue(struct Queue* queue);

uint16_t front(struct Queue* queue);

uint16_t rear(struct Queue* queue);

#endif /* QUEUE_H_ */
