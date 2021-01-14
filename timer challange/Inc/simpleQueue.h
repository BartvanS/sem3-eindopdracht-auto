#ifndef SIMPLE_QUEUE_H
#define SIMPLE_QUEUE_H
// #include "main.h"
// #include "cmsis_os.h"
// #include <string.h>
// #include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct simpleQueue
{
    char* value;
    simpleQueue* nextSQ;
} SimpleQueue;

void addToQueue(SimpleQueue** queue, char* value);
char* retrieveFromQueue(SimpleQueue** queue);

#endif