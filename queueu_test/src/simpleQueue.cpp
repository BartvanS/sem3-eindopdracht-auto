#include "simpleQueue.h"

void addToQueue(SimpleQueue** queue, char* value){
     SimpleQueue* newElement = (SimpleQueue *)malloc(sizeof(SimpleQueue));
     newElement->value = value;
     newElement->nextSQ = NULL;
    if(*queue == NULL){
        *queue = newElement;
    }
    else
    {

        SimpleQueue* currentElement = *queue;
        while(currentElement->nextSQ != NULL)
        {      
            currentElement = currentElement->nextSQ;
        }
        currentElement->nextSQ = newElement;
    }
}

char* retrieveFromQueue(SimpleQueue** queue){
    if(*queue == NULL){
        //error:empty ingekort voor efficientie
        return (char*)"e:e";
    }
   
    SimpleQueue* next_item = (*queue)->nextSQ;
    char* tmpVal = (*queue)->value;
    free(*queue);
    *queue = next_item;
    return tmpVal;
}
