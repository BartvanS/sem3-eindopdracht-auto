#include "simpleQueue.h"

void addToQueue(SimpleQueue** queue, char* value){
    if(*queue == NULL){
        SimpleQueue* newElement = (SimpleQueue *)malloc(sizeof(SimpleQueue));

        newElement->value = value;
        newElement->nextSQ = NULL;
        *queue = newElement;
    }else{
        SimpleQueue* newElement = *queue;
        while(newElement->nextSQ != NULL){      
            newElement = newElement->nextSQ;
        }
        newElement->nextSQ = (SimpleQueue *)malloc(sizeof(SimpleQueue));
        newElement->nextSQ->value = value;
        newElement->nextSQ->nextSQ = NULL;
    }
}

char* retrieveFromQueue(SimpleQueue** queue){
    if(*queue == NULL){
        //error:empty ingekort voor efficientie
        return (char*)"e:e\r\n";
    }
    SimpleQueue* next_item = (*queue)->nextSQ;
    char* tmpVal = (*queue)->value;
    free(*queue);
    *queue = next_item;
    return tmpVal;
}
