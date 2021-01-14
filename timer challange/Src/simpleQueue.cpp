#include "simpleQueue.h"
// static char msgBuf[80];

void addToQueue(SimpleQueue** queue, char* value){
     SimpleQueue* newElement = (SimpleQueue *)malloc(sizeof(SimpleQueue));
     strcpy(newElement->value, value); 
     newElement->nextSQ = NULL;
    if(*queue == NULL){
        *queue = newElement;
//  sprintf(msgBuf, "%s", "adding to empty");
    }
    else
    {
//  sprintf(msgBuf, "%s", "adding to existing");

        SimpleQueue* currentElement = *queue;
        while(currentElement->nextSQ != NULL)
        {      
            currentElement = currentElement->nextSQ;
        }
        currentElement->nextSQ = newElement;
    }
//   HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
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
