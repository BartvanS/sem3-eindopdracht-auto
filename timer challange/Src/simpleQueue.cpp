#include "simpleQueue.h"

void addToQueue(SimpleQueue** queue, char* value){

    SimpleQueue* local = (*queue);
    // if (local == NULL)
    // {
    //     return;
    // }
    
    if (local->value == NULL)
    {
           local->value = value;
        local->nextSQ = NULL;
    }
    else{
        while (local->nextSQ != NULL)
        {
            local = local->nextSQ;
        }
        local->nextSQ = (SimpleQueue *)malloc(sizeof(SimpleQueue));
        local->nextSQ->value = value;
        local->nextSQ->nextSQ = NULL;
    }

}

char* retrieveFromQueue(SimpleQueue** queue){
     SimpleQueue* local = (*queue);
    if(local == NULL){
        return (char*)"faal";
    }
    SimpleQueue* next_item = local->nextSQ;
    char* tmpVal = local->value;
    free(local);
    local = NULL;
    local = next_item;
    return (char*)"faal";
}
