#include "utils.h"

void queue_put(volatile queue_t* q, uint32_t x) {
  
   for (size_t i = QUEUE_SIZE-1; i >= 1 ; i--) {
    q->queue[i] = q->queue[i-1];
  }
  q->queue[0] = x;
}

