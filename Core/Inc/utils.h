#ifndef __UTILS__H
#define __UTILS__H

#include <stdint.h>
#include <stddef.h>

#define QUEUE_SIZE 8
typedef struct {
  uint32_t queue[QUEUE_SIZE];

} queue_t;


void queue_put(queue_t* q, uint32_t x);

#endif
