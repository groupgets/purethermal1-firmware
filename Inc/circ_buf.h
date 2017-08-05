/*
 * circ_buf.h
 *
 *  Created on: Dec 14, 2016
 *      Author: kurt
 */

#ifndef CIRC_BUF_H_
#define CIRC_BUF_H_

#include <stdint.h>

#define CIRC_BUF_HANDLE(ARRAY_PTR) (&ARRAY_PTR##_h)
#define DECLARE_CIRC_BUF_HANDLE(ARRAY_PTR) \
    circ_buf_state_t ARRAY_PTR##_h = { 0, 0, sizeof(ARRAY_PTR) / sizeof(void*), (void**)ARRAY_PTR }

typedef struct {
    uint32_t read;
    uint32_t write;
    uint32_t capacity;
    void **array;
} circ_buf_state_t;

static inline uint32_t mask(circ_buf_state_t* pbuf, uint32_t val)  { return val & (pbuf->capacity - 1); } // { return val % pbuf->capacity; }
static inline void push(circ_buf_state_t* pbuf, void* val)   { /* assert(!full()); */ pbuf->array[mask(pbuf, pbuf->write++)] = val; }
static inline void* shift(circ_buf_state_t* pbuf)            { /* assert(!empty()); */ return pbuf->array[mask(pbuf, pbuf->read++)]; }
static inline bool empty(circ_buf_state_t* pbuf)                { return pbuf->read == pbuf->write; }
static inline uint32_t size(circ_buf_state_t* pbuf)             { return pbuf->write - pbuf->read; }
static inline bool full(circ_buf_state_t* pbuf)                 { return size(pbuf) == pbuf->capacity; }

#endif /* CIRC_BUF_H_ */
