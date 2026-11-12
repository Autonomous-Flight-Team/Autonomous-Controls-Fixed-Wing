#pragma once
#include <stdint.h>

typedef struct
{
    uint8_t *buf;
    volatile uint32_t head; // written ONLY by producer
    volatile uint32_t tail; // written ONLY by consumer
    uint32_t size;          // must be power of two
    uint32_t mask;          // size - 1
} ringbuf_t;

class RingBuffer
{
public:
    bool ringbuf_init(ringbuf_t *rb, uint8_t *buf, uint32_t size)
    {
        if (!rb || !buf || size == 0 || (size & (size - 1)) != 0)
            return false; // size must be a power of two
        rb->buf = buf;
        rb->size = size;
        rb->mask = size - 1;
        rb->head = 0;
        rb->tail = 0;
        return true;
    }

    uint32_t ringbuf_count(ringbuf_t *rb) { return rb->head - rb->tail; }

    uint32_t ring_buf_remaining(ringbuf_t *rb) { return rb->size - (rb->head - rb->tail); }

    bool ringbuf_is_full(ringbuf_t *rb) { return (rb->head - rb->tail) == rb->size; }
    bool ringbuf_is_empty(ringbuf_t *rb) { return rb->head == rb->tail; }

    bool ringbuf_push(ringbuf_t *rb, uint8_t byte)
    {
        if (ringbuf_is_full(rb))
            return false;
        rb->buf[rb->head & rb->mask] = byte;
        rb->head++;
        return true;
    }

    bool ringbuf_pop(ringbuf_t *rb, uint8_t *byte)
    {
        if (ringbuf_is_empty(rb))
            return false;
        *byte = rb->buf[rb->tail & rb->mask];
        rb->tail++;
        return true;
    }
};