#include "logging_subsystem.h"

class RingBuffer
{

public:
    bool ringbuf_init(ringbuf_t *rb, uint8_t *buf, uint32_t size)
    {
        rb->buf = buf;
        rb->size = size;
        rb->mask = size - 1;
        rb->head = 0;
        rb->tail = 0;
        return true;
    }

    int ring_buf_remaining(ringbuf_t *rb)
    {
        return ((rb->head - rb->tail) - rb->size);
    }
    bool ringbuf_is_full(ringbuf_t *rb)
    {
        return (rb->head - rb->tail) == rb->size;
    }

    bool ringbuf_is_empty(ringbuf_t *rb)
    {
        return rb->head == rb->tail;
    }

    bool ringbuf_push(ringbuf_t *rb, uint8_t byte)
    {
        if (ringbuf_is_full(rb))
            return false;
        rb->buf[rb->head & rb->mask] = byte;
        rb->head++;
        return true;
    }

    uint8_t ringbuf_pop(ringbuf_t *rb)
    {
        uint8_t byte = rb->buf[rb->tail & rb->mask];
        rb->tail++;
        return byte;
    }
};