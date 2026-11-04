#include "ring_buffer.h"

void ringbuf_init(ringbuf_t *rb, uint8_t *buf, uint32_t size)
{
    rb->buf = buf;
    rb->size = size;
    rb->mask = size - 1;
    rb->head = 0;
    rb->tail = 0;
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

bool ringbuf_pop(ringbuf_t *rb, uint8_t *byte)
{
    if (ringbuf_is_empty(rb))
        return false;
    *byte = rb->buf[rb->tail & rb->mask];
    rb->tail++;
    return true;
}