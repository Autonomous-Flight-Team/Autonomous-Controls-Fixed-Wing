typedef struct {
    uint8_t *buf;
    volatile uint32_t head;  // ← written ONLY by producer
    volatile uint32_t tail;  // ← written ONLY by consumer
    uint32_t size;           // must be power of two
    uint32_t mask;           // size - 1
} ringbuf_t;