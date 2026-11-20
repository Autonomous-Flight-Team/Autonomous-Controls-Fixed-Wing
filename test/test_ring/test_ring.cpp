#include <unity.h>
#include <string.h>
#include "ring_buffer.h"

static RingBuffer rbm;
static ringbuf_t rb;
static uint8_t mem[16]; // 4 guard bytes | 8 data bytes | 4 guard bytes
static uint8_t *const data = mem + 4;
static const size_t N = 8;

void setUp(void) {
    memset(mem, 0xAA, sizeof(mem));
    memset(&rb, 0, sizeof(rb));
    rbm.ringbuf_init(&rb, data, N);
}
void tearDown(void) {}

static void assert_guards_intact(void) {
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL_HEX8(0xAA, mem[i]);
        TEST_ASSERT_EQUAL_HEX8(0xAA, mem[12 + i]);
    }
}

void test_init_sets_fields(void) {
    TEST_ASSERT_EQUAL_UINT32(N, rb.size);
    TEST_ASSERT_EQUAL_UINT32(N - 1, rb.mask);
    TEST_ASSERT_EQUAL_UINT32(0, rb.head);
    TEST_ASSERT_EQUAL_UINT32(0, rb.tail);
    TEST_ASSERT_TRUE(rb.buf == data);
}

void test_init_rejects_bad_arguments(void) {
    ringbuf_t t;
    TEST_ASSERT_FALSE(rbm.ringbuf_init(nullptr, data, 8));
    TEST_ASSERT_FALSE(rbm.ringbuf_init(&t, nullptr, 8));
    TEST_ASSERT_FALSE(rbm.ringbuf_init(&t, data, 0));
    TEST_ASSERT_FALSE(rbm.ringbuf_init(&t, data, 3));
    TEST_ASSERT_FALSE(rbm.ringbuf_init(&t, data, 100));
}

void test_init_accepts_power_of_two_sizes(void) {
    ringbuf_t t;
    TEST_ASSERT_TRUE(rbm.ringbuf_init(&t, data, 1));
    TEST_ASSERT_TRUE(rbm.ringbuf_init(&t, data, 2));
    TEST_ASSERT_TRUE(rbm.ringbuf_init(&t, data, 4096));
}

void test_new_buffer_is_empty(void) {
    TEST_ASSERT_TRUE(rbm.ringbuf_is_empty(&rb));
    TEST_ASSERT_FALSE(rbm.ringbuf_is_full(&rb));
    TEST_ASSERT_EQUAL_UINT32(0, rbm.ringbuf_count(&rb));
    TEST_ASSERT_EQUAL_UINT32(N, rbm.ring_buf_remaining(&rb));
}

void test_push_updates_count_and_remaining(void) {
    TEST_ASSERT_TRUE(rbm.ringbuf_push(&rb, 0x11));
    TEST_ASSERT_TRUE(rbm.ringbuf_push(&rb, 0x22));
    TEST_ASSERT_EQUAL_UINT32(2, rbm.ringbuf_count(&rb));
    TEST_ASSERT_EQUAL_UINT32(N - 2, rbm.ring_buf_remaining(&rb));
    TEST_ASSERT_FALSE(rbm.ringbuf_is_empty(&rb));
}

void test_fifo_order(void) {
    for (uint8_t i = 0; i < 5; i++)
        rbm.ringbuf_push(&rb, 10 + i);
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t b = 0;
        TEST_ASSERT_TRUE(rbm.ringbuf_pop(&rb, &b));
        TEST_ASSERT_EQUAL_UINT8(10 + i, b);
    }
    TEST_ASSERT_TRUE(rbm.ringbuf_is_empty(&rb));
}

void test_full_buffer_rejects_push_and_keeps_data(void) {
    for (uint8_t i = 0; i < N; i++)
        TEST_ASSERT_TRUE(rbm.ringbuf_push(&rb, i));
    TEST_ASSERT_TRUE(rbm.ringbuf_is_full(&rb));
    TEST_ASSERT_EQUAL_UINT32(0, rbm.ring_buf_remaining(&rb));
    TEST_ASSERT_EQUAL_UINT32(N, rbm.ringbuf_count(&rb));
    TEST_ASSERT_FALSE(rbm.ringbuf_push(&rb, 0x99)); // must not overwrite oldest byte

    uint8_t b = 0xEE;
    TEST_ASSERT_TRUE(rbm.ringbuf_pop(&rb, &b));
    TEST_ASSERT_EQUAL_UINT8(0, b);
    assert_guards_intact();
}

void test_pop_on_empty_fails_and_leaves_output_alone(void) {
    uint8_t b = 0x5A;
    TEST_ASSERT_FALSE(rbm.ringbuf_pop(&rb, &b));
    TEST_ASSERT_EQUAL_UINT8(0x5A, b);
}

void test_wraparound_preserves_order_and_guards(void) {
    uint8_t next_in = 0, next_out = 0;
    for (int round = 0; round < 50; round++) {
        for (int k = 0; k < 6; k++)
            TEST_ASSERT_TRUE(rbm.ringbuf_push(&rb, next_in++));
        for (int k = 0; k < 6; k++) {
            uint8_t b;
            TEST_ASSERT_TRUE(rbm.ringbuf_pop(&rb, &b));
            TEST_ASSERT_EQUAL_UINT8(next_out++, b);
        }
    }
    TEST_ASSERT_TRUE(rbm.ringbuf_is_empty(&rb));
    assert_guards_intact();
}

void test_index_overflow_past_uint32_max(void) {
    rb.head = rb.tail = 0xFFFFFFFEu; // about to wrap
    for (uint8_t i = 0; i < 4; i++)
        TEST_ASSERT_TRUE(rbm.ringbuf_push(&rb, 0x40 + i));
    TEST_ASSERT_EQUAL_UINT32(4, rbm.ringbuf_count(&rb));
    TEST_ASSERT_EQUAL_UINT32(N - 4, rbm.ring_buf_remaining(&rb));
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t b;
        TEST_ASSERT_TRUE(rbm.ringbuf_pop(&rb, &b));
        TEST_ASSERT_EQUAL_UINT8(0x40 + i, b);
    }
    TEST_ASSERT_TRUE(rbm.ringbuf_is_empty(&rb));
}

void test_size_one_buffer(void) {
    ringbuf_t t;
    TEST_ASSERT_TRUE(rbm.ringbuf_init(&t, data, 1));
    TEST_ASSERT_TRUE(rbm.ringbuf_push(&t, 7));
    TEST_ASSERT_TRUE(rbm.ringbuf_is_full(&t));
    TEST_ASSERT_FALSE(rbm.ringbuf_push(&t, 8));
    uint8_t b;
    TEST_ASSERT_TRUE(rbm.ringbuf_pop(&t, &b));
    TEST_ASSERT_EQUAL_UINT8(7, b);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_init_sets_fields);
    RUN_TEST(test_init_rejects_bad_arguments);
    RUN_TEST(test_init_accepts_power_of_two_sizes);
    RUN_TEST(test_new_buffer_is_empty);
    RUN_TEST(test_push_updates_count_and_remaining);
    RUN_TEST(test_fifo_order);
    RUN_TEST(test_full_buffer_rejects_push_and_keeps_data);
    RUN_TEST(test_pop_on_empty_fails_and_leaves_output_alone);
    RUN_TEST(test_wraparound_preserves_order_and_guards);
    RUN_TEST(test_index_overflow_past_uint32_max);
    RUN_TEST(test_size_one_buffer);
    return UNITY_END();
}
