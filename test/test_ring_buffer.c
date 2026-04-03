/**
 * @file test_ring_buffer.c
 * @brief Unit tests for ring buffer
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

// Ring buffer implementation (inline for testing)
typedef struct {
    uint8_t *buffer;
    size_t size;
    size_t head;
    size_t tail;
    size_t count;
} RingBuffer;

void ring_buffer_init(RingBuffer *rb, uint8_t *buffer, size_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

bool ring_buffer_push(RingBuffer *rb, uint8_t data) {
    if (rb->count >= rb->size) return false;
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
    rb->count++;
    return true;
}

bool ring_buffer_pop(RingBuffer *rb, uint8_t *data) {
    if (rb->count == 0) return false;
    if (data) *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    rb->count--;
    return true;
}

bool ring_buffer_is_empty(const RingBuffer *rb) { return rb->count == 0; }
bool ring_buffer_is_full(const RingBuffer *rb) { return rb->count >= rb->size; }
size_t ring_buffer_available(const RingBuffer *rb) { return rb->count; }

void test_init(void) {
    printf("Testing ring_buffer_init...\n");
    
    uint8_t buf[16];
    RingBuffer rb;
    ring_buffer_init(&rb, buf, 16);
    
    TEST_ASSERT(rb.size == 16, "Size is 16");
    TEST_ASSERT(rb.head == 0, "Head starts at 0");
    TEST_ASSERT(rb.tail == 0, "Tail starts at 0");
    TEST_ASSERT(rb.count == 0, "Count starts at 0");
    TEST_ASSERT(ring_buffer_is_empty(&rb), "Buffer is empty");
    TEST_ASSERT(!ring_buffer_is_full(&rb), "Buffer is not full");
}

void test_push_pop(void) {
    printf("\nTesting push/pop...\n");
    
    uint8_t buf[4];
    RingBuffer rb;
    ring_buffer_init(&rb, buf, 4);
    
    // Push
    TEST_ASSERT(ring_buffer_push(&rb, 0xAA), "Push 0xAA");
    TEST_ASSERT(ring_buffer_available(&rb) == 1, "Count is 1");
    
    TEST_ASSERT(ring_buffer_push(&rb, 0xBB), "Push 0xBB");
    TEST_ASSERT(ring_buffer_available(&rb) == 2, "Count is 2");
    
    // Pop
    uint8_t data;
    TEST_ASSERT(ring_buffer_pop(&rb, &data), "Pop succeeds");
    TEST_ASSERT(data == 0xAA, "Got 0xAA");
    TEST_ASSERT(ring_buffer_available(&rb) == 1, "Count is 1");
    
    TEST_ASSERT(ring_buffer_pop(&rb, &data), "Pop succeeds");
    TEST_ASSERT(data == 0xBB, "Got 0xBB");
    TEST_ASSERT(ring_buffer_is_empty(&rb), "Buffer is empty");
}

void test_full_buffer(void) {
    printf("\nTesting full buffer...\n");
    
    uint8_t buf[4];
    RingBuffer rb;
    ring_buffer_init(&rb, buf, 4);
    
    // Fill buffer
    TEST_ASSERT(ring_buffer_push(&rb, 1), "Push 1");
    TEST_ASSERT(ring_buffer_push(&rb, 2), "Push 2");
    TEST_ASSERT(ring_buffer_push(&rb, 3), "Push 3");
    TEST_ASSERT(ring_buffer_push(&rb, 4), "Push 4");
    
    TEST_ASSERT(ring_buffer_is_full(&rb), "Buffer is full");
    TEST_ASSERT(!ring_buffer_push(&rb, 5), "Push to full fails");
}

void test_wrap_around(void) {
    printf("\nTesting wrap-around...\n");
    
    uint8_t buf[4];
    RingBuffer rb;
    ring_buffer_init(&rb, buf, 4);
    
    // Push and pop to advance pointers
    ring_buffer_push(&rb, 1);
    ring_buffer_push(&rb, 2);
    uint8_t data;
    ring_buffer_pop(&rb, &data);
    ring_buffer_pop(&rb, &data);
    
    // Now head=2, tail=2, count=0
    // Push 4 more to wrap
    TEST_ASSERT(ring_buffer_push(&rb, 10), "Push 10");
    TEST_ASSERT(ring_buffer_push(&rb, 20), "Push 20");
    TEST_ASSERT(ring_buffer_push(&rb, 30), "Push 30");
    TEST_ASSERT(ring_buffer_push(&rb, 40), "Push 40");
    TEST_ASSERT(ring_buffer_is_full(&rb), "Full after wrap");
    
    // Pop all
    ring_buffer_pop(&rb, &data);
    TEST_ASSERT(data == 10, "First is 10");
    ring_buffer_pop(&rb, &data);
    TEST_ASSERT(data == 20, "Second is 20");
    ring_buffer_pop(&rb, &data);
    TEST_ASSERT(data == 30, "Third is 30");
    ring_buffer_pop(&rb, &data);
    TEST_ASSERT(data == 40, "Fourth is 40");
    TEST_ASSERT(ring_buffer_is_empty(&rb), "Empty after pop all");
}

void test_empty_pop(void) {
    printf("\nTesting empty pop...\n");
    
    uint8_t buf[4];
    RingBuffer rb;
    ring_buffer_init(&rb, buf, 4);
    
    uint8_t data = 0xFF;
    TEST_ASSERT(!ring_buffer_pop(&rb, &data), "Pop from empty fails");
    TEST_ASSERT(data == 0xFF, "Data unchanged on fail");
}

int main(void) {
    printf("=== Ring Buffer Tests ===\n\n");
    
    test_init();
    test_push_pop();
    test_full_buffer();
    test_wrap_around();
    test_empty_pop();
    
    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    
    return (tests_passed == tests_run) ? 0 : 1;
}
