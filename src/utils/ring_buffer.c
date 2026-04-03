/**
 * @file ring_buffer.c
 * @brief Circular buffer implementation
 */

#include "ring_buffer.h"
#include <string.h>

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

bool ring_buffer_peek(const RingBuffer *rb, uint8_t *data) {
    if (rb->count == 0) return false;
    if (data) *data = rb->buffer[rb->tail];
    return true;
}

size_t ring_buffer_available(const RingBuffer *rb) {
    return rb->count;
}

size_t ring_buffer_free(const RingBuffer *rb) {
    return rb->size - rb->count;
}

bool ring_buffer_is_empty(const RingBuffer *rb) {
    return rb->count == 0;
}

bool ring_buffer_is_full(const RingBuffer *rb) {
    return rb->count >= rb->size;
}

void ring_buffer_clear(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

size_t ring_buffer_write(RingBuffer *rb, const uint8_t *data, size_t len) {
    size_t written = 0;
    while (written < len && !ring_buffer_is_full(rb)) {
        ring_buffer_push(rb, data[written++]);
    }
    return written;
}

size_t ring_buffer_read(RingBuffer *rb, uint8_t *data, size_t len) {
    size_t read_count = 0;
    while (read_count < len && !ring_buffer_is_empty(rb)) {
        ring_buffer_pop(rb, &data[read_count++]);
    }
    return read_count;
}
