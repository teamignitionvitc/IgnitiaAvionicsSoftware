/**
 * @file ring_buffer.h
 * @brief Circular buffer implementation
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint8_t *buffer;
    size_t size;
    size_t head;
    size_t tail;
    size_t count;
} RingBuffer;

void ring_buffer_init(RingBuffer *rb, uint8_t *buffer, size_t size);
bool ring_buffer_push(RingBuffer *rb, uint8_t data);
bool ring_buffer_pop(RingBuffer *rb, uint8_t *data);
bool ring_buffer_peek(const RingBuffer *rb, uint8_t *data);
size_t ring_buffer_available(const RingBuffer *rb);
size_t ring_buffer_free(const RingBuffer *rb);
bool ring_buffer_is_empty(const RingBuffer *rb);
bool ring_buffer_is_full(const RingBuffer *rb);
void ring_buffer_clear(RingBuffer *rb);
size_t ring_buffer_write(RingBuffer *rb, const uint8_t *data, size_t len);
size_t ring_buffer_read(RingBuffer *rb, uint8_t *data, size_t len);

#endif // RING_BUFFER_H
