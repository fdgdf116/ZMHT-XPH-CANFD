#ifndef CANFD_RINGBUFFER_H
#define CANFD_RINGBUFFER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t *data;
    uint32_t in;
    uint32_t out;
    uint32_t mask;
    uint32_t esize;
} ring_buffer_t;

/** Allocate a fixed-element ring buffer. */
ring_buffer_t *ringbuffer_create(uint32_t size, uint32_t element_size);
/** Release a ring buffer. */
void ringbuffer_destroy(ring_buffer_t *ring);
/** Legacy misspelled destructor retained for compatibility. */
void ringbuffer_destory(ring_buffer_t *ring);
/** Remove all queued elements. */
void ringbuffer_reset(ring_buffer_t *ring);
/** Test whether the ring has no queued elements. */
bool ringbuffer_is_empty(const ring_buffer_t *ring);
/** Test whether the ring has no free slots. */
bool ringbuffer_is_full(const ring_buffer_t *ring);
/** Queue one element. */
uint32_t ringbuffer_put(ring_buffer_t *ring, const void *data, uint32_t length);
/** Dequeue one element. */
uint32_t ringbuffer_get(ring_buffer_t *ring, void *data, uint32_t length);
/** Return the number of queued elements. */
uint32_t ringbuffer_len(const ring_buffer_t *ring);

#endif
