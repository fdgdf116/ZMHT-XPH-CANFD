#include "ringbuffer.h"

#include <stdlib.h>
#include <string.h>

/** Round a count down to the largest power of two. */
static uint32_t ringbuffer_power2(uint32_t count)
{
    uint32_t value = 1U;

    while ((value << 1U) != 0U && (value << 1U) <= count) {
        value <<= 1U;
    }
    return value;
}

/** Allocate a ring buffer whose element count is rounded to a power of two. */
ring_buffer_t *ringbuffer_create(uint32_t size, uint32_t element_size)
{
    ring_buffer_t *ring;
    uint32_t element_count;

    if (size == 0U || element_size == 0U) {
        return NULL;
    }
    element_count = ringbuffer_power2(size / element_size);
    if (element_count == 0U) {
        return NULL;
    }

    ring = (ring_buffer_t *)calloc(1U, sizeof(*ring));
    if (ring == NULL) {
        return NULL;
    }
    ring->data = (uint8_t *)calloc(element_count, element_size);
    if (ring->data == NULL) {
        free(ring);
        return NULL;
    }
    ring->mask = element_count - 1U;
    ring->esize = element_size;
    return ring;
}

/** Release the ring object and its backing storage. */
void ringbuffer_destroy(ring_buffer_t *ring)
{
    if (ring != NULL) {
        free(ring->data);
        free(ring);
    }
}

/** Preserve the legacy misspelled destructor used by older applications. */
void ringbuffer_destory(ring_buffer_t *ring)
{
    ringbuffer_destroy(ring);
}

/** Discard all queued elements without releasing the ring storage. */
void ringbuffer_reset(ring_buffer_t *ring)
{
    if (ring != NULL) {
        ring->in = 0U;
        ring->out = 0U;
    }
}

/** Return the number of elements currently queued. */
uint32_t ringbuffer_len(const ring_buffer_t *ring)
{
    return ring != NULL ? ring->in - ring->out : 0U;
}

/** Test whether no elements are queued. */
bool ringbuffer_is_empty(const ring_buffer_t *ring)
{
    return ring == NULL || ring->in == ring->out;
}

/** Test whether all ring slots are occupied. */
bool ringbuffer_is_full(const ring_buffer_t *ring)
{
    return ring != NULL && ringbuffer_len(ring) > ring->mask;
}

/** Queue one fixed-size element; return one on success, zero when full. */
uint32_t ringbuffer_put(ring_buffer_t *ring, const void *data, uint32_t length)
{
    uint32_t slot;

    if (ring == NULL || data == NULL || length < ring->esize ||
        ringbuffer_is_full(ring)) {
        return 0U;
    }
    slot = ring->in & ring->mask;
    memcpy(ring->data + slot * ring->esize, data, ring->esize);
    ring->in++;
    return 1U;
}

/** Dequeue one fixed-size element; return one on success, zero when empty. */
uint32_t ringbuffer_get(ring_buffer_t *ring, void *data, uint32_t length)
{
    uint32_t slot;

    if (ring == NULL || data == NULL || length < ring->esize ||
        ringbuffer_is_empty(ring)) {
        return 0U;
    }
    slot = ring->out & ring->mask;
    memcpy(data, ring->data + slot * ring->esize, ring->esize);
    ring->out++;
    return 1U;
}
