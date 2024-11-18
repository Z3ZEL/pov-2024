#include "ring_buffer.h"



void ring_buffer_init(ring_buffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->full = 0;
}

void ring_buffer_put(ring_buffer_t *rb, uint8_t data) {
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;

    // Si on avance et qu'on écrase la queue, on déplace la queue pour éviter la perte de données
    if (rb->full) {
        rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    }

    // Mettre le drapeau à plein si `head` rattrape `tail`
    rb->full = (rb->head == rb->tail);
}

uint8_t ring_buffer_get(ring_buffer_t *rb) {
    if (rb->head == rb->tail && !rb->full) {
        // Buffer vide, rien à lire
        return 0;
    }

    uint8_t data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;

    // Dès qu'on retire un élément, le buffer n'est plus plein
    rb->full = 0;

    return data;
}

uint8_t ring_buffer_available_bytes(ring_buffer_t *rb) {
    if (rb->full) {
        return RING_BUFFER_SIZE;
    } else if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    } else {
        return RING_BUFFER_SIZE - (rb->tail - rb->head);
    }
}

uint8_t ring_buffer_is_full(ring_buffer_t *rb) {
    return rb->full;
}

uint8_t ring_buffer_is_empty(ring_buffer_t *rb) {
    return (rb->head == rb->tail) && !rb->full;
}
