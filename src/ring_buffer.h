#include <stdint.h>
#define RING_BUFFER_SIZE 64
/**
 * @file ring_buffer.h
 * @brief Header file for ring buffer implementation.
 *
 * This file contains the definitions and function prototypes for a ring buffer.
 * A ring buffer is a fixed-size buffer that works as a circular queue.
 */

 /**
 * @struct ring_buffer
 * @brief A structure representing a ring buffer.
 *
 * This structure defines a ring buffer with a fixed size buffer,
 * and pointers to the head and tail of the buffer.
 *
 * @var ring_buffer::buffer
 * A fixed-size array that holds the data in the ring buffer.
 *
 * @var ring_buffer::head
 * A pointer to the head of the buffer, where data is added.
 *
 * @var ring_buffer::tail
 * A pointer to the tail of the buffer, where data is removed.
 */
typedef struct ring_buffer {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint8_t head;  // Position pour l'ajout
    uint8_t tail;  // Position pour la lecture
    uint8_t full;  // Drapeau pour indiquer si le buffer est plein
} ring_buffer_t;
/**
 * @brief Initialize the ring buffer.
 *
 * This function initializes the ring buffer by setting the head and tail pointers
 * to the start of the buffer.
 *
 * @param rb Pointer to the ring buffer to initialize.
 */
void ring_buffer_init(struct ring_buffer *rb);

/**
 * @brief Add a byte to the ring buffer.
 *
 * This function adds a byte of data to the ring buffer at the position pointed to by the head pointer.
 * If the buffer is full, the head pointer wraps around to the beginning of the buffer.
 *
 * @param rb Pointer to the ring buffer.
 * @param data The byte of data to add to the buffer.
 */
void ring_buffer_put(struct ring_buffer *rb, uint8_t data);

/**
 * @brief Retrieve a byte from the ring buffer.
 *
 * This function retrieves a byte of data from the ring buffer at the position pointed to by the tail pointer.
 * If the buffer is empty, the tail pointer wraps around to the beginning of the buffer.
 *
 * @param rb Pointer to the ring buffer.
 * @return The byte of data retrieved from the buffer.
 */
uint8_t ring_buffer_get(struct ring_buffer *rb);

/**
 * @brief Get the number of available bytes in the ring buffer.
 *
 * This function returns the number of bytes currently stored in the ring buffer.
 *
 * @param rb Pointer to the ring buffer.
 * @return The number of bytes available in the buffer.
 */
uint8_t ring_buffer_available_bytes(struct ring_buffer *rb);

/**
 * @brief Check if the ring buffer is full.
 *
 * This function checks if the ring buffer is full.
 *
 * @param rb Pointer to the ring buffer.
 * @return 1 if the buffer is full, 0 otherwise.
 */
uint8_t ring_buffer_is_full(struct ring_buffer *rb);