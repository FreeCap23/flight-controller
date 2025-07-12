/**
  ******************************************************************************
  * @file           : circular_buffer.h
  * @author         : Dionisie Stratulat
  * @brief          : This file provides the structure and methods to use a Circular Buffer.
  *                   It is useful for making a moving average.
  ******************************************************************************
  */

#ifndef INC_CIRCULAR_BUFFER_H_
#define INC_CIRCULAR_BUFFER_H_

#include <stdint.h>

/**
 * @brief Circular buffer structure for moving average calculation.
 */
typedef struct {
    float *buffer;      /**< Dynamic array to hold the readings */
    int size;           /**< Size of the circular buffer */
    int index;          /**< Current index for inserting new value */
    int count;          /**< Number of values added (up to size) */
    float sum;          /**< Running sum of the values */
} CircularBuffer;

/**
 * @brief Create a new circular buffer with given size.
 *
 * @param size Number of samples for the moving average window.
 * @return Pointer to allocated CircularBuffer, or NULL on failure.
 */
CircularBuffer* circular_buffer_new(int size);

/**
 * @brief Free memory allocated for the circular buffer.
 *
 * @param cb Pointer to the CircularBuffer to free.
 */
void circular_buffer_free(CircularBuffer *cb);

/**
 * @brief Update the circular buffer with a new value.
 *
 * @param cb Pointer to the CircularBuffer.
 * @param new_value New uint16_t value to insert.
 */
void circular_buffer_update(CircularBuffer *cb, float new_value);

/**
 * @brief Get the current moving average from the circular buffer.
 *
 * @param cb Pointer to the CircularBuffer.
 * @return Current average as a float.
 */
float circular_buffer_get_average(const CircularBuffer *cb);

#endif /* INC_CIRCULAR_BUFFER_H_ */
