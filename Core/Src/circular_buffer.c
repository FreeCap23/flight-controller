/**
  ******************************************************************************
  * @file           : circular_buffer.c
  * @author         : Dionisie Stratulat
  * @brief          : TODO
  ******************************************************************************
  */

#include "circular_buffer.h"
#include <stdlib.h>

CircularBuffer* circular_buffer_new(int size) {
    if (size <= 0) return NULL;

    CircularBuffer *cb = (CircularBuffer*)malloc(sizeof(CircularBuffer));
    if (!cb) return NULL;

    cb->buffer = (float*)calloc(size, sizeof(float));
    if (!cb->buffer) {
        free(cb);
        return NULL;
    }

    cb->size = size;
    cb->index = 0;
    cb->count = 0;
    cb->sum = 0;

    return cb;
}

void circular_buffer_free(CircularBuffer *cb) {
    if (!cb) return;
    free(cb->buffer);
    free(cb);
}

void circular_buffer_update(CircularBuffer *cb, float new_value) {
    if (!cb || !cb->buffer) return;

    // Subtract the oldest value from sum
    cb->sum -= cb->buffer[cb->index];
    // Insert new value
    cb->buffer[cb->index] = new_value;
    // Add new value to sum
    cb->sum += new_value;

    // Update index circularly
    cb->index = (cb->index + 1) % cb->size;

    // Update count until full
    if (cb->count < cb->size) cb->count++;
}

float circular_buffer_get_average(const CircularBuffer *cb) {
    if (!cb || cb->count == 0) return 0.0f;

    return cb->sum / cb->count;
}
