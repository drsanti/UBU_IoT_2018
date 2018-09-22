
#ifndef __DATA_H__
#define __DATA_H__

#include "mcu.h"

/*
 * byte data structure
 */
typedef uint8_t byte_t;
typedef struct {
    byte_t *  buffer;   // pointer to data
    uint16_t  length;   // data length represents number of elements in the buffer
}data_t;

int16_t data_copy(data_t *dst, data_t *src);
int16_t data_copy_string(data_t *data, char *string);
int16_t data_copy_array(data_t *data, byte_t *array, uint16_t length);

#endif
