/*
 * utils.c
 *
 *  Created on: 12.06.2018
 *      Author: Daniel Kleebinder
 */


#include "utils.h"

#include <ctype.h>
#include <stdint.h>
#include <string.h>



/**
 * Converts the given string to upper case.
 *
 * @param data String.
 */
void uppercase(unsigned char* data) {
    while(*data != '\0') {
        *data = toupper(*data);
        data++;
    }
}


/**
 * Converts the given string to lower case.
 *
 * @param data String.
 */
void lowercase(unsigned char* data) {
    while(*data != '\0') {
        *data = tolower(*data);
        data++;
    }
}


/**
 * Computes the MultiWii checksum of the given bytes.
 *
 * @param bytes Bytes to compute the checksum.
 * @param numBytes Number of bytes.
 * @return Computed checksum.
 */
uint8_t checksum(uint8_t* bytes, uint8_t size) {
    uint8_t result = 0, i;
    for (i = 0; i < size; i++) {
        result ^= bytes[i];
    }
    return result;
}
