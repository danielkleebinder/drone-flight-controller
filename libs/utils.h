/*
 * utils.h
 *
 *  Created on: 12.06.2018
 *      Author: Daniel Kleebinder
 */

#ifndef LIBS_UTILS_H_
#define LIBS_UTILS_H_

#include <stdint.h>


void uppercase(unsigned char* data);
void lowercase(unsigned char* data);
uint8_t checksum(uint8_t* bytes, uint8_t numBytes);


#endif /* LIBS_UTILS_H_ */
