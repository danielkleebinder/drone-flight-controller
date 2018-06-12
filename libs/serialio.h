/*
 * serialio.h
 *
 *  Created on: 12.06.2018
 *      Author: Daniel Kleebinder
 */

#ifndef LIBS_SERIALIO_H_
#define LIBS_SERIALIO_H_

#include <stdint.h>
#include <stdbool.h>


void uartWrite(uint32_t ui32Base, unsigned char* data);
void uartWriteChar(uint32_t ui32Base, unsigned char c);
void uartWriteLine(uint32_t ui32Base, unsigned char* data);
void uartReadLine(uint32_t ui32Base, unsigned char* data);

void uartWriteCharNonBlocking(uint32_t ui32Base, unsigned char c);
void uartWriteNonBlocking(uint32_t ui32Base, unsigned char* data);
void uartWriteByteNonBlocking(uint32_t ui32Base, uint8_t b);
void uartWriteBytesNonBlocking(uint32_t ui32Base, uint8_t* b, uint16_t size);
bool uartReadByteNonBlocking(uint32_t ui32Base, uint8_t* b);


#endif /* LIBS_SERIALIO_H_ */
