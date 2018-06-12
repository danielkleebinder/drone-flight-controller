/*
 * serialio.c
 *
 *  Created on: 12.06.2018
 *      Author: Daniel Kleebinder
 */


#include "serialio.h"

#include <stdbool.h>
#include <driverlib/uart.h>



/**
 * Writes the given character to the UART serial port.
 *
 * @param ui32Base UART base port.
 * @param c Character.
 */
void uartWriteChar(uint32_t ui32Base, unsigned char c) {
    UARTCharPut(ui32Base, c);
}


/**
 * Writes the given data string to the UART serial port.
 *
 * @param ui32Base UART base port.
 * @param data Data to be written.
 */
void uartWrite(uint32_t ui32Base, unsigned char* data) {
    while (*data) {
        uartWriteChar(ui32Base, *data);
        data++;
    }
}


/**
 * Writes the given data string to the UART serial port and appends a new line at the end.
 *
 * @param ui32Base UART base port.
 * @param data Data to be written.
 */
void uartWriteLine(uint32_t ui32Base, unsigned char* data) {
    uartWrite(ui32Base, data);
    uartWrite(ui32Base, "\r\n");
}


/**
 * Reads a line from the given UART serial port.
 *
 * @param ui32Base UART base port.
 * @param data Result data buffer.
 */
void uartReadLine(uint32_t ui32Base, unsigned char* data) {
    int index = 0, input = 0;
    while (true) {
        input = UARTCharGet(ui32Base);
        UARTCharPut(ui32Base, input);
        if (input == '\0' || input == '\r' || input == '\n') {
            break;
        }
        data[index++] = input;
    }
    data[index] = '\0';
}


/**
 * Writes the given character to the given UART serial port.
 *
 * @param ui32Base UART base port.
 * @param c Character.
 */
void uartWriteCharNonBlocking(uint32_t ui32Base, unsigned char c) {
    UARTCharPutNonBlocking(ui32Base, c);
}


/**
 * Writes the given data non blocking to the given UART serial port.
 *
 * @param ui32Base UART base port.
 * @param data Data to be written.
 */
void uartWriteNonBlocking(uint32_t ui32Base, unsigned char* data) {
    while (*data) {
        uartWriteCharNonBlocking(ui32Base, *data);
        data++;
    }
}


/**
 * Writes a single byte to the given UART serial port.
 *
 * @param ui32Base UART base port.
 * @param data Byte data.
 */
void uartWriteByteNonBlocking(uint32_t ui32Base, uint8_t data) {
    uartWriteCharNonBlocking(ui32Base, (char) data);
}


/**
 * Writes the given bytes to the given UART serial port.
 *
 * @param ui32Base UART base port.
 * @param data Byte data.
 * @param size Number of bytes.
 */
void uartWriteBytesNonBlocking(uint32_t ui32Base, uint8_t* data, uint16_t size) {
    int i;
    for (i = 0; i < size; i++) {
        uartWriteByteNonBlocking(ui32Base, data[i]);
    }
}


/**
 * Reads a byte from the given UART serial port.
 *
 * @param ui32Base UART base port.
 * @param b Byte reference.
 */
bool uartReadByteNonBlocking(uint32_t ui32Base, uint8_t* b) {
    if (!UARTCharsAvail(ui32Base)) {
        return false;
    }
    *b = (uint8_t) UARTCharGetNonBlocking(ui32Base);
    return true;
}
