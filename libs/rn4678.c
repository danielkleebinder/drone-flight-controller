/*
 * rn4678.c
 *
 *  Created on: 15.06.2018
 *      Author: Daniel Kleebinder
 */


#include "rn4678.h"
#include <stdint.h>
#include <stdbool.h>
#include <driverlib/gpio.h>
#include <inc/hw_memmap.h>



/**
 * Toggles the RN4678 reset pin.
 *
 * @param v True if active, otherwise false.
 */
void rn4678Reset(bool v) {
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_4, v ? GPIO_PIN_4 : ~GPIO_PIN_4);
}


/**
 * Toggles the RN4678 software button pin.
 *
 * @param v True if active, otherwise false.
 */
void rn4678SoftwareButton(bool v) {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, v ? GPIO_PIN_2 : ~GPIO_PIN_2);
}


/**
 * Toggles the RN4678 RTS pin.
 *
 * @param v True if active, otherwise false.
 */
void rn4678RTS(bool v) {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, v ? GPIO_PIN_4 : ~GPIO_PIN_4);
}
