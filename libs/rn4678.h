/*
 * rn4678.h
 *
 *  Created on: 15.06.2018
 *      Author: Daniel Kleebinder
 */

#ifndef LIBS_RN4678_H_
#define LIBS_RN4678_H_

#include <stdbool.h>


void rn4678Reset(bool v);
void rn4678SoftwareButton(bool v);
void rn4678RTS(bool v);


#endif /* LIBS_RN4678_H_ */
