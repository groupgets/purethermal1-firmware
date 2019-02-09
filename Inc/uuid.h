/*
 * uuid.h
 *
 *  Created on: Feb 9, 2019
 *      Author: kurt
 */

#ifndef UUID_H_
#define UUID_H_

/**
 * The STM32 device electronic signature.
 * 96 total bits starting at 0x1FFF7A10
 */

#define STM32_ID ((unsigned short *)0x1FFF7A10)

#endif /* UUID_H_ */
