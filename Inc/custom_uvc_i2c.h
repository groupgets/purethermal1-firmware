/*
 * custom_uvc_i2c.h
 *
 *  Created on: Mar 13, 2018
 *      Author: sanderson
 */

#ifndef CUSTOM_UVC_I2C_H_
#define CUSTOM_UVC_I2C_H_

extern struct custom_command {
  LEP_COMMAND_ID command_id; // note 16 bit
  uint16_t length;
  uint8_t buffer[512];
} custom_command;

struct custom_response {
	LEP_RESULT result;
	uint8_t data[512];
};

#endif /* CUSTOM_UVC_I2C_H_ */
