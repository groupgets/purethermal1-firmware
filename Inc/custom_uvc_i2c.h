/*
 * custom_uvc_i2c.h
 *
 *  Created on: Mar 13, 2018
 *      Author: sanderson
 */

#ifndef CUSTOM_UVC_I2C_H_
#define CUSTOM_UVC_I2C_H_

extern union custom_uvc {
	struct {
	  LEP_COMMAND_ID id; // note 16 bit
	  uint16_t length;
	  uint8_t buffer[512];
	} command;
	struct {
      LEP_UINT16 address;
      LEP_UINT16 length;
	  uint8_t data[512];
	} direct;
} custom_uvc;

struct custom_response {
	LEP_RESULT result;
	uint8_t data[512];
};

#endif /* CUSTOM_UVC_I2C_H_ */
