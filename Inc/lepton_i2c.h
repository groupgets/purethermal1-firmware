#ifndef LEPTON_I2C_H_
#define LEPTON_I2C_H_

#include "LEPTON_ErrorCodes.h"
#include "LEPTON_VID.h"

HAL_StatusTypeDef agc_enable();
HAL_StatusTypeDef set_reg(unsigned int reg);
uint16_t read_reg(unsigned int reg);
HAL_StatusTypeDef read_data();
HAL_StatusTypeDef lepton_read_data(uint8_t * data);

HAL_StatusTypeDef init_lepton_command_interface(void);
HAL_StatusTypeDef disable_lepton_agc();
HAL_StatusTypeDef enable_lepton_agc();
HAL_StatusTypeDef disable_rgb888();
HAL_StatusTypeDef disable_telemetry(void);
HAL_StatusTypeDef enable_telemetry(void);
HAL_StatusTypeDef get_scene_stats(uint16_t *min, uint16_t *max, uint16_t *avg);
HAL_StatusTypeDef enable_rgb888(LEP_PCOLOR_LUT_E pcolor_lut);
HAL_StatusTypeDef enable_raw14();

HAL_StatusTypeDef lepton_low_power();
HAL_StatusTypeDef lepton_power_on();

#endif

