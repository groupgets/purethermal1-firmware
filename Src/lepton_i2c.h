#ifndef LEPTON_I2C_H_
#define LEPTON_I2C_H_

HAL_StatusTypeDef init_lepton_command_interface(void);
HAL_StatusTypeDef enable_lepton_agc();
HAL_StatusTypeDef enable_telemetry(void);
HAL_StatusTypeDef get_scene_stats(uint16_t *min, uint16_t *max, uint16_t *avg);

#endif

