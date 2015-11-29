#ifndef LEPTON_I2C_H_
#define LEPTON_I2C_H_

#include "LEPTON_ErrorCodes.h"
#include "LEPTON_VID.h"

#define LEPTON_ADDRESS  (0x2A<<1)

#define POWER_REG (0x00)
#define STATUS_REG (0x02)
#define COMMAND_REG (0x04)
#define DATA_LENGTH_REG (0x06)
#define DATA_0_REG (0x08)
#define DATA_CRC_REG (0x28)

#define AGC (0x01)
#define SYS (0x02)
#define VID (0x03)
#define OEM (0x48)

#define GET (0x00)
#define SET (0x01)
#define RUN (0x02)

#define PING                        (0x00 )
#define CAM_STATUS                  (0x04 )
#define FLIR_SERIAL_NUMBER          (0x08 )
#define CAM_UPTIME                  (0x0C )
#define AUX_TEMPERATURE_KELVIN      (0x10 )
#define FPA_TEMPERATURE_KELVIN      (0x14 )
#define TELEMETRY_ENABLE_STATE      (0x18 )
#define TELEMETRY_LOCATION          (0x1C )
#define EXECTUE_FRAME_AVERAGE       (0x20 )
#define NUM_FRAMES_TO_AVERAGE       (0x24 )
#define CUST_SERIAL_NUMBER          (0x28 )
#define SCENE_STATISTICS            (0x2C )
#define SCENE_ROI                   (0x30 )
#define THERMAL_SHUTDOWN_COUNT      (0x34 )
#define SHUTTER_POSITION            (0x38 )
#define FFC_SHUTTER_MODE_OBJ        (0x3C )
#define RUN_FFC                     (0x42 )
#define FFC_STATUS                  (0x44 )

HAL_StatusTypeDef lepton_write_word(unsigned int reg,uint16_t word);
HAL_StatusTypeDef lepton_read_command(unsigned int moduleID, unsigned int commandID,uint8_t * data );
HAL_StatusTypeDef lepton_command(unsigned int moduleID, unsigned int commandID, unsigned int command);
HAL_StatusTypeDef agc_enable();
HAL_StatusTypeDef set_reg(unsigned int reg);
uint16_t read_reg(unsigned int reg);
HAL_StatusTypeDef read_data();
HAL_StatusTypeDef lepton_read_data(uint8_t * data);

HAL_StatusTypeDef init_lepton_command_interface(void);
HAL_StatusTypeDef enable_lepton_agc();
HAL_StatusTypeDef enable_telemetry(void);
HAL_StatusTypeDef get_scene_stats(uint16_t *min, uint16_t *max, uint16_t *avg);
HAL_StatusTypeDef enable_rgb888(LEP_PCOLOR_LUT_E pcolor_lut);

#endif

