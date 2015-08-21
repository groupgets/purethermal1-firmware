#ifndef LEPTON_I2C_H_
#define LEPTON_I2C_H_

#define LEPTON_ADDRESS  (0x2A<<1)

#define AGC (0x01)
#define SYS (0x02)
#define VID (0x03)
#define OEM (0x08)

#define GET (0x00)
#define SET (0x01)
#define RUN (0x02)

HAL_StatusTypeDef lepton_command(unsigned int moduleID, unsigned int commandID, unsigned int command);
HAL_StatusTypeDef agc_enable();
HAL_StatusTypeDef set_reg(unsigned int reg);
uint16_t read_reg(unsigned int reg);
HAL_StatusTypeDef read_data();
int read_lepton_regs(void);

#endif

