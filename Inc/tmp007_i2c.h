#ifndef TMP007_I2C_H_
#define TMP007_I2C_H_

#define TMP007_VOBJ       0x00
#define TMP007_TDIE       0x01
#define TMP007_CONFIG     0x02
#define TMP007_TOBJ       0x03
#define TMP007_STATUS     0x04
#define TMP007_STATMASK   0x05

#define TMP007_CFG_RESET    0x8000
#define TMP007_CFG_MODEON   0x1000
#define TMP007_CFG_1SAMPLE  0x0000
#define TMP007_CFG_2SAMPLE  0x0200
#define TMP007_CFG_4SAMPLE  0x0400
#define TMP007_CFG_8SAMPLE  0x0600
#define TMP007_CFG_16SAMPLE 0x0800
#define TMP007_CFG_ALERTEN  0x0100
#define TMP007_CFG_ALERTF   0x0080
#define TMP007_CFG_TRANSC   0x0040

#define TMP007_STAT_ALERTEN 0x8000
#define TMP007_STAT_CRTEN   0x4000

#define TMP007_I2CADDR (0x40<<1)
#define TMP007_DEVID 0x1F

long get_last_mili_celisius(void);
HAL_StatusTypeDef tmp007_write_word(unsigned int reg,unsigned int data);
uint16_t tmp007_read_reg(unsigned int reg);
long get_mili_celisius(void);
int convert_C_to_F(int C);
int read_tmp007_regs(void);

#endif

