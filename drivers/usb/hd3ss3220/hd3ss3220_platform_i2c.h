#ifndef HD3SS3220_PLATFORM_I2C_H
#define HD3SS3220_PLATFORM_I2C_H
/*  NBQ - AlbertWu - [NBQ-1056] - [USB] Judge HW ID in HD3SS3220 USB type C driver. */
#include <fih/hwid.h>
/* end  NBQ - AlbertWu - [NBQ-1056] */


void usbpd_pf_i2c_init(struct i2c_adapter *adapter);

void hd3ss3220_platform_wr_reg8(uint8_t addr, uint8_t val);
uint8_t hd3ss3220_platform_rd_reg8(uint8_t addr);
void hd3ss3220_platform_set_bit8(uint8_t addr, uint8_t mask);
void hd3ss3220_platform_clr_bit8(uint8_t addr, uint8_t mask);
void hd3ss3220_platform_put_bit8(uint8_t addr, uint8_t mask, uint8_t val);

void hd3ss3220_platform_fifo_write8(uint8_t addr, const uint8_t *p_data, uint16_t size);
void hd3ss3220_platform_fifo_read8(uint8_t addr, uint8_t *p_data, uint16_t size);
void hd3ss3220_platform_block_write8(const uint8_t addr,const uint8_t *p_data, uint16_t size);
void hd3ss3220_platform_block_read8(const uint8_t addr, uint8_t *p_data, uint16_t size);




#endif
