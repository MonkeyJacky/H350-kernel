
#ifndef __FM_I2C_H_
#define __FM_I2C_H_



#if 0
extern int fm_write_8bit(unsigned char reg,unsigned char *val,int count);

extern int fm_read_8bit(unsigned char reg,unsigned char *buf,int count);

extern int fm_write_bit(unsigned char reg,unsigned char val);
extern int fm_read_bit(unsigned char reg);
#endif

extern int fm5807_i2c_rxdata (char *rxdata,int len);
extern int fm5807_i2c_txdata (char *txdata,int len);
#endif
