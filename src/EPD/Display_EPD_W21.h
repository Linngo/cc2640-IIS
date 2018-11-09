
#ifndef _DISPLAY_EPD_W21_H_
#define _DISPLAY_EPD_W21_H_


extern bool EPD_W21_Init(void);
extern void EPD_Dis_Part(unsigned char xStart,unsigned char xEnd,unsigned long yStart,unsigned long yEnd,unsigned char *DisBuffer,unsigned char Label);
extern void EPD_Dis_Full(unsigned char *DisBuffer,unsigned char Label);
extern void EPD_init_Full(void);
extern void EPD_init_Part(void);
extern void EPD_close(void);
extern void test(void);


#endif
/***********************************************************
						end file
***********************************************************/


