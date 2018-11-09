#include "Display_EPD_W21_Config.h"
#include "board.h"
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ssi.h)

#define SPI_BIT_RATE	12000000

#define EPD_W21_MOSI_0	PIN_setOutputValue(epdpinHandle, IOID_9,0)
#define EPD_W21_MOSI_1	PIN_setOutputValue(epdpinHandle, IOID_9,1)

#define EPD_W21_CLK_0	PIN_setOutputValue(epdpinHandle, IOID_10,0)
#define EPD_W21_CLK_1	PIN_setOutputValue(epdpinHandle, IOID_10,1)

#define EPD_W21_CS_0	PIN_setOutputValue(epdpinHandle, Board_LCD_CSN,0)
#define EPD_W21_CS_1	PIN_setOutputValue(epdpinHandle, Board_LCD_CSN,1)

#define EPD_W21_DC_0	PIN_setOutputValue(epdpinHandle, Board_LCD_MODE,0)
#define EPD_W21_DC_1	PIN_setOutputValue(epdpinHandle, Board_LCD_MODE,1)

#define EPD_W21_RST_0	PIN_setOutputValue(epdpinHandle, Board_LCD_MODE,1)
#define EPD_W21_RST_1	PIN_setOutputValue(epdpinHandle, Board_LCD_MODE,1)

#define EPD_W21_BUSY_LEVEL 0
#define isEPD_W21_BUSY  PIN_getInputValue(Board_LCD_BUSY)

static PIN_Handle epdpinHandle = NULL;
static PIN_State  epdpinState;

//static SPI_Handle spiHandle = NULL;
//static SPI_Params spiParams;

PIN_Config boardEPDTable[] =
{
	Board_LCD_MODE | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MAX,
	Board_LCD_CSN  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL      | PIN_DRVSTR_MAX,
	Board_LCD_BUSY | PIN_INPUT_EN       | PIN_NOPULL    | PIN_IRQ_NEGEDGE   | PIN_HYSTERESIS,
	IOID_9         | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MAX,
	IOID_10        | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL      | PIN_DRVSTR_MAX,
	
	PIN_TERMINATE
};

const uint8_t LUTDefault_part[31] = {
	0x32,	// command

	0x10
	,0x18
	,0x18
	,0x28
	,0x18
	,0x18
	,0x18
	,0x18
	,0x08
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
	,0x13
	,0x11
	,0x22
	,0x63
	,0x11
	,0x00
	,0x00
	,0x00
	,0x00
	,0x00
};

const uint8_t LUTDefault_full[31] = {
	0x32,	// command

	0x00
	,0x02
	,0x12
	,0x11
	,0x51
	,0x61
	,0x61
	,0x62
	,0x62
	,0x66
	,0x26
	,0x2A
	,0x1A
	,0x19
	,0x19
	,0x19
	,0x99
	,0x99
	,0x88
	,0x00
	,0x80
	,0x35
	,0x22
	,0x11
	,0x13
	,0x11
	,0x21
	,0x11
	,0x72
	,0x11
};

//static bool Spi_open(uint32_t bitRate);
//static void Spi_close(void);
//static void IsBusy(void);
//static int Spi_read(uint8_t *buf, size_t length);
//static int Spi_write(const uint8_t *buf, size_t length);
static void SPI_Write(uint8_t value);

static void SPI_Write(uint8_t value)                                    
{                                                           
    uint8_t i;

    for(i=0; i<8; i++)   
    {
        EPD_W21_CLK_0;
        if(value & 0x80)
        	EPD_W21_MOSI_1;
        else
        	EPD_W21_MOSI_0;		
        value = (value << 1); 
        EPD_W21_CLK_1;  
    }
}

static uint8_t ReadBusy(void)
{
	unsigned long i=0;	
	for(i=0;i<400;i++)
	{
		if(isEPD_W21_BUSY==EPD_W21_BUSY_LEVEL) {
			return 1;
		}
		Task_sleep(10000/Clock_tickPeriod);
	}
	return 0;
}

static void EPD_W21_WriteCMD(uint8_t command)
{
	EPD_W21_CS_0;                   
	EPD_W21_DC_0;		// command write
	//Spi_write((const uint8_t*)command,1);
	SPI_Write(command);
	EPD_W21_CS_1;
}
//static void EPD_W21_WriteDATA(uint8_t command)
//{
//    EPD_W21_CS_0;                   
//	EPD_W21_DC_1;		// command write
//	SPI_Write(command);
//	EPD_W21_CS_1;
//}

static void EPD_W21_WriteCMD_p1(uint8_t command,uint8_t para)
{
	//while(isEPD_W21_BUSY == 1);	// wait
	ReadBusy();	

    EPD_W21_CS_0;                   
	EPD_W21_DC_0;		// command write
//	Spi_write((const uint8_t*)command,1);
	SPI_Write(command);
	EPD_W21_DC_1;		// command write
//	Spi_write((const uint8_t*)para,1);
	SPI_Write(para);
	EPD_W21_CS_1;
}

//static void EPD_W21_WriteCMD_p2(uint8_t command,uint8_t para1,uint8_t para2)
//{
//	while(isEPD_W21_BUSY == 1);	// wait	
//
//  EPD_W21_CS_0;                   
//	EPD_W21_DC_0;		// command write
//	SPI_Write(command);
//	EPD_W21_DC_1;		// command write
//	SPI_Write(para1);
//	SPI_Write(para2);
//	EPD_W21_CS_1;
//} 
/*********************************************
功能：第一个字节写入的是命令值，剩余的是参数，
在配置阶段使用

*********************************************/
static void EPD_W21_Write(const uint8_t *buf, size_t len)
{
	uint8_t i = 0;
	uint8_t *ptemp;
	
	ptemp = (uint8_t *)buf;

	EPD_W21_CS_0;
	EPD_W21_DC_0;		// command write
	
//	Spi_write((const uint8_t*)ptemp, 1);
	SPI_Write(*ptemp);
	ptemp++;

	EPD_W21_DC_1;		// data write
	
//	Spi_write((const uint8_t*)ptemp, len-1);
	for(i= 0;i<len-1;i++)	// sub the command
	{
		SPI_Write(*ptemp);
		ptemp++;
	}

	EPD_W21_CS_1;
}
/*********************************************
功能：写显示buffer
参数： 	XSize  x方向数量  128点方向，调整为8的整数倍
  YSize  y方向数量	 	   	
  Dispbuff 显示数据保存位置。 要求数据的排列方式必须正确
*********************************************/
static void EPD_W21_WriteDispRam(uint8_t XSize,uint16_t YSize,
								 uint8_t *Dispbuff)
{
	int i = 0,j = 0;

	if(XSize%8 != 0)
	{
		XSize = XSize+(8-XSize%8);
	}
	XSize = XSize/8;

	//while(isEPD_W21_BUSY == 1);	//wait
	ReadBusy();		
	
	EPD_W21_CS_0;                   
	EPD_W21_DC_0;		//command write
//	Spi_write(&cmd,1);
	SPI_Write(0x24);
	EPD_W21_DC_1;		//data write
//	Spi_write(Dispbuff,YSize*XSize);

	for(i=0;i<YSize;i++)
	{
		for(j=0;j<XSize;j++)
		{
			SPI_Write(*Dispbuff);
			Dispbuff++;
		}
	}
	EPD_W21_CS_1;
}
/*********************************************
功能：  写显示buffer 用来将某一个区域写入相同的显示内容。
参数： 	XSize  x方向数量  128点方向，调整为8的整数倍
  YSize  y方向数量	 	   	
  dispdata 显示数据。
*********************************************/
static void EPD_W21_WriteDispRamMono(uint8_t XSize,uint16_t YSize,
									 uint8_t dispdata)
{
	int i = 0,j = 0;

	if(XSize%8 != 0)
	{
		XSize = XSize+(8-XSize%8);
	}
	XSize = XSize/8;
	//while(isEPD_W21_BUSY == 1);	// wait	
	ReadBusy();	

        EPD_W21_CS_0;                   
	EPD_W21_DC_0;		// command write
//	Spi_write(&cmd,1);
	SPI_Write(0x24);
	EPD_W21_DC_1;		// data write
//	Spi_write(&dispdata,YSize*XSize);
	for(i=0;i<YSize;i++)
	{
		for(j=0;j<XSize;j++)
		{
			SPI_Write(dispdata);
		}
	}
	
	EPD_W21_CS_1;
}

static void EPD_W21_POWERON(void)
{
	EPD_W21_WriteCMD_p1(0x22,0xc0);
	EPD_W21_WriteCMD(0x20);
	//EPD_W21_WriteCMD(0xff);
}
//static void EPD_W21_POWEROFF(void)
//{  	EPD_W21_WriteCMD_p1(0x22,0xc3);
//	EPD_W21_WriteCMD(0x20);
////	EPD_W21_WriteCMD(0xff);
//}


static void EPD_W21_SetRamArea(uint8_t Xstart,uint8_t Xend,
							   uint8_t Ystart,uint8_t Ystart1,uint8_t Yend,uint8_t Yend1)
{
    uint8_t RamAreaX[3];	// X start and end
	uint8_t RamAreaY[5]; 	// Y start and end
	RamAreaX[0] = 0x44;	// command
	RamAreaX[1] = Xstart;
	RamAreaX[2] = Xend;
	RamAreaY[0] = 0x45;	// command
	RamAreaY[1] = Ystart;
	RamAreaY[2] = Ystart1;
	RamAreaY[3] = Yend;
    RamAreaY[4] = Yend1;
	EPD_W21_Write(RamAreaX, sizeof(RamAreaX));
	EPD_W21_Write(RamAreaY, sizeof(RamAreaY));
}
static void EPD_W21_SetRamPointer(uint8_t addrX,uint8_t addrY,uint8_t addrY1)
{
    uint8_t RamPointerX[2];	// default (0,0)
	uint8_t RamPointerY[3]; 	
	RamPointerX[0] = 0x4e;
	RamPointerX[1] = addrX;
	RamPointerY[0] = 0x4f;
	RamPointerY[1] = addrY;
	RamPointerY[2] = addrY1;
	
	EPD_W21_Write(RamPointerX, sizeof(RamPointerX));
	EPD_W21_Write(RamPointerY, sizeof(RamPointerY));
}
static void part_display(uint8_t RAM_XST,uint8_t RAM_XEND,uint8_t RAM_YST,uint8_t RAM_YST1,uint8_t RAM_YEND,uint8_t RAM_YEND1)
{    
	EPD_W21_SetRamArea(RAM_XST,RAM_XEND,RAM_YST,RAM_YST1,RAM_YEND,RAM_YEND1);  	/*set w h*/
        EPD_W21_SetRamPointer (RAM_XST,RAM_YST,RAM_YST1);		    /*set orginal*/
}
//static void EPD_W21_UpdataDisplay(uint8_t *imgbuff,uint8_t xram,uint16_t yram)
//{
//	EPD_W21_WriteDispRam(xram, yram, imgbuff);
////	EPD_W21_Update();
//}

//=========================functions============================

//-------------------------------------------------------
//Func	: void EPD_W21_DispInit(void)
//Desc	: display parameters initinal
//Input	: none
//Output: none
//Return: none
//Author: 
//Date	: 2011/12/24
//-------------------------------------------------------
static void EPD_W21_DispInit(void)
{

//	EPD_W21_Write(softstart, sizeof(softstart));	// X decrease, Y decrease
//
//	EPD_W21_Write(GDOControl, sizeof(GDOControl));	// Pannel configuration, Gate selection
//
//	EPD_W21_Write(Rambypass, sizeof(Rambypass));	// Rambypass
//
//	EPD_W21_Write(MAsequency, sizeof(MAsequency));	// MAsequency
//
//	EPD_W21_Write(GDVol, sizeof(GDVol));		    // GDVol setting
//
//	EPD_W21_Write(SDVol, sizeof(SDVol));		    // SDVol setting
//
//	EPD_W21_Write(VCOMVol, sizeof(VCOMVol));		// VCOM setting
//
//	EPD_W21_Write(DummyLine, sizeof(DummyLine));	// dummy line per gate
//	
//	EPD_W21_Write(Gatetime, sizeof(Gatetime));		// Gage time setting
//	
//	EPD_W21_Write(RamDataEntryMode, sizeof(RamDataEntryMode));	// X increase, Y decrease

	EPD_W21_Write(GDOControl, sizeof(GDOControl));	// Pannel configuration, Gate selection

	EPD_W21_Write(softstart, sizeof(softstart));	// X decrease, Y decrease
	
	//EPD_W21_Write(Rambypass, sizeof(Rambypass));	// RAM bypass setting
//	EPD_W21_Write(MAsequency, sizeof(MAsequency));	// clock enable
	//EPD_W21_Write(GDVol, sizeof(GDVol));			// Gate voltage setting
	//EPD_W21_Write(SDVol, sizeof(SDVol));			// Source voltage setting
	EPD_W21_Write(VCOMVol, sizeof(VCOMVol));		// VCOM setting

	//EPD_W21_Write(BOOSTERFB, sizeof(BOOSTERFB));	// Hi-V feedback selection
	EPD_W21_Write(DummyLine, sizeof(DummyLine));	// dummy line per gate

	EPD_W21_Write(Gatetime, sizeof(Gatetime));		// Gage time setting

	EPD_W21_Write(RamDataEntryMode, sizeof(RamDataEntryMode));	// X increase, Y decrease
	
	EPD_W21_SetRamArea(0x00,(xDot-1)/8,(yDot-1)%256,(yDot-1)/256,0x00,0x00);	// X-source area,Y-gage area
	EPD_W21_SetRamPointer(0x00,(yDot-1)%256,(yDot-1)/256);	// set ram
}
bool EPD_W21_Init(void)
{
    //bool f;

    epdpinHandle = PIN_open(&epdpinState, boardEPDTable);
    if (epdpinHandle == NULL)
    {
        return false;
    }
    ///* Initialise SPI. Subsequent calls will do nothing. */
    //SPI_init();

    ///* Make sure SPI is available */
    //f = Spi_open(SPI_BIT_RATE);

    //if (f)
    //{
		EPD_W21_DispInit();		// pannel configure
	//	if (!f)
	//	{
	//		Spi_close();
	//		PIN_close(epdpinHandle);
	//	}
    //}
    return true;
}

void EPD_close(void)
{
    PIN_close(epdpinHandle);
}
//-------------------------------------------------------
//Func	: EPD_W21_EnableChargepump
//Desc	: 
//Input	:
//Output: 
//Return: 
//Author: 
//Date	: 2011/12/24
//-------------------------------------------------------
//static void EPD_W21_EnableChargepump(void)
//{
//	EPD_W21_WriteCMD_p1(0xf0,0x8f);
//	EPD_W21_WriteCMD_p1(0x22,0xc0);
//	EPD_W21_WriteCMD(0x20);
//	EPD_W21_WriteCMD(0xff);
//}

//-------------------------------------------------------
//Func	: EPD_W21_DisableChargepump
//Desc	: 
//Input	:
//Output: 
//Return: 
//Author: 
//Date	: 2011/12/24
//-------------------------------------------------------
//static void EPD_W21_DisableChargepump(void)
//{
//	EPD_W21_WriteCMD_p1(0x22,0xf0);
//	EPD_W21_WriteCMD(0x20);
//	EPD_W21_WriteCMD(0xff);
//}
//-------------------------------------------------------
//Func	: EPD_W21_Update
//Desc	: 
//Input	:
//Output: 
//Return: 
//Author: 
//Date	: 2011/12/24
//-------------------------------------------------------
static void EPD_W21_Update(void)
{
	EPD_W21_WriteCMD_p1(0x22,0xc4);
	EPD_W21_WriteCMD(0x20);
	EPD_W21_WriteCMD(0xff);
}

static void EPD_W21_Update_Part(void)
{
	EPD_W21_WriteCMD_p1(0x22,0x04);
	//EPD_W21_WriteCMD_p1(0x22,0x08);
	EPD_W21_WriteCMD(0x20);
	EPD_W21_WriteCMD(0xff);
}
//-------------------------------------------------------
//Func	: EPD_W21_WirteLUT(uint8_t *LUTvalue)
//Desc	: write the waveform to the dirver's ram 
//Input	: *LUTvalue, the wavefrom tabe address
//Output: none
//Return: none
//Author: 
//Date	: 2011/12/24
//-------------------------------------------------------
static void EPD_W21_WirteLUT(uint8_t *LUTvalue,uint8_t Size)
{	
	EPD_W21_Write(LUTvalue, Size);
}

void EPD_init_Full(void)
{		
	EPD_W21_Init();			// display
    EPD_W21_WirteLUT((uint8_t *)LUTDefault_full,sizeof(LUTDefault_full));	
	EPD_W21_POWERON();
	Task_sleep(1000/Clock_tickPeriod);
}

void EPD_init_Part(void)
{		
	EPD_W21_Init();			// display
	
	EPD_W21_WirteLUT((uint8_t *)LUTDefault_part,sizeof(LUTDefault_part));
	EPD_W21_POWERON();
//	//清除底色
//	//EPD_W21_SetRamArea(0x00,0x0f,0x27,0x01,0x00,0x00);	  
//	//EPD_W21_SetRamPointer(0x00,0x27,0x01);	// set ram
//	part_display(0,xDot/8,yDot%256,yDot/256,0,0);
//	EPD_W21_WriteDispRamMono(xDot, yDot, 0xff);	// white
//	EPD_W21_Update_Part();
//	driver_delay_xms(500);
//	
//	
//	part_display(0,xDot/8,yDot%256,yDot/256,0,0);
//	EPD_W21_WriteDispRamMono(xDot, yDot, 0xff);	// white	
	
	Task_sleep(1000/Clock_tickPeriod);
}
/**********************************************************
参数： Label  
       =1 显示 DisBuffer中的内容
 =0 显示 DisBuffer中第一个字节的内容
***********************************************************/

void EPD_Dis_Full(uint8_t *DisBuffer,uint8_t Label)
{
    EPD_W21_SetRamPointer(0x00,(yDot-1)%256,(yDot-1)/256);	// set ram
	if(Label==0)
	{
		EPD_W21_WriteDispRamMono(xDot, yDot, 0xff);	// white	
	}
	else
	{
		EPD_W21_WriteDispRam(xDot, yDot, (uint8_t *)DisBuffer);	// white
	}	
	EPD_W21_Update();	
}

/**********************************************************
参数： 
  xStart
   x方向开始坐标
  xEnd
   x方向结束坐标
  yStart
   y方向开始坐标
  yEnd
   y方向结束坐标
  DisBuffer
   显示buffer
  Label  
   =1 显示 DisBuffer中的内容
   =0 显示 DisBuffer中第一个字节的内容
   
***********************************************************/

void EPD_Dis_Part(uint8_t xStart,uint8_t xEnd,unsigned long yStart,unsigned long yEnd,uint8_t *DisBuffer,uint8_t Label)
{
	if(Label==0)
	{
		part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);
		EPD_W21_WriteDispRamMono(xEnd-xStart, yEnd-yStart+1, DisBuffer[0]);	// black
 		EPD_W21_Update_Part();
		//	driver_delay_xms(200);
		Task_sleep(200000/Clock_tickPeriod);
		
		part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);	
		EPD_W21_WriteDispRamMono(xEnd-xStart, yEnd-yStart+1, DisBuffer[0]);	// black
		//driver_delay_xms(200);	
	}
	else
            if(Label==1)
            {
//                    part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);	// set ram	
//                    EPD_W21_WriteDispRam(xEnd-xStart, yEnd-yStart+1,DisBuffer);
//                    EPD_W21_Update_Part();
//
//                    Task_sleep(200000/Clock_tickPeriod);
//                    part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);
//                    EPD_W21_WriteDispRam(xEnd-xStart, yEnd-yStart+1,DisBuffer);
                    part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);	// set ram	
                    EPD_W21_WriteDispRam(xEnd-xStart, yEnd-yStart+1,DisBuffer);
            }
            else
            {
                    part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);	// set ram	
                    EPD_W21_WriteDispRam(xEnd-xStart, yEnd-yStart+1,DisBuffer);
                    EPD_W21_Update_Part();
                    //		driver_delay_xms(300);
                    Task_sleep(200000/Clock_tickPeriod);
                    part_display(xStart/8,xEnd/8,yEnd%256,yEnd/256,yStart%256,yStart/256);
                    EPD_W21_WriteDispRam(xEnd-xStart, yEnd-yStart+1,DisBuffer);
                    //driver_delay_xms(300);
            }
}

void test(void)
{
    EPD_W21_Update_Part();;
//    Task_sleep(200000/Clock_tickPeriod);
//    part_display(0/8,199/8,199%256,199/256,0%256,0/256);
//    EPD_W21_WriteDispRam(199, 200,DisBuffer);
}

///*******************************************************************************
//*
//*   SPI interface
//*
//*******************************************************************************/

///*******************************************************************************
//* @fn          Spi_write
//*
//* @brief       Write to an SPI device
//*
//* @param       buf - pointer to data buffer
//* @param       len - number of bytes to write
//*
//* @return      '0' if success, -1 if failed
//*/
//static int Spi_write(const uint8_t *buf, size_t len)
//{
//    SPI_Transaction masterTransaction;

//    masterTransaction.count  = len;
//    masterTransaction.txBuf  = (void*)buf;
//    masterTransaction.arg    = NULL;
//    masterTransaction.rxBuf  = NULL;

//    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
//}


///*******************************************************************************
//* @fn          Spi_read
//*
//* @brief       Read from an SPI device
//*
//* @param       buf - pointer to data buffer
//* @param       len - number of bytes to write
//*
//* @return      '0' if success, -1 if failed
//*/
//static int Spi_read(uint8_t *buf, size_t len)
//{
//    SPI_Transaction masterTransaction;

//    masterTransaction.count = len;
//    masterTransaction.txBuf = NULL;
//    masterTransaction.arg = NULL;
//    masterTransaction.rxBuf = buf;

//    return SPI_transfer(spiHandle, &masterTransaction) ? 0 : -1;
//}


///*******************************************************************************
//* @fn          Spi_open
//*
//* @brief       Open the RTOS SPI driver
//*
//* @param       bitRate - transfer speed in bits/sec
//*
//* @return      true if success
//*/
//static bool Spi_open(uint32_t bitRate)
//{
//    /*  Configure SPI as master */
//    SPI_Params_init(&spiParams);
//    spiParams.bitRate = bitRate;
//    spiParams.mode = SPI_MASTER;
//    spiParams.transferMode = SPI_MODE_BLOCKING;

//    /* Attempt to open SPI. */
//    spiHandle = SPI_open(Board_SPI0, &spiParams);

//    return spiHandle != NULL;
//}

///*******************************************************************************
//* @fn          Spi_close
//*
//* @brief       Close the RTOS SPI driver
//*
//* @return      none
//*/
//static void Spi_close(void)
//{
//    if (spiHandle != NULL)
//    {
//        // Close the RTOS driver
//        SPI_close(spiHandle);
//        spiHandle = NULL;
//    }
//}


///*******************************************************************************
//* @fn          Spi_flash
//*
//* @brief       Get rid of garbage from the slave
//*
//* @param       none
//*
//* @return      none
//*/
//static void IsBusy(void)
//{
//    /* make sure SPI hardware module is done  */
//    while(SSIBusy(((SPICC26XXDMA_HWAttrsV1*)spiHandle->hwAttrs)->baseAddr))
//    { };
//}

///***********************************************************
//   end file
//***********************************************************/
