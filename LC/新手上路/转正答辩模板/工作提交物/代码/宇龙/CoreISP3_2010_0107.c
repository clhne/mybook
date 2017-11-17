/*
   Copyright (C) 2009, Hisense mobile technology Ltd.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
 
 */
 
#include "camera.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/arch/mfp.h>
#include <asm/arch/gpio.h>
#include <asm/hardware.h>
#include <linux/err.h>
#include <asm/arch/camera.h>
#include <linux/clk.h>
#include <linux/i2c.h>

#include "CoreISP3.h"

/******add for power control****/
#include <asm/gpio.h>
/******add for power control****/

//fang
static int zoom_flag = 0;
//#include "LSC_TABLE.h"
//lcm_dbg #include "LSC_TABLE_Samsung.h"

#ifdef ISP3_MCU_ARRAY  // Please open this define and disable ISP3_EVB_ENABLE define 
//#include "_ISP3_MCU_.h"
//#include "MCNEX_V01.h"
//#include "Hisense_V02.h"
//lcm_dbg #include "Hisense_0511.h"
#include "MCU_BIN.h"
#endif

#ifdef ISP3_EVB_ENABLE
#include "cl_header.h"
#include "gpio.h"
#include "mpeg4ip.h"  // for ROUD
#include "SensorDeviceInterface.h"

#define DOWN_TIME_CHECK  // for time
#endif

/* Unique ID allocation */
static struct i2c_client *g_client;
//enIsp3ZoomRate g_zoom_rate = enISP_ZOOM_480_360;
enIsp3ZoomRate g_zoom_rate = 0;

#if defined(FEATURE_CS7903) 			
int g_faceTrackingOn = Cl_True;
#else
int g_faceTrackingOn = Cl_False;
#endif

/*因为在第一次进camera预览时很慢，
因为coreLogic程序在初始化完后默认就preview了，
此时再调用CLI6000_start_capture()函数反而会很长时间才能预览。
故在第一次进入预览时就不调用CLI6000_start_capture()中的对预览的设置了。
其他情况正常调用该函数。*/
//int g_isFirstInitialized = Cl_False;  //2009.11.16 试验固件没有这样的问题了，故去掉。
int g_isFlash = Cl_False;
static struct timer_list flash_timer;

/* CICLK for QCI */
#define CICLK	2600
//#define CICLK 	5200

#define ISP3_DEBUG
#undef ISP3_DEBUG
#define DM_JPEG 0
#define DM_IIC  1
#define DM_ERROR  2

#ifdef  ISP3_DEBUG
#if 0
#define DebugMessage(prefix,fmt, arg...)     \
	if(prefix == DM_IIC)    {printk(KERN_DEBUG fmt, ##arg);}	\
	else if(prefix == DM_JPEG)    {printk(KERN_INFO fmt, ##arg);}	\
	else	 if(prefix == DM_ERROR)    {printk(KERN_ERR fmt, ##arg);}	
#else
#define DebugMessage(prefix,fmt, arg...)     printk(KERN_INFO "%s(line %d): " fmt "\n",  \
                __FUNCTION__, __LINE__, ##arg)
#endif
#else 
#define DebugMessage(prefix,fmt, arg...)        do {} while (0)
#endif

#define WaitTime_us(cnt)  msleep(cnt/1000)

void CoreISP3_Get_ioctl(void);
void ISP3_Set_FlashOnOff(Cl_Bool flashOn);
static void sensor_flash_ctl(int i);

#define I2C_USE_GPIO

#ifdef I2C_USE_GPIO
static void cam_gpio_i2c_output(unsigned int the_gpio, unsigned char level)
{
	int ret = 0;
	
	ret = gpio_request(the_gpio, "AP I2C GPIO");
	if (ret) {
		printk("gpio_requset failed, return :%d\n", ret);
		return;
	}
	gpio_direction_output(the_gpio, level);
	gpio_free(the_gpio);
}

static int cam_gpio_i2c_input(unsigned int the_gpio)
{
	int ret = 0;

	ret = gpio_request(the_gpio, "AP I2C GPIO");
	if (ret) {
		printk("gpio_requset failed, return :%d\n", ret);
		return;
	}
    ret = gpio_direction_input(the_gpio);
    gpio_free(the_gpio);
    return ret;
}

#define  	I2C_READ					0x01
#define  	READ_CMD					1
#define  	WRITE_CMD					0


#define CAM_5M_I2C_SCL          mfp_to_gpio(MFP_PIN_GPIO107)
#define CAM_5M_I2C_SDA          mfp_to_gpio(MFP_PIN_GPIO108)
#define SENSOR_I2C_DELAY        (80)   // 80KHz for iic speed.... 100 92KHz

#define SET_SCCB_DATA_HIGH      cam_gpio_i2c_output(CAM_5M_I2C_SDA, 1);
#define SET_SCCB_DATA_LOW       cam_gpio_i2c_output(CAM_5M_I2C_SDA, 0);
#define SET_SCCB_CLK_HIGH       cam_gpio_i2c_output(CAM_5M_I2C_SCL, 1);
#define SET_SCCB_CLK_LOW        cam_gpio_i2c_output(CAM_5M_I2C_SCL, 0);
#define SET_SCCB_CLK_OUTPUT
#define SET_SCCB_DATA_INPUT     GPDR3 = (GPDR3 & ~(1<<12))
#define SET_SCCB_DATA_OUTPUT
#define GET_SCCB_DATA_BIT       ((GPLR3 & 0x1000) >> 12)

#define I2C_START_TRANSMISSION \
{ \
    volatile ClUint_8 j; \
    SET_SCCB_CLK_OUTPUT; \
    SET_SCCB_DATA_OUTPUT; \
    SET_SCCB_CLK_HIGH; \
    SET_SCCB_DATA_HIGH; \
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \
    SET_SCCB_DATA_LOW; \
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \
    SET_SCCB_CLK_LOW; \
}

#define I2C_STOP_TRANSMISSION \
{ \
    volatile ClUint_8 j; \
    SET_SCCB_CLK_OUTPUT; \
    SET_SCCB_DATA_OUTPUT; \
    SET_SCCB_CLK_LOW; \
    SET_SCCB_DATA_LOW; \
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \
    SET_SCCB_CLK_HIGH; \
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \
    SET_SCCB_DATA_HIGH; \
}

static void SCCB_send_byte(ClUint_8 send_byte)
{
    volatile signed char i;
    volatile ClUint_8 j;

    for (i = 7; i >= 0; i--) { /* data bit 7~0 */
        if (send_byte & (1 << i)) {
            SET_SCCB_DATA_HIGH;
        }else {
            SET_SCCB_DATA_LOW;
        }

        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SET_SCCB_CLK_HIGH;
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SET_SCCB_CLK_LOW;
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
    }
    /* don't care bit, 9th bit */
    SET_SCCB_DATA_LOW;
    SET_SCCB_DATA_INPUT;
    SET_SCCB_CLK_HIGH;
    for (j = 0; j < SENSOR_I2C_DELAY; j++);
    SET_SCCB_CLK_LOW;
    SET_SCCB_DATA_OUTPUT;
}   /* SCCB_send_byte() */

static ClUint_8 SCCB_get_byte(void)
{
    volatile signed char i;
    volatile ClUint_8 j;
    ClUint_8 get_byte = 0;

    SET_SCCB_DATA_INPUT;

    for (i = 7; i >= 0; i--) { /* data bit 7~0 */
        SET_SCCB_CLK_HIGH;
		for (j = 0; j < SENSOR_I2C_DELAY; j++);
        if (GET_SCCB_DATA_BIT)
            get_byte |= (1 << i);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SET_SCCB_CLK_LOW;
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
    }
    /* don't care bit, 9th bit */
    SET_SCCB_DATA_HIGH;
    SET_SCCB_DATA_OUTPUT;
    for (j = 0; j < SENSOR_I2C_DELAY; j++);
    SET_SCCB_CLK_HIGH;
    for (j = 0; j < SENSOR_I2C_DELAY; j++);
    SET_SCCB_CLK_LOW;

    return get_byte;
}   /* SCCB_get_byte() */



static ClUint_16 I2C_WriteControl(ClUint_8 IICID, ClUint_16 regaddr, ClUint_16 data)
{
   ClUint_8              returnack = Cl_True; 
   
         volatile ClUint_8 j;

        I2C_START_TRANSMISSION;
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(IICID&(~I2C_READ));
		
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(regaddr);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(data);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        I2C_STOP_TRANSMISSION;

    return returnack;
}

static ClUint_16 I2C_ReadControl(ClUint_8 IICID, ClUint_16 regaddr)       
{
    ClUint_16 readdata1 = 0, readdata = 0;
    ClUint_8              returnack;
	
         volatile ClUint_8 j;

        I2C_START_TRANSMISSION;
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
	    SCCB_send_byte(IICID&(~I2C_READ));

        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(regaddr);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);

        I2C_START_TRANSMISSION;
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(IICID |I2C_READ);
		
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        readdata = SCCB_get_byte();
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        I2C_STOP_TRANSMISSION;

    return readdata;
}
#endif

/*********************************************************************
*Name			:	CoreISP3_I2C_Write
*Description	:	Write Data
*Param			:	addr = Addr of Register
					data = Setting value
*return			:	none
*Author			:	
*Remark			:	You have to use instead of System's function
*Log			:	
**********************************************************************/
#ifdef I2C_USE_GPIO
ClUint_8 CoreISP3_I2C_Read( ClUint_16 addr )
{
    I2C_WriteControl(IICID_A, 0x01, addr>>8);
    return I2C_ReadControl(IICID_B, addr);
}

int CoreISP3_I2C_Write( ClUint_16 addr, ClUint_8 data )
{
    I2C_WriteControl(IICID_A, 0x01, addr>>8);
    I2C_WriteControl(IICID_B, addr&0xff, data);
    return 1;
}

ClUint_8  CoreISP3_I2C_Write_Bulk( ClUint_16 addr, ClUint_8 data )
{
    I2C_WriteControl(IICID_B, addr&0xff, data);
    return 1;
}


#ifdef IIC_BURST_MODE
/*********************************************************************
*Name			:	CoreISP3_I2C_Burst_Write
*Description	:	Write IIC Addr
*Param			:	addr		Register's Addr
*return			:	NONE
*Author			:	
*Remark			:	You have to use instead of System's function
*Log			:	
**********************************************************************/
ClUint_8 CoreISP3_I2C_Burst_Write(ClUint_16 addr, ClUint_8 data )
{
    I2C_WriteControl(IICID_B, addr&0xff, data);
    return 1;
}
#endif


#else

/*********************************************************************
*Name			:	CoreISP3_I2C_Read
*Description	:	Read Data from register of addr
*Param			:	addr = read register
*return			:	Register value
*Author			:	
*Remark			:	You have to use instead of System's function
*Log			:	
**********************************************************************/
ClUint_8 CoreISP3_I2C_Read( ClUint_16 addr )
{
	union i2c_smbus_data highbyte;
	union i2c_smbus_data data;

	if(g_client == NULL)
	{
		DebugMessage(DM_ERROR,"CoreISP3_I2C_Read fail, g_client null\n");
		return Cl_False;
	}
	//3 1. IIC ID = 0x82
	//3 2. IIC Addr = 0x01
	//3 3. IIC Data = high 8 bit of addr value
	//3 4. IIC ID = 0x80
	//3 5. IIC Addr = low 8 bit of addr value. & Read Data with IIC ID = 0x80


	//DebugMessage(DM_IIC,"read reg addr = %x\n",addr);
	highbyte.byte =  addr>>8;
	if (i2c_smbus_xfer(g_client->adapter,0x41,g_client->flags,I2C_SMBUS_WRITE, 0x01, I2C_SMBUS_BYTE_DATA, &highbyte)>=0)
	{
		//WaitTime_us(100000);    // 100ms
		//lcm_dbg if (i2c_smbus_xfer(g_client->adapter,0x40,g_client->flags,I2C_SMBUS_READ, addr&0xff, I2C_SMBUS_PROC_CALL_2, &data)>=0)
		if (i2c_smbus_xfer(g_client->adapter,0x40,g_client->flags,I2C_SMBUS_READ, addr&0xff, I2C_SMBUS_PROC_CALL_2, &data)>=0)
		{
			//DebugMessage(DM_IIC,"read reg addr = %x,data = %x  ok!!!!!\n ",addr,data.byte);		
			return data.byte; ;		
		}
		else
		{
			DebugMessage(DM_ERROR,"chip addr = %x,subaddr = %x,g_client->name = %s\n",0x40,addr&0xff,g_client->name);
			return Cl_False;
		}
	}
	else
	{

		DebugMessage(DM_ERROR,"chip addr = %x,highbyte = %x,g_client->name = %s\n",0x41,addr>>8,g_client->name);
		return Cl_False;
	}	

}

int CoreISP3_I2C_Write( ClUint_16 addr, ClUint_8 data )
{
	union i2c_smbus_data highbyte;
	union i2c_smbus_data raw;
	
	//3 1. IIC ID = 0x82
	//3 2. IIC Addr = 0x01
	//3 3. IIC Data = high 8 bit of addr value
	//3 4. IIC ID = 0x80
	//3 5. IIC Addr = low 8 bit of addr value
	//3 6. Set IIC Data as data
	if(g_client == NULL)
		return Cl_False;
		
	highbyte.byte = addr>>8;
	raw.byte = data;
	//DebugMessage(DM_IIC,"write reg addr = %x,data = %x\n",addr,data);
	if (i2c_smbus_xfer(g_client->adapter,0x41,g_client->flags,I2C_SMBUS_WRITE, 0x01, I2C_SMBUS_BYTE_DATA, &highbyte)>=0)
	{
		//WaitTime_us(100000);    // 100ms
		if (i2c_smbus_xfer(g_client->adapter,0x40,g_client->flags,I2C_SMBUS_WRITE, addr&0xff, I2C_SMBUS_BYTE_DATA, &raw)>=0)
		{
			//DebugMessage(DM_IIC,"write reg addr = %x,data = %x  ok!!!!!\n ",addr,data);
			return Cl_True;
		}
		else
		{
			DebugMessage(DM_ERROR,"chip addr = %x,subaddr = %x,g_client->name = %s\n",0x40,addr&0xff,g_client->name);
			return Cl_False;
		}
	}
	else
	{
		DebugMessage(DM_ERROR,"chip addr = %x,subaddr = %x,g_client->name = %s\n",0x41,addr>>8,g_client->name);
		return Cl_False;
	}
	
}

ClUint_8  CoreISP3_I2C_Write_Bulk( ClUint_16 addr, ClUint_8 data )
{
	union i2c_smbus_data raw;

	if(g_client == NULL)
		return Cl_False;
	
	raw.byte = data;
	if (i2c_smbus_xfer(g_client->adapter,0x40,g_client->flags,I2C_SMBUS_WRITE, addr&0xff, I2C_SMBUS_BYTE_DATA, &raw)>=0)
	{
		//DebugMessage(DM_IIC,"write reg addr = %x,data = %x  ok!!!!!\n ",addr,data);
		return Cl_True;
	}
	else
	{
		DebugMessage(DM_ERROR,"chip addr = %x,subaddr = %x,g_client->name = %s\n",0x40,addr&0xff,g_client->name);
		return Cl_False;
	}
}

#ifdef IIC_BURST_MODE
/*********************************************************************
*Name			:	CoreISP3_I2C_Burst_Write
*Description	:	Write IIC Addr
*Param			:	addr		Register's Addr
*return			:	NONE
*Author			:	
*Remark			:	You have to use instead of System's function
*Log			:	
**********************************************************************/
ClUint_8 CoreISP3_I2C_Burst_Write(ClUint_16 addr, ClUint_8 data )
{
	union i2c_smbus_data raw;

	if(g_client == NULL)
		return Cl_False;
	
	raw.byte = data;
	if (i2c_smbus_xfer(g_client->adapter,0x40,g_client->flags,I2C_SMBUS_WRITE, addr&0xff, I2C_SMBUS_BYTE_DATA, &raw)>=0)
	{
		//DebugMessage(DM_IIC,"write reg addr = %x,data = %x  ok!!!!!\n ",addr,data);
		return Cl_True;
	}
	else
	{
		DebugMessage(DM_ERROR,"chip addr = %x,subaddr = %x,g_client->name = %s\n",0x40,addr&0xff,g_client->name);
		return Cl_False;
	}
}

#endif

#endif //4 SWIIC

void ISP3_FlashromBurstWrite(int writeStartAddress, ClUint_8* data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount))
{
	int i;
	int writeEndAddress = writeStartAddress+length-1;
	ClUint_16 val;

	val = CoreISP3_I2C_Read(0xE037);
	CoreISP3_I2C_Write(0xE037, val&~0x80);	// Address Auto Inc Off
	
	// Flash Write
	CoreISP3_I2C_Write(0xE070, 0xFD);
	CoreISP3_I2C_Write(0xE0A3, writeStartAddress>>8);
	CoreISP3_I2C_Write(0xE0A4, writeStartAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A6, writeEndAddress>>8);
	CoreISP3_I2C_Write(0xE0A7, writeEndAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A2, 0x29);
	CoreISP3_I2C_Write(0xE0A2, 0x19);
	WaitTime_us(1000*50);
	for(i=0; i<length; i++)
	{
		CoreISP3_I2C_Burst_Write(0xE0A5, data[i]);
		if(callbackFunc != Cl_Null)
		{
			callbackFunc(data, i, length);
		}
		else
		{
			//Sleep(2);
		}
	}

	CoreISP3_I2C_Write(0xE037, val);
}



ClUint_16 CoreISP3_I2C_SensorRead( ClUint_16 addr )
{
	ClUint_16 rdddata = 0;

	ClUint_8 rddhigh = 0;
	ClUint_8 rddlow = 0;

	CoreISP3_I2C_Write(IicAddrH, (ClUint_8)((addr>>8)&0xff));
	WaitTime_us(100000);    // 100ms
	CoreISP3_I2C_Write(IicAddrL, (ClUint_8)(addr&0xff));
	WaitTime_us(100000);    // 100ms

	rddhigh = CoreISP3_I2C_Read(IicWrDtH);
	rddlow = CoreISP3_I2C_Read(IicWrDtL);
	WaitTime_us(100000);    // 100ms

	rddhigh = CoreISP3_I2C_Read(IicWrDtH);
	rddlow = CoreISP3_I2C_Read(IicWrDtL);
	WaitTime_us(100000);    // 100ms

	rdddata = ((ClUint_16)rddhigh << 8) + ((ClUint_16) rddlow);
	DebugMessage(0x00002000, ">>ISP2_REG>>Micron Sensor 0x%x = 0x%x \n ",(ClUint_16)addr, (ClUint_16)rdddata);

	WaitTime_us(100000);    // 100ms

	return rdddata; 
}

void CoreISP3_I2C_Partial_Write( ClUint_16 Addr, ClUint_16 HighBit, ClUint_16 LowBit, ClUint_8 Data )
{
	ClUint_8 BitMask = 0xff;
	ClUint_8 ReadData;

	BitMask = BitMask<<(7-HighBit);
	BitMask = BitMask>>(7-HighBit+LowBit);
	BitMask = BitMask<<LowBit; 
	BitMask = ~BitMask;

	ReadData = CoreISP3_I2C_Read(Addr);
	if(ReadData < 0)
		return;

	ReadData = ReadData&BitMask;
	ReadData = ReadData |Data<<LowBit;

	CoreISP3_I2C_Write(Addr, ReadData);
	DebugMessage(0x00002000, ">>CoreISP3_I2C_Partial_Write>>Addr 0x%x = 0x%x \n ", Addr, CoreISP3_I2C_Read(Addr));
}

void CoreISP3_I2C_ContinuousWrite( ClUint_16 Addr, ClUint_16 Data )
{

	ClUint_8 HighBit, LowBit;

	HighBit=Data>>8;
	LowBit=Data&0xff;

	CoreISP3_I2C_Write(Addr, HighBit);
	CoreISP3_I2C_Write(Addr+1, LowBit);
	//DebugMessage(0x00002000, ">>CoreISP3_I2C_ContinuousWrite>>Addr 0x%x = 0x%x \n ", Addr, CoreISP3_I2C_Read(Addr));   
	// DebugMessage(0x00002000, ">>CoreISP3_I2C_ContinuousWrite>>Addr 0x%x = 0x%x \n ", Addr+1, CoreISP3_I2C_Read(Addr+1));
}
//3 ======================================================= JADE IIC ==========================================================


/*********************************************************************
*Name			:	CoreISP3_Send_Command
*Description	:	Send "ISP3_COMMAND" to ISP3
*Param			:	Cmd_Type	ISP COMMAND
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_Send_Command( enISP3_CMD_TYPE Cmd_Type )
{
	ClUint_16 retval = 1, i = 0;
	//DebugMessage(DM_JPEG, "Send Command to ISP MCU [Command Type : 0x%x]\n", Cmd_Type);

	CoreISP3_I2C_Write(ISP3_COMMAND, Cmd_Type);
	WaitTime_us(100000);    // 100ms  // Jacky add this
	while(retval)
	{
		retval = CoreISP3_I2C_Read(ISP3_COMMAND);
		//for(i=0;i<0xffff;i++);  // must add this delay tiem... or 100ms delay
		WaitTime_us(4000);    // 40ms   // Jacky change 10ms for init speed
		#if 1
		i++;
		if(i>1000)
			{
			//DebugMessage(DM_JPEG, "send isp cmd[0x%x] fail value 0x%x \n", Cmd_Type, retval);
			printk("send isp cmd[0x%x] fail value 0x%x \n", Cmd_Type, retval);
			break;
			}
		#endif
		//DebugMessage(0x00002000, "@@0x%x\n",retval);
	}
}
/*********************************************************************
*Name			:	CoreISP3_SetClock
*Description	:	ISP3 Configuration(Set PLL of ISP3 & Initialize)
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
Cl_Bool CoreISP3_SetClock( void )
{
	//3 PLL OFF
	//if(CoreISP3_I2C_Write(0xe060, 0x03)==Cl_False)
	//	return Cl_False;

	//3 FOUT = {M x FIN} / {P x 2^S} : FOUT is the MCLK
	//3 (24MHz x 54) / (6 x 2^2) = 54MHz           24M x 24 /(5x2^2) = 
#if 1  // YuLong Current settings...try one time
	/* 54MHz */   // 24M MCLK   54M PCLK
	//3 (24MHz x 45) / (5 x 2^2) = 54MHz  zjw add
	//fang
#if 1
	CoreISP3_I2C_Write(0xe062, 0x05);  //P Div
//	CoreISP3_I2C_Write(0xe061, 0x3C);  //M Div	72M
	CoreISP3_I2C_Write(0xe061, 0x2e);  //M Div  60M
//	CoreISP3_I2C_Write(0xe061, 0x56);  //M Div  96M
//	CoreISP3_I2C_Write(0xe061, 0x2D);  //M Div 54M
//	CoreISP3_I2C_Write(0xe061, 0x28);  //M Div  48M 
//	CoreISP3_I2C_Write(0xe061, 0x14);  //M Div    
	CoreISP3_I2C_Write(0xe063, 0x02);  //S  Div
#else
	CoreISP3_I2C_Write(0xe062, 0x0a);  //P Div
	CoreISP3_I2C_Write(0xe061, 0xb8);  //M Div  60M
	CoreISP3_I2C_Write(0xe063, 0x02);  //S  Div
#endif
	// Jacky for test
	/* 96MHz    // 24M MCLK   96M PCLK
	CoreISP3_I2C_Write(0xe062, 0x05);  //P Div
	CoreISP3_I2C_Write(0xe061, 0x50);  //M Div
	CoreISP3_I2C_Write(0xe063, 0x02);  //S  Div
	*/
#else
	/* 54MHz */
	#if 0
	// This for 24Mhz MCLK  54MHz PCLK
	CoreISP3_I2C_Write(0xe062, 0x06);	//4 P	Div
	CoreISP3_I2C_Write(0xe061, 0x36);	//4 M	Div    0x36 = 54
	CoreISP3_I2C_Write(0xe063, 0x02);	//4 S	Div
	#else  // For MTK v0.1
	/*
	CoreISP3_I2C_Write(0xe061, 0x36);	//4 M	Div     // 54
	// This for 24Mhz MCLK  28.8MHz PCLK
	CoreISP3_I2C_Write(0xe062, 0x05);	//4 P	Div
	CoreISP3_I2C_Write(0xe061, 0x18);	//4 M	Div     // 24
	CoreISP3_I2C_Write(0xe063, 0x02);	//4 S	Div
	*/
	// For MTK ver0.2
	/*
	$I 0xe062 0x__06 #P
	$I 0xe061 0x__18 #M  
	$I 0xe063 0x__02 #S
	// 
	*/ 

	
	// This for 24Mhz MCLK  24MHz PCLK     PCLK=MCLK
	CoreISP3_I2C_Write(0xe062, 0x06);	//4 P	Div
	CoreISP3_I2C_Write(0xe061, 0x18);	//4 M	Div     // 24
	CoreISP3_I2C_Write(0xe063, 0x02);	//4 S	Div
	/*
	// This for 24Mhz MCLK  24MHz PCLK   // for 24M   -->60M, quality still not good
	CoreISP3_I2C_Write(0xe062, 0x05);	//4 P	Div
	CoreISP3_I2C_Write(0xe061, 0x32);	//4 M	Div     // 24
	CoreISP3_I2C_Write(0xe063, 0x02);	//4 S	Div
	*/
	/*
	// This for 20Mhz MCLK  48MHz PCLK   
	CoreISP3_I2C_Write(0xe062, 0x05);	//4 P	Div
	CoreISP3_I2C_Write(0xe061, 0x30);	//4 M	Div     // 24
	CoreISP3_I2C_Write(0xe063, 0x02);	//4 S	Div
	*/
	#endif
#endif

	DebugMessage(DM_JPEG, "0xe062[0x%x], 0xe061[0x%x], 0xe063[0x%x]\n", CoreISP3_I2C_Read(0xe062) , CoreISP3_I2C_Read(0xe061), CoreISP3_I2C_Read(0xe063));

	//3 Enable parallel sensor port 1 interface
	//3 Sensor Reset
	CoreISP3_I2C_Write(0xe010, 0xf0);	//4 [7] : Sensor Port 1 I/F Enable
										//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
										//4 [5] : S1_RST = 1

	//3 Delay
	WaitTime_us(10000);				//4 10ms

	CoreISP3_I2C_Write(0xe010, 0xd0);   //4 [7] : Sensor Port 1 I/F Enable
										//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
										//4 [5] : S1_RST = 0

	WaitTime_us(10000);				//4 10ms
	CoreISP3_I2C_Write(0xe010, 0xf0);	//4 [7] : Sensor Port 1 I/F Enable
										//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
										//4 [5] : S1_RST = 1

/*
6 clocks are generated in CLI6010 for internal processing from PLL output clock (GCLK). 
NO  Clock ID  Description 
1 	MCLK 	It is the system main clock. It is generated from PLL output clock (GCLK). It is the 
			fastest clock 
2 	SCLK 	Pixel processing system clock. It should be the half of MCLK 
3 	JMCLK 	Main clock of JPEG encoder. It should the same with MCLK 
4 	8051CLK 8051 main clock. The frequency should be 15MHz when flash memory is used as 
			code memory to accommodate low speed access time of flash memory. 
5 	CCLK 	Sensor data sampling clock. It is derived from one of S1_PCLK, SMCLK, and SCLK. 
6 	SMCLK 	This clock is used as sensor input clock. The frequency selection is dependant on 
			image sensor’s clock input range and sensor’s PLL configuration. It is typically the 
			same or slower than SCLK. 
addr: 0xe011
	[7:0] rw 0x01 CIS1INTB Parallel sensor interface control set B 0xe011 
	[7:7] rw 0x00 CIS1ASYNC Enable async interface. When this bit is set, sensor pixel 
	clock (S1_PCLK) and CLI6010 pixel data sampling clock 
	(CCLK) operates asynchronously. 
	[6:6] rw 0x00 CIS1SMPCKN Negative edge sampling using the selected sampling clock 
	EG  from CIS1SMPCKSEL[5:4] 
	[5:4] rw 0x00 CIS1SMPCKS Pixel data sampling clock(CCLK) selection 
	EL  3: Reserved 
	2: SCLK (internal pixel processing clock) 
	1: S1_MCLK 
	0: S1_PCLK 
	[3:0] rw 0x01 SMPLRATE CLI6010 sampling rate. These bits should be set to 
	“zero”. 

addr: 0xe012
	[7:0] rw 0x00 CIS1INTC Parallel sensor interface control set C 0xe012 
	[7:7] rw 0x00 CIS1VSNEG S1_VS is interpreted as inverted. 
	[6:6] rw 0x00 CIS1HSNEG S1_HS is interpreted as inverted. 
	[5:4] rw 0x00  Reserved 
	[3:0] rw 0x00 CIS1CLKDIV S1_MCLK clock is defined by dividing PLL GCLK clock as 
	follows. 
	0: x1, 1:x1/2, 2:x1/4, 3:x1/8, 4:x1/16 
	
addr: 0xe013 
	[7:4] rw  Cis1IntD SCLK divider value(1: x1, 2:x1/2, 4:x1/4, 8:x1/8, 16:x1/16) 
	[3:0] rw  Cis1IntD MCLK divider value(1: x1, 2:x1/2, 4:x1/4, 8:x1/8, 16:x1/16) 

	
addr: 0xe014
	[7:4] rw 0x01 Cis1IntE 8051CLK divider value(1: x1, 2:x1/2, 4:x1/4, 8:x1/8, 16:x1/16) 
	[3:0] rw    

*/
	//3 Div from GCLK : MCLK, FCLK, JMCLK
	CoreISP3_I2C_Write(0xe013, 0x21);	//4 [7:4] Cis1IntD(SCLK Divider Value)
										//4 ISP System Clock : FOUT x 1/2 = 27MHz             
										//4 [3:0] Cis1IntD(MCLK Divider Value)
										//4 ISP ISP Memory Clock : FOUT x 1 = 54MHz
	
	CoreISP3_I2C_Write(0xe015, 0x11);    //4[7:4] FClkDiv(FCLK) => 54MHz, [2:0] JMClkDiv(JMCLK) => 54MHz

	//3 Div from MCLK : SMCLK, 8051CLK, SCLK, JCLK
	CoreISP3_I2C_Write(0xe012, 0x01);    //4[2:0] Cis1ClkDiv(S1_MCLK) => 27MHz
	CoreISP3_I2C_Write(0xe014, 0x22);    //4[7:4] i8051_ClkDiv => 27MHz, [3:0] JClkDiv(JCLK) => 27MHz

	/* CCLK Setting */
	CoreISP3_I2C_Write(0xe011, 0x00);    //4[5:4] '0' =>S1_PCLK, '1' => S1_MCLK, '2' => SCLK
	WaitTime_us(10000);    // 100ms    

	/* PLL ON */
	//fang
	CoreISP3_I2C_Write(0xe060, 0x03);	//OFF: 0x03		ON:	0x0
	//WaitTime_us(500000);    // 500ms    
	return Cl_True;
}

/*********************************************************************
*Name			:	CoreISP3_LSC_TableDownLoad
*Description	:	Downloading Lens Shading Data
*Param			:	none
*return			:	Cl_True : Scuuess, Cl_False : Fail
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
#if 0
Cl_Bool CoreISP3_LSC_TableDownLoad( void )
{
	ClUint_32	data_cnt;
#if 0	
	ClUint_16	reg_data;
	ClUint_32	i;
#endif

	DebugMessage(DM_JPEG, "[CoreISP3_LSC_TableDownLoad]LSC Binary Size : :%d\n", LSC_SIZE);

	/* -LSC(Lens Shading Compension) Function disable.*/
	CoreISP3_I2C_Write(IspFenA, 0x00);

	// -LSC Size Setting
	CoreISP3_I2C_Write(ShdLMDsizeH, LSC_SIZE/256);
	CoreISP3_I2C_Write(ShdLMDsizeL, LSC_SIZE%256);

	//DebugMessage(DM_JPEG, ">>ISP2_REG>>0xe2e2==0x%x[0x%x]\n",CoreISP3_I2C_Read(ShdLMDsizeH), LSC_SIZE/256);
	//DebugMessage(DM_JPEG, ">>ISP2_REG>>0xe2e3==0x%x[0x%x]\n",CoreISP3_I2C_Read(ShdLMDsizeL), LSC_SIZE%256);
	if(CoreISP3_I2C_Read(ShdLMDsizeH)!= LSC_SIZE/256)
	{
		DebugMessage(DM_JPEG, "IIC_Write Read error...\n");
		return Cl_False;
	}
	// -Download LSC file
	DebugMessage(DM_JPEG, "LSC Binary Downloading.....>>>>>>>>>>.\n");

	for(data_cnt=0; data_cnt<LSC_SIZE; data_cnt++)
	{
		//4 0x80
		CoreISP3_I2C_Write_Bulk(ShdLMData, LSC_INIT_TABLE[data_cnt]);
	}

	DebugMessage(DM_JPEG, "LSC Binary Download End!!\n");

	CoreISP3_I2C_Write(IspFenA, 0x01);   // -LSC Function enable.

#if 0	//3 Verify LSC Data
	DebugMessage(DM_JPEG, "LSC Verify START @@@@@@@@@@@@@@@@@@@@@\n");
	for( i=0; i<LSC_SIZE; i++ ) 
	{
		//3 Set Address
		CoreISP3_I2C_Write(0xe2e0,	i>>8);		//4 Set Upper Address
		CoreISP3_I2C_Write(0xe2e1,	i & 0xFF);	//4 Set Lower Addreaa
		reg_data = CoreISP3_I2C_Read(0xe2e4);	//4 Read Data of Address
		reg_data = CoreISP3_I2C_Read(0xe2e4);	//4 Read Data of Address

		if (reg_data != LSC_INIT_TABLE[i])
		{
			DebugMessage(DM_JPEG, "LSC Verify ERROR LSC_INIT_TABLE[%d] = 0x%x : 0x%x !!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", i, reg_data, LSC_INIT_TABLE[i]);
			return Cl_False;
		}

	}
	DebugMessage(DM_JPEG, "LSC Verify END @@@@@@@@@@@@@@@@@@@@@\n");
#endif

	return Cl_True;
}
#endif
/*********************************************************************
*Name			:	ISP3_FlashromFormat
*Description	:	Format Flash of ISP3
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromFormat(void)
{
	// Flashrom Format
	CoreISP3_I2C_Write(0xE070, 0xFD);	// MCU hold
	CoreISP3_I2C_Write(0xE0C4, 0x10);
	CoreISP3_I2C_Write(0xE0A2, 0x19);	// Flashrom mode
	CoreISP3_I2C_Write(0xE0A1, 0x01);	// Format
	WaitTime_us(200000);	//Erase processing Max add@0625
}
/*********************************************************************
*Name			:	ISP3_FlashromBlockErase
*Description	:	Erase Flash of ISP3 Block
*Param			:	eraseStartAddress	Start Address of Flash
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromBlockErase(int eraseStartAddress)
{
	// Flash Block Erase
	CoreISP3_I2C_Write(0xE070, 0xFD);	// MCU Hold
	CoreISP3_I2C_Write(0xE0BA, eraseStartAddress>>8);
	CoreISP3_I2C_Write(0xE0BB, eraseStartAddress&0xFF);
	CoreISP3_I2C_Write(0xE0C4, 0x30);
	CoreISP3_I2C_Write(0xE0A1, 0x01);
	WaitTime_us(200000);	//Erase processing Max add@0625
}
/*********************************************************************
*Name			:	ISP3_FlashromWrite
*Description	:	Write to Flash of ISP3
*Param			:	writeStartAddress	Start address of Flash
					data				Point of Data
					length				Data Size
					callbackFunc		Call back function
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromWrite(int writeStartAddress, ClUint_8* data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount))
{
	int i;
	int writeEndAddress = writeStartAddress+length-1;
	// Flash Write
	CoreISP3_I2C_Write(0xE070, 0xFD);
	CoreISP3_I2C_Write(0xE0A3, writeStartAddress>>8);
	CoreISP3_I2C_Write(0xE0A4, writeStartAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A6, writeEndAddress>>8);
	CoreISP3_I2C_Write(0xE0A7, writeEndAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A2, 0x29);
	CoreISP3_I2C_Write(0xE0A2, 0x19);
	//WaitTime_us(1000*50);				//4 50 ms

	for(i=0; i<length; i++)
	{
		CoreISP3_I2C_Burst_Write(0xE0A5, data[i]);
		if(callbackFunc != Cl_Null)
		{
			callbackFunc(data, i, length);
		}
		else
		{
			//			Sleep(2);
		}
	}
}
/*********************************************************************
*Name			:	ISP3_FlashromRead
*Description	:	Read from Flash of ISP3
*Param			:	readStartAddress		Start address of Flash
					data					Save Reading data
					length					Read size
					callbackFunc			Call back function
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromRead(int readStartAddress, ClUint_8 *data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount))
{
	int i;
	// Dump
	CoreISP3_I2C_Write(0xE070, 0xFD);
	CoreISP3_I2C_Write(0xE0A2, 0x31);

	for(i=0; i<length; i++)
	{
		int addr = readStartAddress+i;
		CoreISP3_I2C_Write(0xE0A3, addr>>8);	// Address High
		CoreISP3_I2C_Write(0xE0A4, addr&0xFF);	// Address Low
		data[i] = CoreISP3_I2C_Read(0xE0A5);
		//		data[i] = clIIC_Read(0xE0A5);
		if(callbackFunc != Cl_Null)
		{
			callbackFunc(data, i, length);
		}
		else
		{
			//			Sleep(2);
		}
	}
}
/*********************************************************************
*Name			:	CoreISP3_MCU_CodeDownload
*Description	:	Downloading ISP3 LSC & Main Bin Data
*Param			:	pInitialData	Pointer of MCU Data
					InitialDataSize	Length of MCU Data
					ModeParam		enDownloadMode_StackedMem		Write to Flash
									enDownloadMode_SkipedMCUBin		Read from Flash
									enDownloadMode_CodeRAM			Write to RAM
*return			:	Cl_True : Scuuess, Cl_False : Fail
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
ClUint_8 CoreISP3_MCU_CodeDownload( ClUint_8 * pInitialData, ClUint_32 InitialDataSize, enIsp3DownloadMode ModeParam )
{
	ClUint_8 RetVal = 0;
	ClUint_16 reg_data;
	ClUint_32 i;
	ClUint_32 earseStartAddr;
	
	DebugMessage(DM_JPEG, "[CoreISP3_MCU_CodeDownload]MCU Binary Size : :%d, ModeParam : %d\n", InitialDataSize, ModeParam);

    if(ModeParam == enDownloadMode_StackedMem)
        {
        //CoreISP3_I2C_Write(FlashDEV, 0x29);   // -Set flash on
        CoreISP3_I2C_Write(FlashDEV, 0x19);   // -Set flash on
        #if 1  // for LSC calibration data
	 for(earseStartAddr=0;earseStartAddr<0xf000;earseStartAddr+=0x1000)
	 	ISP3_FlashromBlockErase(earseStartAddr);
	 #else	
        ISP3_FlashromFormat();        
	 #endif
        }
    else
        CoreISP3_I2C_Write(FlashDEV, 0x0C);   // -Set code ram on
        
	
	//WaitTime_us(200000);				//4 200 ms//Max del@0625
	ISP3_FlashromWrite(0x0000, pInitialData, InitialDataSize, Cl_Null);
	//WaitTime_us(200000);				//4 100 ms//Max del@0625


#if 0
	// Code copy    
	CoreISP3_I2C_Write(FlashDEV, 0x29);   //	# flash code mode
    if(ModeParam == enDownloadMode_StackedMem)
    {
        CoreISP3_I2C_Write(FlashDEV, 0x19);   //	# flash code mode
        WaitTime_us(100000);    // 100ms
        CoreISP3_I2C_Write(FlashDEV, 0x30);   //	# flash code copy address reset
        CoreISP3_I2C_Write(FlashDEV, 0x10);   //       # flash code copy address release
        CoreISP3_I2C_Write(FlashDSizeH, 0xc0);   //	# code ram size: 48KB
        CoreISP3_I2C_Write(FlashDSizeL, 0x00);   
        CoreISP3_I2C_Write(FlashDown, 0x01);   //	# flash code copy to code ram start
        WaitTime_us(200000);    // 200ms
        CoreISP3_I2C_Write(FlashDown, 0x00);   //	# flash code copy to code ram end
        
        CoreISP3_I2C_Write(FlashDEV, 0x00);   //	# code ram mode 
    }
    else
        CoreISP3_I2C_Write(FlashDEV, 0x00);   //Code ram mode
#endif
	printk("MCU Bin Verify START @@@@@@@@@@@@@@@@@@@@@\n");

	CoreISP3_I2C_Write(0xE070,	0x05);		//4 Reset 8051
	WaitTime_us(100000);				//4 100 ms

	if(ModeParam == enDownloadMode_CodeRAM)
		CoreISP3_I2C_Write(0xE0A2,	0xa0);		//4 0x31 : Select Flash memory , 0xA0 : Select SDRAM
	else
		CoreISP3_I2C_Write(0xE0A2,	0x31);

	WaitTime_us(100000);				//4 100 ms

	//fang
	i = InitialDataSize - 1;
	CoreISP3_I2C_Write(0xE0A3,	i>>8);		//4 Set Upper Address
	CoreISP3_I2C_Write(0xE0A4, i & 0xFF);	//4 Set Lower Addreaa
	reg_data = CoreISP3_I2C_Read(0xE0A5);	//4 Read Data of Address

	if (reg_data != pInitialData[i])
	{
		printk("MCU Bin Verify ERROR pInitialData[%d] = 0x%x : 0x%x @@@@@@@@@@@@@@@@@@@@@\n", i, reg_data, pInitialData[i]);
		return Cl_False;
	}

	for( i=0; i<InitialDataSize; i+=100 ) 
	{
		CoreISP3_I2C_Write(0xE0A3,	i>>8);		//4 Set Upper Address
		CoreISP3_I2C_Write(0xE0A4, i & 0xFF);	//4 Set Lower Addreaa
		reg_data = CoreISP3_I2C_Read(0xE0A5);	//4 Read Data of Address

		if (reg_data != pInitialData[i])
		{
			printk("MCU Bin Verify ERROR pInitialData[%d] = 0x%x : 0x%x @@@@@@@@@@@@@@@@@@@@@\n", i, reg_data, pInitialData[i]);
			return Cl_False;
		}

	}
	printk("MCU Bin Verify END @@@@@@@@@@@@@@@@@@@@@\n");	

	return 1;
}

/*********************************************************************
*Name			:	CoreISP3_Initialize
*Description	:	Main ISP3 Initialize Routine
*Param			:	param	enDownloadMode_StackedMem		Write to Flash
							enDownloadMode_SkipedMCUBin		Read from Flash
							enDownloadMode_CodeRAM			Write to RAM
*return			:	Cl_True : Scuuess, Cl_False : Fail
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
Cl_Bool CoreISP3_Initialize( enIsp3DownloadMode param)
{
	Cl_Bool RetVal = Cl_True;
	
	//ClUint_32 *ISP3BinaryDataMcu;
	//ClUint_32 ISP3BinarySize = 0;
	DebugMessage(DM_JPEG, "=========================== CoreISP3_Initialize : START ===========================\n");
	DebugMessage(DM_JPEG, "enIsp3DownloadMode : %d\n", param);

	/* ISP Clock Setting */
	if(CoreISP3_SetClock()==Cl_False)
		return Cl_False;

#if 0	
	// LSC Table Download.
	RetVal = CoreISP3_LSC_TableDownLoad();
	if (RetVal == Cl_False)
	{
		DebugMessage(DM_JPEG, "CoreISP3_LSC_TableDownLoad Fail Error : %d \n", RetVal);
		return Cl_False;
	}
#endif

	/* MCU Binary Download */
	if(param != enDownloadMode_SkipedMCUBin)
	{
#ifdef ISP3_MCU_ARRAY  // for .h file 
		RetVal = CoreISP3_MCU_CodeDownload((ClUint_8*)MCU_BIN, sizeof(MCU_BIN), param);
#else	
		RetVal = CoreISP3_MCU_CodeDownload((ClUint_8*)&ISP3BinaryDataMcu, ISP3BinarySize, param);
#endif
		if (RetVal == Cl_False)
		{
			DebugMessage(DM_JPEG, "CoreISP3_MCU_CodeDownload Fail Error : %d \n", RetVal);
			return Cl_False;
		}
#ifdef DOWN_TIME_CHECK
		DebugMessage(DM_JPEG, "enIsp3Download finish time : %d\n", TIMER_curTime_ms(1));
#endif
		
	}

	/* MCU Reset */
	CoreISP3_I2C_Write(MCURST, 0x05);
	WaitTime_us(10000);    // 100ms  // Jacky change 10ms for init speed
	CoreISP3_I2C_Write(MCURST, 0x04);
	WaitTime_us(10000);    // 100ms	// Jacky change 10ms for init speed

	/* Send Command(ISP & Sensor Initialize) to ISP */
	DebugMessage(DM_JPEG, "ISP3_CMD_SENSOR_INIT \n");
	CoreISP3_Send_Command(ISP3_CMD_SENSOR_INIT);
	DebugMessage(DM_JPEG, "ISP3_CMD_SENSOR_INIT end\n");

	/*Sensor Interface*/
	CoreISP3_I2C_Write(SMIABypass, 0x00);   //[7:7] LMBYPASS '0' = Async, '1' = Sync

	/* OUT FORMAT */
	CoreISP3_I2C_Write(OutFmt_JPG, 0x82);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG


//ZhouCS rotate 180 degree. ok
#if 0
//#if defined(FEATURE_CS7903) 

	/*颜色偏蓝
	CoreISP3_I2C_Write(0xE033, 0x30);   
	CoreISP3_I2C_Write(0xE034, 0x40);   
	CoreISP3_I2C_Write(0xE035, 0xc0);   
	CoreISP3_I2C_Write(0xE036, 0x41);   
	*/
	CoreISP3_I2C_Write(0xE033, 0x30);  
	CoreISP3_I2C_Write(0xE034, 0x40);
	CoreISP3_I2C_Write(0xE035, 0xc0);    // before setting is 0xc041, please change to 0xc0c3 
	CoreISP3_I2C_Write(0xE036, 0xc3);
	// ISP3 raw data  pixel director  
	CoreISP3_I2C_Write( 0xe300, 0x02 );  // Add this reg for color issue.
       DebugMessage(DM_JPEG, "rotate 180 degree ok!\n");
#endif	


//end

#ifdef DOWN_TIME_CHECK
		DebugMessage(DM_JPEG, "Initialize end time : %d\n", TIMER_curTime_ms(1));
#endif
	DebugMessage(DM_JPEG, "=========================== CoreISP3_Initialize : END ===========================\n");

	if(param != enDownloadMode_StackedMem)
		RetVal = CoreISP3_Get_Version();
	//printk("@@@0xE050 = 0x%x\n",CoreISP3_I2C_Read(0xE050));
	CORE_ISP_YUV_Swap(enISP_CbYCr);
//	CoreISP3_FaceTracking_On(); //test ZhouCS
	//CORE_ISP_TestPatten();  // Jacky for test

	//fang
	CoreISP3_I2C_Write(0xe061,0x56);
	CoreISP3_I2C_Write(0xe060,0x00);

//	 DebugMessage(DM_JPEG, "clock  %x  %x %x \n", CoreISP3_I2C_Read(0xe061),  CoreISP3_I2C_Read(0xe062), CoreISP3_I2C_Read(0xe063));

	#if 0  // if AP do not invert PCLK, you can try change following code
	{
	ClSint_16 reg_data = 0;
	//#########################################
	//#  VBALNK Pulse Width Expanded Setting
	//##########################################

//	reg_data = CoreISP3_I2C_Read(OutFmt);		// rising edge of pclk
//	CoreISP3_I2C_Write(OutFmt, (reg_data | 0x20));
//	DebugMessage(DM_JPEG, "reg_data 0x%x\n", reg_data);


//	reg_data = CoreISP3_I2C_Read(OutFmt);		// vsynv high active
//	CoreISP3_I2C_Write(OutFmt, (reg_data | 0x80));
//	DebugMessage(DM_JPEG, "reg_data 0x%x\n", reg_data);
	
	/*
	reg_data = CoreISP3_I2C_Read(OutFmt);		// Y First
	CoreISP3_I2C_Write(OutFmt, (reg_data & 0xfd)|0x01);
	*/
	
	}	
	#endif
	//fang
	//CoreISP3_I2C_Write(0xe015, 0x11);	//double clock
	//CoreISP3_I2C_Write(0xe051, 0x04);	//disable cut clock
	
	return RetVal;
}

//fang
/*	Name:		CLI6000_download_firmware
 *
 *  Param: 		mode 1 	force to download firmware
 *  			mode 0 	need to downlaod firmware
 *
 *  Return	 	0:	not succeed
 *				1: 	succeed
 *				2: 	already update
 */
int CLI6000_download_firmware(int mode)
{
	int status = 1;
	if(1 == mode)
		status = CoreISP3_Initialize(enDownloadMode_StackedMem);
	else if(0 == mode)
		status = CoreISP3_Initialize(enDownloadMode_SkipedMCUBin);
	else
		printk("===>%s: mode%d \n", __FUNCTION__, mode);


	if(0 == status)
	{
		printk("===>%s: mode%d update no succeed\n", __FUNCTION__, mode);
	}
	else if(1 == status)
	{
		printk("===>%s: mode%d update succeed\n", __FUNCTION__, mode);
	}
	else if(2 == status)
	{
		printk("===>%s: mode%d already update\n", __FUNCTION__, mode);
	}
	else
	{
		printk("===>%s: mode%d return no found\n", __FUNCTION__, mode);
	}
	return status;
}

/*********************************************************************
*Name			:	CoreISP3_SetResolution
*Description	:	Set Preview Resolution of ISP3
*Param			:	OutResolution	enISP_RES_5MP_FULL		// 2560 x 1920
									enISP_RES_QXGA			// 2048 x 1536
									enISP_RES_UXGA			// 1600 x 1200
									enISP_RES_SXGA			// 1280 x 1024
									enISP_RES_1_3MP			// 1280 x 960
									enISP_RES_XGA			// 1024 x 768
									enISP_RES_SVGA			// 800 x 600
									enISP_RES_VGA			// 640 x 480
									enISP_RES_QVGA			// 320 x 240
									enISP_RES_None
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetResolution( enIsp3OutResolution OutResolution , CMD_Mode Mode )
{
    //return;  // Jacky for test patten test
    DebugMessage(DM_JPEG, "CoreISP3_SetResolution New: %d, mode: %d g_zoom_rate=%d\n", OutResolution, Mode, g_zoom_rate);
    
    switch(OutResolution)
    {
        case enISP_RES_5MP_FULL:    // 2560 x 1920
           //if(Mode == CMD_Capture)  // Jacky add this, can not add this.. now sensor output is 1.3M
           	{
	            CoreISP3_I2C_Write(0xE6A8, 0x00);      
	            CoreISP3_Send_Command(ISP3_CMD_CAPTURE);  

			CoreISP3_I2C_Write(JpgOutMode, 0x26);      
			#if 1
			if(g_zoom_rate!=0)
				ISP3_ImageCrop(2560, 1920, 2560-128*g_zoom_rate, 1920-96*g_zoom_rate, 2560, 1920);
			#else
			switch(g_zoom_rate)
			{
				case enISP_ZOOM_480_360X2: // 1.33
					ISP3_ImageCrop(2560, 1920, 2048, 1536, 2560, 1920);
					break;
				case enISP_ZOOM_480_360X3:	// 1.6
					ISP3_ImageCrop(2560, 1920, 1600, 1200, 2560, 1920);
					break;
				case enISP_ZOOM_480_360X4: // 2.0
					ISP3_ImageCrop(2560, 1920, 1280, 1024, 2560, 1920);	//get red color picture sometimes.
											// 1.78
					//ISP3_ImageCrop(2560, 1920, 1440, 1080, 2560, 1920);
					break;
				case enISP_ZOOM_480_360: // 1.0
				default:
					//ISP3_ImageCrop(2560, 1920, 2560, 1920, 2560, 1920);
					break;					
 			}	
 			#endif
           	}
	   /*else  // 5M no this
	    	{  
	            	CoreISP3_Send_Command(ISP3_CMD_SET_5MP_FULL_SIZE);  //only for preview     
			//CoreISP3_Send_Command(ISP3_CMD_PREVIEW);   // this will make preview fail
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);   // this can make 5M preview
	    	}*/
		break;
   
        case enISP_RES_QXGA:    // 2048 x 1536
           if(Mode == CMD_Capture)    // Jacky for test
            	{
			CoreISP3_I2C_Write(0xE6A8, 0x01);      
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE); 

			CoreISP3_I2C_Write(JpgOutMode, 0x26);      

			#if 1
			if(g_zoom_rate!=0)
				ISP3_ImageCrop(2560, 1920, 2560-128*g_zoom_rate, 1920-96*g_zoom_rate, 2048, 1536);
			#else
			switch(g_zoom_rate)
			{
				case enISP_ZOOM_480_360X2: // 1.33
					ISP3_ImageCrop(2560, 1920, 1920, 1440, 2048, 1536);
					break;
				case enISP_ZOOM_480_360X3:	// 1.6
					ISP3_ImageCrop(2560, 1920, 1600, 1200, 2048, 1536);
					break;
				case enISP_ZOOM_480_360X4: // 2.0
					ISP3_ImageCrop(2560, 1920, 1280, 960, 2048, 1536);	//get red color picture sometimes.
											// 1.78
					//ISP3_ImageCrop(2560, 1920, 1440, 1080, 2048, 1536);

					break;
				case enISP_ZOOM_480_360: // 1.0
				default:
					//ISP3_ImageCrop(2560, 1920, 2560, 1920, 2048, 1536);
					break;					
 			}	
 			#endif
           	}
            else   
            	{
			CoreISP3_Send_Command(ISP3_CMD_SET_3MP_SIZE);  //only for preview     
			CoreISP3_Send_Command(ISP3_CMD_PREVIEW);   
            	}
            break;            

        case enISP_RES_UXGA:     // 1600 x 1200
			CoreISP3_I2C_Write(0xE6A8, 0x02);      
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);   

			CoreISP3_I2C_Write(JpgOutMode, 0x26);      

			#if 1
			if(g_zoom_rate!=0)
				ISP3_ImageCrop(2560, 1920, 2560-128*g_zoom_rate, 1920-96*g_zoom_rate, 1600, 1200);
			#else
			switch(g_zoom_rate)
			{
				case enISP_ZOOM_480_360X2: // 1.33
					ISP3_ImageCrop(2560, 1920, 1920, 1440, 1600, 1200);
					break;
				case enISP_ZOOM_480_360X3:	// 1.6
					ISP3_ImageCrop(2560, 1920, 1600, 1200, 1600, 1200);
					break;
				case enISP_ZOOM_480_360X4: // 2.0
					ISP3_ImageCrop(2560, 1920, 1280, 960, 1600, 1200);//get red color picture sometimes.
											// 1.78
					//ISP3_ImageCrop(2560, 1920, 1440, 1080, 1600, 1200);
					break;
				case enISP_ZOOM_480_360: // 1.0
				default:
					//ISP3_ImageCrop(2560, 1920, 2560, 1920, 1600, 1200);
					break;					
 			}			
 			#endif
            break;
  
        case enISP_RES_1_3MP:   // 1280 x 960
           if(Mode == CMD_Capture)  
           {
	    		CoreISP3_I2C_Write(0xE6A8, 0x03);      
	    		CoreISP3_Send_Command(ISP3_CMD_CAPTURE);

			CoreISP3_I2C_Write(JpgOutMode, 0x26);      

			#if 1
			if(g_zoom_rate!=0)
				ISP3_ImageCrop(2560, 1920, 2560-128*g_zoom_rate, 1920-96*g_zoom_rate, 1280, 960);
			#else
			switch(g_zoom_rate)
			{
				case enISP_ZOOM_480_360X2: // 1.33
					ISP3_ImageCrop(2560, 1920, 1920, 1440, 1280, 960);
					break;
				case enISP_ZOOM_480_360X3:	// 1.6
					ISP3_ImageCrop(2560, 1920, 1600, 1200, 1280, 960);
					break;
				case enISP_ZOOM_480_360X4: // 2.0
					ISP3_ImageCrop(2560, 1920, 1280, 960, 1280, 960);//get red color picture sometimes.
											// 1.78
					//ISP3_ImageCrop(2560, 1920, 1440, 1080, 1280, 960);					
					break;
				case enISP_ZOOM_480_360: // 1.0
				default:
					//ISP3_ImageCrop(2560, 1920, 2560, 1920, 1280, 960);
					break;					
 			}		
 			#endif
           }
            else   
            	{
			//CoreISP3_Send_Command(ISP3_CMD_SET_13MP_SIZE);  //only for preview     
	    		CoreISP3_I2C_Write(0xE6A8, 0x03);      
			CoreISP3_Send_Command(ISP3_CMD_PREVIEW);   
            	}
            break;
            
        case enISP_RES_XGA:     // 1024 x  768
			CoreISP3_I2C_Write(0xE6A8, 0x04);      
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
            break;
            
        case enISP_RES_SVGA:    // 800 x  600
           if(Mode == CMD_Capture)  // Jacky add this
           	{
	            	CoreISP3_I2C_Write(0xE6A8, 0x05);      
	            	CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
           	}
	    else
	    	{
	            	//CoreISP3_Send_Command(ISP3_CMD_SET_SVGA_SIZE);  //only for preview     
	            	CoreISP3_I2C_Write(0xE6A8, 0x05);      
	            	CoreISP3_Send_Command(ISP3_CMD_PREVIEW);       // Added by Jacky
	    	}	
            break;
            
        case enISP_RES_VGA:     // 640 x  480
           if(Mode == CMD_Capture)  // Jacky change this
            {
            		CoreISP3_I2C_Write(0xE6A8, 0x06);      
            		CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    

			CoreISP3_I2C_Write(JpgOutMode, 0x26);      

			#if 1
			if(g_zoom_rate!=0)
				ISP3_ImageCrop(2560, 1920, 2560-128*g_zoom_rate, 1920-96*g_zoom_rate, 640, 480);
			#else
			switch(g_zoom_rate)
			{
				case enISP_ZOOM_480_360X2: // 1.33
					ISP3_ImageCrop(2560, 1920, 1920, 1440, 640, 480);
					break;
				case enISP_ZOOM_480_360X3:	// 1.6
					ISP3_ImageCrop(2560, 1920, 1600, 1200, 640, 480);
					break;
				case enISP_ZOOM_480_360X4: // 2.0
					ISP3_ImageCrop(2560, 1920, 1280, 960, 640, 480);//get red color picture sometimes.
											// 1.78
					//ISP3_ImageCrop(2560, 1920, 1440, 1080, 640, 480);										
					break;
				case enISP_ZOOM_480_360: // 1.0
				default:
					//ISP3_ImageCrop(2560, 1920, 2560, 1920, 640, 480);
					break;					
 			}				
			#endif		
            }
            else 
            	{
	            	//CoreISP3_Send_Command(ISP3_CMD_SET_VGA_SIZE);   
            		CoreISP3_I2C_Write(0xE6A8, 0x06);      
	            	CoreISP3_Send_Command(ISP3_CMD_PREVIEW);       // Added by Jacky
            	}
            break;
            
        case enISP_RES_QVGA:    // 320 x  240
           if(Mode == CMD_Capture)  // Jacky change this
            {
        	CoreISP3_I2C_Write(0xE6A8, 0x07);      
            	CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
            }
            else 
            	{
	            	//CoreISP3_Send_Command(ISP3_CMD_SET_VGA_SIZE);   
        	CoreISP3_I2C_Write(0xE6A8, 0x07);      
            	CoreISP3_Send_Command(ISP3_CMD_PREVIEW);    
            	}		
            break;
            
        default:
            break;
    }  

//ZhouCS rotate 180 degree. ok
#if 0
//#if defined(FEATURE_CS7903) 
	if(Mode == CMD_Capture) 
	{
		// 0: Gr  1: Gb  2: Rg  3: Bg 
		CoreISP3_I2C_Write(0xe300, 0x03);    // 0,1,2 fail    3 for capture test ok
	}
	else
	{
		CoreISP3_I2C_Write(0xE033, 0x30);  // 0x301d    0x0101
		CoreISP3_I2C_Write(0xE034, 0x40);
		CoreISP3_I2C_Write(0xE035, 0xc0);   // Already test ok.. but collor is incorrect
		CoreISP3_I2C_Write(0xE036, 0xc3);
		WaitTime_us(300);
		CoreISP3_I2C_Write(0xe300, 0x02);
		DebugMessage(DM_JPEG, "rotate 180 degree 2 ok!\n");

	}
#endif
	
    DebugMessage(DM_JPEG, "resolution clock  %x  %x %x \n", CoreISP3_I2C_Read(0xe061),  CoreISP3_I2C_Read(0xe062), CoreISP3_I2C_Read(0xe063));
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoWhiteBalance
*Description	:	Set AWB
*Param			:	AWB_Param		ON
									OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoWhiteBalance( enIsp3FunctionsAWB AWB_Param )
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, AWB_Param);

	if(AWB_Param == enISP_FUNC_AWB_ON)
	{
		/* Send command to CoreISP3_Send_CommandISP MCU */   
		CoreISP3_Send_Command(ISP3_CMD_AWB_ON);
	}
	else //AWB_Param == enISP_FUNC_AWB_OFF
	{
		/* Send command to ISP MCU */    
		CoreISP3_Send_Command(ISP3_CMD_AWB_OFF);
	}
	/* Delay Time : 100ms */
	//WaitTime_us(100000);
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoExposure
*Description	:	Set AE
*Param			:	AE_Param		ON
									OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoExposure( enIsp3FunctionsAE AE_Param )
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, AE_Param);

	if(AE_Param == enISP_FUNC_AE_ON)
	{
		/* Send command to ISP MCU */
		CoreISP3_Send_Command(ISP3_CMD_AE_ON);
	}
	else //AE_Param == enISP_FUNC_AE_OFF
	{
		/* Send command to ISP MCU */
		CoreISP3_Send_Command(ISP3_CMD_AE_OFF);
	}
	/* Delay Time : 100ms */
	//WaitTime_us(100000);
}

void CoreISP3_Set_AE_EV(ClUint_16 level)  // 0~8   for -4 leve to +4 level
{
	DebugMessage(DM_JPEG, "\n %s %d \n",__func__, level);

	level *= 16;
	level &= 0xFF;

	CoreISP3_I2C_Write(0xE62A, 0x00);//EV_Y mode
	CoreISP3_I2C_Write(0xE628, (ClUint_8)level);//value
	CoreISP3_Send_Command(ISP3_CMD_SET_AE_CONFIG); 
}

/**
 * \brief Get current Focus motor position
 * \ingroup GROUP_AUTOFOCUS
 * \remarks
 * \return
 * Current focus motor Position
 * \author Lee Suk-Joo
 * \date 2008-08-25
 */
signed short ISP3_GetFocusPos(void) 
{
	signed short focusPos;
	CoreISP3_I2C_Write (0xE62A, 0);//FOCUS_POSITION=0	
	CoreISP3_Send_Command (ISP3_CMD_GET_AF_CONFIG);
	focusPos = (CoreISP3_I2C_Read(0xE628)<<8 | CoreISP3_I2C_Read(0xE629));
	return focusPos;
}

/**
 * \brief Set Focus motor position
 * \ingroup GROUP_AUTOFOCUS
 * \remarks
 * \return
 * void
 * \author Lee Suk-Joo
 * \date 2008-08-25
 */
void ISP3_SetFocusPos(signed short focusPos) 
{
	DebugMessage(DM_JPEG, "ISP3_SetFocusPos: %d\n", focusPos);
	CoreISP3_I2C_Write (0xE628, (focusPos>>8)&0xFF);
	CoreISP3_I2C_Write (0xE629, focusPos&0xFF);
	CoreISP3_I2C_Write (0xE62A, 0);//FOCUS_POSITION=0	
	CoreISP3_Send_Command (ISP3_CMD_SET_AF_CONFIG);
}


/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus
*Description	:	Set AF
*Param			:	AF_Param		ON
									OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoFocus( enIsp3FunctionsAF AF_Param )
{
	ClUint_16 af_value = 0;
	ClUint_8 loop = 40;//最大等待4S
	
	DebugMessage(DM_JPEG, "====> %s::%d\n", __func__, AF_Param);

	if(AF_Param == enISP_FUNC_AF_ON)
	{
		//取消对焦时的辅助闪光，因borqs软件在拍照退出进预览时还会调用AutoFocus,
		//会导致闪光
		//if(g_isFlash==Cl_True)
		//{		
		//	sensor_flash_ctl(Cl_True); //open the focus flashlight
		//}
		/* Send command to ISP MCU */
		CoreISP3_Send_Command(ISP3_CMD_FULLAF);
		//CoreISP3_Send_Command(ISP3_CMD_AF_ON);
		do
		{
			if((af_value=CoreISP3_I2C_Read(0xE661))!=0x23)
			{
				DebugMessage(DM_JPEG, " CoreISP3 set Focus success!val=0x%x,loop=%d\n",af_value,40-loop);
				break;
			}
			WaitTime_us(100000);    // 100ms		
		}while(--loop);
		
		if(loop==0)
			DebugMessage(DM_JPEG, " CoreISP3 set Focus timeout!val=0x%x \n",af_value);

		//if(g_isFlash==Cl_True)
		//{
		//	sensor_flash_ctl(Cl_False); //close the focus flashlight
		//}

	}
	else //AF_Param == enISP_FUNC_AF_OFF
	{
		/* Send command to ISP MCU */
		CoreISP3_Send_Command(ISP3_CMD_AF_OFF);
	}	

//	CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
//	CoreISP3_SetAutoExposure(enISP_FUNC_AE_ON);
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus_FullScan
*Description	:		Set AF
*Param			:	void
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoFocus_FullScan(void)
{
	DebugMessage(DM_JPEG, "\n====> %s::\n", __func__);

	/* Send command to ISP MCU */
	CoreISP3_Send_Command(ISP3_CMD_AF_FULL_SCAN);
	
}

/*
2. JPEG EXIF Information.
 -> ISO Value :
    0xE649[7:0] = ISO Gain 
    calculate =>  ( ISO Gain / 8 ) x 100 = ISO Value

*/
/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus_FullScan
*Description	:		Set AF
*Param			:	void
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
ClUint_32 CoreISP3_GetISOGain(void)
{
	ClUint_32 gain;
	ClUint_16 val;

	val = CoreISP3_I2C_Read(0xE649);
	
	gain = val/8 *100;
	DebugMessage(DM_JPEG, "\n====> vol: [%x]:: Gain: [%x] \n", val, gain);
		

	return gain;
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus_FullScan
*Description	:		Set AF
*Param			:	void
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
 -> Shutter Speed : 
      0xE64A [7:0] = Shutter Speed high byte, 0xE64b [7:0] = Shutter Speed  low byte, 0xE663[7:0] = Hsync Time.
    calculate= >  Shutter Speed ( high byte, low byte) x Hsync Time = Shutter Speed [ uS ]
 
**********************************************************************/
ClUint_32 CoreISP3_GetShutterSpeed(void)
{
	ClUint_32 speed;
	ClUint_16 high;
	ClUint_16 low;
	ClUint_16 Hsync_time;
	
	DebugMessage(DM_JPEG, "\n====> %s::\n", __func__);

	high = CoreISP3_I2C_Read(0xE64A);
	low = CoreISP3_I2C_Read(0xE64B);
	Hsync_time = CoreISP3_I2C_Read(0xE663);
	
	DebugMessage(DM_JPEG, "\n====> %x  %x  %x::\n", high, low, Hsync_time);
	
	speed = (((high&0xff)<<8)|(low&0xff))*Hsync_time;
	

	return speed;
}

/*********************************************************************
*Name			:	CoreISP3_SetANR
*Description	:	Set ANR
*Param			:	ANR_Param		enISP_FUNC_ANR_LEVEL_0
									enISP_FUNC_ANR_LEVEL_1
									enISP_FUNC_ANR_LEVEL_2
									enISP_FUNC_ANR_LEVEL_3
									enISP_FUNC_ANR_LEVEL_4
*return			:	none
*Author			:	
*Remark			:	enISP_FUNC_ANR_LEVEL_0 is ANR OFF.
*Log			:	
**********************************************************************/
void CoreISP3_SetANR( enISP3FunctionsANRLevel ANR_Param )
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, ANR_Param);

	/* Set Adative Noise Reduction Level */
	switch(ANR_Param)
	{
		case enISP_FUNC_ANR_LEVEL_0:    /*Auto*/
			CoreISP3_I2C_Write(0xe671, 0x4);
			CoreISP3_Send_Command(ISP3_CMD_ANR_OFF);
			break;
		case enISP_FUNC_ANR_LEVEL_1:
		case enISP_FUNC_ANR_LEVEL_2:
		case enISP_FUNC_ANR_LEVEL_3:
		case enISP_FUNC_ANR_LEVEL_4:
			CoreISP3_I2C_Write(0xe671, ANR_Param);
			break;
	}
	/* Send command to ISP MCU */
	CoreISP3_Send_Command(ISP3_CMD_ANR_ON);

}
/*********************************************************************
*Name			:	CoreISP3_SetWDR
*Description	:	Set WDR
*Param			:	stIspWdrCtrl		bWDREnable					WDR ON/OFF
										enISP_FUNC_WDR_LEVEL_0
									    enISP_FUNC_WDR_LEVEL_1
									    enISP_FUNC_WDR_LEVEL_2
									    enISP_FUNC_WDR_LEVEL_3
									    enISP_FUNC_WDR_LEVEL_4
									    enISP_FUNC_WDR_LEVEL_5
									    enISP_FUNC_WDR_LEVEL_6
									    enISP_FUNC_WDR_LEVEL_7
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetWDR( _tISP_WDR_CTRL *stIspWdrCtrl )
{
	DebugMessage(DM_JPEG, "\n====> %s::%d,%d\n", __func__, stIspWdrCtrl->bWDREnable, stIspWdrCtrl->WDRlevel);

	if(stIspWdrCtrl->bWDREnable)
	{
		/* Set WDR Level */
		CoreISP3_I2C_Partial_Write(0xe006, 7, 5, stIspWdrCtrl->WDRlevel);

		/* WDR Enable */
		CoreISP3_I2C_Partial_Write(0xe006, 4, 4, stIspWdrCtrl->bWDREnable);
	}
	else
	{
		/* WDR Disable */
		CoreISP3_I2C_Partial_Write(0xe006, 4, 4, stIspWdrCtrl->bWDREnable);
	}
	/* Delay Time : 100ms */


}

/*********************************************************************
*Name			:	CoreISP3_StillStabilizerLevelSetup
*Description	:	Set Stabilizer's Level
*Param			:	enISP_FUNC_SS_LEVEL_1
				    enISP_FUNC_SS_LEVEL_1_2
				    enISP_FUNC_SS_LEVEL_1_3
				    enISP_FUNC_SS_LEVEL_1_4
				    enISP_FUNC_SS_LEVEL_1_5
				    enISP_FUNC_SS_LEVEL_1_6
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_StillStabilizerLevelSetup( ClUint_16 LevelParam )
{
    DebugMessage(0x08000000, "\n====> %s::%d\n", __func__, LevelParam);
     
    switch(LevelParam)
    {
        case enISP_FUNC_SS_LEVEL_NONE:
        case enISP_FUNC_SS_LEVEL_1:
            break;            
        case enISP_FUNC_SS_LEVEL_1_2:
        case enISP_FUNC_SS_LEVEL_1_3:
        case enISP_FUNC_SS_LEVEL_1_4:
        case enISP_FUNC_SS_LEVEL_1_5:
        case enISP_FUNC_SS_LEVEL_1_6:
            CoreISP3_I2C_Write(0xe670, LevelParam);
            CoreISP3_I2C_Write(0xe671, 0x00);
            break;
    }
}

/*********************************************************************
*Name			:	CoreISP3_SetStillStabilizer
*Description	:	Set Stabilizer
*Param			:	stIspStillStabilizerCtrlParam		bStillStabilizerEnable		Stabilizer ON/OFF
													    enISP_FUNC_SS_LEVEL_1
													    enISP_FUNC_SS_LEVEL_1_2
													    enISP_FUNC_SS_LEVEL_1_3
													    enISP_FUNC_SS_LEVEL_1_4
													    enISP_FUNC_SS_LEVEL_1_5
													    enISP_FUNC_SS_LEVEL_1_6
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetStillStabilizer( _tIspStillStabilizerCtrl *stIspStillStabilizerCtrlParam )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d,%d\n", __func__, stIspStillStabilizerCtrlParam->bStillStabilizerEnable, stIspStillStabilizerCtrlParam->StillStabilzerLevel);

    if(stIspStillStabilizerCtrlParam->bStillStabilizerEnable)
    {
        /* Still stabilizer enable */
        CoreISP3_I2C_Write(0xe004, 0x2b);

        /* Set the SS level */
        CoreISP3_StillStabilizerLevelSetup(stIspStillStabilizerCtrlParam->StillStabilzerLevel);

        /* Send command to ISP MCU : Still Stabilizer Enable */
        CoreISP3_Send_Command(ISP3_CMD_STILL_ON);
    }
    else
    {
        /* Still stabilizer disable */
        CoreISP3_I2C_Write(0xe004, 0x22);

        /* Send command to ISP MCU : Still Stabilizer Disable */
        CoreISP3_Send_Command(ISP3_CMD_STILL_OFF);
    }
}
/*********************************************************************
*Name			:	CoreISP3_SetStillStabilizer
*Description	:	Set Stabilizer
*Param			:	enISP_FUNC_IMAGE_NORMAL
				    enISP_FUNC_IMAGE_EMBOSS
				    enISP_FUNC_IMAGE_SKETCH1
				    enISP_FUNC_IMAGE_SKETCH2
				    enISP_FUNC_IMAGE_BLACK_WHITE
				    enISP_FUNC_IMAGE_NORMAL_MOVIE
				    enISP_FUNC_IMAGE_OLD_MOVIE
				    enISP_FUNC_IMAGE_GRAY
				    enISP_FUNC_IMAGE_ACCENT
				    enISP_FUNC_IMAGE_SWAPING
				    enISP_FUNC_IMAGE_ACCENT_SWAPING
				    enISP_FUNC_IMAGE_WARM
				    enISP_FUNC_IMAGE_COOL
				    enISP_FUNC_IMAGE_FOG
				    enISP_FUNC_IMAGE_OPPOSITE_NEGATIVE
				    enISP_FUNC_IMAGE_OPPOSITE_AVERAGE
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetImageEffect( enISPFunctionsImageEffect IspImageEffectParam )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, IspImageEffectParam);

    switch(IspImageEffectParam)
    {
        case enISP_FUNC_IMAGE_NORMAL:
            CoreISP3_I2C_Write(ImgEffectA, 0x02);
            CoreISP3_I2C_Write(ImgEffectB, 0x00);
            CoreISP3_I2C_Write(ImgEffectC, 0x00);            
            break;
        case enISP_FUNC_IMAGE_EMBOSS:
            CoreISP3_I2C_Write(ImgEffectC, 0x01);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On
            break;            
        case enISP_FUNC_IMAGE_SKETCH1:
            CoreISP3_I2C_Write(ImgEffectC, 0x02);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect Ons            
            break;            
        case enISP_FUNC_IMAGE_SKETCH2:
            CoreISP3_I2C_Write(ImgEffectC, 0x03);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_BLACK_WHITE:
            CoreISP3_I2C_Write(ImgEffectC, 0x04);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_NORMAL_MOVIE:
            CoreISP3_I2C_Write(ImgEffectC, 0x05);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On
            break;            
        case enISP_FUNC_IMAGE_OLD_MOVIE:
	#if 0
		//hardware control
            CoreISP3_I2C_Write(ImgEffectC, 0x06);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
	#else
		//software control
		CoreISP3_I2C_Write(Swp_Cb_Min1, 0x00);
		CoreISP3_I2C_Write(Swp_Cb_Max1, 0xff);
		CoreISP3_I2C_Write(Swp_Cr_Min1, 0x00);
		CoreISP3_I2C_Write(Swp_Cr_Max1, 0xff);
		CoreISP3_I2C_Write(Swp_Cb1, 0x68);
		CoreISP3_I2C_Write(Swp_Cr1, 0x94);
		CoreISP3_I2C_Write(Swp_Cb_Min2, 0xff);
		CoreISP3_I2C_Write(Swp_Cb_Max2, 0x00);
		CoreISP3_I2C_Write(Swp_Cr_Min2, 0xff);
		CoreISP3_I2C_Write(Swp_Cr_Max2, 0x00);
		CoreISP3_I2C_Write(Swp_Cb2, 0x00);
		CoreISP3_I2C_Write(Swp_Cr2, 0x00);     
		CoreISP3_I2C_Write(ImgEffectC, 0x09);
		CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On
	#endif
            break;
        case enISP_FUNC_IMAGE_GRAY:
            CoreISP3_I2C_Write(ImgEffectC, 0x07);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;
        case enISP_FUNC_IMAGE_ACCENT:
            CoreISP3_I2C_Write(Acc_Cb_Min, 0x00);
            CoreISP3_I2C_Write(Acc_Cb_Max, 0xff);
            CoreISP3_I2C_Write(Acc_Cr_Min, 0x00);
            CoreISP3_I2C_Write(Acc_Cr_Max, 0x60);
            CoreISP3_I2C_Write(ImgEffectC, 0x08);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_SWAPING:
            CoreISP3_I2C_Write(Swp_Cb_Min1, 0x80);
            CoreISP3_I2C_Write(Swp_Cb_Max1, 0xff);
            CoreISP3_I2C_Write(Swp_Cr_Min1, 0x20);
            CoreISP3_I2C_Write(Swp_Cr_Max1, 0x60);
            CoreISP3_I2C_Write(Swp_Cb1, 0x00);
            CoreISP3_I2C_Write(Swp_Cr1, 0xff);
            CoreISP3_I2C_Write(Swp_Cb_Min2, 0x20);
            CoreISP3_I2C_Write(Swp_Cb_Max2, 0x60);
            CoreISP3_I2C_Write(Swp_Cr_Min2, 0x80);
            CoreISP3_I2C_Write(Swp_Cr_Max2, 0xff);
            CoreISP3_I2C_Write(Swp_Cb2, 0xff);
            CoreISP3_I2C_Write(Swp_Cr2, 0x00);            
            CoreISP3_I2C_Write(ImgEffectC, 0x09);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On
            break;            
        case enISP_FUNC_IMAGE_ACCENT_SWAPING:
            CoreISP3_I2C_Write(ASwp_Cb_Min1, 0x80);
            CoreISP3_I2C_Write(ASwp_Cb_Max1, 0xff);
            CoreISP3_I2C_Write(ASwp_Cr_Min1, 0x20);
            CoreISP3_I2C_Write(ASwp_Cr_Max1, 0x60);
            CoreISP3_I2C_Write(ASwp_Cb1, 0x00);
            CoreISP3_I2C_Write(ASwp_Cr1, 0xff);
            CoreISP3_I2C_Write(ASwp_Cb_Min2, 0x20);
            CoreISP3_I2C_Write(ASwp_Cb_Max2, 0x60);
            CoreISP3_I2C_Write(ASwp_Cr_Min2, 0x80);
            CoreISP3_I2C_Write(ASwp_Cr_Max2, 0xff);
            CoreISP3_I2C_Write(ASwp_Cb2, 0xff);
            CoreISP3_I2C_Write(ASwp_Cr2, 0x00);                        
            CoreISP3_I2C_Write(ImgEffectC, 0x0a);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_WARM:
            CoreISP3_I2C_Write(ImgEffectB, 0x01);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On            
            break;            
        case enISP_FUNC_IMAGE_COOL:
            CoreISP3_I2C_Write(ImgEffectB, 0x02);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;            
        case enISP_FUNC_IMAGE_FOG:
			CoreISP3_I2C_Write(ImgEffectB, 0x03);
			CoreISP3_I2C_Write(ImgEffectD, 0x80);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;            
        case enISP_FUNC_IMAGE_OPPOSITE_NEGATIVE:
            CoreISP3_I2C_Write(ImgEffectB, 0x04);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;            
        case enISP_FUNC_IMAGE_OPPOSITE_AVERAGE:
            CoreISP3_I2C_Write(ImgEffectB, 0x05);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;
        default:
        case 'x': case 'X':
            break;
    }
}

/*********************************************************************
*Name			:	CoreISP3_SetFaceTracking
*Description	:	Set Face Tracking
*Param			:	FaceTrackingParam	ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceTracking( Cl_Bool FaceTrackingParam )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceTrackingParam);

    if(FaceTrackingParam)
    {
        /* Send command to ISP MCU : Face Tracking Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_TRACKING_START);
    }
    else
    {
        /* Send command to ISP MCU : Face Tracking Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_TRACKING_STOP);
    }

}
/*********************************************************************
*Name			:	CoreISP3_SetFaceAE
*Description	:	Set AE with Face Tracking
*Param			:	FaceAEParam			ON
										OFF
					FaceAEGlobalParam	ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceAE( Cl_Bool FaceAEParam)
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceAEParam);
#if 0//ISP3_UPDATE_0202  // update this
    if(FaceAEParam)
    {
        /* Send command to ISP MCU : Face AE Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AE_START);
    }
    else 
    {
        /* Send command to ISP MCU : Face AE Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AE_STOP);
    }
#else
	switch(FaceAEParam)
	{
		case enISP_FUNC_AEwithFACE_1:
			CoreISP3_Send_Command(ISP3_CMD_FACE_AE_STOP);
			break;
		case enISP_FUNC_AEwithFACE_2:
			CoreISP3_Send_Command(ISP3_CMD_FACE_AE_START);
			CoreISP3_Send_Command(ISP3_CMD_AE_GLOBAL);
			break;
		case enISP_FUNC_AEwithFACE_3:
			CoreISP3_Send_Command(ISP3_CMD_FACE_AE_START);
			CoreISP3_Send_Command(ISP3_CMD_AE_MULTI);
			break;
	}

#endif
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceAF
*Description	:	Set AF with Face Tracking
*Param			:	FaceAFParam			ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceAF( Cl_Bool FaceAFParam )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceAFParam);

    if(FaceAFParam)
    {
        /* Send command to ISP MCU : Face AF Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AF_START);

        /* Send command to ISP MCU : Face Full AF Enable  */
        //CoreISP3_Send_Command(ISP3_CMD_FULLAF);       //Max@090520 no need send FULL AF,  it just set a bfaceAF flag
    }
    else 
    {
        /* Send command to ISP MCU : Face AF Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AF_STOP);
    }

}
/*********************************************************************
*Name			:	CoreISP3_SetFaceAWB
*Description	:	Set AWB with Face Tracking
*Param			:	enISP_FUNC_AWBwithFD_Level_1
				    enISP_FUNC_AWBwithFD_Level_2
				    enISP_FUNC_AWBwithFD_Level_3
				    enISP_FUNC_AWBwithFD_Level_4
				    enISP_FUNC_AWBwithFD_Level_5    
				    enISP_FUNC_AWBwithFD_Level_6
				    enISP_FUNC_AWBwithFD_Level_7
				    enISP_FUNC_AWBwithFD_Level_8    
				    enISP_FUNC_AWBwithFD_Level_9    
				    enISP_FUNC_AWBwithFD_Level_10    
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceAWB( Cl_Bool FaceAWBParam )
{
#if 0
    if(FaceAWBParam)
    {
    }
    else 
    {
    }
#else
	ClUint_16 retval = 0;
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceAWBParam);
	switch(FaceAWBParam)
	{
		case enISP_FUNC_AWBwithFD_Level_1:
			retval = CoreISP3_I2C_Read(EnAWBFaceH);
			CoreISP3_I2C_Write(EnAWBFaceH, (retval | 0x02));
			break;
		case enISP_FUNC_AWBwithFD_Level_2:
			retval = CoreISP3_I2C_Read(EnAWBFaceH);
			CoreISP3_I2C_Write(EnAWBFaceH, (retval | 0x01));
			break;
		case enISP_FUNC_AWBwithFD_Level_3:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x80));
			break;
		case enISP_FUNC_AWBwithFD_Level_4:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x40));
			break;
		case enISP_FUNC_AWBwithFD_Level_5:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x20));
			break;
		case enISP_FUNC_AWBwithFD_Level_6:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x10));
			break;
		case enISP_FUNC_AWBwithFD_Level_7:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x08));
			break;
		case enISP_FUNC_AWBwithFD_Level_8:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x04));
			break;
		case enISP_FUNC_AWBwithFD_Level_9:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x02));
			break;
		case enISP_FUNC_AWBwithFD_Level_10:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x01));
			break;
		default:
			break;
	}
#endif
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceRotation
*Description	:	Set Rotation of Face Tracking
*Param			:	FaceRotation		ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceRotation( Cl_Bool FaceRotation )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceRotation);

    if(FaceRotation)
    {
        /* Send command to ISP MCU : Face Rotation Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_ROTATE_START);
    }
    else 
    {
        /* Send command to ISP MCU : Face Rotation Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_ROTATE_STOP);
    }

}
/*********************************************************************
*Name			:	CoreISP3_SetFaceROIThick
*Description	:	Set ROI of Face Tracking
*Param			:	FaceRoiThickParam		enISP_FUNC_FD_ROI_THICK_0
										    enISP_FUNC_FD_ROI_THICK_1
										    enISP_FUNC_FD_ROI_THICK_2
										    enISP_FUNC_FD_ROI_THICK_3
										    enISP_FUNC_FD_ROI_THICK_4
										    enISP_FUNC_FD_ROI_THICK_5
										    enISP_FUNC_FD_ROI_THICK_6
										    enISP_FUNC_FD_ROI_THICK_7
										    enISP_FUNC_FD_ROI_THICK_8
										    enISP_FUNC_FD_ROI_THICK_9
										    enISP_FUNC_FD_ROI_THICK_10
										    enISP_FUNC_FD_ROI_THICK_11
										    enISP_FUNC_FD_ROI_THICK_12
										    enISP_FUNC_FD_ROI_THICK_13
										    enISP_FUNC_FD_ROI_THICK_14
										    enISP_FUNC_FD_ROI_THICK_15
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceROIThick( enIsp3FaceROIThick FaceRoiThickParam )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceRoiThickParam);

    CoreISP3_I2C_Write(0xf20c, FaceRoiThickParam);
    DebugMessage(DM_JPEG, ">>ISP2_REG>>0xf20c==0x%x\n",CoreISP3_I2C_Read(0xf20c));
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceMaxDetectionCount
*Description	:	Set Max Detecting count of Face Tracking
*Param			:	FaceMaxDetectionParam		enISP_FUNC_FD_MAXCOUNT_1
											    enISP_FUNC_FD_MAXCOUNT_2
											    enISP_FUNC_FD_MAXCOUNT_3
											    enISP_FUNC_FD_MAXCOUNT_4
											    enISP_FUNC_FD_MAXCOUNT_5    
											    enISP_FUNC_FD_MAXCOUNT_6
											    enISP_FUNC_FD_MAXCOUNT_7
											    enISP_FUNC_FD_MAXCOUNT_8    
											    enISP_FUNC_FD_MAXCOUNT_9    
											    enISP_FUNC_FD_MAXCOUNT_10    
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceMaxDetectionCount( enIsp3FaceMaxDetectionCount FaceMaxDetectionParam )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceMaxDetectionParam);

    switch(FaceMaxDetectionParam)
    {
        case enISP_FUNC_FD_MAXCOUNT_1:
            CoreISP3_I2C_Write(0xf20a, 0x02);
            CoreISP3_I2C_Write(0xf20b, 0x00);
            break;
        case enISP_FUNC_FD_MAXCOUNT_2:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0x00);
            break;
        case enISP_FUNC_FD_MAXCOUNT_3:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0x80);
            break;
        case enISP_FUNC_FD_MAXCOUNT_4:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xc0);
            break;
        case enISP_FUNC_FD_MAXCOUNT_5:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xe0);
            break;
        case enISP_FUNC_FD_MAXCOUNT_6:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xf0);
            break;
        case enISP_FUNC_FD_MAXCOUNT_7:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xf8);
            break;
        case enISP_FUNC_FD_MAXCOUNT_8:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xfc);
            break;
        case enISP_FUNC_FD_MAXCOUNT_9:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xfe);
            break;
        case enISP_FUNC_FD_MAXCOUNT_10:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xff);
            break;            
        default:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xc0);
            break;
    }
    
}

/*********************************************************************
*Name			:	CoreISP3_SetFaceUserMode
*Description	:	Set Face Area to select Face by user
*Param			:	enIspFaceUserMode		enISP_FUNC_FDUser_Level_0 : Disable User Mode
											enISP_FUNC_FDUser_Level_1
											enISP_FUNC_FDUser_Level_2
											enISP_FUNC_FDUser_Level_3
											enISP_FUNC_FDUser_Level_4
											enISP_FUNC_FDUser_Level_5    
											enISP_FUNC_FDUser_Level_6
											enISP_FUNC_FDUser_Level_7
											enISP_FUNC_FDUser_Level_8    
											enISP_FUNC_FDUser_Level_9    
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceUserMode( enIspFaceUserMode FaceUserModeParam )
{
	ClUint_8 area = 0;
	ClUint_8 count1 = 1, count2 = 1, count3 = 1, count4 = 1, count5 = 1, count6 = 1, count7 = 1, count8 = 1, count9 = 1; 

    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, FaceUserModeParam);
	
    switch(FaceUserModeParam)
    {
        case enISP_FUNC_FDUser_Level_1:
			area = enISP_FUNC_FDUser_Level_1;
			
			count1++;
			if(count1 > 5)	count1 = 1;

			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count1 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
			//CoreISP3_I2C_Write(0xE640, (count9 & 0xF0) | (area & 0x0F));		//ZhouCS count9 error???????????????? should be count1??????
			CoreISP3_I2C_Write(0xE640, (count1 & 0xF0) | (area & 0x0F));		
            break;
			
        case enISP_FUNC_FDUser_Level_2:
			area = enISP_FUNC_FDUser_Level_2;
			count2++;
			if(count2 > 5)	count2 = 1;
			
			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count2 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count2 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_3:
			area = enISP_FUNC_FDUser_Level_3;
			count3++;
			if(count3 > 5)	count3 = 1;

			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count3 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count3 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_4:
			area = enISP_FUNC_FDUser_Level_4;
			count4++;
			if(count4 > 5)	count4 = 1;

			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count4 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count4 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_5:
			area = enISP_FUNC_FDUser_Level_5;
			count5++;
			if(count5> 5)	count5 = 1;
			
			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count5 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count5 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_6:
			area = enISP_FUNC_FDUser_Level_6;
			count6++;
			if(count6 > 5)	count6 = 1;

			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count6 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count6 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_7:
			area = enISP_FUNC_FDUser_Level_7;
			count7++;
			if(count7 > 5)	count7 = 1;

			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count7 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count7 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_8:
			area = enISP_FUNC_FDUser_Level_8;
			count8++;
			if(count8 > 5)	count8 = 1;

			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count8 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count8 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_9:
			area = enISP_FUNC_FDUser_Level_9;
			count9++;
			if(count9 > 5)	count9 = 1;

			DebugMessage(DM_JPEG, "FaceUserModeParam 0x%x\n", (count9 & 0xF0) | (area & 0x0F));
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count9 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_0:
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_STOP);
			CoreISP3_I2C_Write(0xE640, 0x00);
            break;         
			
        default:
			CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_STOP);
			CoreISP3_I2C_Write(0xE640, 0x00);
            break;
    }
    
}

/*********************************************************************
*Name			:	CoreISP3_ControlFaceApplication
*Description	:	Set All param of Face Tracking
*Param			:	stIspFaceDetectionCtrlParam		bFaceDetectionEnable
													bFaceTrackingEnable
													bFaceAE
													bFaceAF
													bFaceAWB
													bFaceRotation
													Isp3FaceROIThick
													Isp3FaceMaxDetectionCount
													enIsp3OutResolution OutputResolution
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_ControlFaceApplication( _tIspFaceDetectionCtrl *stIspFaceDetectionCtrlParam )
{
	DebugMessage(DM_JPEG, "\n====> %s::%d,%d,%d,%d,%d,%d,%d,%d,%d\n", __func__, 
								stIspFaceDetectionCtrlParam->bFaceAE, 						stIspFaceDetectionCtrlParam->bFaceAF,
								stIspFaceDetectionCtrlParam->bFaceAWB, 						stIspFaceDetectionCtrlParam->bFaceDetectionEnable,
								stIspFaceDetectionCtrlParam->bFaceRotation, 				stIspFaceDetectionCtrlParam->bFaceTrackingEnable,
								stIspFaceDetectionCtrlParam->Isp3FaceMaxDetectionCount, 	stIspFaceDetectionCtrlParam->Isp3FaceROIThick,
								stIspFaceDetectionCtrlParam->OutputResolution);

	/*if((CoreISP3_I2C_Read(0xed6c)!=0x01)||(CoreISP3_I2C_Read(0xed6d)!=0x40)||(CoreISP3_I2C_Read(0xed6e)!=0x00)||(CoreISP3_I2C_Read(0xed6f)!=0x40))
	{
		DebugMessage(DM_JPEG, "Thumbnail size not 320x240\n");
		CoreISP3_I2C_Write(0xed6c, 0x01);
		CoreISP3_I2C_Write(0xed6d, 0x40);
		CoreISP3_I2C_Write(0xed6e, 0x00);
		CoreISP3_I2C_Write(0xed6f, 0x40);
	}
	*/
	
	if(stIspFaceDetectionCtrlParam->bFaceDetectionEnable == Cl_True)
	{
		CoreISP3_Send_Command(ISP3_CMD_FACE_DETECTION_START);

		/* Check each functions */
		if(stIspFaceDetectionCtrlParam->bFaceTrackingEnable)
		{
			CoreISP3_SetFaceTracking(stIspFaceDetectionCtrlParam->bFaceTrackingEnable);
		}
		if(stIspFaceDetectionCtrlParam->bFaceAE)
		{
			CoreISP3_SetFaceAE(stIspFaceDetectionCtrlParam->bFaceAE);
		}
		if(stIspFaceDetectionCtrlParam->bFaceAF)
		{
			CoreISP3_SetFaceAF(stIspFaceDetectionCtrlParam->bFaceAF);
		}
		if(stIspFaceDetectionCtrlParam->bFaceAWB)
		{
			CoreISP3_SetFaceAWB(stIspFaceDetectionCtrlParam->bFaceAWB);
		}
		if(stIspFaceDetectionCtrlParam->bFaceRotation)
		{
			CoreISP3_SetFaceRotation(stIspFaceDetectionCtrlParam->bFaceRotation);
		}
		if(stIspFaceDetectionCtrlParam->Isp3FaceROIThick>enISP_FUNC_FD_ROI_THICK_0)
		{
			CoreISP3_SetFaceROIThick(stIspFaceDetectionCtrlParam->Isp3FaceROIThick);
		}
		if(stIspFaceDetectionCtrlParam->Isp3FaceMaxDetectionCount>enISP_FUNC_FD_MAXCOUNT_1)
		{
			CoreISP3_SetFaceMaxDetectionCount(stIspFaceDetectionCtrlParam->Isp3FaceMaxDetectionCount);
		}
		if(stIspFaceDetectionCtrlParam->bFaceUserMode>enISP_FUNC_FDUser_Level_0)
		{
			CoreISP3_SetFaceUserMode(stIspFaceDetectionCtrlParam->bFaceUserMode);
		}
	}
	else
	{
		CoreISP3_Send_Command(ISP3_CMD_FACE_DETECTION_STOP);

		CoreISP3_SetFaceTracking(Cl_False);
	}
}
//Added by Jacky for Face Tracking

/*脸部识别的优先级选择
-----------------

1    2    3

-----------------

4    5    6

-----------------

7    8    9

-----------------
*/

void CoreISP3_FaceTracking_On(void)
{
	_tIspFaceDetectionCtrl g_tIspFaceDetectCtrl;
	
	g_tIspFaceDetectCtrl.bFaceDetectionEnable		=	Cl_True;
	g_tIspFaceDetectCtrl.bFaceTrackingEnable		=	Cl_True;
	g_tIspFaceDetectCtrl.bFaceAE					=	Cl_False;  // enISP_FUNC_AEwithFACE_1; // Jacky change this
	g_tIspFaceDetectCtrl.bFaceAF					=	Cl_True;
	g_tIspFaceDetectCtrl.bFaceAWB					=	Cl_False;
	g_tIspFaceDetectCtrl.bFaceRotation				=	Cl_True;
	g_tIspFaceDetectCtrl.Isp3FaceROIThick			=	enISP_FUNC_FD_ROI_THICK_4;
	g_tIspFaceDetectCtrl.Isp3FaceMaxDetectionCount	=	enISP_FUNC_FD_MAXCOUNT_10;   // enISP_FUNC_FD_MAXCOUNT_10;  Jacky change this for test
	g_tIspFaceDetectCtrl.bFaceUserMode  = enISP_FUNC_FDUser_Level_5;   // Jacky change user mode
	//g_tIspFaceDetectCtrl.OutputResolution			=	enISP_RES_1_3MP;

	CoreISP3_ControlFaceApplication(&g_tIspFaceDetectCtrl);
}

void CoreISP3_FaceTracking_Off(void)
{
	_tIspFaceDetectionCtrl g_tIspFaceDetectCtrl;
	
	g_tIspFaceDetectCtrl.bFaceDetectionEnable	= Cl_False;

	CoreISP3_ControlFaceApplication(&g_tIspFaceDetectCtrl);
}

/*********************************************************************
*Name			:	CoreISP3_OutpJPEG
*Description	:	Set ISP Mode 
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_OutpJPEG( Cl_Bool bThumbnail )
{
    DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, bThumbnail);

#ifdef __ISP3_R__  
    CoreISP3_I2C_Write(JpgOutMode, 0x18);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 
//	CoreISP3_I2C_Write(JpgOutMode, 0x26);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 

#else //__ISP3_R__
    CoreISP3_I2C_Write(JpgOutMode, 0x08);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 
    CoreISP3_I2C_Write(Jpg_FifoLimit_H, 0x01);   //#0: 288, 1: 192, 2: 96, 3: JPEG fifo size high 5bit, low 8bit, else: 192 
    CoreISP3_I2C_Write(Jpg_FifoLimit_L, 0xc0);   //#JPEG fifo size low byte
#endif //__ISP3_R__

/* Jpeg data + padding data size */
#ifdef __ISP3_R__  
    CoreISP3_I2C_Write(RowCntH, 0x01);   //JPEG Output row count High
    CoreISP3_I2C_Write(RowCntL, 0xe0);   //JPEG Output row count Low
#else //__ISP3_R__
    CoreISP3_I2C_Write(RowCntH, 0x03);   //JPEG Output row count High
    CoreISP3_I2C_Write(RowCntL, 0xcc);   //JPEG Output row count Low
#endif //__ISP3_R__

    CoreISP3_I2C_Write(FlipHold, 0xff);   //JPEG Rotation (8x8) Parameter
    
    CoreISP3_I2C_Write(JHBlank, 0x06);   //JPEG Output Horizontal Blank Size
    CoreISP3_I2C_Write(Still_UpHBlankL, 0x10);   //Up scaling line blank
    
    CoreISP3_I2C_Write(Still_VRdRate_SclMode, 0x0f);   //JPEG Scaling mode[auto mode : 0x0F, average : 0x0A, linear : 0x05, subsample : 0x00]
    CoreISP3_I2C_Write(Thu_VRdRate_SclMode, 0x02);  //Thumbnail Scaling mode[auto mode : 0x0F,  average : 0x0A, linear : 0x05, subsample : 0x00]
    CoreISP3_I2C_Write(Pre_VRdRate_SclMode, 0x0f);   //Preview Scaling mode[auto mode : 0x0F,  average : 0x0A, linear : 0x05, subsample : 0x00]

    if(bThumbnail)
    {   
#ifdef __ISP3_R__
        /*JPCLKDiv = JpgClkDiv[3:0] PrvCLKDiv = JpgClkDiv[3:0] */
        CoreISP3_I2C_Write(JpgClkDiv, 0x02);   //PrvCLKDiv (2 = JClck * 1/2)
        /* Face Detection OFF */
        CoreISP3_I2C_Write(FaceModeReg, 0x00);   //Face off
#else   //__ISP3_R__
        /*JPCLKDiv = JpgClkDiv[3:0] PrvCLKDiv = JpgClkDiv[3:0] */
        CoreISP3_I2C_Write(JpgClkDiv, 0x01);   //PrvCLKDiv
#endif //__ISP3_R__        
    
        CoreISP3_I2C_Write(PrvMode, 0x01);   //Preview Mode On
        //WaitTime_us(100000);    // 100ms
    }
    /* OUT FORMAT */
    CoreISP3_I2C_Write(OutFmt_JPG, 0x83);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG
    //WaitTime_us(100000);    // 100ms
    
    /* Send Command(Output Jpeg Mode) to ISP */
    //CoreISP3_Send_Command(ISP3_CMD_CAPTURE);       


}
/*********************************************************************
*Name			:	CoreISP3_OutpYCbCr
*Description	:	Set ISP Mode
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_OutpYCbCr( void )
{
	DebugMessage(DM_JPEG, "\n====> %s\n", __func__);

    /* OUT FORMAT */
    CoreISP3_I2C_Write(OutFmt_JPG, 0x82);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG

	//fang
	//for preview 30fps
    CoreISP3_I2C_Write(OutFmt_EXT, 0x00);
#if 0
			//fang
			//before preview
			printk("Before Preview:sensor reg: 0xe010 = 0x%x \n", CoreISP3_I2C_Read(0xe010));
			printk("sensor reg: 0xe011 = 0x%x \n", CoreISP3_I2C_Read(0xe011));
			printk("sensor reg: 0xe012 = 0x%x \n", CoreISP3_I2C_Read(0xe012));
			printk("sensor reg: 0xe013 = 0x%x \n", CoreISP3_I2C_Read(0xe013));
			printk("sensor reg: 0xe014 = 0x%x \n", CoreISP3_I2C_Read(0xe014));
			printk("sensor reg: 0xe015 = 0x%x \n", CoreISP3_I2C_Read(0xe015));

			printk("sensor reg: 0xe050 = 0x%x \n", CoreISP3_I2C_Read(0xe050));
			printk("sensor reg: 0xe051 = 0x%x \n", CoreISP3_I2C_Read(0xe051));
			printk("sensor reg: 0xe058 = 0x%x \n", CoreISP3_I2C_Read(0xe058));
			printk("sensor reg: 0xe059 = 0x%x \n", CoreISP3_I2C_Read(0xe059));

			printk("sensor reg: 0xe060 = 0x%x \n", CoreISP3_I2C_Read(0xe060));
			printk("sensor reg: 0xe061 = 0x%x \n", CoreISP3_I2C_Read(0xe061));
			printk("sensor reg: 0xe062 = 0x%x \n", CoreISP3_I2C_Read(0xe062));
			printk("sensor reg: 0xe063 = 0x%x \n", CoreISP3_I2C_Read(0xe063));

			printk("sensor reg: 0xe649 = 0x%x \n", CoreISP3_I2C_Read(0xe649));
			printk("sensor reg: 0xe64a = 0x%x \n", CoreISP3_I2C_Read(0xe64a));
			printk("sensor reg: 0xe64b = 0x%x \n", CoreISP3_I2C_Read(0xe64b));

			printk("sensor reg: 0xe672 = 0x%x \n", CoreISP3_I2C_Read(0xe672));
			printk("sensor reg: 0xe651 = 0x%x \n", CoreISP3_I2C_Read(0xe651));
			printk("sensor reg: 0xe652 = 0x%x \n", CoreISP3_I2C_Read(0xe652));
			printk("sensor reg: 0xe663 = 0x%x \n", CoreISP3_I2C_Read(0xe663));
#endif
		CoreISP3_Send_Command(ISP3_CMD_PREVIEW);
}

/*********************************************************************
*Name			:	CoreISP3_SystemControl
*Description	:	Set ISP Sleep & Wake Up mode
*Param			:	param		ON : Sleep Mode
								OFF : Wake Up Mode
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
#if 1
/*
void CoreISP3_SystemControl(int sleep)
{
	if (sleep) {
		// ISP not use Sync, Image DATA pin
		CoreISP3_I2C_Write(0xE058, 0x3F);
		msleep(1);    // 100 ms
		// Additional item
		CoreISP3_I2C_Write(0xE052, 0xFF);
		msleep(1);    // 100 ms
		// PLL OFF, I2C SDA input 
		CoreISP3_I2C_Write(0xE060, 0x02);
		//msleep(100);    // 100 ms 
	} else {
		//3 PLL ON
		CoreISP3_I2C_Write(0xE060, 0x00);
		//SetReg_POWCNT(Cl_True);
	}
}
*/
void CoreISP3_SystemControl(void)
{
	//MCU off
	CoreISP3_I2C_Write(0xE070, 0x05);
	msleep(1);    // 100 ms
	//Flashrom off
	CoreISP3_I2C_Write(0xE0A2, 0x00);
	msleep(1);    // 100 ms
	//SDRAM off
	CoreISP3_I2C_Write(0xE0E0, 0x00);
	msleep(1);    // 100 ms
	//make sync & clock tri-state
	CoreISP3_I2C_Write(0xE058, 0x3F);
	msleep(1);    // 100 ms
	//make data pad tri-state
	CoreISP3_I2C_Write(0xE052, 0xFF);
	msleep(1);    // 100 ms

	//GPIO Pad input mode setting
	CoreISP3_I2C_Write(0xE080, 0xFF);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE081, 0xFF);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE082, 0xFF);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE083, 0xFF);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE089, 0xFF);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE08A, 0xFF);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE08B, 0xFF);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE08C, 0xFF);
	msleep(1);    // 100 ms

	//PLL OFF,SDA input mode
	CoreISP3_I2C_Write(0xE060, 0x02);
	msleep(1);    // 100 ms
}

void CoreISP3_Wakeup(void)
{
	CoreISP3_I2C_Write(0xE070, 0x04);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE070, 0x05);
	msleep(1);    // 100 ms
	CoreISP3_I2C_Write(0xE070, 0x04);
	msleep(1);    // 100 ms


}

EXPORT_SYMBOL(CoreISP3_SystemControl);

#else
void CoreISP3_SystemControl( ClUint_16 param )
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, param);

	if(param)
	{
		//ExtGPIO_WriteData(1, 0);
		//ExtGPIO_WriteData(2, 1);

		//3 ㄧPWDN Low
		//CoreISP3_I2C_Write(0xE010, 0xE0);
		//WaitTime_us(100000);				//4 100 ms

		//3 ㄨPWDN High
#if 0 //4 Sensor Reset
		CoreISP3_I2C_Write(0xE010, 0xF0);
		WaitTime_us(10000);				//4 100 ms

		
		//3 All GPIO port is output Low
		CoreISP3_I2C_Write(0xE071, 0x00);
		CoreISP3_I2C_Write(0xE072, 0x00);
		CoreISP3_I2C_Write(0xE073, 0x00);
		CoreISP3_I2C_Write(0xE074, 0x00);

		
		//3 Enable parallel sensor port 1 interface
		//3 Sensor Reset
		CoreISP3_I2C_Write(0xe010, 0xe0);	//4 [7] : Sensor Port 1 I/F Enable
											//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
											//4 [5] : S1_RST = 1
											
	
		//3 Delay
		WaitTime_us(10000);				//4 100ms
	
		CoreISP3_I2C_Write(0xe010, 0xc0);	//4 [7] : Sensor Port 1 I/F Enable
											//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
											//4 [5] : S1_RST = 0
	
		WaitTime_us(10000);				//4 100 ms

		CoreISP3_I2C_Write(0xe010, 0xe0);	//4 [7] : Sensor Port 1 I/F Enable
											//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
											//4 [5] : S1_RST = 1
 
		WaitTime_us(100000);				//4 100 ms
#else	//4 Senor SW Standby mode   Jacky update this code
		if(CoreISP3_I2C_Write(0xE033, 0x01)==Cl_False)
			return;
		CoreISP3_I2C_Write(0xE034, 0x00);
		CoreISP3_I2C_Write(0xE035, 0x00);
		CoreISP3_I2C_Write(0xE036, 0x00);
		//3 All GPIO port is output Low
		CoreISP3_I2C_Write(0xE071, 0x00);
		CoreISP3_I2C_Write(0xE072, 0x00);
		CoreISP3_I2C_Write(0xE073, 0x00);
		CoreISP3_I2C_Write(0xE074, 0x00);
#endif

		#if 1
		//3 ㄧISP not use Sync, Image DATA pin
		CoreISP3_I2C_Write(0xE058, 0x3F);
		WaitTime_us(100000);				//4 100 ms

		//3 ㄨAdditional item
		CoreISP3_I2C_Write(0xE052, 0xFF);
		WaitTime_us(100000);				//4 100 ms

		//3 ㄩPLL OFF, I2C SDA input 
		CoreISP3_I2C_Write(0xE060, 0x02);
		#endif

		
		WaitTime_us(100000);				//4 100 ms

		#if 0
		//3 ㄧISP not use Sync, Image DATA pin
		CoreISP3_I2C_Write(0xE058, 0x3F);
		WaitTime_us(100000);				//4 100 ms

		//3 ㄨAdditional item
		CoreISP3_I2C_Write(0xE052, 0xFF);

		//3 ㄩPLL OFF, I2C SDA input 
		CoreISP3_I2C_Write(0xE060, 0x03);

		
		//3 I2C Hi-Z
		CoreISP3_I2C_Write(0xE032, 0x80);
		#endif		
		DebugMessage(DM_JPEG, "@@@@@@@@@@@@@@@@@@@@@@@@\n");

		
	}
	else
	{
		//ExtGPIO_WriteData(1, 1);
		//ExtGPIO_WriteData(2, 0);

		//3 ISP  RESET
		//3 PLL ON
		CoreISP3_I2C_Write(0xE060, 0x00);

		//SetReg_POWCNT(Cl_True);
	}
}
#endif

enIsp3OutResolution CoreISP3_SetSensorSize(unsigned int width, unsigned int height)
{
	enIsp3OutResolution tempRes = enISP_RES_None;

	if( width<=176 && height<=144 )
		tempRes = enISP_RES_QCIF;
	else if( width<=320 && height<=240 )
		tempRes = enISP_RES_QVGA;
	else if( width<=352 && height<=288 )
		tempRes = enISP_RES_CIF;
	else if( width<=480 && height<=360 )
		tempRes = enISP_RES_480_360;
	else if( width<=640 && height<=480 )
		tempRes = enISP_RES_VGA;
	else if( width<=800 && height<=600 )
		tempRes = enISP_RES_SVGA;
	else if( width<=1024 && height<=768 )
		tempRes = enISP_RES_XGA;
	else if( width<=1280 && height<=960 )
		tempRes = enISP_RES_1_3MP;
	else if( width<=1280 && height<=1024 )
		tempRes = enISP_RES_SXGA;
	else if( width<=1600 && height<=1200 )
		tempRes = enISP_RES_UXGA;
	else if( width<=2048 && height<=1536 )
		tempRes = enISP_RES_QXGA;
	else if( width<=2560 && height<=1920 )
		tempRes = enISP_RES_5MP_FULL;
	
	return tempRes;
}

void CoreISP3_PLLOn(Cl_Bool bOn)
{
	//bPLL = bOn;
	if(bOn)
	{
		CoreISP3_I2C_Write(0xE060, 0x00);
	}
	else
	{
		CoreISP3_I2C_Write(0xE060, 0x03);
	}
}

void CoreISP3_FlickerSuppression(FLICKER_TYPE type)
{
    	DebugMessage(DM_JPEG, "\n %s %d \n",__func__, type);
		
#if 0  // old 	
	switch(type)
	{
		case FLICKER_OFF:
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
			break;

		case FLICKER_AUTO:
			CoreISP3_I2C_Write(0xE628, 0x01);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_ON);
			break;
			
		case FLICKER_50HZ:
			CoreISP3_I2C_Write(0xE628, 0x32);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
			break;
			
		case FLICKER_60HZ:
			CoreISP3_I2C_Write(0xE628, 0x3c);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
			break;
	}
#else  // update latest 
		switch(type)
		{
			case FLICKER_50HZ_OFF:  // 0
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
				break;

			case FLICKER_50HZ_ON:
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
				break;
				
			case FLICKER_60HZ_OFF:
				CoreISP3_I2C_Write(0xE628, 0x3C);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
				break;
				
			case FLICKER_60HZ_ON:
				CoreISP3_I2C_Write(0xE628, 0x3C);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
				break;
				
			case FLICKER_AUTO_OFF:
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
				break;
				
			case FLICKER_AUTO_ON:
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_ON);
				break;
		}

#endif
}

void CoreISP3_AE_SetISO(enIsp3AEISO isoLevel/**< ISO Level : 1~8, 0: Auto 1: Low ISO, 8: High ISO*/)
{
    	DebugMessage(DM_JPEG, "\n %s %d \n",__func__, isoLevel);
		
	switch(isoLevel)
	{
		case enISP_AE_ISO_AUTO:	// AUTO
			CoreISP3_I2C_Write(0xE65A, 0x00);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_100: // ISO 100
			CoreISP3_I2C_Write(0xE65A, 0x01);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_200:// ISO 200
			CoreISP3_I2C_Write(0xE65A, 0x02);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_300:// ISO 300
			CoreISP3_I2C_Write(0xE65A, 0x03);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_400:// ISO 400
			CoreISP3_I2C_Write(0xE65A, 0x04);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_500:// ISO 500
			CoreISP3_I2C_Write(0xE65A, 0x05);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_600:// ISO 600
			CoreISP3_I2C_Write(0xE65A, 0x06);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_700:// ISO 700
			CoreISP3_I2C_Write(0xE65A, 0x07);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_800:// ISO 800
			CoreISP3_I2C_Write(0xE65A, 0x08);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		default:
			break;
	}

	CoreISP3_GetISOGain();  // Jacky for test
}

ClUint_32 CoreISP3_Get_Version(void)
{
	ClUint_32 SW_Ver;
	ClUint_16 majorVer;
	ClUint_16 minorVer;
	ClUint_16 data1;
	ClUint_16 data0;
	ClUint_8 RetVal;
	
	CoreISP3_Send_Command(ISP3_CMD_GET_VERSION);  // Jacky change this
#if 0
	data1  = CoreISP3_I2C_Read(0xE628);
	data0  = CoreISP3_I2C_Read(0xE629);
	majorVer   = data1<<8 | data0;

	data1 = CoreISP3_I2C_Read(0xE62A);
	data0 = CoreISP3_I2C_Read(0xE62B);
	minorVer  = data1<<8 | data0;

	SW_Ver = majorVer <<16 | minorVer;

    	printk("\n ====Get ISP vision  %d [0x%x]====\n", SW_Ver, SW_Ver);
    if(SW_Ver < 0x168) 
#else
	data1  = CoreISP3_I2C_Read(0xE628);
	data0  = CoreISP3_I2C_Read(0xE629);
	majorVer   = data1<<8 | data0;

	data1 = CoreISP3_I2C_Read(0xE62A);
	data0 = CoreISP3_I2C_Read(0xE62B);
	minorVer  = data1<<8 | data0;

	SW_Ver = majorVer <<16 | minorVer;

    	DebugMessage(DM_JPEG, "\n ====Get ISP vision  %d [0x%x]====\n", SW_Ver, SW_Ver);
    //if(SW_Ver < 0x4128900)
    //if(!SW_Ver)
    if(SW_Ver < 0x500) 
#endif
    {
    	RetVal = CoreISP3_MCU_CodeDownload((ClUint_8*)MCU_BIN, sizeof(MCU_BIN), enDownloadMode_StackedMem);
		if (RetVal == Cl_False)
		{
			printk("CoreISP3_MCU_CodeDownload Fail Error : %d \n", RetVal);
			return Cl_False;
		}
		/* MCU Reset */
		CoreISP3_I2C_Write(MCURST, 0x05);
		WaitTime_us(10000);    // 100ms  // Jacky change 10ms for init speed
		CoreISP3_I2C_Write(MCURST, 0x04);
		WaitTime_us(10000);    // 100ms	// Jacky change 10ms for init speed

		/* Send Command(ISP & Sensor Initialize) to ISP */
		DebugMessage(DM_JPEG, "ISP3_CMD_SENSOR_INIT \n");
		CoreISP3_Send_Command(ISP3_CMD_SENSOR_INIT);
		DebugMessage(DM_JPEG, "ISP3_CMD_SENSOR_INIT end\n");

		/*Sensor Interface*/
		CoreISP3_I2C_Write(SMIABypass, 0x00);   //[7:7] LMBYPASS '0' = Async, '1' = Sync

		/* OUT FORMAT */
		CoreISP3_I2C_Write(OutFmt_JPG, 0x82);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG
		return 1;
    }
	else
	{
		printk("Firmware is already update\n");
		return 2;
	}
	return SW_Ver;

}

void CoreISP3_SetAWBMode(AWB_MANUAL_TYPE type)
{
    	DebugMessage(DM_JPEG, "\n %s %d \n",__func__, type);
	CoreISP3_I2C_Write(0xEb98, (ClUint_8)type);
}
				

// Added by Jacky according ISP3R function flow
void CoreISP3_SetSceneMode(enIsp3SceneMode mode)
{
    	 DebugMessage(DM_JPEG, "\n %s %d \n",__func__, mode);
		 
	 CoreISP3_I2C_Write(0xE628, mode);
	 CoreISP3_Send_Command(ISP3_CMD_MODE_AUTOMODE);  // Jacky change this   0x20
}

/* =============================================================================================================
Description :	UnSet WDR Mode
============================================================================================================= */
void CoreISP3_Set_WDR_Off(void)
{
	_tISP_WDR_CTRL IspWdrCtrl;

	IspWdrCtrl.bWDREnable = Cl_False;
	CoreISP3_SetWDR(&IspWdrCtrl);
}


void CoreISP3_Set_StillStabilizer_On(void)
{
	_tIspStillStabilizerCtrl  StillStabilizerParam;
	StillStabilizerParam.bStillStabilizerEnable = Cl_True;
	StillStabilizerParam.StillStabilzerLevel = enISP_FUNC_SS_LEVEL_1_2;
	CoreISP3_SetStillStabilizer(&StillStabilizerParam);
}


void CoreISP3_Set_StillStabilizer_Off(void)
{
	_tIspStillStabilizerCtrl  StillStabilizerParam;
	StillStabilizerParam.bStillStabilizerEnable = Cl_False;
	StillStabilizerParam.StillStabilzerLevel = enISP_FUNC_SS_LEVEL_1_2;
	CoreISP3_SetStillStabilizer(&StillStabilizerParam);
}

#define STEP_GAP  12  // 5

void CoreISP3_SetBrightness(enIsp3Level_Value val)
{
    	 DebugMessage(DM_JPEG, "\n %s %d ->[0x%x] \n", __func__, val, (0x40+((int)val-5)*STEP_GAP));
		 
	 CoreISP3_I2C_Write(0xE5E1, (ClUint_8)(0x40+((int)val-5)*STEP_GAP));	 
	 CoreISP3_I2C_Partial_Write(0xE002, 2, 2, 1); // brightness enable
}
 
void CoreISP3_SetContrast(enIsp3Level_Value val)
{
    	 DebugMessage(DM_JPEG, "\n %s %d ->[0x%x] \n", __func__, val, ((0x00+((int)val)*STEP_GAP)&0xFF));
		 
	 CoreISP3_I2C_Write(0xE5E0, (ClUint_8)((0x00+((int)val)*STEP_GAP)&0xFF));
	 CoreISP3_I2C_Partial_Write(0xE002, 2, 2, 1); // contrast enable
}
 
void CoreISP3_SetSaturation(enIsp3Level_Value val)
{
    	 DebugMessage(DM_JPEG, "\n %s %d ->[0x%x] \n", __func__, val, (0x80+((int)val-5)*STEP_GAP));
		 
	 CoreISP3_I2C_Write(0xE5B0, (ClUint_8)(0x80+((int)val-5)*STEP_GAP));
	 CoreISP3_I2C_Partial_Write(0xE002, 6, 6, 1); // satuation enable
}
 
void CoreISP3_SetHue(enIsp3Level_Value val)
{
    	 DebugMessage(DM_JPEG, "\n %s %d ->[0x%x] \n", __func__, val, ((0x00+((int)val)*STEP_GAP)&0xFF));
		 
	 CoreISP3_I2C_Write(0xE5B1, (ClUint_8)((0x00+((int)val)*STEP_GAP)&0xFF));
	 CoreISP3_I2C_Partial_Write(0xE002, 7, 7, 1); //  hue enable
}

void CoreISP3_Brightness_OnOff(Cl_Bool Brightness, ClUint_8 val)
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, Brightness);

	if(Brightness)
	{
		//# Level (Default =0x40) 0xE5E1[7:0] = 10step(gap = 0x05)
		//CoreISP3_I2C_Write(0xE5E1, 0x40);
		CoreISP3_I2C_Write(0xE5E1, val);
		//# Enable = 0xE002[2:2] = On(1)
		CoreISP3_I2C_Write(0xE002, 1<<2);		
	}
	else
	{
	    //# Disable = 0xE002[2:2] = Off (0)
	    CoreISP3_I2C_Write(0xE002, 0<<2);	
	}
}

void CoreISP3_Contrast_OnOff(Cl_Bool Contrast, ClUint_8 val)
{

	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, Contrast);
	
	if(Contrast)
	{
		//# Level (Default =0x00) 0xE5E0[7:0] = 10step(gap = 0x05)
		//CoreISP3_I2C_Write(0xE5E0, 0x00);
		CoreISP3_I2C_Write(0xE5E0, val);
		//# Enable = 0xE002[2:2] = On(1)
		CoreISP3_I2C_Write(0xE002, 1<<2);		
	}
	else
	{
		//# Disable = 0xE002[2:2] = On(1)
		CoreISP3_I2C_Write(0xE002, 0<<2);	
	}
}

void CoreISP3_Saturation_OnOff(Cl_Bool Saturation)
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, Saturation);
	if(Saturation)
    {
		//# Level (Default =0x80) 0xE5B0 [7:0] = 10step(gap = 0x05)
		CoreISP3_I2C_Write(0xE5B0, 0x80);
		//# Enable = 0xE002[6:6] = On(1)
		CoreISP3_I2C_Write(0xE002, 1<<6);		
    }
    else
    {
        //# Enable = 0xE002[6:6] = Off (0)
        CoreISP3_I2C_Write(0xE002, 0<<6);	
    }
}

void CoreISP3_Hue_OnOff(Cl_Bool Hue)
{
	DebugMessage(DM_JPEG, "\n====> %s::%d\n", __func__, Hue);
	if(Hue)
    {
		//# Level (Default =0x00) 0xE5B1[7:0] = 10step(gap = 0x05)
		CoreISP3_I2C_Write(0xE5B1, 0x00);
		//# Enable = 0xE002[7:7] = On(1)
		CoreISP3_I2C_Write(0xE002, 1<<7);		
    }
    else
    {
        //# Disable = 0xE002[7:7] = Off (0)
        CoreISP3_I2C_Write(0xE002, 0<<7);	
    }
}

//  // Jacky update 
// In Sunny module case, We can change YUV swap as below.
void CORE_ISP_YUV_Swap(enIsp3YUV_Swap mode)
{
/*===========================================================

YuV   Format 
1.  Register :  0xE050  
     [1]= 1 or 0   1: Cb pixel first , 0 : Cr pixel first
     [0]= 1 or 0   1: Y pixel first ,  0 : Cb or Cr first

2. setting 
    2-1) 0xE050 <== 0x0A   Cb first   : CbY CrY CbY CrY 
    2-2) 0xE050 <== 0x08    Cr first   : CrY  CbY CrY CbY 
    2-3) 0xE050 <== 0x0B   Y first Cb: YCb YCr YCb YCr 
    2-4) 0xE050 <== 0x09    Y first Cr: YCr YCb YCr Ycb 
==========================================================*/
    DebugMessage(DM_JPEG, "\n %s 0x%x  \n", __func__, mode);
    CoreISP3_I2C_Write(0xE050, mode);

}

void CORE_ISP_TestPatten(void)
{
    DebugMessage(DM_JPEG, "\n %s \n", __func__);
/*
ISP3 Test Pattern
   Address            		Value
1. 0xE0C8                  	0x84   ( RGB value )
2. 0xE0C8                    0x85   ( Red value )
3. 0xE0C8                    0x87   ( Gray value )

*/
    //CoreISP3_I2C_Write(0xE0C8, 0x84);
    //CoreISP3_I2C_Write(0xE0C8, 0x85);
    CoreISP3_I2C_Write(0xE0C8, 0x87);

}

#ifdef ISP3_ZOOM_ENABLE
ClUint_16 ispOutputWidth;
ClUint_16 ispOutputHeight;
ClUint_16 sensorOutputWidth;
ClUint_16 sensorOutputHeight;
// Jacky add this for zoom capture
ClUint_16 ispCaptureOutputWidth;  // this is capture resolution
ClUint_16 ispCaptureOutputHeight;
ClUint_16 sensorCaptureOutputWidth = 2560;    // 5M is 2560x1920
ClUint_16 sensorCaptureOutputHeight = 1920;  // 2M is 1600x1200
//end

#ifdef DEFINE_FLOAT
float zoomStepSize;
float g_curDigitalZoomRate = 1;
#endif

unsigned char g_curDigitalZoom = 0;

#ifndef ISP3_EVB_ENABLE
#define ROUND(d) ((d) > 0.0 ? (int) ((d) + 0.5) : -(int) ((-(d)) + 0.5))
#endif

void ISP3_RegWrite_Partial( ClUint_16 addr, ClUint_16 data2,  ClUint_16 data1, ClUint_16 data0)
{
	CoreISP3_I2C_Partial_Write(addr, data2,  data1, data0);

}

int ISP3_RegRead(ClUint_16 addr)
{
	return (int)CoreISP3_I2C_Read(addr);
}



void ISP3_Set_StillWindowsStartX(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Still_WinXStrH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinXStrH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_WinXStrL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinXStrL, nLow);
	}
}

void ISP3_Set_StillWindowsStartY(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Still_WinYStrH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinYStrH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_WinYStrL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinYStrL, nLow);
	}
}

void ISP3_Set_StillWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);
	
	if(CoreISP3_I2C_Write(Still_WinWidthH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinWidthH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_WinWidthL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinWidthL, nLow);
	}
}

void ISP3_Set_StillWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Still_WinHeightH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinHeightH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_WinHeightL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_WinHeightL, nLow);
	}
}

void ISP3_Set_ScaleInputWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Still_SclWidthIH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclWidthIH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_SclWidthIL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclWidthIL, nLow);
	}
}

void ISP3_Set_ScaleInputWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Still_SclHeightIH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclHeightIH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_SclHeightIL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclHeightIL, nLow);
	}
}

void ISP3_Set_ScaleOutputWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Still_SclWidthOH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclWidthOH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_SclWidthOL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclWidthOL, nLow);
	}
}

void ISP3_Set_ScaleOutputWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Still_SclHeightOH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclHeightOH, nHigh);
	}

	if(CoreISP3_I2C_Write(Still_SclHeightOL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Still_SclHeightOL, nLow);
	}
}

void ISP3_Set_JpegWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(JpgWidthH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(JpgWidthH, nHigh);
	}

	if(CoreISP3_I2C_Write(JpgWidthL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(JpgWidthL, nLow);
	}
}

void ISP3_Set_JpegWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(JpgHeightH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(JpgHeightH, nHigh);
	}
	
	if(CoreISP3_I2C_Write(JpgHeightL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(JpgHeightL, nLow);
	}
}

void ISP3_Set_ThumbnailScaleInputWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Thu_SclWidthIH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Thu_SclWidthIH, nHigh);
	}

	if(CoreISP3_I2C_Write(Thu_SclWidthIL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Thu_SclWidthIL, nLow);
	}
}

void ISP3_Set_ThumbnailScaleInputHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Thu_SclHeightIH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Thu_SclHeightIH, nHigh);
	}

	if(CoreISP3_I2C_Write(Thu_SclHeightIL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Thu_SclHeightIL, nLow);
	}
}

void ISP3_Set_ScalePreviewWindowsInputWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Pre_SclWidthIH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Pre_SclWidthIH, nHigh);
	}

	if(CoreISP3_I2C_Write(Pre_SclWidthIL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Pre_SclWidthIL, nLow);
	}
}

void ISP3_Set_ScalePreviewWindowsInputHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	DebugMessage(DM_JPEG, "====> %s, nData: %d \n", __func__, nData);

	if(CoreISP3_I2C_Write(Pre_SclHeightIH, nHigh)==Cl_False)
	{
		CoreISP3_I2C_Write(Pre_SclHeightIH, nHigh);
	}

	if(CoreISP3_I2C_Write(Pre_SclHeightIL, nLow)==Cl_False)
	{
		CoreISP3_I2C_Write(Pre_SclHeightIL, nLow);
	}	
}

/**
 * \brief Set Zoom for ISP3
 * \remarks
 * Change Digital Zoom rate
 * \return
 * Changed Digital Zoom rate
 * \author Lee Suk-Joo
 * \date 2008-06-01
 */
 #if 0
//float ISP3_ZoomScaleSet(int sensorOutputWidth, int sensorOutputHeight, float zoomRate, int *outputWidth, int *outputHeight)
float ISP3_ZoomScaleSet(UINT sensorOutputWidth, UINT sensorOutputHeight, float zoomRate)
{
	RETAILMSG(DBMSG, (TEXT(" CAM!++ ISP3_ZoomScaleSet.\r\n")));
	ISP3_WriteCmd(ISP3_CMD_AE_OFF);	// AE OFF

	clIIC_WriteReg_Partial_ISP3(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
//	Sleep(500);

	if(zoomRate <= ZOOMRATE_1X)
	{
		//// Downscaling ////
		//UINT width = ROUND(sensorOutputWidth * zoomRate);
		//UINT height = ROUND(sensorOutputHeight * zoomRate);
		UINT width = (UINT)((float)sensorOutputWidth * zoomRate);
		UINT height = (UINT)((float)sensorOutputHeight * zoomRate);
		RETAILMSG(DBMSG, (TEXT(" CAM!Downscaling,sensorOutputWidth = %d,sensorOutputHeight = %d.preview output width =%d,height = %d\r\n"),sensorOutputWidth,sensorOutputHeight,width,height));

		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		clIIC_WriteReg_Partial_ISP3(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
/*
		///// Sensor狼 Frame Length甫 承腮促.
*/

		ISP3_Set_StillWindowsStartX(0);
		ISP3_Set_StillWindowsStartY(0);
		ISP3_Set_StillWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_StillWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_ScaleInputWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_ScaleInputWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_ScaleOutputWindowsWidth(width);
		ISP3_Set_ScaleOutputWindowsHeight(height);
		ISP3_Set_JpegWindowsWidth(width);
		ISP3_Set_JpegWindowsHeight(height);
		ISP3_Set_ThumbnailScaleInputWidth(width);
		ISP3_Set_ThumbnailScaleInputHeight(height);
		ISP3_Set_ScalePreviewWindowsInputWidth(width);
		ISP3_Set_ScalePreviewWindowsInputHeight(height);

		//*outputWidth 	= width;
		//*outputHeight	 = height;
	}
	else
	{
		// Upscaling
		UINT startPosDiffx, startPosDiffy;

		//UINT width = ROUND(sensorOutputWidth / zoomRate);
		//UINT height = ROUND(sensorOutputHeight / zoomRate);
		UINT width = (UINT)((float)sensorOutputWidth / zoomRate);
		UINT height = (UINT)((float)sensorOutputHeight / zoomRate);

		RETAILMSG(DBMSG, (TEXT(" CAM!Downscaling,sensorOutputWidth = %d,sensorOutputHeight = %d.preview output width =%d,height = %d\r\n"),width,height,sensorOutputWidth, sensorOutputHeight));

		int valueOfE059 = CoreISP3_I2C_Read(0xE059);
		if(valueOfE059 & 0x01)
		{
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}

		clIIC_WriteReg_Partial_ISP3(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
		CoreISP3_I2C_Write(0xE012, 0x0002);
/*
		clIIC_Write(0xE033, 0x03);
		clIIC_Write(0xE034, 0x42);
		clIIC_Write(0xE035, 0x20);
		clIIC_Write(0xE036, 0x00);
*/

		if(zoomRate > 2.0f)
		{
			CoreISP3_I2C_Write(0xEDE2, 0x0004);
		}
/*
		if(g_curDigitalZoomRate == 1.0f)
		{
			clIIC_Write(0xE013, 0x0021);
			clIIC_WriteReg_Partial_ISP3(0xE014, 3, 0, 0x02);
			clIIC_Write(0xED27, 0x0001);
			clIIC_Write(0xEDE2, 0x0003);
//			clIIC_Write(0xE012, 0x0001);

			///// Sensor狼 Frame Length甫 承腮促.
		}
*/
		startPosDiffx = ((int)sensorOutputWidth - width)/2;
		startPosDiffy = ((int)sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_ScaleOutputWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_JpegWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_JpegWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth((int)sensorOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight((int)sensorOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth((int)sensorOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight((int)sensorOutputHeight);

		//*outputWidth 	= width;
		//*outputHeight	 = height;
	}

	clIIC_WriteReg_Partial_ISP3(0xE010, 7, 7, 1);	// Sensor 1 Input off

	ISP3_WriteCmd(ISP3_CMD_AE_ON);	// AE ON
	
	return zoomRate;
	RETAILMSG(DBMSG, (TEXT(" CAM!-- ISP3_ZoomScaleSet.\r\n")));
}

// 1280*960 (640*480) -> 1280*960
void Zoom1M_2X()
{
	ISP3_ZoomScaleSet(1280, 960, 2.0f);
}
#endif

//crop and downscale image for preview
void ISP3_VGA_PreviewZoomScaleSet(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom)
{
	unsigned int startPosDiffx = 0;
	unsigned int	startPosDiffy = 0;
	unsigned int nWidth = 0;
	unsigned int nHeight = 0;
	
    	DebugMessage(DM_JPEG, " %s %dx%d, zoom: %d \n",__func__, OutputWidth, OutputHeight, dwZoom);
		
	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
	//Sleep(500);
	switch(dwZoom)
	{
		case CAM_ZOOM_100X:
		{
			nWidth = 640;
			nHeight = 480;		
			break;
		}
		case CAM_ZOOM_125X:
		{
			nWidth = 512;   //裁剪后的大小
			nHeight = 384;
			break;
		}
		case CAM_ZOOM_150X:  // 1.6
		{
			nWidth = 400;  //裁剪后的大小,640/1.6 = 400
			nHeight = 300;	  
			break;
		}
		case CAM_ZOOM_200X:  // only this scale ok
		{
			nWidth = 320;   //裁剪后的大小
			nHeight = 240;	
			break;
		}
		case CAM_ZOOM_250X:
		{
			nWidth = 256;   //裁剪后的大小
			nHeight = 192;	
			break;
		}
		case CAM_ZOOM_VP:
		{
			//nWidth = 1056;   //裁剪后的大小,CIF*3 for 5M camera VP
			//nHeight = 864;	
			break;
		}
		default:   //%100
		{
			nWidth = 640;
			nHeight = 480;		
			break;
		}
	}

	startPosDiffx = (640 - nWidth)/2;
	startPosDiffy = (480 - nHeight)/2;	

	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);
/*	if(nWidth >= OutputWidth)  //downscale
	{
		RETAILMSG(DBMSG, (TEXT(" CAM!downscale.\r\n")));
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
	}
	else   //upscale
	{
		RETAILMSG(DBMSG, (TEXT(" CAM!upscale.\r\n")));
		int valueOfE059 = CoreISP3_I2C_Read(0xE059);
		if(valueOfE059 & 0x01)
		{
			RETAILMSG(DBMSG, (TEXT(" CAM!CoreISP3_I2C_Write(0xE013, 0x0021);.\r\n")));
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			RETAILMSG(DBMSG, (TEXT(" CAM!CoreISP3_I2C_Write(0xE013, 0x0022);.\r\n")));
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}

		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
		CoreISP3_I2C_Write(0xE012, 0x0002);
	}
*/	
	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(nWidth);  // width
	ISP3_Set_StillWindowsHeight(nHeight);
	ISP3_Set_ScaleInputWindowsWidth(nWidth);
	ISP3_Set_ScaleInputWindowsHeight(nHeight);
	
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);

	//Sleep(500);
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	
}


//crop and downscale image for preview
void ISP3_PreviewZoomScaleSet(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom)
{
	unsigned int startPosDiffx, startPosDiffy,nWidth,nHeight;
	
    	DebugMessage(DM_JPEG, " %s %dx%d, zoom: %d \n",__func__, OutputWidth, OutputHeight, dwZoom);
		
	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
	//Sleep(500);
	switch(dwZoom)
	{
		case CAM_ZOOM_100X:
		{
			nWidth = 1280;
			nHeight = 960;		
			break;
		}
		case CAM_ZOOM_125X:
		{
			nWidth = 1024;   //裁剪后的大小
			nHeight = 768;
			break;
		}
		case CAM_ZOOM_150X:
		{
			nWidth = 864;  //裁剪后的大小,1280/864 = 1.48
			nHeight = 648;	  //960/648 = 1.48
			break;
		}
		case CAM_ZOOM_200X:
		{
			nWidth = 640;   //裁剪后的大小
			nHeight = 480;	
			break;
		}
		case CAM_ZOOM_250X:
		{
			nWidth = 512;   //裁剪后的大小
			nHeight = 384;	
			break;
		}
		case CAM_ZOOM_VP:
		{
			nWidth = 1056;   //裁剪后的大小,CIF*3 for 5M camera VP
			nHeight = 864;	
			break;
		}
		default:   //%100
		{
			nWidth = 1280;
			nHeight = 960;		
			break;
		}
	}

	startPosDiffx = (1280 - nWidth)/2;
	startPosDiffy = (960 - nHeight)/2;	

	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);
/*	if(nWidth >= OutputWidth)  //downscale
	{
		RETAILMSG(DBMSG, (TEXT(" CAM!downscale.\r\n")));
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
	}
	else   //upscale
	{
		RETAILMSG(DBMSG, (TEXT(" CAM!upscale.\r\n")));
		int valueOfE059 = CoreISP3_I2C_Read(0xE059);
		if(valueOfE059 & 0x01)
		{
			RETAILMSG(DBMSG, (TEXT(" CAM!CoreISP3_I2C_Write(0xE013, 0x0021);.\r\n")));
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			RETAILMSG(DBMSG, (TEXT(" CAM!CoreISP3_I2C_Write(0xE013, 0x0022);.\r\n")));
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}

		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
		CoreISP3_I2C_Write(0xE012, 0x0002);
	}
*/	
	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(nWidth);
	ISP3_Set_StillWindowsHeight(nHeight);
	ISP3_Set_ScaleInputWindowsWidth(nWidth);
	ISP3_Set_ScaleInputWindowsHeight(nHeight);
	
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);

	//Sleep(500);
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	
}
#if 0
//crop and downscale image for capture
void ISP3_ImageCrop(unsigned int sensorOutputWidth, unsigned int sensorOutputHeight,unsigned int CropedWidth,unsigned int CropedHeight, unsigned int OutputWidth,unsigned int OutputHeight)
{
	unsigned int startPosDiffx, startPosDiffy;

	
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off 
	WaitTime_us(50*1000);

	startPosDiffx = (sensorOutputWidth - CropedWidth)/2;
	startPosDiffy = (sensorOutputHeight - CropedHeight)/2;	

    	DebugMessage(DM_JPEG, " %s %dx%d \n",__func__, startPosDiffx, startPosDiffy);

	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);
	
	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(CropedWidth);
	ISP3_Set_StillWindowsHeight(CropedHeight);
	ISP3_Set_ScaleInputWindowsWidth(CropedWidth);
	ISP3_Set_ScaleInputWindowsHeight(CropedHeight);
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);

	WaitTime_us(50*1000);
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

}
#endif

//crop and downscale image for capture
void ISP3_ImageCrop(unsigned int sensorOutputWidth, unsigned int sensorOutputHeight,unsigned int CropedWidth,unsigned int CropedHeight, unsigned int OutputWidth,unsigned int OutputHeight)
{
	 unsigned int startPosDiffx, startPosDiffy;
	 int valueOfE059;

	 CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0); // Sensor 1 Input off 
	 WaitTime_us(50*1000);
	 startPosDiffx = (sensorOutputWidth - CropedWidth)/2;
	 startPosDiffy = (sensorOutputHeight - CropedHeight)/2; 
	  DebugMessage(DM_JPEG, " %s %dx%d \n",__func__, startPosDiffx, startPosDiffy);
	 valueOfE059 = ISP3_RegRead(0xE059);
	 DebugMessage(DM_JPEG, " 0xe059: 0x%x \n", valueOfE059);
	 
	 if(valueOfE059 & 0x01)
	 {
		 // JPEG 捞搁
		 CoreISP3_I2C_Write(0xE013, 0x0021);
	 }
	 else
	 {
		 // YCbCr 捞搁
		 CoreISP3_I2C_Write(0xE013, 0x0022);
	 }
	 
	 ISP3_RegWrite_Partial(0xE014, 3, 0, 0x02);
	 CoreISP3_I2C_Write(0xED27, 0x0001);
	 CoreISP3_I2C_Write(0xEDE2, 0x0003);
	 CoreISP3_I2C_Write(0xE012, 0x0002);
	 //if(zoomRate > 2.0f)
	 {
	 //CoreISP3_I2C_Write(0xEDE2, 0x0004);
	 }
	 ISP3_Set_StillWindowsStartX(startPosDiffx);
	 ISP3_Set_StillWindowsStartY(startPosDiffy);
	 ISP3_Set_StillWindowsWidth(CropedWidth);
	 ISP3_Set_StillWindowsHeight(CropedHeight);
	 ISP3_Set_ScaleInputWindowsWidth(CropedWidth);
	 ISP3_Set_ScaleInputWindowsHeight(CropedHeight);
	 ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	 ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
	 // following code  need for jpeg 
	 ISP3_Set_JpegWindowsWidth(OutputWidth);
	 ISP3_Set_JpegWindowsHeight(OutputHeight);
	 ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	 ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	 ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	 ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);
	 /**/
	 WaitTime_us(50*1000);
	 CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1); // Sensor 1 Input on
}



void ISP3_ZoomInit(void)
{
	#ifdef ISP3_EVB_ENABLE
	DebugMessage(DM_JPEG, "====> %s, [%dx%d]\n", __func__, g_Isp3SensorInfo.width, g_Isp3SensorInfo.height);

       sensorOutputWidth = CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL);
      	sensorOutputHeight = CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL);
      	ispOutputWidth = CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL);
      	ispOutputHeight= CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL);
	#else
       sensorOutputWidth = CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL);
      	sensorOutputHeight = CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL);
      	ispOutputWidth = CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL);
      	ispOutputHeight= CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL);
	#endif
	DebugMessage(DM_JPEG, "====> %s,   I[%d %d]->O[%d %d]\n", __func__, sensorOutputWidth, sensorOutputHeight, ispOutputWidth, ispOutputHeight);

	#ifdef DEFINE_FLOAT
	zoomStepSize = 0.1f;
	// SlideBar 狼 Range绰 	0 ~ upscaleStepCount + downscaleStepCount 捞 等促.
	g_curDigitalZoomRate = 1.0f;
	#endif
	
}

void Yulong_TestZoom(void)
{
#ifdef DEFINE_FLOAT
	DebugMessage(DM_JPEG, "====> %s \n", __func__);
	/////////////////////////////////////
	// Zoom In Full mode
	/////////////////////////////////////
	CoreISP3_SetResolution(enISP_RES_5MP_FULL , CMD_Preview);

	sensorOutputWidth = 2560;
	sensorOutputHeight = 1920;
	ispOutputWidth = 2560;
	ispOutputHeight = 1920;
/////// up scaleing /////////
	ISP3_SetDigitalZoomRate((float)2560/(float)1280);	// 2.0
	ISP3_SetDigitalZoomRate((float)2560/(float)1600);	// 1.6
	ISP3_SetDigitalZoomRate((float)2560/(float)2048);	// 1.25
	ISP3_SetDigitalZoomRate((float)2560/(float)2560);	// 1.0

/////// down scaleing /////////
	ISP3_SetDigitalZoomRate((float)2048/(float)2560);
	ISP3_SetDigitalZoomRate((float)1600/(float)2560);
	ISP3_SetDigitalZoomRate((float)1280/(float)2560);
	ISP3_SetDigitalZoomRate((float)1024/(float)2560);
	ISP3_SetDigitalZoomRate((float)800/(float)2560);
	ISP3_SetDigitalZoomRate((float)640/(float)2560);
	ISP3_SetDigitalZoomRate((float)320/(float)2560);
	ISP3_SetDigitalZoomRate((float)160/(float)2560);

	/////////////////////////////////////
	// Zoom In Preview mode
	/////////////////////////////////////
	//CoreISP3_SetResolution(ISP3_CMD_PREVIEW, CMD_Preview);
	CoreISP3_SetResolution(enISP_RES_5MP_FULL, CMD_Preview);  // Jacky change this
	
	sensorOutputWidth = 1280;
	sensorOutputHeight = 960;
	ispOutputWidth = 1280;
	ispOutputHeight = 960;
	
/////// up scaleing /////////
	ISP3_SetDigitalZoomRate((float)1280/(float)640);	// 2.0
	ISP3_SetDigitalZoomRate((float)1280/(float)800);
	ISP3_SetDigitalZoomRate((float)1280/(float)1024);
	ISP3_SetDigitalZoomRate((float)1280/(float)1280);	// 1.0

/////// down scaleing /////////
	ISP3_SetDigitalZoomRate((float)1024/(float)1280);
	ISP3_SetDigitalZoomRate((float)800/(float)1280);
	ISP3_SetDigitalZoomRate((float)640/(float)1280);
	ISP3_SetDigitalZoomRate((float)320/(float)1280);
	ISP3_SetDigitalZoomRate((float)160/(float)1280);

	/////////////////////////////////////
	// Zoom In VGA mode
	/////////////////////////////////////
	CoreISP3_SetResolution(enISP_RES_VGA, CMD_Preview);  // Jacky change this
	
	sensorOutputWidth = 640;
	sensorOutputHeight = 480;
	ispOutputWidth = 640;
	ispOutputHeight = 480;

/////// up scaleing /////////
	ISP3_SetDigitalZoomRate((float)640/(float)320);		// 2.0
	ISP3_SetDigitalZoomRate((float)640/(float)640);		// 1.0

/////// down scaleing /////////
	ISP3_SetDigitalZoomRate((float)320/(float)640);		// 0.5
	ISP3_SetDigitalZoomRate((float)160/(float)640);		// 0.25
#endif	
}


void ISP3_Set_QVGAPreview(void)  // for 480x360 resolution
{
	int width;
	int height;
	int startPosDiffx, startPosDiffy;
	int valueOfE059;

	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 320;   
	ispOutputHeight = 240; 

      DebugMessage(DM_JPEG, "ISP3_Set_QVGAPreview Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺

	{
		// Upscaling
		//fang
		/*
		width = 320;   //             
		height = 240;   //    960
		
		valueOfE059 = ISP3_RegRead(0xE059);
		
		DebugMessage(DM_JPEG, "Upscaling  [%d %d], 0xe059: 0x%x \n", width, height, valueOfE059);
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
		
		startPosDiffx = (sensorOutputWidth - width)/2;
		startPosDiffy = (sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		*/
		ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); // Jacky change and test ok
		ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight+2);

	}
      DebugMessage(DM_JPEG, "ISP3_Set_QVGAPreview Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );

	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	CoreISP3_Send_Command(0xd5);	//record for 30fps
	CoreISP3_Brightness_OnOff(Cl_True, 0x20); //record for fix brightness
}


void ISP3_SetDigitalZoom(enIsp3ZoomRate rate)  // for 480x360 resolution
{
	int width;
	int height;
	int startPosDiffx, startPosDiffy;
	int valueOfE059;

	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 640;   
	ispOutputHeight = 480; 

      DebugMessage(DM_JPEG, "Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺

	{
		// Upscaling
		DebugMessage(DM_JPEG, "===zoom rate: %d \n", rate);
		#if 1
		//fang
		if(rate == 10)
			rate = 9;
		width = 1280-64*rate;   //             1408x1152    - 176x144  = 1232x1008           -176x144 = 1056x864
		height = 960-48*rate;   //    960
		#else
		switch(rate)
		{ 
			// Image scaling: 1/32 ~ 2x linear (area 4x)  
			case enISP_ZOOM_CIF:
			{
				ispOutputWidth = 352;   //  240   //   176x144 ->88x72 ->44x36->22x18->11x9   ->110x90
				ispOutputHeight = 290;  //  160  	// first line of coreisp3 sensor outputed is error data, so discard one line . 
				width = 1166;   //             1408x1152    - 176x144  = 1232x1008           -176x144 = 1056x864
				height = 954;   //    960
				break;
			}
			case enISP_ZOOM_QCIF:
			{ 
				ispOutputWidth = 176;   //  240   //   176x144 ->88x72 ->44x36->22x18->11x9   ->110x90
				ispOutputHeight = 146;  //  160 // first line of coreisp3 sensor outputed is error data, so discard one line . 
				width = 1166;   //             1408x1152    - 176x144  = 1232x1008           -176x144 = 1056x864
				height = 954;   //    960
				break;
			} 	
			case enISP_ZOOM_QVGA:
			{ 
				ispOutputWidth = 320;   //  240   //   176x144 ->88x72 ->44x36->22x18->11x9   ->110x90
				ispOutputHeight = 242;  //  160 // first line of coreisp3 sensor outputed is error data, so discard one line . 
				width = 1280;   //             1408x1152    - 176x144  = 1232x1008           -176x144 = 1056x864
				height = 960;   //    960
				break;
			} 
			case enISP_ZOOM_VGA:
			{ 
				ispOutputWidth = 640;   //  240   //   176x144 ->88x72 ->44x36->22x18->11x9   ->110x90
				ispOutputHeight = 482;  //  160  // first line of coreisp3 sensor outputed is error data, so discard one line . 
				width = 1280;   //             1408x1152    - 176x144  = 1232x1008           -176x144 = 1056x864
				height = 960;   //    960
				break;
			} 				
			case enISP_ZOOM_480_360X2:   // 1.33
			{ 
				width = 960;   
				height = 720;  
				break;
			}
			case enISP_ZOOM_480_360X3:   // 1.6
			{ 
				width = 800;
				height = 600;
				break;
			}
			case enISP_ZOOM_480_360X4:   // 2.0
			{ 
				width = 640; 
				height = 480;
				break;
			}
 			case enISP_ZOOM_480_360:   // 1.0			
			default:
			{ 
				width = 1280;   
				height = 960;  
				break;
			}

		}
		#endif
		
		valueOfE059 = ISP3_RegRead(0xE059);
		
		DebugMessage(DM_JPEG, "Upscaling  [%d %d], 0xe059: 0x%x \n", width, height, valueOfE059);
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
		
		startPosDiffx = (sensorOutputWidth - width)/2;
		startPosDiffy = (sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); // Jacky change and test ok
		ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight+2);

		 /*// following ..   do not set following register, it will effect face tracking		  
		ISP3_Set_JpegWindowsWidth(ispOutputWidth);
		ISP3_Set_JpegWindowsHeight(ispOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth(ispOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight(ispOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth(ispOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight(ispOutputHeight);
		*/
	}
      DebugMessage(DM_JPEG, "Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );

	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
}



#ifdef DEFINE_FLOAT
float ISP3_SetDigitalZoomRate(float zoomRate)
{
	int width;
	int height;

	#if 0
       sensorOutputWidth = CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL);
      	sensorOutputHeight = CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL);
      	ispOutputWidth = CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL);
      	ispOutputHeight= CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL);
	//#else
	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 640;
	ispOutputHeight = 480;
	#endif
      DebugMessage(DM_JPEG, "Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	
	DebugMessage(DM_JPEG, "====> %s, rate: %d    I[%d %d]->O[%d %d]\n", __func__, zoomRate, sensorOutputWidth, sensorOutputHeight, ispOutputWidth, ispOutputHeight);

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
//	Sleep(500);

	if(zoomRate <= ZOOMRATE_1X)
	{
		//// Downscaling ////

		width = ROUND((float)sensorOutputWidth * zoomRate);
		height = ROUND((float)sensorOutputHeight * zoomRate);
		
		DebugMessage(DM_JPEG, "Downscaling  [%d %d] \n", width, height);

		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
 		CoreISP3_I2C_Write(0xE012, 0x0001);

		ISP3_Set_StillWindowsStartX(0);
		ISP3_Set_StillWindowsStartY(0);
		ISP3_Set_StillWindowsWidth(sensorOutputWidth);
		ISP3_Set_StillWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleInputWindowsWidth(sensorOutputWidth);
		ISP3_Set_ScaleInputWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleOutputWindowsWidth(width);
		ISP3_Set_ScaleOutputWindowsHeight(height);
		ISP3_Set_JpegWindowsWidth(width);
		ISP3_Set_JpegWindowsHeight(height);
		ISP3_Set_ThumbnailScaleInputWidth(width);
		ISP3_Set_ThumbnailScaleInputHeight(height);
		ISP3_Set_ScalePreviewWindowsInputWidth(width);
		ISP3_Set_ScalePreviewWindowsInputHeight(height);


	}
	else
	{
		// Upscaling
		int startPosDiffx, startPosDiffy;
		int valueOfE059;
			
		//width = ROUND((float)sensorOutputWidth * zoomRate);  // Jacky for test
		//height = ROUND((float)sensorOutputHeight * zoomRate);
		width = ((float)sensorOutputWidth / zoomRate);   // ROUND
		height = ((float)sensorOutputHeight / zoomRate);  //ROUND
		
		valueOfE059 = ISP3_RegRead(0xE059);
		
		DebugMessage(DM_JPEG, "Upscaling  [%d %d], 0xe059: 0x%x \n", width, height, valueOfE059);
		#if 0  // test ok
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
		#else// jacky open this for test ok
		if(valueOfE059 & 0x01)
		{
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
 		CoreISP3_I2C_Write(0xE012, 0x0002);

		if(zoomRate > 2.0f)
		{
			CoreISP3_I2C_Write(0xEDE2, 0x0004);
		}
		#endif
		
		startPosDiffx = (sensorOutputWidth - width)/2;
		startPosDiffy = (sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); // Jacky change and test ok
		ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight);
		ISP3_Set_JpegWindowsWidth(ispOutputWidth);
		ISP3_Set_JpegWindowsHeight(ispOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth(ispOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight(ispOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth(ispOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight(ispOutputHeight);

	}

	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	return zoomRate;
}


float ISP3_SetDigitalZoomRate_Face(float zoomRate, int startPosDiffx, int startPosDiffy)
{
	int width;
	int height;

	#if 0
	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 640;
	ispOutputHeight = 480;
	#endif
      DebugMessage(DM_JPEG, "Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	
	DebugMessage(DM_JPEG, "====> %s, rate%d:  I[%d %d]->O[%d %d]\n", __func__, sensorOutputWidth, sensorOutputHeight, ispOutputWidth, ispOutputHeight);

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
//	Sleep(500);

	if(zoomRate <= ZOOMRATE_1X)
	{
		//// Downscaling ////

		width = ROUND((float)sensorOutputWidth * zoomRate);
		height = ROUND((float)sensorOutputHeight * zoomRate);
		
		DebugMessage(DM_JPEG, "Downscaling  [%d %d] \n", width, height);

		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
 		CoreISP3_I2C_Write(0xE012, 0x0001);

		ISP3_Set_StillWindowsStartX(0);
		ISP3_Set_StillWindowsStartY(0);
		ISP3_Set_StillWindowsWidth(sensorOutputWidth);
		ISP3_Set_StillWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleInputWindowsWidth(sensorOutputWidth);
		ISP3_Set_ScaleInputWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleOutputWindowsWidth(width);
		ISP3_Set_ScaleOutputWindowsHeight(height);
		ISP3_Set_JpegWindowsWidth(width);
		ISP3_Set_JpegWindowsHeight(height);
		ISP3_Set_ThumbnailScaleInputWidth(width);
		ISP3_Set_ThumbnailScaleInputHeight(height);
		ISP3_Set_ScalePreviewWindowsInputWidth(width);
		ISP3_Set_ScalePreviewWindowsInputHeight(height);


	}
	else
	{
		// Upscaling
		//int startPosDiffx, startPosDiffy;
		int valueOfE059;
			
		//width = ROUND((float)sensorOutputWidth * zoomRate);  // Jacky for test
		//height = ROUND((float)sensorOutputHeight * zoomRate);
		width = ((float)sensorOutputWidth / zoomRate);   // ROUND
		height = ((float)sensorOutputHeight / zoomRate);  //ROUND
		
		valueOfE059 = ISP3_RegRead(0xE059);
		
		DebugMessage(DM_JPEG, "Upscaling  [%d %d], 0xe059: 0x%x \n", width, height, valueOfE059);
		#if 0  // test ok
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
		#else// jacky open this for test
		if(valueOfE059 & 0x01)
		{
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
 		CoreISP3_I2C_Write(0xE012, 0x0002);

		if(zoomRate > 2.0f)
		{
			CoreISP3_I2C_Write(0xEDE2, 0x0004);
		}
		#endif
		
		//startPosDiffx = (sensorOutputWidth - width)/2;
		//startPosDiffy = (sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); // Jacky change and test ok
		ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight);
		ISP3_Set_JpegWindowsWidth(ispOutputWidth);
		ISP3_Set_JpegWindowsHeight(ispOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth(ispOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight(ispOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth(ispOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight(ispOutputHeight);

	}

	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	return zoomRate;
}
#endif

void ISP3_SetZoomRate(enIsp3ZoomRate rate)
{
	//Yulong_TestZoom();
	//ISP3_PreviewZoomScaleSet(240, 320, CAM_ZOOM_200X);  // Jacky for test
	//ISP3_PreviewZoomScaleSet(1280, 960, CAM_ZOOM_250X);  // Jacky for test
	//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_200X);  // Jacky for test
	//return;
	DebugMessage(DM_JPEG, "====> %s, rate: %d \n", __func__, rate);
	g_curDigitalZoom = rate;
	
	switch(rate)
	{
		case enISP_ZOOM_RATE_1:
			//ISP3_SetDigitalZoomRate(1.0f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_100X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(640, 480, CAM_ZOOM_100X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_1_25:
			//ISP3_SetDigitalZoomRate(1.25f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_125X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(640, 480, CAM_ZOOM_125X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_1_5:
			//ISP3_SetDigitalZoomRate(1.5f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_150X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(640, 480, CAM_ZOOM_150X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_1_75:
			//ISP3_SetDigitalZoomRate(1.75f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_200X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(640, 480, CAM_ZOOM_200X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_2:
			//ISP3_SetDigitalZoomRate(2.0f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_250X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(640, 480, CAM_ZOOM_250X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_2_5:
			//ISP3_SetDigitalZoomRate(2.5f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_100X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(640, 480, CAM_ZOOM_100X);  // Jacky for test
			break;
		default:
			break;
		}
}
#endif

void ISP3_Set_FlashOnOff(Cl_Bool flashOn)
{
#if 0
	if(flashOn)
		CoreISP3_Send_Command(0xb4);
	else
		CoreISP3_Send_Command(0xb5);
#endif		
}


/*****************************************************************************
 *  Power & Reset
 *****************************************************************************/
static void sensor_power_on(void)
{
	struct sensor_platform_data *pdata;
	pdata = g_client->dev.platform_data;
	if(pdata->power_on)
	{
		pdata->power_on(pdata->id);
    	CoreISP3_Wakeup();
	}
}

static void sensor_power_off(void)
{
	struct sensor_platform_data *pdata;
	pdata = g_client->dev.platform_data;
	CoreISP3_SystemControl();
	if(pdata->power_off)
	{
		pdata->power_off(pdata->id);
	}
}
static void sensor_flash_ctl(int i)
{
	struct sensor_platform_data *pdata;
	pdata = g_client->dev.platform_data;
	if(pdata->flash_ctrl)
	{
		pdata->flash_ctrl(i);
	}
}

static void CLIhw_power_down( u8 powerMode )
{
	/* OV3640 PWRDWN, 0 = NORMAL, 1=POWER DOWN */
	if( powerMode == CAMERA_POWER_OFF )
		sensor_power_off();
	else
		sensor_power_on();

	msleep(30);
}

static void CLIhw_reset(void)
{
	return;
}

void sensor_flash_function(unsigned long data)
{	
	if(data==0)
	{
		//printk("FLASH!!!!!!!!!!!!!\n");
		sensor_flash_ctl(Cl_True);
		flash_timer.data = 1;
		mod_timer(&flash_timer, jiffies + HZ/5);	    //200ms;
	}
	else if(data==1)
	{
		//printk("FLASH OVER!!!!!!!!!!!!!\n");
		sensor_flash_ctl(Cl_False);
	}	
}


/*****************************************************************************
 * CLI6000  for linux device Functions
 *****************************************************************************/
int CLI6000_init( p_camera_context_t camera_context )
{

	DebugMessage(DM_JPEG,"CLI6000_init\n");
	//fang
	//msleep(1000);

	/* provide informat about the capabilities of the sensor */
	camera_context->sensor_status.caps |= SENSOR_CAP_MANUAL_CONTRAST |
		SENSOR_CAP_MANUAL_WHITEBALANCE |
		SENSOR_CAP_MANUAL_EXPOSURE;

	/* Configure CI according to CLI6000's hardware
	 * master parallel with 8 data pins
	 */
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8);
	
	/* enable pixel clock(sensor will provide pclock)
	 * and master clock = 26MHZ 
	 */
	/* software work around*/
	ci_set_clock(1, 1, CICLK);

	/* data sample on  falling, hsync active high,vsync active low */	
	ci_set_polarity(1, 0, 1);

	/* fifo control */
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_0_TO_7);


	/* CLI6000 Power on sequence
	 * Take out of Power down mode (GPIO_103), PWRDWN=1, NORMAL=0
	 * Assert Reset
	 * Delay
	 * Remove reset
	 * Delay
	 */
	CLIhw_power_down(CAMERA_POWER_FULL);
	CLIhw_reset();

	if(CoreISP3_Initialize(enDownloadMode_SkipedMCUBin)==Cl_False)
		return -EIO;

	init_timer(&flash_timer);

	//g_isFirstInitialized = Cl_True;	
	return 0;
}

int CLI6000_deinit(p_camera_context_t camera_context)
{
		//fang
		//edit for black bug
	g_zoom_rate = enISP_ZOOM_CIF;
	//g_zoom_rate = enISP_ZOOM_480_360;
#if defined(FEATURE_CS7903) 				
	g_faceTrackingOn = Cl_True;	
#else
	g_faceTrackingOn = Cl_False;	
#endif
	//g_isFirstInitialized = Cl_False;
	g_isFlash = Cl_False;

	/* power off the external module */
	CLIhw_power_down( CAMERA_POWER_OFF );
	del_timer(&flash_timer);

	return 0;
}

int CLI6000_sleep(  p_camera_context_t camera_context )
{
	return CLI6000_deinit( camera_context );
}

int CLI6000_wake(  p_camera_context_t camera_context )
{
	return CLI6000_init( camera_context );
}

int CLI6000_set_capture_format(  p_camera_context_t camera_context )
{
	enIsp3OutResolution outRes;
	CI_MP_TIMING timing;

	switch(camera_context->capture_input_format) 
	{			
		case V4L2_PIX_FMT_JPEG:
			timing.BFW = 0x00;
			timing.BLW = 0x00;
			break;
		case V4L2_PIX_FMT_YUV422P:		
			outRes = CoreISP3_SetSensorSize(camera_context->capture_input_width,camera_context->capture_input_height);
			//lcd is 480*320, 20+320+20=360, so wait 20 line clocks
			if(outRes==enISP_RES_480_360)
			{
				timing.BFW = 0x14;	
			}
			// first line of coreisp3 sensor outputed is error data, so discard one line . 
			else
			{
				timing.BFW = 0x01;	
			}
			timing.BLW = 0x00;
			break;
		default:
			break;
	}	
	ci_configure_mp(camera_context->capture_input_width-1, camera_context->capture_input_height-1, &timing);
	return 0;
}

int CLI6000_start_capture(p_camera_context_t camera_context, unsigned int frames)
{
	enIsp3OutResolution outRes;

	//fang
	DebugMessage(DM_JPEG, "===>%s\n",__FUNCTION__);
	switch(camera_context->capture_input_format) 
	{
		case V4L2_PIX_FMT_YUV422P:
			if(0 == zoom_flag)
			{
			DebugMessage(DM_JPEG, "para yuv 422\n");
			CoreISP3_Brightness_OnOff(Cl_True, 0x0);
			outRes = CoreISP3_SetSensorSize(camera_context->capture_input_width,camera_context->capture_input_height);
			//if(g_isFirstInitialized==Cl_False)
			{
				CoreISP3_OutpYCbCr();
				#if 0//def ISP3_ZOOM_ENABLE
				if(outRes==enISP_RES_QCIF)
				{
					ISP3_SetDigitalZoom(enISP_ZOOM_QCIF);
				}
				else if(outRes==enISP_RES_CIF)
				{
					ISP3_SetDigitalZoom(enISP_ZOOM_CIF);
				}				
				else if(outRes==enISP_RES_QVGA)
				{
					ISP3_SetDigitalZoom(enISP_ZOOM_QVGA);
				}
				else if(outRes==enISP_RES_VGA)
				{
					ISP3_SetDigitalZoom(enISP_ZOOM_VGA);
				}
				else//if(outRes==enISP_RES_480_360)
				{
					ISP3_SetDigitalZoom(g_zoom_rate);
				}
				#endif
				//lcm_dbg
				//CoreISP3_SetResolution(enISP_RES_VGA,CMD_Preview);
				ISP3_SetDigitalZoom(g_zoom_rate);
				ISP3_SetFocusPos(0);//保证每次对焦位置都在初始状态
			}
			//if((outRes==enISP_RES_480_360)&&(g_faceTrackingOn==Cl_True))
			if((outRes==enISP_RES_VGA)&&(g_faceTrackingOn==Cl_True))
			{
				CoreISP3_FaceTracking_On();
			}
			//lcm_dbg
			if((outRes==enISP_RES_QVGA)&&(g_faceTrackingOn==Cl_True))
			{
				CoreISP3_FaceTracking_Off();
			}
			
			if(outRes==enISP_RES_QVGA)
				ISP3_Set_QVGAPreview();
			//g_isFirstInitialized = Cl_False;
			CoreISP3_FlickerSuppression(FLICKER_50HZ_ON);
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
			CoreISP3_SetAutoExposure(enISP_FUNC_AE_ON);
			msleep(100);

			// for test
			//CoreISP3_SetAutoFocus(enISP_FUNC_AF_ON);
#if 0
			if(g_isFlash==Cl_True)
			{
				ISP3_Set_FlashOnOff(0);	//flash off
			}
			//CoreISP3_Get_ioctl(); //ZhouCS test			
#endif
			}
			else
				zoom_flag = 0;
			break;
			
		case V4L2_PIX_FMT_JPEG:
			DebugMessage(DM_JPEG, "para JPG\n");
			//CoreISP3_Set_StillStabilizer_On();
			CoreISP3_FaceTracking_Off();			
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_OFF);
			CoreISP3_SetAutoExposure(enISP_FUNC_AE_OFF);
			CoreISP3_OutpJPEG(Cl_False); // TRUE -> Thumbnail ON
			outRes = CoreISP3_SetSensorSize(camera_context->capture_input_width,camera_context->capture_input_height);
#if 0
			if(g_zoom_rate!=enISP_ZOOM_480_360)
				CoreISP3_I2C_Write(0xE660, 0x0043);    //ZOOMMODE_ON 0x43
			//read
			printk("sensor reg: 0x0202 = %x \n", CoreISP3_I2C_SensorRead(0x0202));
			printk("sensor reg: 0x0204 = %x \n", CoreISP3_I2C_SensorRead(0x0204));
#endif       
#if 0
			//fang
			//before capture
			printk("Before capture:sensor reg: 0xe010 = 0x%x \n", CoreISP3_I2C_Read(0xe010));
			printk("sensor reg: 0xe011 = 0x%x \n", CoreISP3_I2C_Read(0xe011));
			printk("sensor reg: 0xe012 = 0x%x \n", CoreISP3_I2C_Read(0xe012));
			printk("sensor reg: 0xe013 = 0x%x \n", CoreISP3_I2C_Read(0xe013));
			printk("sensor reg: 0xe014 = 0x%x \n", CoreISP3_I2C_Read(0xe014));
			printk("sensor reg: 0xe015 = 0x%x \n", CoreISP3_I2C_Read(0xe015));

			printk("sensor reg: 0xe050 = 0x%x \n", CoreISP3_I2C_Read(0xe050));
			printk("sensor reg: 0xe051 = 0x%x \n", CoreISP3_I2C_Read(0xe051));
			printk("sensor reg: 0xe058 = 0x%x \n", CoreISP3_I2C_Read(0xe058));
			printk("sensor reg: 0xe059 = 0x%x \n", CoreISP3_I2C_Read(0xe059));

			printk("sensor reg: 0xe060 = 0x%x \n", CoreISP3_I2C_Read(0xe060));
			printk("sensor reg: 0xe061 = 0x%x \n", CoreISP3_I2C_Read(0xe061));
			printk("sensor reg: 0xe062 = 0x%x \n", CoreISP3_I2C_Read(0xe062));
			printk("sensor reg: 0xe063 = 0x%x \n", CoreISP3_I2C_Read(0xe063));

			printk("sensor reg: 0xe649 = 0x%x \n", CoreISP3_I2C_Read(0xe649));
			printk("sensor reg: 0xe64a = 0x%x \n", CoreISP3_I2C_Read(0xe64a));
			printk("sensor reg: 0xe64b = 0x%x \n", CoreISP3_I2C_Read(0xe64b));

			printk("sensor reg: 0xe672 = 0x%x \n", CoreISP3_I2C_Read(0xe672));
			printk("sensor reg: 0xe651 = 0x%x \n", CoreISP3_I2C_Read(0xe651));
			printk("sensor reg: 0xe652 = 0x%x \n", CoreISP3_I2C_Read(0xe652));
			printk("sensor reg: 0xe663 = 0x%x \n", CoreISP3_I2C_Read(0xe663));

			printk("sensor reg: 0xec04 = 0x%x \n", CoreISP3_I2C_Read(0xec04));
			printk("sensor reg: 0xec05 = 0x%x \n", CoreISP3_I2C_Read(0xec05));
			printk("sensor reg: 0xec06 = 0x%x \n", CoreISP3_I2C_Read(0xec06));
			printk("sensor reg: 0xec07 = 0x%x \n", CoreISP3_I2C_Read(0xec07));
			printk("sensor reg: 0xec08 = 0x%x \n", CoreISP3_I2C_Read(0xec08));
			printk("sensor reg: 0xec09 = 0x%x \n", CoreISP3_I2C_Read(0xec09));
			printk("sensor reg: 0xec0a = 0x%x \n", CoreISP3_I2C_Read(0xec0a));
			printk("sensor reg: 0xec0b = 0x%x \n", CoreISP3_I2C_Read(0xec0b));
			printk("sensor reg: 0xec0c = 0x%x \n", CoreISP3_I2C_Read(0xec0c));
			printk("sensor reg: 0xec0d = 0x%x \n", CoreISP3_I2C_Read(0xec0d));
#endif
			CoreISP3_SetResolution(outRes, CMD_Capture);
#if 0
			//read
			printk("sensor reg: 0x0202 = %x \n", CoreISP3_I2C_SensorRead(0x0202));
			printk("sensor reg: 0x0204 = %x \n", CoreISP3_I2C_SensorRead(0x0204));
			if(g_zoom_rate!=enISP_ZOOM_480_360)
				 CoreISP3_I2C_Write(0xE660, 0x0044);    //ZOOMMODE_OFF 0x44

			msleep(100);
			if(g_isFlash==Cl_True)
			{
				ISP3_Set_FlashOnOff(1);	//flash on
				flash_timer.function = sensor_flash_function;
				flash_timer.expires = jiffies + HZ/10;
				flash_timer.data = 0;
				add_timer(&flash_timer);	    
			}
#endif
			//udelay(100);
			break;
		
		default:
			DebugMessage(DM_JPEG, "para defaut yuv 422\n");
			break;
	}

	return 0;
}

int CLI6000_stop_capture(  p_camera_context_t camera_context )
{
	return 0;
}

int CLI6000_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	CLIhw_power_down( mode );
	return 0;
}

int CLI6000_set_contrast( p_camera_context_t camera_context, u8 mode, u32 value)
{
	ClUint_8 level = enISP_Level_Default;

	DebugMessage(DM_JPEG, "\n====> %s:: value=%d\n", __func__, value);
	
	//if (mode == SENSOR_MANUAL_CONTRAST)
	{
		switch (value)
		{
			case 0: 
				level = 0x20;
				break;
			case 1: 
				level = 0x30;
				break;
			case 2: 
				level = 0x40;
				break;
			case 3: 
				level = 0x60;
				break;
			case 4: 
				level = 0x90;
				break;
			default:
				 level = 0x40;
				 break;
		}
		CoreISP3_Contrast_OnOff(Cl_True, level);
	}
	//else if(mode == SENSOR_AUTO_CONTRAST)
	//else
	//{
	//	CoreISP3_Contrast_OnOff(Cl_True, level);	
	//}		
	return 0;
}

int CLI6000_set_brightness( p_camera_context_t camera_context, u8 mode, u32 value)
{
	ClUint_8 level = enISP_Level_Default;

	DebugMessage(DM_JPEG, "\n====> %s:: value=%d\n", __func__, value);

	//if (mode == Cl_True)
	{
		switch (value)
		{
			case 0: 
				level = 0xc0;
				break;
			case 1: 
				level = 0xa0;
				break;
			case 2: 
				level = 0;
				break;
			case 3: 
				level = 0x20;
				break;
			case 4: 
				level = 0x40;
				break;
			default:
				 level = 0;
				 break;
		}
		//CoreISP3_SetBrightness(level);
		CoreISP3_Brightness_OnOff(Cl_True, level);

	}
	//else
	//{
	//	CoreISP3_Brightness_OnOff(Cl_False, 0);
	//}
	return 0;
}

int CLI6000_set_exposure( p_camera_context_t camera_context, u8 mode, u32 value)
{
	ClUint_16 ev_value = 0;
	
	if(mode==SENSOR_MANUAL_EXPOSURE)
	{
		switch(value)
		{
			case 1:
				ev_value = 0;
				break;
			case 2:
				ev_value = 2;
				break;
			case 3:
				ev_value = 4;
				break;
			case 4:
				ev_value = 6;
				break;
			case 5:
				ev_value = 8;
				break;
			default:
				ev_value = 0;
				break;
		}
		CoreISP3_Set_AE_EV(ev_value);	
	}
	else
	{
		CoreISP3_SetAutoExposure(ISP3_CMD_AE_ON);	
	}
	return 0;
}

#if 0
int CLI6000_set_evoffset( p_camera_context_t camera_context, int value)
{
	ClUint_16 ev_value = 0;
	
	printk("%s,value=%x\n", __FUNCTION__,value);
	if(value<=-100)
	{
		ev_value = 0;
	}
	else if(value<=-70)
	{
		ev_value = 1;
	}
	else if(value<=-30)
	{
		ev_value = 2;
	}
	else if(value<=0)
	{
		ev_value = 3;
	}
	else if(value<=30)
	{
		ev_value = 4;
	}
	else if(value<=70)
	{
		ev_value = 5;
	}
	else if(value<=100)
	{
		ev_value = 6;
	}
	else if(value<=130)
	{
		ev_value = 7;
	}
	else
	{
		ev_value = 0;
	}
	CoreISP3_Set_AE_EV(ev_value);	

	return 0;

}
#else
int CLI6000_set_evoffset( p_camera_context_t camera_context, int value)
{
	ClUint_8 level = enISP_Level_Default;

	DebugMessage(DM_JPEG, "\n====> %s:: value=%d\n", __func__, value);

	//if (mode == Cl_True)
	{
		switch (value)
		{
			case -199: 
				level = 0xc0;
				break;
			case -169: 
				level = 0xc0;
				break;
			case -129: 
				level = 0xc0;
				break;
			case -99: 
				level = 0xa0;
				break;
			case -69: 
				level = 0xa0;
				break;
			case -29: 
				level = 0x0;
				break;
			case 0: 
				level = 0x0;
				break;
			case 30: 
				level = 0x0;
				break;
			case 70: 
				level = 0x20;
				break;
			case 100: 
				level = 0x20;
				break;
			case 130: 
				level = 0x40;
				break;
			case 170: 
				level = 0x40;
				break;
			case 200: 
				level = 0x40;
				break;
			default:
				 level = 0x0;
				 break;
		}
		//CoreISP3_SetBrightness(level);
		CoreISP3_Brightness_OnOff(Cl_True, level);

	}
	//else
	//{
	//	CoreISP3_Brightness_OnOff(Cl_False, 0);
	//}
	return 0;
}
#endif
int CLI6000_set_quality( p_camera_context_t camera_context, u32 value)
{
	DebugMessage(DM_JPEG, "====> %s::%d\n", __func__, value);
	
	switch (value)
	{
		case 0:
			value = 25; //Middle
			break;
		case 1:
			value = 20; //High
			break;
		case 2:
			value = 28; //low
			break;
		case 3:
			value = 18; //Fine
			break;
		defalut:
			value = 25;
			break;
	}
	CoreISP3_I2C_Write(0xee15, value);
	return 0 ;
}
int CLI6000_set_scene( p_camera_context_t camera_context, u32 value)
{
	enIsp3SceneMode mode;
	switch(value)
	{
		case 1:
			//normal, coreisp not supported
			break;
			
		case 2:
			mode = enISP_Scene_Portrait;
			CoreISP3_FaceTracking_On();
			g_faceTrackingOn = Cl_True;
			break;
			
		case 3:
			mode = enISP_Scene_Landscape;
			CoreISP3_FaceTracking_Off();
			g_faceTrackingOn = Cl_False;
			break;
			
		case 4:
			mode = enISP_Scene_Sports;
			CoreISP3_FaceTracking_Off();			
			g_faceTrackingOn = Cl_False;
			break;

		case 5:
			mode = enISP_Scene_Night;
			CoreISP3_FaceTracking_Off();			
			g_faceTrackingOn = Cl_False;
			break;			
		case 7:
			mode = enISP_Scene_Sunset;
			CoreISP3_FaceTracking_Off();			
			g_faceTrackingOn = Cl_False;
			break;			
		case 11:
			mode = enISP_Scene_Snow;
			CoreISP3_FaceTracking_Off();			
			g_faceTrackingOn = Cl_False;			
			break;		
		case 0:			
		default:
			mode = enISP_Scene_Auto;
			CoreISP3_FaceTracking_Off();
			g_faceTrackingOn = Cl_False;
			break;			
	}
	
	CoreISP3_SetSceneMode(mode);
	return 0;
}

int CLI6000_set_effect( p_camera_context_t camera_context, u32 value)
{
	enISPFunctionsImageEffect effectParam;
	switch(value)
	{
		case 0:
			effectParam = enISP_FUNC_IMAGE_NORMAL;
			break;
			
		case 3:
		//	effectParam = enISP_FUNC_IMAGE_BLACK_WHITE;//噪点较重
			effectParam = enISP_FUNC_IMAGE_GRAY;
			break;
			
		case 5:
			effectParam = enISP_FUNC_IMAGE_OPPOSITE_NEGATIVE;
			break;
			
		case 4:
			effectParam = enISP_FUNC_IMAGE_OLD_MOVIE;
			break;
			
		case 7:
			effectParam = enISP_FUNC_IMAGE_EMBOSS;
			break;
			
		default:
			effectParam = enISP_FUNC_IMAGE_NORMAL;
			break;			
	}
	
	CoreISP3_SetImageEffect(effectParam);
	return 0;
}

int CLI6000_set_zoom( p_camera_context_t camera_context, u32 value)
{
	#if 0//def ISP3_ZOOM_ENABLE
	if(value == 100)
		g_zoom_rate = enISP_ZOOM_480_360;
	else if(value<= 140)
		g_zoom_rate = enISP_ZOOM_480_360X2;
	else if(value<= 170)
		g_zoom_rate = enISP_ZOOM_480_360X3;
	else if(value<= 200)
		g_zoom_rate = enISP_ZOOM_480_360X4;
	else
		g_zoom_rate = enISP_ZOOM_480_360;
	
	ISP3_SetDigitalZoom(g_zoom_rate);
	#else
	DebugMessage(DM_JPEG, "zoom value: %d \n", value);
	g_zoom_rate = (value-100)/10;
	//fang
	if(g_zoom_rate == 0)
	{
	//	printk("Send Command 0x44\n");
		CoreISP3_Send_Command(0x44);	// zoom off
	}
	else
	{
	//	printk("Send Command 0x43\n");
		CoreISP3_Send_Command(0x43);	// zoom on
	}
	ISP3_SetDigitalZoom(g_zoom_rate);
	zoom_flag = 1;
	#endif
	return 0;
}

int CLI6000_set_white_balance( p_camera_context_t camera_context, u8 mode, u32 value)
{

	switch(value)
	{
		case 0:
			CoreISP3_SetAWBMode(AWB_AUTO);
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
			break;
		case 1:
			CoreISP3_SetAWBMode(AWB_TUNGSTEN);		
			//白炽灯
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
			break;
		case 2:
			CoreISP3_SetAWBMode(AWB_SUNNY);
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
			break;
		case 3:
			CoreISP3_SetAWBMode(AWB_FLOCSNET);
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
			break;
		case 4:
			CoreISP3_SetAWBMode(AWB_CLOUDY);
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
			break;			
		case 10:
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_OFF);
			break;
		default:
			CoreISP3_SetAWBMode(AWB_AUTO);
			CoreISP3_SetAutoWhiteBalance(enISP_FUNC_AWB_ON);
			break;			
	}

	return 0;
}

int CLI6000_get_framerate(u32 format, u32 width, u32 height, u32 *numerator, u32 *denominator)
{
	
	return 0;
}

int CLI6000_set_framerate(u32 numerator, u32 denominator)
{
	return 0;
}

int CLI6000_set_flashmode( p_camera_context_t camera_context, u8 mode, u32 value)
{
	if(value)
	{
		g_isFlash = Cl_True;	
	}
	else
	{
		g_isFlash = Cl_False;
	}
	return 0;
}

int CLI6000_set_autofocus( p_camera_context_t camera_context, u8 mode, u32 value)
{
	CoreISP3_SetAutoFocus(enISP_FUNC_AF_ON);
	return 0;
}

void CoreISP3_Get_ioctl(void)
{
    	DebugMessage(DM_JPEG, "contrast=%x\n", CoreISP3_I2C_Read(0xE5E0));
    	DebugMessage(DM_JPEG, "SetBrightness=%x\n", CoreISP3_I2C_Read(0xE5E1));
    	DebugMessage(DM_JPEG, "SetSceneMode=%x\n", CoreISP3_I2C_Read(0xE628));
    	DebugMessage(DM_JPEG, "SetAWBMode=%x\n", CoreISP3_I2C_Read(0xEb98));
    	DebugMessage(DM_JPEG, "SetSaturation=%x\n", CoreISP3_I2C_Read(0xE5B0));
    	DebugMessage(DM_JPEG, "GetFocusPos=%x\n", ISP3_GetFocusPos());
}



//I2C driver
static char name[] = "CLI6000";

static p_camera_function_t camera_functions;

static int format_list[] = {

	V4L2_PIX_FMT_YUV422P,
	
	V4L2_PIX_FMT_JPEG, 
	
	-1	
};

static int CLI6000_register(int id)
{

	/* allocte camera functions context */
	camera_functions = kzalloc(sizeof(camera_function_t), GFP_KERNEL);
	if (!camera_functions) {
		printk("Can't allocate buffer for camera functions structure \n");
		return -ENOMEM;
	}

	camera_functions->width_max = 2560;
	camera_functions->height_max = 1920;
	camera_functions->width_min = 2;
	camera_functions->height_min = 2;

	camera_functions->v_power = -1;
	camera_functions->v_io = -1;

	camera_functions->format_list = format_list;

	camera_functions->init = CLI6000_init;     	
	camera_functions->deinit = CLI6000_deinit;     
	camera_functions->set_capture_format =	CLI6000_set_capture_format;
	camera_functions->start_capture =	CLI6000_start_capture;
	camera_functions->stop_capture =	CLI6000_stop_capture;
	camera_functions->sleep =		CLI6000_sleep;
	camera_functions->wakeup =		CLI6000_wake;
	camera_functions->read_8bit =		NULL;			
	camera_functions->write_8bit =		NULL;
	camera_functions->read_16bit =		NULL;
	camera_functions->write_16bit =		NULL;
	camera_functions->read_32bit =		NULL;
	camera_functions->write_32bit =		NULL;
	camera_functions->set_power_mode = 	CLI6000_set_power_mode;
	camera_functions->set_contrast =	CLI6000_set_contrast;
	//camera_functions->set_brightness = CLI6000_set_brightness;	
	camera_functions->set_whitebalance =	CLI6000_set_white_balance;
	camera_functions->set_exposure =	CLI6000_set_exposure;
	camera_functions->set_ev_offset=	CLI6000_set_evoffset;
	camera_functions->set_jpeg_quality=	CLI6000_set_quality;
	camera_functions->set_scene = CLI6000_set_scene;
	camera_functions->set_zoom = CLI6000_set_zoom;
	camera_functions->set_digital_zoom = CLI6000_set_zoom;
	//camera_functions->set_effect = CLI6000_set_effect;
	camera_functions->set_colortone = CLI6000_set_effect;
	
	camera_functions->get_framerate =	CLI6000_get_framerate;
	camera_functions->set_framerate =	CLI6000_set_framerate;
	camera_functions->set_autofocus =	CLI6000_set_autofocus;
	camera_functions->set_flashmode =	CLI6000_set_flashmode;

	//fang
	camera_functions->download_firmware = CLI6000_download_firmware;

	camera_functions->name = name;
	camera_functions->id = id;

	/*register CLI6000 as high resolution */

	if(sensor_register(camera_functions, id) < 0){
		printk("sensor_register failed !\n");
		kfree(camera_functions);
		camera_functions = NULL;
		return -1;
	}
	return 0;
}

static int CLI6000_unregister(int id)
{
	sensor_unregister(id);
	
	if(!camera_functions){
		kfree(camera_functions);
		camera_functions = NULL;
	}
	return 0;
}
static int CLI6000_sensorID_detect(void)
{
	unsigned int sensorID;
//	CoreISP3_Initialize(enDownloadMode_StackedMem);

	//CoreISP3_I2C_Write(0xe063, 2);
	sensorID = CoreISP3_I2C_Read(0xe060);
	if(3 == sensorID)
	{
		DebugMessage(DM_JPEG, "CLI6000_sensorID_detect\n");
		return 0;
	}
	else
		return -1;
}

static ssize_t coreispattr_flashled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_isFlash);
}

static ssize_t coreispattr_flashled_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	int val;
	sscanf(buf, "%x", &val);
	if( val<0 )
        	printk(KERN_ERR "invalid input\n");
	else 
	{
		g_isFlash = val;

		if(g_isFlash==Cl_True)
		{
			init_timer(&flash_timer);
		
			ISP3_Set_FlashOnOff(1);	//flash on
			flash_timer.function = sensor_flash_function;
			flash_timer.expires = jiffies + HZ/10;
			flash_timer.data = 0;
			add_timer(&flash_timer);	    
		}

	}
	return n;
}

static DEVICE_ATTR(flashled, S_IRUGO|S_IWUSR, coreispattr_flashled_show, coreispattr_flashled_store);

static struct attribute *coreisp_sysfs_attr[] = {
	&dev_attr_flashled.attr,
	NULL
};

static int CLI6000_sysfs_add(struct kobject *kobj)
{
	int i, n, ret;
	n = ARRAY_SIZE(coreisp_sysfs_attr);
	for (i = 0; i < n; ++i) {
		if( coreisp_sysfs_attr[i] ) {
			ret = sysfs_create_file(kobj, coreisp_sysfs_attr[i]);
			if (ret)
				return ret;
		}
	}
	return 0;
}

static int CLI6000_sysfs_rm(struct kobject *kobj)
{
	int i, n;
	n = ARRAY_SIZE(coreisp_sysfs_attr);
	for (i = 0; i < n; i++) {
		if( coreisp_sysfs_attr[i] )
			sysfs_remove_file(kobj, coreisp_sysfs_attr[i]);
	}
	return 0;
}

#define CAM_1P8V_PIN         mfp_to_gpio(MFP_PIN_GPIO7)


static int __devinit sensor_probe(struct i2c_client *client)
{
	struct sensor_platform_data *pdata;
	struct clk *clk = NULL;
	int id = 0;
	
	pdata = client->dev.platform_data;
	id = pdata->id;

	clk = clk_get(NULL, "CAMCLK");
	if (IS_ERR(clk)) {
		printk("sensor failed to get camera clock\n");
		return -1;
	}
	clk_enable(clk);
	
	ci_set_clock(1, 1, CICLK);
	
	g_client = client;

	pdata->power_on(id);
	
	mdelay(1);

/******add for power control****/
	//CoreISP3_SystemControl(1);
	//gpio_direction_output(CAM_1P8V_PIN, 0);
/******add for power control****/
	
	
	if (CLI6000_sensorID_detect()<0){
		goto err;	// TODO: remove for debugging
	}

	if(CLI6000_register(id)<0)
		goto err;

	CoreISP3_SystemControl();
	pdata->power_off(id);
	ci_set_clock(0, 0, CICLK);

	clk_disable(clk);
	clk_put(clk);

	CLI6000_sysfs_add(&client->dev.kobj);

	return 0;
err:
	printk("CLI6000 detect failed.\n");
	ci_set_clock(0, 0, CICLK);
	pdata->power_off(id);

	if(clk){
		clk_disable(clk);
		clk_put(clk);

	}
	return -1;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	
	CLI6000_unregister(pdata->id);
	g_client = NULL;
	CLI6000_sysfs_rm(&client->dev.kobj);

	return 0;
}


static struct i2c_driver sensor_driver = {
	.driver = {
		.name	= "CLI6000",
	},
	.probe		= sensor_probe,
	.remove		= sensor_remove,
};

static int __init sensor_init(void)
{
	return i2c_add_driver(&sensor_driver);
}

static void __exit sensor_exit(void)
{
	i2c_del_driver(&sensor_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_DESCRIPTION("CLI6000 I2C Client driver");
MODULE_LICENSE("GPL");


