/***

Author: WHD
Function: A FM radio.

***/


#include <stdio.h>
#include <string.h>
#include	"config.h"
#include  <math.h>    //Keil library
#include	"binary.h"
#include	"USART1.h"
#include	"src\delay.h"
#include	"src\OLED.h"
#include	"src\BMP_BACKGROUND.h"


//#include "stdlib.h"

#define	COM_TX1_Lenth	20
#define	COM_RX1_Lenth	20

COMx_Define	COM1;
u8	idata TX1_Buffer[COM_TX1_Lenth];	//发送缓冲
u8 	xdata RX1_Buffer[COM_RX1_Lenth];	//接收缓冲




sbit LED=P2^5;

u8 BUTTON1 = 0;
u8 BUTTON2 = 0;
u8 BUTTON3 = 0;

// button & status machine
u8 BUTTON1_DELAY= 1;
u8 BUTTON2_DELAY= 1;
u8 BUTTON3_DELAY= 1;
bit BUTTON1_TRIG = 0;
bit BUTTON2_TRIG = 0;
bit BUTTON3_TRIG = 0;

u8 timer3_cnt=0;
u8 led_cnt=0;
u16 ADC_RESULT = 0;
u16 BUTTON_ADC_RESULT = 0;


/*************	本地函数声明 local function statement	**************/
u8 USART_Configuration(u8 UARTx, COMx_InitDefine *COMx)
{
	u16	i;
	u32	j;
	
	if(UARTx == USART1)
	{
		COM1.id = 1;
		COM1.TX_read    = 0;
		COM1.TX_write   = 0;
		COM1.B_TX_busy  = 0;
		COM1.RX_Cnt     = 0;
		COM1.RX_TimeOut = 0;
		COM1.B_RX_OK    = 0;
		for(i=0; i<COM_TX1_Lenth; i++)	TX1_Buffer[i] = 0;
		for(i=0; i<COM_RX1_Lenth; i++)	RX1_Buffer[i] = 0;

		if(COMx->UART_Mode > UART_9bit_BRTx)	return 1;	//模式错误
		if(COMx->UART_Polity == PolityHigh)		PS = 1;	//高优先级中断
		else									PS = 0;	//低优先级中断
		SCON = (SCON & 0x3f) | COMx->UART_Mode;
		if((COMx->UART_Mode == UART_9bit_BRTx) ||(COMx->UART_Mode == UART_8bit_BRTx))	//可变波特率
		{
			j = (MAIN_Fosc / 4) / COMx->UART_BaudRate;	//按1T计算
			if(j >= 65536UL)	return 2;	//错误
			j = 65536UL - j;
			if(COMx->UART_BRT_Use == BRT_Timer1)
			{
				TR1 = 0;
				AUXR &= ~0x01;		//S1 BRT Use Timer1;
				TMOD &= ~(1<<6);	//Timer1 set As Timer
				TMOD &= ~0x30;		//Timer1_16bitAutoReload;
				AUXR |=  (1<<6);	//Timer1 set as 1T mode
				TH1 = (u8)(j>>8);
				TL1 = (u8)j;
				ET1 = 0;	//禁止中断
				TMOD &= ~0x40;	//定时
				INT_CLKO &= ~0x02;	//不输出时钟
				TR1  = 1;
			}
			else if(COMx->UART_BRT_Use == BRT_Timer2)
			{
				AUXR &= ~(1<<4);	//Timer stop
				AUXR |= 0x01;		//S1 BRT Use Timer2;
				AUXR &= ~(1<<3);	//Timer2 set As Timer
				AUXR |=  (1<<2);	//Timer2 set as 1T mode
				TH2 = (u8)(j>>8);
				TL2 = (u8)j;
				IE2  &= ~(1<<2);	//禁止中断
				AUXR &= ~(1<<3);	//定时
				AUXR |=  (1<<4);	//Timer run enable
			}
			else return 2;	//错误
		}
		else if(COMx->UART_Mode == UART_ShiftRight)
		{
			if(COMx->BaudRateDouble == ENABLE)	AUXR |=  (1<<5);	//固定波特率SysClk/2
			else								AUXR &= ~(1<<5);	//固定波特率SysClk/12
		}
		else if(COMx->UART_Mode == UART_9bit)	//固定波特率SysClk*2^SMOD/64
		{
			if(COMx->BaudRateDouble == ENABLE)	PCON |=  (1<<7);	//固定波特率SysClk/32
			else								PCON &= ~(1<<7);	//固定波特率SysClk/64
		}
		if(COMx->UART_Interrupt == ENABLE)	ES = 1;	//允许中断
		else								ES = 0;	//禁止中断
		if(COMx->UART_RxEnable == ENABLE)	REN = 1;	//允许接收
		else								REN = 0;	//禁止接收
		P_SW1 = (P_SW1 & 0x3f) | (COMx->UART_P_SW & 0xc0);	//切换IO
		if(COMx->UART_RXD_TXD_Short == ENABLE)	PCON2 |=  (1<<4);	//内部短路RXD与TXD, 做中继, ENABLE,DISABLE
		else									PCON2 &= ~(1<<4);
		return	0;
	}
	return 3;	//其它错误
}


/*************** 装载串口发送缓冲 *******************************/

void TX1_write2buff(u8 dat)	//写入发送缓冲，指针+1
{
	TX1_Buffer[COM1.TX_write] = dat;	//装发送缓冲
	if(++COM1.TX_write >= COM_TX1_Lenth)	COM1.TX_write = 0;

	if(COM1.B_TX_busy == 0)		//空闲
	{  
		COM1.B_TX_busy = 1;		//标志忙
		TI = 1;					//触发发送中断
	}
}

void PrintString1(u8 *puts)
{
    for (; *puts != 0;	puts++)  TX1_write2buff(*puts); 	//遇到停止符0结束
}


/********************* UART1中断函数************************/
void UART1_int (void) interrupt UART1_VECTOR
{
	if(RI)
	{
		RI = 0;
		if(COM1.B_RX_OK == 0)
		{
			if(COM1.RX_Cnt >= COM_RX1_Lenth)	COM1.RX_Cnt = 0;
			RX1_Buffer[COM1.RX_Cnt++] = SBUF;
			COM1.RX_TimeOut = TimeOutSet1;
		}
	}

	if(TI)
	{
		TI = 0;
		if(COM1.TX_read != COM1.TX_write)
		{
		 	SBUF = TX1_Buffer[COM1.TX_read];
			if(++COM1.TX_read >= COM_TX1_Lenth)		COM1.TX_read = 0;
		}
		else	COM1.B_TX_busy = 0;
	}
}

/*************  外部函数和变量声明 *****************/
void Timer3Init(void)		//1??@24.000MHz
{
	T4T3M |= 0x02;		//?????1T??
	T3L = 0x40;		//??????
	T3H = 0xA2;		//??????
	T4T3M |= 0x08;		//???3????
	IE2 |= B00100000;
}

u16 read_adc(u8 P_NUM)
{

	P1ASF = 1<<P_NUM;
	ADC_CONTR = B11100000 + P_NUM;
	CLK_DIV &=  B11011111;
	ADC_CONTR = B11101000+ P_NUM;
	delay_us(10);
	while(!(ADC_CONTR&0x10))
	{delay_us(10);}
	ADC_CONTR = B11100000+ P_NUM;
						
	ADC_RESULT = ADC_RES;
	ADC_RESULT<<=2;
	ADC_RESULT+=ADC_RESL;
	ADC_RES = 0;
	ADC_RESL = 0;
	return ADC_RESULT;
}

void Timer3_Init (void) interrupt TIMER3_VECTOR
{
	u16	i;
	EA = 0;	
	timer3_cnt++;
	if(timer3_cnt == 10)
	{
		timer3_cnt = 0;
		led_cnt++;
		if(led_cnt==20){led_cnt=0;LED =! LED;}
		
		BUTTON_ADC_RESULT = read_adc(4);
		
		if(BUTTON_ADC_RESULT>>2<10){BUTTON1 = 0;}else{BUTTON1 = 1;}
		if(BUTTON_ADC_RESULT>>2>100 && BUTTON_ADC_RESULT>>2<150){BUTTON2 = 0;}else{BUTTON2 = 1;}
		if(BUTTON_ADC_RESULT>>2>160 && BUTTON_ADC_RESULT>>2<200){BUTTON3 = 0;}else{BUTTON3 = 1;}
		
		if(BUTTON1_DELAY == B11110000){BUTTON1_TRIG = 1;}else{BUTTON1_TRIG = 0;}
		if(BUTTON2_DELAY == B11110000){BUTTON2_TRIG = 1;}else{BUTTON2_TRIG = 0;}
		if(BUTTON3_DELAY == B11110000){BUTTON3_TRIG = 1;}else{BUTTON3_TRIG = 0;}
		
		BUTTON1_DELAY = (BUTTON1_DELAY<<1) + BUTTON1;
		BUTTON2_DELAY = (BUTTON2_DELAY<<1) + BUTTON2;
		BUTTON3_DELAY = (BUTTON3_DELAY<<1) + BUTTON3;
		
		OLED_ShowNum(0,6,BUTTON_ADC_RESULT,4,16);
		
	}			
	EA = 1; 
}

sbit SCL =P2^6; 
sbit SDA =P2^7; 

void I2C_START()
{
SDA = 1;
SCL = 1;
delay_us(100);
SDA = 0;
delay_us(10);
SCL = 0;
delay_us(10);
}
void I2C_STOP()
{
SDA = 0;
SCL = 0;
delay_us(10);
SCL = 1;
delay_us(10);
SDA = 0;
delay_us(10);

}
void I2C_BIT_SEND(bit value_bit)
{
SDA = value_bit;
delay_us(10);
SCL = 1;
delay_us(10);
SCL = 0;
delay_us(10);
}
void I2C_ACKW()
{
	I2C_BIT_SEND(1);
}
void I2C_ACKR()
{
	I2C_BIT_SEND(0);
}

void I2C_WRITE_BYTE(u8 byte_in)
{
u8 i=0;
u8 temp=0;
temp = byte_in;
for(i=0;i<8;i++)
{
	if(temp&0x80){I2C_BIT_SEND(1);}else{I2C_BIT_SEND(0);}
	temp<<=1;
}

}


/**********************************************/
void main(void)
{
	u16	i;
	u8 k = 0;
	u8 j = 0;
	u16 fre = 6;
	
	EA = 1;
	
	
	P1M1 = B00000000;
	P1M0 = B00100000;
	
	P2M1 = B10001010;
	P2M0 = B11111111;
	
	P3M1 = B00001000;
	P3M0 = B11111100;

	P5M1 = B00010000;
	P5M0 = B00110000;
	
	OLED_Init();			//初始化OLED  
	OLED_Clear(); 
	OLED_Clear();
		
	OLED_DrawBMP(0,0,128,8,BIT_LOGO);

	delay_ms(2000);
	OLED_Clear();

	OLED_ShowCHinese(16,0,0);
	OLED_ShowCHinese(32,0,1);
	OLED_ShowCHinese(48,0,2);
	OLED_ShowCHinese(64,0,3);
	OLED_ShowCHinese(80,0,4);
	OLED_ShowCHinese(96,0,5);
	
	OLED_ShowCHinese(0,2,6);
	OLED_ShowCHinese(16,2,7);
	OLED_ShowCHinese(32,2,8);
	OLED_ShowCHinese(48,2,9);
	OLED_ShowCHinese(64,2,10);
	OLED_ShowCHinese(80,2,11);
	OLED_ShowCHinese(96,2,12);
	OLED_ShowCHinese(112,2,13);




	UART_config();
	Timer3Init();		
	
	I2C_START();
	I2C_WRITE_BYTE(0x20);
	I2C_ACKW();
	I2C_WRITE_BYTE(0xD0);
	I2C_ACKW();
	I2C_WRITE_BYTE(0x01);
	I2C_ACKW();

	I2C_WRITE_BYTE((fre>>2)&0x00FF);
	I2C_ACKW();
	
	I2C_WRITE_BYTE(((fre&0x0003)<<6) | 0x10);
	I2C_ACKW();

	I2C_WRITE_BYTE(0x00);
	I2C_ACKW();
	I2C_WRITE_BYTE(0x40);
	I2C_ACKW();

	I2C_WRITE_BYTE(0x90);
	I2C_ACKW();
	I2C_WRITE_BYTE(0x80);
	I2C_ACKW();	
	
	I2C_STOP();

	while (1)
	{
		if(COM1.RX_TimeOut > 0)		//超时计数
		{
			if(--COM1.RX_TimeOut == 0)
			{
				if(COM1.RX_Cnt > 0)
				{
					 for(i=0;i<COM1.RX_Cnt;i++)
					  {
					   		TX1_write2buff(RX1_Buffer[i]);
					  }					  
				}
				COM1.RX_Cnt = 0;
			}
		}
		delay_ms(1000);
		k++;
		TX1_write2buff(k);

					
	}
}
