/***

Author: WHD
Function: A Heart rate monitor and Pedometer

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

#define	COM_TX1_Lenth	20
#define	COM_RX1_Lenth	20

COMx_Define	COM1;
u8	idata TX1_Buffer[COM_TX1_Lenth];	//????
u8 	xdata RX1_Buffer[COM_RX1_Lenth];	//????


sbit LED=P2^5;

sbit E1=P1^3; 
sbit E2=P1^2;

sbit HAL=P1^1;
//????
int MENU_1=0;
int MENU_2=0;
int MENU_3=0;

float reat=0;		 //??
int reat_count=0;//??????
int beat=0;	 	 //????????
bit fl;			 //??????
u8 fl_TRIG=0;	 //????????
int level=600;	 //?????
int count=0;
u8 rem1_DELAY=1;  //????????
//u8 rem2_DELAY=1;//????????
float cou1=0;	  //??????
//int cou2=0;	  //????????
int cspd=0;		  //??????
float speed=0;	  //??

int HAL_count=0;  //??????????

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

/*************	??????	**************/
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

		if(COMx->UART_Mode > UART_9bit_BRTx)	return 1;	//????
		if(COMx->UART_Polity == PolityHigh)		PS = 1;	//??????
		else									PS = 0;	//??????
		SCON = (SCON & 0x3f) | COMx->UART_Mode;
		if((COMx->UART_Mode == UART_9bit_BRTx) ||(COMx->UART_Mode == UART_8bit_BRTx))	//?????
		{
			j = (MAIN_Fosc / 4) / COMx->UART_BaudRate;	//?1T??
			if(j >= 65536UL)	return 2;	//??
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
				ET1 = 0;	//????
				TMOD &= ~0x40;	//??
				INT_CLKO &= ~0x02;	//?????
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
				IE2  &= ~(1<<2);	//????
				AUXR &= ~(1<<3);	//??
				AUXR |=  (1<<4);	//Timer run enable
			}
			else return 2;	//??
		}
		else if(COMx->UART_Mode == UART_ShiftRight)
		{
			if(COMx->BaudRateDouble == ENABLE)	AUXR |=  (1<<5);	//?????SysClk/2
			else								AUXR &= ~(1<<5);	//?????SysClk/12
}
		else if(COMx->UART_Mode == UART_9bit)	//?????SysClk*2^SMOD/64
		{
			if(COMx->BaudRateDouble == ENABLE)	PCON |=  (1<<7);	//?????SysClk/32
			else								PCON &= ~(1<<7);	//?????SysClk/64
		}
		if(COMx->UART_Interrupt == ENABLE)	ES = 1;	//????
		else								ES = 0;	//????
		if(COMx->UART_RxEnable == ENABLE)	REN = 1;	//????
		else								REN = 0;	//????
		P_SW1 = (P_SW1 & 0x3f) | (COMx->UART_P_SW & 0xc0);	//??IO
		if(COMx->UART_RXD_TXD_Short == ENABLE)	PCON2 |=  (1<<4);	//????RXD?TXD, ???, ENABLE,DISABLE
		else									PCON2 &= ~(1<<4);
		return	0;
	}
	return 3;	//????
}


/*************** ???????? *******************************/

void TX1_write2buff(u8 dat)	//??????,??+1
{
	TX1_Buffer[COM1.TX_write] = dat;	//?????
	if(++COM1.TX_write >= COM_TX1_Lenth)	COM1.TX_write = 0;

	if(COM1.B_TX_busy == 0)		//??
	{  
		COM1.B_TX_busy = 1;		//???
		TI = 1;					//??????
	}
}

void PrintString1(u8 *puts)
{
    for (; *puts != 0;	puts++)  TX1_write2buff(*puts); 	//?????0??
}


/********************* UART1????************************/
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

/*************  ????????? *****************/
void Timer3Init(void)		//1??@24.000MHz
{
	T4T3M |= 0x02;		
	T3L = 0x40;	
	T3H = 0xA2;	
	T4T3M |= 0x08;	
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

		if(BUTTON1_TRIG==1){MENU_1=1;MENU_2=0;MENU_3=0;OLED_Clear();goto br;}
		if(BUTTON2_TRIG==1){MENU_1=0;MENU_2=1;MENU_3=0;OLED_Clear();reat_count=0;goto br;}
		if(BUTTON3_TRIG==1){MENU_1=0;MENU_2=0;MENU_3=1;OLED_Clear();goto br;}
		
		//if(rem2_DELAY==B11110000){cou2++;}
		if(MENU_1==1){
			if(rem1_DELAY==B11110000){//??1?0??
				cou1++;//???1
				speed=0.6/(cspd*0.01);//????
				cspd=0;//?????????
			}
			else{
				cspd++;//???????10ms??
			}
			//??
			OLED_ShowString(1,1,"Count");
			OLED_ShowNum(40,1,cou1,4,15);
			OLED_ShowString(1,3,"Speed");
OLED_ShowNum(40,3,speed,4,15);
			OLED_ShowString(1,5,"Mails");
			OLED_ShowNum(40,5,cou1*0.6,4,15);
			rem1_DELAY=(rem1_DELAY<<1)|E1;//????????
		}
		

		if(MENU_2==1){
			beat++;
			OLED_ShowNum(40,5,read_adc(0),4,20);
			OLED_ShowString(1,1,"reat");
			OLED_ShowNum(40,1,reat,4,20);
			OLED_ShowString(1,3,"beat");
			OLED_ShowNum(40,3,reat_count,4,20);
			if(count==1){
				reat=60*count*100/beat;//????
				count=0;beat=0;
			}
			if(read_adc(0)>level){fl=1;}//?????????1
			else{fl=0;}					//?????????0
			fl_TRIG=(fl_TRIG<<1)|fl;	//????????
			if(fl_TRIG==B01111111){		//??0?1??
				reat_count++;
				count++;	
//?????1
			}
		}
		if(MENU_3==1){
			OLED_ShowString(1,1,"omige");
			//OLED_ShowNum(40,3,HAL,4,20);
			while(HAL==0){//????????
				HAL_count++;//????10ms?????
			}
			OLED_ShowNum(40,1,2*3.14/(HAL_count*0.01),5,20);//??,??????
			HAL_count=0;//????10ms?????	
		}
	}
	br:		
	EA = 1; 
}
/**********************************************/
void main(void)
{
	u16	i;
	u8 k = 0;
	u8 j = 0;

	EA = 1;
	P1M1 = B00000000;
	P1M0 = B00100000;
	P2M1 = B10001010;
	P2M0 = B11111111;
	P3M1 = B00001000;
	P3M0 = B11111100;
	P5M1 = B00010000;
P5M0 = B00110000;
	
	OLED_Init();			//???OLED  
	OLED_Clear(); 
	OLED_Clear();
		
	OLED_DrawBMP(0,0,128,8,BIT_LOGO);
	delay_ms(1000);
	OLED_Clear();

	UART_config();
	Timer3Init();		
	while (1)
	{
		if(COM1.RX_TimeOut > 0)		//????
		{
			if(--COM1.RX_TimeOut == 0)
			{
				if(COM1.RX_Cnt > 0)
				{	
					 for(i=0;i<COM1.RX_Cnt;i++)
					  {
					  		//TX1_write2buff(RX1_Buffer[i]);
					  }					  
				}
				COM1.RX_Cnt = 0;
			}
		}
	}
}
