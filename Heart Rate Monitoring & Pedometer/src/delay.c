
#include <stdio.h>
#include	"config.h"
#include	"delay.h"


void Delay1us()		//@24.000MHz
{
	unsigned char i;

	_nop_();
	_nop_();
	i = 3;
	while (--i);
}
void delay_us(u16 us)		//@24.000MHz
{
	 do{
	     Delay1us();
     }while(--us);	
}
void Delay1ms()		//@24.000MHz
{
	unsigned char i, j;

	i = 24;
	j = 85;
	do
	{
		while (--j);
	} while (--i);
}
void delay_ms(u16 ms)		//@24.000MHz
{
	 do{
	     Delay1ms();
     }while(--ms);	
}