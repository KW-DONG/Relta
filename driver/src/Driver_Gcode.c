#include "Driver_Gcode.h"
#include "Driver_Motion.h"
#include <math.h>

void Decoder()
{
    u8 len;
    int t;
		
	if(USART_RX_STA&0x8000)
	{
		len=USART_RX_STA&0x3fff;
		for(t=0;t<len;t++)
		{
			if (USART_RX_BUF[t] == 0x47)//ASCii code of "G"
			{
				printf("\r\nThis is Gcode\r\n");
				if (USART_RX_BUF[t+1]==0x30)//ASCii code of "0"
				{
					Gcode_G00(len);
				}else if(USART_RX_BUF[t+1]==0x31){//ASCii code of "1"
					Gcode_G01(len);
				}else if(USART_RX_BUF[t+1]==0x32){//ASCii code of "2"
					Gcode_G02(len);
				}else if(USART_RX_BUF[t+1]==0x33){//Ascii code of "3"
					Gcode_G03(len);
				}			
			}else 
			{
				printf("\r\nThis is not Gcode\r\n");
			}
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		}			
		printf("\r\n\r\n");
		USART_RX_STA=0;	
	}
}

void Gcode_G00(uint8_t len)
{
	int t;
	Motion_End Linear;
	for (t=0;t<len;t++)
	{
		if (USART_RX_BUF[t] == 0x58)//X
		Linear.x = Read_Data(t);

		if (USART_RX_BUF[t] == 0x59)//Y
		Linear.y = Read_Data(t);

		if (USART_RX_BUF[t] == 0x5A)//Z
		Linear.z = Read_Data(t);
	}
	Linear.f = 1;
	Motion_Linear(&END_EFFECTOR, &SLIDER, &Linear);
}

void Gcode_G01(uint8_t len)
{
	uint32_t t;
	Motion_End Linear;
	for (t=0;t<len;t++)
	{
		if (USART_RX_BUF[t] == 0x58)//X
		Linear.x = Read_Data(t);

		if (USART_RX_BUF[t] == 0x59)//Y
		Linear.y = Read_Data(t);

		if (USART_RX_BUF[t] == 0x5A)//Z
		Linear.z = Read_Data(t);

		if (USART_RX_BUF[t] == 0x46)//F
		Linear.f = Read_Data(t);
	}
	Motion_Linear(&END_EFFECTOR, &SLIDER, &Linear);
}

void Gcode_G02(uint8_t len)
{
	uint32_t t;
	Motion_End Circular;
	for (t=0;t<len;t++)
	{
		if (USART_RX_BUF[t] == 0x58)//X
		Circular.x = Read_Data(t);

		if (USART_RX_BUF[t] == 0x59)//Y
		Circular.y = Read_Data(t);

		if (USART_RX_BUF[t] == 0x5A)//Z
		Circular.z = Read_Data(t);

		if (USART_RX_BUF[t] == 0x46)//F
		Circular.f = Read_Data(t);

		if (USART_RX_BUF[t] == 0x49)//I
		Circular.i = Read_Data(t);

		if (USART_RX_BUF[t] == 0x4A)//J
		Circular.j = Read_Data(t);

	}
	Motion_Circular(&END_EFFECTOR, &SLIDER, &Circular, clockwise);
}

void Gcode_G03(uint8_t len)
{
	uint32_t t;
	Motion_End Circular;
	for (t=0;t<len;t++)
	{
		if (USART_RX_BUF[t] == 0x58)//X
		Circular.x = Read_Data(t);

		if (USART_RX_BUF[t] == 0x59)//Y
		Circular.y = Read_Data(t);

		if (USART_RX_BUF[t] == 0x5A)//Z
		Circular.z = Read_Data(t);

		if (USART_RX_BUF[t] == 0x46)//F
		Circular.f = Read_Data(t);

		if (USART_RX_BUF[t] == 0x49)//I
		Circular.i = Read_Data(t);

		if (USART_RX_BUF[t] == 0x4A)//J
		Circular.j = Read_Data(t);

	}
	Motion_Circular(&END_EFFECTOR, &SLIDER, &Circular, anticlockwise);
}

uint32_t Read_Data(uint32_t t)
{
	uint32_t dataLength;
	uint32_t result = 0;
	dataLength = t+1;
	while(USART_RX_BUF[dataLength] != 0x20)//space
	{
		dataLength ++;
	}
	dataLength = dataLength - t;

	if (USART_RX_BUF[dataLength-1] != 0x2E)//period
	{
		for (uint32_t n=t+1;n<dataLength+t;n++)
		{
			result = result + (uint32_t)USART_RX_BUF[n] * (uint32_t)pow(10.0,(double)(dataLength-n-t));
		}
	}else if (USART_RX_BUF[dataLength-1] == 0x2E)
	{
		for (uint32_t n=t+1;n<dataLength+t-2;n++)
		{
			result = result + (uint32_t)USART_RX_BUF[n] * (uint32_t)pow(10.0,(double)(dataLength-n-t));
		}
		result = result + (uint32_t)USART_RX_BUF[dataLength+t];
	}
	return result;
	
}

void Send_Feedback()
{
	USART_SendData(USART1,"Finished");
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
}