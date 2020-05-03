#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 
 
#define KEY_X 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0) 
#define KEY_Y 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2) 
#define KEY_Z 		GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)	

#define KEYX_PRES 	1	
#define KEYY_PRES	2	
#define KEYZ_PRES	3	
	

void KEY_Init(void);	
u8 KEY_Scan(u8);  		

#endif