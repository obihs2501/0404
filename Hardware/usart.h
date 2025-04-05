#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
extern float Distance1;
extern float Distance2;
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void USART1_Init(void);
void USART1_IRQHandler(void);
void USART2_Init(void);
void USART2_IRQHandler(void);

//void USART3_SendString();
//void USART3_Init(void);
//void USART3_IRQHandler(void);
void USART4_Init(void);
void USART4_IRQHandler(void);
uint8_t Serial_GetRxFlag(void);


void USART_SendChar(uint8_t ch);
//void Serial_SendByte(uint8_t Byte);
//void Serial_SendArray(uint8_t *Array, uint16_t Length);
//void Serial_SendString(char *String);
//void Serial_SendNumber(uint32_t Number, uint8_t Length);
//void Serial_Printf(char *format, ...);

#endif


