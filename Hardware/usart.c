#include "sys.h"
#include "usart.h"	
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

//����3����ң��
#define START_BYTE       0xA2    // ���ݰ���ʼ��־
#define END_BYTE         0xC2    // ���ݰ�������־
#define DATA_LENGTH      0x04    // ���ݳ��ȱ�ʶ
#define CMD_INDEX        4       // ָ���ֽ�λ��
//A2 04 00 00 00 0* C2 
uint8_t USART4_RX_BUF[5] = {0}; // �洢��ͷ���6�ֽ�
uint8_t command_flag = 0;       // �����־λ
uint8_t command = 0;  



/*��ֲ����1*/



int Serial_RxPacket[6];				//����������ݰ����飬���ݰ���ʽ"@MSG\r\n"
uint8_t Serial_RxFlag;					//����������ݰ���־λ


/**
  * ��    ��������1��ʼ��
  * ��    ������
  * �� �� ֵ����
  */
	

// ���� FILE �ṹ�壬��֧�ֱ�׼�����������
#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif	
	
void USART1_Init(void)
{
	/*����ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//����USART1��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//����GPIOA��ʱ��
	
	/*GPIO��ʼ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//��PA9���ų�ʼ��Ϊ�����������
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//��PA10���ų�ʼ��Ϊ��������
	
	/*USART��ʼ��*/
	USART_InitTypeDef USART_InitStructure;					//����ṹ�����
	USART_InitStructure.USART_BaudRate = 115200;				//������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//Ӳ�������ƣ�����Ҫ
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//ģʽ������ģʽ�ͽ���ģʽ��ѡ��
	USART_InitStructure.USART_Parity = USART_Parity_No;		//��żУ�飬����Ҫ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//ֹͣλ��ѡ��1λ
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//�ֳ���ѡ��8λ
	USART_Init(USART1, &USART_InitStructure);				//���ṹ���������USART_Init������USART1
	USART_Cmd(USART1, ENABLE);								//ʹ��USART1�����ڿ�ʼ����
}
/**
  * ��    �����ض���fputc����
  * ��    ����ch Ҫ���͵��ַ�
  *           f ָ���ļ���ָ�루��ʹ�ô���ʱ�ɺ��ԣ�
  * �� �� ֵ�����ط��͵��ַ�
  */
int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  // �ȴ����ͻ�����Ϊ��
    USART_SendData(USART1, (uint8_t)ch);  // �����ַ�
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) ;  // �ȴ��������
    return ch;
}

//PUTCHAR_PROTOTYPE
//{
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  // �ȴ����ͻ�����Ϊ��
//    USART_SendData(USART1, (uint8_t)ch);  // �����ַ�
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  // �ȴ��������
//    return ch;
//}











// ����2���ջ�����
#define USART2_REC_LEN 200


// ����2��ʼ������
void USART2_Init(void)
{
    // 1. ����ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 2. GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
    // USART2_TX PA2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // �����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // USART2_RX PA3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // ��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. ��ʼ������
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;  // ������
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;  // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // �շ�ģʽ
    USART_Init(USART2, &USART_InitStructure);

    // 4. ���������ж�
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    // 5. �����ж����ȼ�
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);

    // 6. ʹ�ܴ���
    USART_Cmd(USART2, ENABLE);
}

// ����ȫ�ֱ����洢����ֵ
float Distance1 = 0.0f;
float Distance2 = 0.0f;
uint8_t USART2_RX_BUF[6];  // ��������СӦΪ6��A3���6�ֽڣ�04 00 01 47 75 C3��
volatile uint8_t USART2_RX_STA = 0;  // ����״̬

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t Res = USART_ReceiveData(USART2);

        if (USART2_RX_STA == 0 && Res == 0xA3) {
            USART2_RX_STA = 1;  // ��ʼ���պ�������
        } else if (USART2_RX_STA >= 1 && USART2_RX_STA <= 6) {
            USART2_RX_BUF[USART2_RX_STA - 1] = Res;  // �洢��������[0]~[5]
            USART2_RX_STA++;

            if (USART2_RX_STA == 7) {  // �ѽ���6�ֽڣ�A3����������ݣ�
                // ��鳤�ȺͰ�β
                if (USART2_RX_BUF[0] == 0x04 && USART2_RX_BUF[5] == 0xC3) {
                    // ��������
                    uint16_t Distance1_int = (USART2_RX_BUF[1] << 8) | USART2_RX_BUF[2];
                    uint16_t Distance2_int = (USART2_RX_BUF[3] << 8) | USART2_RX_BUF[4];
                    Distance1 = Distance1_int / 100.0f;
                    Distance2 = Distance2_int / 100.0f;
                }
                USART2_RX_STA = 0;  // ����״̬
            }
        }

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

//////////////////////*��崮��3*/////////////////////////
void Go_Forward(int8_t a, int8_t b, int8_t c, int8_t d);
void Back_Up(int8_t a, int8_t b, int8_t c, int8_t d);
void Pan_Right(int8_t a, int8_t b, int8_t c, int8_t d);
void Pan_Left(int8_t a, int8_t b, int8_t c, int8_t d);
void motor_Stop(void);
void Left_Turn_Self(int8_t a, int8_t b, int8_t c, int8_t d);
void Right_Turn_Self(int8_t a, int8_t b, int8_t c, int8_t d);

/***********************************************
* �������ƣ�USART3_Init
* ����˵����USART3��ʼ������
* ����˵������
* ����ֵ����
***********************************************/
//void USART3_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;     //GPIO��ʼ���ṹ������
//	USART_InitTypeDef USART_InitStructure;   //���ڳ�ʼ���ṹ������
//	NVIC_InitTypeDef    NVIC_InitStructure;  //NVIC��ʼ���ṹ������
//		

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);      //ʹ��PB�˿�ʱ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    //USART3ʱ��

//		
//	//USART3��Tx---GPIO----PA.10----�����������
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//		
//	//USART3��Rx---GPIO----PA.11----��������
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//		
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;             //�ж�ͨ��ΪUSART3_IRQn
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     //������ռ���ȼ�2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            //������Ӧ���ȼ�2 
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //ͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);                      //����NVIC_Init()������ɶ˿ڳ�ʼ��
//		
//	USART_InitStructure.USART_BaudRate=115200;             //���ò�����Ϊ9600
//	USART_InitStructure.USART_WordLength=USART_WordLength_8b;                      //����λ8λ
//	USART_InitStructure.USART_StopBits=USART_StopBits_1;                           //ֹͣλ1λ
//	USART_InitStructure.USART_Parity=USART_Parity_No;                              //��У��λ
//	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;  //��Ӳ������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;   //���ܺͷ���ģʽ����
//	USART_Init(USART3,&USART_InitStructure);	        //����USART_Init()������ɶ˿ڳ�ʼ��

//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);      //ʹ�ܽ����ж�

//	USART_Cmd(USART3,ENABLE);                           //ʹ�ܴ���3

//}
/***********************************************
* �������ƣ�USART3_IRQHandler
* ����˵����USART3�жϷ�����
* ����˵������
* ����ֵ����
***********************************************/


//          // �洢��ȡ������
//void USART3_IRQHandler(void)
//	{
//	
//	static uint8_t USART3_RX_STA = 0;      // ����״̬
//	static uint8_t pRxPacket = 0;
//	
//    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
//        uint8_t receivedData = USART_ReceiveData(USART3);
//        
//        // ״̬������
//        if (USART3_RX_STA == 0) { // �ȴ���ͷ
//            if (receivedData == 0xA2) {
//                USART3_RX_STA = 1;
//								pRxPacket = 0;
//            }
//        } else if(USART3_RX_STA ==1){
//						USART3_RX_BUF[pRxPacket]=receivedData;
//						pRxPacket++;
//						if(pRxPacket>=5){
//							USART3_RX_STA =2;
//							
//						}
//					}else if(USART3_RX_STA ==2){
//							if(receivedData == 0xC2){
//								USART3_RX_STA=0; 
//								Serial_RxFlag=1;
//								command=USART3_RX_BUF[4];
//							}
//					}
//        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//    }

////    // �����жϴ�����ʱ���ã�
////    if (USART_GetITStatus(USART3, USART_IT_IDLE) == SET) {
////        USART_ReceiveData(USART3); // ������б�־
////        USART3_RX_STA = 0;
////        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
////    }
//}



//uint8_t Serial_GetRxFlag(void)
//{
//	if (Serial_RxFlag == 1)			//�����־λΪ1
//	{
//		Serial_RxFlag = 0;
//		return 1;					//�򷵻�1�����Զ������־λ
//	}
//	return 0;						//�����־λΪ0���򷵻�0
//}




void USART4_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // ʹ��GPIO��USART4ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    // ����USART4 Tx (PC.10)Ϊ�����������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ����USART4 Rx (PC.11)Ϊ��������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // USART4 ����
//		USART_DeInit(UART4); //��λ����4
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);
		
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // ʹ��USART4�����ж�
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    // ʹ��USART4
    USART_Cmd(UART4, ENABLE);

    // NVIC����

}

// ����4�жϷ�����
void UART4_IRQHandler(void)
{
		static uint8_t USART4_RX_STA = 0;      // ����״̬
		static uint8_t pRxPacket = 0;
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(UART4); // ��ȡ���յ�������
        // ״̬������
        if (USART4_RX_STA == 0) { // �ȴ���ͷ
            if (data == 0xA2) {
                USART4_RX_STA = 1;
								pRxPacket = 0;
            }
        } else if(USART4_RX_STA ==1){
						USART4_RX_BUF[pRxPacket]=data;
						pRxPacket++;
						if(pRxPacket==5){
							USART4_RX_STA =2;
							
						}
					}else if(USART4_RX_STA ==2){
							if(data == 0xC2){
								USART4_RX_STA=0; 
								Serial_RxFlag=1;
								command=USART4_RX_BUF[4];
							}
					}
    // �����жϴ�����ʱ���ã�
//    if (USART_GetITStatus(UART4, USART_IT_IDLE) == SET) {
//        USART_ReceiveData(UART4); // ������б�־
//        USART4_RX_STA = 0;
//        USART_ClearITPendingBit(UART4, USART_IT_IDLE);
//    }

        // �����������Ӵ�����յ������ݵĴ���

        USART_ClearITPendingBit(UART4, USART_IT_RXNE); // ����жϱ�־
    }
}




uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//�����־λΪ1
	{
		Serial_RxFlag = 0;
		return 1;					//�򷵻�1�����Զ������־λ
	}
	return 0;						//�����־λΪ0���򷵻�0
}
