#include "sys.h"
#include "usart.h"	
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

//串口3蓝牙遥控
#define START_BYTE       0xA2    // 数据包起始标志
#define END_BYTE         0xC2    // 数据包结束标志
#define DATA_LENGTH      0x04    // 数据长度标识
#define CMD_INDEX        4       // 指令字节位置
//A2 04 00 00 00 0* C2 
uint8_t USART4_RX_BUF[5] = {0}; // 存储包头后的6字节
uint8_t command_flag = 0;       // 命令标志位
uint8_t command = 0;  



/*移植串口1*/



int Serial_RxPacket[6];				//定义接收数据包数组，数据包格式"@MSG\r\n"
uint8_t Serial_RxFlag;					//定义接收数据包标志位


/**
  * 函    数：串口1初始化
  * 参    数：无
  * 返 回 值：无
  */
	

// 定义 FILE 结构体，以支持标准输入输出函数
#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif	
	
void USART1_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 115200;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
}
/**
  * 函    数：重定义fputc函数
  * 参    数：ch 要发送的字符
  *           f 指向文件的指针（在使用串口时可忽略）
  * 返 回 值：返回发送的字符
  */
int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  // 等待发送缓冲区为空
    USART_SendData(USART1, (uint8_t)ch);  // 发送字符
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) ;  // 等待发送完成
    return ch;
}

//PUTCHAR_PROTOTYPE
//{
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  // 等待发送缓冲区为空
//    USART_SendData(USART1, (uint8_t)ch);  // 发送字符
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  // 等待发送完成
//    return ch;
//}











// 串口2接收缓冲区
#define USART2_REC_LEN 200


// 串口2初始化函数
void USART2_Init(void)
{
    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 2. GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    // USART2_TX PA2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // USART2_RX PA3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 初始化串口
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;  // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;  // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // 收发模式
    USART_Init(USART2, &USART_InitStructure);

    // 4. 开启接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    // 5. 配置中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);

    // 6. 使能串口
    USART_Cmd(USART2, ENABLE);
}

// 定义全局变量存储距离值
float Distance1 = 0.0f;
float Distance2 = 0.0f;
uint8_t USART2_RX_BUF[6];  // 缓冲区大小应为6（A3后的6字节：04 00 01 47 75 C3）
volatile uint8_t USART2_RX_STA = 0;  // 接收状态

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t Res = USART_ReceiveData(USART2);

        if (USART2_RX_STA == 0 && Res == 0xA3) {
            USART2_RX_STA = 1;  // 开始接收后续数据
        } else if (USART2_RX_STA >= 1 && USART2_RX_STA <= 6) {
            USART2_RX_BUF[USART2_RX_STA - 1] = Res;  // 存储到缓冲区[0]~[5]
            USART2_RX_STA++;

            if (USART2_RX_STA == 7) {  // 已接收6字节（A3后的所有数据）
                // 检查长度和包尾
                if (USART2_RX_BUF[0] == 0x04 && USART2_RX_BUF[5] == 0xC3) {
                    // 解析数据
                    uint16_t Distance1_int = (USART2_RX_BUF[1] << 8) | USART2_RX_BUF[2];
                    uint16_t Distance2_int = (USART2_RX_BUF[3] << 8) | USART2_RX_BUF[4];
                    Distance1 = Distance1_int / 100.0f;
                    Distance2 = Distance2_int / 100.0f;
                }
                USART2_RX_STA = 0;  // 重置状态
            }
        }

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

//////////////////////*大板串口3*/////////////////////////
void Go_Forward(int8_t a, int8_t b, int8_t c, int8_t d);
void Back_Up(int8_t a, int8_t b, int8_t c, int8_t d);
void Pan_Right(int8_t a, int8_t b, int8_t c, int8_t d);
void Pan_Left(int8_t a, int8_t b, int8_t c, int8_t d);
void motor_Stop(void);
void Left_Turn_Self(int8_t a, int8_t b, int8_t c, int8_t d);
void Right_Turn_Self(int8_t a, int8_t b, int8_t c, int8_t d);

/***********************************************
* 函数名称：USART3_Init
* 功能说明：USART3初始化配置
* 参数说明：无
* 返回值：无
***********************************************/
//void USART3_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;     //GPIO初始化结构体声明
//	USART_InitTypeDef USART_InitStructure;   //串口初始化结构体声明
//	NVIC_InitTypeDef    NVIC_InitStructure;  //NVIC初始化结构体声明
//		

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);      //使能PB端口时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);    //USART3时钟

//		
//	//USART3的Tx---GPIO----PA.10----复用推挽输出
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//		
//	//USART3的Rx---GPIO----PA.11----浮空输入
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//		
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;             //中断通道为USART3_IRQn
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     //设置抢占优先级2
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            //设置响应优先级2 
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //通道使能
//	NVIC_Init(&NVIC_InitStructure);                      //调用NVIC_Init()函数完成端口初始化
//		
//	USART_InitStructure.USART_BaudRate=115200;             //设置波特率为9600
//	USART_InitStructure.USART_WordLength=USART_WordLength_8b;                      //数据位8位
//	USART_InitStructure.USART_StopBits=USART_StopBits_1;                           //停止位1位
//	USART_InitStructure.USART_Parity=USART_Parity_No;                              //无校验位
//	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;  //无硬件流控
//	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;   //接受和发送模式都打开
//	USART_Init(USART3,&USART_InitStructure);	        //调用USART_Init()函数完成端口初始化

//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);      //使能接收中断

//	USART_Cmd(USART3,ENABLE);                           //使能串口3

//}
/***********************************************
* 函数名称：USART3_IRQHandler
* 功能说明：USART3中断服务函数
* 参数说明：无
* 返回值：无
***********************************************/


//          // 存储提取的命令
//void USART3_IRQHandler(void)
//	{
//	
//	static uint8_t USART3_RX_STA = 0;      // 接收状态
//	static uint8_t pRxPacket = 0;
//	
//    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
//        uint8_t receivedData = USART_ReceiveData(USART3);
//        
//        // 状态机处理
//        if (USART3_RX_STA == 0) { // 等待包头
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

////    // 空闲中断处理（超时重置）
////    if (USART_GetITStatus(USART3, USART_IT_IDLE) == SET) {
////        USART_ReceiveData(USART3); // 清除空闲标志
////        USART3_RX_STA = 0;
////        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
////    }
//}



//uint8_t Serial_GetRxFlag(void)
//{
//	if (Serial_RxFlag == 1)			//如果标志位为1
//	{
//		Serial_RxFlag = 0;
//		return 1;					//则返回1，并自动清零标志位
//	}
//	return 0;						//如果标志位为0，则返回0
//}




void USART4_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能GPIO和USART4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    // 配置USART4 Tx (PC.10)为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 配置USART4 Rx (PC.11)为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // USART4 配置
//		USART_DeInit(UART4); //复位串口4
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
    // 使能USART4接收中断
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    // 使能USART4
    USART_Cmd(UART4, ENABLE);

    // NVIC配置

}

// 串口4中断服务函数
void UART4_IRQHandler(void)
{
		static uint8_t USART4_RX_STA = 0;      // 接收状态
		static uint8_t pRxPacket = 0;
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(UART4); // 读取接收到的数据
        // 状态机处理
        if (USART4_RX_STA == 0) { // 等待包头
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
    // 空闲中断处理（超时重置）
//    if (USART_GetITStatus(UART4, USART_IT_IDLE) == SET) {
//        USART_ReceiveData(UART4); // 清除空闲标志
//        USART4_RX_STA = 0;
//        USART_ClearITPendingBit(UART4, USART_IT_IDLE);
//    }

        // 在这里可以添加处理接收到的数据的代码

        USART_ClearITPendingBit(UART4, USART_IT_RXNE); // 清除中断标志
    }
}




uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}
