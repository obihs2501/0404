#include "pstwo.h"
/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File��PS2��������
Author��pinggai    Version:1.0     Data:2015/05/16
Description: PS2��������
**********************************************************/	 
u16 Handkey;
u8 Comd[2]={0x01,0x42};	//��ʼ�����������
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����

u16 MASK[]={
	PSB_SELECT,
	PSB_L3,
	PSB_R3 ,
	PSB_START,
	PSB_PAD_UP,
	PSB_PAD_RIGHT,
	PSB_PAD_DOWN,
	PSB_PAD_LEFT,
	PSB_L2,
	PSB_R2,
	PSB_L1,
	PSB_R1 ,
	PSB_GREEN,
	PSB_RED,
	PSB_BLUE,
	PSB_PINK
};	//����ֵ�밴����

//�ֱ��ӿڳ�ʼ��    ����  DI->PB12 
//                  ���  DO->PB13    CS->PB14  CLK->PB15
////����ֵ�밴����
//��ఴ�����1��11   ��ఴ�����2��9      �Ҳఴ�����1��12  �Ҳఴ�����2��10
//���ϣ�5 ����8 ���ң�6 ���ϣ�5  ���£�7
//���ϣ�13 ����16 ���ң�14 ���ϣ�13  ���£�15


//������95�������ѽ��ֱ�ģʽ��Ϊ�̵�ģʽ����ģʽ�£�
//��ఴ��  �ϣ�KEY=5   �£�KEY=7 �� KEY=8  �ң�KEY=6
//�Ҳఴ��  ���ǣ�KEY=13 �棺KEY=15 ���飺KEY=16  Բ��KEY=14

//���ҡ�� ����8 ���ң�6 ���ϣ�5 ���£�7
//�Ҳ�ҡ�� ����16 ���ң�14 ���ϣ�13 ���£�15

//L1; 11  L2; 9   R1;12 R2;10

void PS2_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	//����  DI->PB12
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);		//ʹ��PORTBʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 				 	//���ó���������������������Կ�
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//���  DO->PB13    CS->PB14  CLK->PB15
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 				//���ó�������� 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//���ֱ���������
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //�����Ϊ����λ
		}
		else DO_L;

		CLK_H;                        //ʱ������
		Delay_us(50);
		CLK_L;
		Delay_us(50);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
}

//�ж��Ƿ�Ϊ���ģʽ
//����ֵ��0�����ģʽ
//		  ����������ģʽ
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	CS_H;
	if( Data[1] == 0X41)   return 0 ;
	else return 1;

}
//��ȡ�ֱ�����
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;

	CS_L;

	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������

	for(byte=2;byte<9;byte++)          //��ʼ��������
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			CLK_L;
			Delay_us(50);
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
       Delay_us(50);
	}
	CS_H;	
}

//�Զ�������PS2�����ݽ��д���      ֻ�����˰�������         Ĭ�������Ǻ��ģʽ  ֻ��һ����������ʱ
//����Ϊ0�� δ����Ϊ1
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //û���κΰ�������
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}




