#include "stm32f10x.h"                  // Device header
#include "Serial.h"

s8 Send_Data[50];//存放向上位机发送的数据
u8 Receive_Data[50];//存放串口3接收到的数据
s16 Get_Data[20];//存放t265的（x,y,z,yaw）


//串口3解析函数
void Serial_Analysis(u8 *DataToAnalysis)
{
	//校验位
	u8 sumcheck = 0;
	u8 addcheck = 0;
	for(u8 i = 0;i < *(DataToAnalysis + 1) + 2;i++)//*(DataToAnalysis + 1)=>数据长度，+2：AA、数据长度
	{
		sumcheck += *(DataToAnalysis + i);
		addcheck += sumcheck;
	}
	//校验
	if(sumcheck == *(DataToAnalysis + *(DataToAnalysis + 1) + 2) && addcheck == *(DataToAnalysis + *(DataToAnalysis + 1) + 3))
	{
		for(u8 i = 0,j = 0;j < *(DataToAnalysis + 1);)//一个数据占16位，分高，低位解析
		{
			Get_Data[i] = (*(DataToAnalysis + 2 + j)) | (*(DataToAnalysis + 3 + j)<<8);//(*(DataToAnalysis + 3 + j)<<8):向右移动8位是为了变成高位
            i++;
            j += 2;//两个字节作为一位数据
		}
	}
}
//串口3接收函数
void my_Receive_Data(u8 bytedata)
{
	static u8 len = 0;static u8 rec_sta = 0;
	Receive_Data[rec_sta] = bytedata;
	if(rec_sta == 0)
	{
		if(bytedata == 0xAA)//帧头
		{
			rec_sta++;
		}
		else
		{
			rec_sta = 0;//校验不通过，重新接收数据
		}
	}
	else if(rec_sta == 1)
	{
		if(bytedata == 0x08)//数据长度
		{
			rec_sta ++;
			len = 8;
		}
		else
		{
			rec_sta = 0;
		}
	}
	else if(rec_sta == (len + 4 -1))//len:数据位长度，4:AA、数据长度、和校验、附加校验
	{
		Serial_Analysis(Receive_Data);//解析数据
		rec_sta = 0;//重新接收数据
	}
	else
	{
		rec_sta ++;//持续接收数据位
	}
}


