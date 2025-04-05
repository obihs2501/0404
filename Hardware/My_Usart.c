#include "stm32f10x.h"                  // Device header
#include "Serial.h"

s8 Send_Data[50];//�������λ�����͵�����
u8 Receive_Data[50];//��Ŵ���3���յ�������
s16 Get_Data[20];//���t265�ģ�x,y,z,yaw��


//����3��������
void Serial_Analysis(u8 *DataToAnalysis)
{
	//У��λ
	u8 sumcheck = 0;
	u8 addcheck = 0;
	for(u8 i = 0;i < *(DataToAnalysis + 1) + 2;i++)//*(DataToAnalysis + 1)=>���ݳ��ȣ�+2��AA�����ݳ���
	{
		sumcheck += *(DataToAnalysis + i);
		addcheck += sumcheck;
	}
	//У��
	if(sumcheck == *(DataToAnalysis + *(DataToAnalysis + 1) + 2) && addcheck == *(DataToAnalysis + *(DataToAnalysis + 1) + 3))
	{
		for(u8 i = 0,j = 0;j < *(DataToAnalysis + 1);)//һ������ռ16λ���ָߣ���λ����
		{
			Get_Data[i] = (*(DataToAnalysis + 2 + j)) | (*(DataToAnalysis + 3 + j)<<8);//(*(DataToAnalysis + 3 + j)<<8):�����ƶ�8λ��Ϊ�˱�ɸ�λ
            i++;
            j += 2;//�����ֽ���Ϊһλ����
		}
	}
}
//����3���պ���
void my_Receive_Data(u8 bytedata)
{
	static u8 len = 0;static u8 rec_sta = 0;
	Receive_Data[rec_sta] = bytedata;
	if(rec_sta == 0)
	{
		if(bytedata == 0xAA)//֡ͷ
		{
			rec_sta++;
		}
		else
		{
			rec_sta = 0;//У�鲻ͨ�������½�������
		}
	}
	else if(rec_sta == 1)
	{
		if(bytedata == 0x08)//���ݳ���
		{
			rec_sta ++;
			len = 8;
		}
		else
		{
			rec_sta = 0;
		}
	}
	else if(rec_sta == (len + 4 -1))//len:����λ���ȣ�4:AA�����ݳ��ȡ���У�顢����У��
	{
		Serial_Analysis(Receive_Data);//��������
		rec_sta = 0;//���½�������
	}
	else
	{
		rec_sta ++;//������������λ
	}
}


