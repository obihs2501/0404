#include "stm32f10x.h"
#include "My_Math.h"
#include "math.h"
#include "motor.h"

#define a_PARAMETER          (0.0825f)               
#define b_PARAMETER          (0.1f) 
#define EPSILON          0.0001 

float Velocity_KP=1,Velocity_KI=3.4;	          //速度（内环）控制PID参数
float Dis_KP = 0.22;//位置（外环）PID参数
extern int Encoder_A;
extern int Encoder_B;
extern int Encoder_C;
extern int Encoder_D;//电机的转速

//定速控制
float Kp_VA=0.32,Ki_VA=0.065,Kd_VA=0;//位置式PID参数
float Kp_VB=0.30,Ki_VB=0.005,Kd_VB=0;//位置式PID参数
float Kp_VC=0.28,Ki_VC=0.005,Kd_VC=0;//位置式PID参数
float Kp_VD=0.32,Ki_VD=0.065,Kd_VD=0;//位置式PID参数


float KP_VA=0.32,KI_VA=0.065,KD_VA=0;//增量式PID参数
float KP_VB=0.32,KI_VB=0.065,KD_VB=0.05;//增量式PID参数
float KP_VC=0.32,KI_VC=0.065,KD_VC=0.05;//增量式PID参数
float KP_VD=0.32,KI_VD=0.065,KD_VD=0.05;//增量式PID参数

float Kp_PA=0.5,Ki_PA=0.06,Kd_PA=0.05;//定位置控制位置式PID参数
float Kp_PB=0.3,Ki_PB=0.1,Kd_PB=0.1;//定位置控制位置式PID参数
float Kp_PC=0.5,Ki_PC=0.1,Kd_PC=0.1;
float Kp_PD=0.5,Ki_PD=0.1,Kd_PD=0.1;

float Pwm_Lowerlimit=-80, Pwm_Upperlimit=80;

/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z（角度） 三轴速度或者位置
返回  值：无
**************************************************************************/
//void Kinematic_Analysis(float Vx,float Vy,float Vz)//逆运动学解算
//{
//	
//	Target_A   = Vx-Vy-Vz*(a_PARAMETER+b_PARAMETER);
//	Target_B   = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
//	Target_C   = Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
//	Target_D   = +Vx-Vy+Vz*(a_PARAMETER+b_PARAMETER);

//}


/*************************对编码电机进行PI控制（增量式PID）***************************************/

//int16_t PID_VelocityPidA(float Spd_Target, int16_t Spd_Now)
//{
//	static float Motor_Pwm_Out;
//	static float bias,bias_last,bias_integral = 0;
//	float para;
//	bias = Spd_Target - Spd_Now;
//	
//	bias_integral += bias;
//	
//	para = MotorA_kp*bias*PID_SCALE + MotorA_kd*(bias-bias_last)*PID_SCALE + MotorA_ki*bias_integral*PID_SCALE;
//	if(para<-1 || para>1)
//	{
//		Motor_Pwm_Out +=para; 
//	}
//	if(Motor_Pwm_Out > 3500) Motor_Pwm_Out = 3500;
//	if(Motor_Pwm_Out < 0)Motor_Pwm_Out = 0;
//	bias_last = bias;	
//	
//	return Motor_Pwm_Out;
//}	









/**********************************************************************************************************
*	函 数 名：PID_Cal
*	功能说明：位置式PID控制
*   输    入：
    Actual:当前值
    Target:目标值
*   输    出：PID控制值，直接赋值给执行函数
**********************************************************************************************************/ 
float PositionalPID_SpeedControl_A(float Actual, float Target)
{
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if(fabs(Ki_VA)>EPSILON){
		bias_integral+=bias;
	}else{
		bias_integral=0;
	}

	Pwm_Out= Kp_VA*bias+Ki_VA*bias_integral+Kd_VA*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	
    
			  	                     //更新上次误差，用于下次计算 
	return Pwm_Out;	//返回控制输出值
}



float PositionalPID_SpeedControl_B(float Actual, float Target)
{
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if(fabs(Ki_VB)>EPSILON){
		bias_integral+=bias;
	}else{
		bias_integral=0;
	}
	Pwm_Out= Kp_VB*bias+Ki_VB*bias_integral+Kd_VB*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	
    
			  	                     //更新上次误差，用于下次计算 
	return Pwm_Out;	//返回控制输出值
}


float PositionalPID_SpeedControl_C(float Actual, float Target)
{
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if(fabs(Ki_VC)>EPSILON){
		bias_integral+=bias;
	}else{
		bias_integral=0;
	}
	Pwm_Out= Kp_VC*bias+Ki_VC*bias_integral+Kd_VC*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	
    
			  	                     //更新上次误差，用于下次计算 
	return Pwm_Out;	//返回控制输出值
}



float PositionalPID_SpeedControl_D(float Actual, float Target)
{
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if(fabs(Ki_VD)>EPSILON){
		bias_integral+=bias;
	}else{
		bias_integral=0;
	}
	Pwm_Out= Kp_VD*bias+Ki_VD*bias_integral+Kd_VD*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	  
			  	                     //更新上次误差，用于下次计算 
	return Pwm_Out;	//返回控制输出值
}



/*************************对编码电机进行PI控制（增量式PID）***************************************/


float IncrementalPID_SpeedControl_A(float Actual, float Target){
	static float Pwm_Out;
	static float Error0,Error1,Error2;//本次误差 上次误差 上上次误差	
	Error1=Error0;
	Error2=Error1;
	Error0=Target-Actual;
	Pwm_Out+=KP_VA*(Error0-Error1)+KI_VA*Error0+KD_VA*(Error0-2*Error1+Error2);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	
	return Pwm_Out;	//返回控制输出值	
}

float IncrementalPID_SpeedControl_B(float Actual, float Target){
	static float Pwm_Out;
	static float Error0,Error1,Error2;//本次误差 上次误差 上上次误差	
	Error1=Error0;
	Error2=Error1;
	Error0=Target-Actual;
	Pwm_Out+=KP_VB*(Error0-Error1)+KI_VB*Error0+KD_VB*(Error0-2*Error1+Error2);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	
	return Pwm_Out;	//返回控制输出值	
}




float IncrementalPID_SpeedControl_C(float Actual, float Target){
	static float Pwm_Out;
	static float Error0,Error1,Error2;//本次误差 上次误差 上上次误差	
	Error1=Error0;
	Error2=Error1;
	Error0=Target-Actual;
	Pwm_Out+=KP_VC*(Error0-Error1)+KI_VC*Error0+KD_VC*(Error0-2*Error1+Error2);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	
	return Pwm_Out;	//返回控制输出值	
}


float IncrementalPID_SpeedControl_D(float Actual, float Target){
	static float Pwm_Out;
	static float Error0,Error1,Error2;//本次误差 上次误差 上上次误差	
	Error1=Error0;
	Error2=Error1;
	Error0=Target-Actual;
	Pwm_Out+=KP_VD*(Error0-Error1)+KI_VD*Error0+KD_VD*(Error0-2*Error1+Error2);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;
	}	
	return Pwm_Out;	//返回控制输出值	
}



//定位置控制，积分分离，若误差绝对值小于指定阈值，则加入积分项的作用，反之，则将误差清零或不加入积分项的作用，防止超调；（误差大：PID，误差小：PD）
//PositionalPID_PositionControl
//

float PositionalPID_PositionControl_A(float Actual, float Target){
	
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if (fabs(Ki_PA)>EPSILON&&fabs(bias) < 15)//积分分离
	{  
		bias_integral += bias;
	}
	else
	{
		bias_integral = 0;
	}
	Pwm_Out= Kp_PA*bias+Ki_PA*bias_integral+Kd_PA*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;}

  return Pwm_Out;	//返回控制输出值	
}



float PositionalPID_PositionControl_B(float Actual, float Target){
	
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if (fabs(Ki_PB)>EPSILON&&fabs(bias) < 10)//积分分离
	{  
		bias_integral += bias;
	}
	else
	{
		bias_integral = 0;
	}
	Pwm_Out= Kp_PB*bias+Ki_PB*bias_integral+Kd_PB*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;}

  return Pwm_Out;	//返回控制输出值	
}


float PositionalPID_PositionControl_C(float Actual, float Target){
	
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if (fabs(Ki_PB)>EPSILON&&fabs(bias) < 15)//积分分离
	{  
		bias_integral += bias;
	}
	else
	{
		bias_integral = 0;
	}
	Pwm_Out= Kp_PB*bias+Ki_PB*bias_integral+Kd_PB*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;}

  return Pwm_Out;	//返回控制输出值	
}



float PositionalPID_PositionControl_D(float Actual, float Target){
	
	static float Pwm_Out;
	static float bias,bias_last,bias_integral = 0;//本次误差 上次误差  误差积累
	bias_last=bias;
	bias=Target-Actual;
	if (fabs(Ki_PB)>EPSILON&&fabs(bias) < 15)//积分分离
	{  
		bias_integral += bias;
	}
	else
	{
		bias_integral = 0;
	}
	Pwm_Out= Kp_PB*bias+Ki_PB*bias_integral+Kd_PB*(bias-bias_last);
	if(Pwm_Out>Pwm_Upperlimit){
	Pwm_Out=Pwm_Upperlimit;
	}if(Pwm_Out<Pwm_Lowerlimit){
	Pwm_Out=Pwm_Lowerlimit;}

  return Pwm_Out;	//返回控制输出值	
}



//int	Incremental_PI_A(int Encoder,int Target)
//{
//	static int Bias,Pwm,Last_bias;
//	Bias=Encoder-Target;                //计算偏差
//	Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
//	if(Pwm>7200)Pwm=7200;
//	if(Pwm<-7200)Pwm=-7200;
//	Last_bias=Bias;	                   //保存上一次偏差 
//	return fabs(Pwm);                         //增量输出
//}

//int	Incremental_PI_B(int Encoder,int Target)
//{
//	static int Bias,Pwm,Last_bias;
//	Bias=Encoder-Target;                //计算偏差
//	Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
//	if(Pwm>7200)Pwm=7200;
//	if(Pwm<-7200)Pwm=-7200;
//	Last_bias=Bias;	                   //保存上一次偏差 
//	return fabs(Pwm);                         //增量输出
//}

//int	Incremental_PI_C(int Encoder,int Target)
//{
//	static int Bias,Pwm,Last_bias;
//	Bias=Encoder-Target;                //计算偏差
//	Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
//	if(Pwm>7200)Pwm=7200;
//	if(Pwm<-7200)Pwm=-7200;
//	Last_bias=Bias;	                   //保存上一次偏差 
//	return fabs(Pwm);                         //增量输出
//}

//int	Incremental_PI_D(int Encoder,int Target)
//{
//	static int Bias,Pwm,Last_bias;
//	Bias=Encoder-Target;                //计算偏差
//	Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
//	if(Pwm>7200)Pwm=7200;//对PWM进行限幅
//	if(Pwm<-7200)Pwm=-7200;
//	Last_bias=Bias;	                    //保存上一次偏差 
//	return fabs(Pwm);                         //增量输出
//}

////串级PID的实现（外环位置环，内环速度环）
//void My_Cascade_PID(float now_x,float now_y,float target_x, float target_y,float angle_z)
//{
//	//外环
//	static float Bias,target_V;
//	float DIF_x = target_x - now_x;
//	float DIF_y = target_y - now_y;
//	Bias = My_Sqrt(DIF_x,DIF_y);//两个点之间的直线距离（误差）
//	target_V = Dis_KP * Bias;
//	float temp = fabs(DIF_y / DIF_x);
//	float angle = atan(temp) * 57.3; //将弧度转成角度
//	float V_x = target_V * cos(angle / 57.3);
//	float V_y = target_V * sin(angle / 57.3);
//	//确定小车沿x轴和y轴的速度方向
//	if(DIF_x >=  0 && DIF_y >= 0)
//	{
//		V_x = V_x;
//		V_y = V_y;
//	}
//	else if(DIF_x >=  0 && DIF_y <= 0)
//	{
//		V_x = V_x;
//		V_y = -V_y;
//	}
//	else if(DIF_x <=  0 && DIF_y <= 0)
//	{
//		V_x = -V_x;
//		V_y = -V_y;
//	}
//	else if(DIF_x <=  0 && DIF_y >= 0)
//	{
//		V_x = -V_x;
//		V_y = V_y;
//	}
//	//内环
//	Kinematic_Analysis(V_x,V_y,angle_z);
//	Motor_A = Incremental_PI_A(Encoder_A,Target_A);
//	Motor_B = Incremental_PI_B(Encoder_B,Target_B);
//	Motor_C = Incremental_PI_C(Encoder_C,Target_C);
//	Motor_D = Incremental_PI_D(Encoder_D,Target_D);
//}

////执行器（编码电机）
//void  Actuator(long int pwm_A,long int pwm_B,long int pwm_C,long int pwm_D,int8_t flag_a,int8_t flag_b,int8_t flag_c,int8_t flag_d)
//{
//	Move(pwm_A,pwm_B,pwm_C,pwm_D,flag_a,flag_b,flag_c,flag_d);
//}

//int temp1 = 0;
//int temp2 = 0;
////小车画圆
//void drawcircle(float speedv,float R)
//{
//	float k = a_PARAMETER * 100;
//	int v1 = speedv + k*(speedv / R);//外环的速度
//	int v2 = speedv - k*(speedv / R);//内环的速度
//	temp1 = v1;
//	temp2 = v2;
//	//速度环控制
//	Motor_A = Incremental_PI_A(Encoder_A,v1);
//	Motor_B = Incremental_PI_B(Encoder_B,v1);
//	Motor_C = Incremental_PI_C(Encoder_C,v2);
//	Motor_D = Incremental_PI_D(Encoder_D,v2);
//	Actuator(Motor_A,Motor_B,Motor_C,Motor_D,1,1,1,1);
//}
