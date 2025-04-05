#ifndef __MY_PID_H
#define __MY_PID_H

float PositionalPID_SpeedControl_A(float Actual, float Target);
float PositionalPID_SpeedControl_B(float Actual, float Target);
float PositionalPID_SpeedControl_C(float Actual, float Target);
float PositionalPID_SpeedControl_D(float Actual, float Target);


float IncrementalPID_SpeedControl_A(float Actual, float Target);
float IncrementalPID_SpeedControl_B(float Actual, float Target);
float IncrementalPID_SpeedControl_C(float Actual, float Target);
float IncrementalPID_SpeedControl_D(float Actual, float Target);
	
float PositionalPID_PositionControl_A(float Actual, float Target);
float PositionalPID_PositionControl_B(float Actual, float Target);
float PositionalPID_PositionControl_C(float Actual, float Target);
float PositionalPID_PositionControl_D(float Actual, float Target);
	

void drawcircle(float speedv,float R);
#endif

