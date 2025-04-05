#include<My_Math.h>
#include "math.h"
#include "stm32f10x.h"

//求两个数的平方根
int My_Sqrt(int num1,int num2)
{
	int num = num1 * num1 + num2 * num2;
	return sqrt(num);
}


int abs_int(int num) {
    return (num < 0) ? -num : num;
}

int16_t abs_int16(int16_t num) {
    return (num < 0) ? -num : num;
}

float abs_float(float num) {
    return (num < 0) ? -num : num;
}

double abs_double(double num) {
    return (num < 0) ? -num : num;
}
