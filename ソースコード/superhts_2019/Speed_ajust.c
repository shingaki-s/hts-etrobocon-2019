#ifndef _FUNC_H_
#include "func.h"
#include <math.h>
#endif

int Speed_adjust(int turn){
	int turn1 = abs(turn);
	//対策２
	float a = (0-50)/225;
	if (turn1 >= 40){
		return 0;
	}else if (turn1 > 15 && turn1 < 40 ){
		return (int)(0.08*turn1*turn1-6.4*turn1)+128;
	}else if (turn1 >= 0 && turn1 <= 15){
		return (int)(a*turn1*turn1)+100;
	}
	
	//対策１
	//float a = 95/2116;
	//float b = -100*a;
	//float c = 2500*a;

	//if (turn1 >= 50){
	//	return 0;
	//}else if (turn1 > 4 && turn1 < 50 ){
	//	return a*turn1*turn1 + b*turn1 + c;
	//}else if (turn1 >= 0 && turn1 <= 4){
	//	return -0.3125*turn1*turn1+100;
	//}
}