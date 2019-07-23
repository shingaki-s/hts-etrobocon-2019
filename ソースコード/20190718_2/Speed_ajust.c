#ifndef _FUNC_H_
#include "func.h"
#include <math.h>
#endif
//80以上の数値を受け取ったとき、少し減らした値を返す


//スピード調整を(4 * SPEED_AJUST_CALL_FREQUENCY)msごとに行う
//#define SPEED_AJUST_CALL_FREQUENCY 25

//*****************************************************************************
// 関数名 : Speed_adjust
// 引数 : now_speed(現在の速度)，target_speed(目標の速度)
// 返り値 : 調整した速度
// 概要 : 速度を目標値に近づける
//*****************************************************************************
int Speed_adjust(int turn){
	//speed_down_counter++;
	turn = abs(turn);
	if (turn >= 50){
		return 0;
	}else if (turn > 40 && turn < 50){
		return 50 - sqrt(50*50 - (turn-50)*(turn-50));
	}else if (turn > 30 && turn <= 40){
		return 60 - sqrt(50*50 - (turn-50)*(turn-50));
	}else if (turn > 20 && turn <= 30) {
		return 70 - sqrt(50*50 - (turn-50)*(turn-50));
	}else if (turn > 10 && turn <= 20) {
		return 85 - sqrt(50*50 - (turn-50)*(turn-50));
	}else {
		return 100 - 1.5 * turn;
	}
	//if(turn < 5 && turn > -5){
	//	return 100;
	//}else if(turn <= -10){
	//	return 100 - sqrt(100*100 - (turn+100)*(turn+100));
	//}else {
	//	return 100 - sqrt(100*100 - (turn-100)*(turn-100));
	//}
		//目標値と現在値の差
		//int diff = now_speed - target_speed;
		//
		//if(diff > 0){
		//	now_speed = now_speed - 1;
		//}
		//else if(diff < 0){
		//	now_speed = now_speed + 1;
		//}
		//speed_down_counter = 0;
	//}
	//return now_speed;
}

