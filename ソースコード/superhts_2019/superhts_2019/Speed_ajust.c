#ifndef _FUNC_H_
#include "func.h"
#endif
//80以上の数値を受け取ったとき、少し減らした値を返す


//スピード調整を(4 * SPEED_AJUST_CALL_FREQUENCY)msごとに行う
#define SPEED_AJUST_CALL_FREQUENCY 25

//*****************************************************************************
// 関数名 : Speed_adjust
// 引数 : now_speed(現在の速度)，target_speed(目標の速度)
// 返り値 : 調整した速度
// 概要 : 速度を目標値に近づける
//*****************************************************************************
int Speed_adjust(signed char now_speed,int target_speed){
	speed_down_counter++;
	if(speed_down_counter > SPEED_AJUST_CALL_FREQUENCY){
		//目標値と現在値の差
		int diff = now_speed - target_speed;
		//
		if(diff > 0){
			now_speed = now_speed - 1;
		}
		else if(diff < 0){
			now_speed = now_speed + 1;
		}
		speed_down_counter = 0;
	}
	return now_speed;
}

