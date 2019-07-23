#ifndef _FUNC_H_
#include "func.h"
#endif

#ifndef _EV3API_H_
#include "ev3api.h"
#endif  //_EV3API_h_

#ifndef _MATH_H_
#include <math.h>
#endif

#ifndef _BALANCER_H_
#include "balancer.h"
#endif

#ifndef _STDBOOL_H_
#include <stdbool.h>
#endif
#include <stdio.h>

//90度曲がる際のturnの値
#define TURN_PHASE5 -30


//曲がるまでの待機時間cnt
#define WAIT_PHASE1 250

//曲がるまでの待機時間cnt
#define WAIT_PHASE4 200

#define FORWARD_STAIR 80;

void stair_up(int target);
bool stair_up_detect(int gyro);

//****************************************************************************
// 関数名　：　stair_up
// 引数 ：　gyro（ジャイロセンサの値）
//　返り値 : T/F Ｔ→非転倒　Ｆ→転倒
// 概要　：　転倒検知時にモーターストップをかける
//****************************************************************************
void stair_up(int target){
	//変数宣言 OR 引き渡し
	EV3RT_sensor_param sensor;
	signed char pwm_L, pwm_R;
	int sensor_ave = 0;
	int sensor_sum = 0;
	int cnt = 0;
	int phase = 1;
	int turn = 0;
	int forward;
	forward = FORWARD_STAIR;
	
	while(1){
		sensor = GetParam();
		turn = pid_reflection(sensor.color, target);
			
		switch(phase){
			//乗り越えるまでの待機　phase2へ
			case 1:	
			//ev3_speaker_play_tone(NOTE_F6,50);
			balance_control(
			(float)forward,
			(float)turn,
			(float)sensor.gyro,
			(float)GYRO_OFFSET,
			(float)sensor.left,
			(float)sensor.right,
			(float)sensor.volt,
			&pwm_L,
			&pwm_R);
			
			if(cnt == WAIT_PHASE4){
				phase = 2;
			}
			
			cnt = cnt + 1;
			
			break;
			
			
			//定常状態になったらphase3へ
			case 2:
			//ev3_speaker_play_tone(NOTE_D6,1000);
			balance_control(
			(float)forward,
			(float)turn,
			(float)sensor.gyro,
			(float)GYRO_OFFSET,
			(float)sensor.left,
			(float)sensor.right,
			(float)sensor.volt,
			&pwm_L,
			&pwm_R);
			
			if(cnt < 250){
				sensor_ave = sensor_sum / 250;
				sensor_sum = 0;
				cnt = 0;
			}
			
			if(sensor_ave < 10){
				phase = 3;
				cnt = 0;
			}
			
			cnt = cnt + 1;
			sensor_sum = sensor_sum + sensor.gyro;
			
			break;
			
			
			//段差を検知したらphase4へ
			case 3:
			//ev3_speaker_play_tone(NOTE_E6,50);
			forward = 30;
			balance_control(
			(float)forward,
			(float)turn,
			(float)sensor.gyro,
			(float)GYRO_OFFSET,
			(float)sensor.left,
			(float)sensor.right,
			(float)sensor.volt,
			&pwm_L,
			&pwm_R);
		
			if(sensor.gyro > 120){
				phase = 4;
			}
			break;
				
			//曲がるまでの待機　WAIT_PHASE4カウントまったらphase5へ
			case 4:	
			//ev3_speaker_play_tone(NOTE_F6,50);
			forward = FORWARD_STAIR;
			balance_control(
			(float)forward,
			(float)turn,
			(float)sensor.gyro,
			(float)GYRO_OFFSET,
			(float)sensor.left,
			(float)sensor.right,
			(float)sensor.volt,
			&pwm_L,
			&pwm_R);
			
			if(cnt == WAIT_PHASE4){
				phase = 5;
			}
			
			cnt = cnt + 1;
			
			break;
			
			//90度曲がりたくて、がんばったらphase6へ
			case 5:	
			//ev3_speaker_play_tone(NOTE_A6,50);
			balance_control(
			(float)forward,
			(float)TURN_PHASE5,
			(float)sensor.gyro,
			(float)GYRO_OFFSET,
			(float)sensor.left,
			(float)sensor.right,
			(float)sensor.volt,
			&pwm_L,
			&pwm_R);
			
			
			if(sensor.gyro > 120){
				phase = 6;
			}
			break;
			
			//落下後のバランス調整　WAIT_PHASE4カウントまったらphase7へ
			case 6:	
			//ev3_speaker_play_tone(NOTE_B6,50);
			balance_control(
			(float)forward,
			(float)TURN_PHASE5,
			(float)sensor.gyro,
			(float)GYRO_OFFSET,
			(float)sensor.left,
			(float)sensor.right,
			(float)sensor.volt,
			&pwm_L,
			&pwm_R);
			
			if(cnt == 0){
				phase = 7;
			}
			
			if(cnt == 25){
				phase = 7;
			}
			
			cnt = cnt + 1;
			
			break;
			
			//線を探して、乗っかったらおわり
			case 7:	
			//ev3_speaker_play_tone(NOTE_C5,50);
			
			break;
			
		default:
			break;
		}
	
		//衝撃後、反動待機 250回　= 1s
		//for(int wait = 0;wait < 250; wait++){
		//}
		
		//走行
		EV3RT_Running(pwm_L, pwm_R,turn);
		
		//緊急停止用
		if (ev3_touch_sensor_is_pressed(touch_sensor) == 1){
			EV3RT_Running(0, 0,0);
			break; /* タッチセンサが押された */
		}
		
		tslp_tsk(4);
	}
}

//****************************************************************************
// 関数名　：　stair_up_detect
// 引数 ：　gyro（ジャイロセンサの値）
//　返り値 : T/F Ｔ→非転倒　Ｆ→転倒
// 概要　：　転倒検知時にモーターストップをかける
//****************************************************************************
bool stair_up_detect(int gyro){
	
	if(gyro > 120){
		return true;
	}
	return false;
}

