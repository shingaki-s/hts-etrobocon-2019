#include "func.h"

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

#ifndef _STDLIB_H_
#include <stdlib.h>
#endif

//関数プロトタイプ定義
void 				First_setup(void);
EV3RT_sensor_param 	GetParam(void);
void 				EV3RT_Running(signed char pwm_L, signed char pwm_R);
int 				pid_reflection(int sensor_val, int target_val);
void 				tail_control(signed int angle, float tail_speed);
void 				EV3RT_Balancer(EV3RT_sensor_param sensor, int forward, int turn,signed char *pwm_L, signed char *pwm_R);
int 				sonar_alert(void);
int 				light_reflection_calibration(void);
void 				initialize_paramater(void);

static float total = 0;	//PID制御のtotal

void First_setup(){
	/* センサー入力�?��?�ト�?�設�? */
	ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
	ev3_sensor_config(color_sensor, COLOR_SENSOR);
	ev3_color_sensor_get_reflect(color_sensor); /* 反�?�?モー�? */
	ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
	ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
	/* モーター出力�?��?�ト�?�設�? */
	ev3_motor_config(left_motor, LARGE_MOTOR);
	ev3_motor_config(right_motor, LARGE_MOTOR);
	ev3_motor_config(tail_motor, LARGE_MOTOR);
}


//****************************************************************************
// 関数名　?��　GetParam
// 引数 ?��　無�?
//　返り値 : センサ値すべて
// 概要　?��　センサ値取得関数
//****************************************************************************
EV3RT_sensor_param GetParam(){
	EV3RT_sensor_param buf;
	buf.color = ev3_color_sensor_get_reflect(color_sensor);
	buf.sonar = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
	buf.gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
	buf.left = ev3_motor_get_counts(left_motor);
	buf.right = ev3_motor_get_counts(right_motor);
	buf.tail = ev3_motor_get_counts(tail_motor);
	buf.volt = ev3_battery_voltage_mV();

	return buf;
}

//****************************************************************************
// 関数名　?��　EV3RT_Running
// 引数 ?��　  pwm_L, pwm_R
//　返り値 :   な�? 
// 概要　?��　走行を行う
//****************************************************************************
void EV3RT_Running(signed char pwm_L, signed char pwm_R){
	if (pwm_L == 0){
		ev3_motor_stop(left_motor, true);
	}else{
		ev3_motor_set_power(left_motor, (int)pwm_L);
	}
	
	if (pwm_R == 0){
		ev3_motor_stop(right_motor, true);
	}else{
		ev3_motor_set_power(right_motor, (int)pwm_R);
	}
}

/*----------------------------------------------------------------------------
 *	関数�?	:	pid_reflection
 *	引数		:	sensor_val(反�?光�?�実測値) ,  target_val(反�?光�?��?想値)
 *	戻り値	:	turn (どれだけ曲がる�?)
 *	概�?		:	反�?光を基にしたPID制御 
 *----------------------------------------------------------------------------*/
int pid_reflection(int sensor_val, int target_val){
    /* 追�? */
	//int p,i,d;
    float p, i, d;
 //   static float total;
	float diff;

	diff = sensor_val - target_val;

	p = KP * (diff - reflection_diff[1]);

    i = KI * (diff + reflection_diff[1]) * DELTA_T / 2;

	d = KD * (diff - 2*reflection_diff[1] + reflection_diff[0]) / DELTA_T;

	reflection_diff[0] = reflection_diff[1];
	reflection_diff[1] = diff;

	total = total + p + i + d;
	// if(total >= 100){
	// 	total = 100;
	// }else if(total <= -100){
	// 	total = -100;
	// }

	return (int)(total);
}

//*****************************************************************************
// 関数�? : tail_control
// 引数 : angle (モータ目標角度[度]), tail_speed(尻尾を下ろす速度)
// 返り値 : 無�?
// 概�? : 走行体完�?�停止用モータの角度制御
//*****************************************************************************
void tail_control(signed int angle, float tail_speed){

	float pwm = (float)(angle - ev3_motor_get_counts(tail_motor)) * tail_speed;/* 比例制御 */
	if (pwm > PWM_ABS_MAX){
		pwm = PWM_ABS_MAX;
	}else if (pwm < -PWM_ABS_MAX){
		pwm = -PWM_ABS_MAX;
	}

	if (pwm == 0){
		ev3_motor_stop(tail_motor, true);
	}else{
		ev3_motor_set_power(tail_motor, (signed char)pwm);
	}
}




//****************************************************************************
// 関数名　?��　EV3RT_Balancer
// 引数 ?��　　
//　返り値 : 
// 概要　?��　
//****************************************************************************
void EV3RT_Balancer(EV3RT_sensor_param sensor, int forward, int turn,signed char *pwm_L, signed char *pwm_R){
	//モーター出力決�?
	balance_control(
				(float)forward,
				(float)turn,
				(float)sensor.gyro,
				(float)GYRO_OFFSET,
				(float)sensor.left,
				(float)sensor.right,
				(float)sensor.volt,
				pwm_L,
				pwm_R);
}

//*****************************************************************************
// 関数�? : sonar_alert
// 引数 : 無�?
// 返り値 : 1(障害物あり)/0(障害物無�?)
// 概�? : �?音波センサによる障害物検知
//*****************************************************************************
int sonar_alert(void)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if (++counter == 40/4) /* �?40msec周期毎に障害物検知  */
	{
		/*
		 * �?音波センサによる距離測定周期�?�、�?音波の減衰特性に依存します�?
		 * NXTの場合�?��?40msec周期程度が経験上�?�最短測定周期です�?
		 * EV3の場合�?�、要確�?
		 */

		distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
		if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
		{
			alert = 1; /* 障害物を検知 */
		}
		else
		{
			alert = 0; /* 障害物無�? */
		}
		counter = 0;
	}

	return alert;
}


int light_reflection_calibration(){
/*=====================================キャリブレーション==========================================*/
	/*------------------------ set up ----------------------------*/
	int white_line_reflection = 0;
	int black_line_reflection = 0;
	int cnt = 0;						/*counter*/



	/*-------------------- get white param -----------------*/
	while(1){	//ボタンが押されるまでウェイ�?
		if(ev3_touch_sensor_is_pressed(touch_sensor) == 1){
			break;
		}
		tslp_tsk(10); /* 10msecウェイ�? */
	}
	while(1){	//ボタン押下�?
		/*�?ータを�?��?*/
		white_line_reflection += ev3_color_sensor_get_reflect(color_sensor);
		cnt++;
		if(cnt == 20){
			white_line_reflection = white_line_reflection / 20;

			break;
		}
		tslp_tsk(4); /* 4msecウェイ�? */
	}
	
	
	tslp_tsk(1000);


	/*-------------------- get black param -----------------*/
	while(1){//ボタンが押されるまでウェイ�?
		if(ev3_touch_sensor_is_pressed(touch_sensor) == 1){
			cnt = 0;
			break;
		}
		tslp_tsk(10); /* 10msecウェイ�? */
	}
	while(1){		//ボタン押下�?
		/*�?ータを�?��?*/
		black_line_reflection += ev3_color_sensor_get_reflect(color_sensor);
		cnt++; //count up
		if(cnt == 20){//平�?値演�?
			black_line_reflection = black_line_reflection / 20;
			break;
		}
		tslp_tsk(4); /* 4msecウェイ�? */
	}

	tslp_tsk(1000);
	ev3_speaker_play_tone(NOTE_G4,30);

	return (white_line_reflection + black_line_reflection) / 2;

}

void initialize_paramater(){
	sumX = 0;
	sumY = 0;
	sumXbyX = 0;
	sumXbyY = 0;
	deg_calib_counter = 0;
	cnt_barrier = 0;
	deg_calib_counter = 0;
	speed_down_counter = 0;
}

