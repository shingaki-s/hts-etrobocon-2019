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

//髢｢謨ｰ繝励Ο繝医ち繧､繝怜ｮ夂ｾｩ
void 				First_setup(void);
EV3RT_sensor_param 	GetParam(void);
void 				EV3RT_Running(signed char pwm_L, signed char pwm_R);
int 				pid_reflection(int sensor_val, int target_val);
void 				tail_control(signed int angle, float tail_speed);
void 				EV3RT_Balancer(EV3RT_sensor_param sensor, int forward, int turn,signed char *pwm_L, signed char *pwm_R);
int 				sonar_alert(void);
int 				light_reflection_calibration(void);
void 				initialize_paramater(void);

void First_setup(){
	/* 繧ｻ繝ｳ繧ｵ繝ｼ蜈･蜉帙�昴�ｼ繝医�ｮ險ｭ螳� */
	ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
	ev3_sensor_config(color_sensor, COLOR_SENSOR);
	ev3_color_sensor_get_reflect(color_sensor); /* 蜿榊ｰ�邇�繝｢繝ｼ繝� */
	ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
	ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
	/* 繝｢繝ｼ繧ｿ繝ｼ蜃ｺ蜉帙�昴�ｼ繝医�ｮ險ｭ螳� */
	ev3_motor_config(left_motor, LARGE_MOTOR);
	ev3_motor_config(right_motor, LARGE_MOTOR);
	ev3_motor_config(tail_motor, LARGE_MOTOR);
}


//****************************************************************************
// 髢｢謨ｰ蜷阪�ｼ壹GetParam
// 蠑墓焚 �ｼ壹辟｡縺�
//縲霑斐ｊ蛟､ : 繧ｻ繝ｳ繧ｵ蛟､縺吶∋縺ｦ
// 讎りｦ√�ｼ壹繧ｻ繝ｳ繧ｵ蛟､蜿門ｾ鈴未謨ｰ
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
// 髢｢謨ｰ蜷阪�ｼ壹EV3RT_Running
// 蠑墓焚 �ｼ壹  pwm_L, pwm_R
//縲霑斐ｊ蛟､ :   縺ｪ縺� 
// 讎りｦ√�ｼ壹襍ｰ陦後ｒ陦後≧
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
 *	髢｢謨ｰ蜷�	:	pid_reflection
 *	蠑墓焚		:	sensor_val(蜿榊ｰ�蜈峨�ｮ螳滓ｸｬ蛟､) ,  target_val(蜿榊ｰ�蜈峨�ｮ逅�諠ｳ蛟､)
 *	謌ｻ繧雁､	:	turn (縺ｩ繧後□縺第峇縺後ｋ縺�)
 *	讎りｦ�		:	蜿榊ｰ�蜈峨ｒ蝓ｺ縺ｫ縺励◆PID蛻ｶ蠕｡ 
 *----------------------------------------------------------------------------*/
int pid_reflection(int sensor_val, int target_val){
    /* 霑ｽ蜉� */
	//int p,i,d;
    float p, i, d;
    static float total = 0;
	float diff;
	// if (sensor_val > 35){
	// 	sensor_val -= 18;
	// }

	diff = sensor_val - target_val;

	p = KP * (diff - reflection_diff[1]);

    i = KI * (diff + reflection_diff[1]) * DELTA_T / 2;

	d = KD * (diff - 2*reflection_diff[1] + reflection_diff[0]) / DELTA_T;

	reflection_diff[0] = reflection_diff[1];
	reflection_diff[1] = diff;

	total = total + p + i + d;

	return (int)(total);
}

//*****************************************************************************
// 髢｢謨ｰ蜷� : tail_control
// 蠑墓焚 : angle (繝｢繝ｼ繧ｿ逶ｮ讓呵ｧ貞ｺｦ[蠎ｦ]), tail_speed(蟆ｻ蟆ｾ繧剃ｸ九ｍ縺咎溷ｺｦ)
// 霑斐ｊ蛟､ : 辟｡縺�
// 讎りｦ� : 襍ｰ陦御ｽ灘ｮ悟�ｨ蛛懈ｭ｢逕ｨ繝｢繝ｼ繧ｿ縺ｮ隗貞ｺｦ蛻ｶ蠕｡
//*****************************************************************************
void tail_control(signed int angle, float tail_speed){

	float pwm = (float)(angle - ev3_motor_get_counts(tail_motor)) * tail_speed;/* 豈比ｾ句宛蠕｡ */
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
// 髢｢謨ｰ蜷阪�ｼ壹EV3RT_Balancer
// 蠑墓焚 �ｼ壹縲
//縲霑斐ｊ蛟､ : 
// 讎りｦ√�ｼ壹
//****************************************************************************
void EV3RT_Balancer(EV3RT_sensor_param sensor, int forward, int turn,signed char *pwm_L, signed char *pwm_R){
	//繝｢繝ｼ繧ｿ繝ｼ蜃ｺ蜉帶ｱｺ螳�
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
// 髢｢謨ｰ蜷� : sonar_alert
// 蠑墓焚 : 辟｡縺�
// 霑斐ｊ蛟､ : 1(髫懷ｮｳ迚ｩ縺ゅｊ)/0(髫懷ｮｳ迚ｩ辟｡縺�)
// 讎りｦ� : 雜�髻ｳ豕｢繧ｻ繝ｳ繧ｵ縺ｫ繧医ｋ髫懷ｮｳ迚ｩ讀懃衍
//*****************************************************************************
int sonar_alert(void)
{
	static unsigned int counter = 0;
	static int alert = 0;

	signed int distance;

	if (++counter == 40/4) /* 邏�40msec蜻ｨ譛滓ｯ弱↓髫懷ｮｳ迚ｩ讀懃衍  */
	{
		/*
		 * 雜�髻ｳ豕｢繧ｻ繝ｳ繧ｵ縺ｫ繧医ｋ霍晞屬貂ｬ螳壼捉譛溘�ｯ縲∬ｶ�髻ｳ豕｢縺ｮ貂幄｡ｰ迚ｹ諤ｧ縺ｫ萓晏ｭ倥＠縺ｾ縺吶�
		 * NXT縺ｮ蝣ｴ蜷医�ｯ縲�40msec蜻ｨ譛溽ｨ句ｺｦ縺檎ｵ碁ｨ謎ｸ翫�ｮ譛遏ｭ貂ｬ螳壼捉譛溘〒縺吶�
		 * EV3縺ｮ蝣ｴ蜷医�ｯ縲∬ｦ∫｢ｺ隱�
		 */

		distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
		if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
		{
			alert = 1; /* 髫懷ｮｳ迚ｩ繧呈､懃衍 */
		}
		else
		{
			alert = 0; /* 髫懷ｮｳ迚ｩ辟｡縺� */
		}
		counter = 0;
	}

	return alert;
}


int light_reflection_calibration(){
/*=====================================繧ｭ繝｣繝ｪ繝悶Ξ繝ｼ繧ｷ繝ｧ繝ｳ==========================================*/
	/*------------------------ set up ----------------------------*/
	int white_line_reflection = 0;
	int black_line_reflection = 0;
	int cnt = 0;						/*counter*/



	/*-------------------- get white param -----------------*/
	while(1){	//繝懊ち繝ｳ縺梧款縺輔ｌ繧九∪縺ｧ繧ｦ繧ｧ繧､繝�
		if(ev3_touch_sensor_is_pressed(touch_sensor) == 1){
			break;
		}
		tslp_tsk(10); /* 10msec繧ｦ繧ｧ繧､繝� */
	}
	while(1){	//繝懊ち繝ｳ謚ｼ荳句ｾ�
		/*繝�繝ｼ繧ｿ繧呈�ｼ邏�*/
		white_line_reflection += ev3_color_sensor_get_reflect(color_sensor);
		cnt++;
		if(cnt == 20){
			white_line_reflection = white_line_reflection / 20;

			break;
		}
		tslp_tsk(4); /* 4msec繧ｦ繧ｧ繧､繝� */
	}
	
	
	tslp_tsk(1000);


	/*-------------------- get black param -----------------*/
	while(1){//繝懊ち繝ｳ縺梧款縺輔ｌ繧九∪縺ｧ繧ｦ繧ｧ繧､繝�
		if(ev3_touch_sensor_is_pressed(touch_sensor) == 1){
			cnt = 0;
			break;
		}
		tslp_tsk(10); /* 10msec繧ｦ繧ｧ繧､繝� */
	}
	while(1){		//繝懊ち繝ｳ謚ｼ荳句ｾ�
		/*繝�繝ｼ繧ｿ繧呈�ｼ邏�*/
		black_line_reflection += ev3_color_sensor_get_reflect(color_sensor);
		cnt++; //count up
		if(cnt == 20){//蟷ｳ蝮�蛟､貍皮ｮ�
			black_line_reflection = black_line_reflection / 20;
			break;
		}
		tslp_tsk(4); /* 4msec繧ｦ繧ｧ繧､繝� */
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

