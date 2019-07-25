#include "func.h"


bool 	Barrier_sensor 			(void);
void 	change_tailRunning_Mode	(void);
int 	tilt_pid_reflection		(int sensor_val, int target_val);


//****************************************************************************
// é¢æ°åã?¼ãBarrier_sensor
// å¼æ° ?¼ããªã?
// è¿ãå¤ : bool
// æ¦è¦ã?¼ãã«ã?ã¯ã¢ã?ãã²ã¼ããæ¤ç¥ãããtrueãè¿ãã?
//****************************************************************************
bool Barrier_sensor(){
	if(cnt_barrier >= 200/4){//0.5ç§ä»¥ä¸éå®³ç©åã§æ­¢ã¾ã£ã¦ã? ãã¨ãã¨500
		return true;
		ev3_speaker_play_tone(NOTE_C4,1000);
	}
	if(sonar_alert() == 1){
		cnt_barrier = cnt_barrier + 1;
	}
	return false;
}


//****************************************************************************
// é¢æ°åã?¼ãchange_tailRunning_Mode
// å¼æ° ?¼ããªã?
// è¿ãå¤ : void
// æ¦è¦ã?¼ãå°»å°¾èµ°è¡ã«ç§»è¡ããï¼æéå¶å¾¡ã¯ãã?®é¢æ°å?ã§è¡ã?¼?
//****************************************************************************
void change_tailRunning_Mode(){
	//å¤æ°å®ç¾©ã¨åæå?
	EV3RT_sensor_param sensor;
	int phase = 0;
	int turn = 0;
	int forward = 0;
	signed char pwm_R = 0,pwm_L = 0;
	int timer = 0;

	int loop_flg = 1;
	while(loop_flg){
		switch(phase){

		case 0: //phase0 å°»å°¾ãä¸ãã?
			//ã»ã³ãµå¤åå¾?
			sensor = GetParam();
			//å°»å°¾ãä¸ãã? (80åº¦ã¾ã§ã?ã£ãã)
			tail_control(STOP_TAIL_ANGLE,P_GAIN_STOP);
			//ãã©ã³ã¹ãã¨ã?
			EV3RT_Balancer(sensor,forward,turn,&pwm_L,&pwm_R);
			EV3RT_Running(pwm_L,pwm_R);

			//ã«ã¦ã³ãã¢ã?ã?
			timer += 1;

			// 1sçµã£ãã
			if(timer > 250){
				//æ¬¡ã®ãã§ã¼ãºã¸
				phase = 1;
				timer = 0;
				//ev3_speaker_play_tone(NOTE_G4,500);
			}
			break;

		case 1: // phase1 å°ãåã«é²ã
			//å°ãåã¸
			forward = STOP_MOTOR_PARAM;
			//ã»ã³ãµå¤åå¾?
			sensor = GetParam();
			//å°»å°¾ãä¸ãã? (80åº¦ã§ã­ã¼ã?)
			tail_control(STOP_TAIL_ANGLE,P_GAIN_FORWARD);
			//ãã©ã³ã¹ãã¨ã?
			EV3RT_Balancer(sensor,forward,turn,&pwm_L,&pwm_R);
			//ã¡ã?ã£ã¨ã?ãé²ã
			EV3RT_Running(pwm_L,pwm_R);

			//ã«ã¦ã³ãã¢ã?ã?
			timer += 1;

			// 0.1sçµã£ãã
			if(timer > 25){
				//æ¬¡ã®ãã§ã¼ãºã¸
				phase = 2;
				timer = 0;
				//ev3_speaker_play_tone(NOTE_C5,500);
			}
			break;

		case 2: // phase2 ãã©ã³ã¹ã¨ããã¨ãæ¾æ£?ãã¦å°»å°¾ã«ä½éãä¹ãã?
			//ã¾ã?å°ãåã«é²ã + å°»å°¾ã§ãã©ã³ã¹ã¨ãã?®ã§ã»ã³ãµå¤ã¯è¦ããªã?
			pwm_R = forward;
			pwm_L = pwm_R;
			//å°»å°¾ãä¸ãã? (80åº¦ã§ã­ã¼ã?)
			tail_control(STOP_TAIL_ANGLE,P_GAIN_FORWARD);
			//ã¡ã?ã£ã¨ã?ãé²ã
			EV3RT_Running(pwm_L,pwm_R);

			// ã«ã¦ã³ãã¢ã?ã?
			timer += 1;

			// 0.3sçµã£ãã
			if(timer > 25 * 3){
				//æ¬¡ã®ãã§ã¼ãºã¸
				phase = 3;
				timer = 0;
				//ev3_speaker_play_tone(NOTE_G5,500);
			}
			break;

		case 3: //phase3 å°»å°¾ã®è§åº¦ãæå¤§ã«åãã?
			tail_control(TILT_MOTOR_PARAM,P_GAIN_STOP);
			timer += 1;

			// 2ç§ãã£ãã
			if(timer >= 500){
				phase = 99;
				timer = 0;
			}


		case 99: //çµäº?å¦ç?
			//åæ­¢ãã
			pwm_R = 0;
			pwm_L = 0;
			//ã¨ã¾ã?
			EV3RT_Running(pwm_L,pwm_R);
			//ev3_speaker_play_tone(NOTE_C6,500);

			//ã«ã¼ããæãã?
			loop_flg = 0;
			break;
		}
		tslp_tsk(4);		//4msãã¨ã«ç¨¼å? 
	}
}

/*----------------------------------------------------------------------------
 *	é¢æ°å?	:	tilt_pid_reflection
 *	å¼æ°		:	sensor_val(åå°?åã?®å®æ¸¬å¤) ,  target_val(åå°?åã?®ç?æ³å¤)
 *	æ»ãå¤	:	turn (ã©ãã ãæ²ããã?)
 *	æ¦è¦?		:	åå°?åãåºã«ããPIDå¶å¾¡ (å°»å°¾èµ°è¡ç)
 *----------------------------------------------------------------------------*/
int tilt_pid_reflection(int sensor_val, int target_val){
	int p,i,d;
	reflection_diff[0] = reflection_diff[1];
	reflection_diff[1] = sensor_val - target_val;

	p = TILT_KP * reflection_diff[1];
	i = KI * (reflection_diff[0] + reflection_diff[1]) * DELTA_T / 2; 
	d = KD * (reflection_diff[1] - reflection_diff[0]) / DELTA_T;

	return (int)(p+i+d);
}