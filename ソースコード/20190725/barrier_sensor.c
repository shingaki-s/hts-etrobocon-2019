#include "func.h"


bool 	Barrier_sensor 			(void);
void 	change_tailRunning_Mode	(void);
int 	tilt_pid_reflection		(int sensor_val, int target_val);


//****************************************************************************
// 関数名　?��　Barrier_sensor
// 引数 ?��　な�?
// 返り値 : bool
// 概要　?��　ル�?クア�?プゲートを検知したらtrueを返す�?
//****************************************************************************
bool Barrier_sensor(){
	if(cnt_barrier >= 200/4){//0.5秒以上障害物前で止まって�? もともと500
		return true;
		ev3_speaker_play_tone(NOTE_C4,1000);
	}
	if(sonar_alert() == 1){
		cnt_barrier = cnt_barrier + 1;
	}
	return false;
}


//****************************************************************************
// 関数名　?��　change_tailRunning_Mode
// 引数 ?��　な�?
// 返り値 : void
// 概要　?��　尻尾走行に移行する（時間制御はこ�?�関数�?で行う?�?
//****************************************************************************
void change_tailRunning_Mode(){
	//変数定義と初期�?
	EV3RT_sensor_param sensor;
	int phase = 0;
	int turn = 0;
	int forward = 0;
	signed char pwm_R = 0,pwm_L = 0;
	int timer = 0;

	int loop_flg = 1;
	while(loop_flg){
		switch(phase){

		case 0: //phase0 尻尾を下げ�?
			//センサ値取�?
			sensor = GetParam();
			//尻尾を下げ�? (80度まで�?っくり)
			tail_control(STOP_TAIL_ANGLE,P_GAIN_STOP);
			//バランスもと�?
			EV3RT_Balancer(sensor,forward,turn,&pwm_L,&pwm_R);
			EV3RT_Running(pwm_L,pwm_R);

			//カウントア�?�?
			timer += 1;

			// 1s経ったら
			if(timer > 250){
				//次のフェーズへ
				phase = 1;
				timer = 0;
				//ev3_speaker_play_tone(NOTE_G4,500);
			}
			break;

		case 1: // phase1 少し前に進む
			//少し前へ
			forward = STOP_MOTOR_PARAM;
			//センサ値取�?
			sensor = GetParam();
			//尻尾を下げ�? (80度でキー�?)
			tail_control(STOP_TAIL_ANGLE,P_GAIN_FORWARD);
			//バランスもと�?
			EV3RT_Balancer(sensor,forward,turn,&pwm_L,&pwm_R);
			//ち�?っと�?け進む
			EV3RT_Running(pwm_L,pwm_R);

			//カウントア�?�?
			timer += 1;

			// 0.1s経ったら
			if(timer > 25){
				//次のフェーズへ
				phase = 2;
				timer = 0;
				//ev3_speaker_play_tone(NOTE_C5,500);
			}
			break;

		case 2: // phase2 バランスとることを放�?して尻尾に体重を乗せ�?
			//ま�?少し前に進む + 尻尾でバランスとる�?�でセンサ値は要らな�?
			pwm_R = forward;
			pwm_L = pwm_R;
			//尻尾を下げ�? (80度でキー�?)
			tail_control(STOP_TAIL_ANGLE,P_GAIN_FORWARD);
			//ち�?っと�?け進む
			EV3RT_Running(pwm_L,pwm_R);

			// カウントア�?�?
			timer += 1;

			// 0.3s経ったら
			if(timer > 25 * 3){
				//次のフェーズへ
				phase = 3;
				timer = 0;
				//ev3_speaker_play_tone(NOTE_G5,500);
			}
			break;

		case 3: //phase3 尻尾の角度を最大に取る�?
			tail_control(TILT_MOTOR_PARAM,P_GAIN_STOP);
			timer += 1;

			// 2秒たったら
			if(timer >= 500){
				phase = 99;
				timer = 0;
			}


		case 99: //終�?処�?
			//停止する
			pwm_R = 0;
			pwm_L = 0;
			//とま�?
			EV3RT_Running(pwm_L,pwm_R);
			//ev3_speaker_play_tone(NOTE_C6,500);

			//ループを抜け�?
			loop_flg = 0;
			break;
		}
		tslp_tsk(4);		//4msごとに稼�? 
	}
}

/*----------------------------------------------------------------------------
 *	関数�?	:	tilt_pid_reflection
 *	引数		:	sensor_val(反�?光�?�実測値) ,  target_val(反�?光�?��?想値)
 *	戻り値	:	turn (どれだけ曲がる�?)
 *	概�?		:	反�?光を基にしたPID制御 (尻尾走行版)
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