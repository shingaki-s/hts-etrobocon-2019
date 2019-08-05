#ifndef _FUNC_H_
#define _FUNC_H_

#ifndef _EV3API_H_
#include "ev3api.h"
#endif //_EV3API_H_

#ifndef _STDBOOL_H_
#include <stdbool.h>
#endif

// ===============構造体================== //
//センサ値
typedef struct{
	// メンバ変数の定義 
	int color;
	int gyro;
	int tail;
	int sonar;
	int volt;
	int left;
	int right;
}  EV3RT_sensor_param;
//場所
typedef struct {
	float x;
	float y;
	float theta;
}Position ,*p_position;


//===センサー、モーターの接続を定義 ===/
static const sensor_port_t
	touch_sensor    = EV3_PORT_1,
	sonar_sensor    = EV3_PORT_2,
	color_sensor    = EV3_PORT_3,
	gyro_sensor     = EV3_PORT_4;

static const motor_port_t
	left_motor      = EV3_PORT_C,
	right_motor     = EV3_PORT_B,
	tail_motor      = EV3_PORT_A;

//==============制御系パラメータ設定===================//
#define GYRO_OFFSET  0          /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */
//#define TAIL_ANGLE_STAND_UP  90 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_STAND_UP  85 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3 /* バランス走行時の角度[度] */
#define TAIL_ANGLE_DRIVE_2    85 /* バランス走行時の角度[度] */
#define P_GAIN_FORWARD     2.5F /* 完全停止用モータ制御比例係数 */
#define P_GAIN_STOP		   0.3F /* 停止時尻尾下げ速度 */
#define PWM_ABS_MAX          60 /* 完全停止用モータ制御PWM絶対最大値 */
/*PID制御用パラメタ*/

//#define DELTA_T 0.004
#define DELTA_T 0.004

//#define KP 0.38
#define KP 0.70
//#define KP 0.64
//#define KP 0.66
//#define KP 0.72
//#define KP 0.705
//#define KP 0.40
//#define KP 0.50

//#define KI 0.002
//#define KI 0.003
//#define KI 0.0022
//#define KI 0.05
//#define KI 0.0015
#define KI 0.002
//#define KI 1.2
//#define KI 0.03
//#define KI 0


//#define KD 0.09
//#define KD 0.15
//#define KD 0.20
//#define KD 0.05
//#define KD 0.10
//#define KD 0.12
//#define KD 0.11
#define KD 0.09
//#define KD 0.085
//#define KD 0.10
//#define KD 0.03
//#define KD 0

/* KDはKP,KIとは異符号にしなければならないのでは？ */
/*TILT_PID制御用パラメタ*/
#define TILT_KP 4.20
/*尻尾を用いた3点でたつときのパラメタ*/
#define STOP_TAIL_ANGLE 71 /*尻尾を着地させるためのパラメタ68*/
#define STOP_MOTOR_PARAM 20 /*ちょっとだけ進むためのパラメタ*/
#define TAIL MOTOR PARAM 10 /*仮決め定数、リンボー後のスピード*/
#define TILT_MOTOR_PARAM 80 /*斜め状態になるためのパラメタ*/
/*自己位置推定用パラメタ*/
#define WHEEL_R 10

#define EV3RT_WIDTH 16.2
#define PI 3.14159
/*転倒検知パラメタ*/
#define FALL_GYRO_PARAM 30

// 尻尾走行用CalibParam
#define TILT_RUNNING_LIGHT_PARAM 10

// モータ判別用
#define LEFT_MOTOR 		0
#define RIGHT_MOTOR 	1

//スピードダウン先の値
#define GRAY_DETECT_SPEED 30


//リンボーのときの車庫までの距離
#define DISTANCE_GARAGE_LIMBO 2400	//本番用。1m先が車庫
//#define DISTANCE_GARAGE_LIMBO 600
#define DISTANCE_GARAGE_LIMBO_A 700
#define DISTANCE_GARAGE_LIMBO_B 800

#define DISTANCE_GARAGE_STAIRS 650

// 角度によるコース終わり検知用パラメータ(リンボー用)
#define DETECT_COURSE_END_DISTANCE 2200	//本番用。2000にしたら10m進むで！
//#define DETECT_COURSE_END_DISTANCE 600
#define DETECT_COURCE_END_DEGREE   4.712385 	// 3/2 PI() 角度が変わったら
#define DETECT_COURCE_END_DEGREE_DELTA 0.392698	// ずれ補正用
//===================================================//

//=================表示系パラメータ設定========================//
/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)
//==================================================//

//-------------何秒間サンプルとるか
#define DEGREE_CALIB_SAMPLING_TIME 5



/*PID制御用buffer*/
static int reflection_diff[2] = {0,0};
int deg_diff[2];


//---------角度キャリブレーション用-------//
float sumX;
float sumY;
float sumXbyX;
float sumXbyY;
int deg_calib_counter;
int speed_down_counter;

//*********障害確認用カウンタ**********//
int cnt_barrier;



void 				First_setup(void);
EV3RT_sensor_param 	GetParam(void);
void 				EV3RT_Running(signed char pwm_L, signed char pwm_R);
int 				pid_reflection(int sensor_val, int target_val);
int 				tilt_pid_reflection(int sensor_val, int target_val);
void 				tail_control(int angle);	
void 				EV3RT_Balancer(EV3RT_sensor_param sensor, int forward, int turn,signed char *pwm_L, signed char *pwm_R);
int 				sonar_alert(void);
int 				light_reflection_calibration(void);
void 				initialize_paramater(void);
bool 				Barrier_sensor(void);
void 				change_tailRunning_Mode(void);
void 				stair_up(int target);
bool 				stair_up_detect(int gyro);
int 				Speed_adjust(int turn);
bool 				gray_detection(int now_color);

#endif //_FUNC_H_
