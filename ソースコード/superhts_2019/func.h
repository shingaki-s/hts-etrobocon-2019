#ifndef _FUNC_H_
#define _FUNC_H_

#ifndef _EV3API_H_
#include "ev3api.h"
#endif //_EV3API_H_

#ifndef _STDBOOL_H_
#include <stdbool.h>
#endif

// ===============�\����================== //
//�Z���T�l
typedef struct{
	// �����o�ϐ��̒�` 
	int color;
	int gyro;
	int tail;
	int sonar;
	int volt;
	int left;
	int right;
}  EV3RT_sensor_param;
//�ꏊ
typedef struct {
	float x;
	float y;
	float theta;
}Position ,*p_position;


//===�Z���T�[�A���[�^�[�̐ڑ����` ===/
static const sensor_port_t
	touch_sensor    = EV3_PORT_1,
	sonar_sensor    = EV3_PORT_2,
	color_sensor    = EV3_PORT_3,
	gyro_sensor     = EV3_PORT_4;

static const motor_port_t
	left_motor      = EV3_PORT_C,
	right_motor     = EV3_PORT_B,
	tail_motor      = EV3_PORT_A;

//==============����n�p�����[�^�ݒ�===================//
#define GYRO_OFFSET  0          /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
#define SONAR_ALERT_DISTANCE 30 /* �����g�Z���T�ɂ���Q�����m����[cm] */
//#define TAIL_ANGLE_STAND_UP  90 /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_STAND_UP  85 /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE      3 /* �o�����X���s���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE_2    85 /* �o�����X���s���̊p�x[�x] */
#define P_GAIN_FORWARD     2.5F /* ���S��~�p���[�^������W�� */
#define P_GAIN_STOP		   0.3F /* ��~���K���������x */
#define PWM_ABS_MAX          60 /* ���S��~�p���[�^����PWM��΍ő�l */
/*PID����p�p�����^*/

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

/* KD��KP,KI�Ƃ͈ٕ����ɂ��Ȃ���΂Ȃ�Ȃ��̂ł́H */
/*TILT_PID����p�p�����^*/
#define TILT_KP 4.20
/*�K����p����3�_�ł��Ƃ��̃p�����^*/
#define STOP_TAIL_ANGLE 71 /*�K���𒅒n�����邽�߂̃p�����^68*/
#define STOP_MOTOR_PARAM 20 /*������Ƃ����i�ނ��߂̃p�����^*/
#define TAIL MOTOR PARAM 10 /*�����ߒ萔�A�����{�[��̃X�s�[�h*/
#define TILT_MOTOR_PARAM 80 /*�΂ߏ�ԂɂȂ邽�߂̃p�����^*/
/*���Ȉʒu����p�p�����^*/
#define WHEEL_R 10

#define EV3RT_WIDTH 16.2
#define PI 3.14159
/*�]�|���m�p�����^*/
#define FALL_GYRO_PARAM 30

// �K�����s�pCalibParam
#define TILT_RUNNING_LIGHT_PARAM 10

// ���[�^���ʗp
#define LEFT_MOTOR 		0
#define RIGHT_MOTOR 	1

//�X�s�[�h�_�E����̒l
#define GRAY_DETECT_SPEED 30


//�����{�[�̂Ƃ��̎Ԍɂ܂ł̋���
#define DISTANCE_GARAGE_LIMBO 2400	//�{�ԗp�B1m�悪�Ԍ�
//#define DISTANCE_GARAGE_LIMBO 600
#define DISTANCE_GARAGE_LIMBO_A 700
#define DISTANCE_GARAGE_LIMBO_B 800

#define DISTANCE_GARAGE_STAIRS 650

// �p�x�ɂ��R�[�X�I��茟�m�p�p�����[�^(�����{�[�p)
#define DETECT_COURSE_END_DISTANCE 2200	//�{�ԗp�B2000�ɂ�����10m�i�ނŁI
//#define DETECT_COURSE_END_DISTANCE 600
#define DETECT_COURCE_END_DEGREE   4.712385 	// 3/2 PI() �p�x���ς������
#define DETECT_COURCE_END_DEGREE_DELTA 0.392698	// ����␳�p
//===================================================//

//=================�\���n�p�����[�^�ݒ�========================//
/* LCD�t�H���g�T�C�Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)
//==================================================//

//-------------���b�ԃT���v���Ƃ邩
#define DEGREE_CALIB_SAMPLING_TIME 5



/*PID����pbuffer*/
static int reflection_diff[2] = {0,0};
int deg_diff[2];


//---------�p�x�L�����u���[�V�����p-------//
float sumX;
float sumY;
float sumXbyX;
float sumXbyY;
int deg_calib_counter;
int speed_down_counter;

//*********��Q�m�F�p�J�E���^**********//
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
