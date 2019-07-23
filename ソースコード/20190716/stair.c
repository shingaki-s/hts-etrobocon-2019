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

//90�x�Ȃ���ۂ�turn�̒l
#define TURN_PHASE5 -30


//�Ȃ���܂ł̑ҋ@����cnt
#define WAIT_PHASE1 250

//�Ȃ���܂ł̑ҋ@����cnt
#define WAIT_PHASE4 200

#define FORWARD_STAIR 80;

void stair_up(int target);
bool stair_up_detect(int gyro);

//****************************************************************************
// �֐����@�F�@stair_up
// ���� �F�@gyro�i�W���C���Z���T�̒l�j
//�@�Ԃ�l : T/F �s����]�|�@�e���]�|
// �T�v�@�F�@�]�|���m���Ƀ��[�^�[�X�g�b�v��������
//****************************************************************************
void stair_up(int target){
	//�ϐ��錾 OR �����n��
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
			//���z����܂ł̑ҋ@�@phase2��
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
			
			
			//����ԂɂȂ�����phase3��
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
			
			
			//�i�������m������phase4��
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
				
			//�Ȃ���܂ł̑ҋ@�@WAIT_PHASE4�J�E���g�܂�����phase5��
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
			
			//90�x�Ȃ��肽���āA����΂�����phase6��
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
			
			//������̃o�����X�����@WAIT_PHASE4�J�E���g�܂�����phase7��
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
			
			//����T���āA����������炨���
			case 7:	
			//ev3_speaker_play_tone(NOTE_C5,50);
			
			break;
			
		default:
			break;
		}
	
		//�Ռ���A�����ҋ@ 250��@= 1s
		//for(int wait = 0;wait < 250; wait++){
		//}
		
		//���s
		EV3RT_Running(pwm_L, pwm_R);
		
		//�ً}��~�p
		if (ev3_touch_sensor_is_pressed(touch_sensor) == 1){
			EV3RT_Running(0, 0);
			break; /* �^�b�`�Z���T�������ꂽ */
		}
		
		tslp_tsk(4);
	}
}

//****************************************************************************
// �֐����@�F�@stair_up_detect
// ���� �F�@gyro�i�W���C���Z���T�̒l�j
//�@�Ԃ�l : T/F �s����]�|�@�e���]�|
// �T�v�@�F�@�]�|���m���Ƀ��[�^�[�X�g�b�v��������
//****************************************************************************
bool stair_up_detect(int gyro){
	
	if(gyro > 120){
		return true;
	}
	return false;
}

