#ifndef _FUNC_H_
#include "func.h"
#endif
//80�ȏ�̐��l���󂯎�����Ƃ��A�������炵���l��Ԃ�


//�X�s�[�h������(4 * SPEED_AJUST_CALL_FREQUENCY)ms���Ƃɍs��
#define SPEED_AJUST_CALL_FREQUENCY 25

//*****************************************************************************
// �֐��� : Speed_adjust
// ���� : now_speed(���݂̑��x)�Ctarget_speed(�ڕW�̑��x)
// �Ԃ�l : �����������x
// �T�v : ���x��ڕW�l�ɋ߂Â���
//*****************************************************************************
int Speed_adjust(signed char now_speed,int target_speed){
	speed_down_counter++;
	if(speed_down_counter > SPEED_AJUST_CALL_FREQUENCY){
		//�ڕW�l�ƌ��ݒl�̍�
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

