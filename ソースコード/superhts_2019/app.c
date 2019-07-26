#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "func.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef _STDBOOL_H_
#include <stdbool.h>
#endif

//曲がるステート宣言
//State定義
#define PHASE2 2 /* 通常走行 */

/* 階段関連 */
#define PHASE20 20 /* 階段に突入したとき */
#define PHASE21 21 /* 階段を下りた後の状態 */
#define PHASE23 23 /* しっぽ走行を一定距離した後99に行く */
#define PHASE24 24
#define PHASE25 25
#define PHASE26 26
#define PHASE80 80
#define PHASE99 99 /* 終わり。止まる */

static int TURN_STATE;

//============blue tooth ==================/
static int bt_cmd = 0;  /* Bluetoothコマンド 1:リモートスタート */
static FILE *bt = NULL; /* Bluetoothファイルハンドル */
#define CMD_START '1'   /* リモートスタートコマンド */
//========================================//

/* Bluetooth通信タスクの起動 */
//act_tsk(BT_TASK);

void main_task(intptr_t unused)
{

	/*-------------------------セットアップ--------------------------------*/
	// 変数定義
	signed char forward = 60;	   // 前進命令(-100～100)
	signed char turn = 0;		   // 転回命令(-100～100)
	signed char pwm_L, pwm_R;	  // モータ回転量(-100～100)
	char output_string[300] = {0}; // 文字出力用(bluetooth)
	// センサ・モータポートセットアップ
	First_setup();

	/*------------------------blue tooth------------------------------*/
	// Bluetooth接続開始
	bt = ev3_serial_open_file(EV3_SERIAL_BT);
	assert(bt != NULL);
	// Bluetooth通信タスクの起動
	/* bluetoothからの信号が入るとbt_cmdが1になる */
	act_tsk(BT_TASK);

	/*-----------------------キャリブレーション-----------------------------*/
	int calib_light = 0;
	// キャリブレーションを行う（白→黒）
	/* calib_lightに白黒判定のしきい値が入る */
	calib_light = light_reflection_calibration();

	// PCにデータを送信(Bluetooth)
	sprintf(output_string, "calib_light:%d\n", calib_light);
	fputs(output_string, bt);

	// モータの角位置をゼロにリセットする
	ev3_motor_reset_counts(tail_motor);

	/*------------------------スタート待ち------------------------------*/
	while (1)
	{
		// 尻尾で立つ
		tail_control(TAIL_ANGLE_STAND_UP, P_GAIN_FORWARD);

		// タッチセンサーが押された場合スタート
		if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
		{
			break;
		}
		// Bluetoothで1が送られてきたらスタート
		if (bt_cmd == 1)
		{
			break; /* リモートスタート */
		}
		// 10msでループする
		tslp_tsk(10);
	}

	/*-------------------------走行準備-----------------------------*/
	// 走行モーターエンコーダーリセット
	ev3_motor_reset_counts(left_motor);
	ev3_motor_reset_counts(right_motor);

	// ジャイロセンサーリセット
	ev3_gyro_sensor_reset(gyro_sensor);
	// 倒立振子API初期化
	balance_init();

	// スタート通知
	ev3_led_set_color(LED_GREEN);

	// グローバル変数初期化
	initialize_paramater();

	// ローカル変数初期化
	double angle = 0;		   // 今向いている角度(rad)
	int motor_before[2] = {0}; // モータの前回の値(角度演算用)
	float distance = 0;		   // 現在進んだ距離(cm)
	float limbo_distance1 = 0;
	float limbo_distance2 = 0;
	float limbo_distance3 = 0;
	float tmp_distance = 0;

	EV3RT_sensor_param sensor;	 // センサ値格納
	motor_before[RIGHT_MOTOR] = 0; // モータの初期値
	motor_before[LEFT_MOTOR] = 0;

	// 最初はPHASE2へ (角度補正多分要らない)
	TURN_STATE = PHASE2;

	tslp_tsk(500); //追加 500ms待つ
	/*---------------------------走行----------------------------*/

	while (1)
	{
		/* 追加 タッチセンサ押されたら強制終了 */
		if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
		{
			break;
		}

		if (bt_cmd == 0)
		{
			break;
		}

		/* 追加終わり */

		// 角度演算と距離計算はずっと行う
		// センサ値取得
		sensor = GetParam();

		// 距離演算
		/* 2で割らなくてよいのか？ちなみに、WHEEL_Rは半径ではなく、直径 */
		distance = 0.5 * (sensor.right + sensor.left) / 360 * PI * WHEEL_R;


		// モータの前回の値更新
		motor_before[RIGHT_MOTOR] = sensor.right;
		motor_before[LEFT_MOTOR] = sensor.left;

		switch (TURN_STATE)
		{
		// PHASE2 通常走行
		case PHASE2:
			// 速度設定
			//forward = 60;
			//forward = Speed_adjust(forward, 100);
			//尻尾を上にキープ
			tail_control(TAIL_ANGLE_DRIVE, P_GAIN_FORWARD);
			// PID制御
			turn = pid_reflection(sensor.color, calib_light);
			if(turn >= 100){
				turn = 100;
			}else if(turn <= -100){
				turn = -100;
			}
			forward = Speed_adjust(turn);
			//if(turn >= 10 || turn <= -10){
			//	forward = Speed_adjust(forward, 0);
			//}

			//倒立振子API
			EV3RT_Balancer(sensor, forward, turn, &pwm_L, &pwm_R);
			//走行
			EV3RT_Running(pwm_L, pwm_R);

			//データ送信
			// ログ出力
			for(int str_clear_cnt = 0; str_clear_cnt < 300; str_clear_cnt++){
				output_string[str_clear_cnt] = 0;
			}
			sprintf(output_string, "color:%d  forward:%d  turn:%d\n", sensor.color, forward, turn);
			fputs(output_string, bt);

			if (sonar_alert() == 1) /* 障害物検知 */

			{
				TURN_STATE = PHASE21; /* 障害物を検知したら停止 */
			}

			//灰色検知
			//if(gray_detection(sensor.color)){
			//TURN_STATE = PHASE20;
			//}

			// 距離が2400越え(リンボー用)
			if (distance > DISTANCE_GARAGE_LIMBO)
			{
				ev3_speaker_play_tone(NOTE_D6, 1000);
				//尻尾を上にキープ
				tail_control(TAIL_ANGLE_DRIVE_2, P_GAIN_FORWARD);
				//スピードダウン
				TURN_STATE = PHASE21;
			}

			//tslp_tsk(4);
			break;

		// 階段下りた後
		// case PHASE21:
		// 	// 尻尾走行に移行
		// 	forward = 0;
		// 	turn = pid_reflection(sensor.color, calib_light); //仮
		// 	change_tailRunning_Mode();
		// 	tmp_distance = distance;
		// 	ev3_motor_steer(left_motor, right_motor, forward, 0);
		// 	TURN_STATE = PHASE23;
		// 	break;

		//尻尾走行移行後一定距離走る
		case PHASE23:
			//リンボー後の距離測定開始

			limbo_distance1 = 0.5 * (sensor.right + sensor.left) / 360 * PI * WHEEL_R - tmp_distance;
			if (limbo_distance1 < 100)
			{
				//尻尾固定
				tail_control(TILT_MOTOR_PARAM, P_GAIN_FORWARD);
				//走行速度
				forward = 10;
				//forward = Speed_adjust(forward, 0);
				//走行
				ev3_motor_steer(left_motor, right_motor, forward, 0);
			}
			// 一定距離走ったら
			else
			{
				//バック走行へ
				tmp_distance = distance;
				TURN_STATE = PHASE25;
			}
			//tslp_tsk(4);		//4msごとに稼働
			break;
			//case PHASE80:
			//ev3_speaker_play_tone(NOTE_D6,1000);
			//tslp_tsk(3000);
			//TURN_STATE = PHASE25;
			//break;
		case PHASE24:
			// 尻尾走行に移行
			forward = 0;
			turn = pid_reflection(sensor.color, calib_light); //仮
			change_tailRunning_Mode();
			TURN_STATE = PHASE25;
			break;

		case PHASE25:
			//尻尾固定
			//リンボー後の距離測定開始

			limbo_distance2 = 0.5 * (sensor.right + sensor.left) / 360 * PI * WHEEL_R - tmp_distance;
			if (limbo_distance2 > -100)
			{
				//尻尾固定
				tail_control(TILT_MOTOR_PARAM, P_GAIN_FORWARD);
				//走行速度
				forward = -10;
				//forward = Speed_adjust(forward, 0);
				//走行
				ev3_motor_steer(left_motor, right_motor, forward, 0);
			}
			// 一定距離走ったら
			else
			{
				//尻尾走行へ
				tmp_distance = distance;
				TURN_STATE = PHASE26;
			}
			break;

		case PHASE26:
			//尻尾固定
			//リンボー後の距離測定開始

			limbo_distance3 = 0.5 * (sensor.right + sensor.left) / 360 * PI * WHEEL_R - tmp_distance;
			if (limbo_distance3 < 150)
			{
				//尻尾固定
				tail_control(TILT_MOTOR_PARAM, P_GAIN_FORWARD);
				//走行速度
				forward = 10;
				//forward = Speed_adjust(forward, 0);
				//走行
				ev3_motor_steer(left_motor, right_motor, forward, 0);
			}
			// 一定距離走ったら
			else
			{
				//バック走行へ
				tmp_distance = distance;
				TURN_STATE = PHASE99;
			}
			break;

		// 停止処理
		case PHASE99:
			//尻尾固定
			tail_control(TILT_MOTOR_PARAM, P_GAIN_FORWARD);
			// 速度を0へ
			//forward = forward = Speed_adjust(forward, 0);
			//走行
			ev3_motor_steer(left_motor, right_motor, forward, 0);
			//tslp_tsk(4);		//4msごとに稼働
			break;
		}
		tslp_tsk(4);
	}

	ev3_motor_stop(left_motor, false);
	ev3_motor_stop(right_motor, false);
	ev3_motor_stop(tail_motor, false);

	/*停止通知*/
	ev3_led_set_color(LED_RED);

	/* ラベルを付けているが、ここにジャンプするgoto文は見つからない */
exit_func: //非常停止
	ter_tsk(BT_TASK);
	/*bluetooth終了*/
	fclose(bt);
	ext_tsk();
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       b1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
	while (1)
	{
		uint8_t c = fgetc(bt); /* 受信 */
		switch (c)
		{
		case '0':
			ev3_speaker_play_tone(NOTE_CS6, 1000);
			bt_cmd = 0;
			break;
		case '1':
			ev3_speaker_play_tone(NOTE_CS6, 1000);
			bt_cmd = 1;
			break;
		default:
			break;
		}
		fputc(c, bt); /* エコーバック */
	}
}
