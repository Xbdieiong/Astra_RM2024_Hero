/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"




//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//��곤���ж�
#define PRESS_LONG_TIME             1//400
//�������ο����й�
#define SINGLE_SHOOT_MAX_TIME       400

//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              400
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rpm �仯�� ��ת�ٶȵı���  //�Ѿ���������1:220���ٱȵ���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f				//0.00047454545454545454545454545454545f   
#define MOTOR_ECD_TO_ANGLE          0.0000035008911105796594881674897663f	//0.000021305288720633905968306772076277f
#define FULL_COUNT                  110     //18
//�����ٶ�
#define TRIGGER_SPEED               9.0f   //10.0f   //19
#define CONTINUE_TRIGGER_SPEED      12.0f  //1.5f 
#define READY_TRIGGER_SPEED         5.0f
#define SINGLE_TRIGGER_SPEED        18.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f
//�Ƕ�����
#define FULL_ROUND									0.3292125f						

//�����ֵ���ٶ�PID
#define TRIGGER_SPEED_PID_KP        50.f
#define TRIGGER_SPEED_PID_KI				0.f
#define TRIGGER_SPEED_PID_KD				0.f
#define TRIGGER_SPEED_PID_MAX_OUT   16000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT  7000.0f
//�����ֵ���Ƕ�PID
#define TRIGGER_ANGLE_PID_KP        2000.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        0.0f
#define TRIGGER_ANGLE_PID_MAX_OUT   16000.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT  7000.0f

#define TRIGGER_BULLET_PID_MAX_OUT  16000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 7000.0f

#define TRIGGER_READY_PID_MAX_OUT   16000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

//Ħ���ֵ��PID
#define FRIC_PID_KP         15000.0f
#define FRIC_PID_KI         10.0f
#define FRIC_PID_KD         0.0f

#define FRIC_PID_MAX_OUT    17000.0f
#define FRIC_PID_MAX_IOUT   2000.0f

#define SHOOT_HEAT_REMAIN_VALUE       50

#define SHOOT_HEAT_REMAIN_VALUE_1V1   30

//�����ı䵯��������
#define KEY_TO_FRIC_SPEED_SEN    0.002f


typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
	  shoot_mode_e last_shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
	  const motor_measure_t *fric1_motor_measure;
	  const motor_measure_t *fric2_motor_measure;
    pid_type_def trigger_motor_speed_pid;
		pid_type_def trigger_motor_angle_pid;
	  pid_type_def fric1_motor_pid;
	  pid_type_def fric2_motor_pid;
    fp32 trigger_speed_set;
		//������
		uint16_t shoot_time_count;//������ӳټ�����
		fp32 shoot_bullet_count;//����״̬��־λ
    fp32 speed;
		fp32 speed_set;
		fp32 angle;
    fp32 set_angle;
		fp32 angle_add;
		int16_t current_set;
		int16_t given_current;
	  int8_t ecd_count;
	//Ħ����
	  fp32 fric1_speed;
	  fp32 fric2_speed; 
		fp32 fric1_speed_set;
	  fp32 fric2_speed_set;
		int16_t fric1_given_current;
		int16_t fric2_given_current;


    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
extern int shoot_init_flag;
extern void judge_shoot_speed_add_little(void);
extern shoot_control_t shoot_control;

#endif