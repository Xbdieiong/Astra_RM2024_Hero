/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
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

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "autoaim.h"

#include "chassis_task.h"

//#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
//#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
//#define shoot_fric_off()    fric_off()      //�ر�����Ħ����

#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
  * @brief          �����ֵ�������
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_control(shoot_control_t *shoot_control);
/**
  * @brief          ������PID
  * @param[in]      void
  * @retval         void
  */

static fp32 trigger_PID_calc(pid_type_def *pid, fp32 get, fp32 set, fp32 error_delta);

/**
  * @brief          ���״̬������
  * @param[in]      void
  * @retval         void
  */

static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ����У׼�������У�
  * @param[in]      void
  * @retval         void
  */
static void trigger_reload(void);

/**
  * @brief          ���У׼���ƣ����Ʋ�������ٶȣ�У׼�������ʼλ��
  * @param[in]      void
  * @retval         void
  */
static void shoot_calibrate_control(void);



shoot_control_t shoot_control;          //�������
//����ģʽ����
int reload_speed;
int reload_limit;
int reload_flag = 0;
int reload_count =0;
int shoot_speed_add_little = 0;
int last_shoot_speed_add_little = 0;
//�ѻ�ģʽ��־λ
extern int sniper_flag;
//�������״̬��
int fire_flag = 0;
int last_fire_flag = 0;
//�������ر�־λ

extern int id_of_controller;
/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
	  //����PID����
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		static const fp32 Trigger_angle_pid[3]={TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
		//Ħ����PID����
		static const fp32 Fric1_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};  
		static const fp32 Fric2_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
		shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
    //��ʼ��PID
    PID_init(&shoot_control.trigger_motor_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_init(&shoot_control.trigger_motor_angle_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric1_motor_pid, PID_POSITION, Fric1_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		PID_init(&shoot_control.fric2_motor_pid, PID_POSITION, Fric2_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		//��ʼ������
    shoot_feedback_update();
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
		shoot_control.current_set =0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
		shoot_control.angle_add=0.635*2;//ÿ�β�����λ�û���������ʱ������//0.27434*2
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		shoot_control.shoot_time_count = 0; //��¼��������ʱ��
		shoot_control.shoot_bullet_count=0;//��¼����״̬����
		autoaim_mode_flag = 0;//����ģʽ��ʼ��
		reload_speed =1000;
		sniper_flag=0;
		reload_limit=500;
}




/**9
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{
    judge_shoot_speed_add_little();
    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������
		shoot_control.last_shoot_mode = shoot_control.shoot_mode;
		trigger_motor_control(&shoot_control);//���ֵ�������
		if (reload_flag==1)
		{
			reload_count++;
			if(reload_count==reload_limit)
			{
				reload_flag=0;
			}
		}
		
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
				shoot_control.set_angle=shoot_control.set_angle;//����������ԭλ
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
				//sniper_flag=0;//�ѻ�ģʽң��������
				autoaim_mode_flag = 0;//����ģʽң��������
				 //���״̬������
				if(shoot_control.press_l)//��⵽������Ƿ���   
				{
					fire_flag = 1;//���������Ϊ1 ����Ϊ0
				}
				else fire_flag = 0;
				
				if(fire_flag && (!last_fire_flag))//�����ж� ��ǰ�Ƿ񳤰� ��ʼ����ֵΪ0
				{
					shoot_control.set_angle=rad_format(shoot_control.angle+shoot_control.angle_add);  //����λ�û�����  ��ʼ����ֵΪ0
					reload_flag = 1;//����У׼��־λ
					//shoot_control.shoot_mode == SHOOT_DONE //����֮�����У׼ģʽ
				}
				last_fire_flag = fire_flag;// ��ʷ������¼ ��ֹ���������ֵ��¶�μ��� �����������㣨״̬����λ��
				
				if(get_remain_HP() == 0)//����ʱ���³�ʼ��
				{					
					shoot_control.set_angle=shoot_control.set_angle;//����������ԭλ					
				}
		}
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {		
			//sniper_flag=1;//�ѻ�ģʽң��������
			//autoaim_mode_flag = 1;//����ģʽң��������
			
	
			shoot_control.shoot_time_count++;
			if(shoot_control.shoot_time_count==2000)
				 {//������ÿ�μ���������Ԫ���㣬������Ŀ��Ƕ�����
					shoot_control.shoot_time_count=0;
					//����ǰ����Ŀ��Ƕ�Ϊ��ǰλ�ã����ǰ�ȶԵ�ǰ�ǶȽ��г�ʼ����
					reload_flag = 0;//����ǰreload״̬���豣֤����
					shoot_control.set_angle=shoot_control.angle;
					shoot_control.set_angle=rad_format(shoot_control.angle+shoot_control.angle_add);	
					reload_flag = 1;//����У׼��־λ
				 }
			
				 
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        //У׼ģʽ 
				//reload_flag == 0  //У׼����ʱ�ظ�ԭ��ģʽ
				//shoot_control.shoot_mode == SHOOT_READY
    }

    if(shoot_control.shoot_mode == SHOOT_STOP) //ֹͣģʽ�¹ر�Ħ���� ���� ����ֹͣ���
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
			  shoot_speed_add_little = 0;
			  last_shoot_speed_add_little = 0;
			  //Ħ����ֹͣ
        shoot_control.fric1_speed_set = 0;
			  shoot_control.fric2_speed_set = 0;
				shoot_control.set_angle=shoot_control.set_angle;//����������ԭλ
			  PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_speed_set);
			  PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_speed_set);
				shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
			  shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
			  CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current, 0, 0);   //Ħ���ֵ����ֵ
    }
    else  //�����ֹͣģʽ ���ݲ��ж�������Ħ�����ٶ� ��������
    {
			if(get_robot_shoot_speed_limit() == 10)
			{
				
//					shoot_control.fric1_speed_set = -(2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);
//					shoot_control.fric2_speed_set = (2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);
				shoot_control.fric1_speed_set =-2.5;
					shoot_control.fric2_speed_set = 2.5;
				
			}
			else if(get_robot_shoot_speed_limit() == 16)
			{
				
				
//					shoot_control.fric1_speed_set = -(2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);
//					shoot_control.fric2_speed_set = (2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);
				shoot_control.fric1_speed_set =-2.5;
					shoot_control.fric2_speed_set = 2.5;
							
			}
			
			else//���ٲ��Խ����16m/s ��������2.5 �׷���15.5 ������Χ��15.7-15.8 �� 10m/s�������� �׷��� ������Χ����
			{
//					shoot_control.fric1_speed_set = -(2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);//1.754
//					shoot_control.fric2_speed_set = (2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);//1.754
				shoot_control.fric1_speed_set =-2.5;
					shoot_control.fric2_speed_set = 2.5;
				
			}
      shoot_laser_on(); //���⿪��
      //���㲦���ֵ��PID
      //PID_calc(&shoot_control.trigger_motor_speed_pid, shoot_control.speed, shoot_control.speed_set);
			//����Ħ���ֵ��PID
			PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_speed_set);
			PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_speed_set);
			
      //shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_speed_pid.out);
			shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
			shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
			
      if(shoot_control.shoot_mode < SHOOT_READY)
      {
            shoot_control.given_current = 0;
						shoot_control.fric1_given_current = 0; 
					  shoot_control.fric2_given_current = 0;
      }

   
    }
		
		CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current, 0, 0);   //Ħ���ֵ����ֵ
    return shoot_control.given_current;
}


extern uint8_t get_game_type(void);

/**
  * @brief          ���״̬������
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))   //&& !(shoot_control.press_l)
    {
        shoot_control.shoot_mode = SHOOT_READY;//�������� ��Ħ���֣����� ��������Է���
    }
    else if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;//�������� ��ͣ
    }
		
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //ң�����󲦸˵���(��¼�嵯��)����갴��һ�Σ��������״̬
        if (switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])) //��������
        {
						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//�������������������״̬ һ����������
        }
    }   
    
    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		
}

extern uint8_t get_game_type(void);


/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

		//Ħ���ֵ�����ݸ���
		shoot_control.fric1_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric1_motor_measure->speed_rpm; 
		shoot_control.fric2_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric2_motor_measure->speed_rpm; 
    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
		//����Ӣ�۲��̣�����תһȦ �����ת219.9645454545Ȧ��Լ����220Ȧ 
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE) //��ת���Ȧ���ۼ�
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)//��ת���Ȧ���ۼ�
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)//����ת�����ڼ������ޣ�110��
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);//ȡ���ŵ�Ŀ������int8��-128~127����Լ�ڴӿռ�
    }
		//�����ͷ�ת��Ȧ����¼���ܳ�ͻ
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ� :������ֵ��-884628~909210  �黯�Ƕ����䷶ΧΪ-0.31~0.318 �����������Ƕ�����0.3292125����һ��Ȧ ��������г̣�0.027434
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //΢������
    shoot_control.key = BUTTEN_TRIG_PIN;
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

}



//��ת��ת
static void trigger_reload(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}
int G_pressed = 0;
int B_pressed = 0;

int last_G_pressed = 0;
int last_B_pressed = 0;

int G_pressed_count = 0;
int B_pressed_count = 0;


//����΢��
void judge_shoot_speed_add_little(void)
{
	if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_G)
	{
		G_pressed = 1;
	}
	else
	{
		G_pressed = 0;
	}
	
	if(G_pressed == 1 && last_G_pressed == 0)
	{
		G_pressed_count++;
		shoot_speed_add_little += 5;
		last_shoot_speed_add_little = 1;
	}
	
	if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_B)
	{
		B_pressed = 1;
	}
	else 
	{
		B_pressed = 0;
	}
	
	if(B_pressed == 1 && last_B_pressed == 0)
	{
		B_pressed_count++;
		shoot_speed_add_little -= 5;
		last_shoot_speed_add_little = 1;
	}
	
	last_G_pressed = G_pressed;
	last_B_pressed = B_pressed;
}


//�����������ƺ���
static void trigger_motor_control(shoot_control_t *shoot_control)
{						//PID��ֵ��
						shoot_control->trigger_motor_angle_pid.Kp = 100;   //15 38
						shoot_control->trigger_motor_angle_pid.Ki = 0;    //0
						shoot_control->trigger_motor_angle_pid.Kd = 0;    //0
						//		ShootMotorPositionPID.kp =50;//100;
						//		ShootMotorPositionPID.ki=0.001;//0.005
						//		ShootMotorPositionPID.kd=3;//18
						shoot_control->trigger_motor_speed_pid.Kp = 1000; //2900  4500  //3200
						shoot_control->trigger_motor_speed_pid.Ki = 0;      //60       //5
						shoot_control->trigger_motor_speed_pid.Kd = 0;       //0     //100
						//	ShootMotorSpeedPID.kp = 70;//100.0;
						//	ShootMotorSpeedPID.ki = 0;
						//	ShootMotorSpeedPID.kd = 2;//5;//18
	//����PID���ƣ�
	shoot_control->speed_set = trigger_PID_calc(&shoot_control->trigger_motor_angle_pid, shoot_control->angle, shoot_control->set_angle, shoot_control->speed);
	shoot_control->current_set = PID_calc(&shoot_control->trigger_motor_speed_pid, shoot_control->speed, shoot_control->speed_set);
	//�����������
	//����ģʽ�����������������ԣ�
	if(reload_flag==1)
	{
	shoot_control->given_current = (int16_t)(shoot_control->current_set)+(int16_t)reload_speed;//ȡ�������ȡ���ֵ ��ֹ������ǰλ�õĸ�ֵ�ջ�����뿪����������ì��
	}
	else
	{
		shoot_control->given_current = (int16_t)(shoot_control->current_set);
	}
}
//����PID����
static fp32 trigger_PID_calc(pid_type_def *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->fdb = get;
    pid->set = set;

    err = set - get;
    pid->error[0] = rad_format(err);
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dout = pid->Kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

