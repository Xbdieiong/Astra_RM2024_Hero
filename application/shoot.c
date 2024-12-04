/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

//#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
//#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
//#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
  * @brief          拨弹轮单发控制
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_control(shoot_control_t *shoot_control);
/**
  * @brief          拨弹轮PID
  * @param[in]      void
  * @retval         void
  */

static fp32 trigger_PID_calc(pid_type_def *pid, fp32 get, fp32 set, fp32 error_delta);

/**
  * @brief          射击状态机设置
  * @param[in]      void
  * @retval         void
  */

static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          拨盘校准（开发中）
  * @param[in]      void
  * @retval         void
  */
static void trigger_reload(void);

/**
  * @brief          射击校准控制，控制拨弹电机速度，校准电机到初始位置
  * @param[in]      void
  * @retval         void
  */
static void shoot_calibrate_control(void);



shoot_control_t shoot_control;          //射击数据
//重载模式参数
int reload_speed;
int reload_limit;
int reload_flag = 0;
int reload_count =0;
int shoot_speed_add_little = 0;
int last_shoot_speed_add_little = 0;
//狙击模式标志位
extern int sniper_flag;
//键鼠射击状态机
int fire_flag = 0;
int last_fire_flag = 0;
//弹仓重载标志位

extern int id_of_controller;
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
	  //波轮PID参数
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		static const fp32 Trigger_angle_pid[3]={TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
		//摩擦轮PID参数
		static const fp32 Fric1_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};  
		static const fp32 Fric2_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
		shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
    //初始化PID
    PID_init(&shoot_control.trigger_motor_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_init(&shoot_control.trigger_motor_angle_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric1_motor_pid, PID_POSITION, Fric1_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		PID_init(&shoot_control.fric2_motor_pid, PID_POSITION, Fric2_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		//初始化数据
    shoot_feedback_update();
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
		shoot_control.current_set =0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
		shoot_control.angle_add=0.635*2;//每次拨弹的位置环增量（暂时翻倍）//0.27434*2
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		shoot_control.shoot_time_count = 0; //记录拨弹持续时间
		shoot_control.shoot_bullet_count=0;//记录发射状态计数
		autoaim_mode_flag = 0;//自瞄模式初始化
		reload_speed =1000;
		sniper_flag=0;
		reload_limit=500;
}




/**9
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{
    judge_shoot_speed_add_little();
    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
		shoot_control.last_shoot_mode = shoot_control.shoot_mode;
		trigger_motor_control(&shoot_control);//拨轮单发控制
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
				shoot_control.set_angle=shoot_control.set_angle;//拨盘设置在原位
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
				//sniper_flag=0;//狙击模式遥控器测试
				autoaim_mode_flag = 0;//自瞄模式遥控器测试
				 //射击状态机设置
				if(shoot_control.press_l)//检测到射击键是否按下   
				{
					fire_flag = 1;//射击键按下为1 弹起为0
				}
				else fire_flag = 0;
				
				if(fire_flag && (!last_fire_flag))//长按判断 以前是否长按 初始化赋值为0
				{
					shoot_control.set_angle=rad_format(shoot_control.angle+shoot_control.angle_add);  //拨弹位置环增量  初始化赋值为0
					reload_flag = 1;//拨盘校准标志位
					//shoot_control.shoot_mode == SHOOT_DONE //发射之后进入校准模式
				}
				last_fire_flag = fire_flag;// 历史按键记录 防止长按不松手导致多次计数 按键弹起后归零（状态机复位）
				
				if(get_remain_HP() == 0)//死亡时重新初始化
				{					
					shoot_control.set_angle=shoot_control.set_angle;//拨盘设置在原位					
				}
		}
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {		
			//sniper_flag=1;//狙击模式遥控器测试
			//autoaim_mode_flag = 1;//自瞄模式遥控器测试
			
	
			shoot_control.shoot_time_count++;
			if(shoot_control.shoot_time_count==2000)
				 {//计数器每次计数计数单元归零，且设置目标角度增量
					shoot_control.shoot_time_count=0;
					//发射前设置目标角度为当前位置（射击前先对当前角度进行初始化）
					reload_flag = 0;//发射前reload状态必需保证归零
					shoot_control.set_angle=shoot_control.angle;
					shoot_control.set_angle=rad_format(shoot_control.angle+shoot_control.angle_add);	
					reload_flag = 1;//拨盘校准标志位
				 }
			
				 
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        //校准模式 
				//reload_flag == 0  //校准结束时回复原有模式
				//shoot_control.shoot_mode == SHOOT_READY
    }

    if(shoot_control.shoot_mode == SHOOT_STOP) //停止模式下关闭摩擦轮 激光 拨盘停止输出
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
			  shoot_speed_add_little = 0;
			  last_shoot_speed_add_little = 0;
			  //摩擦轮停止
        shoot_control.fric1_speed_set = 0;
			  shoot_control.fric2_speed_set = 0;
				shoot_control.set_angle=shoot_control.set_angle;//拨盘设置在原位
			  PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_speed_set);
			  PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_speed_set);
				shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
			  shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
			  CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current, 0, 0);   //摩擦轮电机赋值
    }
    else  //非射击停止模式 根据裁判读数设置摩擦轮速度 开启激光
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
			
			else//弹速测试结果：16m/s （参数：2.5 首发：15.5 后续范围：15.7-15.8 ） 10m/s（参数： 首发： 后续范围：）
			{
//					shoot_control.fric1_speed_set = -(2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);//1.754
//					shoot_control.fric2_speed_set = (2.0 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);//1.754
				shoot_control.fric1_speed_set =-2.5;
					shoot_control.fric2_speed_set = 2.5;
				
			}
      shoot_laser_on(); //激光开启
      //计算拨弹轮电机PID
      //PID_calc(&shoot_control.trigger_motor_speed_pid, shoot_control.speed, shoot_control.speed_set);
			//计算摩擦轮电机PID
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
		
		CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current, 0, 0);   //摩擦轮电机赋值
    return shoot_control.given_current;
}


extern uint8_t get_game_type(void);

/**
  * @brief          射击状态机设置
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))   //&& !(shoot_control.press_l)
    {
        shoot_control.shoot_mode = SHOOT_READY;//拨杆在中 开摩擦轮，激光 鼠标点击可以发射
    }
    else if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;//拨杆在下 急停
    }
		
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //遥控器左拨杆到上(检录清弹用)或鼠标按下一次，进入射击状态
        if (switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])) //拨杆在上
        {
						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;//无视热量进入持续发射状态 一般做调试用
        }
    }   
    
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		
}

extern uint8_t get_game_type(void);


/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

		//摩擦轮电机数据更新
		shoot_control.fric1_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric1_motor_measure->speed_rpm; 
		shoot_control.fric2_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric2_motor_measure->speed_rpm; 
    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
		//对于英雄拨盘，拨盘转一圈 电机轴转219.9645454545圈，约等于220圈 
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE) //正转电机圈数累加
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)//反转电机圈数累加
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)//正传转数等于计数上限（110）
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);//取负号的目的是用int8（-128~127）节约内从空间
    }
		//正传和反转的圈数记录不能冲突
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度 :总码盘值域：-884628~909210  归化角度区间范围为-0.31~0.318 抽样计数，角度增量0.3292125等于一整圈 半个弹丸行程：0.027434
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //微动开关
    shoot_control.key = BUTTEN_TRIG_PIN;
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
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

    //射击开关下档时间计时
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



//堵转反转
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


//弹速微调
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


//单发拨弹控制函数
static void trigger_motor_control(shoot_control_t *shoot_control)
{						//PID赋值：
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
	//串级PID控制：
	shoot_control->speed_set = trigger_PID_calc(&shoot_control->trigger_motor_angle_pid, shoot_control->angle, shoot_control->set_angle, shoot_control->speed);
	shoot_control->current_set = PID_calc(&shoot_control->trigger_motor_speed_pid, shoot_control->speed, shoot_control->speed_set);
	//控制器输出：
	//重载模式下输出额外电流（测试）
	if(reload_flag==1)
	{
	shoot_control->given_current = (int16_t)(shoot_control->current_set)+(int16_t)reload_speed;//取拨盘输出取最大值 防止超过当前位置的负值闭环输出与开环控制量相矛盾
	}
	else
	{
		shoot_control->given_current = (int16_t)(shoot_control->current_set);
	}
}
//拨弹PID计算
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

