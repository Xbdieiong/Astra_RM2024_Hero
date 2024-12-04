/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"


#define POWER_LIMIT_level_0         40.0f

#define POWER_LIMIT_level_1_power_priority    60.f   //功率优先
#define POWER_LIMIT_level_2_power_priority    80.f
#define POWER_LIMIT_level_3_power_priority    100.f

#define POWER_LIMIT_level_1_blood_priority    45.0f
#define POWER_LIMIT_level_2_blood_priority    50.0f
#define POWER_LIMIT_level_3_blood_priority    55.0f

#define POWER_LIMIT_level_1_balance_chassis   60.0f
#define POWER_LIMIT_level_2_balance_chassis   80.0f
#define POWER_LIMIT_level_3_balance_chassis   100.0f

#define POWER_LIMIT_1V1_game  120.0f  //120.0f     60.0f

#define WARNING_POWER_0       35.0f   
#define WARNING_POWER_1       55.0f
#define WARNING_POWER_2       75.0f
#define WARNING_POWER_3       90.0f

#define WARNING_POWER_1V1_game   100.0f//50.0f

#define WARNING_POWER_BUFF  45.0f 
#define WARNING_SUPERCAP_VOTALGE 18.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    	 32000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      	 16000.0f
#define POWER_TOTAL_CURRENT_LIMIT          20000.0f
#define FLYING_BUFFER_TOTAL_CURRENT_LIMIT  35000.0f
#define SUPERCAP_TOTAL_CURRENT_LIMIT       20000.0f
#define FLYING_SUPERCAP_TOTAL_CURRENT_LIMIT     35000.0f

extern uint8_t get_robot_level(void);
extern uint8_t get_game_type(void);
fp32 capacitor_power = 0;
fp32 flying_buff_flag = 0;

/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
	  fp32 add_power_limit = 0.0f;
	
    uint8_t robot_id = get_robot_id();
		
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(Capacitor_Voltage)
    {
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
        //功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
				//chassis_power_buffer
			  if(chassis_power_buffer == 250)
				{
					 flying_buff_flag = 1;
				}

				//输入控制
//				if(flying_buff_flag)
//				{
//						if(chassis_power_buffer < 250*3/4)
//						{
//								fp32 power_scale;
//								if(chassis_power_buffer > 75.0f)
//								{
//										//scale down WARNING_POWER_BUFF
//										//缩小WARNING_POWER_BUFF
//										power_scale = chassis_power_buffer / 250;
//								}
//								else
//								{
//										//only left 10% of WARNING_POWER_BUFF
//										power_scale = 0;
//								}
//								//scale down
//								//缩小
//								add_power_limit = 10 * power_scale;					
//						}
//						else add_power_limit = 10;
//						
//						if(chassis_power_buffer<60)
//						{
//							flying_buff_flag = 0;
//						}
//					
//				}
//				
//				else
				{
					 if(chassis_power_buffer < 60)
						{
								fp32 power_scale;
								if(chassis_power_buffer > 40.0f)
								{
										//scale down WARNING_POWER_BUFF
										//缩小WARNING_POWER_BUFF
										power_scale = chassis_power_buffer /60;
								}
								else
								{
										//only left 10% of WARNING_POWER_BUFF
										power_scale = 0;
								}
								//scale down
								//缩小
								add_power_limit = 5 * power_scale;					
						}
						else add_power_limit = 5;
				}
				capacitor_power = get_chassis_power_limit() + add_power_limit;
				
				//WARNING_SUPERCAP_VOTALGE
				//输出控制			
				if(Capacitor_Voltage < 20)
        {
            fp32 power_scale;
					
						if(Capacitor_Voltage > 14)
						{
								//scale down WARNING_POWER_BUFF
								//缩小WARNING_POWER_BUFF
								power_scale = Capacitor_Voltage/ 7 / WARNING_POWER_BUFF;
						}
						else
						{
								//only left 10% of WARNING_POWER_BUFF
								power_scale = 0.4f / WARNING_POWER_BUFF;
						}
            //scale down
            //缩小
						if(flying_flag)
						{
							total_current_limit = FLYING_BUFFER_TOTAL_CURRENT_LIMIT * power_scale + POWER_TOTAL_CURRENT_LIMIT;
						}
            else total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale + POWER_TOTAL_CURRENT_LIMIT ;        
			  }
					
				else
				{
						if(flying_flag)
						{
							total_current_limit = FLYING_BUFFER_TOTAL_CURRENT_LIMIT+ POWER_TOTAL_CURRENT_LIMIT;
						}					
						else total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT ; 
				}
		}
		
		//无超级电容
		else
		{
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);//裁判系统获取功率及缓存能量
  			if(chassis_power_buffer < WARNING_POWER_BUFF)//当缓冲能量以及用掉一部分，此时一定超功率
				{
						fp32 power_scale;
						if(chassis_power_buffer > 30.0f)
						{
								//scale down WARNING_POWER_BUFF
								//缩小WARNING_POWER_BUFF
								power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
						}
						else
						{
								//only left 10% of WARNING_POWER_BUFF
								power_scale = 4.0f / WARNING_POWER_BUFF;
						}
						//scale down
						//缩小
						total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale*0.7;		//电流值16000是3508最大信号			
				}
			 else//没超功率，即将超也开始限速
			 {
				 if(chassis_power > get_chassis_power_limit()*9/10)
				 {
							fp32 power_scale;
					
							if(chassis_power < get_chassis_power_limit())
							{                    
									//缩小
									power_scale = (( get_chassis_power_limit() - chassis_power)/(get_chassis_power_limit() *1/10));                    
							}
							else
							{
								 power_scale = 0.0f;
							}
							total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT*0.7 + POWER_TOTAL_CURRENT_LIMIT * power_scale;									
				 }
				 else
				 {
							total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT*0.7 + POWER_TOTAL_CURRENT_LIMIT ;							 
				 }				
			}		
		}
  
    total_current = 0.0f;
    //calculate the original motor current set
    //计算原本电机电流设定
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);//这是四个轮的总电流，即底盘电流
    }
    
    if(total_current > total_current_limit)//通过按超功率或缓冲能量的比例限制电流，达到限速的目的
    {
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
}
