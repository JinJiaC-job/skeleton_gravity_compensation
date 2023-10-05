#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "lkmoto.h"
#include "ids830can.h"
#include "fourier_series_traj_exciting.h"
#include "tim.h"

//电机控制时间节点：
unsigned int motor_control_k = 0;
//motor control interval time：每过motor control interval time秒输出一次控制命令
float control_interval_time = 0.05;
// sampling period
float traj_Ts = 0.1;
// trajectory fundamental frequency = 1/T; T = run time of skeleton = 20s.
float traj_f = 0.05;
// trajectory fundamental frequency in radian
float traj_wf;
// number of sampling points
uint8_t traj_n;
// order of trajectory generation 
uint8_t traj_order = 5;
// number of revolute joints
uint8_t dof = 6;
// 6个关节角度信息
float q[7] = {0};
float q_last[7] = {0, 0, 0, 0, 0, 0, 0};
float q_next[7] = {0};
// fourier series params
float traj_param[] = {0,
0.13249,
0.84265,
-0.61763,
-0.62935,
1.1823,
0.51965,
-1.0063,
-0.18653,
0.3091,
-0.079349,
97.383,
-0.16202,
0.037126,
0.14317,
-0.0095187,
-0.13003,
-0.12242,
0.10733,
0.0451,
0.041543,
0.033753,
0.030514,
0.062815,
-0.13427,
0.048746,
0.018498,
-0.025162,
0.093033,
0.33808,
-0.088307,
-0.42448,
0.034281,
-0.34769,
0.04826,
-0.006213,
0.054578,
0.26625,
0.16519,
-0.11289,
-0.12732,
0.040054,
-0.14071,
-0.069567,
-1.299,
0.17329,
0.045145,
-0.14034,
-0.011873,
0.36856,
-0.12058,
0.016315,
0.32496,
-0.41783,
-0.1919,
1.7041,
-0.18458,
0.093306,
-0.21073,
-0.039703,
-0.22417,
-0.07346,
0.72046,
0.13674,
-0.10098,
-0.068099,
0.22133};


void fourier_series_traj(float time)
{
	uint8_t order_prod_2, m;
  order_prod_2 = traj_order * 2;

	traj_wf = traj_f * 2.0 * pi;
	
	for(int i=1; i<=dof; i++)
	{
		m = (order_prod_2 + 1) * (i - 1); 
		q[i] = traj_param[m + order_prod_2 + 1];//q0
		for(int j=1; j<=traj_order; j++)
		{
			// alpha(a)=traj_param(m+2*(j-1)+1), beta(b)=traj_param(m+2*(j-1)+2)
			q[i] = q[i] + ((traj_param[m + 2*(j-1) + 1] / (traj_wf * j)) * sin(traj_wf * j * time) - (traj_param[m + 2*(j-1) + 2] / (traj_wf * j)) * cos(traj_wf * j * time));
//			if(motor_control_k<200)
//			{
//				q_next[i] = q[i] + ((traj_param[m + 2*(j-1) + 1] / (traj_wf * j)) * sin(traj_wf * j * (time+control_interval_time)) - (traj_param[m + 2*(j-1) + 2] / (traj_wf * j)) * cos(traj_wf * j * (time+control_interval_time)));
//			}
//			else
//				q_next[i] = q[i];
		}
	}
}

void traj_exciting_init(void)
{
	traj_n = 1.0 / control_interval_time / traj_f;
}

void run_fourier_series_traj(void)
{
	float motor_speed = 0;

	fourier_series_traj(motor_control_k*control_interval_time);//电机控制信号点
	for(int i=1; i<=6; i++)
	{
		if(i == 1)
		{
			q[i] = q[i]-95.35;//直线电缸单位mm
//			q_next[i] = q_next[i]*1000;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			LinearActuator_startRun_maxspeed_position(i, q[i], motor_speed);
			q_last[i] = q[i];
//			printf("linear=%.3f, linearSpeed=%.3f\r\n",q[1], motor_speed);
		}
		else if(i == 4)
		{
			q[i] = q[i]/pi*180 + 90;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed<1)//速度不能太小
				motor_speed=1;
			angle_close_loop_with_speed(i, -q[i], motor_speed);
			q_last[i] = q[i];
//			printf("MOTORSPEED4=%.3f, MOTORpos4=%.3f\r\n", motor_speed, q[4]);
		}
		else if(i == 5)
		{
			q[i] = q[i]/pi*180 - 90;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed<1)
				motor_speed=1;
			angle_close_loop_with_speed(i, q[i], motor_speed);
			q_last[i] = q[i];
		}
		else if(i == 6)
		{
			if((fabs(q[i]-q_last[i]))>=0.05)
			{
				q[i] = q[i]/pi*180;
//				q_next[i] = q_next[i]/pi*180;
				motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
				if(motor_speed<1)
				  motor_speed=1;
				angle_close_loop_with_speed(i, q[i], motor_speed);
				q_last[i] = q[i];
			}
		}
		else
		{
			q[i] = q[i]/pi*180;
//			q_next[i] = q_next[i]/pi*180;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time);//fabs:float类型的绝对值函数
			if(motor_speed<1)
				motor_speed=1;
			angle_close_loop_with_speed(i, q[i], motor_speed);
			q_last[i] = q[i];
//			if(i==6)
//				printf("MOTORSPEED6=%.3f, MOTORpos=%.3f\r\n", motor_speed, q[6]);
		}
	}
//	printf("k=%d", motor_control_k);
	if(++motor_control_k > 400)
		HAL_TIM_Base_Stop_IT(&htim2);
}




