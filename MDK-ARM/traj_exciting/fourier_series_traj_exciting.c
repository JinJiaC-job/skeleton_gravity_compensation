#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "lkmoto.h"
#include "ids830can.h"
#include "fourier_series_traj_exciting.h"
#include "tim.h"

//电机控制时间点
uint8_t motor_control_k = 0;
//motor control interval time
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
float q_last[7] = {0};
// fourier series params
float traj_param[] = {0,
-0.0017057,
0.009662,
0.00055857,
0.0025355,
-0.026887,
0.058019,
0.0052519,
-0.0055934,
0.023903,
-0.033055,
0.07243,
-0.018499,
-0.079521,
-0.13061,
0.15341,
-0.091243,
0.1497,
0.30147,
-0.026215,
-0.061113,
-0.11431,
0.056254,
0.029636,
-0.02684,
0.051749,
-0.012772,
0.50174,
-0.12077,
-0.39032,
0.074961,
-0.19281,
0.022967,
-0.15964,
-0.06973,
0.22619,
0.59372,
0.003307,
0.1758,
0.12936,
-0.32981,
0.354,
-0.36998,
-0.40737,
-0.68594,
0.018378,
0.048977,
0.42348,
-0.47845,
0.24679,
-0.087001,
-0.6775,
0.32875,
-0.011139,
-0.02922,
1.1159,
0.17247,
-0.26386,
-0.29541,
0.49242,
-0.18677,
0.11182,
0.26263,
-0.14801,
0.047067,
-0.092879,
-0.11444};


void fourier_series_traj(uint8_t time)
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
//		printf("fourier_series_traj %d compute successfully\r\n", k);
	for(int i=1; i<=6; i++)
	{
		if(i == 1)
		{
			q[i] = q[i]*1000;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time)+1;//fabs:float类型的绝对值函数
			LinearActuator_startRun_maxspeed_position(i, motor_speed, q[i]);
			q_last[i] = q[i];
		}
		else
		{
			q[i] = q[i]/pi*180;
			motor_speed = fabs((q[i]-q_last[i])/control_interval_time)+1;//fabs:float类型的绝对值函数
			angle_close_loop_with_speed(i, q[i], motor_speed);
			q_last[i] = q[i];
		}
	}
	motor_control_k++;
}




